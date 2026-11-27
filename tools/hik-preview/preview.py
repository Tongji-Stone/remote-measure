import os
import argparse
import sys
import time
from ctypes import byref, cast, POINTER
from pathlib import Path

import cv2
import numpy as np
from camera_metrics import exposure_score, roi_brightness

sdk = Path(os.environ.get('MVCAM_COMMON_RUNENV', r'C:\Program Files (x86)\MVS\Development'))
sys.path.insert(0, str(sdk / 'Samples' / 'Python' / 'MvImport'))
from MvCameraControl_class import *


def check(code, operation):
    if code != 0:
        raise RuntimeError(f'{operation}: 0x{code & 0xffffffff:08x}')


def set_float_parameter(camera, name, value):
    current = MVCC_FLOATVALUE()
    check(camera.MV_CC_GetFloatValue(name, current), f'Read {name}')
    if not current.fMin <= value <= current.fMax:
        raise ValueError(f'{name} must be in {current.fMin:g}..{current.fMax:g}; got {value:g}')
    check(camera.MV_CC_SetFloatValue(name, float(value)), f'Set {name}')
    print(f'{name}: {value:g} (range {current.fMin:g}..{current.fMax:g})', flush=True)


def run_auto_exposure_once(camera, timeout_seconds=12, upper_limit_us=200000):
    """Temporarily raise the camera's auto exposure ceiling, then lock exposure."""
    old_upper = None
    node = MVCC_INTVALUE_EX()
    rc = camera.MV_CC_GetIntValueEx('AutoExposureTimeUpperLimit', node)
    if rc == 0:
        old_upper = int(node.nCurValue)
        ceiling = max(int(node.nMin), min(int(node.nMax), int(upper_limit_us)))
        check(camera.MV_CC_SetIntValueEx('AutoExposureTimeUpperLimit', ceiling), 'Set auto exposure upper limit')
        print(f'Auto exposure upper limit: {ceiling} us (camera prior value {old_upper} us)', flush=True)
    else:
        print('Warning: camera auto exposure upper limit unavailable; using camera setting', flush=True)
    try:
        check(camera.MV_CC_SetEnumValue('ExposureAuto', 1), 'Start automatic exposure once')
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            frame = MV_FRAME_OUT()
            check(camera.MV_CC_GetImageBuffer(frame, 3000), 'Automatic exposure frame')
            camera.MV_CC_FreeImageBuffer(frame)
            state = MVCC_ENUMVALUE()
            check(camera.MV_CC_GetEnumValue('ExposureAuto', state), 'Read automatic exposure')
            if state.nCurValue == 0:
                value = MVCC_FLOATVALUE()
                check(camera.MV_CC_GetFloatValue('ExposureTime', value), 'Read automatic exposure result')
                print(f'Automatic exposure settled: {value.fCurValue:.0f} us; locked for capture.', flush=True)
                return float(value.fCurValue)
        raise TimeoutError('Automatic exposure did not settle; choose exposure manually')
    finally:
        camera.MV_CC_SetEnumValue('ExposureAuto', 0)
        if old_upper is not None:
            check(camera.MV_CC_SetIntValueEx('AutoExposureTimeUpperLimit', old_upper), 'Restore auto exposure upper limit')


def run_roi_auto_exposure(camera, roi, cancelled=lambda: False, max_steps=8, upper_limit_us=200000,
                          compensation_ev=0.0, diagnostic=None):
    """Adjust manual exposure from ROI brightness; keep full camera resolution.

    roi is normalized (x0,y0,x1,y1). The 90th percentile targets bright
    detail while a clipped-pixel check protects against substantial saturation.
    Frames are discarded after each change to avoid metering a stale exposure.
    """
    from calibration_capture import get_frame
    if not np.isfinite(compensation_ev) or not -2 <= compensation_ev <= 2:
        raise ValueError('曝光补偿应在 -2 到 +2 EV 之间')
    if len(roi)!=4 or not (0<=roi[0]<roi[2]<=1 and 0<=roi[1]<roi[3]<=1):
        raise ValueError('测光区域无效')
    state=MVCC_FLOATVALUE();check(camera.MV_CC_GetFloatValue('ExposureTime',state),'Read exposure')
    original=float(state.fCurValue)
    camera_upper = float(state.fMax)
    lower=max(float(state.fMin),15.0);upper=min(camera_upper,float(upper_limit_us))
    if upper<lower:raise ValueError('相机曝光范围无效')
    if diagnostic is not None:
        bounds = ', '.join(f'{value:.3f}' for value in roi)
        diagnostic(f'海康框选测光：原曝光 {original:.0f} 微秒，实际测光上限 {upper:.0f} 微秒'
                   f'（程序 {upper_limit_us:.0f}，相机 {camera_upper:.0f}），'
                   f'补偿 {compensation_ev:+g} EV，归一化选区 [{bounds}]。')
    check(camera.MV_CC_SetEnumValue('ExposureAuto',0),'Disable automatic exposure')
    current=max(lower,min(upper,original));status='未达到目标';p90=0.0;clipped=0.0
    best = None
    attempted = []
    no_usable_signal = False
    restart_for_metering = False
    actual = MVCC_FLOATVALUE()

    def wait_for_new_stream(exposure_us):
        deadline = time.monotonic() + .15 + exposure_us / 1e6
        while time.monotonic() < deadline:
            if cancelled():
                raise InterruptedError('用户停止区域测光')
            time.sleep(min(.02, max(0., deadline - time.monotonic())))

    def measure_at(exposure, phase='search'):
        nonlocal restart_for_metering
        if cancelled():
            raise InterruptedError('用户停止区域测光')
        if restart_for_metering:
            check(camera.MV_CC_StopGrabbing(), 'Stop stream for ROI exposure change')
        try:
            check(camera.MV_CC_SetFloatValue('ExposureTime', float(exposure)), 'Set ROI exposure')
        finally:
            if restart_for_metering:
                check(camera.MV_CC_StartGrabbing(), 'Start clean stream for ROI metering')
        check(camera.MV_CC_GetFloatValue('ExposureTime', actual), 'Read back ROI exposure')
        expected = float(actual.fCurValue)
        if restart_for_metering:
            wait_for_new_stream(expected)
        check(camera.MV_CC_ClearImageBuffer(), 'Clear buffer for ROI metering')
        samples = []
        for index in range(10):
            if cancelled():
                raise InterruptedError('用户停止区域测光')
            timing = {}
            frame = get_frame(camera, timing)
            measured = timing['camera_exposure_us']
            if index == 0:
                continue  # One frame may already have been in flight.
            matched = np.isfinite(measured) and measured > 0 and abs(measured - expected) <= max(2., expected * .02)
            missing_metadata = not np.isfinite(measured) or measured <= 0
            if missing_metadata and not restart_for_metering:
                restart_for_metering = True
                if diagnostic is not None:
                    diagnostic('海康未提供逐帧曝光值，改用重启取流后测光。')
                check(camera.MV_CC_StopGrabbing(), 'Discard stream with unverified exposure')
                check(camera.MV_CC_StartGrabbing(), 'Start clean stream for ROI metering')
                wait_for_new_stream(expected)
                check(camera.MV_CC_ClearImageBuffer(), 'Clear buffer after starting clean stream')
                samples.clear()
                continue
            if matched or missing_metadata and restart_for_metering:
                p90, clipped = roi_brightness(frame, roi)
                p99_frame = float(np.percentile(frame[::8, ::8], 99))
                samples.append((p90, clipped, p99_frame, measured if matched else None))
                if len(samples) == 2:
                    p90, clipped, p99_frame = np.median([sample[:3] for sample in samples], axis=0)
                    frame_exposures = [sample[3] for sample in samples]
                    attempted.append({'phase': phase, 'requested_us': exposure, 'actual_us': expected,
                                      'frame_exposure_us': frame_exposures,
                                      'p90': float(p90), 'clipped': float(clipped), 'frame_p99': float(p99_frame)})
                    if diagnostic is not None:
                        label = {'search': f'试拍 {len(attempted)}', 'recheck': '复测', 'compensation': '补偿后复测'}[phase]
                        frame_values = '/'.join(f'{value:.0f}' if value is not None else '未知' for value in frame_exposures)
                        diagnostic(f'海康{label}：请求 {exposure:.0f}，回读 {expected:.0f}，'
                                   f'帧曝光 {frame_values} 微秒。选区 P90={p90:.1f}，'
                                   f'全图 P99={p99_frame:.1f}，选区过亮 {clipped:.1%}。')
                    return float(p90), float(clipped)
        raise RuntimeError('曝光已写入，但连续取帧未确认新曝光')

    try:
        for iteration in range(max_steps):
            p90,clipped=measure_at(current)
            current = float(actual.fCurValue)
            sample = (exposure_score(p90, clipped), -current if p90 < 165 and clipped <= .05 else 0., current)
            if best is None or sample < best:
                best = sample
            if 165<=p90<=210 and clipped<=0.05:
                status='已达到目标';break
            if clipped>0.05:
                factor=0.65
            else:
                factor=max(0.35,min(4.0,190.0/max(p90,1.0)))
            target=max(lower,min(upper,current*factor))
            if abs(target-current)<max(1.0,current*0.02):
                status='达到有效曝光上限，选区仍偏暗' if current>=upper*.98 and p90<165 else '达到曝光下限或无法继续调整'
                break
            if iteration==max_steps-1:
                status='达到迭代次数，请检查画面';break
            current=target
        if status != '已达到目标':
            _, _, best_exposure = best
            # At essentially black levels, small differences can reflect noise.
            no_usable_signal = max(sample['p90'] for sample in attempted) <= 16
            if no_usable_signal:
                best_exposure = max(sample['actual_us'] for sample in attempted)
            if best_exposure != current:
                p90,clipped=measure_at(best_exposure, 'recheck')
                if status.startswith('达到有效曝光上限'):
                    status='曾试至有效曝光上限，已回退到实测较优曝光'
        if no_usable_signal:
            status='选区信号过低，测光未达标，暂留本轮最长曝光供排查'
        elif compensation_ev:
            adjusted=max(lower,min(upper,float(actual.fCurValue)*(2.0**compensation_ev)))
            p90,clipped=measure_at(adjusted, 'compensation')
            status=f'已应用 {compensation_ev:+g} EV 曝光补偿'
            if clipped>0.05:status+='；警告：选区过亮像素超过 5%'
            if adjusted>=upper*.999 and compensation_ev>0:status+='；已达到曝光上限'
        return {'exposure_us':float(actual.fCurValue),'p90':p90,'clipped':clipped,'status':status,'steps':iteration+1,
                'compensation_ev':compensation_ev, 'effective_upper_us':upper, 'camera_upper_us':camera_upper,
                'program_upper_us':float(upper_limit_us), 'roi':roi, 'attempts':attempted,
                'no_usable_signal':no_usable_signal}
    except Exception:
        camera.MV_CC_SetFloatValue('ExposureTime',original)
        camera.MV_CC_ClearImageBuffer()
        raise


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--test', action='store_true')
    parser.add_argument('--auto-exposure', action='store_true')
    parser.add_argument('--exposure-us', type=float)
    parser.add_argument('--gain-db', type=float)
    args = parser.parse_args()
    test = args.test
    saved_enums = {}
    camera = MvCamera()
    created = opened = grabbing = False
    previous_trigger = None
    check(MvCamera.MV_CC_Initialize(), 'SDK initialization')
    try:
        devices = MV_CC_DEVICE_INFO_LIST()
        check(MvCamera.MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, devices), 'Enumerating cameras')
        if devices.nDeviceNum != 1:
            raise RuntimeError(f'Found {devices.nDeviceNum} cameras; connect exactly one Hikrobot camera.')
        info = cast(devices.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)).contents
        check(camera.MV_CC_CreateHandle(info), 'Creating camera handle')
        created = True
        check(camera.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0), 'Opening camera (close other camera software first)')
        opened = True
        trigger = MVCC_ENUMVALUE()
        if camera.MV_CC_GetEnumValue('TriggerMode', trigger) == 0:
            previous_trigger = trigger.nCurValue
        check(camera.MV_CC_SetEnumValue('TriggerMode', MV_TRIGGER_MODE_OFF), 'Enabling continuous acquisition')
        for auto, value in [('ExposureAuto', args.exposure_us if not args.auto_exposure else 1), ('GainAuto', args.gain_db)]:
            if value is not None:
                state = MVCC_ENUMVALUE()
                check(camera.MV_CC_GetEnumValue(auto, state), f'Read {auto}')
                saved_enums[auto] = state.nCurValue
                check(camera.MV_CC_SetEnumValue(auto, 0), f'Set {auto} off')
        if args.exposure_us is not None and not args.auto_exposure:
            set_float_parameter(camera, 'ExposureTime', args.exposure_us)
        if args.gain_db is not None:
            set_float_parameter(camera, 'Gain', args.gain_db)
        check(camera.MV_CC_StartGrabbing(), 'Starting acquisition')
        grabbing = True
        if args.auto_exposure:
            run_auto_exposure_once(camera)
        print('Preview: Q or Esc to quit. S saves a full-resolution PNG.', flush=True)
        while True:
            frame = MV_FRAME_OUT()
            check(camera.MV_CC_GetImageBuffer(frame, 3000), 'Receiving frame')
            try:
                width, height = frame.stFrameInfo.nWidth, frame.stFrameInfo.nHeight
                dest = (c_ubyte * (width * height))()
                conv = MV_CC_PIXEL_CONVERT_PARAM_EX()
                conv.nWidth, conv.nHeight = width, height
                conv.pSrcData, conv.nSrcDataLen = frame.pBufAddr, frame.stFrameInfo.nFrameLen
                conv.enSrcPixelType = frame.stFrameInfo.enPixelType
                conv.enDstPixelType = PixelType_Gvsp_Mono8
                conv.pDstBuffer, conv.nDstBufferSize = dest, width * height
                check(camera.MV_CC_ConvertPixelTypeEx(conv), 'Converting to Mono8')
                img = np.frombuffer(dest, dtype=np.uint8).reshape(height, width).copy()
            finally:
                camera.MV_CC_FreeImageBuffer(frame)
            if test:
                print(f'Frame received: {width}x{height}, min={img.min()}, max={img.max()}, mean={img.mean():.1f}', flush=True)
                break
            scale = min(1.0, 1200 / width, 800 / height)
            view = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) if scale < 1 else img
            cv2.imshow('Hikrobot preview - S: save, Q: quit', view)
            key = cv2.waitKey(1) & 0xff
            if key in (ord('q'), ord('Q'), 27):
                break
            if key in (ord('s'), ord('S')):
                from datetime import datetime
                path = Path(__file__).parent / ('capture_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.png')
                ok, encoded = cv2.imencode('.png', img)
                if ok:
                    encoded.tofile(str(path))
                    print('Saved:', path, flush=True)
                else:
                    print('Failed to encode image', flush=True)
    finally:
        if grabbing:
            camera.MV_CC_StopGrabbing()
        if opened:
            for name, value in saved_enums.items():
                camera.MV_CC_SetEnumValue(name, value)
        if opened and previous_trigger is not None:
            camera.MV_CC_SetEnumValue('TriggerMode', previous_trigger)
        if opened:
            camera.MV_CC_CloseDevice()
        if created:
            camera.MV_CC_DestroyHandle()
        MvCamera.MV_CC_Finalize()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('Error:', exc, flush=True)
        sys.exit(1)
