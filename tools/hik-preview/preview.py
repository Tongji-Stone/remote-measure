import os
import argparse
import sys
import time
from ctypes import byref, cast, POINTER
from pathlib import Path

import cv2
import numpy as np

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


def run_roi_auto_exposure(camera, roi, cancelled=lambda: False, max_steps=8, upper_limit_us=200000):
    """Adjust manual exposure from ROI brightness; keep full camera resolution.

    roi is normalized (x0,y0,x1,y1). The 90th percentile targets bright
    detail while a clipped-pixel check protects against substantial saturation.
    Frames are discarded after each change to avoid metering a stale exposure.
    """
    from calibration_capture import get_frame
    if len(roi)!=4 or not (0<=roi[0]<roi[2]<=1 and 0<=roi[1]<roi[3]<=1):
        raise ValueError('测光区域无效')
    state=MVCC_FLOATVALUE();check(camera.MV_CC_GetFloatValue('ExposureTime',state),'Read exposure')
    original=float(state.fCurValue)
    lower=max(float(state.fMin),15.0);upper=min(float(state.fMax),float(upper_limit_us))
    if upper<lower:raise ValueError('相机曝光范围无效')
    check(camera.MV_CC_SetEnumValue('ExposureAuto',0),'Disable automatic exposure')
    current=max(lower,min(upper,original));status='未达到目标';p90=0.0;clipped=0.0
    try:
        for iteration in range(max_steps):
            if cancelled():raise InterruptedError('用户停止区域测光')
            check(camera.MV_CC_SetFloatValue('ExposureTime',float(current)),'Set ROI exposure')
            check(camera.MV_CC_ClearImageBuffer(),'Clear buffer for ROI metering')
            get_frame(camera);img=get_frame(camera)
            h,w=img.shape[:2];x0,y0,x1,y1=roi
            x0=max(0,min(w-1,int(x0*w)));x1=max(x0+1,min(w,int(x1*w)))
            y0=max(0,min(h-1,int(y0*h)));y1=max(y0+1,min(h,int(y1*h)))
            part=img[y0:y1,x0:x1]
            if part.size<400:raise ValueError('测光框过小，请框住更大的物体区域')
            # Subsampling bounds CPU work while preserving spatial coverage.
            sample=part[::max(1,part.shape[0]//400),::max(1,part.shape[1]//400)]
            p90=float(np.percentile(sample,90));clipped=float(np.mean(sample>=250))
            if 165<=p90<=210 and clipped<=0.05:
                status='已达到目标';break
            if clipped>0.05:
                factor=0.65
            else:
                factor=max(0.35,min(4.0,190.0/max(p90,1.0)))
            target=max(lower,min(upper,current*factor))
            if abs(target-current)<max(1.0,current*0.02):
                status='达到曝光上限' if current>=upper*.98 and p90<165 else '达到曝光下限或无法继续调整'
                break
            if iteration==max_steps-1:
                status='达到迭代次数，请检查画面';break
            current=target
        actual=MVCC_FLOATVALUE();check(camera.MV_CC_GetFloatValue('ExposureTime',actual),'Read ROI exposure')
        return {'exposure_us':float(actual.fCurValue),'p90':p90,'clipped':clipped,'status':status,'steps':iteration+1}
    except Exception:
        camera.MV_CC_SetFloatValue('ExposureTime',original)
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
