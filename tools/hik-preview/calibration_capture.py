"""Capture OpenCV Gray-code sequences for procam-calibration on Windows."""
import ctypes
import builtins
import argparse
import sys
import time
import json
from pathlib import Path
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor
from collections import deque

ctypes.windll.user32.SetProcessDpiAwarenessContext(ctypes.c_void_p(-4))
from preview import *

OUT = Path(__file__).resolve().parents[2] / 'data' / 'procam_calibration'
WAIT_SECONDS = 0.01  # UI/projector event processing for non-capture resets
PROJECTOR_SETTLE_SECONDS = 0.10  # conservative allowance; verify with the physical projector


def capture_projected_pattern(camera, pattern, exposure_us, ui=None, settle_seconds=PROJECTOR_SETTLE_SECONDS):
    """Hold each displayed pattern across a settling margin and full exposure.

    HDMI/projector presentation has no hardware acknowledgement. The margin is
    empirical, not a proof of optical synchronization. Keep the existing buffer
    clear and discard an additional in-flight frame after the hold.
    """
    if exposure_us <= 0 or settle_seconds < 0:
        raise ValueError('曝光时间或投影稳定等待时间无效')
    timings = {'exposure_us': float(exposure_us), 'projector_settle_margin_seconds': float(settle_seconds)}
    started = time.perf_counter()
    cv2.imshow('Projector', pattern)
    cv2.waitKey(1)
    # Allow the image to reach the projector, then one whole exposure interval
    # before flushing frames that may have started under the previous pattern.
    required = settle_seconds + exposure_us / 1_000_000.0
    deadline = time.perf_counter() + required
    while True:
        if ui is not None and ui.cancelled():raise CaptureInterrupted('用户停止；不完整的一组已标记为中断')
        remaining = deadline-time.perf_counter()
        if remaining <= 0:break
        cv2.waitKey(max(1,min(20,int(remaining*1000))))
    timings['projection_display_wait_seconds'] = time.perf_counter() - started
    tick = time.perf_counter()
    check(camera.MV_CC_ClearImageBuffer(), 'Clear camera buffer')
    timings['clear_buffer_seconds'] = time.perf_counter() - tick
    timings['discarded_frame'] = {}
    get_frame(camera, timings['discarded_frame'])
    if ui is not None and ui.cancelled():raise CaptureInterrupted('用户停止；不完整的一组已标记为中断')
    timings['saved_frame'] = {}
    img = get_frame(camera, timings['saved_frame'])
    return img, timings


def save_frame(path, img):
    start = time.perf_counter()
    ok, encoded = cv2.imencode('.png', img, [cv2.IMWRITE_PNG_COMPRESSION, 1])
    encoded_at = time.perf_counter()
    if not ok:
        raise RuntimeError(f'Image encoding failed: {path}')
    encoded.tofile(str(path))
    return {'encode_seconds': encoded_at - start, 'write_seconds': time.perf_counter() - encoded_at}


def monitors():
    rects = []
    class RECT(ctypes.Structure):
        _fields_ = [('left', ctypes.c_long), ('top', ctypes.c_long), ('right', ctypes.c_long), ('bottom', ctypes.c_long)]
    callback_type = ctypes.WINFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(RECT), ctypes.c_longlong)
    def callback(monitor, dc, rect, data):
        r = rect.contents
        rects.append((r.left, r.top, r.right-r.left, r.bottom-r.top))
        return 1
    callback_ref = callback_type(callback)
    ctypes.windll.user32.EnumDisplayMonitors(None, None, callback_ref, 0)
    return rects


def get_frame(cam, timing=None):
    frame = MV_FRAME_OUT()
    start = time.perf_counter()
    check(cam.MV_CC_GetImageBuffer(frame, 4000), 'Receiving frame')
    acquired = time.perf_counter()
    try:
        width, height = frame.stFrameInfo.nWidth, frame.stFrameInfo.nHeight
        buf = (c_ubyte * (width * height))()
        conv = MV_CC_PIXEL_CONVERT_PARAM_EX()
        conv.nWidth, conv.nHeight = width, height
        conv.pSrcData, conv.nSrcDataLen = frame.pBufAddr, frame.stFrameInfo.nFrameLen
        conv.enSrcPixelType = frame.stFrameInfo.enPixelType
        conv.enDstPixelType = PixelType_Gvsp_Mono8
        conv.pDstBuffer, conv.nDstBufferSize = buf, width * height
        check(cam.MV_CC_ConvertPixelTypeEx(conv), 'Converting frame')
        return np.frombuffer(buf, dtype=np.uint8).reshape(height, width).copy()
    finally:
        cam.MV_CC_FreeImageBuffer(frame)
        if timing is not None:
            timing['buffer_wait_seconds'] = acquired - start
            timing['conversion_and_release_seconds'] = time.perf_counter() - acquired


class CaptureInterrupted(Exception):
    pass


def main(mode='calibration', nominal_step_degrees=None, auto_turntable=None, exposure_us=None, gain_db=None, auto_exposure=False, ui=None):
    print = ui.log if ui else builtins.print
    # The dashboard is system-DPI-aware. Query projector coordinates in physical
    # pixels even when the projector has a different Windows scaling factor.
    set_thread_dpi = getattr(ctypes.windll.user32, 'SetThreadDpiAwarenessContext', None)
    previous_thread_dpi = None
    if set_thread_dpi:
        set_thread_dpi.argtypes = [ctypes.c_void_p]
        set_thread_dpi.restype = ctypes.c_void_p
        previous_thread_dpi = set_thread_dpi(ctypes.c_void_p(-4))
    if mode not in ('calibration', 'object'):
        raise ValueError('Unknown capture mode')
    test_once = mode == 'calibration' and '--test-once' in sys.argv
    screens = monitors()
    if len(screens) != 2:
        raise RuntimeError(f'Need exactly two extended displays; found {len(screens)}.')
    x, y, w, h = screens[1]
    print(f'Projector screen: {w}x{h} at ({x},{y})')
    gray = cv2.structured_light_GrayCodePattern.create(w, h)
    ok, patterns = gray.generate()
    if not ok:
        raise RuntimeError('Could not generate Gray code patterns')
    patterns = list(patterns) + [np.full((h, w), 255, np.uint8), np.zeros((h, w), np.uint8)]
    preview_window = ('Object scan - SPACE capture, Q quit' if mode == 'object'
                      else 'Calibration preview - SPACE capture, Q quit')
    print(f'{len(patterns)} patterns per pose.' + (' Use the dashboard buttons to capture and stop.' if ui else ' SPACE: capture, E: exposure x2, D: exposure /2, Q: quit.'), flush=True)
    check(MvCamera.MV_CC_Initialize(), 'SDK initialization')
    cam = MvCamera()
    created = opened = grabbing = False
    saved_enums = {}
    session = None
    session_info = None
    try:
        devices = MV_CC_DEVICE_INFO_LIST()
        check(MvCamera.MV_CC_EnumDevices(MV_USB_DEVICE if ui else (MV_USB_DEVICE | MV_GIGE_DEVICE), devices), 'Enumerating cameras')
        if devices.nDeviceNum != 1:
            raise RuntimeError(f'Expected one camera, found {devices.nDeviceNum}')
        check(cam.MV_CC_CreateHandle(cast(devices.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)).contents), 'Create handle')
        created = True
        check(cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0), 'Open camera; close preview or MVS first')
        opened = True
        for name in ('TriggerMode', 'ExposureAuto', 'GainAuto'):
            value = MVCC_ENUMVALUE()
            code = cam.MV_CC_GetEnumValue(name, value)
            if code == 0:
                saved_enums[name] = value.nCurValue
                check(cam.MV_CC_SetEnumValue(name, 0), f'Set {name} off')
            elif (name == 'ExposureAuto' and (exposure_us is not None or auto_exposure)) or (name == 'GainAuto' and gain_db is not None):
                check(code, f'Read {name}')
        if exposure_us is not None and not auto_exposure:
            set_float_parameter(cam, 'ExposureTime', exposure_us)
        if gain_db is not None:
            set_float_parameter(cam, 'Gain', gain_db)
        cv2.namedWindow('Projector', cv2.WINDOW_NORMAL)
        cv2.moveWindow('Projector', x, y)
        cv2.setWindowProperty('Projector', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.imshow('Projector', patterns[-2])
        cv2.waitKey(500)
        check(cam.MV_CC_StartGrabbing(), 'Start grabbing')
        grabbing = True
        if auto_exposure:
            actual_exposure=run_auto_exposure_once(cam)
            print(f'自动曝光完成：{actual_exposure:.0f} 微秒（已锁定）；请检查亮度与高光。',flush=True)
            if ui is not None:ui.auto_exposure_result(actual_exposure)
            check(cam.MV_CC_ClearImageBuffer(), 'Clear buffer after auto exposure')
        if ui is None:
            cv2.namedWindow(preview_window, cv2.WINDOW_NORMAL)
        if mode == 'object':
            base = OUT.parent / 'object_scans'
            base.mkdir(parents=True, exist_ok=True)
            session = base / ('scan_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
            session.mkdir()
            session_info = {'mode': 'manual_turntable', 'created': datetime.now().isoformat(timespec='seconds'),
                            'projector_pixels': [w, h], 'pattern_count': len(patterns),
                            'projector_settle_margin_seconds': PROJECTOR_SETTLE_SECONDS, 'timing_policy': 'settle margin plus actual exposure, then clear and discard one frame', 'nominal_step_degrees': nominal_step_degrees,
                            'angle_measurement': 'nominal, commanded; not measured' if auto_turntable else ('nominal, manually entered; not measured' if nominal_step_degrees is not None else 'not recorded'), 'views': []}
            if auto_turntable:
                session_info['mode'] = 'automatic_turntable'
                session_info['turntable'] = auto_turntable.settings()
            print('Object scan folder:', session, flush=True)
            print(('自动采集即将开始。' if auto_turntable else '请通过操作台按钮采集视角。') if ui else ('SPACE: start automatic scan; Q: quit.' if auto_turntable else 'SPACE: capture one stationary view. Rotate manually only after Capture complete.'), flush=True)
        if mode == 'calibration':
            OUT.mkdir(parents=True, exist_ok=True)
            session = OUT / ('calibration_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
            session.mkdir()
            (session / 'session.json').write_text(json.dumps({'mode':'calibration','created':datetime.now().isoformat(timespec='seconds'),'projector_pixels':[w,h],'pattern_count':len(patterns)},ensure_ascii=False,indent=2),encoding='utf8')
            print('Calibration session folder:', session, flush=True)
        auto_started = False
        while True:
            img = get_frame(cam)
            scale = min(1., 1200/img.shape[1], 800/img.shape[0])
            view = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            if ui is not None:
                ui.show_frame(img)
                cv2.waitKey(1)  # Dispatch projector events on this thread.
                key = 27 if ui.cancelled() else (32 if auto_started else ui.poll_key())
            else:
                cv2.imshow(preview_window, view)
                key = 32 if (test_once or auto_started) else cv2.waitKey(1) & 255
            if key in (27, ord('q'), ord('Q')):
                break
            if key in (ord('e'), ord('E'), ord('d'), ord('D')):
                current = MVCC_FLOATVALUE()
                check(cam.MV_CC_GetFloatValue('ExposureTime', current), 'Read exposure')
                factor = 2.0 if key in (ord('e'), ord('E')) else 0.5
                target = max(current.fMin, min(current.fMax, current.fCurValue * factor))
                check(cam.MV_CC_SetFloatValue('ExposureTime', target), 'Set exposure')
                print(f'Exposure time: {target:.0f} us', flush=True)
                continue
            if key != 32:
                continue
            if ui is not None:
                ui.set_capturing(True)
            if auto_turntable and not auto_started:
                auto_turntable.check_idle()
                auto_started = True
            if mode == 'object':
                folder = session / f"view_{len(session_info['views']):04d}"
                frameinfo = {'folder': folder.name, 'started': datetime.now().isoformat(timespec='seconds'),
                             'status': 'capturing', 'nominal_angle_degrees': (len(session_info['views']) * nominal_step_degrees if nominal_step_degrees is not None else None)}
                for parameter, field in [('ExposureTime', 'exposure_us'), ('Gain', 'gain_db')]:
                    value = MVCC_FLOATVALUE()
                    if cam.MV_CC_GetFloatValue(parameter, value) == 0:
                        frameinfo[field] = value.fCurValue
                session_info['views'].append(frameinfo)
            else:
                OUT.mkdir(parents=True, exist_ok=True)
                prefix = 'test_' if test_once else 'capture_'
                folder = session / (prefix + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
            folder.mkdir()
            if mode == 'object':
                (session / 'session.json').write_text(json.dumps(session_info, ensure_ascii=False, indent=2), encoding='utf-8')
            print('Capturing:', folder, flush=True)
            start = time.monotonic()
            frame_times = []
            camera_rate = MVCC_FLOATVALUE()
            resulting_fps = float(camera_rate.fCurValue) if cam.MV_CC_GetFloatValue('ResultingFrameRate', camera_rate) == 0 else None
            exposure_value = MVCC_FLOATVALUE()
            check(cam.MV_CC_GetFloatValue('ExposureTime', exposure_value), 'Read locked exposure for synchronization')
            locked_exposure_us = float(exposure_value.fCurValue)
            print(f'图案时序：投影稳定余量 {PROJECTOR_SETTLE_SECONDS*1000:.0f} ms + 曝光 {locked_exposure_us/1000:.1f} ms，再清缓存并丢弃一帧。',flush=True)
            pending = deque()
            with ThreadPoolExecutor(max_workers=2) as writer:
                for i, pattern in enumerate(patterns):
                    if ui is not None and ui.cancelled():
                        if mode == 'object':
                            frameinfo['status']='interrupted'
                            (session / 'session.json').write_text(json.dumps(session_info, ensure_ascii=False, indent=2), encoding='utf-8')
                        else:
                            (folder / 'interrupted.json').write_text(json.dumps({'status':'interrupted','saved_frames':i}), encoding='utf-8')
                        raise CaptureInterrupted('用户停止；不完整的一组已标记为中断')
                    try:
                        img, timings = capture_projected_pattern(cam, pattern, locked_exposure_us, ui=ui)
                    except CaptureInterrupted:
                        if mode == 'object':
                            frameinfo['status']='interrupted'
                            (session / 'session.json').write_text(json.dumps(session_info, ensure_ascii=False, indent=2), encoding='utf-8')
                        else:
                            (folder / 'interrupted.json').write_text(json.dumps({'status':'interrupted','saved_frames':i}), encoding='utf-8')
                        raise
                    timings['frame'] = i
                    tick = time.perf_counter()
                    preview = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
                    if ui is None:
                        cv2.imshow(preview_window, preview)
                    else:
                        ui.show_frame(img)
                    cv2.waitKey(1)
                    timings['preview_seconds'] = time.perf_counter() - tick
                    tick = time.perf_counter()
                    pending.append((writer.submit(save_frame, folder / f'graycode_{i:02d}.png', img), timings))
                    if len(pending) >= 4:
                        future, saved_timing = pending.popleft()
                        saved_timing['background_save'] = future.result()
                    timings['writer_queue_wait_seconds'] = time.perf_counter() - tick
                    frame_times.append(timings)
                    print(f'  {i+1}/{len(patterns)}', flush=True)
                tick = time.perf_counter()
                while pending:
                    future, saved_timing = pending.popleft()
                    saved_timing['background_save'] = future.result()
                final_writer_wait = time.perf_counter() - tick
            if mode == 'object':
                frameinfo['status'] = 'complete'
                frameinfo['finished'] = datetime.now().isoformat(timespec='seconds')
                (session / 'session.json').write_text(json.dumps(session_info, ensure_ascii=False, indent=2), encoding='utf-8')
            cv2.imshow('Projector', patterns[-2])
            cv2.waitKey(int(WAIT_SECONDS * 1000))
            check(cam.MV_CC_ClearImageBuffer(), 'Clear camera buffer')
            get_frame(cam)
            elapsed = time.monotonic() - start
            totals = {
                'projection_display_wait_seconds': sum(t['projection_display_wait_seconds'] for t in frame_times),
                'buffer_clear_seconds': sum(t['clear_buffer_seconds'] for t in frame_times),
                'camera_buffer_wait_seconds': sum(t[k]['buffer_wait_seconds'] for t in frame_times for k in ('discarded_frame', 'saved_frame')),
                'frame_conversion_seconds': sum(t[k]['conversion_and_release_seconds'] for t in frame_times for k in ('discarded_frame', 'saved_frame')),
                'preview_seconds': sum(t['preview_seconds'] for t in frame_times),
                'writer_queue_wait_seconds': sum(t['writer_queue_wait_seconds'] for t in frame_times),
                'final_writer_wait_seconds': final_writer_wait,
                'background_encode_seconds_sum': sum(t['background_save']['encode_seconds'] for t in frame_times),
                'background_disk_write_seconds_sum': sum(t['background_save']['write_seconds'] for t in frame_times),
            }
            report = {'total_seconds': elapsed, 'frame_count': len(frame_times), 'exposure_us': locked_exposure_us, 'projector_settle_margin_seconds': PROJECTOR_SETTLE_SECONDS, 'timing_policy': 'settle margin plus exposure, clear buffer, discard one, save next; optical latency unverified', 'resulting_fps_reported_at_start': resulting_fps,
                      'totals': totals, 'frames': frame_times,
                      'note': 'Background encode/write run concurrently; their sums overlap capture time and must not be added to wall time. Total includes final projector reset.'}
            (folder / 'timing.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
            if mode == 'object':
                frameinfo['timing_file'] = folder.name + '/timing.json'
                (session / 'session.json').write_text(json.dumps(session_info, ensure_ascii=False, indent=2), encoding='utf-8')
            print(f'Capture complete in {elapsed:.2f} s. Timing: {folder / "timing.json"}', flush=True)
            print(f"Projection wait: {totals['projection_display_wait_seconds']:.2f}s; camera wait: {totals['camera_buffer_wait_seconds']:.2f}s; "
                  f"conversion: {totals['frame_conversion_seconds']:.2f}s; preview: {totals['preview_seconds']:.2f}s; "
                  f"waiting for saves: {totals['writer_queue_wait_seconds'] + final_writer_wait:.2f}s. "
                  f"Encode+write worker time (overlapping): {totals['background_encode_seconds_sum'] + totals['background_disk_write_seconds_sum']:.2f}s.", flush=True)
            if ui is not None and ui.cancelled():
                raise CaptureInterrupted('用户停止')
            if auto_turntable:
                if len(session_info['views']) >= auto_turntable.views:
                    print('Automatic scan complete.', flush=True)
                    break
                print('Rotating turntable; press Q in preview to stop.', flush=True)
                frameinfo['turn_after_capture'] = {'status': 'commanding'}
                (session / 'session.json').write_text(json.dumps(session_info, ensure_ascii=False, indent=2), encoding='utf-8')
                try:
                    auto_turntable.move_and_wait(lambda: (ui.cancelled() if ui is not None else (cv2.waitKey(10) & 255) in (27, ord('q'), ord('Q'))))
                    frameinfo['turn_after_capture']['status'] = 'completed_reported_by_controller'
                except Exception as exc:
                    frameinfo['turn_after_capture'].update(status='failed_or_unknown', error=str(exc))
                    (session / 'session.json').write_text(json.dumps(session_info, ensure_ascii=False, indent=2), encoding='utf-8')
                    raise
                (session / 'session.json').write_text(json.dumps(session_info, ensure_ascii=False, indent=2), encoding='utf-8')
            else:
                print('请移动物体或标定板，摆稳后点击采集当前视角。' if ui else 'Move the object/board, then press SPACE again.', flush=True)
                if ui is not None:ui.set_capturing(False)
            if test_once:
                break
    finally:
        if grabbing:
            cam.MV_CC_StopGrabbing()
        if opened:
            for name, value in saved_enums.items():
                cam.MV_CC_SetEnumValue(name, value)
            cam.MV_CC_CloseDevice()
        if created:
            cam.MV_CC_DestroyHandle()
        MvCamera.MV_CC_Finalize()
        cv2.destroyAllWindows()
        if set_thread_dpi and previous_thread_dpi:
            set_thread_dpi(previous_thread_dpi)


if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument('--test-once', action='store_true')
        parser.add_argument('--auto-exposure', action='store_true')
        parser.add_argument('--exposure-us', type=float)
        parser.add_argument('--gain-db', type=float)
        args = parser.parse_args()
        main(exposure_us=args.exposure_us, gain_db=args.gain_db, auto_exposure=args.auto_exposure)
    except Exception as exc:
        print('Error:', exc, flush=True)
        sys.exit(1)
