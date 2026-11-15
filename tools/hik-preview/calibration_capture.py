"""Capture OpenCV Gray-code sequences for procam-calibration on Windows."""
import ctypes
import sys
import time
from pathlib import Path
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor
from collections import deque

ctypes.windll.user32.SetProcessDpiAwarenessContext(ctypes.c_void_p(-4))
from preview import *

OUT = Path(__file__).resolve().parents[2] / 'data' / 'procam_calibration'
WAIT_SECONDS = 0.01


def save_frame(path, img):
    ok, encoded = cv2.imencode('.png', img, [cv2.IMWRITE_PNG_COMPRESSION, 1])
    if not ok:
        raise RuntimeError(f'Image encoding failed: {path}')
    encoded.tofile(str(path))


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


def get_frame(cam):
    frame = MV_FRAME_OUT()
    check(cam.MV_CC_GetImageBuffer(frame, 4000), 'Receiving frame')
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


def main():
    test_once = '--test-once' in sys.argv
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
    print(f'{len(patterns)} patterns per pose. SPACE: capture, E: exposure x2, D: exposure /2, Q: quit.', flush=True)
    check(MvCamera.MV_CC_Initialize(), 'SDK initialization')
    cam = MvCamera()
    created = opened = grabbing = False
    saved_enums = {}
    try:
        devices = MV_CC_DEVICE_INFO_LIST()
        check(MvCamera.MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, devices), 'Enumerating cameras')
        if devices.nDeviceNum != 1:
            raise RuntimeError(f'Expected one camera, found {devices.nDeviceNum}')
        check(cam.MV_CC_CreateHandle(cast(devices.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)).contents), 'Create handle')
        created = True
        check(cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0), 'Open camera; close preview or MVS first')
        opened = True
        for name in ('TriggerMode', 'ExposureAuto', 'GainAuto'):
            value = MVCC_ENUMVALUE()
            if cam.MV_CC_GetEnumValue(name, value) == 0:
                saved_enums[name] = value.nCurValue
                check(cam.MV_CC_SetEnumValue(name, 0), f'Set {name} off')
        cv2.namedWindow('Projector', cv2.WINDOW_NORMAL)
        cv2.moveWindow('Projector', x, y)
        cv2.setWindowProperty('Projector', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.imshow('Projector', patterns[-2])
        cv2.waitKey(500)
        check(cam.MV_CC_StartGrabbing(), 'Start grabbing')
        grabbing = True
        cv2.namedWindow('Calibration preview - SPACE capture, Q quit', cv2.WINDOW_NORMAL)
        while True:
            img = get_frame(cam)
            scale = min(1., 1200/img.shape[1], 800/img.shape[0])
            view = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            cv2.imshow('Calibration preview - SPACE capture, Q quit', view)
            key = 32 if test_once else cv2.waitKey(1) & 255
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
            OUT.mkdir(parents=True, exist_ok=True)
            prefix = 'test_' if test_once else 'capture_'
            folder = OUT / (prefix + datetime.now().strftime('%Y%m%d_%H%M%S'))
            folder.mkdir()
            print('Capturing:', folder, flush=True)
            start = time.monotonic()
            pending = deque()
            with ThreadPoolExecutor(max_workers=2) as writer:
                for i, pattern in enumerate(patterns):
                    cv2.imshow('Projector', pattern)
                    cv2.waitKey(int(WAIT_SECONDS * 1000))
                    # Clear queued frames, then discard one in-flight frame before saving.
                    check(cam.MV_CC_ClearImageBuffer(), 'Clear camera buffer')
                    get_frame(cam)
                    img = get_frame(cam)
                    preview = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
                    cv2.imshow('Calibration preview - SPACE capture, Q quit', preview)
                    cv2.waitKey(1)
                    pending.append(writer.submit(save_frame, folder / f'graycode_{i:02d}.png', img))
                    if len(pending) >= 4:
                        pending.popleft().result()
                    print(f'  {i+1}/{len(patterns)}', flush=True)
                while pending:
                    pending.popleft().result()
            cv2.imshow('Projector', patterns[-2])
            cv2.waitKey(int(WAIT_SECONDS * 1000))
            check(cam.MV_CC_ClearImageBuffer(), 'Clear camera buffer')
            get_frame(cam)
            print(f'Capture complete in {time.monotonic()-start:.1f} s. Move the board, then press SPACE again.', flush=True)
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


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('Error:', exc, flush=True)
        sys.exit(1)
