import os
import sys
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


def main():
    test = '--test' in sys.argv
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
        check(camera.MV_CC_StartGrabbing(), 'Starting acquisition')
        grabbing = True
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
