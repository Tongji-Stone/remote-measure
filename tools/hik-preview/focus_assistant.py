"""Manual focus guide: camera texture then projected stripes, with audio feedback."""
import argparse
import threading
import time
import winsound

from calibration_capture import *


def focus_score(gray):
    gray = np.asarray(gray, dtype=np.uint8)
    if gray.size < 400:
        return 0.0, 0.0
    saturation = float(np.mean((gray <= 5) | (gray >= 250)))
    score = float(cv2.Laplacian(cv2.GaussianBlur(gray, (3, 3), 0), cv2.CV_64F).var())
    return score, saturation


def main(ui=None, exposure_us=50000, gain_db=0, auto_exposure=False):
    if ui is None:
        ap=argparse.ArgumentParser();ap.add_argument('--exposure-us',type=float,default=50000);ap.add_argument('--gain-db',type=float,default=0);ap.add_argument('--auto-exposure',action='store_true');args=ap.parse_args()
        exposure_us,gain_db,auto_exposure=args.exposure_us,args.gain_db,args.auto_exposure
    set_thread_dpi=getattr(ctypes.windll.user32,'SetThreadDpiAwarenessContext',None)
    old_dpi=None
    if set_thread_dpi:
        set_thread_dpi.argtypes=[ctypes.c_void_p];set_thread_dpi.restype=ctypes.c_void_p
        old_dpi=set_thread_dpi(ctypes.c_void_p(-4))
    screens=monitors()
    if len(screens)!=2:raise RuntimeError(f'Need two extended displays; found {len(screens)}')
    x,y,w,h=screens[1]
    camera=MvCamera();created=opened=grabbing=False;saved={};mode=0;roi=None;best=0.0;history=[];busy=False;last_beep=0
    check(MvCamera.MV_CC_Initialize(),'SDK initialization')
    try:
        devices=MV_CC_DEVICE_INFO_LIST();check(MvCamera.MV_CC_EnumDevices((MV_USB_DEVICE if ui else MV_USB_DEVICE|MV_GIGE_DEVICE),devices),'Enumerating cameras')
        if devices.nDeviceNum!=1:raise RuntimeError(f'Expected one camera, found {devices.nDeviceNum}')
        check(camera.MV_CC_CreateHandle(cast(devices.pDeviceInfo[0],POINTER(MV_CC_DEVICE_INFO)).contents),'Create handle');created=True
        check(camera.MV_CC_OpenDevice(MV_ACCESS_Exclusive,0),'Open camera');opened=True
        for name in ('TriggerMode','ExposureAuto','GainAuto'):
            value=MVCC_ENUMVALUE();check(camera.MV_CC_GetEnumValue(name,value),f'Read {name}');saved[name]=value.nCurValue;check(camera.MV_CC_SetEnumValue(name,0),f'Set {name} off')
        if not auto_exposure:set_float_parameter(camera,'ExposureTime',exposure_us)
        set_float_parameter(camera,'Gain',gain_db)
        cv2.namedWindow('Projector',cv2.WINDOW_NORMAL);cv2.moveWindow('Projector',x,y);cv2.setWindowProperty('Projector',cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
        patterns=[np.full((h,w),255,np.uint8),np.broadcast_to(((np.arange(w)//8)%2*255).astype(np.uint8),(h,w)).copy(),np.broadcast_to((((np.arange(h)//8)%2)*255).astype(np.uint8)[:,None],(h,w)).copy()]
        cv2.imshow('Projector',patterns[0]);cv2.waitKey(300);check(camera.MV_CC_StartGrabbing(),'Start grabbing');grabbing=True
        if auto_exposure:
            actual=run_auto_exposure_once(camera)
            if ui is not None:ui.auto_exposure_result(actual)
        print('Focus: R select ROI; M change target; B reset best; Q exit. Camera lens first, projector second.',flush=True)
        while True:
            img=get_frame(camera);scale=min(1.,1100/img.shape[1],750/img.shape[0]);view=cv2.resize(img,None,fx=scale,fy=scale,interpolation=cv2.INTER_AREA);H,W=view.shape
            if roi is None: roi=(int(W*.2),int(H*.2),int(W*.6),int(H*.6))
            rx,ry,rw,rh=roi;part=view[ry:ry+rh,rx:rx+rw];score,saturation=focus_score(part);history=(history+[score])[-5:];smoothed=float(np.median(history));best=max(best,smoothed);ratio=smoothed/best if best>0 else 0
            rgb=cv2.cvtColor(view,cv2.COLOR_GRAY2BGR);cv2.rectangle(rgb,(rx,ry),(rx+rw,ry+rh),(0,255,0),2)
            names=['CAMERA: use physical texture, white projector','PROJECTOR: vertical stripes','PROJECTOR: horizontal stripes']
            lines=[names[mode],f'Sharpness {smoothed:.1f}  best {best:.1f}  relative {ratio:.0%}',f'Dark/bright clipping {saturation:.0%}; use textured ROI; high pitch = near best','R: ROI   M: target   B: reset   E/D: exposure x2/0.5   Q: quit']
            for i,t in enumerate(lines):cv2.putText(rgb,t,(12,24+i*24),cv2.FONT_HERSHEY_SIMPLEX,.52,(0,220,255),1,cv2.LINE_AA)
            if ui is None:
                cv2.imshow('Focus guide',rgb);key=cv2.waitKey(1)&255
            else:
                # Send the full-resolution sensor frame for a genuinely detailed
                # ROI display; score/ROI coordinates remain based on `view`.
                ui.show_frame(img)
                ui.focus_status(mode,smoothed,best,ratio,saturation)
                cv2.waitKey(1)
                key=27 if ui.cancelled() else ui.poll_key()
                if isinstance(key,tuple) and key[0]=='auto_roi':
                    try:
                        result=run_roi_auto_exposure(camera,key[1],cancelled=ui.cancelled)
                        best=0;history=[]
                    except Exception as exc:result=str(exc)
                    ui.roi_exposure_result(result)
                    key=-1
                if isinstance(key,tuple) and key[0]=='roi':
                    _,x0,y0,x1,y1=key
                    ix0,iy0=max(0,min(W-1,int(x0*W))),max(0,min(H-1,int(y0*H)))
                    ix1,iy1=max(0,min(W,int(x1*W))),max(0,min(H,int(y1*H)))
                    if ix1-ix0>10 and iy1-iy0>10:
                        roi=(ix0,iy0,ix1-ix0,iy1-iy0);best=0;history=[]
                    key=-1
            if key in (27,ord('q'),ord('Q')):break
            if key in (ord('r'),ord('R')):
                chosen=cv2.selectROI('Focus guide',view,showCrosshair=True,fromCenter=False)
                if chosen[2]>10 and chosen[3]>10:roi=tuple(int(a) for a in chosen);best=0;history=[]
            if key in (ord('m'),ord('M')):
                mode=(mode+1)%3;cv2.imshow('Projector',patterns[mode]);cv2.waitKey(100);check(camera.MV_CC_ClearImageBuffer(),'Clear buffer');get_frame(camera);best=0;history=[]
            if key in (ord('e'),ord('E'),ord('d'),ord('D')):
                current=MVCC_FLOATVALUE();check(camera.MV_CC_GetFloatValue('ExposureTime',current),'Read exposure')
                factor=2.0 if key in (ord('e'),ord('E')) else 0.5
                target=max(current.fMin,min(current.fMax,current.fCurValue*factor));set_float_parameter(camera,'ExposureTime',target);best=0;history=[]
            if key in (ord('b'),ord('B')):best=0;history=[]
            if time.monotonic()-last_beep>.4 and smoothed>1 and saturation<.8 and not busy:
                frequency=int(350+900*np.clip(ratio,0,1));last_beep=time.monotonic();busy=True
                def beep(freq):
                    nonlocal busy
                    try:winsound.Beep(freq,90)
                    finally:busy=False
                threading.Thread(target=beep,args=(frequency,),daemon=True).start()
    finally:
        if grabbing:camera.MV_CC_StopGrabbing()
        if opened:
            for name,value in saved.items():camera.MV_CC_SetEnumValue(name,value)
            camera.MV_CC_CloseDevice()
        if created:camera.MV_CC_DestroyHandle()
        MvCamera.MV_CC_Finalize();cv2.destroyAllWindows()
        if set_thread_dpi and old_dpi:set_thread_dpi(old_dpi)


if __name__=='__main__':
    main()
