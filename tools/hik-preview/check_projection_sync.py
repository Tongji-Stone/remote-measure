"""Compare projected black/white alternation at 100 ms exposure.

This measures observed contrast on the visible scene, not exact HDMI optical
latency. No scan data is changed; only a compact JSON report is saved.
"""
import argparse
import json
import time
from datetime import datetime
from pathlib import Path
from calibration_capture import *


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--exposure-us',type=float,default=100000)
    ap.add_argument('--pairs',type=int,default=6)
    args=ap.parse_args()
    if args.exposure_us<=0 or args.pairs<1:raise ValueError('参数必须大于零')
    screens=monitors()
    if len(screens)!=2:raise RuntimeError('请将投影仪设为扩展屏')
    x,y,w,h=screens[1]
    black=np.zeros((h,w),np.uint8);white=np.full((h,w),255,np.uint8)
    camera=MvCamera();created=opened=grabbing=False;saved={}
    check(MvCamera.MV_CC_Initialize(),'SDK initialization')
    try:
        devices=MV_CC_DEVICE_INFO_LIST();check(MvCamera.MV_CC_EnumDevices(MV_USB_DEVICE,devices),'Enumerate USB camera')
        if devices.nDeviceNum!=1:raise RuntimeError(f'需要一台海康 USB 相机，当前 {devices.nDeviceNum} 台')
        check(camera.MV_CC_CreateHandle(cast(devices.pDeviceInfo[0],POINTER(MV_CC_DEVICE_INFO)).contents),'Create handle');created=True
        check(camera.MV_CC_OpenDevice(MV_ACCESS_Exclusive,0),'打开相机，请先关闭其他预览或采集');opened=True
        for name in ('TriggerMode','ExposureAuto','GainAuto'):
            state=MVCC_ENUMVALUE();check(camera.MV_CC_GetEnumValue(name,state),'Read '+name);saved[name]=state.nCurValue;check(camera.MV_CC_SetEnumValue(name,0),'Set '+name)
        val=MVCC_FLOATVALUE();check(camera.MV_CC_GetFloatValue('ExposureTime',val),'Read exposure');saved['ExposureTime']=val.fCurValue
        check(camera.MV_CC_SetFloatValue('ExposureTime',args.exposure_us),'Set test exposure')
        cv2.namedWindow('Projector',cv2.WINDOW_NORMAL);cv2.moveWindow('Projector',x,y);cv2.setWindowProperty('Projector',cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
        cv2.imshow('Projector',white);cv2.waitKey(500);check(camera.MV_CC_StartGrabbing(),'Start grabbing');grabbing=True
        print('建立白光与黑光参考，期间保持相机、被照物体不动。',flush=True)
        white_ref,_=capture_projected_pattern(camera,white,args.exposure_us,settle_seconds=.5)
        black_ref,_=capture_projected_pattern(camera,black,args.exposure_us,settle_seconds=.5)
        difference=white_ref.astype(np.float32)-black_ref.astype(np.float32)
        mask=(difference>=20)&(white_ref<245)
        if int(mask.sum())<1000:
            mask=(difference>=20)
        if int(mask.sum())<1000:raise RuntimeError('场景中可用于比较的投影亮暗区域太少，请让投影照在固定的浅色漫反射表面上')
        base=black_ref.astype(np.float32)[mask];dynamic=difference[mask];results={}
        for method in ('旧时序','增加等待'):
            rows=[]
            for i in range(args.pairs*2):
                bright=(i%2==0);pattern=white if bright else black
                if method=='旧时序':
                    start=time.perf_counter();cv2.imshow('Projector',pattern);cv2.waitKey(10);check(camera.MV_CC_ClearImageBuffer(),'Clear');get_frame(camera);img=get_frame(camera);elapsed=time.perf_counter()-start
                else:
                    start=time.perf_counter();img,_=capture_projected_pattern(camera,pattern,args.exposure_us);elapsed=time.perf_counter()-start
                ratio=np.clip((img.astype(np.float32)[mask]-base)/dynamic,0,1)
                median=float(np.median(ratio));fraction=float(np.mean(ratio>.5))
                error=float(1-fraction if bright else fraction)
                rows.append({'pattern':'white' if bright else 'black','median_normalized':median,'wrong_side_fraction':error,'elapsed_seconds':elapsed})
                print(f'{method} {i+1}/{args.pairs*2}: {"白" if bright else "黑"}, 亮度比例 {median:.3f}, 反向像素 {error:.1%}, {elapsed:.3f} s',flush=True)
            results[method]=rows
        result={'description':'黑白交替可观测对比；不直接测量投影光学延迟','exposure_us':args.exposure_us,'projector_pixels':[w,h],'comparison_pixels':int(mask.sum()),'results':results}
        root=Path(__file__).resolve().parents[2]/'data'/'sync_tests';root.mkdir(parents=True,exist_ok=True)
        path=root/('sync_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.json');path.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
        print('测试记录:',path,flush=True)
    finally:
        if grabbing:camera.MV_CC_StopGrabbing()
        if opened:
            if 'ExposureTime' in saved:camera.MV_CC_SetFloatValue('ExposureTime',saved['ExposureTime'])
            for name in ('ExposureAuto','GainAuto','TriggerMode'):
                if name in saved:camera.MV_CC_SetEnumValue(name,saved[name])
            camera.MV_CC_CloseDevice()
        if created:camera.MV_CC_DestroyHandle()
        MvCamera.MV_CC_Finalize();cv2.destroyAllWindows()

if __name__=='__main__':main()
