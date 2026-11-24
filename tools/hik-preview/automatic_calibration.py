"""Calibrate a camera/projector from a folder of Gray-code board poses."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np


def read(path):
    image=cv2.imdecode(np.fromfile(str(path),np.uint8),cv2.IMREAD_GRAYSCALE)
    if image is None:raise ValueError(f'无法读取图像: {path}')
    return image


def main():
    ap=argparse.ArgumentParser();ap.add_argument('folder',type=Path);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--square-size',type=float,default=10);ap.add_argument('--columns',type=int,default=9);ap.add_argument('--rows',type=int,default=7);ap.add_argument('--width',type=int,default=1280);ap.add_argument('--height',type=int,default=720);args=ap.parse_args()
    if args.square_size<=0 or min(args.columns,args.rows)<3:raise ValueError('棋盘格尺寸或内角点数量不合理')
    dirs=sorted(p for p in args.folder.iterdir() if p.is_dir() and p.name.startswith('capture_'))
    if len(dirs)<3:raise ValueError('标定文件夹中至少需要 3 组 capture_... 子文件夹')
    gray=cv2.structured_light_GrayCodePattern.create(args.width,args.height);gray.setBlackThreshold(40);gray.setWhiteThreshold(5)
    n=gray.getNumberOfPatternImages();obj=np.zeros((args.rows*args.columns,3),np.float32);obj[:,:2]=np.mgrid[0:args.columns,0:args.rows].T.reshape(-1,2)*args.square_size
    camobj=[];camp=[];sharedobj=[];sharedcam=[];projp=[];camshape=None;used=[]
    for folder in dirs:
        files=sorted(folder.glob('graycode_*.png'))
        if len(files)!=n+2:raise ValueError(f'{folder.name}: 需要 {n+2} 张图，找到 {len(files)} 张')
        imgs=[read(f) for f in files];shape=imgs[-2].shape
        if camshape is not None and shape!=camshape:raise ValueError('相机图像尺寸不一致')
        if any(x.shape!=shape for x in imgs):raise ValueError(f'{folder.name} 图像尺寸不一致')
        camshape=shape;white,black=imgs[-2:];found,corners=cv2.findChessboardCorners(white,(args.columns,args.rows))
        if not found:found,corners=cv2.findChessboardCornersSB(white,(args.columns,args.rows),flags=cv2.CALIB_CB_EXHAUSTIVE|cv2.CALIB_CB_ACCURACY)
        if not found:raise ValueError(f'{folder.name}: 无法检测完整棋盘格；请检查图片或角点数量')
        corners=cv2.cornerSubPix(white,corners,(5,5),(-1,-1),(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,50,.001))
        cp=[];pp=[];op=[];radius=int(np.ceil(shape[1]/180))
        for corner,world in zip(corners,obj):
            cx,cy=np.rint(corner[0]).astype(int);src=[];dst=[]
            for y in range(max(0,cy-radius),min(shape[0],cy+radius+1)):
                for x in range(max(0,cx-radius),min(shape[1],cx+radius+1)):
                    if int(white[y,x])-int(black[y,x])<=40:continue
                    err,proj=gray.getProjPixel(imgs[:n],x,y)
                    if not err:src.append((x,y));dst.append(proj)
            if len(src)<radius**2:continue
            H,inliers=cv2.findHomography(np.array(src,dtype=np.float32),np.array(dst,dtype=np.float32),cv2.RANSAC,2.0)
            if H is None or inliers is None or int(inliers.sum())<radius**2//2:continue
            v=H@np.r_[corner[0],1.0]
            if abs(v[2])<1e-9:continue
            q=v[:2]/v[2]
            if not (0<=q[0]<args.width and 0<=q[1]<args.height):continue
            cp.append(corner);pp.append([q]);op.append(world)
        if len(cp)<6:raise ValueError(f'{folder.name}: 有效投影角点只有 {len(cp)} 个，无法可靠标定')
        used.append({'folder':folder.name,'camera_corners':len(corners),'projector_corners':len(cp)})
        print(f'{folder.name}: {len(corners)} 个相机角点、{len(cp)} 个投影角点',flush=True)
        camobj.append(obj.copy());camp.append(np.float32(corners));sharedobj.append(np.float32(op));sharedcam.append(np.float32(cp));projp.append(np.float32(pp))
    h,w=camshape
    rc,K,d,_,_=cv2.calibrateCamera(camobj,camp,(w,h),None,None)
    rp,P,pd,_,_=cv2.calibrateCamera(sharedobj,projp,(args.width,args.height),None,None)
    rs,K,d,P,pd,R,T,_,_=cv2.stereoCalibrate(sharedobj,sharedcam,projp,K,d,P,pd,(w,h),flags=cv2.CALIB_FIX_INTRINSIC)
    if not np.isfinite(rs):raise ValueError('标定结果无效')
    fs=cv2.FileStorage('',cv2.FILE_STORAGE_WRITE|cv2.FILE_STORAGE_MEMORY)
    for key,value in [('img_shape',np.array([h,w],float)),('cam_int',K),('cam_dist',d),('proj_int',P),('proj_dist',pd),('rotation',R),('translation',T)]:fs.write(key,value)
    fs.write('rms',rs);args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(fs.releaseAndGetString(),encoding='utf8')
    meta={'camera_rms_px':rc,'projector_rms_px':rp,'joint_rms_px':rs,'square_size_mm':args.square_size,'poses':used,'warning':'训练误差不是实测精度，请用留出板和已知尺寸物体验证'}
    args.output.with_suffix('.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2),encoding='utf8')
    print(f'标定完成：相机 RMS {rc:.3f}、投影 RMS {rp:.3f}、联合 RMS {rs:.3f} 像素',flush=True)
    if rs>2: print('警告：联合误差较大，请检查图案同步和标定数据。',flush=True)
    print('标定参数:',args.output,flush=True)

if __name__=='__main__':main()
