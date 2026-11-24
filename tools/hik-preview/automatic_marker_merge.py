"""Offline reconstruction using common ArUco markers for per-view registration."""
import argparse
import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

SCRIPTS=Path(__file__).resolve().parent
ROOT=SCRIPTS.parents[1]


def calibration_path(path):
    path=path.resolve(strict=True)
    if path.is_file():return path
    if not path.is_dir():raise ValueError('标定路径应是 XML 文件或标定结果文件夹')
    candidates=sorted(path.glob('*.xml'))
    if len(candidates)!=1:raise ValueError(f'标定文件夹中找到 {len(candidates)} 个 XML；请明确选择标定 XML 文件')
    return candidates[0]


def read_calibration(path):
    fs=cv2.FileStorage(path.read_text(encoding='utf8'),cv2.FILE_STORAGE_READ|cv2.FILE_STORAGE_MEMORY)
    values={k:fs.getNode(k).mat() for k in ('cam_int','cam_dist','proj_int','proj_dist','rotation','translation')};fs.release()
    if any(v is None for v in values.values()):raise ValueError('标定 XML 缺少相机或投影仪参数')
    return values


def marker_poses(image_path,K,dist,size):
    im=cv2.imdecode(np.fromfile(str(image_path),np.uint8),cv2.IMREAD_GRAYSCALE)
    if im is None:raise ValueError(f'无法读取白图：{image_path}')
    detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    corners,ids,_=detector.detectMarkers(im)
    if ids is None:return {}
    half=size/2
    square=np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]],np.float32)
    output={}
    for pixels,marker_id in zip(corners,ids.ravel()):
        marker_id=int(marker_id)
        if marker_id in output:raise ValueError(f'标记 ID {marker_id} 重复，无法唯一识别')
        solutions=[]
        for algorithm in (cv2.SOLVEPNP_IPPE_SQUARE,cv2.SOLVEPNP_ITERATIVE):
            try:ok,rv,tv=cv2.solvePnP(square,pixels.reshape(4,2),K,dist,flags=algorithm)
            except cv2.error:continue
            if not ok or tv[2,0]<=0:continue
            reproj=cv2.projectPoints(square,rv,tv,K,dist)[0].reshape(-1,2)
            error=float(np.sqrt(np.mean(np.sum((reproj-pixels.reshape(-1,2))**2,axis=1))))
            if np.isfinite(error):solutions.append((error,rv,tv))
        if not solutions:continue
        error,rv,tv=min(solutions,key=lambda item:item[0])
        if error>2.0:
            print(f'警告：标记 {marker_id} 重投影误差 {error:.2f} 像素，已排除。',flush=True)
            continue
        T=np.eye(4);T[:3,:3]=cv2.Rodrigues(rv)[0];T[:3,3]=tv.ravel()
        output[marker_id]=(T,error,pixels.reshape(4,2))
    return output


def read_xyz(path):
    with path.open('rb') as f:
        header=b''
        while not header.endswith(b'end_header\n'):
            line=f.readline()
            if not line:raise ValueError(f'无效 PLY: {path}')
            header+=line
        if b'property float x' not in header or b'property uchar blue' not in header:raise ValueError('不支持该 PLY 格式')
        return np.fromfile(f,dtype=[('xyz','<f4',3),('rgb','u1',3)])['xyz'].astype(float)


def save_ply(path,parts):
    colors=np.array([[44,115,205],[232,133,45],[45,160,90],[195,66,60],[147,90,184],[22,162,170],[190,158,45],[213,100,162]],np.uint8)
    counts=[len(x) for x in parts];p=np.concatenate(parts);c=np.concatenate([np.repeat(colors[i%8][None,:],n,axis=0) for i,n in enumerate(counts)])
    rec=np.empty(len(p),dtype=[('xyz','<f4',3),('rgb','u1',3)]);rec['xyz']=p;rec['rgb']=c
    with path.open('wb') as f:
        f.write((f'ply\nformat binary_little_endian 1.0\ncomment Experimental ArUco registration\nelement vertex {len(p)}\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n').encode('ascii'));rec.tofile(f)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('calibration',type=Path);ap.add_argument('scan',type=Path);ap.add_argument('--marker-size',type=float,default=20);ap.add_argument('--square-size',type=float,default=10);ap.add_argument('--columns',type=int,default=9);ap.add_argument('--rows',type=int,default=7);ap.add_argument('--bits',type=int,choices=(0,1,2),default=1);ap.add_argument('--step',type=int,default=4);args=ap.parse_args()
    if args.marker_size<=0 or args.step<1:raise ValueError('标记边长与采样间隔须大于 0')
    scan=args.scan.resolve(strict=True)
    requested=args.calibration.resolve(strict=True)
    out=scan/('aruco_reconstruction_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'));out.mkdir()
    if requested.is_dir() and not list(requested.glob('*.xml')):
        cal=out/'automatic_calibration.xml'
        subprocess.run([sys.executable,str(SCRIPTS/'automatic_calibration.py'),str(requested),'--output',str(cal),'--square-size',str(args.square_size),'--columns',str(args.columns),'--rows',str(args.rows)],cwd=ROOT,check=True)
    else:cal=calibration_path(requested)
    m=read_calibration(cal)
    session=json.loads((scan/'session.json').read_text(encoding='utf8'));folders=[]
    if session.get('projector_pixels') not in (None,[1280,720]):
        raise ValueError('当前重建只支持 1280×720 投影图案；该扫描记录不匹配')
    for v in session['views']:
        angle=v.get('nominal_angle_degrees');folder=scan/v['folder']
        if angle is not None and abs(angle)>=360:continue
        if not folder.is_dir():raise ValueError(f'视角目录缺失：{folder}')
        if len(list(folder.glob('graycode_*.png')))!=44:raise ValueError(f'视角图片不完整：{folder}')
        folders.append(folder)
    if len(folders)<2:raise ValueError('至少需要两个非回环视角')
    quality_warnings=[]
    marker_sets=[marker_poses(v/'graycode_42.png',m['cam_int'],m['cam_dist'],args.marker_size) for v in folders]
    reference=marker_sets[0];transforms=[np.eye(4)];details=[{'view':folders[0].name,'marker_ids':sorted(reference)}]
    if len(reference)<2:raise ValueError('首视角识别到的不同标记不足 2 个，请检查图像、标记字典和遮挡')
    print('首视角标记:',sorted(reference),flush=True)
    for folder,markers in zip(folders[1:],marker_sets[1:]):
        shared=sorted(set(reference)&set(markers))
        if len(shared)<2:raise ValueError(f'{folder.name} 与首视角共同标记不足 2 个（{shared}）；不自动猜测位姿')
        candidates=[reference[k][0]@np.linalg.inv(markers[k][0]) for k in shared]
        trans=np.median([a[:3,3] for a in candidates],axis=0);rots=Rotation.from_matrix(np.stack([a[:3,:3] for a in candidates]));mean=rots.mean();deviation=np.degrees((mean.inv()*rots).magnitude());td=np.linalg.norm(np.array([a[:3,3] for a in candidates])-trans,axis=1)
        if max(deviation)>10 or max(td)>20:raise ValueError(f'{folder.name} 的共同标记位姿不一致：角度偏差最大 {max(deviation):.1f}°，平移偏差最大 {max(td):.1f} mm；请检查标定、尺寸和标记')
        T=np.eye(4);T[:3,:3]=mean.as_matrix();T[:3,3]=trans;transforms.append(T)
        if max(deviation)>1 or max(td)>2:
            quality_warnings.append(f'{folder.name} 多标记位姿分歧 {max(deviation):.2f} 度 / {max(td):.2f} mm')
            print(f'警告：{folder.name} 多标记估计存在明显分歧，结果不能视为精密配准。',flush=True)
        details.append({'view':folder.name,'marker_ids':sorted(markers),'shared_ids':shared,'reprojection_rms_px':{str(k):markers[k][1] for k in shared},'candidate_rotation_spread_deg':float(max(deviation)),'candidate_translation_spread_mm':float(max(td)),'transform_to_reference':T.tolist()})
        print(folder.name,'共同标记',shared,'位姿差',round(max(deviation),2),'deg',round(max(td),2),'mm',flush=True)
    parts=[]
    for index,(folder,T) in enumerate(zip(folders,transforms)):
        target=out/(folder.name+'.ply')
        subprocess.run([sys.executable,str(SCRIPTS/'reconstruct_object.py'),str(folder),str(cal),'--step',str(args.step),'--ignore-finest-bits',str(args.bits),'--output',str(target),'--pixel-map',str(out/(folder.name+'_pixels.npz'))],cwd=ROOT,check=True)
        xyz=read_xyz(target);data=np.load(out/(folder.name+'_pixels.npz'));uv=data['camera_pixels'];points=data['xyz'];errors=[]
        white=cv2.imdecode(np.fromfile(str(folder/'graycode_42.png'),np.uint8),cv2.IMREAD_GRAYSCALE)
        for key,(marker_pose,error,corners) in marker_sets[index].items():
            mask=np.zeros(white.shape,np.uint8);cv2.fillConvexPoly(mask,np.rint(corners).astype(np.int32),255)
            good=(uv[:,0]>=0)&(uv[:,0]<mask.shape[1])&(uv[:,1]>=0)&(uv[:,1]<mask.shape[0]);chosen=np.zeros(len(uv),bool);chosen[good]=mask[uv[good,1],uv[good,0]]>0
            if chosen.sum()>=5:
                delta=np.abs((points[chosen]-marker_pose[:3,3])@marker_pose[:3,2]);errors.extend(delta.tolist())
        if errors:
            residual=float(np.median(errors));details[index]['marker_plane_depth_median_mm']=residual
            if residual>3:
                quality_warnings.append(f'{folder.name} 标记平面差 {residual:.1f} mm')
                print(f'警告：{folder.name} 结构光点与标记平面中位差 {residual:.1f} mm，标定、标记尺寸或同步可能有问题。',flush=True)
        else:
            quality_warnings.append(f'{folder.name} 缺少标记平面验证点')
            print(f'警告：{folder.name} 标记区域缺少可检查的结构光点。',flush=True)
        parts.append(xyz@T[:3,:3].T+T[:3,3])
    merged=out/'merged_aruco_experimental.ply';save_ply(merged,parts)
    record={'calibration':str(cal),'scan':str(scan),'marker_dictionary':'DICT_4X4_50','marker_black_edge_mm':args.marker_size,'bits_ignored':args.bits,'pixel_step':args.step,'point_counts':[len(a) for a in parts],'views':details,'quality_status':'warning' if quality_warnings else 'unverified','quality_warnings':quality_warnings,'warnings':['experiment only; calibration and marker size must be verified','point cloud retains turntable and background; static background may ghost','no independent 360-degree validation']}
    (out/'registration.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf8')
    print('拼接结果:',merged,flush=True)
    if quality_warnings:print('警告：检测到质量异常，结果仅供排查，不应用于测量。',flush=True)
    print('注意：仅试验性配准，保留转台与背景；标记尺寸、标定及位姿误差仍需验证。',flush=True)

if __name__=='__main__':main()
