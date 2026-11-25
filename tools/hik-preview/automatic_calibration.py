"""用棋盘格或 ChArUco 板标定相机、投影仪及两者的相对位姿。"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from calibration_target import CalibrationTarget, add_target_arguments
from reconstruct_object import image as read


def projector_corners(white, black, patterns, gray, world, pixels, ids, size):
    """在每个角点周围解格雷码，再以局部单应把相机角点映射到亚像素投影坐标。"""
    height, width = white.shape
    radius = int(np.ceil(width / 180))
    indices, projector = [], []
    for i, corner in enumerate(pixels[:, 0]):
        cx, cy = np.rint(corner).astype(int)
        src, dst = [], []
        for y in range(max(0, cy - radius), min(height, cy + radius + 1)):
            for x in range(max(0, cx - radius), min(width, cx + radius + 1)):
                if int(white[y, x]) - int(black[y, x]) <= 40:
                    continue
                error, uv = gray.getProjPixel(patterns, x, y)
                if not error:
                    src.append((x, y))
                    dst.append(uv)
        if len(src) < radius ** 2:
            continue
        H, inliers = cv2.findHomography(np.float32(src), np.float32(dst), cv2.RANSAC, 2.0)
        if H is None or inliers is None or int(inliers.sum()) < radius ** 2 // 2:
            continue
        v = H @ np.r_[corner, 1.0]
        if abs(v[2]) < 1e-9:
            continue
        uv = v[:2] / v[2]
        if np.all(uv >= 0) and np.all(uv < size):
            indices.append(i)
            projector.append(uv)
    return dict(shared_world=world[indices], shared_camera=pixels[indices],
                projector=np.float32(projector).reshape(-1, 1, 2), shared_ids=ids[indices])


def fit(views, camera_size, projector_size):
    """独立求两套内参，再固定内参联合求 X_proj = R @ X_cam + T。"""
    world = [v['world'] for v in views]
    camera = [v['camera'] for v in views]
    shared = [v for v in views if len(v['projector']) >= 6]
    if len(views) < 3 or len(shared) < 3:
        raise ValueError('至少需要 3 组相机观测，并有 3 组有效投影角点')
    obj = [v['shared_world'] for v in shared]
    cam = [v['shared_camera'] for v in shared]
    proj = [v['projector'] for v in shared]
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-9)
    flags = cv2.CALIB_USE_LU
    rc, K, d, _, _ = cv2.calibrateCamera(world, camera, camera_size, None, None, flags=flags, criteria=criteria)
    rp, P, pd, _, _ = cv2.calibrateCamera(obj, proj, projector_size, None, None, flags=flags, criteria=criteria)
    rs, K, d, P, pd, R, T, _, _ = cv2.stereoCalibrate(
        obj, cam, proj, K, d, P, pd, camera_size, flags=cv2.CALIB_FIX_INTRINSIC | flags, criteria=criteria)
    model = dict(cam_int=K, cam_dist=d, proj_int=P, proj_dist=pd, rotation=R, translation=T)
    if not all(np.isfinite(a).all() for a in [rc, rp, rs, *model.values()]):
        raise ValueError('标定结果含非有限值')
    return model, dict(camera_rms_px=rc, projector_rms_px=rp, joint_rms_px=rs)


def pose_residuals(view, model):
    """只用相机角点估板位姿，然后预测相机和投影仪观测。"""
    K, d = model['cam_int'], model['cam_dist']
    ok, rvec, tvec = cv2.solvePnP(view['world'], view['camera'], K, d)
    if not ok:
        raise ValueError('无法计算标靶位姿')
    R = cv2.Rodrigues(rvec)[0]
    camera = cv2.projectPoints(view['world'], rvec, tvec, K, d)[0]
    camera_error = np.linalg.norm((camera - view['camera']).reshape(-1, 2), axis=1)
    projector_error = np.empty(0)
    if len(view['projector']):
        Rp = model['rotation'] @ R
        tp = model['rotation'] @ tvec + model['translation']
        projector = cv2.projectPoints(view['shared_world'], cv2.Rodrigues(Rp)[0], tp,
                                      model['proj_int'], model['proj_dist'])[0]
        projector_error = np.linalg.norm((projector - view['projector']).reshape(-1, 2), axis=1)
    return R, tvec, camera_error, projector_error


def rms(errors):
    return float(np.sqrt(np.mean(errors ** 2))) if len(errors) else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    add_target_arguments(parser)
    args = parser.parse_args()
    cv2.setNumThreads(4)
    target = CalibrationTarget(args)
    dirs = sorted(p for p in args.folder.iterdir() if p.is_dir() and p.name.startswith('capture_'))
    if len(dirs) < 3:
        raise ValueError('标定文件夹中至少需要 3 组 capture_... 子文件夹')
    size = (args.width, args.height)
    session_path = args.folder / 'session.json'
    if session_path.is_file():
        session = json.loads(session_path.read_text(encoding='utf-8'))
        if session.get('projector_pixels', list(size)) != list(size):
            raise ValueError('投影尺寸和采集记录不一致')
    gray = cv2.structured_light_GrayCodePattern.create(*size)
    gray.setBlackThreshold(40)
    gray.setWhiteThreshold(5)
    n = gray.getNumberOfPatternImages()
    views, details = [], []
    shape = None
    for folder in dirs:
        files = [folder / f'graycode_{i:02d}.png' for i in range(n + 2)]
        detail = dict(folder=folder.name, camera_corners=0, projector_corners=0)
        details.append(detail)
        if not all(p.is_file() for p in files):
            detail['status'] = 'missing_frames'
            print(f'{folder.name}: 格雷码序列缺帧', flush=True)
            continue
        white = read(files[-2])
        if shape is None:
            shape = white.shape
        if white.shape != shape:
            raise ValueError(f'{folder.name}: 相机图像尺寸不一致')
        world, pixels, ids = target.observe(white)
        detail['camera_corners'] = len(ids)
        if len(ids) < 6 or np.linalg.matrix_rank(world[:, :2] - world[0, :2]) < 2:
            detail['status'] = 'insufficient_camera_corners'
            print(f'{folder.name}: 标靶角点不足', flush=True)
            continue
        black = read(files[-1])
        patterns = [read(p) for p in files[:-2]]
        if any(frame.shape != shape for frame in [black, *patterns]):
            raise ValueError(f'{folder.name}: 图像尺寸不一致')
        shared = projector_corners(white, black, patterns, gray, world, pixels, ids, size)
        if len(shared['projector']) >= 6:
            xy = shared['shared_world'][:, :2]
            if np.linalg.matrix_rank(xy - xy[0]) < 2:
                shared = {k: v[:0] for k, v in shared.items()}
        count = len(shared['projector'])
        detail.update(projector_corners=count, status='used' if count >= 6 else 'camera_only')
        views.append(dict(world=world, camera=pixels, ids=ids, detail=detail, **shared))
        print(f'{folder.name}: {len(ids)} 个相机角点、{count} 个投影角点', flush=True)

    if shape is None:
        raise ValueError('找不到完整的标定图像序列')
    h, w = shape
    model, stats = fit(views, (w, h), size)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    observations = args.output.with_name(args.output.stem + '_observations')
    observations.mkdir(exist_ok=True)
    for view in views:
        R, t, ec, ep = pose_residuals(view, model)
        detail = view['detail']
        detail.update(camera_pose_rms_px=rms(ec), projector_transfer_rms_px=rms(ep))
        path = observations / (detail['folder'] + '.npz')
        np.savez_compressed(path, **{k: v for k, v in view.items() if k != 'detail'},
                            board_to_camera_R=R, board_to_camera_t=t,
                            camera_residual_px=ec, projector_transfer_residual_px=ep)
    fs = cv2.FileStorage('', cv2.FILE_STORAGE_WRITE | cv2.FILE_STORAGE_MEMORY)
    fs.write('img_shape', np.array([h, w], float))
    fs.write('proj_shape', np.array([args.height, args.width], float))
    for key, value in model.items():
        fs.write(key, value)
    fs.write('rms', stats['joint_rms_px'])
    args.output.write_text(fs.releaseAndGetString(), encoding='utf-8')
    metadata = dict(**target.spec, **stats, camera_size_px=[w, h], projector_size_px=list(size),
                    baseline_mm=float(np.linalg.norm(model['translation'])), source=str(args.folder.resolve()),
                    poses=details, parameters={k: a.tolist() for k, a in model.items()})
    args.output.with_suffix('.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
    print('标定完成', stats, args.output, flush=True)


if __name__ == '__main__':
    main()
