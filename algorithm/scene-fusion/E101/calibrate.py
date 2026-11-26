"""Calibrate both camera–projector models and detect scan board fiducials."""
from concurrent.futures import ThreadPoolExecutor
import json

import cv2 as cv
import numpy as np
from scipy.spatial import cKDTree

from acquire import BRANCHES, CALIBRATIONS, HERE, SCANS, directory, load_frame


def dump(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, allow_nan=False, indent=2,
                               default=lambda a: a.tolist()) + '\n')


def detectors():
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_50)
    board = cv.aruco.CharucoBoard((10, 10), 10., 7., dictionary)
    params = cv.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX
    params.adaptiveThreshWinSizeMax = 43
    return board, cv.aruco.CharucoDetector(board, detectorParams=params)


def board_observations(folder, mode):
    white = load_frame(folder, 42)
    small = cv.resize(white, None, fx=.5, fy=.5, interpolation=cv.INTER_AREA)
    board, detector = detectors()
    if mode == 'Chessboard':
        ok, uv = cv.findChessboardCornersSB(small, (9, 7), cv.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            ok, uv = cv.findChessboardCornersSB(white, (9, 7), cv.CALIB_CB_EXHAUSTIVE)
            if not ok:
                return None
            uv = uv / 2 - .25
        xyz = np.c_[np.mgrid[0:9, 0:7].T.reshape(-1, 2) * 10, np.zeros(63)]
    else:
        uv, ids, marks, mark_ids = detector.detectBoard(small)
        if mode == 'scan':
            if mark_ids is None:
                raise ValueError(f'No ChArUco markers in {folder}')
            marks = np.concatenate(marks).reshape(-1, 1, 2)
            p = cv.cornerSubPix(white, marks * 2 + .5, (4, 4), (-1, -1), (3, 30, .01))
            return (mark_ids.ravel(), p.reshape(-1, 4, 2),
                    np.array(board.getObjPoints())[mark_ids.ravel()])
        if ids is None or len(ids) < 10:
            return None
        xyz = board.getChessboardCorners()[ids.ravel()]
    uv = cv.cornerSubPix(white, uv * 2 + .5, (5, 5), (-1, -1), (3, 35, .015))
    return np.asarray(xyz, np.float32), uv.reshape(-1, 2)


def calibration_observation(folder, mode):
    target = board_observations(folder, mode)
    if target is None:
        return None
    xyz, uv = target
    archive = np.load(directory(folder) / 'gray.npz')
    xy = archive['uv'].astype(float)
    codes, good = archive['codes'], archive['good']
    tree = cKDTree(xy)
    # Local camera→projector homography on the planar board, with progressively coarser codes.
    keep, projector, detail = [], [], []
    for i, group in enumerate(tree.query_ball_point(uv, 23)):
        group = np.asarray(group, int)
        for level in (0, 1, 2):
            use = group[good[level, group]]
            if len(use) < 28:
                continue
            positions = (xy[use] - uv[i]) / 20
            values = codes[level, use].astype(float) / 2
            H, mask = cv.findHomography(positions, values, cv.RANSAC, 1.5 + .55*level)
            if H is None:
                continue
            mask = mask.ravel().astype(bool)
            if mask.sum() < max(24, .65*len(use)):
                continue
            support = positions[mask]
            if np.any(abs(support.mean(0)) > .35) or np.linalg.eigvalsh(np.cov(support.T))[0] < .09:
                continue
            predicted = cv.perspectiveTransform(np.zeros((1, 1, 2)), H)[0, 0]
            if not ((predicted >= 0) & (predicted < [1280, 720])).all():
                continue
            keep.append(i); projector.append(predicted); detail.append(level)
            break
    if len(keep) < 10:
        return None
    return dict(name=folder.name, xyz=xyz[keep], u=uv[keep], p=np.array(projector, np.float32),
                mode=mode, level=detail, n_detect=len(xyz), shape=archive['shape'][::-1])


def optimize(views):
    obj, uc, up = ([v[k] for v in views] for k in ('xyz', 'u', 'p'))
    cbsize = tuple(views[0]['shape'])
    flags = cv.CALIB_FIX_K3
    ce, K, D, _, _ = cv.calibrateCamera(obj, uc, cbsize, None, None, flags=flags)
    pe, L, E, _, _ = cv.calibrateCamera(obj, up, (1280,720), None, None, flags=flags)
    rms, _, _, _, _, R, t, _, _ = cv.stereoCalibrate(obj, uc, up, K, D, L, E, cbsize,
                                                     flags=cv.CALIB_FIX_INTRINSIC,
                                                     criteria=(3, 80, 1e-8))
    return dict(cam_int=K, cam_dist=D, proj_int=L, proj_dist=E, rotation=R, translation=t,
                cam_train_rms=ce, proj_train_rms=pe, stereo_train_rms=rms)


def errors(cal, item):
    K, D, L, E, R, t = (cal[k] for k in ('cam_int','cam_dist','proj_int','proj_dist','rotation','translation'))
    ok, r, offset = cv.solvePnP(item['xyz'], item['u'], K, D)
    if not ok:
        raise ValueError(item['name'])
    cam = cv.projectPoints(item['xyz'], r, offset, K, D)[0][:,0]
    Rp = R @ cv.Rodrigues(r)[0]
    tp = R @ offset + t
    proj = cv.projectPoints(item['xyz'], Rp, tp, L, E)[0][:,0]
    return np.linalg.norm(cam-item['u'], axis=1), np.linalg.norm(proj-item['p'], axis=1)


def scan_target(folder):
    ids, u, xyz = board_observations(folder, 'scan')
    path = directory(folder) / 'target.npz'
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(path, ids=ids, u=u, xyz=xyz)
    print('scan target', folder.parent.name[14:20], folder.name, len(ids), flush=True)


def model_files():
    allobs = {}
    subsets = {}
    for mode in BRANCHES:
        folder = CALIBRATIONS[mode]
        captures = sorted(folder.glob('capture_*'))
        data = []
        with ThreadPoolExecutor(max_workers=4) as pool:
            for idx, item in enumerate(pool.map(lambda p: calibration_observation(p, mode), captures)):
                if item is not None:
                    item['holdout'] = idx % 4 == 1
                    data.append(item)
                    print(mode, item['name'], item['n_detect'], len(item['xyz']), item['holdout'], flush=True)
                else:
                    print(mode, captures[idx].name, 'no usable board', flush=True)
        allobs[mode] = data
        subsets[mode] = [item for item in data if not item['holdout']]
    for mode, training in subsets.items():
        cal = optimize(training)
        # Remove Gross wrong code samples on training captures only.
        for _ in range(2):
            filtered = []
            for item in training:
                ec, ep = errors(cal, item)
                keep = (ec < max(1.1, np.median(ec)*4)) & (ep < max(3.5, np.median(ep)*3.5))
                if keep.sum() >= 10:
                    filtered.append({**item, **{k: item[k][keep] for k in ('xyz', 'u', 'p')}})
            training = filtered
            cal = optimize(training)
        rows = []
        for item in sum(allobs.values(), []):
            ec, ep = errors(cal, item)
            rows.append(dict(name=item['name'], target=item['mode'], holdout=item['holdout'],
                             n=len(ec), camera_rms=np.sqrt((ec**2).mean()), camera_p95=np.percentile(ec,95),
                             projector_p50=np.median(ep), projector_p95=np.percentile(ep,95)))
        cal.update(training=[dict(name=v['name'], corners=len(v['xyz'])) for v in training],
                   validation=rows, code_step_camera_px=3, board_size_mm=10,
                   baseline_mm=np.linalg.norm(cal['translation']))
        print('solved', mode, cal['stereo_train_rms'], cal['baseline_mm'], cal['cam_int'], cal['proj_int'], flush=True)
        dump(HERE / mode / '标定.json', cal)


if __name__ == '__main__':
    cv.ocl.setUseOpenCL(False)
    cv.setNumThreads(1)
    model_files()
    with ThreadPoolExecutor(max_workers=4) as pool:
        list(pool.map(scan_target, (v for s in SCANS for v in sorted(s.glob('view_*')))))
