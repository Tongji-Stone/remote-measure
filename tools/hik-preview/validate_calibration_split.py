"""Hold-out evaluation for the 2026-11-16 projector-camera data.

Square size is nominal 10 mm. Hold-out board poses are estimated from camera
corners; projector residuals then test the fixed camera-projector geometry.
"""
import contextlib
import glob
import json
import os
import types
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / 'data' / 'procam_calibration'
SOURCE = ROOT / 'procam-calibration' / 'calibrate.py'
IDS = ['160951', '161029', '161052', '161113', '161142', '161200',
       '161237', '161309', '161359', '161416', '161432', '161501',
       '161556', '161638', '161719', '161753']
VALIDATION = {'161029', '161237', '161556'}
TEST = {'161113', '161359', '161719'}


def main():
    os.chdir(DATA)
    source = SOURCE.read_text(encoding='utf-8')
    marker = "    print('Initial solution of camera\'s intrinsic parameters')"
    # Match the exact source text, including the Python escape for the quote.
    marker = "    print('Initial solution of camera\\'s intrinsic parameters')"
    if source.count(marker) != 1:
        raise RuntimeError('Calibration source changed; cannot extract correspondences safely')
    source = source.replace(marker, '    return proj_objps_list, cam_corners_list2, proj_corners_list, cam_objps_list, cam_corners_list\n' + marker)
    module = types.ModuleType('calibration_extraction')
    exec(compile(source, str(SOURCE), 'exec'), module.__dict__)
    names = ['capture_20261116_' + i for i in IDS]
    files = [sorted(glob.glob(n + '/graycode_*.png')) for n in names]
    if any(len(f) != 44 for f in files):
        raise RuntimeError('Each capture must contain exactly 44 frames')
    with open('calibration_20261116_split_extraction_log.txt', 'w', encoding='utf-8') as log, contextlib.redirect_stdout(log):
        extracted = module.calibrate(names, files, (720, 1280), (9, 7), 10., 1, 40, 5, None, None)
    if extracted is None:
        raise RuntimeError('Failed to extract correspondences; inspect extraction log')
    pobj, cam_subset, proj_pts, cam_obj, cam_pts = extracted
    train = [i for i, key in enumerate(IDS) if key not in VALIDATION | TEST]
    choose = lambda xs: [xs[i] for i in train]
    image = cv2.imread(files[0][-2], cv2.IMREAD_GRAYSCALE)
    height, width = image.shape
    rc, cm, cd, _, _ = cv2.calibrateCamera(choose(cam_obj), choose(cam_pts), (width, height), None, None)
    rp, pm, pd, _, _ = cv2.calibrateCamera(choose(pobj), choose(proj_pts), (1280, 720), None, None)
    rs, cm, cd, pm, pd, R, T, _, _ = cv2.stereoCalibrate(
        choose(pobj), choose(cam_subset), choose(proj_pts), cm, cd, pm, pd, (width, height), flags=cv2.CALIB_FIX_INTRINSIC)
    fs = cv2.FileStorage('calibration_20261116_split_train_experimental.xml', cv2.FILE_STORAGE_WRITE)
    for key, value in [('cam_int', cm), ('cam_dist', cd), ('proj_int', pm), ('proj_dist', pd), ('rotation', R), ('translation', T)]:
        fs.write(key, value)
    fs.write('rms', rs)
    fs.release()
    rows = []
    for i, key in enumerate(IDS):
        ok, rv, tv = cv2.solvePnP(cam_obj[i], cam_pts[i], cm, cd, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            raise RuntimeError('Cannot estimate board pose for ' + key)
        reproject_cam, _ = cv2.projectPoints(cam_obj[i], rv, tv, cm, cd)
        boardR = cv2.Rodrigues(rv)[0]
        projR = R @ boardR
        projT = R @ tv + T
        reproject_proj, _ = cv2.projectPoints(pobj[i], cv2.Rodrigues(projR)[0], projT, pm, pd)
        cr = np.linalg.norm(reproject_cam.reshape(-1, 2) - cam_pts[i].reshape(-1, 2), axis=1)
        pr = np.linalg.norm(reproject_proj.reshape(-1, 2) - proj_pts[i].reshape(-1, 2), axis=1)
        rows.append({'capture': key, 'split': 'validation' if key in VALIDATION else 'test' if key in TEST else 'train',
                     'camera_points': len(cr), 'projector_points': len(pr),
                     'camera_sse': float(np.sum(cr**2)), 'projector_sse': float(np.sum(pr**2)),
                     'camera_rms': float(np.sqrt(np.mean(cr**2))), 'projector_rms': float(np.sqrt(np.mean(pr**2))),
                     'projector_median': float(np.median(pr))})
    result = {'square_size_mm_assumed': 10, 'projection_resolution': [1280, 720],
              'train_camera_rms': rc, 'train_projector_rms': rp, 'train_stereo_rms': rs,
              'camera_matrix': cm.tolist(), 'projector_matrix': pm.tolist(),
              'rotation': R.tolist(), 'translation_mm': T.reshape(-1).tolist(), 'views': rows}
    Path('calibration_20261116_split_metrics.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print('training rms camera/projector/stereo:', rc, rp, rs)
    for subset in ('train', 'validation', 'test'):
        r = [x for x in rows if x['split'] == subset]
        print(subset, 'views:', len(r), 'camera:', (sum(x['camera_sse'] for x in r)/sum(x['camera_points'] for x in r))**.5,
              'projector:', (sum(x['projector_sse'] for x in r)/sum(x['projector_points'] for x in r))**.5)
        for x in r:
            print(x['capture'], 'corners', x['projector_points'], 'camera', round(x['camera_rms'],2), 'projector', round(x['projector_rms'],2))


if __name__ == '__main__':
    main()
