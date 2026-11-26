"""Reconstruct full scans and solve measured and commanded turntable poses."""
from concurrent.futures import ThreadPoolExecutor
import json

import cv2 as cv
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares

from acquire import BRANCHES, HERE, SCANS, directory, triangulate
from calibrate import dump

CAL_KEYS = ('cam_int', 'cam_dist', 'proj_int', 'proj_dist', 'rotation', 'translation')


def transform(points, matrix):
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def as_pose(R, t):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, np.asarray(t).ravel()
    return T


def axial(axis, center, radians):
    R = Rotation.from_rotvec(np.asarray(radians)[..., None] * axis).as_matrix()
    P = np.broadcast_to(np.eye(4), (*R.shape[:-2], 4, 4)).copy()
    P[..., :3, :3] = R
    P[..., :3, 3] = center - R @ center
    return P


def project(xyz, model):
    return cv.projectPoints(xyz, np.zeros(3), np.zeros(3), model['cam_int'],
                            model['cam_dist'])[0][:, 0]


def read_models():
    models = {}
    for name in BRANCHES:
        data = json.loads((HERE / name / '标定.json').read_text())
        models[name] = {k: np.array(data[k], float) for k in CAL_KEYS}
    return models


def stage_observation(target, model):
    keep = target['ids'] % 4 != 1
    xyz, uv = [target[k][keep].reshape(-1, n) for k, n in [('xyz',3), ('u',2)]]
    _, inliers = cv.findHomography(xyz[:,:2], uv, cv.RANSAC, 4.)
    if inliers is None or inliers.sum() < 8:
        raise ValueError(f'Insufficient planar marker support for {target["ids"]}')
    m = inliers.ravel().astype(bool)
    ok, r, t = cv.solvePnP(xyz[m], uv[m], model['cam_int'], model['cam_dist'], flags=cv.SOLVEPNP_IPPE)
    if not ok:
        raise ValueError(f'Failed planar PnP for {target["ids"]}')
    r, t = cv.solvePnPRefineLM(xyz[m], uv[m], model['cam_int'], model['cam_dist'], r, t)
    held = ~keep
    return dict(pose=as_pose(cv.Rodrigues(r)[0],t), board_xyz=xyz[m], board_uv=uv[m],
                held_xyz=target['xyz'][held].reshape(-1,3), held_uv=target['u'][held].reshape(-1,2),
                n_inliers=int(m.sum()))


def estimate_axis(obs, commanded, model):
    A = np.array([v['pose'] for v in obs])
    relative = A @ np.linalg.inv(A[0])
    # Principal axis from relative rotations, with the sign resolved by the first turn.
    rv = Rotation.from_matrix(relative[:, :3, :3]).as_rotvec()
    _, _, vt = np.linalg.svd(rv)
    axis = vt[0] * np.sign(rv[1] @ vt[0])
    mat = (np.eye(3) - relative[:, :3, :3]).reshape(-1, 3)
    center = np.linalg.lstsq(mat, relative[:, :3, 3].ravel(), rcond=1e-5)[0]
    center += axis * (axis @ (transform(np.array([[50,50,0.]]), A[0])[0] - center))
    angles = np.unwrap(np.arctan2(np.sin(rv @ axis), np.cos(rv @ axis)))
    initial = np.r_[0., 0., 0., 0., angles[1:] - commanded[1:]]
    B = np.linalg.svd(axis[None,:])[2][1:].T

    def unpack(x):
        u = axis + B @ x[:2]
        return u / np.linalg.norm(u), center + B @ x[2:4], commanded + np.r_[0.,x[4:]]

    def residual(x):
        u, c, theta = unpack(x)
        transforms = axial(u, c, theta) @ A[0]
        r = [((project(transform(v['board_xyz'], T), model) - v['board_uv']) / np.sqrt(len(v['board_xyz'])))
             .ravel() for T, v in zip(transforms, obs)]
        return np.r_[np.concatenate(r), x[4:] * .5]

    x = least_squares(residual, initial, loss='soft_l1', f_scale=.22, max_nfev=45).x
    u, c, theta = unpack(x)
    return u, c, theta, A[0] @ np.linalg.inv(A)


def reconstruct(folder, model):
    archive = np.load(directory(folder) / 'gray.npz')
    uv, pixel, codes, mask = [archive[k] for k in ('uv','gray','codes','good')]
    xyz = np.full((len(uv), 3), np.nan)
    err = np.full(len(uv), np.nan)
    level = np.full(len(uv), -1, np.int8)
    for i in range(3):
        idx = np.flatnonzero(mask[i] & (level < 0))
        p, e = triangulate(model, uv[idx].astype(float), codes[i,idx].astype(float) / 2)
        d = e.max(axis=1)
        good = ((p[:, 2] > 50.) & (p[:, 2] < 1600.) &
                (transform(p, as_pose(model['rotation'],model['translation']))[:,2] > 0) &
                np.isfinite(p).all(axis=1) & (d <= (3.5 + i)))
        selected = idx[good]
        xyz[selected], err[selected], level[selected] = p[good], d[good], i
    keep = level >= 0
    return dict(xyz=xyz[keep].astype('float32'), uv=uv[keep], gray=pixel[keep],
                grid_index=np.flatnonzero(keep), code_level=level[keep], error=err[keep].astype('float32'),
                shape=archive['shape'])


def region(xyz, pose):
    local = transform(xyz, np.linalg.inv(pose))
    # Object lies on the front side of the photographed ChArUco board (negative board z).
    return ((local[:,0] >= -14) & (local[:,0] <= 114) & (local[:,1] >= -14) &
            (local[:,1] <= 114) & (local[:,2] < -3) & (local[:,2] > -200))


def write_scene(path, datasets, poses, labels):
    vertex = np.dtype([('xyz','<f4',(3,)),('rgb','u1',(3,)),('view','u1')])
    count = sum(len(data['xyz']) for data in datasets)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix('.part')
    with temp.open('wb') as file:
        head = ('ply\nformat binary_little_endian 1.0\ncomment unit mm nominal target scale\n'
                f'element vertex {count}\nproperty float x\nproperty float y\nproperty float z\n'
                'property uchar red\nproperty uchar green\nproperty uchar blue\nproperty uchar view_id\nend_header\n')
        file.write(head.encode())
        for data, T, idx in zip(datasets, poses, labels):
            rec = np.empty(len(data['xyz']), vertex)
            rec['xyz'] = transform(data['xyz'], T)
            rec['rgb'] = data['gray'][:,None]
            rec['view'] = idx
            rec.tofile(file)
    temp.replace(path)
    return count


def scan_recon(scan, models):
    folders = sorted(scan.glob('view_*'))
    cmd = np.deg2rad([item['nominal_angle_degrees'] for item in json.loads(
        (scan/'session.json').read_text())['views']])
    # The suffix may itself contain underscores.
    name = scan.name.split("_", 4)[4]

    targets = [np.load(directory(v) / 'target.npz') for v in folders]
    data = {key: [reconstruct(v, model) for v in folders] for key, model in models.items()}
    poses = {key: [stage_observation(t, model) for t in targets] for key, model in models.items()}
    masks = {}
    for i in range(len(folders)):
        arrays = [data[key][i] for key in BRANCHES]
        x, a, b = np.intersect1d(arrays[0]['grid_index'],arrays[1]['grid_index'],
                                 assume_unique=True,return_indices=True)
        indices = [a,b]
        # Shared image samples: an evaluation point must be above board in both reconstructions.
        keep = np.ones(len(x),bool)
        for j,k in enumerate(BRANCHES):
            keep &= region(arrays[j]['xyz'][indices[j]],poses[k][i]['pose'])
        masks[i] = {key:indices[j][keep] for j,key in enumerate(BRANCHES)}
    for key, model in models.items():
        desc = poses[key]
        axis, center, phi, pnp = estimate_axis(desc,cmd,model)
        out = HERE / key / name
        cache = directory(scan) / key
        cache.mkdir(parents=True, exist_ok=True)
        audit = []
        for i,(v,d) in enumerate(zip(data[key],desc)):
            sel = masks[i][key]
            np.savez(cache / f'{i:02}.npz', **d, region=sel, **v)
            points = write_scene(out/'单视角'/f'V{i:02}.ply',[v],[np.eye(4)],[i])
            counts = np.bincount(v['code_level'],minlength=3)
            audit.append(dict(view=i, all_points=points, common_region_points=len(sel),
                              coded_bits_lost=counts, train_marker_corners=d['n_inliers'],
                              held_marker_corners=len(d['held_xyz']), epipolar_reproj_p95=np.percentile(v['error'],95)))
        dump(cache/'poses.json', dict(name=name, scan=scan.name, axis=axis, center=center, cmd=cmd, phi=phi, pnp=pnp))
        dump(out/'采集.json', dict(scan=scan.name, nominal_degrees=np.rad2deg(cmd), fitted_degrees=np.rad2deg(phi),
                                  axis=axis, center_mm=center, views=audit, camera_grid_px=3))
        print('prepared',key,name,len(folders),sum(r['all_points'] for r in audit),
              [r['common_region_points'] for r in audit],np.round(np.rad2deg(phi-cmd),2),flush=True)


if __name__ == '__main__':
    cv.ocl.setUseOpenCL(False)
    cv.setNumThreads(1)
    models = read_models()
    with ThreadPoolExecutor(max_workers=2) as pool:
        list(pool.map(lambda s: scan_recon(s, models), SCANS))
