"""Projection, ChArUco board pose and native camera-space PLY export (E101)."""
import cv2 as cv
import numpy as np
from scipy.spatial.transform import Rotation


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


def masks_for_tags(obs, model):
    """Return a calibrated board pose when tag support and reprojection pass."""
    if obs is None or len(obs['ids']) < 3:
        return None
    try:
        result = stage_observation(obs, model)
        translation = result['pose'][:3, 3]
        if np.linalg.norm(translation) < 120 or translation[2] <= 0:
            return None
        residual = project(transform(result['board_xyz'], result['pose']), model) - result['board_uv']
        if np.median(np.linalg.norm(residual, axis=1)) > 3.5:
            return None
        return result
    except (cv.error, ValueError, TypeError):
        return None


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
            rec['rgb'] = data['rgb'] if 'rgb' in data else data['gray'][:, None]
            rec['view'] = idx
            rec.tofile(file)
    temp.replace(path)
    return count
