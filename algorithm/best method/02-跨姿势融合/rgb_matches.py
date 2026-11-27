"""Cross-placement 3D correspondence proposals from the calibrated RGB photographs."""
import cv2 as cv
import numpy as np
from scipy.spatial import cKDTree

from scan_io import image, project, transform


def image_features(photo, pose, xyz, K, D, sift):
    """Lift texture keypoints by a fit to their visible, measured local depth."""
    rgb = image(photo, cv.IMREAD_COLOR)
    gray = cv.cvtColor(rgb, cv.COLOR_BGR2GRAY)
    xyz = transform(xyz, pose)
    xyz = xyz[xyz[:, 2] > 0]
    uv = project(xyz, K, D)
    height, width = gray.shape
    inside = ((uv >= 2) & (uv < [width-2, height-2])).all(axis=1)
    uv, xyz = uv[inside], xyz[inside]
    pixels = np.rint(uv).astype(np.int32)
    depth = np.full((height, width), np.inf, np.float32)
    np.minimum.at(depth, tuple(pixels.T[::-1]), xyz[:, 2])
    near = cv.erode(depth, np.ones((11, 11), np.uint8))
    visible = xyz[:, 2] < near[tuple(pixels.T[::-1])] + 2.0
    uv, xyz, pixels = uv[visible], xyz[visible], pixels[visible]
    mask = np.zeros_like(gray)
    mask[tuple(pixels.T[::-1])] = 255
    mask = cv.dilate(mask, cv.getStructuringElement(cv.MORPH_ELLIPSE, (15, 15)))
    key, desc = sift.detectAndCompute(gray, mask)
    if desc is None or len(uv) < 12:
        return np.empty((0, 3)), np.empty((0, 128), np.float32)
    xy = np.array([k.pt for k in key])
    distance, ids = cKDTree(uv).query(xy, k=12)
    ray = cv.undistortPoints(xy, K, D).reshape(-1, 2)
    ray = np.column_stack([ray, np.ones(len(ray))])
    patches = xyz[ids]
    center = np.mean(patches, axis=1)
    _, eig, vectors = np.linalg.svd(patches-center[:, None, :], full_matrices=False)
    normal = vectors[:, -1, :]
    denom = np.einsum('ij,ij->i', normal, ray)
    use = (distance[:, 0] <= 4.5) & (distance[:, -1] <= 12) & (np.abs(denom) > .28)
    use &= (eig[:, -1] <= 1.8) & (np.ptp(patches[:, :, 2], axis=1) < 5)
    val = np.einsum('ij,ij->i', normal[use], center[use]) / denom[use]
    lifted = val[:, None] * ray[use]
    ok = np.linalg.norm(lifted-center[use], axis=1) < 3.0
    root_sift = np.sqrt(desc[use][ok] / np.maximum(desc[use][ok].sum(axis=1, keepdims=True), 1e-8))
    return transform(lifted[ok], np.linalg.inv(pose)), root_sift


def correspondences(target, source, inverted=False):
    matcher = cv.BFMatcher(cv.NORM_L2)
    output = []
    for i, (x, dx) in enumerate(target):
        for j, (y, dy) in enumerate(source):
            if min(len(dx), len(dy)) < 3:
                continue
            back = {a.queryIdx: a.trainIdx for a, b in matcher.knnMatch(dx, dy, k=2)
                    if a.distance < .86*b.distance}
            for a, b in matcher.knnMatch(dy, dx, k=2):
                if back.get(a.trainIdx) != a.queryIdx or a.distance >= .86*b.distance:
                    continue
                if inverted and (y[a.queryIdx, 2] < 3. or x[a.trainIdx, 2] < 3.):
                    continue
                output.append((y[a.queryIdx], x[a.trainIdx], i, j, a.distance / b.distance))
    if not output:
        return np.empty((0, 3)), np.empty((0, 3)), np.empty((0, 3))
    s, t, i, j, w = zip(*output)
    return np.array(s), np.array(t), np.column_stack([i, j, w])


def rigid(s, t):
    """Solve one or a batch of proper rigid transforms by centered SVD."""
    s, t = np.asarray(s), np.asarray(t)
    a, b = s.mean(axis=-2), t.mean(axis=-2)
    m = (s-a[..., None, :]).swapaxes(-1, -2) @ (t-b[..., None, :])
    u, _, v = np.linalg.svd(m)
    v = v.swapaxes(-1, -2)
    v[..., :, -1] *= np.sign(np.linalg.det(v @ u.swapaxes(-1, -2)))[..., None]
    r = v @ u.swapaxes(-1, -2)
    P = np.broadcast_to(np.eye(4), r.shape[:-2]+(4, 4)).copy()
    P[..., :3, :3] = r
    P[..., :3, 3] = b - (r @ a[..., None])[..., 0]
    return P


def rgb_seeds(s, t, info, inverted):
    train = (((info[:, 0]*7 + info[:, 1]*3) % 5) >= 2)
    x, y, w = s[train], t[train], info[train, 2]
    if len(x) < 4:
        return [], train
    rng = np.random.default_rng(203)
    quality = np.exp(-6*w)
    idx = rng.choice(len(x), size=(16000, 3), p=quality/quality.sum())
    a, b = x[idx], y[idx]
    side = np.linalg.norm(a - np.roll(a, 1, axis=1), axis=2)
    sb = np.linalg.norm(b - np.roll(b, 1, axis=1), axis=2)
    area = np.linalg.norm(np.cross(a[:, 1]-a[:, 0], a[:, 2]-a[:, 0]), axis=1)
    valid = (side.min(axis=1) > 4.) & (area > 12.) & (np.max(abs(side-sb), axis=1) < 3.7)
    if not valid.any():
        return [], train
    mats = rigid(a[valid], b[valid])
    if inverted:
        mats = mats[mats[:, 2, 2] < -.55]
    candidates = []
    for offsets in np.array_split(np.arange(len(mats)), 1+len(mats)//230):
        p = mats[offsets]
        if not len(p):
            continue
        delta = np.linalg.norm(x @ p[:, :3, :3].transpose(0, 2, 1) + p[:, None, :3, 3] - y, axis=2)
        for k in np.argsort(np.sum(quality*np.maximum(0., 1-(delta/4.0)**2), axis=1))[-3:]:
            support = delta[k] < 3.7
            if support.sum() >= 4:
                candidates.append((support.sum(), rigid(x[support], y[support])))
    distinct = []
    for _, m in sorted(candidates, key=lambda i:-i[0]):
        if all(np.linalg.norm(m[:3, :3]-p[:3, :3]) > .23 for p in distinct):
            distinct.append(m)
    return distinct[:6], train
