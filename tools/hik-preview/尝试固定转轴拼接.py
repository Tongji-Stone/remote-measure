"""Exploratory fixed-axis fit. The result is not a validated reconstruction."""
import os
from pathlib import Path

os.environ.setdefault('OMP_NUM_THREADS', '2')
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import colormaps

ROOT = Path(__file__).resolve().parents[2] / 'data' / 'procam_calibration'
IDS = ['124450', '124618', '124643', '124852', '124950']


def load(key):
    folder = next(ROOT.glob('capture_*' + key + '*'))
    old = os.getcwd()
    try:
        os.chdir(folder)
        cloud = o3d.io.read_point_cloud('object_roi_experimental.ply')
    finally:
        os.chdir(old)
    points = np.asarray(cloud.points)
    lo, hi = np.percentile(points, [0.5, 99.5], axis=0)
    idx = np.flatnonzero(np.all((points >= lo) & (points <= hi), axis=1))
    return cloud.select_by_index(idx.tolist()).voxel_down_sample(1.0)


def rotation(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def main():
    clouds = [load(key) for key in IDS]
    rng = np.random.default_rng(7)
    sampled = []
    for cloud in clouds:
        pts = np.asarray(cloud.voxel_down_sample(2.0).points)
        sampled.append(pts[rng.choice(len(pts), min(1000, len(pts)), replace=False)])
    trees = [cKDTree(p) for p in sampled]
    all_pts = np.concatenate(sampled)
    pivot0 = np.median(all_pts[:, [0, 2]], axis=0)
    bounds = (np.r_[pivot0 - 100, np.zeros(4)], np.r_[pivot0 + 100, np.full(4, 70.)])

    def residual(x, direction):
        pivot = np.array([x[0], 0., x[1]])
        result = []
        for i, degrees in enumerate(x[2:]):
            r = rotation(np.deg2rad(direction * degrees))
            moved = (sampled[i+1] - pivot) @ r.T + pivot
            distances = trees[i].query(moved, workers=1)[0]
            # Only the overlapping portion is expected to match.
            result.extend(np.sort(np.minimum(distances, 12))[:400])
        return np.asarray(result)

    best = None
    for direction in (1, -1):
        for initial_angle in (10., 30., 55.):
            x0 = np.r_[pivot0, np.full(4, initial_angle)]
            result = least_squares(residual, x0, args=(direction,), bounds=bounds,
                                   max_nfev=80, diff_step=0.001, ftol=1e-5)
            score = np.mean(residual(result.x, direction)**2)**0.5
            if best is None or score < best[0]:
                best = (score, direction, result.x)
    score, direction, x = best
    pivot = np.array([x[0], 0., x[1]])
    print('axis is assumed parallel to camera Y; point on axis:', pivot, flush=True)
    print('pair angles degrees:', x[2:], 'direction:', direction, 'trimmed fit error:', score, flush=True)
    merged = o3d.geometry.PointCloud()
    fig = plt.figure(figsize=(9, 7))
    plt.rcParams.update({'font.family': 'Times New Roman', 'font.weight': 'bold', 'axes.labelweight': 'bold'})
    ax = fig.add_subplot(111, projection='3d')
    cumulative = 0.
    for i, cloud in enumerate(clouds):
        if i:
            cumulative += direction * x[i+1]
        r = rotation(np.deg2rad(cumulative))
        transform = np.eye(4)
        transform[:3, :3] = r
        transform[:3, 3] = pivot - r @ pivot
        cloud.transform(transform)
        rgb = colormaps['tab10'](i)[:3]
        cloud.paint_uniform_color(rgb)
        merged += cloud
        pts = np.asarray(cloud.points)
        pts = pts[::max(1, len(pts)//7000)]
        ax.scatter(pts[:, 0], pts[:, 1], -pts[:, 2], s=1, color=rgb, label=IDS[i], alpha=.65)
    xyz = np.asarray(merged.points)
    ax.set_box_aspect(np.maximum(np.ptp(xyz, axis=0), 1))
    ax.set(xlabel='X', ylabel='Y', zlabel='-Z', title='Fixed-axis experiment (unverified)')
    ax.view_init(18, -60)
    ax.legend(loc='upper left')
    fig.tight_layout()
    fig.savefig(ROOT / 'box_fixed_axis_experimental.png', dpi=170)
    plt.close(fig)
    old = os.getcwd()
    try:
        os.chdir(ROOT)
        if not o3d.io.write_point_cloud('box_fixed_axis_experimental.ply', merged):
            raise RuntimeError('Failed to save point cloud')
    finally:
        os.chdir(old)
    print('saved points:', len(merged.points), flush=True)


if __name__ == '__main__':
    main()
