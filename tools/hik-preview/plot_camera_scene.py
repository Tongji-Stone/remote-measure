"""Plot a camera-coordinate point cloud and a schematic camera in one figure."""
import argparse
import os
from pathlib import Path

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from matplotlib.lines import Line2D

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CALIBRATION = ROOT / 'data/procam_calibration/calibration_20261116_10mm_experimental.xml'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('point_cloud', type=Path)
    parser.add_argument('--calibration', type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    path = args.point_cloud.resolve(strict=True)
    output = args.output.resolve()
    calibration_path = args.calibration.resolve(strict=True)
    os.chdir(path.parent)
    cloud = o3d.io.read_point_cloud(path.name)
    xyz = np.asarray(cloud.points)
    if not len(xyz):
        raise ValueError('Empty point cloud')
    colors = np.asarray(cloud.colors) if cloud.has_colors() else np.tile([0.15, 0.35, 0.7], (len(xyz), 1))
    calibration = calibration_path.read_text(encoding='utf-8')
    fs = cv2.FileStorage(calibration, cv2.FILE_STORAGE_READ | cv2.FILE_STORAGE_MEMORY)
    K = fs.getNode('cam_int').mat()
    shape = fs.getNode('img_shape').mat()
    fs.release()
    if K is None or shape is None:
        raise ValueError('Camera intrinsics and image size are required')
    height, width = shape.ravel()[:2]

    plt.rcParams.update({'font.family': 'Times New Roman', 'font.weight': 'bold',
                         'axes.labelweight': 'bold', 'axes.titleweight': 'bold'})
    fig = plt.figure(figsize=(12, 8), facecolor='white')
    ax = fig.add_subplot(111, projection='3d')
    # Plot coordinates are (camera X, camera Z, -camera Y) so that up is up.
    stride = max(1, len(xyz) // 18000)
    sample = xyz[::stride]
    ax.scatter(sample[:, 0], sample[:, 2], -sample[:, 1], c=colors[::stride], s=1.0, alpha=.7, linewidths=0)

    def line(a, b, color='#142e78', lw=1.5, **kwargs):
        a, b = np.asarray(a), np.asarray(b)
        ax.plot([a[0], b[0]], [a[2], b[2]], [-a[1], -b[1]], color=color, linewidth=lw, **kwargs)

    # Frustum is computed from pinhole intrinsics; camera body is illustrative.
    z = 55.0
    corners = [np.array([(u-K[0, 2])*z/K[0, 0], (v-K[1, 2])*z/K[1, 1], z])
               for u, v in ((0, 0), (width, 0), (width, height), (0, height))]
    for i in range(4):
        line([0, 0, 0], corners[i], color='#17488c')
        line(corners[i], corners[(i+1) % 4], color='#17488c')
    back = [np.array([x, y, -22.0]) for x, y in ((-18, -14), (18, -14), (18, 14), (-18, 14))]
    front = [np.array([x, y, -2.0]) for x, y in ((-18, -14), (18, -14), (18, 14), (-18, 14))]
    for i in range(4):
        line(back[i], back[(i+1) % 4]); line(front[i], front[(i+1) % 4]); line(back[i], front[i])
    ax.scatter([0], [0], [0], color='#d62828', s=35, depthshade=False)
    ax.text(-12, -38, 38, 'Camera', color='#142e78', fontweight='bold')
    ax.set_xlabel('Camera X (mm)', labelpad=10)
    ax.set_ylabel('Camera Z / forward (mm)', labelpad=10)
    ax.set_zlabel('Up = -camera Y (mm)', labelpad=10)
    ax.set_title('Camera and reconstructed object (experimental)', pad=18)
    lo, hi = np.percentile(xyz, [1, 99], axis=0)
    ax.set_xlim(min(-65, lo[0]-15), max(65, hi[0]+15))
    ax.set_ylim(-45, max(hi[2]+35, 100))
    ax.set_zlim(min(-60, -hi[1]-15), max(60, -lo[1]+15))
    ax.set_box_aspect((max(130, hi[0]-lo[0]+30), max(hi[2]+80, 145), max(120, hi[1]-lo[1]+30)))
    ax.view_init(elev=23, azim=-68)
    ax.legend(handles=[Line2D([0], [0], marker='o', linestyle='', color=c, label=f'View {i:04}', markersize=6)
                       for i, c in enumerate(('#1f77b4', '#ff7f0e', '#2ca02c', '#d62728'))], loc='upper left')
    fig.text(.5, .025, 'Camera position and frustum are schematic; reconstruction and scale are unverified.',
             ha='center', fontsize=10, fontweight='bold')
    fig.savefig(output, dpi=180, bbox_inches='tight')
    print('Saved:', output)


if __name__ == '__main__':
    main()
