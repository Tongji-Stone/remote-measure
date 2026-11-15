"""Open an experimental PLY point cloud, colored by depth for visibility."""
import argparse
import os
from pathlib import Path

import numpy as np
import open3d as o3d
from matplotlib import colormaps


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('file', type=Path)
    parser.add_argument('--preserve-colors', action='store_true')
    args = parser.parse_args()
    path = args.file.resolve(strict=True)
    # Open3D on Windows may not handle non-ASCII parent paths directly.
    os.chdir(path.parent)
    cloud = o3d.io.read_point_cloud(path.name)
    points = np.asarray(cloud.points)
    if not len(points):
        raise ValueError('Point cloud is empty or could not be read')
    if not args.preserve_colors or not cloud.has_colors():
        z = points[:, 2]
        low, high = np.percentile(z, [1, 99])
        colors = colormaps['turbo'](np.clip((z-low) / max(high-low, 1e-9), 0, 1))[:, :3]
        cloud.colors = o3d.utility.Vector3dVector(colors)
    print(f'{len(points)} points; color represents depth, not object color.', flush=True)
    o3d.visualization.draw_geometries([cloud], window_name='Depth colored point cloud')


if __name__ == '__main__':
    main()
