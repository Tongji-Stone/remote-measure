"""Inspect vertex counts, exported view colors, and the existing repeat pair."""

import argparse
import csv
from pathlib import Path
import sys

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree


def audit(root):
    paths = sorted(root.rglob("*.ply"))
    if not paths:
        raise ValueError(f"No PLY files under {root}")
    writer = csv.writer(sys.stdout, delimiter="\t", lineterminator="\n")
    writer.writerow(("path", "vertices", "color_groups", "points_per_color_rgb_sorted",
                     "bidirectional_nn_p50", "bidirectional_nn_p90", "bidirectional_nn_p95"))
    for path in paths:
        cloud = o3d.io.read_point_cloud(str(path))
        points = np.asarray(cloud.points)
        rgb = np.rint(np.asarray(cloud.colors) * 255).astype(np.uint8)
        _, labels, counts = np.unique(rgb, axis=0, return_inverse=True, return_counts=True)
        repeat_distances = ["", "", ""]
        if path.name in ("loop_0000_0004_before.ply", "loop_0000_0004_fitted.ply"):
            if len(counts) != 2:
                raise ValueError(f"Expected two repeat colors in {path}")
            first, second = (points[labels == idx] for idx in range(2))
            distances = np.concatenate((cKDTree(second).query(first)[0],
                                        cKDTree(first).query(second)[0]))
            repeat_distances = [f"{value:.3f}" for value in np.percentile(distances, [50, 90, 95])]
        writer.writerow((path.relative_to(root), len(points), len(counts),
                         ",".join(map(str, counts)), *repeat_distances))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    audit(parser.parse_args().root)
