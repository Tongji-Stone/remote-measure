"""Merge pre-cropped single-view clouds using a calibrated turntable axis."""

import argparse
import json
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def axis_transform(axis, center_mm, angle_deg):
    rotation = Rotation.from_rotvec(np.deg2rad(angle_deg) * axis).as_matrix()
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = center_mm - rotation @ center_mm
    return transform


def nearest_summary(distances):
    values = np.percentile(distances, [50, 90, 95])
    return dict(zip(("p50_mm", "p90_mm", "p95_mm"), values.tolist()))


def neighbor_metrics(first, second, gate_mm):
    first, second = np.asarray(first.points), np.asarray(second.points)
    forward = cKDTree(second).query(first)[0]
    backward = cKDTree(first).query(second)[0]
    result = {}
    for name, distances in (("first_to_second", forward), ("second_to_first", backward)):
        matches = distances[distances <= gate_mm]
        result[name] = {
            "inlier_fraction": len(matches) / len(distances),
            "inlier_distances": nearest_summary(matches) if len(matches) else None,
        }
    return result


def stitch(manifest_path, output_dir, voxel_mm, gate_mm):
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest["unit"] != "mm":
        raise ValueError("Convert point positions and center_mm to millimeters first.")
    axis = np.asarray(manifest["axis"], dtype=float)
    center = np.asarray(manifest["center_mm"], dtype=float)
    views = manifest["views"]
    if axis.shape != (3,) or center.shape != (3,) or not np.isfinite([axis, center]).all():
        raise ValueError("axis and center_mm must be finite XYZ vectors.")
    magnitude = np.linalg.norm(axis)
    if magnitude == 0 or not np.isfinite(magnitude):
        raise ValueError("axis must have a finite, nonzero length.")
    axis /= magnitude
    if len(views) < 2:
        raise ValueError("At least two independent single-view PLY files are required.")
    paths = [(manifest_path.parent / view["file"]).resolve(strict=True) for view in views]
    if len(set(paths)) != len(paths):
        raise ValueError("Each view must reference a different PLY file.")
    angles = np.asarray([view["angle_deg"] for view in views], dtype=float)
    if angles.shape != (len(views),) or not np.isfinite(angles).all():
        raise ValueError("Each view must have a finite scalar angle_deg.")

    merged = o3d.geometry.PointCloud()
    samples = []
    transforms = []
    for path, angle in zip(paths, angles):
        cloud = o3d.io.read_point_cloud(str(path), remove_nan_points=True, remove_infinite_points=True)
        if cloud.is_empty():
            raise ValueError(f"No valid points in {path}")
        transform = axis_transform(axis, center, -(angle - angles[0]))
        cloud.transform(transform)
        merged += cloud
        samples.append(cloud.voxel_down_sample(voxel_mm))
        transforms.append({"file": str(path), "to_reference": transform.tolist()})

    edges = [(i, i + 1) for i in range(len(views) - 1)]
    if len(views) > 2:
        edges.append((len(views) - 1, 0))
    output = {
        "voxel_mm": voxel_mm,
        "gate_mm": gate_mm,
        "transforms": transforms,
        "pairs": [
            {
                "views": [i, j],
                "angles_deg": [float(angles[i]), float(angles[j])],
                **neighbor_metrics(samples[i], samples[j], gate_mm),
            }
            for i, j in edges
        ],
    }
    output_dir.mkdir(parents=True, exist_ok=False)
    if not o3d.io.write_point_cloud(str(output_dir / "merged.ply"), merged.voxel_down_sample(voxel_mm)):
        raise OSError("Failed to write merged.ply.")
    (output_dir / "overlap.json").write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")


def positive_mm(value):
    number = float(value)
    if not np.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("Expected a positive, finite distance in mm.")
    return number


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("output_dir", type=Path, help="A new directory for the merged PLY and metrics.")
    parser.add_argument("--voxel-mm", type=positive_mm, required=True)
    parser.add_argument("--gate-mm", type=positive_mm, required=True)
    args = parser.parse_args()
    stitch(args.manifest, args.output_dir, args.voxel_mm, args.gate_mm)
