"""Register two or three placements and fuse their measured surface points."""
import argparse
import json
from pathlib import Path

import cv2 as cv
import numpy as np
import open3d as o3d

from measured_fusion import coverage, exchange_colors, pick, sample_scans
from registration import compare, distance_pose, graph, split
from rgb_matches import correspondences, image_features
from scan_io import load_manifest, project, rigid_matrix, transform, write_cloud
from seam_repair import Frame, color_model, estimate


def features_for(placements, color_camera):
    if color_camera is None:
        return [[] for _ in placements]
    intrinsic, distortion, hik_to_rgb = color_camera
    sift = cv.SIFT_create(nfeatures=3800, contrastThreshold=.013, edgeThreshold=12)
    all_features = []
    for placement in placements:
        features = []
        for view in placement['views']:
            if view['texture'] is None:
                continue
            local_to_station = np.linalg.inv(placement['B'] @ view['station_to_v00'])
            features.append(image_features(view['texture'], hik_to_rgb @ local_to_station,
                                           placement['xyz'], intrinsic, distortion, sift))
        all_features.append(features)
    return all_features


def align(placements, color_camera):
    """Anchor the first placement, and accept a three-edge graph only on held-out support."""
    nodes = [split(placement) for placement in placements]
    if any(not node['mask'][fold].any() for node in nodes for fold in ('select', 'test')):
        raise ValueError('The clouds have insufficient spatial diversity for independent checks.')
    photos = features_for(placements, color_camera)
    order = [(0, 1)] if len(placements) == 2 else [(0, 1), (0, 2), (1, 2)]
    rows, pair_matches, chosen = [], [], []
    for target, source in order:
        inverted = placements[target]['flipped'] != placements[source]['flipped']
        matches = correspondences(photos[target], photos[source], inverted)
        candidates, failures = compare(nodes[source], nodes[target], inverted, matches)
        valid = [row for row in candidates if row['compatible'] and row['selection']['f1_2'] >= .12]
        if not valid and target == 0:
            raise ValueError(f'Insufficient overlap between {placements[source]["name"]} and the anchor.')
        winner = valid[0] if valid else None
        alternative = next((row for row in valid[1:] if distance_pose(row['pose'], winner['pose']) > .16),
                           None)
        rows.append(dict(source=source, target=target, winner=winner, alternative=alternative,
                         candidates=candidates, failures=failures))
        pair_matches.append(matches)
        if winner is not None:
            chosen.append(winner)

    poses = [np.eye(4), *(row['pose'] for row in chosen[:len(placements) - 1])]
    diagnostics = None
    if len(chosen) == 3:
        poses, diagnostics = graph(nodes, chosen, pair_matches)
    return poses, dict(pairs=rows, joint=diagnostics)


def load_poses(path, placements):
    data = json.loads(path.read_text(encoding='utf-8'))
    if data['names'] != [item['name'] for item in placements]:
        raise ValueError('Pose file names must match the manifest in the same order.')
    poses = [rigid_matrix(pose, name) for pose, name in zip(data['placement_to_primary'],
                                                         data['names'], strict=True)]
    if not np.allclose(poses[0], np.eye(4), atol=1e-7):
        raise ValueError('The first placement is the fixed identity anchor.')
    return poses


def native_frames(placements, poses):
    frames = []
    for i, (placement, pose) in enumerate(zip(placements, poses, strict=True)):
        local = pose @ placement['B']
        for j, view in enumerate(placement['views']):
            frames.append(Frame(view['xyz'], np.linalg.inv(local @ view['station_to_v00']), i, j))
    return frames


def depth_models(frames, camera):
    """Build near/far depth bounds for F03 from the native station samples."""
    intrinsic, distortion, (height, width) = camera
    h, w = (height + 2) // 3, (width + 2) // 3
    kernel = np.ones((5, 5), np.uint8)
    result = []
    for frame in frames:
        xyz = frame.camera_xyz
        x, y = np.rint(project(xyz, intrinsic, distortion) / 3).astype(np.int32).T
        valid = (xyz[:, 2] > 0) & (x >= 0) & (x < w) & (y >= 0) & (y < h)
        pixels = (y[valid], x[valid])
        z = xyz[valid, 2]
        front = np.full((h, w), 1e4, np.float32)
        back = np.full((h, w), -1e4, np.float32)
        support = np.zeros((h, w), np.uint8)
        np.minimum.at(front, pixels, z)
        np.maximum.at(back, pixels, z)
        support[pixels] = 1
        front, back = cv.erode(front, kernel), cv.dilate(back, kernel)
        count = cv.boxFilter(support, -1, (5, 5), normalize=False)
        result.append((frame.placement, frame.world_to_camera, front, back,
                       (count >= 6) & (back - front <= 4.)))
    return result


def run(manifest, output, known_poses=None):
    placements, camera, rgb_camera = load_manifest(manifest)
    if known_poses is None:
        poses, registration = align(placements, rgb_camera)
    else:
        poses, registration = load_poses(known_poses, placements), {'provided_poses': known_poses.name}

    frames = native_frames(placements, poses)
    rig = camera[:2] if camera is not None else (None, None)
    models = depth_models(frames, camera) if frames else []
    samples = sample_scans(placements, poses, rig, models)
    merged = pick(samples, 'F03 surface + depth')
    if not len(merged['xyz']):
        raise ValueError('No connected measured surface survived F03. Check millimetre units and point spacing.')
    rgb, exposure = exchange_colors(merged, samples)
    to_main = np.linalg.inv(placements[0]['B'])
    metrics = dict(names=[item['name'] for item in placements], placement_to_primary=poses,
                   camera_v00_to_primary_v00=[to_main @ pose @ item['B'] for item, pose in
                                              zip(placements, poses, strict=True)],
                   F03=coverage(merged, samples), depth_stations=len(frames), exposure=exposure,
                   **registration)
    output.mkdir(parents=True, exist_ok=True)
    write_cloud(output / 'F03-measured.ply', transform(merged['xyz'], to_main), rgb)
    np.savez_compressed(output / 'F03-provenance.npz', placement=merged['source'],
                        vertex=merged['index'])
    if frames:
        candidates, audit = estimate(frames, camera, vertex_metadata=True)
        inserted, displacement = audit.pop('vertex_arrays')
        surface = candidates['B05-Anchor']
        colors, _, corrections = color_model(
            dict(baseline=(merged['xyz'], rgb), scans=placements, poses=poses), surface)
        write_cloud(output / 'B05-repaired.ply', transform(surface, to_main), colors)
        np.savez_compressed(output / 'B05-provenance.npz', interpolated=inserted,
                            anchor_normal_shift_mm=displacement)
        metrics['B05'] = dict(output_points=len(surface), color_corrections=corrections, **audit)
    (output / 'fusion.json').write_text(
        json.dumps(metrics, ensure_ascii=False, allow_nan=False, indent=2,
                   default=lambda obj: obj.tolist()) + '\n', encoding='utf-8')
    print(f'F03: {len(merged["xyz"]):,} vertices | B05: {metrics.get("B05", {}).get("output_points", 0):,}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path, help='JSON describing each placement and its station PLYs')
    parser.add_argument('output', type=Path, help='Directory for fusion products')
    parser.add_argument('--poses', type=Path, help='Optional previously estimated placement poses')
    args = parser.parse_args()
    cv.setNumThreads(1)
    cv.setRNGSeed(203)
    o3d.utility.random.seed(203)
    run(args.manifest, args.output, args.poses)
