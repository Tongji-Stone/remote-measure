"""Run the local phase-A registration comparisons on independent survey views."""

import argparse
from collections import deque
import csv
import hashlib
import json
from pathlib import Path
import subprocess
from time import perf_counter

import cv2
import numpy as np
import open3d as o3d
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from stitch_turntable import axis_transform

REG = o3d.pipelines.registration
IDENTITY = np.eye(4)
METHODS = ("point_to_plane", "point_to_point", "tukey_icp", "gicp", "gray_icp",
           "cloudcompare_icp", "fpfh_ransac", "fpfh_fgr")
COLOR = np.array([[.10, .48, .71], [.89, .48, .19], [.19, .63, .47],
                  [.65, .32, .65], [.91, .65, .20]])


def json_dump(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def sha256(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def points(path):
    pcd = o3d.io.read_point_cloud(str(path))
    return np.asarray(pcd.points).copy()


def transformed(xyz, pose):
    return xyz @ pose[:3, :3].T + pose[:3, 3]


def cloud(xyz, voxel=None, normals_radius=None):
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    if voxel:
        pcd = pcd.voxel_down_sample(voxel)
    if normals_radius:
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=normals_radius, max_nn=40))
    return pcd


def polygon_mask(pixels, polygon, imshape):
    mask = np.zeros(imshape, dtype=np.uint8)
    cv2.fillPoly(mask, [np.asarray(polygon, dtype=np.int32) * 4], 1)
    xy = pixels.round().astype(int)
    valid = (xy[:, 0] >= 0) & (xy[:, 0] < mask.shape[1]) & (xy[:, 1] >= 0) & (xy[:, 1] < mask.shape[0])
    valid[valid] = mask[xy[valid, 1], xy[valid, 0]].astype(bool)
    return valid


def rectangle_mask(pixels, rect):
    left, top, right, bottom = np.asarray(rect) * 4
    return ((left <= pixels[:, 0]) & (pixels[:, 0] < right)
            & (top <= pixels[:, 1]) & (pixels[:, 1] < bottom))


def dense_keep(xyz):
    distance = cKDTree(xyz).query(xyz, k=12)[0][:, -1]
    median = np.median(distance)
    return distance <= median + 4 * max(np.median(np.abs(distance - median)), .1)


def training_mask(pixels):
    # Fixed image tiles keep evaluation vertices out of the optimization arrays.
    tile = (pixels[:, 0].astype(int) // 24 + 2 * (pixels[:, 1].astype(int) // 24)) % 5
    return tile != 0


def sample_gray(image, pixels):
    xy = np.rint(pixels).astype(int)
    x = np.clip(xy[:, 0], 0, image.shape[1] - 1)
    y = np.clip(xy[:, 1], 0, image.shape[0] - 1)
    return image[y, x].astype(float) / 255


def distances(first, second):
    return cKDTree(second).query(first)[0]


def summary(values, gates):
    return {
        "n": len(values),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
        **{f"coverage_{g:g}": float(np.mean(values <= g)) for g in gates},
    }


def pair_metrics(src, dst, transform, gates):
    xyz = transformed(src, transform)
    a = distances(xyz, dst)
    b = distances(dst, xyz)
    return {
        "forward": summary(a, gates),
        "backward": summary(b, gates),
        "mean_coverage_2": float((np.mean(a <= 2) + np.mean(b <= 2)) / 2),
        "min_coverage_4": float(min(np.mean(a <= 4), np.mean(b <= 4))),
    }


def prepare_pair(xyz, config, gray=None):
    pcd = cloud(xyz)
    if gray is not None:
        pcd.colors = o3d.utility.Vector3dVector(np.repeat(gray[:, None], 3, axis=1))
    levels = []
    for v in config["training_voxels"]:
        sampled = pcd.voxel_down_sample(v)
        sampled.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=max(3 * v, 2.4), max_nn=40))
        levels.append(sampled)
    v = config["fpfh_voxel"]
    coarse = cloud(xyz, v, 2.5 * v)
    coarse.orient_normals_towards_camera_location(np.zeros(3))
    feature = REG.compute_fpfh_feature(coarse, o3d.geometry.KDTreeSearchParamHybrid(radius=5 * v, max_nn=110))
    return levels, coarse, feature


def cloudcompare(src, dst, initial, config, folder, binary):
    folder.mkdir()
    src0 = transformed(src, initial)
    a, b = folder / "moving.ply", folder / "fixed.ply"
    o3d.io.write_point_cloud(str(a), cloud(src0))
    o3d.io.write_point_cloud(str(b), cloud(dst))
    cmd = [str(binary), "-SILENT", "-NO_TIMESTAMP", "-AUTO_SAVE", "OFF", "-O", str(a), "-O", str(b),
           "-ICP", "-OVERLAP", str(config["cloudcompare_overlap"]), "-ITER", "60",
           "-C_EXPORT_FMT", "PLY", "-SELECT_ENTITIES", "-FIRST", "1", "-CLOUD",
           "-SAVE_CLOUDS", "FILE", str(folder / "aligned.ply")]
    proc = subprocess.run(cmd, cwd=folder, capture_output=True, text=True, timeout=45)
    log = proc.stdout + proc.stderr
    (folder / "command.json").write_text(json.dumps(cmd, indent=2))
    (folder / "cloudcompare.log").write_text(log)
    if proc.returncode or not (folder / "aligned.ply").exists():
        raise RuntimeError("CloudCompare ICP did not export an aligned cloud.")
    aligned = points(folder / "aligned.ply")
    if len(aligned) != len(src0):
        raise RuntimeError("CloudCompare changed the point count.")
    correction = rigid_alignment(src0, aligned)
    if np.percentile(np.linalg.norm(transformed(src0, correction) - aligned, axis=1), 99) >= .02:
        raise RuntimeError("CloudCompare export could not be verified as rigid with original vertex order.")
    return correction @ initial


def rigid_alignment(first, second):
    x, y = first.mean(axis=0), second.mean(axis=0)
    u, _, vt = np.linalg.svd((first - x).T @ (second - y))
    parity = np.sign(np.linalg.det(vt.T @ u.T))
    transform = IDENTITY.copy()
    transform[:3, :3] = vt.T @ np.diag([1, 1, parity]) @ u.T
    transform[:3, 3] = y - transform[:3, :3] @ x
    return transform


def go_icp(src, dst, binary, folder, cfg):
    folder.mkdir()
    start = perf_counter()
    ma, mb = src.mean(axis=0), dst.mean(axis=0)
    scale = float(max(np.max(np.abs(src - ma)), np.max(np.abs(dst - mb))) * 1.01)
    a, b = folder / "moving.txt", folder / "fixed.txt"
    rng = np.random.default_rng(cfg["seed"])
    np.savetxt(a, (src[rng.permutation(len(src))] - ma) / scale, header=str(len(src)), comments="")
    np.savetxt(b, (dst - mb) / scale, header=str(len(dst)), comments="")
    settings = folder / "params.txt"
    result = folder / "transform.txt"
    settings.write_text(f"MSEThresh={(1.0 / scale) ** 2:.9f}\n"
                        "rotMinX=-3.1416\nrotMinY=-3.1416\nrotMinZ=-3.1416\nrotWidth=6.2832\n"
                        "transMinX=-0.5\ntransMinY=-0.5\ntransMinZ=-0.5\ntransWidth=1.0\n"
                        f"trimFraction={cfg['goicp_trim_fraction']}\n"
                        f"distTransSize={cfg['goicp_distance_grid']}\ndistTransExpandFactor=2.0\n")
    cmd = [str(binary), str(b), str(a), str(min(700, len(src))), str(settings), str(result)]
    json_dump(folder / "command.json", cmd)
    with (folder / "goicp.log").open("w") as log:
        try:
            subprocess.run(cmd, cwd=folder, stdout=log, stderr=subprocess.STDOUT,
                           timeout=cfg["goicp_timeout_seconds"], check=True)
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError(f"Go-ICP timed out at {cfg['goicp_timeout_seconds']} s.") from exc
    values = np.fromstring(result.read_text(), sep=" ")
    if len(values) != 13:
        raise RuntimeError("Unexpected Go-ICP transform output.")
    pose = IDENTITY.copy()
    pose[:3, :3] = Rotation.from_matrix(values[1:10].reshape(3, 3)).as_matrix()
    pose[:3, 3] = values[10:] * scale + mb - pose[:3, :3] @ ma
    return pose, perf_counter() - start


def fit_pair(method, src, dst, init, config, folder, binary, seed):
    start = perf_counter()
    a, a_coarse, fa = src
    b, b_coarse, fb = dst
    transform = init.copy()
    if method == "cloudcompare_icp":
        if binary is None:
            raise RuntimeError("CloudCompare is unavailable.")
        transform = cloudcompare(np.asarray(a[-1].points), np.asarray(b[-1].points),
                                 transform, config, folder, binary)
    else:
        if method.startswith("fpfh"):
            o3d.utility.random.seed(seed)
            gate = 1.7 * config["fpfh_voxel"]
            if method == "fpfh_ransac":
                result = REG.registration_ransac_based_on_feature_matching(
                    a_coarse, b_coarse, fa, fb, True, gate,
                    REG.TransformationEstimationPointToPoint(False), 3,
                    [REG.CorrespondenceCheckerBasedOnEdgeLength(.9),
                     REG.CorrespondenceCheckerBasedOnDistance(gate)],
                    REG.RANSACConvergenceCriteria(config["fpfh_ransac_iterations"], .999))
            else:
                result = REG.registration_fgr_based_on_feature_matching(
                    a_coarse, b_coarse, fa, fb, REG.FastGlobalRegistrationOption(maximum_correspondence_distance=gate))
            if result.fitness == 0:
                raise RuntimeError("FPFH initialization found no inlier-supported transform.")
            transform = result.transformation
        for first, second, gate in zip(a, b, config["training_gates"]):
            criteria = REG.ICPConvergenceCriteria(max_iteration=50)
            if method == "gicp":
                result = REG.registration_generalized_icp(
                    first, second, gate, transform,
                    REG.TransformationEstimationForGeneralizedICP(1e-3, REG.HuberLoss(gate / 3)), criteria)
            elif method == "gray_icp":
                if not first.has_colors() or not second.has_colors():
                    raise RuntimeError("The gray reference image is unavailable.")
                result = REG.registration_colored_icp(
                    first, second, gate, transform, REG.TransformationEstimationForColoredICP(.968), criteria)
            else:
                if method == "point_to_point":
                    estimation = REG.TransformationEstimationPointToPoint(False)
                else:
                    loss = REG.TukeyLoss(gate * .75) if method == "tukey_icp" else REG.HuberLoss(gate / 3)
                    estimation = REG.TransformationEstimationPointToPlane(loss)
                result = REG.registration_icp(first, second, gate, transform,
                                              estimation, criteria)
            transform = result.transformation
    if not np.isfinite(transform).all() or not np.isclose(np.linalg.det(transform[:3, :3]), 1, atol=.002):
        raise RuntimeError("Registration returned an invalid rigid transform.")
    return transform, perf_counter() - start


def box_image_reference(xyz, images, pixels, cfg, camera, dist, output):
    face = [
        cloud(x[polygon_mask(uv, polygon, im.shape) & ~rectangle_mask(uv, label)], 1.5)
        for x, uv, polygon, label, im in zip(xyz, pixels, cfg["box_face_polygons_quarter_pixels"],
                                             cfg["box_label_rects_quarter_pixels"], images)
    ]
    # Both face frames use the same orientation conventions, with Y down and normals facing the camera.
    planes = []
    frames = []
    for i, pcd in enumerate(face):
        o3d.utility.random.seed(cfg["seed"] + i)
        plane, ids = pcd.segment_plane(distance_threshold=1.1, ransac_n=3, num_iterations=600)
        plane = np.asarray(plane)
        center = np.asarray(pcd.points)[ids].mean(axis=0)
        if plane[:3] @ center > 0:
            plane *= -1
        z = plane[:3]
        y = np.array([0., 1., 0.]) - z[1] * z
        y /= np.linalg.norm(y)
        frames.append((center, np.column_stack([np.cross(y, z), y, z])))
        planes.append(plane)
    first, second = frames
    init = IDENTITY.copy()
    init[:3, :3] = second[1] @ first[1].T
    init[:3, 3] = second[0] - init[:3, :3] @ first[0]

    sift = cv2.SIFT_create(contrastThreshold=.015)
    candidates = []
    for im, (left, top, right, bottom) in zip(images, cfg["box_label_rects_quarter_pixels"]):
        mask = np.zeros_like(im)
        mask[4 * top:4 * bottom, 4 * left:4 * right] = 255
        candidates.append(sift.detectAndCompute(im, mask))
    (ka, da), (kb, db) = candidates
    pairs = [a for a, b in cv2.BFMatcher(cv2.NORM_L2).knnMatch(da, db, k=2) if a.distance < .70 * b.distance]
    uv_a = np.array([ka[m.queryIdx].pt for m in pairs])
    uv_b = np.array([kb[m.trainIdx].pt for m in pairs])
    cv2.setRNGSeed(cfg["seed"])
    _, inliers = cv2.findHomography(uv_a, uv_b, cv2.RANSAC, 3.0)
    keep = np.flatnonzero(inliers.ravel())
    # Spatial thinning reduces duplicate votes from one printed letter or multiscale keypoint.
    cells = {}
    for k in sorted(keep, key=lambda index: pairs[index].distance):
        cells.setdefault(tuple((uv_a[k] // 25).astype(int)), k)
    chosen = list(cells.values())
    if len(chosen) < 12:
        raise RuntimeError("Insufficient spatially distinct box label matches.")
    p3 = []
    for uv, plane in zip((uv_a[chosen], uv_b[chosen]), planes):
        xy = cv2.undistortPoints(uv.reshape(-1, 1, 2), camera, dist).reshape(-1, 2)
        ray = np.column_stack([xy, np.ones(len(xy))])
        p3.append(ray * (-plane[3] / (ray @ plane[:3]))[:, None])
    output.update({"label_inliers": len(keep), "label_spatial_pairs": len(chosen),
                   "label_sift_pixels": np.stack([uv_a[chosen], uv_b[chosen]]).tolist(),
                   "planes": [p.tolist() for p in planes],
                   "label_reference_points": [x.tolist() for x in p3]})
    return init, p3


def refine_angles(train, nominal, train_edges, axis, center, config):
    start = perf_counter()
    delta = np.zeros(len(train) - 1)
    limit = config["axis_refinement_limit_degrees"]
    sigma = config["axis_refinement_prior_sigma_degrees"]

    def poses(values):
        return [axis_transform(axis, center, d) @ t for d, t in zip(np.r_[0., values], nominal)]

    for voxel, gate in zip(config["training_voxels"], config["training_gates"]):
        clouds = [cloud(x, voxel, max(voxel * 3, 2.4)) for x in train]
        xyz = [np.asarray(c.points) for c in clouds]
        normals = [np.asarray(c.normals) for c in clouds]
        for _ in range(6):
            current = poses(delta)
            aligned = [transformed(x, t) for x, t in zip(xyz, current)]
            trees = [cKDTree(x) for x in aligned]
            matches = []
            for a, b in train_edges:
                for i, j in ((a, b), (b, a)):
                    d, index = trees[j].query(aligned[i])
                    accepted = np.flatnonzero(d <= gate)
                    if len(accepted) < 24:
                        continue
                    keep = accepted[np.linspace(0, len(accepted) - 1, min(len(accepted), 450)).astype(int)]
                    matches.append((i, j, keep, index[keep]))
            if not matches:
                break

            def residual(values):
                t = poses(values)
                output = []
                for i, j, first, second in matches:
                    difference = transformed(xyz[i][first], t[i]) - transformed(xyz[j][second], t[j])
                    n = normals[j][second] @ t[j][:3, :3].T
                    output.append(np.sum(difference * n, axis=1))
                # An 8-degree prior and about 25 point constraints per angle keep weak views near the original estimate.
                output.append(5 * values / sigma)
                return np.concatenate(output)

            fit = least_squares(residual, delta, bounds=(-limit, limit), loss="huber",
                                f_scale=gate / 3, max_nfev=30)
            if np.linalg.norm(fit.x - delta) < .005:
                delta = fit.x
                break
            delta = fit.x
    return poses(delta), delta.tolist(), perf_counter() - start


def fit_rim(xyz, pixels, axis, origin, config):
    x = np.array([1., 0, 0]) - axis[0] * axis
    x /= np.linalg.norm(x)
    basis = np.column_stack([x, np.cross(axis, x)])
    offset = xyz - origin
    mask = (np.abs(offset @ axis) < config["rim_plane_gate"]) & rectangle_mask(
        pixels, config["rim_disk_rect_quarter_pixels"])
    plane = offset[mask] @ basis
    radius = np.linalg.norm(plane, axis=1)
    width = config["rim_sector_degrees"]
    sectors = np.floor(np.rad2deg(np.arctan2(plane[:, 1], plane[:, 0])) / width)
    rim = []
    for s in range(-180 // width, 180 // width):
        ids = np.flatnonzero((sectors == s) & (25 < radius) & (radius < 78))
        if len(ids) > 25:
            r = np.quantile(radius[ids], config["rim_sector_radial_quantile"])
            rim.append(plane[ids[np.argmin(np.abs(radius[ids] - r))]])
    if len(rim) < 12:
        raise RuntimeError("Too few sectors of the disk rim are visible.")
    rim = np.asarray(rim)
    fit = least_squares(lambda p: np.linalg.norm(rim - p[:2], axis=1) - p[2],
                        [0., 0., 53.], loss="soft_l1", f_scale=2.,
                        bounds=([-16., -16., 30.], [16., 16., 78.]))
    return {"point": (origin + basis @ fit.x[:2]).tolist(), "radius": float(fit.x[2]),
            "sectors": len(rim), "residual_p90": float(np.percentile(np.abs(fit.fun), 90))}


def optimize_graph(views, edges):
    adjacency = [[] for _ in views]
    accepted = sorted(edges, key=lambda e: e["train_min_coverage_4"], reverse=True)
    for edge in accepted:
        i, j = edge["i"], edge["j"]
        a = edge["transform"]
        adjacency[i].append((j, np.linalg.inv(a), edge))
        adjacency[j].append((i, a, edge))
    poses = [None] * len(views)
    poses[0] = IDENTITY
    queue = deque([0])
    tree = set()
    while queue:
        i = queue.popleft()
        for j, t, edge in adjacency[i]:
            if poses[j] is None:
                poses[j] = poses[i] @ t
                tree.add((edge["i"], edge["j"]))
                queue.append(j)
    if any(p is None for p in poses):
        return None
    graph = REG.PoseGraph()
    graph.nodes.extend(REG.PoseGraphNode(t) for t in poses)
    for edge in accepted:
        i, j = edge["i"], edge["j"]
        a = edge["transform"]
        info = REG.get_information_matrix_from_point_clouds(views[i][0][-1], views[j][0][-1], 4., a)
        graph.edges.append(REG.PoseGraphEdge(i, j, a, info, uncertain=(i, j) not in tree))
    opt = REG.GlobalOptimizationOption(max_correspondence_distance=4., edge_prune_threshold=.25, reference_node=0)
    REG.global_optimization(graph, REG.GlobalOptimizationLevenbergMarquardt(),
                            REG.GlobalOptimizationConvergenceCriteria(), opt)
    return [node.pose.copy() for node in graph.nodes]


def save_apples(root, config, output, cc, go_binary, inputs):
    names = ["view_%04d" % i for i in range(5)]
    folder = root / config["apple"]
    session_path = folder / "session.json"
    meta_path = folder / "merged_ignore1_metadata.json"
    inputs.extend([session_path, meta_path])
    metadata = json.loads(meta_path.read_text())["axis"]
    angles = [v["nominal_angle_degrees"] for v in json.loads(session_path.read_text())["views"]]
    axis = np.asarray(metadata["axis_direction"])
    axis /= np.linalg.norm(axis)
    center = np.asarray(metadata["axis_point_camera_mm"])
    raw, train, evaluate, gray, tag_vectors, rim_fits = [], [], [], [], [], []
    info = []
    for name, polygon in zip(names, config["apple_polygons_quarter_pixels"]):
        data = folder / name
        src = data / "20261120_ignore1_experimental.ply"
        npz = data / "20261120_ignore1_map.npz"
        frame = data / "graycode_01.png"
        inputs.extend([src, npz, frame])
        im = cv2.imread(str(frame), cv2.IMREAD_COLOR)
        with np.load(npz) as manifest:
            uv = manifest["camera_pixels"]
            xyz = manifest["xyz"].astype(float)
        if not np.array_equal(xyz, points(src)):
            raise RuntimeError(f"NPZ and PLY data are inconsistent for {name}.")
        if name != names[-1]:
            fit = fit_rim(xyz, uv, axis, center, config)
            rim_fits.append({"view": name, **fit})
        tag = config["apple_tag_rects_quarter_pixels"].get(name)
        if tag:
            m = rectangle_mask(uv, tag) & (np.abs((xyz - center) @ axis) < 4)
            v = np.median(xyz[m], axis=0) - center
            tag_vectors.append(v - (axis @ v) * axis)
        valid = polygon_mask(uv, polygon, im.shape[:2])
        selected = np.flatnonzero(valid)
        selected = selected[dense_keep(xyz[selected])]
        xyz0, uv0 = xyz[selected], uv[selected]
        use = training_mask(uv0)
        a, b = xyz0[use], xyz0[~use]
        raw.append(xyz0)
        train.append(a)
        evaluate.append(b)
        gray.append(sample_gray(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), uv0[use]))
        info.append({"view": name, "nominal_angle_deg": angles[len(info)], "input": len(xyz),
                     "roi": len(xyz0), "train": len(a), "reserved_tiles": len(b)})
        overlay = cv2.resize(im, (612, 512))
        cv2.polylines(overlay, [np.asarray(polygon)], True, (45, 220, 45), 1)
        for x, y in (uv0[::6] / 4).astype(int):
            cv2.circle(overlay, (x, y), 1, (70, 170, 255), -1)
        cv2.imwrite(str(output / f"roi_{name}.jpg"), overlay)
    a, b = tag_vectors
    tag_sign_check = float(np.rad2deg(np.arctan2(axis @ np.cross(a, b), a @ b)))
    sign = np.sign(tag_sign_check)
    nominal = [axis_transform(axis, center, -sign * a) for a in angles[:4]]
    ring = [(0, 1), (1, 2), (2, 3), (3, 0)]
    models = [prepare_pair(x, config, intensity) for x, intensity in zip(train[:4], gray[:4])]
    gates = config["evaluation_gates"]
    threshold = config["registration_edge_min_bidirectional_coverage_at_4"]
    holdout = raw[4]
    common = holdout[distances(holdout, raw[0]) <= 2.]
    if not len(common):
        raise RuntimeError("Nominal repeat has no supported subset.")
    results = {}

    def evaluate_poses(name, poses, time, fit_details):
        aligned = [transformed(x, t) for x, t in zip(raw[:4], poses)]
        reserved = [transformed(x, t) for x, t in zip(evaluate[:4], poses)]
        ring_eval = [pair_metrics(reserved[i], reserved[j], IDENTITY, gates) for i, j in ring]
        entry = {"state": "complete", "seconds": time, "fit_details": fit_details,
                 "transforms": [t.tolist() for t in poses], "ring_evaluation": ring_eval,
                 "edges_passing_evaluation_gate": sum(x["min_coverage_4"] >= threshold for x in ring_eval),
                 "mean_ring_coverage_2": float(np.mean([x["mean_coverage_2"] for x in ring_eval])),
                 "nominal_correction_deg": [float(np.rad2deg(Rotation.from_matrix(
                     t[:3, :3] @ n[:3, :3].T).magnitude())) for t, n in zip(poses, nominal)],
                 "holdout_to_fused": summary(distances(holdout, np.vstack(aligned)), gates),
                 "holdout_to_views_1_3": summary(distances(holdout, np.vstack(aligned[1:])), gates),
                 "fixed_common_holdout_to_views_1_3": summary(distances(common, np.vstack(aligned[1:])), gates)}
        merged = o3d.geometry.PointCloud()
        for index, xyz in enumerate(aligned):
            colored = cloud(xyz)
            colored.paint_uniform_color(COLOR[index])
            merged += colored
        o3d.io.write_point_cloud(str(output / f"apple_{name}.ply"), merged)
        results[name] = entry
        print(f"apple {name}: mean at 2 = {entry['mean_ring_coverage_2']:.3f}, "
              f"repeat → side {entry['holdout_to_views_1_3']['coverage_2']:.3f}", flush=True)

    evaluate_poses("nominal", nominal, 0., {})
    poses, delta, seconds = refine_angles(train[:4], nominal, ring, axis, center, config)
    evaluate_poses("axis_refined", poses, seconds, {"rotation_correction_about_axis_deg": delta})
    historical_angles = np.r_[0., np.cumsum(metadata["relative_quarter_degrees"])]
    historical = [axis_transform(axis, center, -sign * a) for a in historical_angles]
    evaluate_poses("historical_angles", historical, 0., {"archive_angles_deg": historical_angles.tolist()})
    rim_center = np.median([x["point"] for x in rim_fits], axis=0)
    rim_poses = [axis_transform(axis, rim_center, -sign * a) for a in angles[:4]]
    evaluate_poses("rim_center", rim_poses, 0., {"circle_fits": rim_fits, "axis_point": rim_center.tolist()})
    refined, delta, seconds = refine_angles(train[:4], rim_poses, ring, axis, rim_center, config)
    evaluate_poses("rim_center_refined", refined, seconds, {"rotation_correction_about_axis_deg": delta})
    methods = METHODS + (("go_icp",) if go_binary else ())
    for method in methods:
        global_init = method.startswith("fpfh") or method == "go_icp"
        accepted, details = [], []
        time = 0.
        for i, j in ring:
            init = IDENTITY if global_init else np.linalg.inv(nominal[j]) @ nominal[i]
            try:
                if method == "go_icp":
                    pose, sec = go_icp(np.asarray(models[i][0][-1].points),
                                       np.asarray(models[j][0][-1].points), go_binary,
                                       output / f"goicp_apple_{i}_{j}", config)
                else:
                    pose, sec = fit_pair(method, models[i], models[j], init, config,
                                         output / f"cc_apple_{i}_{j}", cc, config["seed"] + i)
                fit_cov = pair_metrics(np.asarray(models[i][0][-1].points), np.asarray(models[j][0][-1].points),
                                       pose, gates)["min_coverage_4"]
                edge = {"i": i, "j": j, "train_min_coverage_4": fit_cov,
                        "transform": pose, "seconds": sec}
                if fit_cov >= threshold:
                    accepted.append(edge)
                details.append({**edge, "transform": pose.tolist(), "accepted": fit_cov >= threshold})
                time += sec
            except (RuntimeError, subprocess.SubprocessError) as exc:
                details.append({"i": i, "j": j, "error": str(exc)})
        start = perf_counter()
        poses = optimize_graph(models, accepted)
        time += perf_counter() - start
        if poses is None:
            results[method] = {"state": "disconnected", "fit_details": details, "seconds": time}
            print(f"apple {method}: disconnected", flush=True)
        else:
            evaluate_poses(method, poses, time, details)
    return {"views": info, "axis": axis.tolist(), "axis_point": center.tolist(),
            "tag_sign_check_deg_view_2_to_3_approximate": tag_sign_check,
            "repeat_360_to_0": summary(distances(holdout, raw[0]), gates),
            "fixed_common_repeat_points_within_2_of_view_0": len(common),
            "methods": results}


def save_boxes(root, config, output, cc, go_binary, inputs):
    folder = root / config["box"]
    xml_path = next(folder.glob("calibration_result*.xml"))
    inputs.append(xml_path)
    storage = cv2.FileStorage(str(xml_path), cv2.FILE_STORAGE_READ)
    camera, coeffs = storage.getNode("cam_int").mat(), storage.getNode("cam_dist").mat()
    storage.release()
    ids = config["box_real_pair"]
    files = {i: next(folder.glob(f"capture_*_{i}*")) / "object_roi_experimental.ply" for i in config["box_scan_ids"]}
    inputs.extend(files.values())
    audit = {i: len(points(p)) for i, p in files.items()}
    xyz = [points(files[i]) for i in ids]
    images = []
    projected = []
    train, evaluation, gray = [], [], []
    for i, x, rect in zip(ids, xyz, config["box_label_rects_quarter_pixels"]):
        img_path = files[i].parent / "graycode_01.png"
        inputs.append(img_path)
        images.append(cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE))
        uv = cv2.projectPoints(x, np.zeros(3), np.zeros(3), camera, coeffs)[0].reshape(-1, 2)
        projected.append(uv)
        eligible = ~rectangle_mask(uv, rect)
        use = training_mask(uv[eligible])
        train.append(x[eligible][use])
        evaluation.append(x[eligible][~use])
        gray.append(sample_gray(images[-1], uv[eligible][use]))
    record = {"source_target": ids, "file_point_count": audit,
              "train": [len(x) for x in train], "reserved_tiles": [len(x) for x in evaluation]}
    init, label = box_image_reference(xyz, images, projected, config, camera, coeffs, record)
    models = [prepare_pair(x, config, intensity) for x, intensity in zip(train, gray)]
    metrics = {}

    reserved_label = np.arange(len(label[0])) % 2 == 0
    record["label_holdout_indices_for_image_method"] = np.flatnonzero(reserved_label).tolist()

    def log_result(method, pose, seconds):
        scores = pair_metrics(*evaluation, pose, config["evaluation_gates"])
        independent = np.linalg.norm(transformed(label[0], pose) - label[1], axis=1)
        metrics[method] = {**scores, "seconds": seconds, "transform": pose.tolist(),
                           "label_residual": summary(independent, config["evaluation_gates"]),
                           "image_method_holdout": summary(independent[reserved_label], config["evaluation_gates"])}
        merged, fixed = cloud(transformed(xyz[0], pose)), cloud(xyz[1])
        merged.paint_uniform_color(COLOR[0])
        fixed.paint_uniform_color(COLOR[1])
        merged += fixed
        o3d.io.write_point_cloud(str(output / f"box_{method}.ply"), merged)
        print(f"box {method}: held-out label median {np.median(independent):.3f}", flush=True)

    log_result("coarse_face", init, 0.)
    for i, method in enumerate(METHODS):
        start = IDENTITY if method.startswith("fpfh") else init
        try:
            pose, seconds = fit_pair(method, *models, start, config, output / "cc_box", cc, config["seed"] + i)
            log_result(method, pose, seconds)
        except (RuntimeError, subprocess.SubprocessError) as exc:
            metrics[method] = {"error": str(exc)}
            print(f"box {method}: {exc}", flush=True)
    seeds = []
    for i in range(config["ransac_extra_seeds_on_box"]):
        seed = config["seed"] + 510 + i
        try:
            pose, seconds = fit_pair("fpfh_ransac", *models, IDENTITY, config, None, None, seed)
            error = np.linalg.norm(transformed(label[0][reserved_label], pose) - label[1][reserved_label], axis=1)
            seeds.append({"seed": seed, "transform": pose.tolist(), "seconds": seconds,
                          "label_holdout": summary(error, config["evaluation_gates"])})
        except RuntimeError as exc:
            seeds.append({"seed": seed, "error": str(exc)})
    record["ransac_seed_check"] = seeds
    if go_binary:
        try:
            pose, sec = go_icp(np.asarray(models[0][0][-1].points), np.asarray(models[1][0][-1].points),
                               go_binary, output / "goicp_box", config)
            log_result("go_icp", pose, sec)
        except (RuntimeError, subprocess.SubprocessError) as exc:
            metrics["go_icp"] = {"error": str(exc)}
            print(f"box go_icp: {exc}", flush=True)
    start = perf_counter()
    fitted = rigid_alignment(label[0][~reserved_label], label[1][~reserved_label])
    log_result("sift_image", fitted, perf_counter() - start)
    metrics["sift_image"].pop("label_residual")
    record["methods"] = metrics
    return record, files


def known_pose(files, config):
    rows = []
    for shape, identifier in (("single_face", "124450"), ("two_faces", "124618")):
        pcd = cloud(points(files[identifier]), 1.0)
        whole = np.asarray(pcd.points).copy()
        whole -= whole.mean(axis=0)
        low, high = np.percentile(whole[:, 0], [19, 81])
        for n in range(config["known_pose_trials_per_shape"]):
            seed = config["seed"] + 100 + n
            rng = np.random.default_rng(seed)
            a = whole[whole[:, 0] >= low]
            b = whole[whole[:, 0] <= high]
            a = a[rng.permutation(len(a))[:int(.82 * len(a))]]
            b = b[rng.permutation(len(b))[:int(.82 * len(b))]]
            perturb = IDENTITY.copy()
            perturb[:3, :3] = Rotation.from_euler(
                "xyz", [rng.uniform(-7, 7), rng.uniform(22, 42) * (-1)**n, rng.uniform(-8, 8)],
                degrees=True).as_matrix()
            perturb[:3, 3] = rng.uniform(-8, 8, 3)
            truth = np.linalg.inv(perturb)
            a = transformed(a, perturb) + rng.normal(0, .10, a.shape)
            b = b + rng.normal(0, .10, b.shape)
            nearby = IDENTITY.copy()
            nearby[:3, :3] = Rotation.from_euler("xyz", [3., -5., 3.], degrees=True).as_matrix()
            nearby[:3, 3] = [3., -4., 1.]
            init = nearby @ truth
            models = [prepare_pair(x, config) for x in (a, b)]
            for method in ("initial", "point_to_plane", "point_to_point", "tukey_icp",
                           "gicp", "fpfh_ransac", "fpfh_fgr"):
                seconds = 0.
                try:
                    if method == "initial":
                        pose = init
                    else:
                        hint = IDENTITY if method.startswith("fpfh") else init
                        pose, seconds = fit_pair(method, *models, hint, config, None, None, seed)
                    difference = pose @ np.linalg.inv(truth)
                    angle = float(np.rad2deg(Rotation.from_matrix(difference[:3, :3]).magnitude()))
                    center_error = float(np.linalg.norm(difference[:3, 3]))
                    success = (angle <= config["known_pose_success_angle_deg"]
                               and center_error <= config["known_pose_success_center_distance"])
                    row = {"rotation_error_deg": angle, "center_error": center_error,
                           "success": success, "seconds": seconds}
                except RuntimeError as exc:
                    row = {"rotation_error_deg": "", "center_error": "", "success": False,
                           "seconds": "", "error": str(exc)}
                rows.append({"shape": shape, "trial": n + 1, "seed": seed, "method": method,
                             "error": "", **row})
        print(f"known pose {shape}: {config['known_pose_trials_per_shape']} trials", flush=True)
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("parameters", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--cloudcompare", type=Path)
    parser.add_argument("--go-icp", type=Path)
    args = parser.parse_args()
    cfg = json.loads(args.parameters.read_text())
    root, output = Path(cfg["root"]), args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)
    cv2.setNumThreads(2)
    inputs = [args.parameters.resolve(), Path(__file__).resolve(), Path(__file__).with_name("stitch_turntable.py").resolve()]
    start = perf_counter()
    apples = save_apples(root, cfg, output, args.cloudcompare, args.go_icp, inputs)
    json_dump(output / "apple.json", apples)
    boxes, files = save_boxes(root, cfg, output, args.cloudcompare, args.go_icp, inputs)
    json_dump(output / "box.json", boxes)
    rows = known_pose(files, cfg)
    with (output / "known_pose_trials.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    json_dump(output / "run.json", {"seconds": perf_counter() - start,
              "versions": {"Open3D": o3d.__version__, "OpenCV": cv2.__version__, "NumPy": np.__version__},
              "input_sha256": {str(p): sha256(p) for p in dict.fromkeys(inputs)}})


if __name__ == "__main__":
    main()
