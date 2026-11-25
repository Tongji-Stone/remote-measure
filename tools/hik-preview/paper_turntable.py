"""按《基于旋转台参数标定实现多视角点云拼接》重建并拼接一组物体扫描。

论文用相隔一个步距的两次标靶位姿建立旋转关系，再用矩阵的整数次幂
推到其他视角。这里同样用平面标靶搭桥，另外把每帧能看到的角点一起
用于拟合一根公共轴。各视角标靶可被部分遮挡。标靶和物体都固定在转盘上。
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from automatic_marker_merge import calibration_path, read_calibration
from calibration_target import CalibrationTarget, add_target_arguments
from reconstruct_object import decode, image, save_ply, triangulate


PALETTE = np.uint8([[44, 115, 205], [232, 133, 45], [45, 160, 90], [195, 66, 60],
                   [147, 90, 184], [22, 162, 170], [190, 158, 45], [213, 100, 162]])


def write_json(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')


def matrix(R, t):
    """列向量记法的 4×4 齐次变换；t 是三维平移列。"""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).reshape(3)
    return T


def transform(points, T):
    """NumPy 中一行存一个点，因此列向量公式 R@p+t 写成 points@R.T+t。"""
    return points @ T[:3, :3].T + T[:3, 3]


def pnp(world, pixels, K, dist):
    """先剔除误认或偏离平面的像点，再用内点精化板到相机的位姿。"""
    if len(world) < 6:
        raise ValueError('可见标靶角点少于 6 个')
    # 去畸变后，同一块平板的二维坐标与相机像素应满足平面单应。
    # 在二维对应上做 RANSAC，再用专门处理平面标靶的 IPPE 求位姿。
    ideal = cv2.undistortPoints(pixels, K, dist, P=K)
    _, mask = cv2.findHomography(world[:, :2], ideal, cv2.RANSAC, 4.0)
    i = np.flatnonzero(mask) if mask is not None else np.empty(0, dtype=int)
    if len(i) < 6:
        raise ValueError('标靶位姿缺少足够的 PnP 内点')
    ok, rv, tv = cv2.solvePnP(world[i], pixels[i], K, dist, flags=cv2.SOLVEPNP_IPPE)
    if not ok or tv[2, 0] <= 0:
        raise ValueError('未得到相机前方的平面标靶位姿')
    rv, tv = cv2.solvePnPRefineLM(world[i], pixels[i], K, dist, rv, tv)
    return matrix(cv2.Rodrigues(rv)[0], tv), i


def fit_turntable(poses, observations, nominal_angles, K, dist):
    """从多帧 PnP 位姿拟合纯旋转轴，并用实际像素坐标细化。

    H_i 把第 i 帧的板坐标变换到相机。G_i = H_i @ inv(H_0) 表示盘面
    在相机坐标中的正向运动。设轴方向为 a、轴上一点为 c，
    则 G_i(p) = A_i @ (p-c) + c，平移项正好是 (I-A_i) @ c。
    以此把旋转和平移绑在一起，首帧角度固定为 0，轴向平移自由度去掉。
    """
    H0 = poses[0]
    relative = np.stack([H @ np.linalg.inv(H0) for H in poses])
    rotvecs = Rotation.from_matrix(relative[:, :3, :3]).as_rotvec()
    # 最大转角附近的帧对轴方向最敏感，避开接近 0° 或 180° 的帧。
    anchor = int(np.argmin(np.abs(np.abs(np.rad2deg(np.arctan2(
        np.sin(nominal_angles), np.cos(nominal_angles)))) - 90)))
    axis = rotvecs[anchor] / np.linalg.norm(rotvecs[anchor])
    if np.sign(axis @ rotvecs[anchor]) != np.sign(np.sin(nominal_angles[anchor])):
        axis = -axis
    # (I-A) 的秩为 2。以过首帧板原点并垂直于轴的平面固定轴点的自由度。
    basis = np.linalg.svd(axis[None, :])[2][1:].T
    t0 = H0[:3, 3]
    eye_minus_R = np.eye(3) - relative[:, :3, :3]
    xy = np.linalg.lstsq((eye_minus_R @ basis).reshape(-1, 2),
                         (relative[:, :3, 3] - eye_minus_R @ t0).ravel(), rcond=None)[0]
    unwrapped = rotvecs @ axis
    measured = unwrapped + 2 * np.pi * np.rint((nominal_angles - unwrapped) / (2 * np.pi))
    initial = np.r_[Rotation.from_matrix(H0[:3, :3]).as_rotvec(), t0, 0., 0., xy, measured[1:]]

    def unpack(x):
        direction = axis + basis @ x[6:8]
        direction /= np.linalg.norm(direction)
        center = t0 + basis @ x[8:10]
        angles = np.r_[0., x[10:]]
        rotations = Rotation.from_rotvec(angles[:, None] * direction).as_matrix()
        first = matrix(Rotation.from_rotvec(x[:3]).as_matrix(), x[3:6])
        motion = [matrix(R, center - R @ center) for R in rotations]
        return direction, center, angles, first, motion

    def residuals(x):
        _, _, _, first, motion = unpack(x)
        errors = []
        for (world, pixels), G in zip(observations, motion):
            H = G @ first
            prediction = cv2.projectPoints(world, cv2.Rodrigues(H[:3, :3])[0], H[:3, 3], K, dist)[0]
            errors.append((prediction - pixels).ravel())
        return np.concatenate(errors)

    fit = least_squares(residuals, initial, loss='soft_l1', f_scale=1.25, x_scale='jac', max_nfev=150)
    direction, center, angles, first, motion = unpack(fit.x)
    # 原点取旋转轴与首帧板面的交点，这样高度裁剪可以直接以板面为零。
    normal = first[:3, 2]
    center += direction * ((first[:3, 3] - center) @ normal) / (direction @ normal)
    return direction, center, angles, first, motion, anchor, fit


def icp_parts(parts, angle_list, voxel):
    """可选精配准。只在相邻视角的近重叠处做小幅点到面 ICP。

    各目标都使用转台粗配准后的点，不递推 ICP 修正，以免局部误匹配
    累积成整圈漂移。对称件和大平面容易滑移，限制修正量并记录是否采纳。
    """
    import open3d as o3d

    def cloud(points):
        result = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        result = result.voxel_down_sample(voxel)
        result.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=3 * voxel, max_nn=32))
        return result

    clouds = [cloud(points) for points in parts]
    output, records = [parts[0]], [dict(status='reference', correction=np.eye(4).tolist())]
    reg = o3d.pipelines.registration
    anchor = np.median(np.concatenate(parts), axis=0)
    for i, source in enumerate(clouds[1:], start=1):
        neighbors = [(i - 1) % len(parts), (i + 1) % len(parts)]
        neighbors = list(dict.fromkeys(j for j in neighbors
                                      if abs((angle_list[j] - angle_list[i] + 180) % 360 - 180) <= 65))
        target = o3d.geometry.PointCloud()
        for j in neighbors:
            target += clouds[j]
        C = np.eye(4)
        record = dict(status='insufficient_overlap', neighbors=neighbors)
        if len(source.points) >= 30 and len(target.points) >= 30:
            tolerance = max(2 * voxel, 2.0)
            before = reg.evaluate_registration(source, target, tolerance, C)
            if before.fitness >= .12:
                estimation = reg.TransformationEstimationPointToPlane(reg.TukeyLoss(tolerance))
                fitted = reg.registration_icp(source, target, tolerance, C, estimation,
                                              reg.ICPConvergenceCriteria(max_iteration=30))
                candidate = fitted.transformation
                shift = float(np.linalg.norm(transform(anchor, candidate) - anchor))
                rotation = float(np.degrees(Rotation.from_matrix(candidate[:3, :3]).magnitude()))
                accepted = (shift <= 3 and rotation <= 3 and fitted.fitness >= before.fitness * .98
                            and fitted.inlier_rmse < before.inlier_rmse)
                if accepted:
                    C = candidate
                record.update(status='accepted' if accepted else 'rejected',
                              fitness_before=before.fitness, fitness_after=fitted.fitness,
                              rmse_before_mm=before.inlier_rmse, rmse_after_mm=fitted.inlier_rmse,
                              center_shift_mm=shift, rotation_deg=rotation,
                              proposed_correction=candidate.tolist())
        output.append(transform(parts[i], C))
        records.append(dict(**record, correction=C.tolist()))
    return output, records


def depth_errors(xyz, model, camera_pixels, projector_pixels):
    """把三角化点重新投影，保存相机及投影面残差便于追查错码点。"""
    def error(rvec, tvec, K, distortion, observed):
        uv = cv2.projectPoints(xyz, rvec, tvec, K, distortion)[0].reshape(-1, 2)
        return np.linalg.norm(uv - observed, axis=1)

    return (error(np.zeros(3), np.zeros(3), model['cam_int'], model['cam_dist'], camera_pixels),
            error(cv2.Rodrigues(model['rotation'])[0], model['translation'], model['proj_int'],
                  model['proj_dist'], projector_pixels))


def alignment_metrics(parts, tolerance=3):
    """用 3 mm 门限描述相邻视角重叠；分母是两视角全部点。"""
    results = []
    for i, first in enumerate(parts):
        j = (i + 1) % len(parts)
        second = parts[j]
        if not len(first) or not len(second):
            continue
        distance = np.r_[cKDTree(first).query(second)[0], cKDTree(second).query(first)[0]]
        close = distance <= tolerance
        results.append(dict(views=[i, j], within_3mm_fraction=float(close.mean()),
                            overlap_rms_mm=float(np.sqrt(np.mean(distance[close] ** 2))) if close.any() else None))
    return results


def process(cal, scan, out, decoded_dirs, views, observations, args):
    out.mkdir(parents=True, exist_ok=True)
    model = read_calibration(cal)
    K, dist = model['cam_int'], model['cam_dist']
    poses, inlier_observations, detections = [], [], []
    for (world, uv, ids), view in zip(observations, views):
        try:
            H, indices = pnp(world, uv, K, dist)
        except ValueError as exc:
            raise ValueError(f"{view['folder']}: {exc}") from exc
        poses.append(H)
        inlier_observations.append((world[indices].astype(float), uv[indices].astype(float)))
        detections.append(dict(corner_ids=ids, world_mm=world, pixels=uv, pnp_inliers=indices, pnp_pose=H))
    nominal = np.deg2rad([v['nominal_angle_degrees'] for v in views])
    nominal -= nominal[0]
    axis, center, angles, H0, motion, anchor, fit = fit_turntable(poses, inlier_observations, nominal, K, dist)
    # 论文式 (4) 用板坐标中的相对变换。这里同时保存相机坐标中的
    # 正向运动 G 与其逆。第 i 帧点云已经在相机坐标，所以用 inverse(G_i)
    # 复位；需要首帧板坐标时，再乘 inverse(H_0)。
    pair = poses[1] @ np.linalg.inv(poses[0])
    pair_board = np.linalg.inv(poses[0]) @ poses[1]
    multiples = nominal / nominal[1] if abs(nominal[1]) > 1e-8 else None
    integer_steps = (np.rint(multiples).astype(int) if multiples is not None
                     and np.allclose(multiples, np.rint(multiples), atol=1e-5) else None)
    paper = dict(reference_view=views[0]['folder'], second_view=views[1]['folder'],
                 step_degrees=float(np.rad2deg(nominal[1])), board_relative=pair_board.tolist(),
                 camera_forward=pair.tolist(), camera_inverse=np.linalg.inv(pair).tolist())
    base_normal = H0[:3, 2].copy()
    if base_normal @ H0[:3, 3] > 0:
        base_normal *= -1
    details, parts, brightness_parts, pair_parts = [], [], [], []
    for v, data, G, raw, nominal_angle, theta in zip(views, detections, motion, decoded_dirs, nominal, angles):
        folder = out / v['folder']
        folder.mkdir(exist_ok=True)
        with np.load(raw) as decoded:
            xyz, front = triangulate(decoded, K, dist, model['proj_int'], model['proj_dist'],
                                     model['rotation'], model['translation'])
            camera_pixels = decoded['camera_pixels'][front]
            projector_pixels = decoded['projector_pixels'][front]
            brightness = decoded['brightness'][front]
        ec, ep = depth_errors(xyz, model, camera_pixels, projector_pixels)
        T = np.linalg.inv(G)
        aligned = transform(xyz, T)
        height = (aligned - H0[:3, 3]) @ base_normal
        delta = aligned - center
        radius = np.linalg.norm(delta - np.outer(delta @ axis, axis), axis=1)
        mask = ((height > args.clearance) & (height < args.max_height) & (radius < args.radius)
                & (ep < args.max_projector_error))
        part = aligned[mask]
        parts.append(part)
        brightness_parts.append(brightness[mask])
        save_ply(folder / '01_camera.ply', xyz, brightness)
        save_ply(folder / '02_aligned_full.ply', aligned, brightness)
        save_ply(folder / '03_object_coarse.ply', part, brightness[mask])
        H = G @ H0
        predicted = cv2.projectPoints(data['world_mm'], cv2.Rodrigues(H[:3, :3])[0], H[:3, 3], K, dist)[0]
        errors = np.linalg.norm((predicted - data['pixels']).reshape(-1, 2), axis=1)
        indices = data['pnp_inliers']
        # 在相同物体掩膜上复现论文两位姿法，方便单独看单步误差累积。
        # 当采集角度不是同一步距的整数倍时，仍使用上面的公共轴方法。
        extras = {}
        if integer_steps is not None:
            T_two_pose = np.linalg.matrix_power(pair, -int(integer_steps[len(parts) - 1]))
            extras['paper_two_pose_transform'] = T_two_pose
            pair_parts.append(transform(xyz[mask], T_two_pose))
        marker_sample = cKDTree(data['pixels'][indices, 0]).query(camera_pixels)[0] < 7
        marker_sample &= ep < args.max_projector_error
        np.savez_compressed(folder / 'stages.npz', **data, **extras, fit_pose=H, marker_residual_px=errors,
                            source_camera_pixels=camera_pixels, source_projector_pixels=projector_pixels,
                            camera_xyz_mm=xyz, aligned_xyz_mm=aligned.astype(np.float32), object_mask=mask,
                            camera_error_px=ec, projector_error_px=ep,
                            height_mm=height.astype(np.float32), radius_mm=radius.astype(np.float32),
                            coarse_transform=T, decoded_front_mask=front, board_probe_mask=marker_sample)
        details.append(dict(view=v['folder'], nominal_deg=float(np.rad2deg(nominal_angle)),
                            fitted_deg=float(np.rad2deg(theta)), pnp_inliers=len(indices),
                            detected_points=len(data['corner_ids']),
                            fit_reprojection_rms_px=float(np.sqrt(np.mean(errors[indices] ** 2))),
                            triangulated_points=len(xyz), object_points=len(part),
                            board_probe_points=int(marker_sample.sum()),
                            board_probe_median_height_mm=(float(np.median(height[marker_sample]))
                                                          if marker_sample.any() else None),
                            projector_reprojection_median_px=float(np.median(ep)),
                            transform_to_first=T.tolist()))
        print(f'{scan.name} {cal.stem} {v["folder"]} {len(xyz)} → {len(part)}', flush=True)

    used_angles = np.degrees(angles)
    if not sum(map(len, parts)):
        raise ValueError('裁剪后没有物体点，请检查盘面、裁剪半径和高度阈值')
    colors = np.concatenate([np.broadcast_to(PALETTE[i % len(PALETTE)], (len(p), 3))
                             for i, p in enumerate(parts)])
    save_ply(out / 'merged_coarse.ply', np.concatenate(parts), colors)
    summary = dict(source=str(scan.resolve()), calibration=str(cal.resolve()), reference_board_pose=H0.tolist(),
                   crop=dict(radius_mm=args.radius, clearance_mm=args.clearance, max_height_mm=args.max_height,
                             max_projector_error_px=args.max_projector_error),
                   axis_camera=axis.tolist(), axis_plane_intersection_camera_mm=center.tolist(),
                   board_up_camera=base_normal.tolist(), least_squares_success=bool(fit.success),
                   least_squares_evaluations=fit.nfev, axis_initialization_view=views[anchor]['folder'],
                   paper_two_pose=paper, views=details,
                   coarse_overlap=alignment_metrics(parts))
    if pair_parts:
        save_ply(out / 'merged_two_pose.ply', np.concatenate(pair_parts), colors)
        paper['overlap'] = alignment_metrics(pair_parts)
    if args.icp:
        refined, corrections = icp_parts(parts, used_angles, args.voxel)
        for detail, points, brightness, correction in zip(details, refined, brightness_parts, corrections):
            detail['icp'] = correction
            save_ply(out / detail['view'] / '04_object_icp.ply', points, brightness)
        save_ply(out / 'merged_icp.ply', np.concatenate(refined), colors)
        summary['icp_overlap'] = alignment_metrics(refined)
    write_json(out / 'registration.json', summary)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scan', type=Path)
    parser.add_argument('calibrations', nargs='+', type=Path, help='相机—投影仪标定 XML，可同时输入两份')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--step', type=int, default=4, help='相机采样像素间隔')
    parser.add_argument('--ignore-finest-bits', type=int, choices=(0, 1, 2), default=0)
    parser.add_argument('--radius', type=float, default=72, help='以轴为中心的圆柱裁剪半径（mm）')
    parser.add_argument('--clearance', type=float, default=2, help='切除盘面及其上方薄层（mm）')
    parser.add_argument('--max-height', type=float, default=150, help='物体最大高度（mm）')
    parser.add_argument('--max-projector-error', type=float, default=1.5, help='保留点的投影面重投影阈值（px）')
    parser.add_argument('--icp', action='store_true')
    parser.add_argument('--voxel', type=float, default=1.0, help='ICP 降采样体素（mm）')
    add_target_arguments(parser)
    parser.set_defaults(board='charuco')
    args = parser.parse_args()
    if args.step < 1 or min(args.radius, args.voxel, args.max_projector_error) <= 0 or args.max_height <= args.clearance:
        parser.error('请检查采样步长、裁剪范围和 ICP 体素大小')
    cv2.setNumThreads(3)
    target = CalibrationTarget(args)
    session = json.loads((args.scan / 'session.json').read_text(encoding='utf-8'))
    projector_size = tuple(session.get('projector_pixels', (1280, 720)))
    views = [v for v in session['views'] if v.get('status', 'complete') == 'complete']
    if len(views) < 3 or any(v.get('nominal_angle_degrees') is None for v in views):
        raise ValueError('轴拟合需要至少三帧带名义角度的完整视角')
    names = [p.stem for p in map(calibration_path, args.calibrations)]
    if len(names) != len(set(names)):
        raise ValueError('标定文件名称须彼此不同')
    shared = args.output / 'decoded'
    shared.mkdir(parents=True, exist_ok=True)
    settings = dict(source=str(args.scan.resolve()), step=args.step, bits=args.ignore_finest_bits,
                    projector_size_px=list(projector_size), target=target.spec)
    settings_path = shared / 'settings.json'
    if settings_path.exists() and json.loads(settings_path.read_text(encoding='utf-8')) != settings:
        raise ValueError('输出目录含不同扫描或解码设置，请指定新的 --output')
    write_json(settings_path, settings)
    decoded_dirs, observations = [], []
    for v in views:
        name = v['folder']
        folder = args.scan / name
        archive = shared / f'{name}.npz'
        if not archive.exists():
            decoded = decode(folder, projector_size, args.step, args.ignore_finest_bits)
            np.savez_compressed(archive.with_suffix('.tmp.npz'), **decoded)
            archive.with_suffix('.tmp.npz').replace(archive)
        decoded_dirs.append(archive)
        frame = image(folder / f'graycode_{session["pattern_count"] - 2:02d}.png')
        observations.append(target.observe(frame, markers=True))
        print(f'{args.scan.name} {name}: {len(observations[-1][2])} 个标靶点', flush=True)
    for requested in args.calibrations:
        cal = calibration_path(requested)
        process(cal, args.scan, args.output / cal.stem, decoded_dirs, views, observations, args)


if __name__ == '__main__':
    main()
