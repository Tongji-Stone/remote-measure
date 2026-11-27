"""Portable manifest and point-cloud I/O for cross-placement experiments."""
import json

import cv2 as cv
import numpy as np
import open3d as o3d


def image(path, mode):
    # Decode bytes so that Unicode paths also work with Windows builds of OpenCV.
    result = cv.imdecode(np.frombuffer(path.read_bytes(), dtype=np.uint8), mode)
    if result is None:
        raise ValueError(f'Invalid image: {path}')
    return result


def project(points, intrinsic, distortion):
    if not len(points):
        return np.empty((0, 2))
    return cv.projectPoints(points, np.zeros(3), np.zeros(3), intrinsic, distortion)[0][:, 0]


def transform(points, pose):
    return points @ pose[:3, :3].T + pose[:3, 3]


def rigid_matrix(value, name, *, permit_reflection=False):
    pose = np.asarray(value, dtype=float)
    if (pose.shape != (4, 4) or not np.isfinite(pose).all()
            or not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-7)):
        raise ValueError(f'{name} must be a finite homogeneous 4-by-4 matrix.')
    rotate = pose[:3, :3]
    det = np.linalg.det(rotate)
    if (not np.allclose(rotate.T @ rotate, np.eye(3), atol=1e-6)
            or not np.isclose(abs(det) if permit_reflection else det, 1., atol=1e-6)):
        raise ValueError(f'{name} must preserve distances and the specified axis convention.')
    return pose


def read_cloud(path):
    if not path.is_file():
        raise FileNotFoundError(path)
    point_cloud = o3d.io.read_point_cloud(str(path))
    points = np.asarray(point_cloud.points)
    if len(points) < 30 or not np.isfinite(points).all():
        raise ValueError(f'Need at least 30 finite measurements in {path}.')
    rgb = np.rint(255 * np.asarray(point_cloud.colors)).astype(np.uint8) if point_cloud.has_colors() else (
        np.full(points.shape, 145, dtype=np.uint8))
    return points, rgb


def write_cloud(path, points, rgb):
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    cloud.colors = o3d.utility.Vector3dVector(rgb / 255.)
    if not o3d.io.write_point_cloud(str(path), cloud):
        raise ValueError(f'Could not export {path}')


def read_camera(data):
    intrinsic = np.asarray(data['cam_int'], dtype=float)
    distortion = np.asarray(data['cam_dist'], dtype=float)
    size = tuple(data['shape'])
    if (intrinsic.shape != (3, 3) or len(size) != 2 or min(size) <= 0
            or not np.isfinite(intrinsic).all() or not np.isfinite(distortion).all()):
        raise ValueError('Camera requires cam_int, cam_dist and shape [height, width].')
    return intrinsic, distortion, size


def load_manifest(manifest):
    data = json.loads(manifest.read_text(encoding='utf-8'))
    base = manifest.parent
    camera = read_camera(data['camera']) if 'camera' in data else None
    color_camera = None
    if 'rgb_camera' in data:
        rgb = data['rgb_camera']
        rgb_intrinsic, rgb_distortion, _ = read_camera(rgb)
        color_camera = (rgb_intrinsic, rgb_distortion, rigid_matrix(rgb['hik_to_color'], 'hik_to_color'))
    placements = []
    for entry in data['placements']:
        points, colors = read_cloud(base / entry['ring'])
        board = rigid_matrix(entry['camera_v00_to_placement'], 'camera_v00_to_placement',
                             permit_reflection=True)
        views = []
        for station in entry.get('views', ()):
            matrix = rigid_matrix(station['station_to_v00'], 'station_to_v00')
            coords, _ = read_cloud(base / station['cloud'])
            photo = base / station['texture'] if 'texture' in station else None
            views.append(dict(xyz=coords, station_to_v00=matrix, texture=photo))
        if any(view['texture'] is not None for view in views) and color_camera is None:
            raise ValueError('Listed texture images require an rgb_camera calibration.')
        placements.append(dict(name=entry['name'], xyz=transform(points, board), rgb=colors,
                               B=board, views=views, flipped=bool(entry.get('flipped', False))))
    if len(placements) not in (2, 3):
        raise ValueError('This implementation expects two or three placements of the same object.')
    if len({p['name'] for p in placements}) != len(placements):
        raise ValueError('Every placement must have a distinct name.')
    if len({np.sign(np.linalg.det(p['B'][:3, :3])) for p in placements}) != 1:
        raise ValueError('Use the same handedness for every placement coordinate system.')
    if any(p['views'] for p in placements) and (camera is None or not all(p['views'] for p in placements)):
        raise ValueError('Depth fusion requires camera calibration and single-station views for every placement.')
    return placements, camera, color_camera
