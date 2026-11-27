"""Collect measured E201 metrics and render comparable previews from archived PLYs."""
import json
from pathlib import Path
import shutil
import sys

import cv2 as cv
import numpy as np
import open3d as o3d

from 复跑 import GROUPS, HERE, PYRAMIDS, REPO, ROOT, dump
from check_plate import plate_views
from common import acquisition, read_ply, tx

sys.path.insert(0, str(REPO / 'output' / 'final-report'))
from make_e2xx_figures import color_cloud, fit_mesh, view
from summarize import blend_image

S08 = 'S08-PrecisePoisson'
S09 = 'S09-ExposedPlate'
G03 = 'G03-2DGS-anchored'
WIDTH, HEIGHT = 450, 320
FACE = (28, 16)
BOTTOM = (34, -66)


def read_json(path):
    return json.loads(path.read_text())


def panel(image, label):
    out = cv.copyMakeBorder(image, 34, 4, 0, 0, cv.BORDER_CONSTANT, value=(250, 250, 250))
    cv.putText(out, label, (12, 23), cv.FONT_HERSHEY_SIMPLEX, .53, (36, 51, 57), 1, cv.LINE_AA)
    return out


def preview(mesh, cloud_face, center, below, volume, target):
    images = [
        panel(cloud_face, 'A   Input RGB point cloud'),
        panel(blend_image(mesh, center, FACE, WIDTH, HEIGHT), f'B   Reconstructed mesh  |  {volume:.2f} mL'),
        panel(blend_image(mesh, below, BOTTOM, WIDTH, HEIGHT), 'C   Reconstructed mesh, underside'),
    ]
    divider = np.full((images[0].shape[0], 7, 3), 250, dtype=np.uint8)
    canvas = np.hstack([images[0], divider, images[1], divider, images[2]])
    if not cv.imwrite(str(target), canvas):
        raise ValueError(target)


def entry(name, method, source, datum):
    folder = HERE / name
    record_path = folder / ('露板补充实验.json' if method == S09 else '精确闭合.json')
    report = read_json(record_path)
    model = folder / f'{method}.ply'
    plate_audit = read_json(folder / '露板与闭合核验.json')
    value = {
        'input_point_cloud': str(source.relative_to(REPO)),
        'geometry_source': str(model.relative_to(REPO)),
        'metrics_source': str(record_path.relative_to(REPO)),
        'plate_datum': datum,
        'mesh_metrics': report[method],
        'exposed_plate_check': plate_audit.get(method),
        'preview': f'{name}/{method}.png',
        'color_mesh': f'{name}/{method}.ply',
    }
    coarse = f'{method.split("-", 1)[0]}-coarse'
    if coarse in report:
        value['coarse_grid_check'] = report[coarse]
    if method == G03:
        value['fitted_2dgs'] = read_json(folder / '高斯方法.json')
        value['gaussian_disc_input'] = str((folder / '2D高斯盘.npz').relative_to(REPO))
        value['gaussian_tsdf_input'] = str((folder / 'G01-2DGS-TSDF.ply').relative_to(REPO))
    if method == S09:
        value['exposed_board_evidence'] = f'{name}/同期露板证据.npz'
    return value


def collect():
    by_method = {method: {} for method in (S08, S09, G03)}
    controls = read_json(HERE / '已知体积对照.json')
    for name in GROUPS:
        output = HERE / name
        methods = (S08, S09, G03) if name in PYRAMIDS else (S08, G03)
        paths = [output / f'{method}.ply' for method in methods]
        case, source = acquisition(name)
        datum = read_json(output / '底面标定.json')
        fit = np.asarray(datum['world_to_plate_mm'])
        cloud = read_ply(source)
        points = tx(cloud['xyz'], fit @ case['scans'][0]['B'])
        center = fit_mesh(paths, FACE, WIDTH, HEIGHT, margin=1.19)
        below = fit_mesh(paths, BOTTOM, WIDTH, HEIGHT, margin=1.19)
        camera = view(*FACE)
        cloud_face = color_cloud(points, cloud['rgb'], FACE,
                                 limit=(center[0] @ camera, center[1]), width=WIDTH, height=HEIGHT)
        for method, model in zip(methods, paths):
            folder = ROOT / method / name
            folder.mkdir(parents=True, exist_ok=True)
            data = entry(name, method, source, datum)
            mesh = o3d.io.read_triangle_mesh(str(model))
            if not (mesh.has_triangles() and mesh.has_vertex_colors()):
                raise ValueError(f'缺少彩色三角网格：{model}')
            shutil.copy2(model, folder / model.name)
            preview(mesh, cloud_face, center, below, data['mesh_metrics']['volume_ml'],
                    folder / f'{method}.png')
            if method == S09:
                witnesses = folder / '同期露板证据.npz'
                if not witnesses.exists():
                    views = {f'v{i:02}': points for i, points in enumerate(plate_views(name))}
                    np.savez_compressed(witnesses, **views)
            by_method[method][name] = data
            print(name, method, data['mesh_metrics']['volume_ml'], flush=True)

    for method, objects in by_method.items():
        record = {'experiment': 'simulation/E201', 'method': method, 'volume_unit': 'mL',
                  'coordinate_system': 'measured plate coordinate system, millimetres',
                  'objects': objects}
        comparisons = {name: {'true_volume_ml': row['exact_ml'], **row['methods'][method]}
                       for name, row in controls.items() if method in row['methods']}
        if comparisons:
            record['known_volume_controls'] = comparisons
        dump(ROOT / method / f'{method}.json', record)


if __name__ == '__main__':
    cv.setNumThreads(1)
    collect()
