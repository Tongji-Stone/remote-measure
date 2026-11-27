"""Run one archived E201 reconstruction using the original project implementation."""
import argparse
from pathlib import Path
import sys

import cv2 as cv
import numpy as np
import open3d as o3d

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[2]
sys.path.insert(0, str(REPO / 'simulation' / 'E201'))

from common import GROUPS, HERE, PYRAMIDS, audit, dump, measured, save_mesh
from plate_closure import gs_prior, precise_closure
from reconstruct import screened


def poisson(case):
    return screened(case['p'], case['n'])[0]


def run(entry, reconstruct, objects=GROUPS):
    method = entry.stem
    parser = argparse.ArgumentParser(description=f'复跑 Simulation E201 的 {method}')
    parser.add_argument('--object', choices=objects, required=True, help='E201 采集对象')
    parser.add_argument('--output', type=Path, required=True, help='复跑 PLY 与指标 JSON 的输出目录')
    options = parser.parse_args()

    folder = options.output.expanduser().resolve()
    ply, metrics = (folder / f'{method}.{extension}' for extension in ('ply', 'json'))
    if ply.exists() or metrics.exists():
        parser.error(f'输出已存在，请指定新的输出目录：{folder}')

    cv.setNumThreads(1)
    name = options.object
    case = measured(name)
    mesh, points, colors, details = reconstruct(case, entry.parent / name)
    if not mesh.has_triangles():
        raise RuntimeError(f'{name} 的 {method} 未生成三角面。')

    result = {
        'method': method, 'object': name, 'input_point_cloud': case['source'],
        'datum': case['datum'], **details, **audit(mesh, case),
    }
    folder.mkdir(parents=True, exist_ok=True)
    save_mesh(ply, mesh, points, colors, neutral_missing=name in PYRAMIDS)
    dump(metrics, result)
    print(f'{name} | {method} | {result["volume_ml"]} mL')
