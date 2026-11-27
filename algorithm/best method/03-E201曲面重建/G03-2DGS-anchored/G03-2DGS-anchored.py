"""G03 — Adjust measured points along their normals using 2DGS, then close with S08."""
from pathlib import Path
import sys

import numpy as np
import open3d as o3d

ENTRY = Path(__file__).resolve()
sys.path.insert(0, str(ENTRY.parents[1] / '源码'))

from 复跑 import HERE, gs_prior, precise_closure, run

STEP_MM = .55


def reconstruct(case, _archive):
    e201 = HERE / case['name']
    raw = o3d.io.read_triangle_mesh(str(e201 / 'G01-2DGS-TSDF.ply'))
    if not raw.has_triangles():
        raise ValueError(f'缺少 G01 的高斯 TSDF 网格：{e201}')
    prior, adjusted = gs_prior(raw, case)
    mesh = precise_closure(prior, case, STEP_MM)
    with np.load(e201 / '2D高斯盘.npz') as disks:
        points, colors = disks['p'], disks['rgb']
    return mesh, points, colors, {'step_mm': STEP_MM, 'anchored_splat_updates': adjusted}


if __name__ == '__main__':
    run(ENTRY, reconstruct)
