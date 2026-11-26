"""Isolate the optional C++ Poisson reconstruction from the benchmark process."""
import json
from pathlib import Path
import sys

import open3d as o3d

path, target, depth = sys.argv[1:]
cloud = o3d.io.read_point_cloud(path)
mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(cloud, depth=int(depth), n_threads=1)
closed = bool(mesh.is_watertight())
volume = abs(mesh.get_volume()) if closed and mesh.is_orientable() else None
if not o3d.io.write_triangle_mesh(target, mesh):
    raise OSError(target)
print(json.dumps({"watertight": closed, "volume": volume, "depth": int(depth)}, allow_nan=False))
