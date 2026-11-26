"""Run one screened Poisson solve in its own process."""
from pathlib import Path
import sys

import numpy as np
import open3d as o3d

directory=Path(sys.argv[1])
with np.load(directory/"oriented.npz") as data:
    cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(data["points"]))
    cloud.normals=o3d.utility.Vector3dVector(data["normals"])
mesh,density=o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
    cloud,depth=int(sys.argv[2]),scale=1.09,n_threads=1)
if not len(mesh.triangles) or not o3d.io.write_triangle_mesh(str(directory/"surface.ply"),mesh):
    raise RuntimeError("Poisson returned an empty surface")
np.save(directory/"density.npy",np.asarray(density))
