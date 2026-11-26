"""Apply the cuboid completion prior to the 11/16 measured box as a scale-provisional demo."""
import json
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from run import OUT, points

ROOT=Path(__file__).resolve().parents[1]


def main():
    raw=ROOT/"algorithm/scene-fusion/results/clouds/box_1116"
    m=json.loads((raw/"poses_and_metrics.json").read_text())
    fits=m["reference"]["rectangular_face_test"]
    prefix=OUT/"real_box_1116_prior"
    prefix.mkdir(exist_ok=True)
    with np.load(raw/"object_evaluation_points.npz") as archive:
        scan=archive["rectangular_face"]
    n=np.array(m["axis"])
    base=np.array(m["center"])
    v=np.array([1.,0.,0.])-n[0]*n
    v/=np.linalg.norm(v)
    B=np.column_stack((v,np.cross(n,v)))
    angle=fits["reference_yaw_rad"]
    r=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
    X,Y=(B@r).T
    c=base+B@fits["rectangle_center_in_table_coords"]
    half=np.array(fits["half_widths_mm"])
    W,H=2*half
    height_samples=(scan-base)@n
    height_quantiles=np.percentile(height_samples,[97.,99.,99.7])
    h=float(height_quantiles[1])
    mesh=o3d.geometry.TriangleMesh.create_box(W,H,h)
    pose=np.eye(4)
    pose[:3,:3]=np.column_stack((X,Y,n))
    pose[:3,3]=c-X*half[0]-Y*half[1]
    mesh.transform(pose)
    assert mesh.is_watertight() and mesh.is_orientable()
    measured_keep=(height_samples>=-3)&(height_samples<=h+3)
    measured=scan[measured_keep]
    sample=np.array(mesh.sample_points_uniformly(17000).points)
    near=cKDTree(measured).query(sample)[0]<=2.5
    color=np.tile([.92,.47,.19],(len(sample),1))
    color[near]=[.26,.57,.76]
    points(prefix/"predicted_box_points.ply",sample,color)
    if not o3d.io.write_triangle_mesh(str(prefix/"predicted_box_mesh.ply"),mesh):
        raise OSError("Could not save cuboid")
    volume=float(mesh.get_volume())
    checks={"input":str(raw.relative_to(ROOT)/"object_evaluation_points.npz"),
        "source_scene":str(raw.relative_to(ROOT)/"full_scene.ply"),"frame":"11/16 reference camera",
        "units":"mm* provisional from 11/16 archive","dimensions":[float(W),float(H),h],
        "height_quantiles_97_99_99_7":height_quantiles.tolist(),"closed_box_volume":volume,
        "height_only_sensitivity_volume":[float(W*H*q) for q in (height_quantiles[0],height_quantiles[-1])],
        "supported_surface_fraction_at_2_5":float(near.mean()),"scale_validated":False,
        "assumptions":["rectangular box model","flat bottom on measured turntable","outer envelope volume",
                       "height estimated from observed endpoints of side faces"] }
    (prefix/"estimate.json").write_text(json.dumps(checks,indent=2,ensure_ascii=False)+"\n")
    print(checks)

if __name__=="__main__":
    o3d.utility.random.seed(915)
    main()
