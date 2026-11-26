"""Verify complete scene exports and collect evidence from the archived calibration runs."""
import csv
import hashlib

import numpy as np
import open3d as o3d

from optics import CACHE, OUT, ROOT, DATA, CAL16, CAL20, CALIBRATION, load, dump, load_calibration, transform


def digest(p):
    with p.open("rb") as f: return hashlib.file_digest(f,"sha256").hexdigest()


def vertices(path):
    pc=o3d.io.read_point_cloud(str(path))
    return np.asarray(pc.points).copy()


def verify_clouds():
    audit=load(OUT/"optical_inputs.json")
    sessions={}
    for key,meta in audit.items():
        folder=OUT/"clouds"/key
        model=load(folder/"poses_and_metrics.json")
        pose=np.array(model["poses"])
        assert np.isfinite(pose).all()
        assert np.allclose(np.linalg.det(pose[:,:3,:3]),1.,atol=1e-8)
        assert np.allclose(pose[:,:3,:3]@pose[:,:3,:3].transpose(0,2,1),np.eye(3),atol=1e-8)
        stored=vertices(folder/"full_scene.ply")
        cursor=0
        for i,(v,p) in enumerate(zip(meta["views"],pose,strict=True)):
            with np.load(CACHE/key/f"{i:02}.npz") as d: xyz=d["xyz"]
            repeat=i==model["repeat_index"]
            name=v["frame"]+("_360deg_repeat" if repeat else "")
            one=vertices(folder/"single"/f"{name}.ply")
            assert np.array_equal(xyz,one) and np.isfinite(one).all(), (key,name)
            if repeat: continue
            expected=transform(xyz,p)
            assert np.array_equal(stored[cursor:cursor+len(xyz)],expected), (key,name)
            cursor+=len(xyz)
        count=len(stored)
        assert count==cursor==model["fused_count"]
        sessions[key]={"full_scene_vertices":count,"single_views":len(meta["views"]),
                       "fused_unique_views":len(pose)-int(model["repeat_index"] is not None),
                       "full_scene_sha256":digest(folder/"full_scene.ply")}
    return sessions


def archive_evidence():
    raw=load(OUT/"optical_inputs.json")
    evidence={}
    parameters=[CAL16,CAL20,CALIBRATION,DATA/"procam_calibration/calibration_20261116_split_train_experimental.xml"]
    for p in parameters:
        m=load_calibration(p)
        evidence[str(p.relative_to(ROOT))]={"sha256":digest(p),"legacy_stereo_rms":m["rms"],
            "focal_px":m["cam_int"].diagonal()[:2],"baseline_nominal_mm":float(np.linalg.norm(m["translation"]))}
    for file in (DATA/"procam_calibration/calibrate_20261116_temp.py",CAL20.parent/"graycode_validity.json",CAL20.parent/"axis_approx.json"):
        evidence[str(file.relative_to(ROOT))]={"sha256":digest(file)}
    runs={}
    for key,metadata,ply in (
        ("minion","merged_ignore0_corrected_uncropped_metadata.json","merged_ignore0_corrected_uncropped_experimental.ply"),
        ("apple","merged_ignore0_uncropped_metadata.json","merged_ignore0_uncropped_experimental.ply"),
        ("box_1116","axis_fit_box_metadata.json","axis_fit_box_experimental.ply")):
        scan=ROOT/raw[key]["path"]
        m=load(scan/metadata)
        cloud=vertices(scan/ply)
        runs[key]={"archive":str((scan/ply).relative_to(ROOT)),"metadata":str((scan/metadata).relative_to(ROOT)),
                   "metadata_sha256":digest(scan/metadata),"archive_vertices":len(cloud),"metadata_point_count":m.get("points_total"),
                   "archive_repeat_median_nearest_mm":m.get("repeat_view_median_nearest_mm"),
                   "same_pixel_sampling_assumed":False}
    scan=ROOT/raw["charger"]["path"]
    prior=[]
    for p in sorted(scan.glob("aruco_reconstruction*/registration.json")):
        original=load(p)
        prior.append({"file":str(p.relative_to(ROOT)),"sha256":digest(p),"warnings":original.get("quality_warnings",original.get("warnings")),
                      "marker_dictionary":original.get("marker_dictionary"),"view_points":original.get("point_counts")})
    runs["charger"]=prior
    return {"calibration_files":evidence,"original_results":runs}


if __name__=="__main__":
    model={"archive_review":archive_evidence(),"complete_export_verification":verify_clouds()}
    dump(OUT/"archive_and_export_audit.json",model)
    sessions=model["complete_export_verification"]
    print("Verified unique fusion",sum(r["fused_unique_views"] for r in sessions.values()),
          "single frames",sum(r["single_views"] for r in sessions.values()),
          "vertices",sum(r["full_scene_vertices"] for r in sessions.values()))
