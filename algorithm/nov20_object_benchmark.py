"""Reconstruct and benchmark November 20 object views, keeping the 360° scan out of fusion."""
from pathlib import Path
import argparse
import csv
import json
from time import perf_counter

import cv2
import numpy as np

from prepare_calibrated_benchmark import DATA, ROOT, SCANS, decode, load_calibration, load_frame, triangulate
from calibrate_benchmark_turntable import transform, project, image_plane
from calibrated_registration_benchmark import (
    CONFIG, adjacent_from, anchored_graph, cloud_sample, dump, loop_drift, pair_fit, pair_metric,
    paired_axis, reduce, rotvec, save_cloud, training_support,
)
from registration_experiment import fit_rim, polygon_mask, prepare_pair, sha256
from stitch_turntable import axis_transform

OUT = ROOT / "output/object-benchmarks"
CAL = DATA / "procam_calibration/20261120_analysis/calibration_10mm_experimental.xml"
AXIS = CAL.parent / "axis_approx.json"
OLD = ROOT / "output/registration-experiment/parameters.json"
METHODS = ("axis_step","rim_axis","axis_refine","point_to_point","point_to_plane",
           "tukey_icp","gicp","gray_icp","fpfh_ransac","fpfh_fgr","pose_graph")
MINION = {
    0:[118,72,191,151], 1:[119,71,190,152], 2:[114,73,179,156],
    4:[112,73,180,154], 5:[105,73,186,155], 6:[108,71,176,155],
    7:[114,72,185,154], 8:[113,72,187,155], 9:[118,72,191,151],
}
STRIP_MINION = {
    0:[85,106,123,141],4:[176,110,213,143],5:[190,131,214,171],
    6:[154,162,201,191],7:[97,160,156,190],8:[78,132,116,178],9:[85,106,123,141],
}
LEGACY_BOX = {0:[66,96,139,162],1:[82,96,119,166],2:[64,96,139,161],3:[84,96,120,166],4:[63,96,137,162]}
STRIP_APPLE = {2:[192,130,214,169],3:[108,150,165,184]}
EYE = np.eye(4)


def rectangle(uv, bounds, factor):
    a,b,c,d=np.array(bounds)*factor
    return (uv[:,0]>=a)&(uv[:,0]<c)&(uv[:,1]>=b)&(uv[:,1]<d)


def strip_center(image, bounds):
    a,b,c,d=np.array(bounds)*8
    cut=image[b:d,a:c]
    blur=cv2.GaussianBlur(cut,(5,5),0)
    threshold=np.percentile(blur,18)
    keep=blur<min(100,threshold+15)
    yy,xx=np.nonzero(keep)
    if len(xx)<40:
        return None
    return np.array([xx.mean()+a, yy.mean()+b])


def optical_views(key, scan, names, cal, axis, center, config, case):
    selected=[]
    observations={}
    inventory=[]
    rims=[]
    diag=[]
    for ix,name in enumerate(names):
        i=int(name[-1])
        v=scan/name
        start=perf_counter()
        uv, st, gray, lit=decode(v,camera_roi=(280,540,1900,1840))
        xyz,error=triangulate(cal,uv,st)
        dist=error.max(1)
        valid=np.isfinite(xyz).all(1)&(xyz[:,2]>100)&(xyz[:,2]<550)&(dist<8)
        h=(xyz-center)@axis
        img=load_frame(v,42)
        if key=="minion":
            mask=rectangle(uv,MINION[i],8)&(h>3)&(h<105)
            rect=STRIP_MINION.get(i)
        elif key=="legacy-box":
            mask=rectangle(uv,LEGACY_BOX[i],10) & (xyz[:,2]>275) & (xyz[:,2]<475)
            rect=None
        else:
            mask=polygon_mask(uv,config["apple_polygons_quarter_pixels"][i],img.shape)&(h>2)&(h<130)
            rect=STRIP_APPLE.get(i)
        use=valid&mask
        selected.append(cloud_sample(xyz[use],uv[use],gray[use]))
        if rect is not None:
            obs=strip_center(img,rect)
            if obs is not None:
                observations[ix]=obs
        try:
            if key=="legacy-box":
                raise RuntimeError("The archived rotation axis is camera vertical")
            rim=fit_rim(xyz[valid],uv[valid],-axis,center,{**config,"rim_plane_gate":10.})
            rims.append({"view":name,**rim})
        except RuntimeError:
            pass
        inventory.append({"frame":name,"illuminated":lit,"decoded":len(uv),"triangulated":int(valid.sum()),
                          "roi":len(selected[-1]["p"]),"train":len(selected[-1]["train"]),
                          "holdout":len(selected[-1]["val"]),"proj_p50":float(np.median(dist[valid])),
                          "proj_p95":float(np.percentile(dist[valid],95)),"seconds":perf_counter()-start})
        preview=cv2.cvtColor(cv2.resize(img,(306,256)),cv2.COLOR_GRAY2BGR)
        l=uv[use][::5]//8
        preview[l[:,1].astype(int),l[:,0].astype(int)]=(65,202,254)
        if ix in observations:
            cv2.circle(preview,np.round(observations[ix]/8).astype(int),3,(220,140,10),-1)
        cv2.putText(preview,name,(6,20),0,.51,(255,255,255),1,cv2.LINE_AA)
        diag.append(preview)
        np.savez_compressed(case/"single_views"/f"{name}.npz", xyz=xyz[use].astype(np.float32), uv=uv[use].astype(np.uint16), gray=gray[use])
        print(key,name,inventory[-1]["roi"],flush=True)
    while len(diag)%5:
        diag.append(np.zeros_like(diag[0]))
    cv2.imwrite(str(case/"input.jpg"),np.vstack([np.hstack(diag[i:i+5]) for i in range(0,len(diag),5)]))
    return selected, inventory, observations, rims


def image_markers(poses, observations, axis, center, cal):
    if len(observations)<2:
        return []
    view=min(observations)
    plane=EYE.copy()
    plane[:3,2]=axis
    plane[:3,3]=center
    xyz=image_plane(observations[view][None],plane,cal)
    base=transform(xyz,poses[view])
    checks=[]
    for i,uv in observations.items():
        if i==view:
            continue
        pred=project(base,np.linalg.inv(poses[i]),cal)[0]
        checks.append({"view":i,"actual":uv,"predicted":pred,"error_px":float(np.linalg.norm(uv-pred))})
    return checks


def configured_fit(method, prepared, views, priors, a,b):
    try:
        t=pair_fit(method,prepared,a,b,adjacent_from(priors,a,b),192+a*31+b)
        if training_support(views,a,b,t) >= .03:
            return t
    except RuntimeError:
        pass
    return None


def stress_test(prepared, views, nominal, axis, center):
    rows=[]
    for magnitude in (0,5,12,25):
        for sign in (-1,1) if magnitude else (1,):
            priors=nominal.copy()
            priors[1]=axis_transform(axis,center,magnitude*sign)@nominal[1]
            for method in ("point_to_point","point_to_plane","gicp","gray_icp"):
                pose=configured_fit(method,prepared,views,priors,1,0)
                rows.append({"method":method,"initial_yaw":sign*magnitude,"valid":pose is not None,
                             "f1_2":pair_metric(views[1],views[0],pose)["f1_2"] if pose is not None else 0.0})
    return rows


def run(key, scan):
    cv2.setNumThreads(2)
    case=OUT/key
    (case/"single_views").mkdir(parents=True,exist_ok=True)
    legacy=key=="legacy-box"
    calibration=DATA/"procam_calibration/calibration_20261116_10mm_experimental.xml" if legacy else CAL
    axis_file=scan/"axis_fit_box_metadata.json" if legacy else AXIS
    cal=load_calibration(calibration)
    axisinfo=json.loads(axis_file.read_text())
    if legacy:
        x,z=axisinfo["axis_xz_assumed_camera_vertical"]
        axis=np.array([0.,-1.,0.])
        center=np.array([x,0.,z])
    else:
        axis=-np.array(axisinfo["axis_direction"],float)
        axis/=np.linalg.norm(axis)
        center=np.array(axisinfo["axis_point_camera_mm"])
    session=json.loads((scan/"session.json").read_text())
    if key=="minion":
        angle_map=json.loads((scan/"corrected_nominal_angles.json").read_text())
    else:
        angle_map={x["folder"]:x["nominal_angle_degrees"] for x in session["views"]}
    names=list(angle_map)
    angles=np.array(list(angle_map.values()))
    if angles[-1]!=360 or any(not (scan/v).is_dir() for v in names):
        raise ValueError("The corrected view mapping and stored images disagree")
    config=json.loads(OLD.read_text())
    views,inventory,obs,rims=optical_views(key,scan,names,cal,axis,center,config,case)
    n=len(names)-1
    full=list(range(n))
    ring=[(i,(i+1)%n) for i in full]
    path=ring[:-1]
    # A physical camera-frame repeat from the same nominal view is never put into the cloud.
    loop=[(n-1,n)]
    base=np.array([axis_transform(axis,center,a) for a in angles])
    rim_centers=[p["point"] for p in rims if p["view"]!=names[-1]]
    corrected_center=np.median(rim_centers,axis=0) if rim_centers else None
    rim_pose=np.array([axis_transform(axis,corrected_center,a) for a in angles]) if corrected_center is not None else None
    prep={i:prepare_pair(v["train"],CONFIG,v["gray"]) for i,v in enumerate(views)}
    independent_repeat=pair_metric(views[n],views[0],EYE)
    # For raw comparisons, validate the direction by reporting the opposite model as well.
    sign_check={}
    for sign in (-1,1):
        pp=np.array([axis_transform(axis,center,sign*a) for a in angles])
        errs=image_markers(pp, {i:p for i,p in obs.items() if i<n},axis,center,cal)
        sign_check[sign]=reduce(errs,"error_px") if not legacy else float(np.mean(
            [training_support(views,a,b,adjacent_from(pp,a,b)) for a,b in ring]))
    if legacy:
        if sign_check[-1]>sign_check[1]:
            axis=-axis
            base=np.array([axis_transform(axis,center,a) for a in angles])
    elif sign_check[1] is None or sign_check[1]>sign_check[-1]:
        raise ValueError(f"Image marker comparison contradicts axis sign: {sign_check}")
    results=[]
    all_edges=[]
    methods=[m for m in METHODS if m!="rim_axis" or rim_pose is not None]
    for method in methods:
        t=perf_counter()
        poses=base.copy()
        attempts={}
        accepted=0
        fallback=0
        pair_fallback=False
        ring_closure=None
        if method=="rim_axis":
            poses=rim_pose.copy()
        elif method=="axis_refine":
            poses[:n]=paired_axis(axis,center,angles[:n],views,full,ring)
        elif method=="pose_graph":
            graph_edges=sorted(set(ring+[(a,(a+2)%n) for a in full if (a+2)%n!=a]))
            attempts={pair:configured_fit("point_to_plane",prep,views,base,*pair) for pair in graph_edges}
            poses[:n],accepted=anchored_graph(views,base[:n],attempts,center)
            attempts[(1,0)]=configured_fit("point_to_plane",prep,views,base,1,0)
            attempts[loop[0]]=configured_fit("point_to_plane",prep,views,base,*loop[0])
        elif method!="axis_step":
            attempts={pair:configured_fit(method,prep,views,base,*pair) for pair in sorted(set(ring+[(1,0)]+loop))}
            for a,b in path:
                pose=attempts[(a,b)]
                if pose is None:
                    fallback+=1
                else:
                    poses[b]=poses[a]@np.linalg.inv(pose)
        if method=="axis_refine":
            pair_pose=paired_axis(axis,center,angles[:n],views,[0,1],[(1,0)])[1]
        elif attempts:
            candidate=attempts.get((1,0))
            pair_fallback=candidate is None
            pair_pose=base[1] if pair_fallback else candidate
        else:
            pair_pose=poses[1]
        repeat_pose=None
        if attempts:
            close=attempts[loop[0]]
            if close is not None:
                repeat_pose=poses[n-1]@np.linalg.inv(close)
            ring_closure=loop_drift({(i,(i+1)%8):attempts.get(edge) for i,edge in enumerate(ring)},center) if n==8 else None
            if n!=8 and all(attempts.get(edge) is not None for edge in ring):
                p=EYE.copy()
                for edge in ring:
                    p=attempts[edge]@p
                ring_closure=(float(np.rad2deg(np.linalg.norm(rotvec(p)))),float(np.linalg.norm(transform(center[None],p)[0]-center)))
        runtime=perf_counter()-t
        pairs=pair_metric(views[1],views[0],pair_pose)
        ring_scores=[{"source":a,"target":b,**pair_metric(views[a],views[b],adjacent_from(poses,a,b))} for a,b in ring]
        markers=image_markers(poses[:n],{i:p for i,p in obs.items() if i<n},axis,center,cal)
        rep=pair_metric(views[n],views[0],repeat_pose) if repeat_pose is not None else {}
        row={"method":method,"frames":n,"pair_f1_2":pairs["f1_2"],"pair_fallback":pair_fallback,
             "marker_px":reduce(markers,"error_px"),"marker_p95_px":reduce(markers,"error_px",95),
             "repeat_f1_2":rep.get("f1_2"),"repeat_p95_mm":rep.get("p95"),
             "ring_closure_deg":ring_closure[0] if ring_closure else None,
             "ring_closure_axis_mm":ring_closure[1] if ring_closure else None,
             "worst_2":reduce(ring_scores,"f1_2",0),"seconds":runtime,
             "failures":sum(attempts.get(e) is None for e in ring) if attempts else 0,
             "fallback":fallback,"graph_links":accepted}
        for k in ("f1_1","f1_2","f1_4","rmse_2","median","p95","seam_p95"):
            row[k]=reduce(ring_scores,k)
        d=case/"runs"/method
        d.mkdir(parents=True,exist_ok=True)
        ortho=poses.copy()
        ortho[0]=EYE
        ortho[1]=pair_pose
        save_cloud(d/"pair_00_01.ply",views,ortho,[0,1])
        save_cloud(d/"all_views.ply",views,poses,full)
        dump(d/"poses_and_scores.json",{"summary":row,"poses":poses[:n],"pair_pose":pair_pose,"repeat_pose":repeat_pose,
              "pair":pairs,"ring":ring_scores,"marker_checks":markers,"repeat":rep,
              "independent_edges":{f"{a}-{b}":v for (a,b),v in attempts.items()}})
        results.append(row)
        for metric in ({"scope":"pair","source":1,"target":0,**pairs}, *({"scope":"ring",**x} for x in ring_scores),
                       {"scope":"repeat","source":n,"target":0,**rep}):
            all_edges.append({"method":method,**metric})
        print(key,method,round(row["f1_2"],4),round(row["marker_px"],1) if row["marker_px"] is not None else None,row["repeat_f1_2"],row["failures"],flush=True)
    for filename,rows in (("summary",results),("edge_metrics",all_edges),("view_inventory",inventory),
                          ("initial_pose_stress",stress_test(prep,views,base,axis,center))):
        fields=list(dict.fromkeys(k for r in rows for k in r))
        with (case/f"{filename}.csv").open("w",newline="") as f:
            wr=csv.DictWriter(f,fieldnames=fields)
            wr.writeheader()
            wr.writerows(rows)
    dump(case/"experiment.json",{"scan":str(scan.relative_to(ROOT)),"capture":session["created"],
        "calibration":str(calibration.relative_to(ROOT)),"calibration_hash":sha256(calibration),
        "rms_px":cal["rms"],"axis_file":str(axis_file.relative_to(ROOT)),"axis":axis,"center":center,
        "rim_center":corrected_center,"rims":rims,"frames":names,"angles":angles,
        "marker_sign_check":sign_check,"marker_centers":obs,"repeat_same_place":independent_repeat,"roi":inventory})


if __name__=="__main__":
    choices={"minion":"scan_20261120*小黄人","apple":"scan_20261120*苹果",
             "legacy-box":"scan_20261116_174014*"}
    p=argparse.ArgumentParser()
    p.add_argument("cases",nargs="*",default=list(choices))
    for key in p.parse_args().cases:
        run(key,next(SCANS.glob(choices[key])))
