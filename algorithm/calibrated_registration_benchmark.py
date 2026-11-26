"""Compare turntable calibration and point-cloud registration on held-out pixels."""
import csv
import json
import platform
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np
import open3d as o3d
from scipy.optimize import least_squares, minimize_scalar
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from calibrate_benchmark_turntable import OUTPUT, ROOT, project, roi_mask, transform
from prepare_calibrated_benchmark import CALIBRATION, load_calibration
from registration_experiment import REG, fit_pair, prepare_pair, training_mask
from stitch_turntable import axis_transform

METHODS = ("axis_step","aruco","axis_refine","point_to_point","point_to_plane",
           "tukey_icp","gicp","gray_icp","fpfh_ransac","fpfh_fgr","pose_graph")
LOCAL = METHODS[3:-1]
LABELS = {"171640":"充电头 · 棋盘格","174129":"充电头 · ArUco", "174531":"包装盒 · 居中",
          "174747":"包装盒 · 偏心", "175006":"包装盒 · 17:50"}
COLORS = np.array([[46,111,196],[236,106,53],[36,158,116],[156,90,172],
                   [209,158,50],[27,157,181],[216,83,137],[100,113,132]],np.uint8)
CONFIG = {"training_voxels":[2.4,1.25,.75],"training_gates":[8.,4.,2.5],
          "fpfh_voxel":2.2,"fpfh_ransac_iterations":24000}
RAD = (1.,2.,4.)
RING = [(i,(i+1)%8) for i in range(8)]
IDENTITY = np.eye(4)


def dump(path,obj):
    path.write_text(json.dumps(obj,ensure_ascii=False,indent=2,allow_nan=False,
                               default=lambda x:x.tolist() if isinstance(x,np.ndarray) else x.item())+"\n")


def pose(x):
    T=np.eye(4)
    T[:3,:3]=Rotation.from_rotvec(x[:3]).as_matrix()
    T[:3,3]=x[3:]
    return T


def rotvec(T):
    return Rotation.from_matrix(T[:3,:3]).as_rotvec()


def adjacent_from(poses, a, b):
    return np.linalg.inv(poses[b])@poses[a]


def cloud_sample(p, uv, gray):
    correct=np.isfinite(p).all(1)
    p,uv,gray=p[correct],uv[correct],gray[correct]
    if len(p)>50:
        spacing = cKDTree(p).query(p,k=9)[0][:,-1]
        keep = spacing <= max(1.8,float(np.percentile(spacing,94)))
        p,uv,gray=p[keep],uv[keep],gray[keep]
    tr=training_mask(uv)
    val=p[~tr]
    normals_pcd=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(val))
    if len(val)>=10:
        normals_pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=3,max_nn=26))
    return {"p":p, "train":p[tr], "val":val, "gray":gray[tr].astype(float)/255,
            "normals":np.asarray(normals_pcd.normals).copy()}


def get_views(sid):
    views=[]
    for i in range(8):
        a=np.load(OUTPUT/"recovery"/sid/f"{i:02}.npz")
        mask=a["obj"]
        views.append(cloud_sample(a["xyz"][mask],a["uv"][mask],a["gray"][mask]))
    return views


def pair_metric(a,b,T):
    if min(len(a["val"]),len(b["val"]))<20:
        return {"status":"insufficient_validation"}
    A, B=transform(a["val"],T), b["val"]
    da, ia=cKDTree(B).query(A)
    db, ib=cKDTree(A).query(B)
    d=np.r_[da,db]
    z={"status":"scored","source_eval":len(A),"target_eval":len(B)}
    for gate in RAD:
        x,y=(da<=gate).mean(),(db<=gate).mean()
        z.update({f"forward_{gate:g}":float(x),f"backward_{gate:g}":float(y),
                  f"f1_{gate:g}":float(2*x*y/(x+y)) if x+y else 0.})
    m=(d<=2)
    z.update(rmse_2=float(np.sqrt(np.mean(d[m]**2))) if m.any() else None, inliers_2=int(m.sum()),
             median=float(np.median(d)),p95=float(np.percentile(d,95)),
             nn_mean=float(.5*(da.mean()+db.mean())))
    mutual=(ib[ia]==np.arange(len(A)))&(da<=4)
    z["reciprocal_n"]=int(mutual.sum())
    z["seam_p95"]=float(np.percentile(np.abs(np.sum((A[mutual]-B[ia[mutual]])*b["normals"][ia[mutual]],axis=1)),95)) if mutual.sum()>=12 else None
    return z


def pair_fit(method, prep, a,b, ini, seed):
    result, _=fit_pair(method,prep[a],prep[b],ini,CONFIG,None,None,seed)
    return result


def paired_axis(axis,center,angles,views,active,edges):
    sampled={i:views[i]["train"][::max(1,len(views[i]["train"])//1000)] for i in active}
    def objective(delta, i, target_poses):
        xyz=transform(sampled[i],axis_transform(axis,center,angles[i]+delta))
        residual=[]
        for j, other in target_poses:
            forward=cKDTree(other).query(xyz)[0]
            reverse=cKDTree(xyz).query(other)[0]
            for distances in (forward,reverse):
                q=np.quantile(distances,.65)
                good=distances<=q
                residual.append(np.minimum(distances[good]**2,64).mean())
        return np.mean(residual)+.012*delta**2 if residual else 0.
    deltas=np.zeros(8)
    for _ in range(3):
        for i in active:
            if not i:
                continue
            js=[b if a==i else a for a,b in edges if i in (a,b)]
            targets=[(j,transform(sampled[j],axis_transform(axis,center,angles[j]+deltas[j]))) for j in js]
            if targets:
                grid=np.arange(-16,17,4)
                best=min(grid,key=lambda d:objective(d,i,targets))
                sol=minimize_scalar(lambda d:objective(d,i,targets),bounds=(max(-18,best-4),min(18,best+4)),method="bounded",
                                    options={"xatol":.12})
                deltas[i]=sol.x
    return np.array([axis_transform(axis,center,a+d) for a,d in zip(angles,deltas)])


def training_support(views,a,b,T):
    A=transform(views[a]["train"][::3],T); B=views[b]["train"][::3]
    da=cKDTree(B).query(A)[0]
    db=cKDTree(A).query(B)[0]
    return min((da<4).mean(),(db<4).mean())


def anchored_graph(views, priors, corrections, pivot):
    n=len(priors)
    links=[]
    for (a,b), T in corrections.items():
        if T is None:
            continue
        ini=adjacent_from(priors,a,b)
        anchor=views[a]["train"].mean(0)
        shift=np.linalg.norm(transform(anchor[None],T)[0]-transform(anchor[None],ini)[0])
        angle=np.linalg.norm(rotvec(T@np.linalg.inv(ini)))*180/np.pi
        support=training_support(views,a,b,T)
        if support>=.16 and shift<12 and angle<15:
            links.append((a,b,T, np.sqrt(support)*2,anchor))
    def adjustments(x):
        output = [priors[0]]
        for v,p in zip(x.reshape(-1,6),priors[1:]):
            correction = pose(np.r_[v[:3]/55,v[3:]])
            correction[:3,3] += pivot - correction[:3,:3] @ pivot
            output.append(correction @ p)
        return output
    def resid(x):
        ps=adjustments(x)
        parts=[x*2.5]
        for a,b,t,w,center in links:
            m=adjacent_from(ps,a,b)
            error=np.r_[rotvec(np.linalg.inv(t)@m)*40,(transform(center[None],t)-transform(center[None],m))[0]]
            parts.append(error*w)
        return np.concatenate(parts)
    limit=np.tile([1.15,1.15,1.15,1.,1.,1.], n-1)
    solved=least_squares(resid,np.zeros((n-1)*6),bounds=(-limit,limit),loss="soft_l1",f_scale=2,max_nfev=24)
    return np.array(adjustments(solved.x)), len(links)


def save_cloud(path, views, transforms, active):
    ps=np.concatenate([transform(views[i]["p"],transforms[i]) for i in active])
    cols=np.concatenate([np.broadcast_to(COLORS[i],(len(views[i]["p"]),3)) for i in active])
    pcd=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(ps))
    pcd.colors=o3d.utility.Vector3dVector(cols/255)
    if not o3d.io.write_point_cloud(str(path),pcd):
        raise IOError(path)


def marker_metrics(model, poses, cal, views):
    values=[]
    m=np.array(model["prior"])
    for s, uv in model["heldout_image"].items():
        i=int(s)
        if i not in views:
            continue
        xyz=np.asarray(model["holdout_xyz"])
        reprojection=project(xyz,np.linalg.inv(poses[i]),cal)
        p=np.linalg.norm(reprojection-np.array(uv),axis=1)
        reference=np.linalg.inv(m[i])
        prediction=np.linalg.inv(poses[i])
        displacement=np.linalg.norm(transform(xyz,reference)-transform(xyz,prediction),axis=1)
        values.append({"view":i,"pixels":p,"delta_deg":np.linalg.norm(rotvec(prediction@np.linalg.inv(reference)))*180/np.pi,
                       "tag_displacement":displacement})
    return values


def reduce(rows,key,percentile=50):
    values=[r[key] for r in rows if key in r and r[key] is not None]
    return float(np.percentile(values,percentile)) if values else None


def loop_drift(estimates,center):
    P=IDENTITY.copy()
    for key in RING:
        if estimates.get(key) is None:
            return None,None
        P=estimates[key]@P
    return float(np.linalg.norm(rotvec(P))*180/np.pi), float(np.linalg.norm(transform(center[None],P)[0]-center))


def main():
    cv2.setNumThreads(2)
    cal=load_calibration(CALIBRATION)
    models=json.loads((OUTPUT/"recovery/model.json").read_text())
    run=OUTPUT/"runs"
    run.mkdir(exist_ok=True)
    edge_rows,metric_rows,view_rows,stress_rows=[],[],[],[]
    for sid, m in models.items():
        print("Case",sid,flush=True)
        views=get_views(sid)
        active=[i for i,v in enumerate(views) if len(v["train"])>=80 and len(v["val"])>=20]
        if len(active)!=8:
            print("Sparse",sid,active,flush=True)
        for i,v in enumerate(views):
            view_rows.append(dict(session=sid,view=i,retained=len(v["p"]),train=len(v["train"]),evaluation=len(v["val"]),active=i in active))
        assert 0 in active and 1 in active
        angles=np.arange(8)*45.
        axis,center=np.asarray(m["axis"]),np.asarray(m["center"])
        prior=np.asarray(m["prior"])
        step=np.array([axis_transform(axis,center,a) for a in angles])
        prep={i:prepare_pair(views[i]["train"],CONFIG,views[i]["gray"]) for i in active}
        graph_edges={(a,b) for a,b in RING if a in active and b in active}
        graph_edges|={(i,(i+2)%8) for i in active if (i+2)%8 in active}
        path=list(zip(active[:-1],active[1:]))
        pair=(1,0)
        evaluation=[(a,b) for a,b in RING if a in active and b in active]
        root=run/sid
        root.mkdir(exist_ok=True)
        previous={}
        series={}
        for number, method in enumerate(METHODS):
            t=perf_counter()
            trials={}; failures=[]; fallback=0; links=0; pair_fallback=False
            if method in ("axis_step","aruco"):
                poses=step if method=="axis_step" else prior
                pair_pose=poses[1]
            elif method=="axis_refine":
                poses=paired_axis(axis,center,angles,views,active,evaluation)
                pair_pose=paired_axis(axis,center,angles,views,[0,1],[pair])[1]
            elif method=="pose_graph":
                for a,b in sorted(graph_edges):
                    ini=adjacent_from(prior,a,b)
                    try:
                        trials[a,b]=pair_fit("point_to_plane",prep,a,b,ini,928+a*8+b)
                    except RuntimeError:
                        trials[a,b]=None
                poses,links=anchored_graph(views,prior,trials,center)
                pair_pose=previous[pair] if previous[pair] is not None else prior[1]
                pair_fallback=previous[pair] is None
            else:
                for a,b in sorted(set(evaluation+path+[pair])):
                    ini=adjacent_from(prior,a,b)
                    try:
                        T=pair_fit(method,prep,a,b,ini,928+a*8+b)
                        if training_support(views,a,b,T) < .03:
                            raise RuntimeError("Pair produced under 3% bidirectional support at 4 mm")
                        trials[a,b]=T
                    except RuntimeError as e:
                        trials[a,b]=None
                        failures.append({"edge":[a,b],"message":str(e).strip().splitlines()[-1]})
                poses=prior.copy()
                for a,b in path:
                    edge=trials[a,b]
                    if edge is None:
                        fallback+=1
                    else:
                        poses[b]=poses[a]@np.linalg.inv(edge)
                pair_fallback=trials[pair] is None
                pair_pose=prior[1] if pair_fallback else trials[pair]
                if method=="point_to_plane":
                    previous=trials
            seconds=perf_counter()-t
            d=root/method
            d.mkdir(exist_ok=True)
            pp=np.repeat(IDENTITY[None],8,axis=0)
            pp[1]=pair_pose
            save_cloud(d/"pair_00_01.ply",views,pp,[0,1])
            save_cloud(d/"all_views.ply",views,poses,active)
            pair_result=pair_metric(views[1],views[0],pair_pose)
            pair_result.update(session=sid,method=method,scope="pair",source=1,target=0)
            edge_rows.append(pair_result)
            rowset=[]
            for a,b in evaluation:
                z=pair_metric(views[a],views[b],adjacent_from(poses,a,b))
                z.update(session=sid,method=method,scope="full",source=a,target=b)
                rowset.append(z)
                edge_rows.append(z)
            markers=marker_metrics(m,poses,cal,active)
            mark=np.concatenate([e["pixels"] for e in markers])
            shift=np.concatenate([e["tag_displacement"] for e in markers])
            closure,closure_mm=loop_drift(trials,center)
            row={"session":sid,"method":method,"views":len(active),"edges":len(rowset),"valid_marker_views":len(markers),
                 "tag_p50_px":float(np.median(mark)),"tag_p95_px":float(np.percentile(mark,95)),
                 "marker_shift_p50_mm":float(np.median(shift)), "rot_vs_marker_p50_deg":reduce(markers,"delta_deg"),
                 "pair_f1_2":pair_result["f1_2"],"pair_rmse_2_mm":pair_result["rmse_2"],
                 "seconds":seconds,"closure_deg":closure,"closure_at_axis_mm":closure_mm,
                 "edge_failures":len(failures),"pose_fallback":fallback,"pair_fallback":any(f["edge"]==[1,0] for f in failures),"graph_links":links}
            for key in ("f1_1","f1_2","f1_4","rmse_2","median","p95","seam_p95","nn_mean"):
                row[key]=reduce(rowset,key)
            row["f1_2_worst"]=reduce(rowset,"f1_2",0)
            row["f1_2_edge_p10"]=reduce(rowset,"f1_2",10)
            row["seam_available"]=sum(e["seam_p95"] is not None for e in rowset)
            metric_rows.append(row)
            series[method]=row
            dump(d/"poses_and_scores.json",{"poses":poses,"pair_pose":pair_pose,"active":active,"pair":pair_result,
                                            "markers":markers,"full_edges":rowset,"independent_edges":{f"{a}-{b}":v for (a,b),v in trials.items()},
                                            "fallbacks":failures,"summary":row})
            print(method,round(row["f1_2"],3),round(row["tag_p50_px"],2),len(failures),round(seconds,2),flush=True)
        for strength in (0,5,12,25):
            for factor in (-1,1):
                pert=axis_transform(axis,center,factor*strength)@prior[1]
                for method in ("point_to_point","point_to_plane","gicp","gray_icp"):
                    try:
                        T=pair_fit(method,prep,1,0,pert,303)
                        res=pair_metric(views[1],views[0],T)
                        d=rotvec(T@np.linalg.inv(prior[1]))
                        x=views[1]["train"].mean(0)[None]
                        stress_rows.append({"session":sid,"method":method,"perturbation_deg":strength*factor,
                                            "eval_f1_2":res["f1_2"],"drift_deg":float(np.linalg.norm(d)*180/np.pi),
                                            "drift_at_center_mm":float(np.linalg.norm(transform(x,T)-transform(x,prior[1]))),"status":"scored"})
                    except RuntimeError:
                        stress_rows.append({"session":sid,"method":method,"perturbation_deg":strength*factor,"status":"failed"})
        dump(root/"metrics.json",series)
    for name,rows in (("summary",metric_rows),("edge_metrics",edge_rows),("view_inventory",view_rows),("initial_pose_stress",stress_rows)):
        with (OUTPUT/f"{name}.csv").open("w",newline="") as f:
            writer=csv.DictWriter(f,fieldnames=list(dict.fromkeys(k for row in rows for k in row)))
            writer.writeheader(); writer.writerows(rows)
    dump(OUTPUT/"benchmark_manifest.json",{"open3d":o3d.__version__,"opencv":cv2.__version__,"machine":platform.machine(),
                                           "config":CONFIG,"software_fit":list(METHODS),"split":"(x//24 + 2*(y//24)) % 5 == 0 is evaluation",
                                           "algorithm_scope":"post-17:41 ARUCO sessions; raw camera-frame clouds; no archived merges used",
                                           "angles":"nominal commanded; actual optical pose proxy in marker frame",
                                           "pair":[1,0],"full_eval":"8 physical neighbor edges incl. 7 to 0; all valid unique views fused"})


if __name__ == "__main__":
    main()
