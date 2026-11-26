"""E003 registration. Run from the repository root with the project Python."""
import csv
from pathlib import Path
import sys
from time import perf_counter

import cv2
import numpy as np
import open3d as o3d
from scipy.optimize import least_squares
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "E002"))
import common as archive
from common import KEYS, NAMES, BOX, table_basis, dump, load, load_case, ply, transform
from experiment import COLORS as VIEW_COLORS
from segment import evaluate_masks
from registration import metric, incremental, edge_closure, cuboid_distance, fit_cuboid
from calibrated_registration_benchmark import adjacent_from
from registration_experiment import training_mask, rigid_alignment

OUT = HERE / "results"
PRIOR = HERE.parent / "E002/results"
if not archive.PREV.exists():
    archive.PREV = HERE.parent / "E001"
    archive.CACHE = archive.PREV / "intermediate"
REG = o3d.pipelines.registration


def csv_write(path, rows):
    with path.open("w", encoding="utf-8-sig", newline="") as file:
        w=csv.DictWriter(file, fieldnames=list(dict.fromkeys(k for r in rows for k in r)))
        w.writeheader()
        w.writerows(rows)


def cloud(p, radius=None, voxel=None):
    c=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(p,dtype=float)))
    if voxel:
        c=c.voxel_down_sample(voxel)
    if radius and len(c.points)>=5:
        c.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius,max_nn=42))
        c.orient_normals_towards_camera_location(np.zeros(3))
    return c


def connected_filter(p):
    # Connected support is sought across registered views, so a thin edge can survive in one view.
    grid, idx, counts=np.unique(np.floor(p/2.35).astype(np.int64),axis=0,return_inverse=True,return_counts=True)
    ab=cKDTree(grid).query_pairs(1.75,output_type="ndarray")
    graph=coo_matrix((np.ones(len(ab)),(ab[:,0],ab[:,1])),shape=(len(grid),len(grid))).tocsr()
    _,groups=connected_components(graph,directed=False)
    sizes=np.bincount(groups,weights=counts)
    core=np.argmax(sizes)
    near=cKDTree(grid[groups==core]).query(grid)[0]
    mins=np.full(len(sizes),np.inf)
    np.minimum.at(mins,groups,near)
    allowed=(sizes>=max(28.,len(p)*.00055))&(mins<=4.)
    return allowed[groups[idx]], len(sizes), float(sizes[core]/len(p))


def denoise(scans, poses, source_masks):
    local=[]
    worlds=[]
    for i,(s,q) in enumerate(zip(scans,source_masks)):
        p=s["xyz"][q]
        if len(p)<20:
            local.append((np.ones(len(p),bool),np.ones(len(p),bool)))
        else:
            kdist=cKDTree(p).query(p,k=17)[0][:,1:]
            d=kdist.mean(1)
            sor=d<=d.mean()+2*d.std()
            local_median=np.median(kdist[:,7])
            tolerant=kdist[:,7]<=max(2.6,local_median*3.1)
            local.append((sor,tolerant))
        T=poses[i] if i<len(poses) else poses[0]
        worlds.append(transform(p[local[-1][1]],T))
    is_core,num_comps,ratio=connected_filter(np.concatenate(worlds[:len(poses)]))
    output=[]
    start=0
    for i,((sor,tolerant),q,world) in enumerate(zip(local,source_masks,worlds)):
        n=len(world)
        final=tolerant.copy()
        if i<len(poses):
            final[tolerant]=is_core[start:start+n]
            start+=n
        masks=[q.copy(),q.copy(),q.copy()]
        masks[1][q]=sor
        masks[2][q]=final
        output.append(masks)
    return output, num_comps, ratio


def build_views(scans, masks, method):
    views=[]
    for s,m in zip(scans,masks):
        train=training_mask(s["uv"])
        raw_held=m[0]&~train
        train_points=s["xyz"][m[method]&train]
        test=s["xyz"][raw_held]
        views.append({"train":train_points,"val":test,
            "train_normals":np.asarray(cloud(train_points,4).normals).copy(),
            "normals":np.asarray(cloud(test,3).normals).copy()})
    return views


def keep_support(P,Q,T):
    a=transform(P,T)
    da=cKDTree(Q).query(a)[0]
    db=cKDTree(a).query(Q)[0]
    return float(min(np.mean(da<=4),np.mean(db<=4)))


def feature_pack(points):
    c=cloud(points,4.5,1.65)
    pack=np.asarray(c.points)
    iss=o3d.geometry.keypoint.compute_iss_keypoints(c,salient_radius=7.5,non_max_radius=4.2,
        gamma_21=.975,gamma_32=.975,min_neighbors=6)
    q=np.asarray(iss.points)
    if len(q)<4:
        return pack,None,None,len(q)
    ids=cKDTree(pack).query(q)[1]
    vec=np.asarray(REG.compute_fpfh_feature(c,o3d.geometry.KDTreeSearchParamHybrid(radius=9.,max_nn=90)).data).T
    return pack,q,vec[ids].copy(),len(q)


def sac_ia(source,target,rng,shots=720):
    """SAC-IA-style minimal sampling, nearest FPFH candidates and robust symmetric scoring."""
    P,A,fa,_=source
    Q,B,fb,_=target
    if A is None or B is None:
        return None,None,0
    near=cKDTree(fb).query(fa,k=min(6,len(B)))[1]
    p=P[::max(1,len(P)//370)]
    q=Q[::max(1,len(Q)//370)]
    qt=cKDTree(Q)
    candidates=[]
    # Three spaced non-collinear ISS points; select among the similar FPFH target candidates.
    s=rng.integers(len(A),size=(shots,3))
    t=near[s,rng.integers(near.shape[1],size=(shots,3))]
    x,y=A[s],B[t]
    ij=np.array([0,0,1]); ik=np.array([1,2,2])
    L=np.linalg.norm(x[:,ij]-x[:,ik],axis=2)
    R=np.linalg.norm(y[:,ij]-y[:,ik],axis=2)
    area=np.linalg.norm(np.cross(x[:,1]-x[:,0],x[:,2]-x[:,0]),axis=1)
    good=(L.min(1)>=4.)&(R.min(1)>=3.)&(np.abs(L-R).max(1)<=4.)&(area>=9.)
    for aa,bb in zip(x[good],y[good]):
        T=rigid_alignment(aa,bb)
        forward=qt.query(transform(p,T))[0]
        score=np.minimum(forward,8.).mean()
        candidates.append((score,T))
    chosen=None
    for _,T in sorted(candidates,key=lambda a:a[0])[:24]:
        A1=transform(P,T)
        d=cKDTree(A1).query(q)[0]
        e=qt.query(transform(p,T))[0]
        score=.5*(np.minimum(d,8.).mean()+np.minimum(e,8.).mean())
        if chosen is None or score<chosen[0]:
            chosen=(float(score),T)
    return (chosen[1],chosen[0],len(candidates)) if chosen else (None,None,0)


def delta(T,prior,anchor):
    rot=Rotation.from_matrix(T[:3,:3]@prior[:3,:3].T).magnitude()
    shift=np.linalg.norm(transform(anchor[None],T)-transform(anchor[None],prior))
    return float(np.degrees(rot)),float(shift)


def local_icp(levels,a,b,initial):
    T=initial.copy()
    for ds,gate,source,target in zip((2.2,1.25,.8),(7.5,4.,2.6),levels[a],levels[b]):
        est=REG.TransformationEstimationPointToPlane(REG.TukeyLoss(k=gate))
        res=REG.registration_icp(source,target,gate,T,est,REG.ICPConvergenceCriteria(max_iteration=24))
        T=res.transformation
    return T


def ppf_control(v,ring,base,cuts):
    """Chapter 4 coarse pose check on a real pair and a known rigid self-transform."""
    candidates=[(keep_support(v[a]["train"][::8],v[b]["train"][::8],adjacent_from(base,a,b)),a,b)
                for a,b in ring if min(len(v[a]["train"]),len(v[b]["train"]))>=120]
    if not candidates:
        return []
    _,a,b=max(candidates)
    pa,pb=(cloud(v[i]["train"],6.,2.65) for i in (a,b))
    ax=np.c_[np.asarray(pa.points),np.asarray(pa.normals)].astype("float32")
    bx=np.c_[np.asarray(pb.points),np.asarray(pb.normals)].astype("float32")
    ax=ax[::max(1,len(ax)//525)]
    bx=bx[::max(1,len(bx)//525)]
    syn=np.eye(4)
    syn[:3,:3]=Rotation.from_euler("zyx",[27,-13,15],degrees=True).as_matrix()
    syn[:3,3]=[14,-9,6]
    yy=np.c_[transform(ax[:,:3],syn),ax[:,3:]@syn[:3,:3].T].astype("float32")
    rows=[]
    for mode,Y,reference in (("known_transform",yy,syn),("actual_adjacent",bx,adjacent_from(base,a,b))):
        T=None
        tt=perf_counter()
        # OpenCV uses the model diameter to scale the quantization.
        try:
            detector=cv2.ppf_match_3d_PPF3DDetector(.06,.065,28)
            detector.trainModel(ax)
            votes=detector.match(Y,.45,.07)
            best=max(votes[:5],key=lambda v:v.numVotes,default=None)
            if best is not None:
                T=best.pose.copy()
        except cv2.error:
            best=None
        row={"test":mode,"source":a,"target":b,"votes":int(best.numVotes) if T is not None else None,
             "seconds":perf_counter()-tt,"source_samples":len(ax),"target_samples":len(Y)}
        if T is not None:
            angle,shift=delta(T,reference,ax[:,:3].mean(0))
            row.update(coarse_deg=angle,coarse_center_shift=shift,coarse_support=keep_support(ax[:,:3],Y[:,:3],T))
            tgt=b if mode=="actual_adjacent" else -1
            arrays=cuts if tgt>=0 else dict(cuts)
            if tgt==-1:
                arrays[-1]=[cloud(Y[:,:3],max(2.6,3*x),x) for x in (2.2,1.25,.8)]
            refined=local_icp(arrays,a,tgt,T)
            angle,shift=delta(refined,reference,ax[:,:3].mean(0))
            row.update(refined_deg=angle,refined_center_shift=shift,
                       refined_support=keep_support(ax[:,:3],Y[:,:3],refined))
            if tgt==-1:
                row["ADD_match"]=np.linalg.norm(transform(ax[:,:3],refined)-transform(ax[:,:3],reference),axis=1).mean()
        rows.append(row)
    return rows


def fixed_axis_box(v,base,axis,center,meta,cal,training_layout):
    """Fit one shared lateral axis displacement and bounded per-view angles."""
    _,initial=fit_cuboid(v,axis,center,base)
    B=table_basis(axis)
    start=np.r_[initial,np.zeros(2+len(base)-1)]
    step=np.r_[[.10,5.,5.,1.2,5.,5.,5.],2.8,2.8,np.full(len(base)-1,1.5)]
    P=[a["train"][::max(1,len(a["train"])//850)] for a in v]
    tag_hits=[]
    if training_layout:
        for i,record in enumerate(meta["markers"][:len(base)]):
            for marker,uv in record["tags"].items():
                if marker in training_layout:
                    tag_hits.append((i,np.asarray(training_layout[marker]),np.asarray(uv)))

    def matrices(t):
        shift=B[:,:2]@t[7:9]
        correction=Rotation.from_rotvec(np.deg2rad(np.r_[0.,t[9:]])[:,None]*axis).as_matrix()
        T=base.copy()
        T[:,:3,:3]=correction@base[:,:3,:3]
        pivot=center+shift
        updated_translation=base[:,:3,3]+shift-base[:,:3,:3]@shift
        T[:,:3,3]=pivot+(correction@(updated_translation-pivot)[:,:,None])[:,:,0]
        return T

    def residual(t):
        T=matrices(t)
        parts=[cuboid_distance((transform(p,tx)-center)@B,t[:7]) for p,tx in zip(P,T)]
        parts.extend([np.repeat(t[7:9]*5.,8),np.repeat(t[9:]*3.5,12)])
        inv=np.linalg.inv(T)
        if tag_hits:
            for i,obj,uv in tag_hits:
                cam=transform(obj,inv[i])
                pix=cv2.projectPoints(cam,np.zeros(3),np.zeros(3),cal["cam_int"],cal["cam_dist"])[0][:,0]
                parts.append((2.8*(pix-uv)).ravel())
        return np.concatenate(parts)

    result=least_squares(residual,start,bounds=(start-step,start+step),max_nfev=35,
                         loss="soft_l1",f_scale=1.8)
    return matrices(result.x), {"axis_delta_xy":result.x[7:9],"max_angle_delta":np.max(np.abs(result.x[9:]))}


def register(key, meta, scans, masks, base, pure, axis, center, cal, marker_layout):
    ring=[(i,(i+1)%len(base)) for i in range(len(base))]
    eval_rows=[]
    edge_rows=[]
    routes={"R01":(base,0),"R02":(pure,0)}
    closures={}
    ppf=[]
    for denoiser,number in enumerate(("R03","R04","R05")):
        v=build_views(scans[:len(base)],masks[:len(base)],denoiser)
        levels={i:[cloud(t["train"],max(2.6,3*s),s) for s in (2.2,1.25,.8)] for i,t in enumerate(v) if len(t["train"])>=65}
        if denoiser==2:
            features={i:feature_pack(v[i]["train"]) for i in levels}
            ppf=ppf_control(v,ring,base,levels)
            feature_edges={}
        e={}
        for a,b in ring:
            if a not in levels or b not in levels:
                continue
            tt=perf_counter()
            T0=adjacent_from(base,a,b)
            P,Q=v[a]["train"][::3],v[b]["train"][::3]
            U=local_icp(levels,a,b,T0)
            us=keep_support(P,Q,U)
            e[a,b]=(U,us)
            row={"route":number,"a":a,"b":b,"calibrated_support4":keep_support(P,Q,T0),
                 "refined_support4":us}
            if denoiser==2:
                estimate,loss,hypotheses=sac_ia(features[a],features[b],np.random.default_rng(904+a*11+b))
                row.update(iss_a=features[a][-1],iss_b=features[b][-1],
                    hypotheses=hypotheses,sac_score=loss,feature_accepted=False)
                if estimate is not None:
                    ang0,dist0=delta(estimate,T0,P.mean(0))
                    f=local_icp(levels,a,b,estimate)
                    ang,dist=delta(f,T0,P.mean(0))
                    fs=keep_support(P,Q,f)
                    use=ang<=7.5 and dist<=7. and fs>=max(.13,us-.015)
                    row.update(sac_deg_from_cal=ang0,sac_shift_from_cal=dist0,
                        feature_deg_from_cal=ang,feature_shift_from_cal=dist,
                        feature_support4=fs,feature_accepted=bool(use))
                    if use:
                        feature_edges[a,b]=(f,fs)
            row["seconds"]=perf_counter()-tt
            edge_rows.append(row)
        P,count=incremental(v,base,e,center)
        routes[number]=(P,count)
        closures[number]=edge_closure(e,ring,center)
        if denoiser==2:
            combined={**e,**feature_edges}
            P,count=incremental(v,base,combined,center)
            routes["R06"]=(P,count)
            closures["R06"]=edge_closure(combined,ring,center)
            eval_v=v
    axis_details={}
    if key in BOX:
        poses,axis_details=fixed_axis_box(eval_v,base,axis,center,meta,cal,marker_layout)
        routes["R07"]=(poses,0)
    for route,(poses,n) in routes.items():
        m=metric(eval_v,poses,ring,axis,center,meta,cal)
        closeR,closeT=closures.get(route,(None,None))
        m.update(route=route,edges_used=n,independent_loop_rotation_deg=closeR,independent_loop_center_motion=closeT)
        eval_rows.append(m)
    # Shared held-out tiles and support guards. Benchmarks R01–R04 provide ablations.
    reference=eval_rows[0]
    def qualifies(row):
        if row["route"] not in ("R05","R06","R07") or (row["edges_used"]==0 and row["route"]!="R07"):
            return False
        if row["tag_P50"] is not None and row["tag_P50"]>reference["tag_P50"]+1.:
            return False
        return (row["max_gap"]<=reference["max_gap"]+2.5 and
              row["depth_supported"]>=reference["depth_supported"]-.025 and
              row["depth_abs_P50"]<reference["depth_abs_P50"]*.97 and
              row["F1_2"]>=reference["F1_2"]-.015)
    best=min((r for r in eval_rows if qualifies(r)),key=lambda r:r["depth_abs_P50"],default=reference)
    return routes,best["route"],eval_rows,edge_rows,ppf,axis_details


def main():
    cv2.setNumThreads(2)
    cv2.ocl.setUseOpenCL(False)
    o3d.utility.random.seed(518)
    OUT.mkdir(exist_ok=True)
    all_reg=[]; all_edges=[]; all_noise=[]; all_ppf=[]; index={}
    for j,key in enumerate(KEYS,1):
        begin=perf_counter()
        tag=f"E003-S{j:02d}"
        meta,previous,_,s,cal=load_case(key)
        number=tag.replace("E003","E002")
        earlier=load(PRIOR/"clouds"/number/f"{number}-03.json")
        base=np.array(earlier["selected_poses"])
        axial=np.array(earlier["axis"])
        center=np.array(earlier["center"])
        pure=np.array(earlier["variants"]["R02"])
        references=[]; original=[]
        for i,sc in enumerate(s):
            with np.load(PRIOR/"cache"/number/f"V{i:02}.npz") as block:
                references.append({k:block[k] for k in ("h","n","c","silhouette")})
                original.append(block[earlier["separation"]])
        masks,clusters,ratio=denoise(s,base,original)
        print(tag,"denoised",flush=True)
        marker_layout=previous.get("reference",{}).get("training_layout",{})
        routes,chosen,reg,edges,ppf,axis_details=register(key,meta,s,masks,base,pure,axial,center,cal,marker_layout)
        poses,nb=routes[chosen]
        folder=OUT/tag
        folder.mkdir(exist_ok=True)
        points=[[],[],[]]; color=[[],[],[]]
        for i,(sc,mm,aux) in enumerate(zip(s,masks,references)):
            T=poses[i] if i<len(poses) else base[0]
            for di,m in enumerate(mm):
                stats=evaluate_masks(key,i,sc, {**dict.fromkeys([f"M{k:02d}" for k in range(1,8)],m), "h":aux["h"]})
                row={"id":tag,"view":i,"filter":f"D{di:02d}","point_count":int(m.sum())}
                if stats:
                    row.update(stats["test"]["M01"])
                all_noise.append(row)
                if i<len(poses):
                    points[di].append(transform(sc["xyz"][m],T))
                    color[di].append(np.broadcast_to(VIEW_COLORS[i%len(VIEW_COLORS)],(m.sum(),3)))
            m=mm[2]
            ply(folder/"views"/f"{tag}-V{i:02d}.ply",transform(sc["xyz"][m],T),
                sc["gray"][m]/255.)
            np.savez_compressed(folder/f"{tag}-V{i:02d}.npz",mask=m,silhouette=aux["silhouette"])
        for i,ps in enumerate(points):
            ply(folder/f"{tag}-D{i:02d}.ply",np.concatenate(ps),np.concatenate(color[i]))
        for r in reg:
            all_reg.append({"id":tag,"selected":r["route"]==chosen,**r})
        all_edges.extend({"id":tag,**r} for r in edges)
        all_ppf.extend({"id":tag,**r} for r in ppf)
        entry={"id":tag,"key":key,"label":NAMES[key],"source":earlier["source"],"selected":chosen,
               "counts":[sum(map(len,ps)) for ps in points],"global_filter_component_count":clusters,"main_component_ratio":ratio,
               "axis":axial,"center":center,"poses":poses,"pose_variants":{k:v[0] for k,v in routes.items()},
               "box_axis_test":axis_details,
               "run_seconds":perf_counter()-begin}
        dump(folder/f"{tag}-01.json",entry)
        index[tag]=entry
        print(tag,chosen,entry["counts"],"sec",round(entry["run_seconds"],1),flush=True)
    for n,table in enumerate((all_noise,all_reg,all_edges,all_ppf),1):
        csv_write(OUT/f"E003-0{n}.csv",table)
    dump(OUT/"E003-05.json",index)


if __name__=="__main__":
    main()
