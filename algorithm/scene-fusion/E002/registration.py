"""Compare turntable initialization, sequential ICP and a gated incremental graph."""
from time import perf_counter

import cv2
import numpy as np
import open3d as o3d
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from common import BOX, table_basis, transform
from calibrated_registration_benchmark import CONFIG, adjacent_from, cloud_sample, pair_metric, training_support
from registration_experiment import fit_pair, prepare_pair, training_mask
from run import image_axis, stage_poses


def normalize_views(scenes, segmented, method):
    # Keep registration sampling independent from the numbered output masks.
    v=[]
    for s,r in zip(scenes,segmented):
        keep=r[method]
        row=cloud_sample(s["xyz"][keep],s["uv"][keep],s["gray"][keep])
        cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(row["train"]))
        if len(cloud.points)>=20:
            cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=4.,max_nn=32))
        row["train_normals"]=np.asarray(cloud.normals).copy()
        v.append(row)
    return v


def reg_pairs(v,prior,available,ring,method):
    prep={i:prepare_pair(v[i]["train"],CONFIG,v[i]["gray"]) for i in available}
    edges={}
    for a,b in ring:
        if a not in prep or b not in prep:continue
        init=adjacent_from(prior,a,b)
        try:
            t,_=fit_pair(method,prep[a],prep[b],init,CONFIG,None,None,91+a*11+b)
            support=training_support(v,a,b,t)
            if support>=.04:
                edges[a,b]=(t,support)
        except (RuntimeError,ValueError):
            continue
    return edges


def edge_closure(edges,ring,center):
    if not all(e in edges for e in ring):
        return None,None
    T=np.eye(4)
    for e in ring: T=edges[e][0]@T
    return float(np.degrees(Rotation.from_matrix(T[:3,:3]).magnitude())),float(np.linalg.norm(transform(center[None],T)[0]-center))


def incremental(v,init,edges,center):
    # The 0th pose is fixed. Admit the strongest edges first, adding weak links only after warm start.
    surface=[]
    for (a,b),(edge,support) in edges.items():
        P,Q=v[a]["train"][::3],v[b]["train"][::3]
        ai,bi=cKDTree(Q).query(transform(P,edge)); _,rev=cKDTree(transform(P,edge)).query(Q)
        keep=(ai<3)&(rev[bi]==np.arange(len(P)))
        na=v[a]["train_normals"][::3]@edge[:3,:3].T
        nb=v[b]["train_normals"][::3][bi]
        keep &= abs(np.sum(na*nb,axis=1))>=np.cos(np.deg2rad(30))
        prior=adjacent_from(init,a,b)
        delta=edge@np.linalg.inv(prior)
        orig=transform(center[None],init[b])
        deg=np.rad2deg(Rotation.from_matrix(delta[:3,:3]).magnitude())
        dist=np.linalg.norm(transform(orig,delta)-orig)
        if keep.sum()>=15 and support>=.12 and deg<=8 and dist<=9:
            surface.append((support,a,b,P[keep][::2],Q[bi[keep]][::2]))
    surface.sort(key=lambda e:-e[0])
    n=len(init)
    def poses(x):
        isos=np.repeat(np.eye(4)[None],n,axis=0)
        isos[1:,:3,:3]=Rotation.from_rotvec(x.reshape(n-1,6)[:,:3]/65).as_matrix()
        isos[1:,:3,3]=center-(isos[1:,:3,:3]@center)+x.reshape(n-1,6)[:,3:]
        return isos@init
    x=np.zeros((n-1)*6)
    active=sorted(set((min(2,len(surface)),min(5,len(surface)),len(surface))))
    for N in active:
        if N==0:continue
        def err(x):
            T=poses(x)
            out=[x*1.7]
            for w,a,b,P,Q in surface[:N]:
                res=transform(P,T[a])-transform(Q,T[b])
                out.append((res*np.sqrt(3*w/len(res))).ravel())
            return np.concatenate(out)
        x=least_squares(err,x,bounds=(-1.4,1.4),loss="soft_l1",f_scale=.5,max_nfev=17).x
    return poses(x),len(surface)


def cuboid_distance(local,theta):
    yaw,x,y,bottom,half_x,half_y,height=theta
    c,s=np.cos(yaw),np.sin(yaw)
    uv=(local[:,:2]-[x,y])@np.array([[c,-s],[s,c]])
    xyz=np.c_[abs(uv),local[:,2]-bottom-height/2]
    delta=abs(xyz)-[half_x,half_y,height/2]
    return np.linalg.norm(np.maximum(delta,0),axis=1)+np.minimum(delta.max(1),0)


def fit_cuboid(samples,axis,center,poses,joint=False,marker=None):
    B=table_basis(axis)
    valid=[i for i,v in enumerate(samples) if len(v["train"])>=80]
    a=[samples[i]["train"][::max(1,len(samples[i]["train"])//850)] for i in valid]
    world=[(transform(p,poses[i])-center)@B for p,i in zip(a,valid)]
    p=np.concatenate(world)
    upper=p[p[:,2]>max(3,np.percentile(p[:,2],15))]
    center0=np.median(upper[:,:2],axis=0)
    zz=np.clip(np.percentile(upper[:,2],94),12,135)
    candidates=[]
    for yaw in np.deg2rad(np.arange(0,90,7.5)):
        c,s=np.cos(yaw),np.sin(yaw)
        r=np.array([[c,-s],[s,c]])
        lo,hi=np.percentile(upper[:,:2]@r,[4,96],axis=0)
        q=(lo+hi)/2@r.T
        lengths=np.clip((hi-lo)/2,8,85)
        start=np.r_[yaw,q,0.,lengths,zz]
        lower=np.r_[yaw-.22,center0-35,-3.,np.maximum(7.,lengths*.69),max(8.,zz*.65)]
        upper_bound=np.r_[yaw+.22,center0+35,4.,np.minimum(100.,lengths*1.43+3),min(150.,zz*1.38+3)]
        def residual(theta):
            return np.r_[cuboid_distance(p,theta),theta[3]*6.]
        x=least_squares(residual,start,bounds=(lower,upper_bound),loss="soft_l1",f_scale=2.5,max_nfev=30)
        candidates.append(x)
    initial=min(candidates,key=lambda v:v.cost).x
    def adjustments(v):
        n=len(poses)
        A=np.repeat(np.eye(4)[None],n,axis=0)
        A[1:,:3,:3]=Rotation.from_rotvec(v[:,0,None]*axis).as_matrix()
        A[1:,:3,3]=center-(A[1:,:3,:3]@center)+v[:,1:]@B[:,:2].T
        return A@poses
    P=poses.copy()
    fit=initial
    if joint:
        caps=[.028,1.,1.] if marker else [.044,1.6,1.6]
        shape_bounds=np.r_[[.075,4,4,1.2,5,5,6],np.tile(caps,len(poses)-1)]
        def residual(t):
            T=adjustments(t[7:].reshape(-1,3))
            z=[cuboid_distance((transform(p,T[i])-center)@B,t[:7]) for p,i in zip(a,valid)]
            reg=t[7:].reshape(-1,3)*[160,4,4]
            out=[np.concatenate(z),np.repeat(reg.ravel(),8),[t[3]*6]]
            if marker:
                cal,observations=marker
                for i,xyz,uv in observations:
                    camera_xyz=transform(xyz,np.linalg.inv(T[i]))
                    pred=cv2.projectPoints(camera_xyz,np.zeros(3),np.zeros(3),cal["cam_int"],cal["cam_dist"])[0][:,0]
                    out.append(((pred-uv)*2.8).ravel())
            return np.concatenate(out)
        start=np.r_[initial,np.zeros((len(poses)-1)*3)]
        x=least_squares(residual,start,bounds=(start-shape_bounds,start+shape_bounds),
                        loss="soft_l1",f_scale=2.5,max_nfev=30).x
        fit=x[:7]
        P=adjustments(x[7:].reshape(-1,3))
    return P,fit


def shape_score(v,poses,axis,center,theta):
    B=table_basis(axis); rr=[]
    for T,z in zip(poses,v):
        if len(z["val"]):rr.append(abs(cuboid_distance((transform(z["val"],T)-center)@B,theta)))
    x=np.concatenate(rr)
    return {"box_surface_P50":float(np.median(x)),"box_surface_P95":float(np.percentile(x,95)),
       "box_inlier_at_2":float(np.mean(x<=2)),"box_dimensions":[float(2*theta[4]),float(2*theta[5]),float(theta[6])]}


def visible_depth(v,poses,cal):
    # Leave the whole test scan out of the reference. Both methods share 8×8 px depth bins.
    size=(306,256)
    support=[];err=[];bad=[];agreement=[];rear=[]
    for j,(view,T) in enumerate(zip(v,poses)):
        if len(view["val"])<40:continue
        P=np.concatenate([transform(v[i]["train"][::2],p) for i,p in enumerate(poses) if i!=j])
        xyz=transform(P,np.linalg.inv(T))
        def uv(xyz):
            p=cv2.projectPoints(xyz,np.zeros(3),np.zeros(3),cal["cam_int"],cal["cam_dist"])[0][:,0]
            xy=np.rint(p/8).astype(int)
            u,w=xy.T
            good=(u>=0)&(u<size[0])&(w>=0)&(w<size[1])&(xyz[:,2]>0)
            return u,w,good
        x,y,valid=uv(xyz)
        image=np.full(size[0]*size[1],np.inf)
        np.minimum.at(image,y[valid]*size[0]+x[valid],xyz[valid,2])
        x,y,valid=uv(view["val"])
        ix=y[valid]*size[0]+x[valid]
        z=image[ix]
        has=np.isfinite(z)
        if not len(z):continue
        support.append(float(has.mean()))
        d=z[has]-view["val"][valid,2][has]
        err.extend(abs(d));bad.extend(d<-3.);rear.extend(d>3.)
        agreement.append(float((abs(d)<=3.).sum()/max(len(z),1)))
    return {"depth_supported":float(np.median(support)) if support else None,
      "depth_abs_P50":float(np.median(err)) if len(err) else None,"depth_front_violation":float(np.mean(bad)) if len(bad) else None,
      "depth_behind_fraction":float(np.mean(rear)) if len(rear) else None,
      "depth_agree_and_supported":float(np.median(agreement)) if agreement else None}


def metric(v,poses,ring,n,center,meta,cal):
    def mean_of(es):
        return float(np.median(es)) if es else None
    infos=[pair_metric(v[a],v[b],adjacent_from(poses,a,b)) for a,b in ring]
    infos=[x for x in infos if x["status"]=="scored"]
    ds=[]
    for a,b in ring:
        if min(len(v[a]["val"]),len(v[b]["val"]))<20:continue
        A=transform(v[a]["val"],poses[a])
        B=transform(v[b]["val"],poses[b])
        d=np.r_[cKDTree(A).query(B)[0],cKDTree(B).query(A)[0]]
        ds.append({"chamfer_l1":float(np.mean(d)),"p95":float(np.percentile(d,95)),
                 "rmse_inlier_2":float(np.sqrt(np.mean(d[d<=2]**2))) if np.any(d<=2) else None})
    B=table_basis(n)
    pp=(poses[:,:3,3]-center)@B[:,:2]
    a=np.sort(np.degrees(np.arctan2(pp[:,1],pp[:,0])))
    gaps=np.diff(np.r_[a,a[0]+360])
    marker=[]
    for i,obs in meta.get("heldout",{}).items():
        pix=cv2.projectPoints(transform(np.array(meta["holdout_xyz"]),np.linalg.inv(poses[int(i)])),np.zeros(3),np.zeros(3),
               cal["cam_int"],cal["cam_dist"])[0][:,0]
        marker.extend(np.linalg.norm(pix-np.array(obs),axis=1))
    return {"F1_1":mean_of([r["f1_1"] for r in infos]),"F1_2":mean_of([r["f1_2"] for r in infos]),
      "F1_4":mean_of([r["f1_4"] for r in infos]),"worst_F1_2":min([r["f1_2"] for r in infos],default=None),
      "seam_normal_P95":mean_of([r["seam_p95"] for r in infos if r["seam_p95"] is not None]),
      "chamfer_l1":mean_of([r["chamfer_l1"] for r in ds]),"P95_NN":mean_of([r["p95"] for r in ds]),
      "inlier_RMSE2":mean_of([r["rmse_inlier_2"] for r in ds if r["rmse_inlier_2"] is not None]),
      "max_gap":float(max(gaps)),"tag_P50":mean_of(marker),"edge_count":len(infos),
      **visible_depth(v,poses,cal)}


def run_registration(key,meta,previous,scenes,segmented,method,cal):
    N=len(scenes)-int(previous["repeat_index"] is not None)
    v=normalize_views(scenes[:N],segmented[:N],method)
    P=np.array(previous["poses"][:N])
    n,center=np.array(previous["axis"]),np.array(previous["center"])
    available=[i for i,x in enumerate(v) if len(x["train"])>=80]
    ring=[(a,(a+1)%N) for a in range(N)]
    output={"R01":(P,{},0,0.)}
    axis=np.array(previous["pose_variants"]["physical_axis"])
    output["R02"]=(axis,{},0,0.)
    sequential=(("R03","point_to_point",P),("R04","point_to_plane",P),("R05","gicp",P),
                ("R09","point_to_point",axis),("R10","point_to_plane",axis),("R11","gicp",axis))
    for number,name,prior in sequential:
        t=perf_counter()
        edges=reg_pairs(v,prior,available,ring,name)
        chain=prior.copy()
        for a,b in ring[:-1]:
            if (a,b) in edges:chain[b]=chain[a]@np.linalg.inv(edges[a,b][0])
        output[number]=(chain,edges,len(edges),perf_counter()-t)
    t=perf_counter()
    guarded,edges=incremental(v,P,output["R04"][1],center)
    output["R06"]=(guarded,output["R04"][1],edges,perf_counter()-t)
    shape={}
    if key in BOX:
        t=perf_counter()
        for m in ("R01","R02","R06"):
            _,shape[m]=fit_cuboid(v,n,center,output[m][0])
        updated,fitted=fit_cuboid(v,n,center,P,True)
        output["R07"]=(updated,{},0,perf_counter()-t)
        shape["R07"]=fitted
        layout=previous.get("reference",{}).get("training_layout",{})
        if layout:
            observation=[]
            for i,view in enumerate(meta["markers"][:N]):
                for k,uv in view["tags"].items():
                    if k in layout: observation.append((i,np.array(layout[k]),np.array(uv)))
            t=perf_counter()
            stable,fit_stable=fit_cuboid(v,n,center,P,True,marker=(cal,observation))
            output["R08"]=(stable,{},0,perf_counter()-t)
            shape["R08"]=fit_stable
    rows={}
    for label,(poses,e,n_links,t) in output.items():
        d=metric(v,poses,ring,n,center,meta,cal)
        loop,shift=edge_closure(e,ring,center)
        d.update(independent_loop_deg=loop,independent_loop_center_mm=shift,links=n_links,seconds=t)
        if label in shape:d.update(shape_score(v,poses,n,center,shape[label]))
        rows[label]=d
    return output,rows,shape
