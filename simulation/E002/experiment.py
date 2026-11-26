"""Real-scan E002 completion. Predictions are separate from measured scenes."""
import csv
import json
from pathlib import Path
import sys
from time import perf_counter

import cv2
import numpy as np
import open3d as o3d
from scipy.ndimage import label
from scipy.optimize import least_squares
from scipy.spatial import ConvexHull, cKDTree

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path.insert(0,str(HERE.parent))
import run as volume
sys.path.insert(0,str(ROOT/"algorithm/scene-fusion/E002"))
from common import BOX, KEYS, IDS, NAMES, OUTPUT as PREV, dump, load, load_case, table_basis, transform

OUT=HERE/"results"
MODELS={"C01":"平底凸包","C02":"平底高度场","C03":"轮廓与深度约束的高度场","C04":"长方体几何先验","C05":"补盘面法向的 Poisson","C06":"苹果椭球先验"}


def setup(flat):
    low,high=np.percentile(flat,[.25,99.75],axis=0)
    dims=np.maximum(high[:2]-low[:2]+12,16)
    step=float(max(1.35,max(dims)/113.,(high[2]+8)/86.))
    volume.PITCH=step
    volume.ORIGIN=np.r_[low[:2]-6,0.]
    volume.SIZE=tuple(np.ceil(np.r_[dims, max(13,high[2]+7)]/step).astype(int))
    volume.GRID=volume.ORIGIN+(np.indices(volume.SIZE).reshape(3,-1).T+.5)*step
    return step


def shape_hull(p):
    flat=np.concatenate([p,np.c_[p[:,:2],np.zeros(len(p))]])
    hull=ConvexHull(flat)
    filled=np.ones(len(volume.GRID),bool)
    for e in hull.equations:
        ix=np.flatnonzero(filled)
        filled[ix]=volume.GRID[ix]@e[:3]<-e[3]+.32
    return filled


def calc_camera_grid(grid,world_center,basis,pose,cal):
    camera=transform(grid@basis.T+world_center,np.linalg.inv(pose))
    u=cv2.projectPoints(camera,np.zeros(3),np.zeros(3),cal["cam_int"],cal["cam_dist"])[0][:,0]
    uv=np.rint(u/8).astype(int)
    x,y=uv.T
    ok=(x>=0)&(x<306)&(y>=0)&(y<256)&(camera[:,2]>0)
    return camera[:,2],x,y,ok


def free_space(initial,frames,local,poses,cal,center,B,shadow):
    indices=np.flatnonzero(initial)
    points=volume.GRID[indices]
    votes=np.zeros(len(indices),np.int16)
    outline=np.zeros(len(indices),np.int16)
    active=0
    for i,(obs,world,T) in enumerate(zip(frames,local,poses)):
        if len(world)<220:continue
        z,x,y,ok=calc_camera_grid(points,center,B,T,cal)
        uv=np.rint(obs["uv"]/8).astype(int)
        u,v=uv.T
        u=u.clip(0,305);v=v.clip(0,255)
        image=np.full((256,306),2000.,np.float32)
        np.minimum.at(image,(v,u),obs["xyz"][:,2])
        nz=z[ok];u0=x[ok];v0=y[ok]
        d=image[v0,u0]
        ix=np.where(ok)[0]
        votes[ix]+=(d<1900.)&(nz<d-3.8)
        if not shadow:
            if obs["silhouette"].max()>0:
                im=cv2.resize(obs["silhouette"],(306,256),interpolation=cv2.INTER_NEAREST)
                im=cv2.dilate(im,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(15,15)))
                if np.count_nonzero(im)>240:
                    outline[ix]+=(im[v0,u0]==0)
                    active+=1
    result=initial.copy()
    # Require evidence from two camera passes for carving, and use silhouettes only if multiple masks agree.
    prune=(votes>=2)|((outline>=max(3,active//2))&(active>=4))
    result[indices[prune]]=False
    return result,{"voxel_votes_removed":int(prune.sum()),"usable_silhouettes":active}


def major(mask):
    m=mask.reshape(volume.SIZE)
    structure=np.zeros((3,3,3),np.uint8)
    structure[1,1,:]=1;structure[1,:,1]=1;structure[:,1,1]=1
    tags,n=label(m,structure)
    if n==0:return mask
    q=np.bincount(tags.ravel()); q[0]=0
    return (tags==q.argmax()).ravel()


def mesh_closed(mesh):
    f=np.asarray(mesh.triangles).astype(np.int64)
    n=len(mesh.vertices)
    edges=f[:,[0,1,1,2,2,0]].reshape(-1,2)
    code=edges.min(1)*n+edges.max(1)
    _,inv,ct=np.unique(code,return_inverse=True,return_counts=True)
    signs=np.bincount(inv,weights=np.where(edges[:,0]<edges[:,1],1,-1))
    return bool(len(ct) and np.all(ct==2) and np.all(signs==0))


def measured_frame(s,mask,silhouette,pose,center,B):
    p=(transform(s["xyz"][mask],pose)-center)@B
    take=np.isfinite(p).all(1)&(p[:,2]>=-2.5)&(p[:,2]<155)
    row={k:s[k][mask][take] for k in ("uv","xyz")}
    row["silhouette"]=silhouette
    p=p[take];p[:,2]=p[:,2].clip(.04,None)
    return p,row


def box_mesh(shape):
    yaw,x,y,z,a,b,h=shape
    c,s=np.cos(yaw),np.sin(yaw)
    R=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
    mesh=o3d.geometry.TriangleMesh.create_box(2*a,2*b,h)
    mesh.translate([-a,-b,0.])
    mesh.rotate(R,center=np.zeros(3))
    mesh.translate([x,y,max(0,z)])
    return mesh


def predict(code,local,frames,poses,cal,center,B,shape,shadow):
    pts=np.concatenate(local)
    step=setup(pts)
    base=shape_hull(pts)
    height=volume.infer_height(pts,base.reshape(volume.SIZE).any(2).ravel())
    carved,diagnosis=free_space(height,frames,local,poses,cal,center,B,shadow)
    occ={"C01":base,"C02":height,"C03":carved}
    if shape is not None:occ["C04"]=None
    if code=="E002-S02":occ["C06"]=None
    if not shadow:occ["C05"]=None
    result={}
    for method,vox in occ.items():
        if method=="C04":
            mesh,_=volume.fitted_cuboid(pts) if shadow else (box_mesh(shape),None)
        elif method=="C06":
            # Fit the axes from available points. Constraining the ellipsoid to be tangent to the table supplies the bottom prior.
            a=pts[::max(1,len(pts)//10000)]
            loc=np.median(a[:,:2],axis=0)
            xx,yy=np.clip(np.ptp(np.percentile(a[:,:2],[3.,97.],axis=0),axis=0)/2.,10.1,70.)
            zz=np.clip(np.percentile(a[:,2],94.)*.5,10.1,67.)
            seed=np.r_[loc,xx,yy,zz]
            def residual(v):
                q=(a-np.r_[v[:2],v[4]])/v[2:]
                return np.r_[(np.linalg.norm(q,axis=1)-1)*np.mean(v[2:]),(v-seed)*.35]
            opt=least_squares(residual,seed,bounds=(np.r_[loc-20,[10.,10.,10.]],np.r_[loc+20,[82.,82.,78.]]),
                               loss="soft_l1",f_scale=2.8,max_nfev=55).x
            mesh=o3d.geometry.TriangleMesh.create_sphere(radius=1.,resolution=50)
            T=np.eye(4); T[:3,:3]=np.diag(opt[2:]);T[:3,3]=np.r_[opt[:2],opt[4]]
            mesh.transform(T)
        elif method=="C05":
            # Invent only the flat support face, then compare how Poisson closes the remaining unknown gaps.
            ids=np.flatnonzero(base.reshape(volume.SIZE).any(2).ravel())[::3]
            plane=volume.GRID.reshape((*volume.SIZE,3))[:,:,0,:2].reshape(-1,2)[ids]
            if len(plane)<50:continue
            cloud=[{"observed":p[::3],"camera":type("C",(),{"origin":(t[:3,3]-center)@B})}
                   for p,t in zip(local,poses) if len(p)>60]
            cloud.append({"observed":np.c_[plane,np.zeros(len(plane))],
                          "camera":type("C",(),{"origin":np.r_[np.mean(plane,axis=0),-80.]})})
            mesh,meta=volume.poisson_mesh(cloud)
            if mesh is None:
                result[method]=(None,None,False,step,diagnosis)
                continue
        else:
            vox=major(vox)
            if vox.sum()<50:
                result[method]=(None,None,False,step,diagnosis)
                continue
            mesh=volume.voxel_mesh(vox)
        good=mesh_closed(mesh) if method!="C05" else meta["watertight"]
        if method=="C05":
            vv=meta["volume"] if good else None
        else:
            f=np.asarray(mesh.triangles)
            v=np.asarray(mesh.vertices)
            vv=float(abs(np.einsum("ij,ij->i",v[f[:,0]],np.cross(v[f[:,1]],v[f[:,2]])).sum()/6.)) if good else None
        result[method]=(mesh,vv,good,step,diagnosis)
    return result


def main():
    cv2.setNumThreads(2);cv2.ocl.setUseOpenCL(False)
    o3d.utility.random.seed(642)
    OUT.mkdir(exist_ok=True)
    rows=[];index={}
    for key in KEYS:
        begin=perf_counter()
        tag=IDS[key]
        s=OUT/tag
        s.mkdir(exist_ok=True)
        optical,prev,_,scenes,cal=load_case(key)
        result=load(PREV/"clouds"/tag/f"{tag}-03.json")
        poses=np.array(result["selected_poses"])
        center=np.array(result["center"])
        B=table_basis(np.array(result["axis"]))
        local=[]; frames=[]
        for i,T in enumerate(poses):
            with np.load(PREV/"cache"/tag/f"V{i:02}.npz") as pack:
                p,obs=measured_frame(scenes[i],pack[result["separation"]],pack["silhouette"],T,center,B)
            local.append(p);frames.append(obs)
        combined=np.concatenate(local)
        volume.points(s/f"{tag}-C00.ply",combined)
        if key in BOX:
            shape=np.array(result["shape"].get(result["registration"],result["shape"]["R01"]))
        else:
            shape=None
        # Remove one spatial patch from all frames, retaining measured points from that patch only for scoring.
        median=np.median(combined,axis=0)
        q=max(4.,np.percentile(combined[:,2],53))
        drop=[(p[:,0]>median[0]-.8)&(p[:,2]>q)&(p[:,1]>median[1]-8) for p in local]
        held=np.concatenate([p[k] for p,k in zip(local,drop)])
        training=[p[~m] for p,m in zip(local,drop)]
        clean=[{**a,"xyz":a["xyz"][~m],"uv":a["uv"][~m]} for a,m in zip(frames,drop)]
        assert len(held)>40 and len(combined)-len(held)>500
        volume.points(s/f"{tag}-C00-T1-reference.ply",held)
        representations=[]
        for stress,data,cameras in ((0,local,frames),(1,training,clean)):
            variants=predict(tag,data,cameras,poses,cal,center,B,shape,bool(stress))
            source=np.concatenate(data)
            sampletree=cKDTree(source)
            for method,(mesh,value,closed,pitch,decision) in variants.items():
                stem=f"{tag}-{method}-T{stress}"
                row={"id":tag,"case":key,"test":stress,"method":method,"watertight":closed,"volume_mm_star_3":value,
                     "pitch_mm_star":pitch,"input_points":len(source),"removed_for_validation":len(held),**decision}
                if mesh is not None:
                    points=np.asarray(mesh.sample_points_uniformly(number_of_points=13000).points).copy()
                    unknown=sampletree.query(points)[0]>2.5
                    color=np.tile([.21,.53,.74],(len(points),1))
                    color[unknown]=[.94,.44,.18]
                    row["unsupported_area_sample_fraction"]=float(unknown.mean())
                    err=cKDTree(points).query(source[::3])[0]
                    row["observed_to_surface_P50"]=float(np.median(err))
                    row["observed_to_surface_P95"]=float(np.percentile(err,95))
                    if stress:
                        d=cKDTree(points).query(held[::2])[0]
                        row.update(hidden_recall_2=float(np.mean(d<=2)),hidden_P95=float(np.percentile(d,95)))
                    volume.points(s/f"{stem}.ply",points,color)
                    if not o3d.io.write_triangle_mesh(str(s/f"{stem}-mesh.ply"),mesh):
                        raise IOError(stem)
                rows.append(row)
                representations.append(row)
            if stress:volume.points(s/f"{tag}-C00-T1.ply",source)
        dump(s/f"{tag}-C00.json",{"source":result["source"],"case":key,"id":tag,"rows":representations,
             "table_coordinate_transform":{"up":result["axis"],"basis":B,"origin_in_ref_camera":center},
             "T0":"all valid unique captures after segmentation","T1":"spatial patch is masked across all source views",
             "volume_is_provisional":True})
        index[tag]=key
        print(tag,key,len(combined),len(held),round(perf_counter()-begin,1),flush=True)
    with (OUT/"E002-01.csv").open("w",newline="",encoding="utf-8-sig") as f:
        wr=csv.DictWriter(f,fieldnames=list(dict.fromkeys(k for o in rows for k in o)));wr.writeheader();wr.writerows(rows)
    dump(OUT/"E002-02.json",{"cases":index,"methods":MODELS})


if __name__=="__main__":
    main()
