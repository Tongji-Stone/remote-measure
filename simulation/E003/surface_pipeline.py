"""E003 white meshes, local completion, and held-out surface benchmarks."""
import importlib.util
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
from time import perf_counter

import cv2
import numpy as np
import open3d as o3d
from scipy import ndimage
from scipy.spatial import cKDTree, ConvexHull, QhullError
from scipy.spatial.transform import Rotation

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
REG=ROOT/"algorithm/scene-fusion/E003"
sys.path.insert(0,str(REG))
from registration_pipeline import OUT as PREV, PRIOR, BOX, KEYS, load, load_case, table_basis, dump, csv_write, ply, transform
from registration_experiment import training_mask
from registration import fit_cuboid
from geometry import (axis_sensitivity, box_analysis, box_mesh, camera_foot_guides,
                      charger_planes, patch_adjust, smooth_voxel_surface)

spec=importlib.util.spec_from_file_location("e003_voxel",HERE.parent/"run.py")
voxel=importlib.util.module_from_spec(spec)
sys.modules[spec.name]=voxel
spec.loader.exec_module(voxel)
OUT=HERE/"results"


def load_frames(key, tag):
    meta,_,_,s,cal=load_case(key)
    folder=PREV/tag
    entry=load(folder/f"{tag}-01.json")
    poses=np.array(entry["poses"])
    center=np.array(entry["center"])
    basis=table_basis(np.array(entry["axis"]))
    frames=[]
    for i,(scan,T) in enumerate(zip(s,poses)):
        with np.load(folder/f"{tag}-V{i:02d}.npz") as f:
            inds=f["mask"]
        p=(transform(scan["xyz"][inds],T)-center)@basis
        good=(p[:,2]>=-2.8)&np.isfinite(p).all(1)
        origins=(T[:3,3]-center)@basis
        frames.append({"pts":p[good],"pixels":scan["uv"][inds][good],"depth":scan["xyz"][inds,2][good],
            "camera":origins,"pose":T})
    return frames,poses,basis,center,cal,entry


def oriented(frames):
    p,n=[],[]
    for f in frames:
        if len(f["pts"])<35:
            continue
        cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(f["pts"])).voxel_down_sample(1.15)
        cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=4.9,max_nn=35))
        cloud.orient_normals_towards_camera_location(f["camera"])
        p.append(np.asarray(cloud.points).copy())
        n.append(np.asarray(cloud.normals).copy())
    return np.concatenate(p),np.concatenate(n)


def select_bottom(p):
    lo=np.quantile(p[:,2],.04)
    high=np.quantile(p[:,2],.96)
    return max(4.0,min(lo+7.,lo+.21*(high-lo)))


def remove_free(points,frames,center,basis,cal,min_votes=2):
    votes=np.zeros(len(points),np.uint8)
    size=(306,256)
    for f in frames:
        if len(f["depth"])<60:
            continue
        q=transform(points@basis.T+center,np.linalg.inv(f["pose"]))
        uv=cv2.projectPoints(q,np.zeros(3),np.zeros(3),cal["cam_int"],cal["cam_dist"])[0][:,0]
        pix=np.clip(np.rint(uv/8),-1,500).astype(int)
        src=np.rint(f["pixels"]/8).astype(int)
        good=(src[:,0]>=0)&(src[:,0]<size[0])&(src[:,1]>=0)&(src[:,1]<size[1])
        img=np.full(size[0]*size[1],np.inf)
        np.minimum.at(img,src[good,1]*size[0]+src[good,0],f["depth"][good])
        v=(pix[:,0]>=0)&(pix[:,0]<size[0])&(pix[:,1]>=0)&(pix[:,1]<size[1])&(q[:,2]>0)
        deep=img[pix[v,1]*size[0]+pix[v,0]]
        votes[v]+=(deep<1900)&(q[v,2]<deep-4.)
    return votes<min_votes


def polar_patch(p):
    # Extrapolate only the lowest measured contours, independently along the azimuths.
    high=np.quantile(p[:,2],.96)
    mid=p[(p[:,2]>.28*high)&(p[:,2]<.68*high)]
    c=np.median(mid[:,:2],axis=0) if len(mid)>40 else np.median(p[:,:2],axis=0)
    xy=p[:,:2]-c
    angles=np.mod(np.arctan2(xy[:,1],xy[:,0]),2*np.pi)
    radius=np.linalg.norm(xy,axis=1)
    xs=np.linspace(0,2*np.pi,32,endpoint=False)
    z=np.full(len(xs),np.nan); rr=z.copy(); slope=z.copy()
    for i,a in enumerate(xs):
        m=(abs(np.angle(np.exp(1j*(angles-a))))<.23)&(p[:,2]<high*.73)&(radius>4)
        if m.sum()<25:
            continue
        uv=p[m,2]
        rad=radius[m]
        z[i]=max(2.2,np.quantile(uv,.13))
        rr[i]=np.median(rad[uv<=np.quantile(uv,.28)])
        take=uv<=np.quantile(uv,.62)
        if np.ptp(uv[take])>=2.5:
            slope[i]=np.clip(np.polyfit(uv[take],rad[take],1)[0],.06,1.75)
        else:
            slope[i]=.57
    valid=np.isfinite(z)
    if valid.sum()<12:
        return footprint_patch(p)
    def interpolate(a):
        return ndimage.gaussian_filter1d(np.interp(xs,xs[valid],a[valid],period=2*np.pi),.85,mode="wrap")
    z,rr,slope=map(interpolate,(z,rr,slope))
    r0=np.maximum(rr*.35,rr-slope*np.maximum(z,3.5))
    # Small local bottom concavity is modeled by keeping an almost flat contact patch.
    u=np.c_[np.cos(xs),np.sin(xs)]
    pts=[]; normal=[]
    for frac in (.12,.31,.50,.69,.86):
        rr1=r0+(rr-r0)*frac
        z1=.62+np.maximum(z-.62,0)*frac
        pts.append(np.c_[c+rr1[:,None]*u,z1])
        normal.append(np.c_[u,-np.clip(slope,.25,1.4)])
    for t in (.08,.30,.52,.73,.91):
        pts.append(np.c_[c+(t*r0)[:,None]*u,.5+(.3*(1-t))*np.ones(len(u))])
        normal.append(np.tile([0.,0.,-1.],(len(u),1)))
    out=np.concatenate(pts);N=np.concatenate(normal)
    N/=np.linalg.norm(N,axis=1,keepdims=True)
    return out,N,{"kind":"lower_azimuthal_contours","supported_angle_bins":int(valid.sum()),"anchor_height_median":np.median(z)}


def footprint_patch(p):
    level=select_bottom(p)
    low=p[p[:,2]<level]
    if len(low)<25:
        return np.empty((0,3)),np.empty((0,3)),{"kind":"low_components","components":0}
    lo=low[:,:2].min(0)-4; hi=low[:,:2].max(0)+4
    pitch=1.35
    size=(np.ceil((hi-lo)/pitch).astype(int)+1)
    xy=lo+np.indices(tuple(size)).reshape(2,-1).T*pitch
    tree=cKDTree(low[:,:2])
    d,j=tree.query(xy,k=5)
    mask=(d[:,0]<2.8)&(d[:,-1]<5.0)
    mask=mask.reshape(size)
    mask=ndimage.binary_fill_holes(ndimage.binary_closing(mask,iterations=1))
    lab,num=ndimage.label(mask)
    counts=np.bincount(lab.ravel()); counts[0]=0
    selected=counts>=12
    mask=selected[lab]
    if not mask.any():
        return np.empty((0,3)),np.empty((0,3)),{"kind":"low_components","components":0}
    interior=xy[mask.ravel()]
    i=mask&~ndimage.binary_erosion(mask)
    edge=xy[i.ravel()]
    near=tree.query(edge,k=min(8,len(low)))[1]
    edge_height=np.clip(np.mean(low[near,2],axis=1),1.4,level)
    dist=ndimage.distance_transform_edt(mask)-ndimage.distance_transform_edt(~mask)
    gradients=np.stack(np.gradient(dist),axis=-1)[i]
    gradients/=-np.maximum(np.linalg.norm(gradients,axis=1,keepdims=True),1e-4)
    P=np.concatenate([np.c_[interior,np.full(len(interior),.55)],
         np.c_[edge,.45+edge_height*.48],np.c_[edge,.45+edge_height*.78]])
    N=np.concatenate([np.tile([0.,0.,-1.],(len(interior),1)),
        np.c_[gradients*.85,np.full(len(edge),-.53)],np.c_[gradients*.94,np.full(len(edge),-.34)]])
    return P,N,{"kind":"low_components","components":int(selected.sum()),"anchor_height":level}


def poisson(p,n):
    with TemporaryDirectory(prefix="E003-poisson-") as folder:
        d=Path(folder)
        np.savez(d/"oriented.npz",points=np.asarray(p,dtype=float),normals=np.asarray(n,dtype=float))
        for depth in (8,7):
            child=subprocess.run([sys.executable,str(HERE/"poisson_mesh.py"),folder,str(depth)],
                capture_output=True,text=True,timeout=60)
            if child.returncode==0:
                return o3d.io.read_triangle_mesh(str(d/"surface.ply")),np.load(d/"density.npy")
            print("Poisson retry",depth,child.returncode,child.stderr[-250:],flush=True)
        raise RuntimeError("Screened Poisson solver stopped at both depths")


def trim_surface(mesh,density,p):
    q=np.asarray(mesh.vertices)
    dist=cKDTree(p).query(q)[0]
    keep=(dist<=4.)&(density>=np.quantile(density,.07))&(q[:,2]>=-2.)
    mesh.remove_vertices_by_mask(~keep)
    mesh.remove_unreferenced_vertices()
    labels,count,area=mesh.cluster_connected_triangles()
    if len(count)>1:
        valid=(np.asarray(count)>=95)&(np.asarray(area)>=7.)
        valid[np.argmax(area)]=True
        mesh.remove_triangles_by_mask(~valid[np.asarray(labels)])
        mesh.remove_unreferenced_vertices()
    return mesh


def sealed_model(mesh,p,virtual,footprint=None):
    """Re-isosurface within a bounded, sample-supported domain if Poisson extrapolates."""
    support=np.concatenate([p,virtual]) if len(virtual) else p
    lo=np.percentile(support,.2,axis=0)-4.2
    hi=np.percentile(support,99.8,axis=0)+4.2
    lo[2]=0.
    step=1.20
    voxel.ORIGIN=lo
    voxel.SIZE=tuple(np.maximum(10,np.ceil((hi-lo)/step).astype(int)))
    voxel.PITCH=step
    grid=lo+(np.indices(voxel.SIZE).reshape(3,-1).T+.5)*step
    try:
        h=ConvexHull(support[::max(1,len(support)//3800)])
        inside=np.ones(len(grid),bool)
        # Hull is a loose exterior bound, with 3.4 mm padding. It is never the fitted shape.
        for i in range(0,len(h.equations),28):
            u=h.equations[i:i+28]
            pos=np.flatnonzero(inside)
            keep=(grid[pos]@u[:,:3].T+u[:,3]<=3.4).all(1)
            inside[pos[~keep]]=False
    except QhullError:
        inside=np.ones(len(grid),bool)
    s=o3d.t.geometry.RaycastingScene(nthreads=2)
    s.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    ids=np.flatnonzero(inside)
    for keys in np.array_split(ids,max(1,int(len(ids)/65000)+1)):
        q=o3d.core.Tensor(grid[keys].astype(np.float32))
        inside[keys]=s.compute_occupancy(q,nthreads=2,nsamples=3).numpy()>.5
    if footprint is not None:
        # Intervene only at the height of the observed flippers. Keep the torso geometry unchanged.
        near_floor=(grid[:,2]<=3.45)
        ix=np.flatnonzero(near_floor)
        xy=grid[ix,:2]
        hull=ConvexHull(footprint[:,:2])
        select=(xy@hull.equations[:,:2].T+hull.equations[:,2]<=2.4).all(1)
        footprint_dist=cKDTree(footprint[:,:2]).query(xy)[0]
        inside[ix[select&(footprint_dist>1.6)]]=False
        inside[ix[footprint_dist<=.95]]=True
    m=inside.reshape(voxel.SIZE)
    m=ndimage.binary_closing(np.pad(m,1),iterations=1)[1:-1,1:-1,1:-1]|m
    lab,_=ndimage.label(m)
    counts=np.bincount(lab.ravel())
    if len(counts)<2:
        return None
    counts[0]=0
    m=ndimage.binary_fill_holes(lab==np.argmax(counts))
    smooth=smooth_voxel_surface(m,lo,step)
    if len(smooth.triangles)>100 and smooth.is_watertight():
        return smooth
    repaired=voxel.voxel_mesh(m.ravel())
    if len(repaired.triangles)<100:
        return None
    smooth=repaired.filter_smooth_taubin(number_of_iterations=5,lambda_filter=.38,mu=-.40)
    return smooth if smooth.is_watertight() else repaired


def save_mesh(mesh,path):
    is_box="-M04-" in path.name
    if not is_box:
        mesh.compute_vertex_normals()
    mesh.paint_uniform_color([.95,.95,.95])
    if not o3d.io.write_triangle_mesh(str(path),mesh,write_vertex_normals=not is_box):
        raise IOError(path)
    if is_box and "-T0" in path.name:
        # OBJ uses per-face normal indices while retaining shared corner vertices.
        mesh.compute_triangle_normals()
        normals,which=np.unique(np.round(np.asarray(mesh.triangle_normals),10),axis=0,return_inverse=True)
        with path.with_suffix(".obj").open("w") as f:
            f.write("# E003 white geometric model; normals are discontinuous across box edges\n")
            for v in np.asarray(mesh.vertices):
                f.write("v %.7f %.7f %.7f\n"%tuple(v))
            for n in normals:
                f.write("vn %.8f %.8f %.8f\n"%tuple(n))
            for indices,normal in zip(np.asarray(mesh.triangles)+1,which+1):
                f.write("f "+" ".join(f"{int(i)}//{int(normal)}" for i in indices)+"\n")


def distance_to(mesh,p):
    sc=o3d.t.geometry.RaycastingScene(nthreads=2)
    sc.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    return sc.compute_distance(o3d.core.Tensor(p.astype(np.float32)),nthreads=2).numpy()


def visible_slice_scores(mesh,section):
    xy,reference=section
    sc=o3d.t.geometry.RaycastingScene(nthreads=2)
    sc.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    pts=np.c_[xy,np.full(len(xy),2.2)].astype(np.float32)
    occupied=sc.compute_occupancy(o3d.core.Tensor(pts),nthreads=2,nsamples=3).numpy()>.5
    common=np.sum(reference&occupied)
    return {"visible_coverage":float(common/max(1,np.sum(reference))),
        "section_overlap_iou":float(common/max(1,np.sum(reference|occupied))),
        "occupied_fraction_outside_visible_outline":float(np.sum(occupied&~reference)/max(1,np.sum(occupied)))}


def measurements(mesh,p,held,seed=472):
    o3d.utility.random.seed(seed)
    mp=np.asarray(mesh.sample_points_uniformly(number_of_points=16000).points).copy()
    closed=mesh.is_watertight()
    stats={"vertices":len(mesh.vertices),"triangles":len(mesh.triangles),"watertight":closed,
        "volume_mm_star3":abs(mesh.get_volume()) if closed and mesh.is_orientable() else None,
        "unknown_surface_fraction":float(np.mean(cKDTree(p).query(mp)[0]>2.5))}
    d=distance_to(mesh,p[::max(1,len(p)//16000)])
    stats.update(observed_P50=float(np.median(d)),observed_P95=float(np.percentile(d,95)))
    if held is not None:
        d=distance_to(mesh,held[::max(1,len(held)//13000)])
        stats.update(hidden_recall_2=float(np.mean(d<=2)),hidden_P95=float(np.percentile(d,95)))
    return stats


def fit_box(frames):
    data=[{"train":q["pts"]} for q in frames]
    _,p=fit_cuboid(data,np.array([0.,0.,1.]),np.zeros(3),np.repeat(np.eye(4)[None],len(data),axis=0))
    return p


def mask_observations(frames):
    p=np.concatenate([f["pts"] for f in frames])
    below=p[:,2]<np.quantile(p[:,2],.55)
    cutoff=np.quantile(p[below,0],.61)
    z=np.quantile(p[:,2],.55)
    side=np.quantile(p[below,1],.23)
    retained=[]; held=[]
    for f in frames:
        q=f["pts"]
        test=(q[:,0]>cutoff)&(q[:,1]>side)&(q[:,2]<z)
        held.append(q[test])
        retained.append({**f,**{k:f[k][~test] for k in ("pts","pixels","depth")}})
    return retained,np.concatenate(held)


def run_case(key,tag,folder,frames,poses,basis,center,cal,entry):
    rows=[]; detail={}; diagnosis={}
    all_measured=np.concatenate([f["pts"] for f in frames])
    ply(folder/f"{tag}-C00.ply",all_measured)
    # Photo tiles remain independent of the fit for the measured plane / corner diagnosis.
    train=[f["pts"][training_mask(f["pixels"])] for f in frames]
    validation=[f["pts"][~training_mask(f["pixels"])] for f in frames]
    if key in BOX:
        theta=fit_box([{"pts":t} for t in train])
        number=tag.replace("E003","E002")
        old=load(PRIOR/"clouds"/number/f"{number}-03.json")
        # Move the same measured samples back to the earlier alignment.
        e002=[(transform((p@basis.T+center),np.array(n)@np.linalg.inv(T))-center)@basis
              for p,n,T in zip(validation,old["selected_poses"],poses)]
        diagnosis={"kind":"box","local_box_parameters":theta,"heldout_E002":box_analysis(train,e002,theta),
            "heldout_E003":box_analysis(train,validation,theta),
            "axis_sensitivity_fixed_geometry":axis_sensitivity(train,validation,theta,poses,basis)}
    if "charger" in key:
        planes,diagnosis=charger_planes(train)
    T1, held=mask_observations(frames)
    assert len(held)>80
    ply(folder/f"{tag}-C00-T1-reference.ply",held)
    for test,source,hidden in ((0,frames,None),(1,T1,held)):
        start=perf_counter()
        all_pts=np.concatenate([f["pts"] for f in source])
        p,n=oriented(source)
        raw,density=poisson(p,n)
        open_surface=trim_surface(raw,density,all_pts)
        candidates={"M01":open_surface}
        if key in BOX:
            theta=fit_box(source)
            candidates["M04"]=box_mesh(theta)
            detail[str(test)]={"box_parameters":theta}
        else:
            virtual,vn,aux=polar_patch(all_pts) if key=="apple" else footprint_patch(all_pts)
            if len(virtual):
                accept=remove_free(virtual,source,center,basis,cal)
                accept&=cKDTree(all_pts).query(virtual)[0]>1.25
                virtual,vn=virtual[accept],vn[accept]
                ply(folder/f"{tag}-C01-T{test}.ply",virtual)
            corrected,nn=p,n
            if "charger" in key:
                if test:
                    use_planes,_=charger_planes([f["pts"] for f in source])
                else:
                    use_planes=planes
                corrected,nn,applied=patch_adjust(p,n,use_planes)
                aux.update(regularized_samples=int(np.sum(applied>.15)),
                           maximum_local_move=float(np.linalg.norm(corrected-p,axis=1).max()))
                if not test:
                    diagnosis.update({k:aux[k] for k in ("regularized_samples","maximum_local_move")})
                    if use_planes:
                        before=[]; after=[]
                        for original,target,c,tree,B in use_planes:
                            V=np.concatenate(validation)
                            choose=(tree.query((V-c)@B)[0]<=1.8)&(np.abs((V-c)@original)<1.45)
                            if choose.sum()>55:
                                A=V[choose]
                                U,_,_=patch_adjust(A,np.tile(original,(len(A),1)),use_planes)
                                before.append(np.linalg.svd(A-A.mean(0),full_matrices=False)[2][-1])
                                after.append(np.linalg.svd(U-U.mean(0),full_matrices=False)[2][-1])
                        if len(after)==2:
                            angle=lambda x:np.degrees(np.arccos(np.clip(abs(x[0]@x[1]),0.,1.)))
                            diagnosis.update(heldout_pre_angle=angle(before),heldout_regularized_angle=angle(after))
            mix=np.r_[corrected,virtual]; mixn=np.r_[nn,vn]
            augmented,_=poisson(mix,mixn)
            candidates["M02"]=augmented
            bounded=sealed_model(augmented,corrected,virtual)
            if bounded is not None:
                candidates["M03"]=bounded
            aux.update(virtual_points=len(virtual))
            if key=="minion" and test==0:
                rules=load(HERE/"minion_footprints.json")
                flipper,fn,hint,held_slice=camera_foot_guides(source,center,basis,cal,rules)
                accept=remove_free(flipper,source,center,basis,cal)&(cKDTree(all_pts).query(flipper)[0]>.75)
                flipper,fn=flipper[accept],fn[accept]
                if len(flipper):
                    # In the footprint neighborhood the two image-supported flippers replace generic low-band seeds.
                    vicinity=cKDTree(flipper[:,:2]).query(virtual[:,:2])[0]
                    remaining=(vicinity>5.2)|(virtual[:,2]>3.8)
                    seeds=np.r_[virtual[remaining],flipper]
                    guided,_=poisson(np.r_[p,seeds],np.r_[n,vn[remaining],fn])
                    closed=sealed_model(guided,p,seeds)
                    if closed is not None:
                        candidates["M05"]=closed
                    constrained=sealed_model(guided,p,seeds,footprint=flipper)
                    if constrained is not None:
                        candidates["M06"]=constrained
                    if bounded is not None and closed is not None:
                        hint["heldout_section_M03"]=visible_slice_scores(bounded,held_slice)
                        hint["heldout_section_M05"]=visible_slice_scores(closed,held_slice)
                    if constrained is not None:
                        hint["heldout_section_M06"]=visible_slice_scores(constrained,held_slice)
                    ply(folder/f"{tag}-C02-T0.ply",flipper)
                aux["photo_support"]={**hint,"accepted_samples":len(flipper)}
            detail[str(test)]=aux
        for method,mesh in candidates.items():
            filename=folder/f"{tag}-{method}-T{test}.ply"
            save_mesh(mesh,filename)
            if method=="M01" and test==0:
                measured=PREV/tag/filename.name
                measured.unlink(missing_ok=True)
                measured.hardlink_to(filename)
            row=measurements(mesh,all_pts,hidden)
            rows.append({"id":tag,"method":method,"test":test,"input_points":len(all_pts),
               "reference_removed":len(held),**row})
        print(tag,"T"+str(test),len(all_pts),len(held),round(perf_counter()-start,1),flush=True)
    dump(folder/f"{tag}-02.json",{"diagnosis":diagnosis,"local_completion":detail,"center":center,"basis":basis})
    return rows,diagnosis


def main():
    OUT.mkdir(exist_ok=True)
    o3d.utility.random.seed(530)
    cv2.setNumThreads(2)
    rows=[]; diagnostics={}
    subset=set(sys.argv[1:])
    if subset=={"--audit-foot"}:
        tag="E003-S01"
        f,_,B,center,cal,_=load_frames("minion",tag)
        _,_,_,section=camera_foot_guides(f,center,B,cal,load(HERE/"minion_footprints.json"))
        folder=OUT/tag
        path=folder/f"{tag}-02.json"
        info=load(path)
        foot=info["local_completion"]["0"]["photo_support"]
        for variant in ("M03","M05","M06"):
            mesh=o3d.io.read_triangle_mesh(str(folder/f"{tag}-{variant}-T0.ply"))
            foot.pop(f"heldout_section_recall_{variant}",None)
            foot[f"heldout_section_{variant}"]=visible_slice_scores(mesh,section)
        print(foot)
        dump(path,info)
        return
    if subset=={"--collect"}:
        import csv
        for j in range(1,len(KEYS)+1):
            tag=f"E003-S{j:02d}"
            folder=OUT/tag
            with (folder/f"{tag}-01.csv").open(encoding="utf-8-sig") as file:
                rows.extend(csv.DictReader(file))
            diagnostics[tag]=load(folder/f"{tag}-02.json")["diagnosis"]
        csv_write(OUT/"E003-01.csv",rows)
        dump(OUT/"E003-02.json",diagnostics)
        return
    for j,key in enumerate(KEYS,1):
        tag=f"E003-S{j:02d}"
        if subset and tag not in subset:
            continue
        folder=OUT/tag
        folder.mkdir(exist_ok=True)
        frames,poses,basis,center,cal,entry=load_frames(key,tag)
        new,diag=run_case(key,tag,folder,frames,poses,basis,center,cal,entry)
        diagnostics[tag]=diag
        csv_write(folder/f"{tag}-01.csv",new)
        rows.extend(new)
    if rows and not subset:
        csv_write(OUT/"E003-01.csv",rows)
        dump(OUT/"E003-02.json",diagnostics)


if __name__=="__main__":
    main()
