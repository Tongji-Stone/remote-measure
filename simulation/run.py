"""Occlusion and volumetric completion benchmark with geometric ground truth."""
from dataclasses import dataclass
from pathlib import Path
import csv
import json
import os
import subprocess
import sys
from tempfile import TemporaryDirectory
from time import perf_counter

import cv2
import numpy as np
import open3d as o3d
from scipy.ndimage import gaussian_filter
from scipy.spatial import ConvexHull, cKDTree

HERE=Path(__file__).resolve().parent
OUT=HERE/"results"
PITCH=1.5
SIZE=(80,72,51)
ORIGIN=np.array([-60.,-54.,0.])
GRID=ORIGIN+(np.indices(SIZE).reshape(3,-1).T+.5)*PITCH
PI=np.pi
ID=o3d.core.Dtype.Float32
SEED=441

@dataclass
class Camera:
    origin: np.ndarray
    axes: np.ndarray
    focal: float=245.
    width: int=224
    height: int=196

    def screen(self, xyz):
        c=(xyz-self.origin)@self.axes
        uv=c[:,:2]*(self.focal/c[:,2:])+[self.width/2,self.height/2]
        return uv,c[:,2]

    def rays(self):
        y,x=np.indices((self.height,self.width))
        uv=(np.stack([x,y],-1)-[self.width/2,self.height/2])/self.focal
        directions=np.concatenate([uv,np.ones((*uv.shape[:-1],1))],-1)@self.axes.T
        return np.concatenate([np.broadcast_to(self.origin,directions.shape),directions],-1).astype(np.float32)


def camera(angle, elevation):
    a,e=np.deg2rad([angle,elevation])
    target=np.array([0.,0.,26.])
    pos=target+198*np.array([np.cos(a)*np.cos(e),np.sin(a)*np.cos(e),np.sin(e)])
    v=(target-pos)/np.linalg.norm(target-pos)
    h=np.cross(v,[0,0,1.])
    h/=np.linalg.norm(h)
    return Camera(pos,np.column_stack((h,np.cross(v,h),v)))


def height(case, xy):
    x,y=xy.T
    if case=="A_box":
        a=np.deg2rad(17)
        X=x*np.cos(a)+y*np.sin(a)
        Y=y*np.cos(a)-x*np.sin(a)
        return 51.*((abs(X)<36)&(abs(Y)<27))
    if case=="B_features":
        body=(abs(x)<39)&(abs(y)<31)
        apron=x>23
        z=np.where(apron,4.5,48.)*body
        groove=(abs(x+9)<10)&(abs(y+16)<8)
        blind=(x+16)**2+(y-13)**2<8.5**2
        through=(x-30)**2+(y-10)**2<7.5**2
        z-=4.5*(groove|blind)*body
        return z*(~(through&apron))
    q=(x/45)**2+(y/32)**2
    z=(24 + 23*np.exp(-((x+14)/23)**2-((y-6)/16)**2)
          + 17*np.exp(-((x-19)/17)**2-((y+9)/15)**2)+4*np.cos(x/13)*np.sin(y/10))
    return z*(q<=1)


def analytic_volume(case):
    # Midpoint quadrature at 0.25 mm independently from reconstruction cells.
    y,x=np.mgrid[-54+.125:54:.25,-60+.125:60:.25]
    return float(height(case,np.c_[x.ravel(),y.ravel()]).sum()*.25**2)


def voxel_mesh(inside):
    """Extract one oriented isosurface using shared-face marching tetrahedra."""
    volume=np.pad(inside.reshape(SIZE),1)
    cube=np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]])
    cubes=np.stack([volume[x:x+volume.shape[0]-1,y:y+volume.shape[1]-1,z:z+volume.shape[2]-1]
                    for x,y,z in cube],axis=-1)
    loc=np.argwhere(cubes.any(-1)&~cubes.all(-1))
    signs=cubes[tuple(loc.T)]
    tetrahedra=((0,5,1,6),(0,1,2,6),(0,2,3,6),(0,3,7,6),(0,7,4,6),(0,4,5,6))
    all_faces=[]
    for tet in tetrahedra:
        bit=signs[:,tet].astype(np.int32)@np.array([1,2,4,8])
        offset=cube[list(tet)]
        for state in range(1,15):
            bad=[i for i in range(4) if not state&(1<<i)]
            ok=[i for i in range(4) if state&(1<<i)]
            base=loc[bit==state]
            if not len(base): continue
            if len(ok)==1:
                polygon=offset[ok[0]]+offset[bad]
            elif len(bad)==1:
                polygon=offset[bad[0]]+offset[ok]
            else:
                a,b=ok;c,d=bad
                polygon=np.array([offset[a]+offset[c],offset[a]+offset[d],offset[b]+offset[d],offset[b]+offset[c]])
            outward=offset[bad].mean(0)-offset[ok].mean(0)
            if np.cross(polygon[1]-polygon[0],polygon[2]-polygon[0])@outward<0:
                polygon=polygon[::-1]
            for triangle in ([0,1,2],[0,2,3]) if len(polygon)==4 else ([0,1,2],):
                all_faces.append(2*base[:,None,:]+polygon[triangle])
    if not all_faces: return o3d.geometry.TriangleMesh()
    node,inv=np.unique(np.concatenate(all_faces).reshape(-1,3),axis=0,return_inverse=True)
    return o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(ORIGIN+(node/2.-.5)*PITCH),
                                     o3d.utility.Vector3iVector(inv.reshape(-1,3)))


def scene(mesh):
    cast=o3d.t.geometry.RaycastingScene()
    cast.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    return cast


def observe(cast,camera_cfg,laser,shadow,rng):
    ray=camera_cfg.rays()
    hit=cast.cast_rays(o3d.core.Tensor(ray))["t_hit"].numpy()
    y,x=np.nonzero(np.isfinite(hit))
    p=ray[y,x,:3]+hit[y,x,None]*ray[y,x,3:]
    d=p-laser.origin
    length=np.linalg.norm(d,axis=1)
    origin=np.broadcast_to(laser.origin,p.shape)
    line=np.c_[origin,d/length[:,None]].astype(np.float32)
    illumination=cast.cast_rays(o3d.core.Tensor(line))["t_hit"].numpy()>=length-.82
    if shadow:
        # Deliberate lack of projected texture over a contiguous top patch.
        blocked=(p[:,0]>-24)&(p[:,0]<9)&(p[:,1]>4)&(p[:,1]<25)&(p[:,2]>27)
        illumination &= ~blocked
    z=np.full(hit.shape,np.nan)
    z[y[illumination],x[illumination]]=hit[y[illumination],x[illumination]]
    observed=p[illumination]+rng.normal(0.,.18,p[illumination].shape)
    mask=cv2.dilate(np.isfinite(hit).astype(np.uint8),np.ones((3,3),np.uint8)).astype(bool)
    return {"camera":camera_cfg,"observed":observed,"silhouette":mask,"depth":z,
            "silhouette_samples":int(np.isfinite(hit).sum()),"depth_samples":len(observed)}


def projections(xyz,view):
    c=view["camera"]
    uv,z=c.screen(xyz)
    x,y=np.rint(uv).astype(int).T
    inside=(x>=0)&(x<c.width)&(y>=0)&(y<c.height)&(z>0)
    return x,y,z,inside


def carve(views, use_depth):
    active=np.ones(len(GRID),bool)
    for v in views:
        index=np.flatnonzero(active)
        x,y,z,inside=projections(GRID[index],v)
        selected=np.flatnonzero(inside)
        supported=np.zeros(len(index),bool)
        supported[selected]=v["silhouette"][y[selected],x[selected]]
        if use_depth:
            d=v["depth"][y[selected],x[selected]]
            supported[selected] &= ~(np.isfinite(d)&(z[selected]<d-1.6))
        active[index]=supported
    return active


def convex(pts):
    p=np.concatenate([pts,np.c_[pts[:,:2],np.zeros(len(pts))]])
    hull=ConvexHull(p)
    result=np.ones(len(GRID),bool)
    for equation in hull.equations:
        result &= GRID@equation[:3] <= -equation[3]+.28
    return result


def fitted_cuboid(pts):
    _,_,degree=cv2.minAreaRect(pts[:,:2].astype(np.float32))
    def transform(degree):
        a=np.deg2rad(degree)
        return np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
    options=[transform(a) for a in np.arange(degree-6,degree+6.1,.45)]
    def metric(r):
        q=np.percentile(pts[:,:2]@r,[3.5,96.5],axis=0)
        return np.prod(q[1]-q[0]),q
    R=min(options,key=lambda r:metric(r)[0])
    _,(lo,hi)=metric(R)
    z=float(np.percentile(pts[:,2],98))
    model=o3d.geometry.TriangleMesh.create_box(*(hi-lo),z)
    model.translate(np.r_[lo,0.])
    pose=np.eye(4);pose[:2,:2]=R
    model.transform(pose)
    uv=GRID[:,:2]@R
    occupied=((uv>=lo)&(uv<=hi)).all(1)&(GRID[:,2]<z)
    return model,occupied


def infer_height(pts,footprint):
    xy=GRID.reshape((*SIZE,3))[:,:,0,:2].reshape(-1,2)
    # Robust per-cell upper envelope from depth, with local interpolation where texture is absent.
    ij=np.floor((pts[:,:2]-ORIGIN[:2])/PITCH).astype(int)
    valid=(ij>=0).all(1)&(ij<np.array(SIZE[:2])).all(1)
    linear=np.ravel_multi_index(ij[valid].T,SIZE[:2])
    order=np.argsort(linear)
    ids,first,count=np.unique(linear[order],return_index=True,return_counts=True)
    z=pts[valid,2][order]
    values=np.array([np.quantile(z[start:start+n],.86) for start,n in zip(first,count)])
    # Lower vertical-side samples can otherwise pull down interpolation close to the boundary.
    nearest=cKDTree(xy[ids]).query(xy,k=min(7,len(ids)))[1]
    local=values[nearest]
    h=np.quantile(local,.73,axis=1)
    h[ids]=values
    for _ in range(2):
        fallback=gaussian_filter(h.reshape(SIZE[:2]),1.03).ravel()
        known=np.zeros(len(xy),bool);known[ids]=True
        h[~known]=.63*h[~known]+.37*fallback[~known]
    return np.repeat(footprint,SIZE[2])&(GRID[:,2]<=np.repeat(h,SIZE[2]))


def poisson_mesh(views):
    ps=[]; ns=[]
    for v in views:
        p=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(v["observed"])).voxel_down_sample(1.5)
        p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=4.4,max_nn=25))
        p.orient_normals_towards_camera_location(v["camera"].origin)
        ps.append(np.asarray(p.points).copy());ns.append(np.asarray(p.normals).copy())
    c=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.concatenate(ps)))
    c.normals=o3d.utility.Vector3dVector(np.concatenate(ns))
    c=c.voxel_down_sample(1.3)
    c.normalize_normals()
    with TemporaryDirectory(prefix="completion-poisson-") as work:
        work=Path(work)
        source,target=work/"cloud.ply",work/"mesh.ply"
        if not o3d.io.write_point_cloud(str(source),c):
            raise OSError(source)
        for depth in (6,5):
            call=subprocess.run([sys.executable,str(HERE/"poisson_worker.py"),str(source),str(target),str(depth)],
                                 env={**os.environ,"OMP_NUM_THREADS":"1","OPENBLAS_NUM_THREADS":"1"},
                                 capture_output=True,text=True,timeout=45)
            if call.returncode==0 and target.exists():
                return o3d.io.read_triangle_mesh(str(target)),json.loads(call.stdout.strip().splitlines()[-1])
            print("Poisson retry",depth,call.returncode,call.stderr[-220:],flush=True)
    return None,{"watertight":False,"volume":None,"depth":None}


def points(path, xyz, rgb=None):
    pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz.astype(float)))
    if rgb is not None:
        pc.colors=o3d.utility.Vector3dVector(rgb.astype(float))
    if not o3d.io.write_point_cloud(str(path),pc):
        raise OSError(path)


def p_sample(m,n):
    return np.asarray(m.sample_points_uniformly(number_of_points=n).points).copy()


def score(mesh, vol, gt_volume, gt_surface, measured, gt_occ=None, occ=None):
    sample=p_sample(mesh,9000)
    # A hidden reference surface point has no measured sample within 2.5 mm.
    tree=cKDTree(measured)
    hidden=tree.query(gt_surface)[0]>2.5
    observed=cKDTree(sample).query(gt_surface)[0]
    reverse=cKDTree(gt_surface).query(sample)[0]
    vox=None if occ is None else float(np.sum(occ&gt_occ)/np.sum(occ|gt_occ))
    return sample,{"volume_mm3":vol,"volume_error_pct":100*(vol/gt_volume-1) if vol is not None else None,
          "voxel_iou":vox,"hidden_surface_recall_2":float(np.mean(observed[hidden]<=2.)),
          "surface_f1_2":float(2*np.mean(observed<=2)*np.mean(reverse<=2)/max(.000001,np.mean(observed<=2)+np.mean(reverse<=2))),
          "hidden_reference_n":int(hidden.sum()),"observed_fraction":float((~hidden).mean()),
          "scene_error_p95_mm":float(np.percentile(reverse,95))}


def main():
    cv2.ocl.setUseOpenCL(False)
    OUT.mkdir(exist_ok=True)
    o3d.utility.random.seed(SEED)
    rng=np.random.default_rng(SEED)
    rows=[]
    truth={}
    scenario={
        "four_low":[(a,24) for a in range(0,360,90)],
        "eight_low":[(a,25) for a in range(0,360,45)],
        "eight_plus_high":[(a,25) for a in range(0,360,45)]+[(30,76),(190,76)],
    }
    for case in ("A_box","B_features","C_freeform"):
        occ=(GRID[:,2] < height(case,GRID[:,:2]))
        mesh=voxel_mesh(occ)
        gtvol=analytic_volume(case)
        c=scene(mesh)
        surface=p_sample(mesh,14500)
        folder=OUT/case
        folder.mkdir(exist_ok=True)
        o3d.io.write_triangle_mesh(str(folder/"ground_truth_mesh.ply"),mesh)
        points(folder/"ground_truth_surface.ply",surface)
        truth[case]={"quadrature_reference_mm3":gtvol,"voxel_reference_mm3":float(occ.sum()*PITCH**3),
                      "reference_watertight":bool(mesh.is_watertight())}
        for key,setup in scenario.items():
            t=perf_counter()
            dest=folder/key
            dest.mkdir(exist_ok=True)
            views=[observe(c,camera(a,e),camera(a+17,e+3),key=="four_low",rng) for a,e in setup]
            measured=np.concatenate([v["observed"] for v in views])
            points(dest/"observed.ply",measured)
            hull=carve(views,False)
            depth=carve(views,True)
            footprint=depth.reshape(SIZE).any(2).ravel()
            h_field=infer_height(measured,footprint)
            variants={
                "convex_base":convex(measured),
                "visual_hull":hull,
                "visual_hull_depth":depth,
                "heightfield_base":h_field,"heightfield_depth":h_field&depth,
            }
            if case=="A_box": variants["cuboid_fit"]=None
            for method,estimated in [*variants.items(),("poisson",None)]:
                if method=="cuboid_fit":
                    predicted,estimated=fitted_cuboid(measured)
                    watertight=bool(predicted.is_watertight())
                    volume=abs(predicted.get_volume())
                elif estimated is None:
                    predicted,note=poisson_mesh(views)
                    watertight,volume=note["watertight"],note["volume"]
                    if predicted is None:
                        rows.append({"case":case,"scenario":key,"method":method,"watertight":False,
                             "measured_points":len(measured),"status":"worker_failure"})
                        continue
                else:
                    predicted=voxel_mesh(estimated)
                    watertight=bool(predicted.is_watertight())
                    volume=abs(predicted.get_volume()) if watertight and predicted.is_orientable() else None
                sample,result=score(predicted,volume,gtvol,surface,measured,occ,estimated)
                generated=cKDTree(measured).query(sample)[0]>2.5
                color=np.tile([.24,.57,.77],(len(sample),1));color[generated]=[.94,.49,.19]
                points(dest/f"{method}_points.ply",sample,color)
                o3d.io.write_triangle_mesh(str(dest/f"{method}_mesh.ply"),predicted)
                rows.append({"case":case,"scenario":key,"method":method,"watertight":watertight,
                             "measured_points":len(measured),"predicted_points_tagged":int(generated.sum()),"status":"measured",**result})
            print(case,key,len(measured),round(perf_counter()-t,2),flush=True)
            (dest/"capture.json").write_text(json.dumps({"poses":setup,"passing_depth":[v["depth_samples"] for v in views],
                    "seen_in_white":[v["silhouette_samples"] for v in views],"forced_shadow":key=="four_low"},indent=2))
    with (OUT/"benchmark.csv").open("w",newline="") as f:
        wr=csv.DictWriter(f,fieldnames=list(dict.fromkeys(k for row in rows for k in row)));wr.writeheader();wr.writerows(rows)
    (OUT/"parameters.json").write_text(json.dumps({"units":"mm","pitch":PITCH,"seed":SEED,"truth":truth,"scenario":scenario,
       "ground_truth_distinct_from_inputs":"ray-cast measured surfaces only; no GT geometric features passed to reconstruction",
       "mesh":"shared-face marching tetrahedra, iso-value 0.5", "sample_spacing_depth_noise_sigma":.18,"counterexample_blind_hole_volume_span_mm3":PI*8.5**2*5},indent=2))

if __name__=="__main__":
    main()
