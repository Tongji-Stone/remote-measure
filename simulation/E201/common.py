"""E204 inputs, measured contact datum, mesh colour and volume audit."""
import json
from pathlib import Path
import sys

import cv2 as cv
import numpy as np
import open3d as o3d
from scipy.optimize import least_squares
from scipy.spatial import cKDTree

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[1]
FUSION=REPO/'algorithm/scene-fusion'
sys.path[:0]=[str(FUSION/'E204'),str(HERE.parent/'E104'),str(REPO/'algorithm/Math/M01')]
from core import GROUPS as MULTI_PLACEMENT, Frame, get_case, prepare, read_ply
from register import load
from reconstruct import tx, cloud, clean, scene
from volume_math import Surface, checked_volume, column_integral

PYRAMIDS={'三棱锥_大面坐地':'021427', '三棱锥_小面坐地':'022214'}
GROUPS={**MULTI_PLACEMENT, **{name:(code,) for name,code in PYRAMIDS.items()}}


def acquisition(name):
    if name not in PYRAMIDS:
        return get_case(name), FUSION/'E204'/name/'融合.ply'
    scan=load(PYRAMIDS[name])
    origin=FUSION/'E202'/scan['name']
    to_camera=np.linalg.inv(scan['P'])@np.linalg.inv(scan['B'])
    frames=[Frame(read_ply(origin/'单视角'/f'V{j:02}.ply')['xyz'].astype(float),M,0,j)
            for j,M in enumerate(to_camera)]
    cal=json.loads((FUSION/'E201/标定.json').read_text())['hik_projector']
    camera=np.asarray(cal['cam_int']),np.asarray(cal['cam_dist']),(2048,2448)
    return dict(frames=frames,scans=[scan],camera=camera),origin/'整圈/推荐.ply'


def dump(path,data):
    path.write_text(json.dumps(data,ensure_ascii=False,allow_nan=False,indent=2,
                    default=lambda x:x.tolist())+'\n')


def plate_datum(case):
    primary=case['scans'][0]
    by=[]
    for j in range(len(primary['P'])):
        sub=f"{primary['name']}/单视角/V{j:02}.ply"
        a=read_ply(FUSION/'E201'/sub)['xyz']
        b=read_ply(FUSION/'E202'/sub)['xyz']
        dtype=np.dtype([('v','V12')])
        keep=~np.isin(a.copy().view(dtype).ravel(),b.copy().view(dtype).ravel())
        # Board plane is fitted in the rotating board coordinates for every station.
        W=primary['B']@primary['P'][j]
        p=tx(a[keep][::8],W)
        by.append(p[abs(p[:,2])<3.2])
    samples=np.vstack(by)
    q=samples[:,:2]-[75.,75.]
    ans=least_squares(lambda t:q@t[:2]+t[2]-samples[:,2],np.zeros(3),
                      loss='soft_l1',f_scale=.5)
    coef=ans.x
    n=np.array([-coef[0],-coef[1],1.])
    n/=np.linalg.norm(n)
    u=np.array([1.,0.,0.])-n*n[0]
    u/=np.linalg.norm(u)
    R=np.vstack([u,np.cross(n,u),n])
    T=np.eye(4)
    T[:3,:3]=R
    T[:3,3]=-R@np.array([75.,75.,coef[2]])
    errors=np.abs(q@coef[:2]+coef[2]-samples[:,2])
    return T,dict(plate_points=len(q),plane_coefficients=coef,plane_residual_p50_mm=np.median(errors),
                  plane_residual_p95_mm=np.percentile(errors,95),world_to_plate_mm=T)


def measured(name):
    c,path=acquisition(name)
    A,datum=plate_datum(c)
    p=read_ply(path)
    from_camera=c['scans'][0]['B']
    pxyz=tx(p['xyz'],A@from_camera)
    color=p['rgb']/255.
    old=[Frame(f.camera_xyz,f.world_to_camera@np.linalg.inv(A),f.placement,f.station) for f in c['frames']]
    src,n,w,_=prepare(old,c['camera'])
    a=cloud(pxyz)
    a.colors=o3d.utility.Vector3dVector(color)
    a=a.voxel_down_sample(.48)
    pxyz,color=np.asarray(a.points),np.asarray(a.colors)
    a.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=1.9,max_nn=40))
    norm=np.asarray(a.normals).copy()
    nearest=cKDTree(src).query(pxyz,k=5)[1]
    guide=np.sum(n[nearest]*w[nearest,None],axis=1)
    norm[np.einsum('ij,ij->i',norm,guide)<0]*=-1
    # Floating speckles are pruned using the same neighbourhood support rule for every object.
    k=cKDTree(pxyz).query(pxyz,k=7)[0][:,-1]<1.6
    placed_rgb=[]
    rig=json.loads((FUSION/'E201/彩色投影核验.json').read_text())['hik_to_color_used']
    cal=json.loads((FUSION/'E201/标定.json').read_text())['usb_color']
    for f,s in zip(old,[folder for s in c['scans'] for folder in s['folders']]):
        placed_rgb.append(dict(W=np.array(rig)@f.world_to_camera,photo=s/'texture.png',
                               station=f.station,placement=f.placement,
                               K=np.asarray(cal['cam_int']),D=np.asarray(cal['cam_dist'])))
    return dict(name=name,p=pxyz[k],n=norm[k],rgb=color[k],raw=old,rgb_views=placed_rgb,
                depth_points=src,depth_normals=n,depth_weights=w,camera=c['camera'],
                datum=datum,source=str(path.relative_to(REPO)))


def save_mesh(path,m,p,rgb,*,neutral_missing=False):
    q=np.asarray(m.vertices)
    d,k=cKDTree(p).query(q,k=4)
    w=np.exp(-.5*(d-d[:,0,None])**2/.48**2)
    c=np.einsum('ij,ijk->ik',w,rgb[k])/w.sum(1)[:,None]
    if neutral_missing:
        gap=np.clip((d[:,0]-1.2)/1.8,0.,1.)[:,None]
        c=(1.-gap)*c+gap*.57
    m.vertex_colors=o3d.utility.Vector3dVector(c.clip(0,1))
    m.compute_vertex_normals()
    if not o3d.io.write_triangle_mesh(str(path),m):
        raise ValueError(path)


def as_mesh(m):
    return m is not None and m.has_triangles()


def oriented_shell(m):
    m=clean(m,40)
    if not m.has_triangles() or not m.is_orientable():
        return m
    m.orient_triangles()
    surf=Surface.of(m)
    if surf.signed()<0:
        m.triangles=o3d.utility.Vector3iVector(np.asarray(m.triangles)[:,::-1].copy())
    return m


def intersection_pairs(m, block=5500):
    """Use the same Open3D pair test, partitioning triangle AABBs for dense meshes."""
    points=np.asarray(m.vertices)
    faces=np.asarray(m.triangles)
    if len(faces)<=block:
        return np.asarray(m.get_self_intersecting_triangles())
    bounds=points[faces]
    lower=bounds.min(1)
    upper=bounds.max(1)
    mid=(lower+upper)*.5
    todo=[(np.arange(len(faces)),0)]
    pairs=[]
    while todo:
        ids,depth=todo.pop()
        axis=np.argmax(np.ptp(mid[ids],axis=0))
        split=np.median(mid[ids,axis])
        left=ids[lower[ids,axis]<=split]
        right=ids[upper[ids,axis]>=split]
        # Every intersecting pair shares at least one child, including on the split plane.
        if len(ids)>block and max(len(left),len(right))<.9*len(ids) and depth<12:
            todo.extend(((left,depth+1),(right,depth+1)))
            continue
        f=faces[ids]
        chosen,inverse=np.unique(f.ravel(),return_inverse=True)
        part=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(points[chosen]),
                    o3d.utility.Vector3iVector(inverse.reshape(f.shape).astype('int32')))
        bad=np.asarray(part.get_self_intersecting_triangles())
        if len(bad):
            pairs.append(np.sort(ids[bad],axis=1))
    return np.unique(np.vstack(pairs),axis=0) if pairs else np.empty((0,2),dtype=int)


def audit(m,case):
    if not as_mesh(m):
        return dict(status='empty')
    s=Surface.of(m)
    topology=s.topology()
    overlap=scene(m)
    p,n=case['p'],case['n']
    ref=case.get('reference',p)
    e=overlap.compute_distance(o3d.core.Tensor(ref.astype(np.float32)),nthreads=1).numpy()
    e0=overlap.compute_distance(o3d.core.Tensor(p.astype(np.float32)),nthreads=1).numpy()
    o3d.utility.random.seed(1201)
    samp=np.asarray(m.sample_points_uniformly(9000).points)
    back=cKDTree(p).query(samp)[0]
    held=case['raw']
    lp,lc=[],[]
    for f in held:
        if f.station not in (2,7):
            continue
        S=np.linalg.inv(f.world_to_camera)
        lp.append(tx(f.camera_xyz[::21],S))
        lc.append(np.repeat(S[:3,3][None],len(lp[-1]),axis=0))
    err=np.empty(0)
    front=0.
    hit_rate=0.
    if lp:
        view=np.vstack(lc)
        d=np.vstack(lp)-view
        length=np.linalg.norm(d,axis=1)
        t=overlap.cast_rays(o3d.core.Tensor(np.c_[view,d/length[:,None]].astype(np.float32)),nthreads=1)['t_hit'].numpy()
        ok=np.isfinite(t)
        hit_rate=float(np.mean(ok))
        err=t[ok]-length[ok]
        front=float(np.mean(err<-1.5)) if len(err) else 0.
    floor=np.asarray(m.vertices)[:,2]
    ta=s.vertices[s.faces]
    cap_area=float(s.g()[1][(np.max(np.abs(ta[:,:,2]),axis=1)<.04)].sum())
    metrics=dict(status='scored',vertices=len(s.vertices),triangles=len(s.faces),**topology,
            observed_P50_mm=np.median(e0),observed_P95_mm=np.percentile(e0,95),
            reference_P95_mm=np.percentile(e,95),unsupported_fraction_2mm=np.mean(back>2.),
            below_plate_area_fraction=np.mean(samp[:,2]<-.65),min_z_mm=floor.min(),
            plate_cap_area_mm2=cap_area,ray_hit_fraction=hit_rate,ray_front_1p5_fraction=front,
            ray_abs_P95_mm=np.percentile(abs(err),95) if len(err) else None)
    volume=None
    self_intersect=None
    columns=[]
    try:
        v=checked_volume(s)
        self_intersect=bool(len(intersection_pairs(m)))
        if not self_intersect:
            volume=v/1000
            for h in (.8,.4):
                for axis in range(3):
                    record=column_integral(s,overlap,h,axis,(.31,.62))
                    columns.append({**record,'relative_difference':record['column_volume_mm3']/v-1.})
    except ValueError:
        pass
    metrics.update(volume_ml=volume,self_intersections=self_intersect,column_checks=columns)
    if case.get('true_volume'):
        metrics['relative_volume_error']=volume/case['true_volume']-1 if volume is not None else None
    return metrics
