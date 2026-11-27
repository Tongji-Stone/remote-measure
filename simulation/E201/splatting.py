"""CPU 2D Gaussian discs fit to calibrated metric depth and coloured photos."""
from dataclasses import dataclass
import ctypes as ct
import hashlib
from pathlib import Path
import subprocess
import tempfile

import cv2 as cv
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from common import HERE, cloud, oriented_shell, tx


@dataclass
class View:
    world_to_camera: np.ndarray
    intr: np.ndarray
    width: int
    height: int
    camera: tuple
    photo: np.ndarray | None = None
    station: int = -1
    placement: int = -1


def load_engine():
    source=HERE/'splat.cpp'
    tag=hashlib.sha256(source.read_bytes()).hexdigest()[:12]
    so=Path(tempfile.gettempdir())/f'remote-measure-E201-{tag}.dylib'
    if not so.exists():
        build=so.with_suffix('.tmp.dylib')
        subprocess.run(['clang++','-O3','-std=c++17','-dynamiclib',str(source),'-o',str(build)],check=True)
        build.replace(so)
    lib=ct.CDLL(str(so))
    fun=lib.render_splats
    fl=np.ctypeslib.ndpointer(dtype=np.float32,flags='C_CONTIGUOUS')
    it=np.ctypeslib.ndpointer(dtype=np.int32,flags='C_CONTIGUOUS')
    fun.argtypes=[fl,ct.c_int,fl,ct.c_int,ct.c_int,fl,fl,fl,fl,it]
    fun.restype=None
    return fun


def axes(p,samples,n):
    q=cloud(p,n)
    _,j=cKDTree(p).query(p,k=13)
    diff=p[j]-p[:,None,:]
    shift=np.einsum('ijk,ik->ij',diff,n)
    tangent=diff-shift[:,:,None]*n[:,None,:]
    cov=np.einsum('nki,nkj->nij',tangent,tangent)/13 + 1e-6*np.eye(3)
    _,bins=np.linalg.eigh(cov)
    u=bins[:,:,2]
    u-=np.einsum('ij,ij->i',u,n)[:,None]*n
    u/=np.maximum(np.linalg.norm(u,axis=1,keepdims=True),1e-9)
    v=np.cross(n,u)
    distance=cKDTree(p).query(p,k=7)[0][:,-1]
    radius=np.clip(.54*distance,.29,.81)
    return u,v,radius


def seed(case):
    p0=case['p']; n0=case['n']
    q=cloud(p0,n0)
    q.colors=o3d.utility.Vector3dVector(case['rgb'])
    q=q.voxel_down_sample(.57).normalize_normals()
    p,n=np.asarray(q.points).copy(),np.asarray(q.normals).copy()
    rgb=np.asarray(q.colors).copy()
    raw,rv,inc=case['depth_points'],case['depth_normals'],case['depth_weights']
    near,idx=cKDTree(raw).query(p,k=17)
    a=raw[idx]-p[:,None,:]
    axial=np.einsum('nki,ni->nk',a,n)
    align=np.einsum('nki,ni->nk',rv[idx],n)
    weights=np.exp(-.5*(near/.83)**2)*inc[idx]*(align>.70)
    weights/=1+(axial/.39)**2
    mass=weights.sum(1)
    update=(weights*axial).sum(1)/np.maximum(1e-7,mass)
    update=np.where(mass>1,np.clip(update,-.24,.24),0.)
    count_eligible=int(np.sum(mass>1))
    p+=.72*update[:,None]*n
    nn=n+np.einsum('nk,nki->ni',weights,rv[idx])/np.maximum(1.,mass)[:,None]*.36
    n=nn/np.maximum(np.linalg.norm(nn,axis=1,keepdims=True),1e-8)
    # Prefer angular consistency with observed oriented depth, even where colour is weak.
    u,v,radius=axes(p,raw,n)
    return dict(p=p,n=n,u=u,v=v,r=radius,rgb=rgb),dict(n_discs=len(p),observed_fit_count=count_eligible,
                      max_normal_update_mm=float(np.max(abs(.72*update))))


def orthoviews(p):
    mid=np.median(p,axis=0)
    spread=np.ptp(np.percentile(p,[.2,99.8],axis=0),axis=0)
    side=264
    dist=max(70.,2.45*max(spread))
    focal=(side*.72)*dist/max(spread)
    intr=np.array([focal,focal,side/2,side/2],np.float32)
    out=[]
    for elev in (-72,-40,-12,17,45,72):
        for j in range(8 if abs(elev)<46 else 4):
            a=np.deg2rad(j*(45 if abs(elev)<46 else 90)+elev*.43)
            b=np.deg2rad(elev)
            v=np.array([np.cos(a)*np.cos(b),np.sin(a)*np.cos(b),np.sin(b)])
            origin=mid+dist*v
            z=-v
            x=np.cross(z,[0.,0.,1.])
            x/=np.linalg.norm(x)
            R=np.vstack([x,np.cross(z,x),z])
            T=np.eye(4)
            T[:3,:3],T[:3,3]=R,-R@origin
            out.append(View(T,intr,side,side,camera=(None,None,None)))
    return out


def cropped(p, W, K, D, shape, scale, raw=None, station=-1, placement=-1):
    q=tx(p,W)
    uv=np.c_[(q[:,0]/q[:,2])*K[0,0]+K[0,2],(q[:,1]/q[:,2])*K[1,1]+K[1,2]]
    lo=np.maximum(0,np.floor(np.percentile(uv,.2,axis=0)-22)).astype(int)
    hi=np.minimum(np.array(shape[::-1]),np.ceil(np.percentile(uv,99.8,axis=0)+22)).astype(int)
    w,h=np.rint((hi-lo)*scale).astype(int)
    intr=np.array([K[0,0]*scale,K[1,1]*scale,(K[0,2]-lo[0]+.5)*scale-.5,
                   (K[1,2]-lo[1]+.5)*scale-.5],dtype=np.float32)
    photo=None
    if raw is not None:
        image=cv.imread(str(raw),cv.IMREAD_COLOR)
        K2=np.array([[intr[0],0,intr[2]],[0,intr[1],intr[3]],[0,0,1.]])
        a,b=cv.initUndistortRectifyMap(K,D,None,K2,(w,h),cv.CV_32FC1)
        photo=cv.remap(image,a,b,cv.INTER_LINEAR)[:,:,::-1].astype(np.float32)/255.
    return View(W,intr,w,h,(K,D,shape),photo,station,placement)


def camera_views(case):
    K,D,shape=case['camera']
    geometry=[cropped(case['p'],f.world_to_camera,K,D,shape,.40,None,f.station,f.placement) for f in case['raw']]
    photos=[cropped(case['p'],f['W'],f['K'],f['D'],(1080,1920),.61,f['photo'],f['station'],f['placement'])
            for f in case.get('rgb_views',[])]
    return geometry,photos


def render(discs,v,factor,alpha,engine):
    W=v.world_to_camera
    R=W[:3,:3].T
    p=tx(discs['p'],W)
    r=discs['r']*factor
    data=np.column_stack([p,discs['n']@R,discs['u']@R,discs['v']@R,r,r,
                        np.full(len(p),alpha),discs['rgb']]).astype('float32')
    h,w=v.height,v.width
    z=np.zeros((h,w),np.float32)
    a=np.zeros_like(z)
    d=np.zeros_like(z)
    c=np.zeros((h,w,3),np.float32)
    i=np.full((h,w),-1,np.int32)
    engine(data,len(p),v.intr,w,h,z,c,a,d,i)
    return z,c,a,d,i


def raw_target(f,v):
    p=f.camera_xyz
    fx,fy,cx,cy=v.intr
    uv=np.rint(np.c_[fx*p[:,0]/p[:,2]+cx,fy*p[:,1]/p[:,2]+cy]).astype(int)
    x,y=uv.T
    ok=(x>=0)&(x<v.width)&(y>=0)&(y<v.height)
    depth=np.full((v.height,v.width),np.inf,np.float32)
    np.minimum.at(depth,(y[ok],x[ok]),p[ok,2])
    return depth


def depth_eval(discs,views,frames,factor,alpha,engine,station):
    e=[];coverage=[]
    var=[]
    for v,f in zip(views,frames):
        if v.station not in station:
            continue
        target=raw_target(f,v)
        d,c,a,dist,_=render(discs,v,factor,alpha,engine)
        active=np.isfinite(target)
        vld=active&(a>=.5)
        coverage.append(vld.sum()/active.sum())
        e.extend(abs(target[vld]-d[vld]))
        var.extend(dist[vld])
    return dict(coverage=np.mean(coverage),depth_p50_mm=np.median(e),depth_p95_mm=np.percentile(e,95),
                mean_ray_distortion_mm=np.mean(var))


def fit_rgb(seed,views,engine,factor,alpha):
    if not views:
        return []
    history=[]
    for step in range(2):
        targets=np.zeros_like(seed['rgb']);total=np.zeros(len(targets))
        before=[]
        for v in views:
            if v.station in (2,7):
                continue
            z,c,a,dist,uid=render(seed,v,factor,alpha,engine)
            use=(a>.87)&(uid>=0)&(dist<.58)
            use&=cv.erode(use.astype('uint8'),np.ones((7,7),np.uint8))>0
            x=uid[use]
            old=c[use]
            observed=v.photo[use]
            delta=observed-old
            shade=np.clip(np.median(delta,axis=0),-.12,.12)
            obs=(observed-shade).clip(0.,1.)
            robust=np.exp(-np.mean(abs(obs-old),axis=1)/.30)*a[use]
            before.append(np.mean(abs(obs-old)))
            np.add.at(targets,x,obs*robust[:,None])
            np.add.at(total,x,robust)
        prior=7.
        seed['rgb']=(prior*seed['rgb']+targets)/(prior+total[:,None])
        history.append(dict(iteration=step+1,aligned_photometric_MAE=np.mean(before),
                            directly_observed_splats=int(np.count_nonzero(total))))
    return history


def color_eval(seed,views,engine,factor,alpha):
    err=[]
    for v in views:
        if v.station not in (2,7):
            continue
        _,rgb,alpha_map,_,_=render(seed,v,factor,alpha,engine)
        mask=cv.erode((alpha_map>.87).astype('uint8'),np.ones((7,7),np.uint8))>0
        if mask.any():
            err.append(np.mean((rgb[mask]-v.photo[mask])**2))
    return -10*np.log10(np.mean(err)) if err else None


def projective_grid(p,voxel):
    """Millimetre lattice used for the projective truncated distance fusion."""
    lo=np.floor((np.percentile(p,.04,axis=0)-4.)/voxel)*voxel
    hi=np.percentile(p,99.96,axis=0)+4.
    shape=tuple((np.ceil((hi-lo)/voxel)+1).astype(int))
    grid=(lo+np.indices(shape).reshape(3,-1).T*voxel).astype('float32')
    return lo,shape,grid


def rendered_views(discs, views,factor,alpha,engine):
    for v in views:
        z,_,a,s,i=render(discs,v,factor,alpha,engine)
        good=(a>=.55)&(s<1.2)&(i>=0)
        n=discs['n']@v.world_to_camera[:3,:3].T
        weight=a*np.maximum(-n[np.maximum(i,0),2],.08)**1.5
        yield v,np.where(good,z,0),np.where(good,weight,0)


def raster_geometry(discs,views,factor,alpha,engine,voxel):
    from reconstruct import projective_surface
    lo,shape,grid=projective_grid(discs['p'],voxel)
    num=np.zeros(len(grid),np.float32)
    weights=np.zeros_like(num)
    tau=max(1.4,2.9*voxel)
    for v,z,opacity in rendered_views(discs,views,factor,alpha,engine):
        p=tx(grid,v.world_to_camera.astype('float32'))
        fx,fy,cx,cy=v.intr
        x=np.rint(p[:,0]/p[:,2]*fx+cx).astype(np.int32)
        y=np.rint(p[:,1]/p[:,2]*fy+cy).astype(np.int32)
        rows=np.flatnonzero((p[:,2]>0)&(x>=0)&(x<v.width)&(y>=0)&(y<v.height))
        depth=z[y[rows],x[rows]]
        delta=depth-p[rows,2]
        pick=(depth>0)&(delta>=-tau)&(delta<=2.6*tau)
        i=rows[pick]
        weight=opacity[y[i],x[i]]
        num[i]+=weight*np.minimum(delta[pick]/tau,1.)
        weights[i]+=weight
    value=np.divide(num,weights,out=np.zeros_like(num),where=weights>0)
    return oriented_shell(projective_surface(value,weights>.14,lo,shape,voxel))


def run_splat(case, folder):
    engine=load_engine()
    discs,info=seed(case)
    geo,photos=camera_views(case)
    parameters=[]
    for factor in (.86,1.13,1.40):
        for alpha in (.74,.90):
            row=depth_eval(discs,geo,case['raw'],factor,alpha,engine,(1,6))
            score=row['depth_p95_mm']+.24*row['mean_ray_distortion_mm']+2.8*(1-row['coverage'])
            parameters.append(dict(radius_factor=factor,opacity=alpha,criterion=score,**row))
    chosen=min(parameters,key=lambda r:r['criterion'])
    s,a=chosen['radius_factor'],chosen['opacity']
    b=color_eval(discs,photos,engine,s,a)
    train=fit_rgb(discs,photos,engine,s,a)
    h=color_eval(discs,photos,engine,s,a)
    virtual=orthoviews(discs['p'])
    active=[v for v in geo if v.station not in (2,7)]
    np.savez_compressed(folder/'2D高斯盘.npz', **{k:v.astype('float32') for k,v in discs.items()},
                        fitted_radius_factor=s,opacity=a)
    v=virtual[17]
    z,c,alpha,d,_=render(discs,v,s,a,engine)
    rgb=np.rint(255*np.where((alpha>=.5)[:,:,None],c,.93)).astype('uint8')
    cv.imwrite(str(folder/'2DGS渲染.png'),rgb[:,:,::-1])
    return dict(**info, tuning_stations=[1,6], held_stations=[2,7],
                         sampling_search=parameters,selected=chosen,rgb_updates=train,
                         rgb_held_PSNR_before=b,rgb_held_PSNR_after=h,
                         depth_held=depth_eval(discs,geo,case['raw'],s,a,engine,(2,7)),
                         projected_camera_views=len(active), virtual_views=len(virtual)),discs
