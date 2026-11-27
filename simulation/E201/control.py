"""Run classical and splatted reconstruction on calibrated analytic controls."""
import math

import cv2 as cv
import numpy as np
from scipy.spatial import cKDTree

from common import HERE, Frame, audit, dump, prepare, tx
from complete_splats import close_splat
from plate_closure import precise_closure, gs_prior
from reconstruct import envelope, footprint, grid_box, rays, screened
from splatting import camera_views, depth_eval, load_engine, orthoviews, raster_geometry, seed
from traditional import legacy_frames

R=13.


def analytic(name):
    centers=np.array([[0.,0.,R]]) if name=='球' else np.array([[-6.5,0.,R],[6.5,0.,R]])
    exact=4*math.pi*R**3/3
    if len(centers)>1:
        distance=np.linalg.norm(centers[1]-centers[0])
        exact=2*exact-math.pi*(4*R+distance)*(2*R-distance)**2/12
    k=np.arange(36000)+.5
    nz=1-2*k/len(k)
    phi=k*np.pi*(3.-np.sqrt(5.))
    points=R*np.c_[np.sqrt(1-nz*nz)*np.cos(phi),np.sqrt(1-nz*nz)*np.sin(phi),nz]
    ref=np.vstack([points+c for c in centers])
    d=np.linalg.norm(ref[:,None,:]-centers,axis=2)
    exterior=d.min(1)>=R-1e-7
    ref=ref[exterior]
    camera=np.array([[498,0,188.],[0,498,151.],[0,0,1.]])
    D=np.zeros(5)
    shape=(302,376)
    y,x=np.meshgrid(np.arange(0,shape[0],2),np.arange(0,shape[1],2),indexing='ij')
    xy=np.c_[(x.ravel()-camera[0,2])/camera[0,0],(y.ravel()-camera[1,2])/camera[1,1],np.ones(x.size)]
    frames=[]
    angles=[(36*i,22.) for i in range(10)]+[(45+36*i,66.) for i in range(10)]
    for j,(phi,theta) in enumerate(angles):
        a,b=np.deg2rad([phi,theta])
        n=np.array([np.cos(a)*np.cos(b),np.sin(a)*np.cos(b),np.sin(b)])
        eye=n*72+centers.mean(0)
        left=np.cross(-n,[0,0,1.])
        left/=np.linalg.norm(left)
        Rw=np.vstack([left,np.cross(-n,left),-n])
        W=np.eye(4)
        W[:3,:3],W[:3,3]=Rw,-Rw@eye
        view=xy@Rw
        orig=eye-centers
        qa=np.sum(view**2,axis=1)
        qb=view@orig.T
        qc=np.sum(orig**2,axis=1)-R*R
        discriminant=qb*qb-qa[:,None]*qc
        t=(-qb-np.sqrt(np.maximum(discriminant,0)))/qa[:,None]
        t[discriminant<=0]=np.inf
        t=t.min(1)
        f=np.isfinite(t)
        p=xy[f]*t[f,None]
        q=tx(p,np.linalg.inv(W))
        # Deliberately omit an underside cap and a lateral spot from every scan.
        keep=(q[:,2]>2.05)&(np.linalg.norm(q-np.array([0,12,17]),axis=1)>2.25)
        frames.append(Frame(p[keep],W,j//10,j%10))
    cam=(camera,D,shape)
    p,n,c,_=prepare(frames,cam,bin_width=2)
    near=cKDTree(centers).query(p)[1]
    n=(p-centers[near])/R
    rgb=np.broadcast_to([.54,.69,.51],p.shape).copy()
    return dict(name=name,p=p,n=n,rgb=rgb,depth_points=p,depth_normals=n,depth_weights=c,
                raw=frames,camera=cam,reference=ref,true_volume=exact/1000.)


def trials(case):
    p,n=case['p'],case['n']
    g,gn=footprint(p)
    gs=gn/np.maximum(np.linalg.norm(gn,axis=1,keepdims=True),1e-8)
    f,cam=legacy_frames(case)
    lo,shape,grid=grid_box(p,.67)
    free,v,groups=rays(grid,f,cam)
    shell=screened(p,n)[0]
    prior=screened(np.vstack([p,g]),np.vstack([n,gs]))[0]
    closed=envelope(prior,p,g,lo,shape,grid,free,.67)
    discs,info=seed(case)
    engine=load_engine()
    geo,_=camera_views(case)
    choices=[dict(factor=r,alpha=a,**depth_eval(discs,geo,case['raw'],r,a,engine,(1,6)))
             for r in (.86,1.13,1.40) for a in (.74,.9)]
    best=min(choices,key=lambda d:d['depth_p95_mm']+.24*d['mean_ray_distortion_mm']+2.8*(1-d['coverage']))
    virtual=orthoviews(discs['p'])
    active=[q for q in geo if q.station not in (2,7)]
    projected=raster_geometry(discs,active+virtual,best['factor'],best['alpha'],engine,.47)
    full,detail=close_splat(projected,case,discs,.67)
    p08=precise_closure(shell,case,.55)
    anchor,_=gs_prior(projected,case)
    g03=precise_closure(anchor,case,.55)
    return {'S01-Poisson':audit(shell,case),'S04-GuideEnvelope':audit(closed,case),
            'G01-2DGS-TSDF':audit(projected,case),'G02-2DGS-closed':audit(full,case),
            'S08-PrecisePoisson':audit(p08,case),'G03-2DGS-anchored':audit(g03,case)},best


if __name__=='__main__':
    cv.setNumThreads(1)
    rows={}
    for name in ('球','双球相交'):
        case=analytic(name)
        print(name,case['true_volume'],len(case['p']),flush=True)
        m,params=trials(case)
        rows[name]=dict(exact_ml=case['true_volume'],methods=m,raster=params)
        print(name,[(k,d.get('volume_ml'),d.get('relative_volume_error')) for k,d in m.items()],flush=True)
    dump(HERE/'已知体积对照.json',rows)
