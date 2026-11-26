"""Volume-oriented radial section model for samples with star-shaped horizontal sections."""
import json
from pathlib import Path
import sys

import numpy as np
import open3d as o3d
from numpy.polynomial.chebyshev import chebvander

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
sys.path.insert(0,str(ROOT/'simulation/E104'))
sys.path.insert(0,str(ROOT/'computing volume/V001'))
from experiment import real_input, sample_frames, simulated, split
from run import save_csv
from surface_bias import metrics
from volume_math import Surface, checked_volume


def angular(theta):
    k=np.arange(1,11)
    return np.c_[np.ones(len(theta)),np.cos(theta[:,None]*k),np.sin(theta[:,None]*k)]


def features(s,theta,point):
    z=chebvander(2*s-1,3)*((4*s*(1.-s)) if point else (1.-s))[:,None]
    return np.einsum('ni,nj->nij',z,angular(theta)).reshape(len(s),-1)


def fit(p,point):
    top=np.percentile(p[:,2],99.86)
    band=p[(p[:,2]>.37*top)&(p[:,2]<.64*top)]
    xy=(np.percentile(band[:,:2],1.,axis=0)+np.percentile(band[:,:2],99.,axis=0))/2.
    p=p[(p[:,2]>.30)&(p[:,2]<top-.10)]
    x=p[:,:2]-xy
    angle=np.arctan2(x[:,1],x[:,0])
    s=p[:,2]/top
    r2=np.sum(x*x,axis=1)
    normalized=max(np.quantile(r2,.94),1.)
    target=r2/normalized
    an=(np.mod(angle+np.pi,2*np.pi)*36/(2*np.pi)).astype(int)
    idx=np.minimum((s*24).astype(int),23)*36+an
    weights=1./np.bincount(idx,minlength=24*36)[idx]
    B=features(s,angle,point)
    reg=.038*(np.arange(4)[:,None]**3+.024*np.r_[0,np.arange(1,11)**2,np.arange(1,11)**2])
    D=np.diag(reg.ravel()+.005)
    h=np.zeros(B.shape[1])
    for _ in range(5):
        clamp=np.minimum(1.,.12/np.maximum(abs(B@h-target),1e-8))
        w=weights*clamp
        h=np.linalg.solve(B.T@(w[:,None]*B)+D,B.T@(w*target))
    return xy,top,h*normalized


def predict(coeff,s,az,point,minimum=True):
    shape=4*s*(1.-s) if point else 1.-s
    raw=np.einsum('ni,ij,mj->nm',chebvander(2*s-1,3)*shape[:,None],
                  coeff.reshape(4,-1),angular(az))
    # Keep a small positive interior radius and collapse analytic endpoints to a pole.
    return np.maximum(.6*shape[:,None],raw) if minimum else raw


def radius_mesh(xy,top,coeff,point):
    # Retain individual angular modes in slices. A point-contact model uses single endpoint vertices.
    z=np.linspace(0,1,91)
    a=np.arange(192)*(2*np.pi/192)
    slices=z[1:-1] if point else z[:-1]
    radii=np.sqrt(np.maximum(1e-4,predict(coeff,slices,a,point)))
    circles=radii[:,:,None]*np.c_[np.cos(a),np.sin(a)]+xy[None,None]
    xyz=np.c_[circles.reshape(-1,2),np.repeat(slices*top,len(a))]
    faces=[]
    n=len(a)
    for k in range(len(slices)-1):
        j=np.arange(n)+k*n
        j1=np.roll(j,-1)
        faces.extend(np.c_[j,j1,j+n])
        faces.extend(np.c_[j1,j1+n,j+n])
    bottom=len(xyz)
    sky=bottom+1
    j=np.arange(n)
    jt=j+(len(slices)-1)*n
    faces.extend(np.c_[np.full(n,bottom),np.roll(j,-1),j])
    faces.extend(np.c_[jt,np.roll(jt,-1),np.full(n,sky)])
    return Surface(np.r_[xyz,[[*xy,0.],[*xy,top]]],np.array(faces))


def main():
    rows=[]
    curves=[]
    for code in ('G3','K1'):
        if code=='G3':
            frames,_,_,_,(_,truth),_,_=simulated(code)
            cases=['T0','T1','T2']
        else:
            frames,_,_,_,_,_,_=real_input('K1','ChArUco')
            truth=None
            cases=['T0']
        for trial in cases:
            train,held,hidden,_=split(frames,trial)
            p,_=sample_frames(train,each=4700)
            val,cam=sample_frames(held,each=1800)
            extra,_=sample_frames(hidden,each=1000)
            for method,point in (('R-pole',True),('R-flat',False)):
                xy,top,co=fit(p,point)
                m=radius_mesh(xy,top,co,point)
                vm=checked_volume(m)
                valid=not m.legacy().is_self_intersecting()
                sc=m.scene()
                s=np.linspace(0,1,401)
                a=np.arange(384)*2*np.pi/384
                R2=predict(co,s,a,point)
                A=np.pi*R2.mean(axis=1)
                volume=np.trapezoid(A,x=s*top)
                assert abs(vm/volume-1)<.005,(method,vm,volume)
                row=dict(case=code,trial=trial,method=method,quadrature_ml=volume/1000,
                         mesh_ml=vm/1000,relative_error=None if truth is None else volume/truth-1.,
                         fit_height_mm=top,valid=valid,nonpositive_fraction=np.mean(predict(co,s[1:-1],a,point,False)<=0),reference_ml=None if truth is None else truth/1000.,
                         **metrics(sc,val,cam))
                if len(extra):
                    row['hidden_recall_2mm']=metrics(sc,extra)['within2']
                rows.append(row)
                curves.extend(dict(case=code,trial=trial,method=method,height_mm=t,area_mm2=q)
                              for t,q in zip(s[::4]*top,A[::4]))
                if code=='K1' and valid:
                    assert o3d.io.write_triangle_mesh(str(HERE/'results/meshes'/f'{code}-{method}.ply'),m.legacy())
                print(row,flush=True)
    save_csv(HERE/'results/sections.csv',rows)
    save_csv(HERE/'results/section_areas.csv',curves)


if __name__=='__main__':
    main()
