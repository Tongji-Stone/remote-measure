"""Regularize near-closed candidates in a measured-board coordinate system."""
from time import perf_counter

import cv2 as cv
import numpy as np
import open3d as o3d
from scipy import ndimage as ndi
from scipy.spatial import cKDTree

from common import GROUPS,HERE,PYRAMIDS,as_mesh,audit,dump,measured,oriented_shell,save_mesh,scene
from deck import contact_carve
from reconstruct import projective_surface,rays,screened
from traditional import legacy_frames


def remove_micro_shells(m):
    """Discard isolated lattice islands smaller than 0.3% of the main surface."""
    ids,_,area=m.cluster_connected_triangles()
    sizes=np.asarray(area)
    if len(sizes)>1:
        m.remove_triangles_by_mask(sizes[np.asarray(ids)]<.003*sizes.max())
        m.remove_unreferenced_vertices()
    return oriented_shell(m)


def precise_closure(prior,case,step,*,exposed=()):
    p=case['p']
    lo=np.floor((np.min(p,axis=0)-2.2)/step)*step
    lo[2]=0.
    hi=np.max(p,axis=0)+2.2
    dims=tuple(np.ceil((hi-lo)/step).astype(int)+1)
    grid=lo+np.indices(dims).reshape(3,-1).T*step
    f,cal=legacy_frames(case)
    free,_,_=rays(grid,f,cal)
    tracer=scene(prior)
    val=np.empty(len(grid),np.float32)
    for ids in np.array_split(np.arange(len(grid)),1+len(grid)//85000):
        t=o3d.core.Tensor(grid[ids].astype('float32'))
        val[ids]=tracer.compute_signed_distance(t,nthreads=1,nsamples=3).numpy()
    measured_free=(free>=3)
    # A small amount of smoothing suppresses lattice-scale features while retaining sharp edges.
    val=ndi.gaussian_filter(np.clip(val,-3.,3.).reshape(dims),sigma=.36).ravel()
    val[measured_free]=np.maximum(val[measured_free],.10)
    if exposed:
        field=val.reshape(dims)
        xy=grid.reshape(*dims,3)[:,:,0,:2].reshape(-1,2)
        empty=contact_carve(xy,exposed,p).reshape(dims[:2])
        # Restrict the board constraint to the measured contact layer.
        for z in range(int(np.ceil(1.2/step))):
            sl=field[:,:,z]
            sl[empty]=np.maximum(sl[empty],.30-z*step*.8)
    val=np.copysign(np.maximum(abs(val),.023),val)
    m=projective_surface(val,np.ones_like(val,bool),lo,dims,step,cap_floor=True)
    return remove_micro_shells(m)


def gs_prior(raw,case):
    p=case['p'];n=case['n']
    q=np.asarray(raw.vertices)
    raw.compute_vertex_normals()
    b=np.asarray(raw.vertex_normals)
    dx,idx=cKDTree(q).query(p,k=7)
    move=q[idx]-p[:,None,:]
    axial=np.einsum('nki,ni->nk',move,n)
    align=abs(np.einsum('nki,ni->nk',b[idx],n))
    weight=np.exp(-.5*(dx/.70)**2)*(align>.65)
    drift=(weight*axial).sum(1)/np.maximum(weight.sum(1),1e-9)
    change=(weight.sum(1)>1.25)&(dx[:,0]<.70)
    # Commit only an anchored, half-step update toward the high-confidence splat surface.
    pnew=p+n*np.where(change,.5*np.clip(drift,-.30,.30),0.)[:,None]
    return screened(pnew,n)[0],int(change.sum())


def run(case,folder,candidates=None):
    if candidates is None:
        candidates={}
    stats={}
    neutral=case['name'] in PYRAMIDS
    tests=[('S08-PrecisePoisson',None,.55),('S08-coarse',None,.74),
           ('G03-2DGS-anchored','G01-2DGS-TSDF',.55),('G03-coarse','G01-2DGS-TSDF',.74)]
    sources={}
    for name,key,d in tests:
        t=perf_counter()
        if key not in sources:
            if key is None:
                source=candidates.get('S01-Poisson')
                if source is None:
                    source=o3d.io.read_triangle_mesh(str(folder/'S01-Poisson.ply'))
                aux=0
            else:
                source=candidates.get(key)
                if source is None:
                    source=o3d.io.read_triangle_mesh(str(folder/f'{key}.ply'))
                source,aux=gs_prior(source,case)
            sources[key]=source,aux
        source,n_adj=sources[key]
        m=precise_closure(source,case,d)
        if as_mesh(m):
            if key and (folder/'2D高斯盘.npz').is_file():
                with np.load(folder/'2D高斯盘.npz') as splat:
                    save_mesh(folder/f'{name}.ply',m,splat['p'],splat['rgb'],neutral_missing=neutral)
            else:
                save_mesh(folder/f'{name}.ply',m,case['p'],case['rgb'],neutral_missing=neutral)
        stats[name]=dict(**audit(m,case),step_mm=d,anchored_splat_updates=n_adj,
                         elapsed_s=perf_counter()-t)
        print(case['name'],name,stats[name].get('volume_ml'),stats[name].get('observed_P95_mm'),
              stats[name].get('ray_front_1p5_fraction'),flush=True)
    dump(folder/'精确闭合.json',stats)


if __name__=='__main__':
    cv.setNumThreads(1)
    for name in GROUPS:
        run(measured(name),HERE/name)
