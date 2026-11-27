"""Complete the near-measured 2DGS surface using the same measured-board guard."""
from time import perf_counter

import cv2 as cv
import numpy as np
from scipy.spatial import cKDTree

from common import GROUPS, HERE, PYRAMIDS, audit, as_mesh, dump, measured, oriented_shell, save_mesh, intersection_pairs
from reconstruct import envelope, footprint, grid_box, rays, screened
from splatting import load_engine, camera_views, orthoviews, raster_geometry
from traditional import legacy_frames


def close_splat(source,case,discs,pitch):
    source.compute_vertex_normals()
    q=np.asarray(source.vertices)
    n=np.asarray(source.vertex_normals)
    d,j=cKDTree(case['p']).query(q)
    good=d<1.35
    sg=np.einsum('ij,ij->i',n,case['n'][j])
    keep=good&(abs(sg)>.51)
    n=n[keep]*np.where(sg[keep]>=0,1.,-1.)[:,None]
    s=q[keep]
    if len(s)<100:
        raise ValueError('Insufficient equally oriented raster surface support.')
    g,gn=footprint(case['p'])
    frames,cam=legacy_frames(case)
    ok=rays(g,frames,cam)[0]<2
    # Complement the raster surface where calibrated measured depth still has no splat support.
    p0,n0=case['p'],case['n']
    d0=cKDTree(s).query(p0)[0]
    retain=d0>1.18
    g,gn=g[ok],gn[ok]
    gn=gn/np.maximum(np.linalg.norm(gn,axis=1,keepdims=True),1e-9)
    prior=screened(np.vstack([s,p0[retain],g]),np.vstack([n,n0[retain],gn]))[0]
    lo,sz,grid=grid_box(np.vstack([p0,g]),pitch)
    free,_,_=rays(grid,frames,cam)
    out=oriented_shell(envelope(prior,p0,g,lo,sz,grid,free,pitch,intersections=intersection_pairs))
    return out,dict(splat_surface_support=len(s),fallback_measured_points=int(retain.sum()),
                    contact_guides=len(g),closure_grid_mm=pitch)


def finish(case,folder,discs=None):
    if discs is None:
        with np.load(folder/'2D高斯盘.npz') as a:
            discs=dict(a)
    engine=load_engine()
    factor,alpha=(float(discs[k]) for k in ('fitted_radius_factor','opacity'))
    geo,_=camera_views({**case,'rgb_views':[]})
    positions=[v for v in geo if v.station not in (2,7)]+orthoviews(discs['p'])
    results={}
    raw=None
    neutral=case['name'] in PYRAMIDS
    for code,step in (('G01-2DGS-TSDF',.47),('G01-coarse',.66)):
        t=perf_counter()
        m=raster_geometry(discs,positions,factor,alpha,engine,step)
        if as_mesh(m):
            if raw is None:
                raw=m
            save_mesh(folder/f'{code}.ply',m,discs['p'],discs['rgb'],neutral_missing=neutral)
        print(case['name'],code,len(m.triangles),'audit',flush=True)
        results[code]=dict(**audit(m,case),voxel_mm=step,elapsed_s=perf_counter()-t)
        print(case['name'],code,results[code].get('volume_ml'),flush=True)
    if raw is not None:
        for code,step in [('G02-2DGS-closed',.67),('G02-coarse',.82)]:
            t=perf_counter()
            m,detail=close_splat(raw,case,discs,step)
            if as_mesh(m):
                save_mesh(folder/f'{code}.ply',m,discs['p'],discs['rgb'],neutral_missing=neutral)
            print(case['name'],code,len(m.triangles),'audit',flush=True)
            results[code]=dict(**audit(m,case),**detail,elapsed_s=perf_counter()-t)
            print(case['name'],code,results[code].get('volume_ml'),flush=True)
    dump(folder/'高斯网格.json',results)


if __name__=='__main__':
    cv.setNumThreads(1)
    for name in GROUPS:
        finish(measured(name),HERE/name)
