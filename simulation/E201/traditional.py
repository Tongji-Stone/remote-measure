"""Run the previously evaluated fitting families on the E204 sample set."""
from time import perf_counter

import numpy as np
import open3d as o3d

from common import PYRAMIDS, oriented_shell, as_mesh, intersection_pairs
from reconstruct import (screened, bpa, cloud, footprint, hull_mesh, envelope, rays, grid_box,
                         projective_surface, polar_guides, fit_planes, tx)


def legacy_frames(case):
    f=[]
    for a in case['raw']:
        T=np.linalg.inv(a.world_to_camera)
        f.append(dict(p=tx(a.camera_xyz,T), camera=T[:3,3],to_camera=a.world_to_camera))
    K,D,shape=case['camera']
    return f,dict(cam_int=K,cam_dist=D,shape=shape)


def refine_planar_colour_mesh(m):
    """Subdivide planar faces for vertex colours while keeping the fitted planes fixed."""
    points=np.asarray(m.vertices)[np.asarray(m.triangles)]
    longest=np.linalg.norm(points-np.roll(points,1,axis=1),axis=2).max()
    steps=max(0,int(np.ceil(np.log2(longest/2.))))
    return m.subdivide_midpoint(steps) if steps else m


def traditional(case,pitch=.67):
    p,n=case['p'],case['n']
    f,cal=legacy_frames(case)
    results={}
    times={}
    def run(name,build):
        start=perf_counter()
        try:
            m=build()
            if as_mesh(m):
                results[name]=oriented_shell(m)
        except (RuntimeError,ValueError,ArithmeticError) as exc:
            print(case['name'],name,'error',str(exc)[-480:],flush=True)
        times[name]=perf_counter()-start
        print(case['name'],name,'surface',len(results[name].triangles) if name in results else 'empty',
              f'{times[name]:.2f}s',flush=True)
    cache={}
    def plain():
        a,b=screened(p,n)
        cache['visible']=b
        return a
    run('S01-Poisson',plain)
    if 'visible' in cache and as_mesh(cache['visible']):
        results['S01-visible']=cache['visible']
    def pivot():
        if case['name'] not in PYRAMIDS:
            return bpa(p,n)
        sampled=cloud(p,n).voxel_down_sample(.83).normalize_normals()
        ids=np.random.default_rng(774).choice(len(sampled.points),min(len(sampled.points),27000),
                                               replace=False)
        return bpa(np.asarray(sampled.points)[ids],np.asarray(sampled.normals)[ids])
    run('S02-BPA',pivot)
    g,gn=footprint(p)
    if len(g):
        eligibility,_,_=rays(g,f,cal)
        g,gn=g[eligibility<2],gn[eligibility<2]
    gn=gn/np.maximum(np.linalg.norm(gn,axis=1,keepdims=True),1e-9)
    run('S03-Hull',lambda:hull_mesh(np.vstack([p[p[:,2]>-.45],g])))
    origin,shape,grid=grid_box(np.vstack([p,g]),pitch)
    free,vals,count=rays(grid,f,cal)
    gp,gnp=(polar_guides(p) if case['name']=='小南瓜' else (g,gn))
    if len(gp) and case['name']=='小南瓜':
        v,_,_=rays(gp,f,cal)
        gp,gnp=gp[v<2],gnp[v<2]
    if len(gp):
        gnp=gnp/np.maximum(np.linalg.norm(gnp,axis=1,keepdims=True),1e-9)
    run('S05-GuidedPoisson',lambda:screened(np.vstack([p,gp]),np.vstack([n,gnp]))[0])
    if case['name'] in PYRAMIDS:
        def planar():
            fit=fit_planes(p,n)
            return refine_planar_colour_mesh(fit) if as_mesh(fit) else fit
        o3d.utility.random.seed(201)
        run('S05-Planes',planar)
    hold='S05-GuidedPoisson'
    if case['name']=='小南瓜':
        run('S04-GuideEnvelope',lambda:envelope(screened(np.vstack([p,g]),np.vstack([n,gn]))[0],
                                               p,g,origin,shape,grid,free,pitch,intersections=intersection_pairs))
        if hold in results:
            run('S07-PolarEnvelope',lambda:envelope(results[hold],p,gp,origin,shape,grid,free,pitch,
                                                    intersections=intersection_pairs))
    elif hold in results:
        run('S04-GuideEnvelope',lambda:envelope(results[hold],p,g,origin,shape,grid,free,pitch,
                                                intersections=intersection_pairs))
    run('S06-RawTSDF',lambda:projective_surface(vals,count,origin,shape,pitch))
    return results,times
