"""Check the E104 base against the E102 board pixels and trace the physical datum."""
import argparse
import csv
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from experiment import HERE, read, real_input
from reconstruct import FLOOR_EPS, scene


def overlap_checker(path):
    model=o3d.io.read_triangle_mesh(str(path))
    vertices=np.asarray(model.vertices)
    faces=np.asarray(model.triangles)
    floor=faces[(abs(vertices[faces,2])<=FLOOR_EPS).all(1)]
    if not len(floor):
        raise ValueError(f'Expected coplanar triangles in {path}')
    plane=o3d.geometry.TriangleMesh(model.vertices,o3d.utility.Vector3iVector(floor))
    sc=scene(plane)
    edges=np.concatenate((floor[:,[0,1]],floor[:,[1,2]],floor[:,[2,0]]))
    unique,n=np.unique(np.sort(edges,axis=1),axis=0,return_counts=True)
    ends=vertices[unique[n==1],:2]
    edge_xy=np.concatenate([a*ends[:,0]+(1-a)*ends[:,1] for a in np.linspace(0,1,7)])
    boundary=cKDTree(edge_xy)

    def inside(xy):
        rays=np.c_[xy,np.ones(len(xy)),np.zeros((len(xy),2)),-np.ones(len(xy))]
        hit=np.isfinite(sc.cast_rays(o3d.core.Tensor(rays.astype('float32')),nthreads=1)['t_hit'].numpy())
        return hit & (boundary.query(xy)[0]>1.5)
    return inside


def main(baseline):
    rows=[]
    models=read(HERE/'推荐.json')
    for group,row in models.items():
        code,branch=group.split('-')
        _,_,cal,plane,_,_,_=real_input(code,branch,with_board_evidence=True)
        traces=cal['exposed_board']
        method=row['watertight']
        folder=HERE/'场景'/group
        if method is None:
            source=read(folder/'结果.json')['tests']['T0']['methods']
            method=next(k for k in ('S04','S07','S05','S03')
                        if source.get(k,{}).get('watertight') and source[k]['touches_floor'])
        after=overlap_checker(folder/f'T0-{method}.ply')
        old=(baseline/f'{group}.ply') if baseline else None
        before=overlap_checker(old) if old is not None and old.is_file() else None
        prior=[]
        final=[]
        deviations=[]
        datum_shift=[]
        for i,board in enumerate(traces):
            q=board[::3]
            if i:
                deviations.append(np.median(abs(q[:,2])))
                datum_shift.append(abs(np.median(q[:,2])))
            final.append(after(q[:,:2]).sum())
            if before is not None:
                prior.append(before(q[:,:2]).sum())
        observed=sum(len(z[::3]) for z in traces)
        rows.append(dict(scene=group,method=method,board_baseline='E102/V00/E102-E103',
                         source_board_fit_points=plane['deck_points'],visible_board_views=len(traces),
                         exposed_points_total=sum(len(z) for z in traces),
                         independent_station_median_abs_z_mm=np.median(deviations),
                         largest_abs_station_median_z_mm=max(datum_shift),
                         board_test_points=observed,
                         overlap_at_least_1p5mm_before=sum(prior)/observed if prior else None,
                         overlap_at_least_1p5mm_after=sum(final)/observed))
        print(rows[-1],flush=True)
    with (HERE/'E102支撑面核验.csv').open('w',newline='',encoding='utf-8-sig') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--prior',type=Path)
    main(p.parse_args().prior)
