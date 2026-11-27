"""Synthetic sensitivity to a systematic depth bias and finite sampling resolution."""
from dataclasses import replace

import cv2 as cv
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from core import ROOT, estimate
from evaluate import fixtures, pooled, rays_for
from prepare import dump


def bias_probes():
    _, sphere = fixtures()[0]
    case=rays_for(sphere,'sphere')
    reliable=pooled(case['frames'])
    rng=np.random.default_rng(137)
    train=[]
    for f in case['frames']:
        sign=f.station%2
        p=f.camera_xyz.copy()
        dz=(2*sign-1)*.30 + rng.normal(0.,.12,len(p))
        p+=p/p[:,2:3]*dz[:,None]
        train.append(replace(f,camera_xyz=p,placement=sign))
    variants,_=estimate(train,case['camera'])
    variants['B00-measured']=pooled(train)
    data=[]
    reference=cKDTree(reliable)
    for name,p in variants.items():
        # Score the regions the synthetic optical setup observes.
        q=p[reference.query(p)[0]<.85]
        error=np.linalg.norm(q,axis=1)-13.
        data.append(dict(method=name,scored_points=len(error),
                         absolute_radial_p95_mm=np.percentile(abs(error),95),
                         absolute_radial_median_mm=np.median(abs(error))))
    return data


def groove_probes():
    rows=[]
    for width in (1.2,1.8,2.8):
        x=.5*width
        groove=o3d.geometry.TriangleMesh.create_box(24,16,1.9).translate((-12,-8,-6.4))
        for loc in (-12,x):
            groove+=o3d.geometry.TriangleMesh.create_box(12-x,16,6.5).translate((loc,-8,-4.5))
        case=rays_for(groove,'channel')
        variants,_=estimate(case['frames'],case['camera'])
        for name,p in variants.items():
            bad=(abs(p[:,0])<x-.22)&(abs(p[:,1])<6.3)&(p[:,2]>-3.5)
            rows.append(dict(width_mm=width,method=name,false_free_points=int(bad.sum()),
                             output_points=len(p)))
    return rows


if __name__=='__main__':
    cv.setNumThreads(1)
    d=dict(signed_two_group_bias_mm=.30,bias_noise_sigma_mm=.12,
           biased_sphere=bias_probes(),groove_widths=groove_probes())
    dump(ROOT/'精度与窄槽压力测试.json',d)
    print(d,flush=True)
