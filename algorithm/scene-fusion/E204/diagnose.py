"""Inspect the reported regions without introducing object-specific fusion parameters."""

import cv2 as cv
import numpy as np
from scipy.spatial import cKDTree

from core import GROUPS, ROOT, get_case, read_ply, transform
from evaluate import pooled
from prepare import dump
from preview import fit, panel, view
from run import geometric_gray

# Display and scoring crops only. Core fusion receives the full data.
CROPS = {'小南瓜': ((-1.4, 9.2), [(27, 7), (-90, 7)]),
         '坐着人偶': ((18., 31.), [(28, 11), (-113, 13)])}


def region(ids, x, low, high):
    return ids & (x[:,2] >= low) & (x[:,2] <= high)


def inspect(name):
    case=get_case(name)
    folder=ROOT/name
    old,c=case['baseline']
    new=read_ply(folder/'融合.ply')
    p=transform(new['xyz'],case['scans'][0]['B'])
    r=new['rgb']
    l,h=CROPS[name][0]
    a,b=(region(np.ones(len(q),bool),q,l,h) for q in (old,p))
    references=pooled(case['frames'])
    references=references[region(np.ones(len(references),bool),references,l,h)]
    d0,d1=(cKDTree(q).query(references)[0] for q in (old,p))
    diag=dict(height_band_mm=[l,h],reference_measured_points=len(references),
              before_p95_mm=np.percentile(d0,95),after_p95_mm=np.percentile(d1,95),
              before_fraction_within_half_mm=np.mean(d0<.5),after_fraction_within_half_mm=np.mean(d1<.5))
    E203=ROOT.parent/'E203'/name
    with np.load(E203/'测点索引.npz') as data:
        source=data['source']
    d=cKDTree(p)
    gap_before=[]
    gap_after=[]
    probe=np.flatnonzero(a)
    for label in np.unique(source):
        i=probe[source[probe]==label]
        alt=np.flatnonzero(source!=label)
        dis,j=cKDTree(old[alt]).query(old[i])
        use=(dis>.78)&(dis<1.45)
        i,j=i[use],alt[j[use]]
        d1,k=d.query(old[i]);d2,k2=d.query(old[j])
        use=(d1<.65)&(d2<.65)&(k!=k2)
        gap_before.append(np.mean(abs(c[i[use]].astype(float)-c[j[use]]),axis=1))
        gap_after.append(np.mean(abs(r[k[use]].astype(float)-r[k2[use]]),axis=1))
    pre,post=np.concatenate(gap_before),np.concatenate(gap_after)
    diag.update(cross_source_pair_count=len(pre),cross_source_pair_distance_mm=[.78,1.45],
                cross_source_rgb_jump_before_median=np.median(pre) if len(pre) else None,
                cross_source_rgb_jump_after_median=np.median(post) if len(post) else None)
    dump(folder/'接缝诊断.json',diag)

    rows=[]
    for yaw,elev in CROPS[name][1]:
        M=view(yaw,elev)
        w,h=480,224
        center,scale=fit(np.vstack([old[a],p[b]]),M,w,h,1.12)
        elems=[(old,c,'E203 RGB'),(p,r,'E204 RGB'),
               (old,geometric_gray(old,M),'E203 Geometry'),(p,geometric_gray(p,M),'E204 Geometry')]
        row=[]
        for pts,rgb,label in elems:
            pic=panel((pts,rgb),M,center,scale,w,h,True)
            cv.rectangle(pic,(5,6),(136,26),(36,45,53),-1)
            cv.putText(pic,label,(11,21),0,.48,(231,232,231),1,cv.LINE_AA)
            row.append(pic)
        rows.append(np.hstack(row))
    cv.imwrite(str(folder/'局部放大.png'),np.vstack(rows))
    print(name,diag,flush=True)


if __name__=='__main__':
    cv.setNumThreads(1)
    for case in GROUPS:
        inspect(case)
