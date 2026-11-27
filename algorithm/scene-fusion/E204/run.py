"""Run E204, export vertex-only PLYs, and draw comparable views."""
import argparse

import cv2 as cv
import numpy as np
from scipy.spatial import cKDTree

from core import GROUPS, PITCH, ROOT, estimate, get_case, one_per_voxel, transform, write_ply
from preview import fit, panel, view
from prepare import dump


def color_model(case, samples):
    """Blend real colors with per-placement exposure alignment and a local detail weight."""
    base, rgb = case['baseline']
    rgb = rgb.astype(float)/255.
    traces = [base]
    colors = [rgb]
    quality = [np.where(np.ptp(rgb,axis=1)>.008, 2.25,.36)]
    corrections = []
    reference = cKDTree(base)
    for i,(s,P) in enumerate(zip(case['scans'],case['poses'])):
        x = transform(s['xyz'],P)
        color = s['rgb'].astype(float)/255.
        k = np.flatnonzero(np.ptp(color,axis=1)>.009)
        d,j = reference.query(x[k])
        use=(d<.45)&(np.ptp(rgb[j],axis=1)>.025)
        # Pairwise correction is robust to remaining local occlusion or projector glare.
        a,b=color[k[use]],rgb[j[use]]
        zones=np.floor(x[k[use]]/4.0).sum(1).astype(int)%5==0
        shift=np.zeros(3)
        old,new=None,None
        if np.sum(zones)>25 and np.sum(~zones)>25:
            delta=np.clip(np.median((b-a)[~zones],axis=0),-.10,.10)
            old=float(np.median(np.mean(abs(b[zones]-a[zones]),axis=1)))
            new=float(np.median(np.mean(abs(b[zones]-(a[zones]+delta).clip(0,1)),axis=1)))
            if new<.97*old:
                shift=delta
        ids=k[one_per_voxel(x[k],.52)]
        traces.append(x[ids]); colors.append((color[ids]+shift).clip(0,1))
        quality.append(np.ones(len(ids)))
        corrections.append(dict(source=i,shift_rgb=shift,held_before=old,held_after=new))
    X=np.vstack(traces); C=np.vstack(colors); Q=np.concatenate(quality)
    tree=cKDTree(X)
    depth,k=tree.query(samples,k=32)
    near=depth[:,0]
    w=np.exp(-.5*(depth/.54)**2)*Q[k]
    # Preserve actual color boundaries as well as a continuous scan transition.
    ref=C[k[:,0]]
    difference=np.sqrt(np.mean((C[k]-ref[:,None,:])**2,axis=-1))
    l=np.ptp(ref,axis=1)>.015
    w*=np.where(l[:,None],.24+.76*np.exp(-.5*(difference/.16)**2),1.0)
    total=np.sum(w,axis=1,keepdims=True)
    mixed=np.einsum('ij,ijk->ik',w,C[k])/np.maximum(total,1e-8)
    mixed[total[:,0]<1e-7]=ref[total[:,0]<1e-7]
    return np.rint(255*mixed.clip(0.,1.)).astype('uint8'), near, corrections


def geometric_gray(p,M):
    # A depth-shaded geometry preview. Colors in PLY are independent of this shading.
    from register import make
    cloud=make(p,np.ones_like(p)*.6,.78)
    q,n=np.asarray(cloud.points),np.asarray(cloud.normals)
    light=.22+.65*np.abs(n@(M@np.array([-.21,.37,.90])))
    normal=light[cKDTree(q).query(p)[1]]
    return np.repeat(np.clip(255*normal,0,255).astype('uint8')[:,None],3,axis=1)


def preview_pair(path,obj,base,result,inserted):
    before,c=base
    after,r=result
    angles=[(28,12),(-90,22),(130,30),(40,-72)]
    if obj=='小南瓜':
        angles=[(28,10),(-90,16),(120,12),(35,-77)]
    rows=[]
    layers=[]
    for j,(a,b) in enumerate(angles):
        M=view(a,b); dims=(410,300)
        center,scale=fit(np.vstack([before,after]),M,*dims,1.18)
        columns=[panel((before,c),M,center,scale,*dims,True),
                 panel((after,r),M,center,scale,*dims,True),
                 panel((before,geometric_gray(before,M)),M,center,scale,*dims,True),
                 panel((after,geometric_gray(after,M)),M,center,scale,*dims,True)]
        labels=['E203 RGB','E204 RGB','E203 Geometry','E204 Geometry']
        for col,label in zip(columns,labels):
            cv.putText(col,label,(12,22),0,.55,(231,231,231),1,cv.LINE_AA)
        rows.append(np.hstack(columns))
        rgb=np.tile(np.array([149,196,222],np.uint8),(len(after),1))
        rgb[inserted]=[247,157,69]
        image=panel((after,rgb),M,center,scale,*dims,True)
        cv.putText(image,'Orange: points added by local interpolation',(9,22),0,.45,(231,231,231),1,cv.LINE_AA)
        layers.append(image)
    cv.imwrite(str(path/'断层对照.png'),np.vstack(rows))
    cv.imwrite(str(path/'插值区域.png'),np.hstack([np.vstack(layers[:2]),np.vstack(layers[2:])]))


def export(obj,pitch=PITCH):
    cv.setNumThreads(1)
    path=ROOT/obj
    path.mkdir(exist_ok=True)
    case=get_case(obj)
    models,debug=estimate(case['frames'],case['camera'],pitch,vertex_metadata=True)
    inserted,shifts=debug.pop('vertex_arrays')
    out=models['B05-Anchor']
    rgb,_,color=color_model(case,out)
    merged=np.vstack([transform(f.camera_xyz,np.linalg.inv(f.world_to_camera)) for f in case['frames']])
    # Separate interpolated neighborhoods from vertices close to a measured sample.
    rawdist=cKDTree(merged).query(out)[0]
    far=rawdist>.75
    h=np.linalg.inv(case['scans'][0]['B'])
    write_ply(path/'融合.ply',transform(out,h),rgb)
    preview_pair(path,obj,case['baseline'],(out,rgb),inserted)
    # This is a vertex-aligned attribute array, separate from XYZ and RGB in the PLY.
    np.savez_compressed(path/'支撑距离.npz',measurement_distance_mm=rawdist,
                        inserted_by_local_field=inserted,anchor_normal_shift_mm=shifts)
    primary=case['baseline'][0]
    fwd=cKDTree(out).query(primary)[0]
    bwd=cKDTree(primary).query(out)[0]
    dump(path/'融合核验.json',dict(output_vertices=len(out),farther_than_measured_075mm=int(far.sum()),
                                surface_variants={k:len(x) for k,x in models.items()},
                                old_to_new_P50_mm=np.median(fwd),old_to_new_P95_mm=np.percentile(fwd,95),
                                new_to_old_P50_mm=np.median(bwd),new_to_old_P95_mm=np.percentile(bwd,95),
                                old_to_new_within_1mm=np.mean(fwd<1.),color=color,
                                pitch_mm=pitch,**debug))
    print(obj,len(out),far.sum(),np.percentile(fwd,[50,95]),np.percentile(bwd,[50,95]),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--object',choices=GROUPS)
    parser.add_argument('--pitch',type=float,default=PITCH)
    opts=parser.parse_args()
    for name in ([opts.object] if opts.object else GROUPS):
        export(name,opts.pitch)
