"""E204 B05 local TSDF/MLS fusion with measured anchors and a free-space gate."""
from dataclasses import dataclass

import cv2 as cv
import numpy as np
import open3d as o3d
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree

from measured_fusion import one_per_voxel
from scan_io import project, transform

PITCH = .40
BAND = 2.65
BIN = 3


@dataclass
class Frame:
    camera_xyz: np.ndarray
    world_to_camera: np.ndarray
    placement: int
    station: int


def make_depth(points, normals, K, D, size, bin_width):
    h,w = np.ceil(np.asarray(size) / bin_width).astype(int) + 1
    xy = np.rint(project(points, K, D) / bin_width).astype(int)
    u,v = xy.T
    good = (u>=0)&(u<w)&(v>=0)&(v<h)&(points[:,2]>0)
    idx = np.flatnonzero(good)
    # Near surface first in a bin receiving duplicate samples.
    order = idx[np.argsort(points[idx,2])[::-1]]
    depth = np.zeros((h,w),np.float32)
    incidence = np.zeros_like(depth)
    depth[v[order],u[order]] = points[order,2]
    cosine = -np.einsum('ij,ij->i', points, normals) / np.linalg.norm(points,axis=1)
    incidence[v[order],u[order]] = np.maximum(cosine[order],.1)
    valid = depth>0
    kernel = np.ones((3,3),np.uint8)
    hi = cv.dilate(depth,kernel)
    lo = cv.erode(np.where(valid,depth,1e5).astype('float32'),kernel)
    count = cv.boxFilter(valid.astype('uint8'),-1,(3,3),normalize=False)
    stable = valid&(count>=4)&(hi-lo<2.6)
    return depth,incidence,stable


def prepare(frames, camera, bin_width=BIN):
    samples = []
    models = []
    K,D,size = camera
    for f in frames:
        p = f.camera_xyz
        if len(p)<24:
            continue
        cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p))
        cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=1.75,max_nn=30))
        cloud.orient_normals_towards_camera_location()
        n = np.asarray(cloud.normals)
        incidence = (-np.einsum('ij,ij->i',p,n)/np.linalg.norm(p,axis=1)).clip(0.,1.)
        depth,cos,stable = make_depth(p,n,K,D,size,bin_width)
        models.append((f.world_to_camera,depth,cos,stable))
        pos = np.linalg.inv(f.world_to_camera)
        k = cKDTree(p).query(p,k=6)[0][:,-1]
        use=(k<1.55)&(incidence>.19)
        samples.append((transform(p[use],pos),n[use]@pos[:3,:3].T,incidence[use],
                        np.full(use.sum(),f.placement)))
    if not samples or sum(len(sample[0]) for sample in samples) < 24:
        raise ValueError('Too few valid depth samples for B05.')
    x,n,c,group=map(np.concatenate,zip(*samples))
    keep=[]
    for s in np.unique(group):
        i=np.flatnonzero(group==s)
        keep.append(i[one_per_voxel(x[i],.56, .5*(1-c[i]))])
    keep=np.concatenate(keep)
    p,n,c=x[keep],n[keep],c[keep]
    # Voxel support is queryable across sources; isolated samples are excluded.
    d=cKDTree(p).query(p,k=7)[0][:,-1]
    use=d<1.9
    if use.sum() < 24:
        raise ValueError('Too few supported depth samples for B05.')
    return p[use],n[use],c[use],models


def projection(points, models, camera, bin_width=BIN):
    K,D,_=camera
    weight=np.zeros(len(points),np.float32)
    offset=np.zeros_like(weight)
    free=np.zeros(len(points),np.uint8)
    hit=np.zeros(len(points),np.uint8)
    for M,dep,inc,stable in models:
        p=transform(points,M)
        h,w=dep.shape
        fx,fy=np.rint(project(p,K,D)/bin_width).astype(int).T
        ids=np.flatnonzero((fx>=0)&(fx<w)&(fy>=0)&(fy<h)&(p[:,2]>0))
        d=dep[fy[ids],fx[ids]]
        good=d>0
        j=ids[good]
        c=inc[fy[j],fx[j]]
        consistent=stable[fy[j],fx[j]]
        along=(d[good]-p[j,2])*c
        valid=(along>-2.0)&(along<2.0)&(c>.27)
        wgt=c*c*np.where(consistent,1.,.22)
        weight[j[valid]]+=wgt[valid]
        offset[j[valid]]+=(wgt*along)[valid]
        free[j]+=(consistent&(c>.38)&(along>1.18))
        hit[j]+=(consistent&(c>.38)&(abs(along)<.55))
    return np.divide(offset,weight,out=np.zeros_like(weight),where=weight>0),weight,free,hit


def clearance(points, models, camera, bin_width=BIN):
    """Count depth rays traversing the candidate before reaching a measured surface."""
    K,D,_=camera
    free=np.zeros(len(points),np.uint8)
    near=np.zeros(len(points),np.uint8)
    for M,depth,inc,stable in models:
        p=transform(points,M)
        h,w=depth.shape
        u,v=np.rint(project(p,K,D)/bin_width).astype(int).T
        ids=np.flatnonzero((u>=0)&(u<w)&(v>=0)&(v<h)&(p[:,2]>0))
        i,j=v[ids],u[ids]
        d=depth[i,j]-p[ids,2]
        good=depth[i,j]>0
        c=inc[i,j]
        border=~stable[i,j]
        # A large positive depth remains useful at narrow silhouettes, but needs repeats.
        free[ids]+=good&(c>.22)&(d>np.where(border,2.55,1.35))
        near[ids]+=good&(c>.40)&~border&(abs(d)<.75)
    return free,near


def narrow_grid(points,pitch):
    lower=np.floor((np.percentile(points,.025,axis=0)-3.2)/pitch)*pitch
    size=tuple(np.ceil((np.percentile(points,99.975,axis=0)+3.2-lower)/pitch).astype(int)+1)
    if np.prod(size)>14_000_000:
        raise ValueError(f'Surface grid {size} too large at pitch {pitch:.3f} mm')
    grid=np.zeros(size,bool)
    a=np.rint((points-lower)/pitch).astype(int)
    a=a[((a>=0)&(a<np.array(size))).all(axis=1)]
    grid[tuple(a.T)]=True
    band=distance_transform_edt(~grid)<(BAND/pitch+.5)
    ij=np.column_stack(np.nonzero(band))
    return size,lower,ij,lower+ij*pitch


def implicit(points,x,n,c):
    tree=cKDTree(x)
    values=np.zeros(len(points),np.float32)
    support=np.zeros(len(points),bool)
    bridge=np.zeros_like(support)
    nearest=np.zeros_like(values)
    for ids in np.array_split(np.arange(len(points)),1+len(points)//12000):
        q=points[ids]
        d,idx=tree.query(q,k=min(len(x),52))
        raw=n[idx]
        N=raw[:,0,:]
        dot=np.einsum('ijk,ik->ij',raw,N)
        delta=q[:,None,:]-x[idx]
        signed=np.einsum('ijk,ijk->ij',delta,raw)
        w=np.exp(-.5*(d/1.18)**2)*c[idx]**1.5*(dot>.63)*(d<3.05)
        mu=np.sum(w*signed,axis=1)/np.maximum(w.sum(1),1e-8)
        w*=1/(1+((signed-mu[:,None])/.62)**2)
        mass=w.sum(1)
        z=np.sum(w*signed,axis=1)/np.maximum(mass,1e-8)
        scatter=np.sqrt(np.sum(w*(signed-z[:,None])**2,axis=1)/np.maximum(mass,1e-8))
        support[ids]=(d[:,0]<1.0)&(mass>.38)&(scatter<.82)
        nearest[ids]=d[:,0]
        values[ids]=z
        # Points in a hole need well-spread boundary samples in their tangent plane.
        extra=np.flatnonzero((d[:,0]<2.12)&(d[:,0]>=.73)&(mass>.42)&(scatter<.54))
        if len(extra):
            N=N[extra]
            axes=np.eye(3)[np.argmin(np.abs(N),axis=1)]
            u=np.cross(N,axes)
            u/=np.linalg.norm(u,axis=1,keepdims=True)
            v=np.cross(N,u)
            a=np.einsum('ijk,ik->ij',delta[extra],u)
            b=np.einsum('ijk,ik->ij',delta[extra],v)
            theta=np.arctan2(b,a)+np.pi
            live=(w[extra]>.012)&(np.hypot(a,b)>.45)
            # Duplicate the first angle at +2π to measure the wrap interval.
            theta=np.sort(np.where(live,theta,20.),axis=1)
            m=live.sum(1)
            max_gap=np.max(np.where(np.arange(theta.shape[1]-1)<m[:,None]-1,
                                    np.diff(theta,axis=1),0.),axis=1)
            wrap=theta[:,0]+2*np.pi-theta[np.arange(len(m)),np.maximum(0,m-1)]
            spread=np.maximum(max_gap,wrap)
            bridge[ids[extra]]=(m>=7)&(spread<2.70)
    return values,support,bridge,nearest


def crossings(field,mask):
    parts=[]
    for axis in range(3):
        hi=[slice(None)]*3
        lo=hi.copy()
        hi[axis],lo[axis]=slice(1,None),slice(None,-1)
        hi,lo=tuple(hi),tuple(lo)
        a,b=field[lo],field[hi]
        take=(a>=0)!=(b>=0)
        take&=mask[lo]&mask[hi]
        p=np.column_stack(np.nonzero(take)).astype(float)
        p[:,axis]+=a[take]/(a[take]-b[take])
        parts.append(p)
    return np.concatenate(parts)


def combine_measured(p,n,c,y,ray,agree,pitch):
    """Keep measured geometry at high curvature and interpolate gaps without depth conflicts."""
    d,idx=cKDTree(y).query(p,k=7)
    span,j=cKDTree(p).query(p,k=13)
    normal_agreement=np.mean(np.einsum('ijk,ik->ij',n[j],n)>.82,axis=1)
    delta=y[idx]-p[:,None,:]
    normal_offset=np.einsum('ijk,ik->ij',delta,n)
    moment=np.median(normal_offset,axis=1)
    coherent=np.median(abs(normal_offset-moment[:,None]),axis=1)<.23
    adjust=(d[:,0]<.63)&(normal_agreement>.84)&(span[:,-1]<1.55)&coherent
    # Keep the measurement as an anchor; move only along its normal, with a hard cap.
    correction=np.where(adjust,.65*np.clip(moment,-.35,.35),0)
    update=p+correction[:,None]*n
    keep=one_per_voxel(update,pitch*.98,.23*(1-c))
    measured=update[keep]
    gap=cKDTree(measured).query(y)[0]>.57
    clear=(ray==0)|((ray==1)&(agree>=2))
    inferred=y[gap&clear]
    y=np.vstack([measured,inferred])
    ids=one_per_voxel(y,pitch*.84, .24*np.r_[np.zeros(len(measured)),np.ones(len(inferred))])
    inserted=ids>=len(measured)
    shift=np.r_[correction[keep],np.full(len(inferred),np.nan)][ids]
    stats=dict(anchor_points=int((~inserted).sum()), input_anchors_eligible=int(adjust.sum()),
               extra_candidates=len(inferred), inserted_points=int(inserted.sum()),
               anchor_max_shift_mm=float(np.nanmax(abs(shift))))
    return y[ids],stats,(inserted,shift)


def estimate(frames,camera,pitch=PITCH,bin_width=BIN,*,vertex_metadata=False):
    """Build TSDF/MLS candidates and keep B05 repairs supported by the measured views."""
    p,n,c,models=prepare(frames,camera,bin_width)
    size,lo,ij,xyz=narrow_grid(p,pitch)
    inds=tuple(ij.T)
    tsdf,weights,_,_=projection(xyz,models,camera,bin_width)
    mls,local,bridge,dist=implicit(xyz,p,n,c)
    direct=(weights>.34)&(dist<1.25)
    hybrid=(direct|local|bridge)
    blend=np.clip(weights/(weights+1.05),0,.72)
    value=np.where(local|bridge,blend*tsdf+(1-blend)*mls,tsdf)
    value[weights<=.12]=mls[weights<=.12]
    methods={}
    for tag,vec,mask in [('B01-TSDF',tsdf,direct),('B02-MLS',mls,local|bridge),
                         ('B03-Hybrid',value,hybrid)]:
        field=np.zeros(size,np.float32)
        supported=np.zeros(size,bool)
        field[inds]=vec
        supported[inds]=mask
        y=lo+crossings(field,supported)*pitch
        y=y[one_per_voxel(y,pitch*.88)]
        # Evaluate free space at the surface, not at the grid cell above it.
        _,_,fail,agree=projection(y,models,camera,bin_width)
        known=(fail>=2)&(fail>agree+1)
        k=np.ones(len(y),bool) if tag=='B02-MLS' else ~known
        methods[tag]=y[k]
    y=methods['B03-Hybrid']
    d=cKDTree(p).query(y)[0]
    ray,agree=clearance(y,models,camera,bin_width)
    # Make the guard tighter as indirect reconstruction moves away from measured surfaces.
    credible=(d<=.48)|((ray<2)&((ray==0)|(agree>=1)|(d<=.82)))
    credible|=(d<=.75)&(agree>=2)&(ray<agree)
    methods['B04-Clearance']=y[credible]
    methods['B05-Anchor'],anchor,meta=combine_measured(p,n,c,y[credible],ray[credible],agree[credible],pitch)
    stats=dict(input_oriented_samples=len(p),query_voxels=len(xyz),
               active_bridge_voxels=int(np.sum(bridge&~local&~direct)),
               rejected_inferred_points=int((~credible).sum()),**anchor)
    if vertex_metadata:
        stats['vertex_arrays']=meta
    return methods,stats


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
