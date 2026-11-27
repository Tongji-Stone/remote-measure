"""E203 F03 measured-point selection, depth voting, and exposure checks."""
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from registration import make
from scan_io import project, transform


def one_per_voxel(points, spacing, order=None):
    coordinate = np.floor(points/spacing)
    closeness = np.linalg.norm(points/spacing-coordinate-.5, axis=1)
    priority = closeness if order is None else order + closeness
    idx = np.argsort(priority, kind='stable')
    _, chosen = np.unique(coordinate[idx].astype(np.int32), axis=0, return_index=True)
    return np.sort(idx[chosen])


def votes(x, camera, models, own):
    K,D = camera
    conflicts = np.zeros(len(x), np.int16)
    supports = np.zeros(len(x), np.int16)
    valid_count = np.zeros(len(x), np.int16)
    for placement, M, front, back, good in models:
        if placement == own:
            continue
        z=transform(x,M)
        h,w=good.shape
        uv=np.rint(project(z,K,D)/3).astype('int32')
        u,v=uv.T
        valid=(z[:,2]>0)&(u>=0)&(v>=0)&(u<w)&(v<h)
        u,v = np.clip(u,0,w-1), np.clip(v,0,h-1)
        valid &= good[v,u]
        valid_count += valid
        conflicts += valid & (z[:,2] < front[v,u]-2.3)
        supports += valid & (z[:,2] > front[v,u]-1.55) & (z[:,2] < back[v,u]+1.55)
    return conflicts, supports, valid_count


def sample_scans(scans, poses, camera, models):
    output = []
    for i,(scan,M) in enumerate(zip(scans,poses)):
        # Reuse the raw vertex chosen nearest the voxel center.
        x=transform(scan['xyz'],M)
        rgb=scan['rgb']
        ind=one_per_voxel(x,.43)
        dist=cKDTree(x[ind]).query(x[ind],k=8)[0][:,-1]
        norm=make(x[ind],rgb[ind]/255,.85)
        simp=np.asarray(norm.points)
        n=np.asarray(norm.normals)[cKDTree(simp).query(x[ind])[1]]
        v,h,known=votes(x[ind],camera,models,i)
        output.append(dict(xyz=x[ind],rgb=rgb[ind],index=ind,spacing=dist,normals=n,
                           front=v,support=h,known=known))
    return output


def duplicates(x, n, y, m):
    if not len(y):
        return np.zeros(len(x), dtype=bool)
    distance, j=cKDTree(y).query(x,k=list(range(1, min(8,len(y))+1)))
    z=n[:,None,:]
    aligned=(np.abs(np.einsum('nij,nij->ni',m[j],np.broadcast_to(z,m[j].shape)))>.83)
    step=x[:,None,:]-y[j]
    normal=np.abs(np.einsum('nij,nij->ni',step,m[j]))
    lateral=np.sqrt(np.maximum(0.,distance**2-normal**2))
    return (distance[:,0]<.95)|((aligned&(lateral<.89)&(normal<2.7)).any(axis=1))


def main_component(x):
    if not len(x):
        return np.zeros(0, dtype=bool)
    cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(x))
    labels=np.asarray(cloud.cluster_dbscan(eps=1.65,min_points=1))
    count=np.bincount(labels)
    large=count[labels]>=max(34, int(.003*len(x)))
    if not large.any():
        return large
    # Retain small measured components when close to the main surface.
    return large | (cKDTree(x[large]).query(x)[0] <= 2.2)


def pick(scans, method):
    sets=[]
    for i,s in enumerate(scans):
        x=s['xyz']
        strong = (s['front']>=2)&(s['front']>s['support']+1)
        take=np.ones(len(x),bool)
        if method=='F03 surface + depth':
            limit=2.0 if i==0 else 1.8
            take &= (s['spacing']<limit) & ~strong
        if i and method!='F01 voxel union':
            y=np.vstack([scans[j]['xyz'][g] for j,g in enumerate(sets)])
            if method=='F02 distance':
                take &= cKDTree(y).query(x)[0]>.95
            else:
                n=np.vstack([scans[j]['normals'][g] for j,g in enumerate(sets)])
                take &= ~duplicates(x,s['normals'],y,n)
        sets.append(take)
    points=np.vstack([s['xyz'][g] for s,g in zip(scans,sets)])
    source=np.concatenate([np.full(g.sum(),j,np.int8) for j,g in enumerate(sets)])
    local=np.concatenate([np.flatnonzero(g) for g in sets])
    best=one_per_voxel(points,.47,.18*source)
    if method=='F03 surface + depth':
        best=best[main_component(points[best])]
    source,local=source[best],local[best]
    points=points[best]
    def read(field):
        empty=np.empty((len(best),)+scans[0][field].shape[1:],scans[0][field].dtype)
        for k,s in enumerate(scans):
            mask=source==k
            empty[mask]=s[field][local[mask]]
        return empty
    return dict(xyz=points,source=source,index=read('index'),rgb=read('rgb'),
                front=read('front'),support=read('support'))


def coverage(candidate,sampled):
    x=candidate['xyz']
    tree=cKDTree(x)
    base=cKDTree(sampled[0]['xyz'])
    row=dict(points=len(x), sources=np.bincount(candidate['source'],minlength=len(sampled)),
             confirmed_cross_depth_conflict=float(np.mean((candidate['front']>=2)&
                              (candidate['front']>candidate['support']+1))),
             newly_sampled_further_than_2mm=int(np.sum(base.query(x)[0]>2.)))
    rows=[]
    for i,s in enumerate(sampled):
        valid=(s['spacing']<1.8)&~((s['front']>=2)&(s['front']>s['support']+1))
        q=s['xyz'][valid]
        d=tree.query(q)[0]
        before=base.query(q)[0]
        if not len(q):
            rows.append(dict(id=i, measured_evaluation_points=0))
            continue
        new=before>2.
        rows.append(dict(id=i, measured_evaluation_points=len(q),
                         before_coverage_1mm=np.mean(before<1.), fused_coverage_1mm=np.mean(d<1.),
                         before_coverage_2mm=np.mean(before<2.), fused_coverage_2mm=np.mean(d<2.),
                         measured_missing_from_primary=len(q[new]),
                         recovered_missing_within_1mm=np.mean(d[new]<1.) if np.any(new) else None))
    return dict(**row,coverage=rows)


def exchange_colors(p,sampled):
    """Apply a mild global per-channel shift only if spatially held-out overlaps agree."""
    color=p['rgb'].copy()
    base=sampled[0]
    tree=cKDTree(base['xyz'])
    stats=[]
    for i,s in enumerate(sampled[1:],1):
        d,k=tree.query(s['xyz'])
        take=(d<.68)&(s['support']>=1)
        a,b=s['rgb'][take]/255.,base['rgb'][k[take]]/255.
        lab=np.floor(s['xyz'][take]/4.0).astype(int).sum(1)%5==0
        if min(lab.sum(),(~lab).sum())<30:
            stats.append(dict(source=i,used=False,reason='insufficient_overlap'))
            continue
        # Median shift preserves the relative contrast and hue gradients of the source texture.
        shift=np.clip(np.median(b[~lab]-a[~lab],axis=0),-.065,.065)
        before=np.mean(np.mean(abs(a[lab]-b[lab]),axis=1))
        after=np.mean(np.mean(abs(np.clip(a[lab]+shift,0,1)-b[lab]),axis=1))
        use=bool(after<before*.95)
        if use:
            # Gray fallback points have no measured color to correct.
            add=(p['source']==i)&(np.ptp(color,axis=1)>2)
            color[add]=np.rint(255*np.clip(color[add]/255+shift,0,1)).astype(np.uint8)
        stats.append(dict(source=i,used=use,RGB_shift_0_1=shift,
                          held_before=before,held_after=after,held_points=int(lab.sum())))
    return color,stats
