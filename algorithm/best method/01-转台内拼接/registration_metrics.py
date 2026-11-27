"""R06c overlap, independent marker and projected-depth checks (E101/E003)."""
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from camera_geometry import project, transform

RAD = (1., 2., 4.)


def cloud(p, radius=None, voxel=None):
    c=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(p,dtype=float)))
    if voxel:
        c=c.voxel_down_sample(voxel)
    if radius and len(c.points)>=5:
        c.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius,max_nn=42))
        c.orient_normals_towards_camera_location(np.zeros(3))
    return c


def ring(n):
    return [(i,(i+1)%n) for i in range(n)]


def rel(poses,a,b):
    return np.linalg.inv(poses[b]) @ poses[a]


def support(a,b,T):
    P,Q = a['train'][::3],b['train'][::3]
    P = transform(P,T)
    return min(np.mean(cKDTree(Q).query(P)[0]<=4),np.mean(cKDTree(P).query(Q)[0]<=4))


def signed_depth(V, T, model, shape):
    H,W = shape
    nw,nh = (int(W/8)+1,int(H/8)+1)
    train = [transform(v['train'][::2],t) for v,t in zip(V,T)]
    rows=[]
    for i,(row,pose) in enumerate(zip(V,T)):
        p = transform(np.concatenate([w for j,w in enumerate(train) if j!=i]),np.linalg.inv(pose))
        x,y = np.rint(project(p,model)/8).T.astype(int)
        good = (x>=0)&(x<nw)&(y>=0)&(y<nh)&(p[:,2]>0)
        bins = np.full(nw*nh,np.inf)
        np.minimum.at(bins,y[good]*nw+x[good],p[good,2])
        uv = np.rint(row['val_uv']/8).astype(int)
        errors = bins[uv[:,1]*nw+uv[:,0]]-row['val'][:,2]
        hit = np.isfinite(errors)
        rows.append(dict(support=float(hit.mean()), agreement=float((abs(errors)<=3).mean()),
                         errors=errors[hit]))
    D = np.concatenate([r['errors'] for r in rows])
    if not len(D):
        return dict(depth_supported=0.,depth_agree_and_supported=0.,depth_abs_P50=None,
                    depth_front_violation=None,depth_behind_fraction=None)
    return dict(depth_supported=np.median([r['support'] for r in rows]),
                depth_agree_and_supported=np.median([r['agreement'] for r in rows]),
                depth_abs_P50=np.median(abs(D)),depth_front_violation=np.mean(D<-3),depth_behind_fraction=np.mean(D>3))


def summarize(v, raw, T, reference, axis, center, model, edges=None):
    r = ring(len(v))
    p = [pair_metric(v[a],v[b],rel(T,a,b)) for a,b in r]
    good = [item for item in p if item['status']=='scored']
    def values(k):
        return [item[k] for item in good if item[k] is not None]
    def median(k):
        x=values(k)
        return float(np.median(x)) if x else None
    B = np.linalg.svd(axis[None,:])[2][1:].T
    xy = (T[:,:3,3]-center) @ B
    ang = np.sort(np.rad2deg(np.arctan2(xy[:,1],xy[:,0])))
    gaps = np.diff(np.r_[ang,ang[0]+360])
    held=[]
    for pose,data in zip(T,raw):
        if len(data['held_xyz']):
            xyz = transform(transform(data['held_xyz'],raw[0]['pose']),np.linalg.inv(pose))
            held.extend(np.linalg.norm(project(xyz,model)-data['held_uv'],axis=1))
    D = T @ np.linalg.inv(reference)
    translation = np.linalg.norm((D[:,:3,:3] @ center + D[:,:3,3])-center,axis=-1)
    closure = edge_closure(edges,r,center) if edges is not None else (None,None)
    return dict(
        F1_1=median('f1_1'),F1_2=median('f1_2'),F1_4=median('f1_4'),
        worst_F1_2=min(values('f1_2'),default=None), F1_2_P10=np.percentile(values('f1_2'),10),
        seam_normal_P95=median('seam_p95'),NN_P95=median('p95'),inlier_RMSE2=median('rmse_2'),
        max_gap=max(gaps),minimum_gap=min(gaps),
        tag_P50=np.median(held) if held else None,tag_P95=np.percentile(held,95) if held else None,
        held_marker_corners=len(held),
        rotation_change_max_deg=np.rad2deg(Rotation.from_matrix(D[:,:3,:3]).magnitude().max()),
        pivot_change_max_mm=np.max(translation),edge_count=len(good),
        independently_solved_edges=len(edges) if edges is not None else None,
        loop_deg=closure[0],loop_pivot_mm=closure[1],
        **signed_depth(v,T,model,raw[0]['shape'])
    ),p


def pair_metric(a,b,T):
    if min(len(a["val"]),len(b["val"]))<20:
        return {"status":"insufficient_validation"}
    A, B=transform(a["val"],T), b["val"]
    da, ia=cKDTree(B).query(A)
    db, ib=cKDTree(A).query(B)
    d=np.r_[da,db]
    z={"status":"scored","source_eval":len(A),"target_eval":len(B)}
    for gate in RAD:
        x,y=(da<=gate).mean(),(db<=gate).mean()
        z.update({f"forward_{gate:g}":float(x),f"backward_{gate:g}":float(y),
                  f"f1_{gate:g}":float(2*x*y/(x+y)) if x+y else 0.})
    m=(d<=2)
    z.update(rmse_2=float(np.sqrt(np.mean(d[m]**2))) if m.any() else None, inliers_2=int(m.sum()),
             median=float(np.median(d)),p95=float(np.percentile(d,95)),
             nn_mean=float(.5*(da.mean()+db.mean())))
    mutual=(ib[ia]==np.arange(len(A)))&(da<=4)
    z["reciprocal_n"]=int(mutual.sum())
    z["seam_p95"]=float(np.percentile(np.abs(np.sum((A[mutual]-B[ia[mutual]])*b["normals"][ia[mutual]],axis=1)),95)) if mutual.sum()>=12 else None
    return z


def edge_closure(edges,ring,center):
    if not all(e in edges for e in ring):
        return None,None
    T=np.eye(4)
    for e in ring: T=edges[e][0]@T
    return float(np.degrees(Rotation.from_matrix(T[:3,:3]).magnitude())),float(np.linalg.norm(transform(center[None],T)[0]-center))


def prepare_tukey_levels(points):
    """Build only the three normal-estimation levels used by R06c."""
    source = cloud(points)
    levels = []
    for step in (2.4, 1.4, .85):
        sampled = source.voxel_down_sample(step)
        sampled.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
            radius=max(3 * step, 2.4), max_nn=40))
        levels.append(sampled)
    return levels


def tukey_icp(source, target, initial):
    """Refine the calibrated relative pose, coarse to fine."""
    registration = o3d.pipelines.registration
    pose = initial.copy()
    for a, b, gate in zip(source, target, (7., 4., 2.5), strict=True):
        loss = registration.TukeyLoss(gate * .75)
        result = registration.registration_icp(
            a, b, gate, pose, registration.TransformationEstimationPointToPlane(loss),
            registration.ICPConvergenceCriteria(max_iteration=50))
        pose = result.transformation
    if not np.isfinite(pose).all() or not np.isclose(np.linalg.det(pose[:3, :3]), 1, atol=.002):
        raise ValueError('ICP returned an invalid rigid transform.')
    return pose
