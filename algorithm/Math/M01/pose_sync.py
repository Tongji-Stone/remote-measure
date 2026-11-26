"""Anchor the exact part of small pose corrections via a weighted graph Poisson solve."""
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation


def skew(v):
    x,y,z=v
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def jacobian(w):
    a=np.linalg.norm(w)
    h=skew(w)
    s=.5 if a<1e-5 else (1-np.cos(a))/a**2
    t=1./6. if a<1e-5 else (a-np.sin(a))/a**3
    return np.eye(3)+s*h+t*h@h


def exp(x,center):
    w=x[:3]
    r=Rotation.from_rotvec(w).as_matrix()
    t=jacobian(w)@x[3:]
    out=np.eye(4)
    out[:3,:3]=r
    out[:3,3]=center-r@center+t
    return out


def log(transform,center):
    r=transform[:3,:3]
    w=Rotation.from_matrix(r).as_rotvec()
    t=transform[:3,3]+r@center-center
    return np.r_[w,np.linalg.solve(jacobian(w),t)]


def graph_edges(clouds,center):
    reg=o3d.pipelines.registration
    dense=[]
    for points in clouds:
        p=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points)).voxel_down_sample(1.55)
        p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=4.,max_nn=30))
        dense.append(p)
    edges=[]
    n=len(dense)
    for i in range(n):
        for step in (1,2):
            j=(i+step)%n
            result=reg.registration_icp(dense[i],dense[j],2.7,np.eye(4),
                        reg.TransformationEstimationPointToPlane(reg.TukeyLoss(1.8)),
                        reg.ICPConvergenceCriteria(max_iteration=24))
            d=log(result.transformation,center)
            back=reg.evaluate_registration(dense[j],dense[i],2.7,np.linalg.inv(result.transformation))
            overlap=min(result.fitness,back.fitness)
            count=len(result.correspondence_set)
            if overlap>=.13 and count>=30 and np.linalg.norm(d[:3])<=np.deg2rad(4.) and np.linalg.norm(d[3:])<=3.:
                edges.append((i,j,result.transformation,max(.1,overlap)))
    return edges


def synchronize(n,edges,center,prior=.12):
    # Convert radians to equivalent displacement at a 40 mm lever arm.
    scale=np.array([40.,40.,40.,1.,1.,1.])
    if not edges:
        return np.repeat(np.eye(4)[None],n,axis=0),0.,0.
    a=np.zeros((len(edges),n))
    p=[]
    w=[]
    for k,(i,j,e,s) in enumerate(edges):
        a[k,i],a[k,j]=1.,-1.
        p.append(log(e,center)*scale)
        w.append(s)
    a=a[:,1:]
    p,w=np.asarray(p),np.asarray(w)
    fit=np.zeros((n-1,6))
    for _ in range(5):
        error=np.linalg.norm(a@fit-p,axis=1)
        weight=w*np.minimum(1.,.7/np.maximum(error,1e-8))
        fit=np.linalg.solve(a.T@(weight[:,None]*a)+prior*np.eye(n-1),
                            a.T@(weight[:,None]*p))
    delta=np.r_[np.zeros((1,6)),fit/scale]
    max_r=np.max(np.linalg.norm(delta[:,:3],axis=1))
    max_t=np.max(np.linalg.norm(delta[:,3:],axis=1))
    factor=min(1.,np.deg2rad(1.8)/max(max_r,1e-8),1.35/max(max_t,1e-8))
    out=np.array([exp(x,center) for x in delta*factor])
    before=float(np.median(np.linalg.norm(p,axis=1)))
    after=float(np.median([np.linalg.norm(log(out[j]@e@np.linalg.inv(out[i]),center)*scale)
                           for i,j,e,_ in edges]))
    return out,before,after


def checks(n=10):
    rng=np.random.default_rng(12)
    center=np.array([2.,-4.,18.])
    directions=rng.normal(size=(n,6))*np.array([.006]*3+[.4]*3)
    directions[0]=0.
    corrections=np.array([exp(x,center) for x in directions])
    for d,t in zip(directions,corrections):
        np.testing.assert_allclose(log(t,center),d,atol=1e-11)
        np.testing.assert_allclose(np.linalg.det(t[:3,:3]),1.,atol=1e-11)
    edges=[(i,(i+d)%n,np.linalg.inv(corrections[(i+d)%n])@corrections[i],.6)
           for i in range(n) for d in (1,2)]
    r,pre,post=synchronize(n,edges,center,prior=1e-8)
    err=np.array([log(t@np.linalg.inv(o),center) for t,o in zip(r,corrections)])
    worst_shift=np.max(np.linalg.norm(err[:,3:],axis=1))
    worst_angle_deg=np.rad2deg(np.max(np.linalg.norm(err[:,:3],axis=1)))
    np.testing.assert_allclose(r[0],np.eye(4),atol=1e-13)
    assert worst_shift<.025 and worst_angle_deg<.025
    i,j,e,w=edges[1]
    noisy=edges.copy()
    noisy[1]=(i,j,exp(np.array([0.,0.,.028,2.2,0.,0.]),center)@e,w)
    rw,pre_noisy,post_noisy=synchronize(n,noisy,center)
    ee=np.array([log(t@np.linalg.inv(o),center) for t,o in zip(rw,corrections)])
    # This is a stress measurement; no claim that a bounded erroneous edge is always localized.
    loop=np.linalg.inv(r[0])@r[-1]
    for a,b in zip(range(n-2,-1,-1),range(n-1,0,-1)):
        loop=loop@np.linalg.inv(r[b])@r[a]
    np.testing.assert_allclose(loop,np.eye(4),atol=1e-12)
    return dict(toy_clean_worst_shift_mm=worst_shift,toy_clean_worst_angle_deg=worst_angle_deg,
                toy_clean_edge_after=post,toy_clean_edge_before=pre,toy_noise_edge_after=post_noisy,
                toy_noise_edge_before=pre_noisy,toy_noise_worst_shift_mm=np.max(np.linalg.norm(ee[:,3:],axis=1)),
                node_product_identity_norm=np.linalg.norm(loop-np.eye(4)))
