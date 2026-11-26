"""Constrained rectangular face assembly for the older 90-degree paper box."""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree

from optics import transform
from stitch_turntable import axis_transform


def face_line(points, center, basis, seed):
    p=(points-center)@basis
    rng=np.random.default_rng(seed)
    n=len(p)
    test=p[::max(1,n//2500)]
    pairs=rng.integers(0,len(test),(210,2))
    best=None
    for i,j in pairs:
        d=test[j]-test[i]
        length=np.linalg.norm(d)
        if length<8: continue
        normal=np.array([-d[1],d[0]])/length
        dist=abs((test-test[i])@normal)
        hits=(dist<1.35).sum()
        if best is None or hits>best[0]: best=hits,normal,test[i]
    if best is None:
        raise RuntimeError("Too little planar support")
    _,normal,position=best
    mask=abs((p-position)@normal)<1.45
    middle=np.mean(p[mask],axis=0)
    _,_,vs=np.linalg.svd(p[mask]-middle,full_matrices=False)
    normal=vs[-1]
    camera=(-center)@basis
    if normal@(camera-middle)<0: normal=-normal
    side=np.array([-normal[1],normal[0]])
    ds=(p[mask]-middle)@side
    points=np.array([middle+np.quantile(ds,q)*side for q in (.045,.50,.955)])
    inward=middle@normal
    return points,normal,inward,mask


def axset(n,center,angles):
    return np.array([axis_transform(n,center,a) for a in angles])


def estimate(views, n, center, B, nominal):
    if len(views)!=4:
        raise ValueError("Plane assembly expects four nominal 90-degree scans")
    faces=[face_line(x["train"],center,B,33+i) for i,x in enumerate(views)]
    # Fit in coordinates of the calibrated table plane. Transform the complete cloud later.
    world=[center+f[0]@B.T for f in faces]
    normals=[B@f[1] for f in faces]
    base=axset(n,center,nominal)
    p0=np.array([(transform(p,T)-center)@B for T,p in zip(base,world)])
    rotate=lambda a: np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
    first=normals[0]@B
    yaw=np.arctan2(first[1],first[0])
    directions=np.array([[1,0],[0,1],[-1,0],[0,-1]])
    expected=[int(np.argmax(directions@(rotate(yaw).T@((T[:3,:3]@nm)@B)))) for T,nm in zip(base,normals)]
    if len(set(expected))!=4:
        raise RuntimeError(f"Observed faces do not support four distinct directions: {expected}")
    center0=np.mean(p0[:,1],axis=0)
    extent0=np.clip(np.ptp(p0.reshape(-1,2)@rotate(yaw),axis=0)/2,10,85)
    def unpack(x):
        c=center+B@x[:2]
        a=nominal+np.r_[0.,x[7:]]
        return c,x[2:4],x[4:6],yaw+x[6]/30.,a
    def loss(x):
        c,location,extent,y,a=unpack(x)
        rot=rotate(y)
        ps=axset(n,c,a)
        all_res=[]
        for i,T in enumerate(ps):
            uv=((transform(world[i],T)-center)@B-location)@rot
            nt=(T[:3,:3]@normals[i])@B@rot
            kind=expected[i]
            j=kind%2;s=1 if kind<2 else -1
            d=1-j
            ends=np.sort(uv[[0,2],d])
            span=np.linalg.norm(faces[i][0][2]-faces[i][0][0])
            # Match face midline and two observed lateral ends with a small trim allowance.
            all_res.extend(np.r_[uv[:,j]-s*extent[j],3*(nt-directions[kind]),
                         .6*(ends-[-extent[d]+.045*span,extent[d]-.045*span])])
        return np.r_[all_res,x[:2]*.12,x[7:]*.15,x[6]*.15]
    init=np.r_[0.,0.,center0,extent0,0.,0.,0.,0.]
    low=np.r_[-25.,-25.,center0-45.,[8.,8.],-8.,[-13.]*3]
    high=np.r_[25.,25.,center0+45.,[95.,95.],8.,[13.]*3]
    opt=least_squares(loss,init,bounds=(low,high),loss="soft_l1",f_scale=1.4,max_nfev=90)
    c,location,extent,y,a=unpack(opt.x)
    ps=axset(n,c,a)
    # Re-fit each side independently on its training tile, then use only evaluation tiles
    # to measure the distance of the physical adjoining boundaries in the horizontal plane.
    def seam(poses):
        edge_points=[]
        for i,T in enumerate(poses):
            p=(transform(views[i]["val"],T)-center)@B
            f=faces[i]
            world_inlier=abs(((views[i]["val"]-center)@B)@f[1]-f[2])<1.7
            p=p[world_inlier]
            if len(p)<30: return None
            line=(T[:3,:3]@normals[i])@B
            tang=np.array([-line[1],line[0]])
            projections=p@tang
            low,high=np.quantile(projections,[.06,.94])
            edge_points.append([np.median(p[projections<=low],axis=0),np.median(p[projections>=high],axis=0)])
        corner=[]
        for i in range(4):
            j=(i+1)%4
            corner.append(float(np.linalg.norm(np.asarray(edge_points[i])[:,None]-edge_points[j],axis=2).min()))
        return corner
    before,after=seam(base),seam(ps)
    return ps,{"axis_center":c,"angles":a,"half_widths_mm":extent,"reference_yaw_rad":y,"rectangle_center_in_table_coords":location,"side_kinds":expected,
         "training_residual_p50_before_after":[float(np.median(np.abs(loss(init)))) ,float(np.median(np.abs(loss(opt.x))))],
         "heldout_edge_gap_before_after":[before,after],
         "edge_gap_p50_before_after":[float(np.median(z)) if z else None for z in (before,after)],
         "side_train_counts":[int(x[3].sum()) for x in faces]}
