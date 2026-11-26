"""Measured patch diagnostics and intentionally local geometry regularization."""
from itertools import combinations

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def robust_plane(p, axis):
    inlier=np.ones(len(p),bool)
    for _ in range(3):
        center=np.mean(p[inlier],axis=0)
        _,_,v=np.linalg.svd(p[inlier]-center,full_matrices=False)
        n=v[-1]
        if n@axis<0:
            n=-n
        if abs(n@axis)<.77:
            return None
        residual=abs((p-center)@n)
        inlier=residual<=max(.7,np.percentile(residual,76))
    return n,float(center@n),int(inlier.sum())


def rectify(p,theta):
    yaw,x,y,z,a,b,h=theta
    R=Rotation.from_euler("z",yaw).as_matrix()
    return (p-[x,y,z])@R, np.array([a,b,h]),R,np.array([x,y,z])


def box_analysis(train,test,theta):
    A,ext,_,_=rectify(np.concatenate(train),theta)
    B,_,_,_=rectify(np.concatenate(test),theta)
    details=[]
    planes={}
    for axis,sign in ((0,1),(1,1),(0,-1),(1,-1)):
        tang=1-axis
        direction=np.eye(3)[axis]*sign
        def mask(p,margin):
            return (abs(p@direction-ext[axis])<=4.5)&(abs(p[:,tang])<=ext[tang]+margin)&(p[:,2]>.10*ext[2])&(p[:,2]<.91*ext[2])
        narrow=A[mask(A,-max(1.,ext[tang]*.13))]
        if len(narrow)<65:
            continue
        plane=robust_plane(narrow,direction)
        if plane is None:
            continue
        n,b,_=plane
        q=B[mask(B,14.)]
        q=q[abs(q@n-b)<=2.6]
        if len(q)<24:
            continue
        planes[axis,sign]=n
        exceed=np.maximum(abs(q[:,tang])-ext[tang],0.)
        details.append({"face":f"{'XY'[axis]}{'+' if sign>0 else '-'}","held_count":len(q),
            "signed_center_offset":(b-n[2]*ext[2]*.5)/(n@direction)-ext[axis],
            "protrusion_P95":np.percentile(exceed,95),
            "protrusion_over_2":np.mean(exceed>2.),"normal":n,"distance_P95":np.percentile(abs(q@n-b),95)})
    perpendicular=[];parallel=[]
    for (i,j),(k,l) in combinations(planes,2):
        ang=np.degrees(np.arccos(np.clip(abs(planes[i,j]@planes[k,l]),0.,1.)))
        (parallel if i==k else perpendicular).append(float(ang if i==k else abs(90.-ang)))
    sigs=[row["signed_center_offset"] for row in details]
    return {"planes":details,"mean_protrusion_over_2":np.mean([p["protrusion_over_2"] for p in details]) if details else None,
        "max_face_protrusion_P95":max((p["protrusion_P95"] for p in details),default=None),
        "opposite_angle_error":["%.3f"%a for a in parallel],
        "adjacent_angle_error":["%.3f"%a for a in perpendicular],
        "signed_offsets":sigs}


def axis_sensitivity(train,test,theta,poses,basis):
    shifts={"base":np.zeros(3),"+X":np.array([4.,0.,0.]),"-X":np.array([-4.,0.,0.]),
        "+Y":np.array([0.,4.,0.]),"-Y":np.array([0.,-4.,0.])}
    report={}
    for name,d in shifts.items():
        shift=np.array([((np.eye(3)-T[:3,:3])@(basis@d))@basis for T in poses])
        q=[p+s for p,s in zip(test,shift)]
        a=box_analysis(train,q,theta)
        # Common, held-out model and support region for axis stress comparisons.
        D,ext,_,_=rectify(np.concatenate(q),theta)
        side=(D[:,2]>ext[2]*.12)&(D[:,2]<ext[2]*.88)
        offset=abs(np.maximum(abs(D[side,0])-ext[0],abs(D[side,1])-ext[1]))
        report[name]={"max_face_protrusion_P95":a["max_face_protrusion_P95"],"envelope_side_P95":np.percentile(offset,95),
                      "normal_distortion":np.linalg.norm(shift,axis=1).max()}
    return report


def box_mesh(theta):
    yaw,x,y,z,a,b,h=theta
    floor=max(0.,z)
    mesh=o3d.geometry.TriangleMesh.create_box(2*a,2*b,h+z-floor)
    mesh.translate([-a,-b,0.]).rotate(Rotation.from_euler("z",yaw).as_matrix(),center=(0,0,0))
    mesh.translate([x,y,floor])
    return mesh


def charger_planes(train):
    seed=np.concatenate(train)
    model=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(seed)).voxel_down_sample(1.25)
    parts=[]
    center=np.median(seed,axis=0)
    for _ in range(7):
        if len(model.points)<100:
            break
        coeff,inliers=model.segment_plane(distance_threshold=.95,ransac_n=3,num_iterations=220)
        if len(inliers)<58:
            break
        P=np.asarray(model.points)[inliers].copy()
        model=model.select_by_index(inliers,invert=True)
        n=np.array(coeff[:3])
        if n@(np.mean(P,axis=0)-center)<0:
            n=-n
        _,spread,axes=np.linalg.svd(P-P.mean(0),full_matrices=False)
        span=np.ptp(P@axes[:2].T,axis=0)
        if min(span)<13:
            continue
        parts.append((P,n,P.mean(0),span))
    groups=[]
    for (i,(A,u,c,s)),(j,(B,v,d,t)) in combinations(enumerate(parts),2):
        angle=np.degrees(np.arccos(np.clip(-u@v,-1,1)))
        axis=(u-v)/max(np.linalg.norm(u-v),1e-9)
        width=(c-d)@axis
        if angle>21 or width<8 or min((c-center)@axis,(center-d)@axis)<1.8:
            continue
        # Paired plane patches must overlap tangentially enough to describe the same body.
        m=(c-d)-width*axis
        if np.linalg.norm(m)>.65*min(max(s),max(t)):
            continue
        groups.append((min(len(A),len(B)),i,j,axis,float(angle),float(width)))
    if not groups:
        plane_candidates=[{"samples":len(P),"normal":n,"center":c,"span":s} for P,n,c,s in parts]
        return [],{"plane_candidates":plane_candidates,"accepted_pairs":0}
    _,i,j,n,angle,width=max(groups,key=lambda z:z[0])
    if angle>1.0:
        selected=[]
        for k in (i,j):
            P,u,c,_=parts[k]
            _,_,r=np.linalg.svd(P-c,full_matrices=False)
            basis=r[:2].T
            selected.append((u,n if n@u>0 else -n,c,cKDTree((P-c)@basis),basis))
    else:
        selected=[]
    return selected,{"plane_candidates":len(parts),"accepted_pairs":int(bool(selected)),
        "initial_parallel_error_deg":angle,"separation":width,"patch_sizes":[len(parts[i][0]),len(parts[j][0])]}


def patch_adjust(points,normal,planes):
    out=points.copy();N=normal.copy()
    best=np.zeros(len(points))
    for old,target,c,tree,basis in planes:
        rel=points-c
        dist=tree.query(rel@basis)[0]
        thickness=abs(rel@old)
        coherence=abs(normal@old)
        support=np.clip((3.3-thickness)/1.8,0.,1.)*np.clip((3.4-dist)/2.,0.,1.)*np.clip((coherence-.75)/.16,0.,1.)
        shift=np.clip(rel@target,-2.3,2.3)
        good=support>best
        out[good]=points[good]-(shift[good]*support[good])[:,None]*target
        orientation=np.sign(normal@target)
        N[good]=(1-support[good,None])*normal[good]+support[good,None]*orientation[good,None]*target
        best[good]=support[good]
    N/=np.maximum(np.linalg.norm(N,axis=1,keepdims=True),1e-12)
    return out,N,best


def camera_foot_guides(frames,center,basis,cal,annotations):
    """Back-project visible flipper masks onto a near-table band."""
    import cv2
    from scipy import ndimage

    def plane_coords(view,pixels,height):
        T=frames[int(view)]["pose"]
        uv=np.asarray(pixels,dtype=np.float64).reshape(-1,1,2)
        lines=cv2.undistortPoints(uv,cal["cam_int"],cal["cam_dist"])[:,0]
        lines=np.c_[lines,np.ones(len(lines))]@T[:3,:3].T@basis
        origin=(T[:3,3]-center)@basis
        return (origin+lines*((height-origin[2])/lines[:,2])[:,None])[:,:2]

    def raster(polygons,xy):
        return np.logical_or.reduce([np.array([cv2.pointPolygonTest(p.astype("float32"),tuple(a),False)>=0
                                               for a in xy]) for p in polygons])

    polygons={k:[plane_coords(k,s,2.2) for s in v] for k,v in annotations["polygons"].items()}
    refs=polygons[str(annotations["reference_view"])]
    all_ref=np.concatenate(refs)
    origin=np.min(all_ref,axis=0)-4
    pitch=.92
    shape=tuple(np.ceil((np.max(all_ref,axis=0)+4-origin)/pitch).astype(int)+1)
    xy=origin+np.indices(shape).reshape(2,-1).T*pitch
    anchor=raster(refs,xy).reshape(shape)
    alt=raster(polygons[str(annotations["secondary_view"])],xy).reshape(shape)
    test=raster(polygons[str(annotations["heldout_view"])],xy).reshape(shape)
    # Use the less occluded front view as primary and admit minor extensions supported in view 1.
    extent=anchor|(alt&ndimage.binary_dilation(anchor,iterations=1))
    part,number=ndimage.label(extent)
    counts=np.bincount(part.ravel()); counts[0]=0
    keep=counts>=10
    extent=keep[part]
    coords=xy[extent.ravel()]
    edge=extent&~ndimage.binary_erosion(extent)
    ed=xy[edge.ravel()]
    sdf=ndimage.distance_transform_edt(extent)-ndimage.distance_transform_edt(~extent)
    direction=-np.stack(np.gradient(sdf),axis=-1)[edge]
    direction/=np.maximum(np.linalg.norm(direction,axis=1,keepdims=True),1e-5)
    p=np.concatenate([np.c_[coords,np.full(len(coords),.48)],np.c_[coords,np.full(len(coords),3.25)],
                      np.c_[ed,np.full(len(ed),1.7)]])
    n=np.concatenate([np.tile([0.,0.,-1.],(len(coords),1)),np.tile([0.,0.,1.],(len(coords),1)),
                      np.c_[direction,np.zeros(len(ed))]])
    info={"footprint_components":int(keep.sum()),"heldout_visible_area_agreement":float(np.sum(extent&test)/max(1,np.sum(test))),
        "training_area_agreement":float(np.sum(extent&alt)/max(1,np.sum(alt))),"footprint_mm_star2":float(pitch**2*len(coords)),
        "heldout_view":annotations["heldout_view"],"inferred_height_range":[.48,3.25]}
    return p,n,info,(xy,test.ravel())


def smooth_voxel_surface(inside,origin,pitch):
    """Linearly interpolate a smoothed occupancy field on a shared tetrahedral grid."""
    from scipy.ndimage import gaussian_filter

    field=gaussian_filter(np.pad(inside,2).astype(float),.68)-.48
    inside=field>0
    corner=np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]])
    signs=np.stack([inside[a:a+inside.shape[0]-1,b:b+inside.shape[1]-1,c:c+inside.shape[2]-1]
                    for a,b,c in corner],axis=-1)
    loc=np.argwhere(signs.any(-1)&~signs.all(-1))
    if not len(loc):
        return o3d.geometry.TriangleMesh()
    signs=signs[tuple(loc.T)]
    tetra=((0,5,1,6),(0,1,2,6),(0,2,3,6),(0,3,7,6),(0,7,4,6),(0,4,5,6))
    edges=[];dims=field.shape
    for tet in tetra:
        state=signs[:,tet].astype(np.int32)@np.array([1,2,4,8])
        xyz=corner[list(tet)]
        for k in range(1,15):
            bases=loc[state==k]
            if not len(bases):
                continue
            pos=[i for i in range(4) if k&(1<<i)]
            neg=[i for i in range(4) if not k&(1<<i)]
            if len(pos)==1 or len(neg)==1:
                pair=[(a,b) for a in pos for b in neg]
            else:
                a,b=pos;c,d=neg
                pair=[(a,c),(a,d),(b,d),(b,c)]
            P=np.array(pair)
            mid=xyz[P].mean(1)
            outward=xyz[neg].mean(0)-xyz[pos].mean(0)
            if np.cross(mid[1]-mid[0],mid[2]-mid[0])@outward<0:
                P=P[::-1]
            for tri in ([[0,1,2],[0,2,3]] if len(P)==4 else [[0,1,2]]):
                coords=bases[:,None,None,:]+xyz[P[tri]][None]
                ids=np.ravel_multi_index(np.moveaxis(coords,-1,0),dims)
                edges.append(np.sort(ids,axis=2))
    pair,face=np.unique(np.concatenate(edges).reshape(-1,2),axis=0,return_inverse=True)
    a,b=field.ravel()[pair].T
    frac=np.clip(a/(a-b),0.,1.)
    l,r=(np.array(np.unravel_index(c,dims)).T for c in pair.T)
    p=origin+(l+(r-l)*frac[:,None]-1.5)*pitch
    return o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(p),
                                    o3d.utility.Vector3iVector(face.reshape(-1,3)))
