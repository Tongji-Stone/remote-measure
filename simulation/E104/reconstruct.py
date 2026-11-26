"""Surface reconstruction and measured-ray operations for E104."""
import cv2 as cv
import numpy as np
import open3d as o3d
from scipy import ndimage as ndi
from scipy.optimize import least_squares
from scipy.spatial import ConvexHull, HalfspaceIntersection, cKDTree
from pathlib import Path

HERE = Path(__file__).resolve().parent
PITCH=1.25
BIN=7
FLOOR_EPS=1e-6
METHODS={
    'S01':'Screened Poisson',
    'S02':'Ball Pivoting + 限径补孔',
    'S03':'贴盘凸包',
    'S04':'接触引导 Poisson + 实测射线包络',
    'S05':'局部几何先验',
    'S06':'项目式 TSDF',
    'S07':'局部沟槽延拓 + 封闭包络',
}


def tx(p,T):
    return np.asarray(p) @ T[:3,:3].T + T[:3,3]


def transform(R,t):
    T=np.eye(4)
    T[:3,:3],T[:3,3]=R,t
    return T


def cloud(p,n=None):
    c=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p))
    if n is not None:
        c.normals=o3d.utility.Vector3dVector(n)
    return c


def oriented(frames):
    points,normals=[],[]
    for f in frames:
        p=f['p']
        if len(p)<35:
            continue
        c=cloud(p).voxel_down_sample(.95)
        c.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=3.6,max_nn=36))
        c.orient_normals_towards_camera_location(f['camera'])
        points.append(np.asarray(c.points).copy())
        normals.append(np.asarray(c.normals).copy())
    p,n=np.vstack(points),np.vstack(normals)
    keep=cKDTree(p).query(p,k=5)[0][:,-1]<=4.
    c=cloud(p[keep],n[keep]).voxel_down_sample(.83).normalize_normals()
    p,n=np.asarray(c.points).copy(),np.asarray(c.normals).copy()
    ok=np.linalg.norm(n,axis=1)>.9
    return p[ok],n[ok]


def mesh(v,f):
    return o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(np.asarray(v,float)),
                                     o3d.utility.Vector3iVector(np.asarray(f,int)))


def clean(m,min_component=30):
    m.remove_duplicated_vertices().remove_degenerate_triangles().remove_duplicated_triangles()
    if not len(m.triangles):
        return m
    labels,n,area=m.cluster_connected_triangles()
    keep=(np.asarray(n)>=min_component)&(np.asarray(area)>=1.)
    keep[np.argmax(area)]=True
    m.remove_triangles_by_mask(~keep[np.asarray(labels)])
    m.remove_unreferenced_vertices()
    return m


def screened(p,n):
    m,d=o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(cloud(p,n),depth=7,scale=1.13,n_threads=1)
    # Keep a dense-supported visible mesh separately. For fair extrapolation the raw method is also saved.
    visible=o3d.geometry.TriangleMesh(m)
    density=np.asarray(d)
    q=np.asarray(visible.vertices)
    weak=(density<np.quantile(density,.04))|(cKDTree(p).query(q)[0]>3.5)
    visible.remove_vertices_by_mask(weak)
    return clean(m),clean(visible)


def hull_mesh(p):
    hull=ConvexHull(p)
    tri=hull.simplices.copy()
    a,b,c=np.moveaxis(p[tri],1,0)
    flip=np.sum(np.cross(b-a,c-a)*hull.equations[:,:3],axis=1)<0
    tri[flip,1:]=tri[flip,1:][:,::-1]
    m=mesh(p,tri)
    m.remove_unreferenced_vertices()
    return m


def bpa(p,n):
    m=o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(cloud(p,n),
                    o3d.utility.DoubleVector([1.25,2.4,3.6]))
    m=clean(m)
    if len(m.triangles):
        closed=o3d.t.geometry.TriangleMesh.from_legacy(m).fill_holes(hole_size=4.)
        return clean(closed.to_legacy())
    return m


def scene(m):
    sc=o3d.t.geometry.RaycastingScene(nthreads=1)
    sc.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(m))
    return sc


def pixels(frame,p,cal):
    q=tx(p,frame['to_camera'])
    u=cv.projectPoints(q,np.zeros(3),np.zeros(3),cal['cam_int'],cal['cam_dist'])[0][:,0]
    uv=np.rint(np.clip(u/BIN,-20000,20000)).astype(int)
    return uv,q[:,2]


def depth_map(frame,cal):
    wh=np.ceil(np.array(cal['shape'])[::-1]/BIN).astype(int)+1
    uv,z=pixels(frame,frame['p'],cal)
    ok=((uv>=0)&(uv<wh)).all(1)&(z>0)
    flat=np.full(wh.prod(),np.inf)
    np.minimum.at(flat,uv[ok,1]*wh[0]+uv[ok,0],z[ok])
    return flat,wh


def rays(grid,frames,cal,mu=4.5):
    free=np.zeros(len(grid),np.uint8)
    total=np.zeros(len(grid))
    count=np.zeros(len(grid),np.uint8)
    for f in frames:
        depth,wh=depth_map(f,cal)
        uv,z=pixels(f,grid,cal)
        ids=np.flatnonzero(((uv>=0)&(uv<wh)).all(1)&(z>0))
        d=depth[uv[ids,1]*wh[0]+uv[ids,0]]-z[ids]
        free[ids]+=np.isfinite(d)&(d>3.)
        good=np.isfinite(d)&(d>=-mu)
        take=ids[good]
        total[take]+=np.clip(d[good]/mu,-1.,1.)
        count[take]+=1
    avg=np.divide(total,count,out=np.ones_like(total),where=count>0)
    return free,avg,count


def grid_box(p,pitch=PITCH):
    lo=np.floor((np.percentile(p,.1,axis=0)-5.)/pitch)*pitch
    hi=np.percentile(p,99.9,axis=0)+5.
    lo[2]=min(-2*pitch,lo[2])
    size=tuple(np.ceil((hi-lo)/pitch).astype(int)+1)
    return lo,size,lo+np.indices(size).reshape(3,-1).T*pitch


def footprint(p,pitch=1.1):
    """Continue local bottom contours to the board datum, with separate low components."""
    high=np.percentile(p[:,2],95)
    percentile=np.percentile(p[:,2],6)
    limit=max(4.,min(percentile+5.,max(9.,.31*high)))
    low=p[(p[:,2]>-.9)&(p[:,2]<limit)]
    if len(low)<60:
        low=p[p[:,2]<=np.percentile(p[:,2],25.)]
    xy0=low[:,:2].min(0)-5.
    size=np.ceil((low[:,:2].max(0)+5.-xy0)/pitch).astype(int)+1
    xy=xy0+np.indices(tuple(size)).reshape(2,-1).T*pitch
    dist,idx=cKDTree(low[:,:2]).query(xy,k=min(5,len(low)))
    keep=(dist[:,0]<2.6)&(dist[:,-1]<4.)
    mask=keep.reshape(tuple(size))
    mask=ndi.binary_fill_holes(ndi.binary_closing(mask,iterations=1))
    labels,n=ndi.label(mask)
    vals=np.bincount(labels.ravel());vals[0]=0
    mask=vals[labels]>=max(8,vals.max()*.008)
    xy_bottom=xy[mask.ravel()]
    edge=mask&~ndi.binary_erosion(mask)
    xy_edge=xy[edge.ravel()]
    height=np.maximum(.6,np.mean(low[idx[edge.ravel()],2],axis=1))
    D=ndi.distance_transform_edt(mask)-ndi.distance_transform_edt(~mask)
    grad=-np.stack(np.gradient(D),axis=-1)[edge]
    grad/=np.maximum(np.linalg.norm(grad,axis=1,keepdims=True),1e-6)
    return np.vstack([np.c_[xy_bottom,np.zeros(len(xy_bottom))],np.c_[xy_edge,.58*height]]),np.vstack([
        np.tile([0.,0.,-1.],(len(xy_bottom),1)),np.c_[grad,np.full(len(xy_edge),-.42)]])


def envelope(poisson,p,guide,lo,shape,grid,free,pitch=PITCH,*,exposed=()):
    sc=scene(poisson)
    # The envelope is applied after Poisson with 2-out-of-N measured free-space evidence.
    occ=np.zeros(len(grid),bool)
    gate=(grid[:,2]>=0.)&(free<2)
    h=ConvexHull(np.r_[p[::4],guide])
    for part in np.array_split(h.equations,max(1,len(h.equations)//22+1)):
        k=np.flatnonzero(gate)
        gate[k]&=(grid[k]@part[:,:3].T+part[:,3]<=2.5).all(1)
    ids=np.flatnonzero(gate)
    for members in np.array_split(ids,max(1,len(ids)//75000+1)):
        if len(members):
            q=o3d.core.Tensor(grid[members].astype('float32'))
            occ[members]=sc.compute_occupancy(q,nthreads=1,nsamples=3).numpy()>.5
    a=occ.reshape(shape)
    a=(a|ndi.binary_closing(a,iterations=1))&gate.reshape(shape)
    ground=int(round(-lo[2]/pitch))
    at_floor=np.arange(shape[2])*pitch+lo[2]
    if ground<0 or ground>=shape[2] or not np.isclose(at_floor[ground],0.):
        raise ValueError('The reconstruction lattice must include the calibrated board plane.')
    allowed=gate.reshape(shape)[:,:,ground:]
    above=a[:,:,ground:]
    empty=np.zeros(above.shape[:2],bool)
    if exposed:
        from deck import contact_carve
        xy=grid.reshape(*shape,3)[:,:,ground,:2].reshape(-1,2)
        empty=contact_carve(xy,exposed,p).reshape(empty.shape)
        allowed[:,:,0]&=~empty
        above[:,:,0]&=~empty
    a=extend_contact(above,allowed,pitch)
    # Copy the basal slice below z=0 for filtering; padding with air would round it off.
    a=np.pad(a,((2,2),(2,2),(0,2)))
    a=np.concatenate([np.repeat(a[:,:,:1],2,axis=2),a],axis=2)
    origin=np.array([lo[0]-2*pitch,lo[1]-2*pitch,0.])
    candidate=None
    for sigma in (.55,.75,.95):
        field=.47-ndi.gaussian_filter(a.astype(float),sigma)[:,:,2:]
        forbidden=np.pad(empty,2)
        field[:,:,0][forbidden]=np.maximum(field[:,:,0][forbidden],.09)
        # Avoid near-zero crossings that yield coincident small faces after export.
        field=np.copysign(np.maximum(abs(field),.035),field)
        candidate=projective_surface(field.ravel(),np.ones(field.size,bool),origin,field.shape,pitch,
                                     cap_floor=True)
        if len(candidate.triangles) and candidate.is_watertight():
            return candidate
    return repair_contact_mesh(candidate)


def repair_contact_mesh(original):
    """Remove local numerical intersections while pinning all vertices at z=0."""
    if not len(original.triangles):
        return original
    if not original.is_edge_manifold(False) or not original.is_vertex_manifold():
        return original
    m=o3d.geometry.TriangleMesh(original)
    t=np.asarray(m.triangles)
    p=np.asarray(m.vertices)
    adjacency=np.unique(np.concatenate((t[:,[0,1]],t[:,[1,2]],t[:,[2,0]],
                                        t[:,[1,0]],t[:,[2,1]],t[:,[0,2]])),axis=0)
    for _ in range(3):
        bad=np.asarray(m.get_self_intersecting_triangles())
        if not len(bad):
            return m
        affected=np.unique(t[bad])
        affected=affected[p[affected,2]>FLOOR_EPS]
        if not len(affected):
            break
        edge=adjacency[np.isin(adjacency[:,0],affected)]
        total=np.zeros_like(p)
        np.add.at(total,edge[:,0],p[edge[:,1]])
        count=np.bincount(edge[:,0],minlength=len(p))
        delta=(total[affected]/count[affected,None]-p[affected])*.26
        ratio=np.maximum(1.,np.linalg.norm(delta,axis=1)/.09)
        p[affected]+=delta/ratio[:,None]
    if m.is_watertight():
        return m
    # Rare export-precision intersections may survive local relaxation.
    if len(original.triangles)>24000:
        for target in (24000,12000):
            simplified=original.simplify_quadric_decimation(target)
            verts=np.asarray(simplified.vertices)
            verts[abs(verts[:,2])<=FLOOR_EPS,2]=0.
            if simplified.is_watertight() and ground_contact(simplified)['touches_floor']:
                return simplified
    return original


def extend_contact(inside,allowed,pitch):
    """Continue only supported low cross-sections to z=0 in the ray-limited envelope."""
    layers=min(inside.shape[2],1+int(round(3.75/pitch)))
    low=inside[:,:,:layers]
    hits=np.count_nonzero(low,axis=2)
    first=np.argmax(low,axis=2)
    proposals=(hits>=2)
    z=np.arange(layers)[None,None,:]
    added=proposals[:,:,None] & (z<first[:,:,None])
    result=inside.copy()
    result[:,:,:layers]|=added&allowed[:,:,:layers]
    labs,_=ndi.label(result)
    sizes=np.bincount(labs.ravel())
    contact=np.bincount(labs[:,:,0].ravel(),minlength=len(sizes))
    keep=(sizes>=8)&(contact>0)
    keep[0]=False
    return keep[labs]


def ground_contact(m):
    p=np.asarray(m.vertices)
    a=p[np.asarray(m.triangles)]
    on=np.max(abs(a[:,:,2]),axis=1)<=FLOOR_EPS
    q=a[on]
    area=np.linalg.norm(np.cross(q[:,1]-q[:,0],q[:,2]-q[:,0]),axis=1).sum()/2
    low=p[:,2].min()
    return dict(min_z_mm=float(low),floor_cap_area_mm2=float(area),
                floor_cap_triangles=int(on.sum()),touches_floor=bool(abs(low)<=FLOOR_EPS and area>=1.))


def floor_cap_keys(field,covered,shape):
    """Clip the z=0 lattice triangles; use the same edge IDs as the upper surface."""
    x,y=np.indices(np.array(shape[:2])-1)
    corners=np.stack([np.c_[x.ravel()+a,y.ravel()+b,np.zeros(x.size,dtype=int)]
                      for a,b in ((0,0),(1,0),(1,1),(0,1))],axis=1)
    ids=np.ravel_multi_index(np.moveaxis(corners,-1,0),shape)
    values=field.ravel()
    caps=[]
    for triangle in ((0,2,1),(0,3,2)):
        indices=ids[:,triangle]
        valid=covered.ravel()[indices].all(1)
        state=(values[indices]>0).astype(int)@np.array([1,2,4])
        for code in range(1,8):
            q=indices[valid&(state==code)]
            if not len(q):
                continue
            keys=[]
            for i in range(3):
                j=(i+1)%3
                inside=bool(code&(1<<i))
                other=bool(code&(1<<j))
                if inside:
                    keys.append(np.c_[q[:,i],q[:,i]])
                if inside!=other:
                    keys.append(np.sort(q[:,[i,j]],axis=1))
            polygon=np.stack(keys,axis=1)
            for j in range(1,len(keys)-1):
                caps.append(polygon[:,[0,j,j+1]])
    return caps


def floor_hull(p,guide):
    support=guide[abs(guide[:,2])<=FLOOR_EPS]
    return hull_mesh(np.r_[p[p[:,2]>=0],support]) if len(support)>=3 else None


def projective_surface(val,count,lo,shape,pitch=PITCH,*,cap_floor=False):
    """March tetrahedra and optionally triangulate the exact calibrated planar base."""
    field=-val.reshape(shape)
    covered=count.reshape(shape)>0
    if cap_floor and abs(lo[2])>FLOOR_EPS:
        raise ValueError('A capped surface must start exactly on the calibrated board plane.')
    offsets=np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]])
    size=np.array(shape)-1
    def slices(a):
        return tuple(slice(x,x+y) for x,y in zip(a,size))
    valid=np.logical_and.reduce([covered[slices(a)] for a in offsets])
    signs=np.stack([field[slices(a)]>0 for a in offsets],axis=-1)
    loc=np.argwhere(valid & signs.any(-1) & ~signs.all(-1))
    if not len(loc):
        return o3d.geometry.TriangleMesh()
    signs=signs[tuple(loc.T)]
    tetra=((0,5,1,6),(0,1,2,6),(0,2,3,6),(0,3,7,6),(0,7,4,6),(0,4,5,6))
    edges=[]
    for tet in tetra:
        state=signs[:,tet].astype(int)@np.array([1,2,4,8])
        xyz=offsets[list(tet)]
        for code in range(1,15):
            bases=loc[state==code]
            if not len(bases):
                continue
            pos=[i for i in range(4) if code&(1<<i)]
            neg=[i for i in range(4) if not code&(1<<i)]
            if len(pos)==2:
                a,b=pos;c,d=neg
                pair=[[a,c],[a,d],[b,d],[b,c]]
            else:
                pair=[[a,b] for a in pos for b in neg]
            pair=np.array(pair)
            middle=xyz[pair].mean(1)
            outward=xyz[neg].mean(0)-xyz[pos].mean(0)
            if np.cross(middle[1]-middle[0],middle[2]-middle[0])@outward<0:
                pair=pair[::-1]
            for tri in ([[0,1,2],[0,2,3]] if len(pair)==4 else [[0,1,2]]):
                coords=bases[:,None,None,:]+xyz[pair[tri]][None]
                ids=np.ravel_multi_index(np.moveaxis(coords,-1,0),shape)
                edges.append(np.sort(ids,axis=2))
    if cap_floor:
        edges.extend(floor_cap_keys(field,covered,shape))
    keys,faces=np.unique(np.concatenate(edges).reshape(-1,2),axis=0,return_inverse=True)
    values=field.ravel()[keys]
    weight=np.divide(values[:,0],values[:,0]-values[:,1],out=np.zeros(len(keys)),
                     where=keys[:,0]!=keys[:,1])
    a,b=[np.array(np.unravel_index(k,shape)).T for k in keys.T]
    points=lo+(a+(b-a)*weight[:,None])*pitch
    return clean(mesh(points,faces.reshape(-1,3)),10)


def fit_planes(p,n):
    candidates=p[p[:,2]>1.1]
    equations=[]
    for _ in range(3):
        if len(candidates)<100:
            return None
        c=cloud(candidates)
        coeff,idx=c.segment_plane(1.15,3,700,probability=.997)
        if len(idx)<max(60,.042*len(p)):
            return None
        patch=candidates[idx]
        center=patch.mean(0)
        _,_,V=np.linalg.svd(patch-center,full_matrices=False)
        normal=V[-1]
        if (np.median(p,axis=0)-center)@normal>0:
            normal=-normal
        equations.append(np.r_[normal,-center@normal])
        candidates=np.delete(candidates,idx,axis=0)
    eq=np.array([*equations,[0.,0.,-1.,0.]])
    try:
        from scipy.optimize import linprog
        x=linprog([0,0,0,-1],A_ub=np.c_[eq[:,:3],np.ones(len(eq))],
                  b_ub=-eq[:,3],bounds=[(None,None)]*3+[(0,None)],method='highs')
        if not x.success or x.x[3]<=1.:
            return None
        q=HalfspaceIntersection(eq,x.x[:3]).intersections
        if len(q)!=4 or np.max(cKDTree(p).query(q)[0])>16.:
            return None
        q[abs(q[:,2])<=FLOOR_EPS,2]=0.
        return hull_mesh(q)
    except (ValueError,ArithmeticError):
        return None


def cylinder(p):
    h=np.percentile(p[:,2],99.5)
    mid=p[(p[:,2]>.18*h)&(p[:,2]<.78*h)]
    x0=np.median(mid,axis=0)
    rad=np.median(np.linalg.norm(mid[:,:2]-x0[:2],axis=1))
    opt=least_squares(lambda a:np.linalg.norm(mid[:,:2]-a[:2],axis=1)-a[2],
                      [*x0[:2],rad],loss='soft_l1',f_scale=.9)
    center=opt.x[:2]
    theta=np.linspace(0,2*np.pi,96,endpoint=False)
    z=np.r_[0.,np.arange(1.,max(2,h-1),1.9),h]
    r=np.linalg.norm(p[:,:2]-center,axis=1)
    radii=np.array([np.median(r[(abs(p[:,2]-s)<2.4)&(r>=opt.x[2]*.72)])
                    if np.count_nonzero((abs(p[:,2]-s)<2.4)&(r>=opt.x[2]*.72))>=15 else np.nan for s in z])
    valid=np.isfinite(radii)
    if valid.sum()<6:
        return None
    radii=ndi.median_filter(np.interp(z,z[valid],radii[valid]),size=3,mode='nearest')
    points=np.vstack([np.c_[center+R*np.c_[np.cos(theta),np.sin(theta)],np.full(len(theta),s)] for s,R in zip(z,radii)])
    points=np.r_[points,[[*center,z[0]],[*center,z[-1]]]]
    faces=[]
    for j in range(len(z)-1):
        for i in range(len(theta)):
            a,b=j*len(theta)+i,j*len(theta)+(i+1)%len(theta)
            faces.extend([[a,b,b+len(theta)],[a,b+len(theta),a+len(theta)]])
    for i in range(len(theta)):
        j=(i+1)%len(theta); k=(len(z)-1)*len(theta)
        faces.extend([[len(points)-2,j,i],[len(points)-1,k+i,k+j]])
    return mesh(points,faces)


def polar_guides(p):
    """Continue the lower radial contours without forcing a global convex primitive."""
    c=np.median(p[(p[:,2]>=np.percentile(p[:,2],35))&(p[:,2]<=np.percentile(p[:,2],70)),:2],axis=0)
    a=np.arctan2(p[:,1]-c[1],p[:,0]-c[0])
    r=np.linalg.norm(p[:,:2]-c,axis=1)
    guide=[];N=[]
    for t in np.linspace(-np.pi,np.pi,80,endpoint=False):
        angle=abs(np.angle(np.exp(1j*(a-t))))
        k=(angle<.18)&(p[:,2]<np.percentile(p[:,2],32))&(r>1.)
        if k.sum()<10:
            continue
        local=p[k,2]
        height=np.clip(np.percentile(local,15),2.2,13.)
        take=k&(p[:,2]<np.percentile(local,40))
        rho=np.percentile(r[take],62)
        u=np.array([np.cos(t),np.sin(t)])
        for s in (0.,.30,.61,.83):
            rad=rho*(.70+.30*np.sin(s*np.pi/2))
            guide.append(np.r_[c+rad*u,height*s])
            N.append(np.r_[u*s,-(1.-.73*s)])
        for f in (.09,.27,.49,.69):
            guide.append(np.r_[c+rho*f*u,0.])
            N.append([0,0,-1.])
    if len(guide)<28:
        return np.empty((0,3)),np.empty((0,3))
    add=np.array(N);add/=np.linalg.norm(add,axis=1,keepdims=True)
    return np.array(guide),add
