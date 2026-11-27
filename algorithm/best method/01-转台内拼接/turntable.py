"""E104 R02 shared-axis calibration and gated R06c pose-graph refinement."""
from time import perf_counter

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

import registration_metrics as bmk
from camera_geometry import axial, masks_for_tags, project, transform


def unit(x):
    return x/np.linalg.norm(x)


def fold(uv):
    uv=np.asarray(uv,dtype=int)
    return (uv[:,0]//24+2*(uv[:,1]//24))%10


def axis_prior(observations, supplied, model):
    """Fit a shared axis; interpolate bounded residual angles where the marker board is occluded."""
    cmd=np.asarray(supplied,float)
    if len(cmd)<4 or len(cmd)!=len(observations) or not np.isfinite(cmd).all():
        raise ValueError('Expected at least four scans with finite commanded angles.')
    cmd=cmd-cmd[0]
    if np.any(np.diff(cmd)<=0.) or cmd[-1]>=2*np.pi:
        raise ValueError('Angles must increase within one revolution. Remove a repeated 360-degree capture.')
    usable=np.flatnonzero([q is not None for q in observations])
    if len(usable)<4:
        raise ValueError('At least four well-distributed marker poses are required to estimate the axis.')
    observed_angles=cmd[usable]
    gaps=np.diff(np.r_[observed_angles,observed_angles[0]+2*np.pi])
    if max(gaps)>=np.deg2rad(190):
        raise ValueError('Valid marker poses leave an angular gap of at least 190 degrees.')
    fits=[observations[i] for i in usable]
    A=np.array([v['pose'] for v in fits])
    nominal=observed_angles-observed_angles[0]
    nominal_all=cmd-observed_angles[0]
    relative=A@np.linalg.inv(A[0])
    rotvec=Rotation.from_matrix(relative[:,:3,:3]).as_rotvec()
    _,s,V=np.linalg.svd(rotvec)
    if s[0]<.2:
        raise ValueError('Board poses provide insufficient rotation about the turntable.')
    axis=V[0]
    def deltas(axis):
        angle=rotvec@axis
        return np.angle(np.exp(1j*(angle-nominal)))
    if np.median(abs(deltas(axis)))>np.median(abs(deltas(-axis))):
        axis=-axis
    residual0=deltas(axis)
    B=np.linalg.svd(axis[None,:])[2][1:].T
    M=(np.eye(3)-relative[:,:3,:3]).reshape(-1,3)
    xcenter=np.linalg.lstsq(M@B,relative[:,:3,3].ravel(),rcond=None)[0]
    plate=transform(np.array([[50.,50.,0.]]),A[0])[0]
    center=B@xcenter+axis*(axis@plate)
    if not np.isfinite(center).all() or np.linalg.norm(center-plate)>160:
        raise ValueError('The fitted axis does not pass near the target plate.')

    def unpack(theta):
        u=unit(axis+B@theta[:2])
        c=center+B@theta[2:4]
        angles=nominal+np.r_[0.,theta[4:]]
        return u,c,angles

    def err(theta):
        u,c,a=unpack(theta)
        poses=axial(u,c,a)@A[0]
        out=[]
        for q,T in zip(fits,poses):
            e=project(transform(q['board_xyz'],T),model)-q['board_uv']
            out.append((e/np.sqrt(len(e))).ravel())
        return np.r_[np.concatenate(out),theta[4:]*.5]

    bound=np.r_[.12,.12,16.,16.,np.full(len(usable)-1,np.deg2rad(9.))]
    x0=np.r_[np.zeros(4),np.clip(residual0[1:],-bound[4:]*.95,bound[4:]*.95)]
    opt=least_squares(err,x0,bounds=(-bound,bound),loss='soft_l1',f_scale=.22,max_nfev=50)
    u,c,a=unpack(opt.x)
    d=np.interp(cmd,observed_angles,a-nominal,period=2*np.pi)
    theta=nominal_all+d
    if np.any(np.diff(theta)<=0):
        raise ValueError('Estimated angles changed the acquisition order.')
    P=axial(u,c,-(theta-theta[0]))
    A0=axial(u,c,theta[0])@A[0]
    return P,u,c,A0,dict(marker_stations=usable,estimated_angles_deg=np.rad2deg(theta-theta[0]),
                         axis=u,center_mm=c,fit_success=bool(opt.success),
                         training_marker_rms_weighted_px=np.sqrt(np.mean(err(opt.x)**2)))


def relative_error(edge,base,a,b,center):
    delta=edge@np.linalg.inv(bmk.rel(base,a,b))
    # delta lives in target camera b; take the world-axis pivot into that frame.
    pivot=transform(center[None],np.linalg.inv(base[b]))[0]
    shift=np.linalg.norm(transform(pivot[None],delta)[0]-pivot)
    degree=np.rad2deg(Rotation.from_matrix(delta[:3,:3]).magnitude())
    return degree,shift


def fit_graph(v,base,edges,center):
    equations=[]
    audit=[]
    for (i,j),(T,overlap) in edges.items():
        A,B=v[i]['train'][::3],v[j]['train'][::3]
        moved=transform(A,T)
        d,k=cKDTree(B).query(moved);_,l=cKDTree(moved).query(B)
        dot=np.abs(np.sum((v[i]['train_normals'][::3]@T[:3,:3].T)*
                          v[j]['train_normals'][::3][k],axis=1))
        keep=(d<3.) & (l[k]==np.arange(len(A))) & (dot>=np.cos(np.deg2rad(30.)))
        degree,shift=relative_error(T,base,i,j,center)
        accepted=bool(keep.sum()>=15 and overlap>=.12 and degree<=8. and shift<=9.)
        audit.append(dict(source=i,target=j,accepted=accepted,mutual_count=int(keep.sum()),
                          edge_overlap=overlap,delta_deg=degree,pivot_delta_mm=shift))
        if accepted:
            equations.append((overlap,i,j,A[keep][::2],B[k[keep]][::2]))
    equations.sort(key=lambda e:-e[0])
    def matrices(x):
        D=np.broadcast_to(np.eye(4),base.shape).copy()
        v=x.reshape(-1,6)
        D[1:,:3,:3]=Rotation.from_rotvec(v[:,:3]/65.).as_matrix()
        D[1:,:3,3]=center-D[1:,:3,:3]@center+v[:,3:]
        return D@base
    x=np.zeros(6*(len(base)-1))
    for n in sorted(set((min(2,len(equations)),min(5,len(equations)),len(equations)))-{0}):
        def residual(z):
            M=matrices(z)
            out=[1.7*z]
            for w,i,j,p,q in equations[:n]:
                out.append(((transform(p,M[i])-transform(q,M[j]))*np.sqrt(3*w/len(p))).ravel())
            return np.concatenate(out)
        x=least_squares(residual,x,bounds=(-1.4,1.4),loss='soft_l1',f_scale=.5,max_nfev=17).x
    return matrices(x),audit


class InsufficientSupport(ValueError):
    """The anchor can be used even when an independent geometric check is too sparse."""


def sample(frame,which,noise=None):
    xy=frame['uv']
    xyz=np.asarray(frame['xyz'],float)
    f=fold(xy)
    train=np.flatnonzero((f!=0)&(f!=5))
    val=np.flatnonzero(f==which)
    if min(len(train),len(val))<30:
        raise InsufficientSupport('Insufficient points in a training or holdout partition.')
    rng=np.random.default_rng(104)
    t=rng.permutation(train)[:8500]
    q=rng.permutation(val)[:1600]
    d=cKDTree(xyz[t]).query(xyz[t],k=12)[0][:,-1]
    mad=np.median(abs(d-np.median(d)))
    keep=d<=max(2.8,np.median(d)+4*max(.16,mad))
    t=t[keep]
    train=xyz[t].copy()
    if noise is not None:
        rng=np.random.default_rng(noise)
        train+=rng.normal(0,.64,train.shape)
        contam=rng.choice(len(train),int(.10*len(train)),replace=False)
        train[contam]+=rng.normal(0,9.,(len(contam),3))
    eval_points=xyz[q].copy()
    return dict(train=train,val=eval_points,uv=xy[t],val_uv=xy[q],gray=np.asarray(frame['gray'])[t]/255.,
                train_normals=np.asarray(bmk.cloud(train,4.5).normals),
                normals=np.asarray(bmk.cloud(eval_points,4.5).normals))


def raw_fold(frames,tags,anchor,which):
    raw=[]
    for frame,v in zip(frames,tags):
        match=v['ids']%8==which if v is not None else np.array([],bool)
        obj=v['xyz'][match].reshape(-1,3) if v is not None else np.empty((0,3))
        uv=v['u'][match].reshape(-1,2) if v is not None else np.empty((0,2))
        raw.append(dict(shape=frame['shape'],held_xyz=obj,held_uv=uv,pose=anchor))
    return raw


def contact(frames,pose,base,up,which):
    dz=[]
    for F,T,U in zip(frames,pose,base):
        if 'height' not in F:
            continue
        h=F['height']
        keep=(h<=5)&(h>=-.85)&(fold(F['uv'])==which)
        p=F['xyz'][keep][::max(1,int(keep.sum()/360))]
        dz.extend((transform(p,T)-transform(p,U))@up)
    return np.percentile(abs(np.asarray(dz)),95) if len(dz) else None


def select(base,trial,admitted,contact_error):
    if (admitted<2 or base['tag_P50'] is None or trial['tag_P50'] is None or
        base['held_marker_corners']<12 or trial['depth_abs_P50'] is None or base['depth_abs_P50'] is None):
        return False
    return bool(trial['max_gap']<=base['max_gap']+2.5 and trial['tag_P50']<=base['tag_P50']+1. and
                trial['F1_2']>=base['F1_2']-.015 and trial['depth_supported']>=base['depth_supported']-.025 and
                trial['depth_abs_P50']<.97*base['depth_abs_P50'] and
                (contact_error is None or contact_error<=1.5))


def stitch(frames, tags, cmd, model, *, noise=False, markers_hidden=(), bad_edge=False):
    """Return original-frame-to-V00 transforms and separate policy/evaluation evidence."""
    if not(len(frames)==len(tags)==len(cmd)):
        raise ValueError('Frames, tag observations and angles must have identical lengths.')
    t0=perf_counter()
    observations=[masks_for_tags(t,model) if i not in markers_hidden else None for i,t in enumerate(tags)]
    prior,axis,center,anchor,cal=axis_prior(observations,cmd,model)
    t1=perf_counter()
    # Masking a station also removes its independent tag evidence from the production-time decision.
    tune_tags=[t if i not in markers_hidden else None for i,t in enumerate(tags)]
    try:
        a=[sample(f,0,1040+i if noise else None) for i,f in enumerate(frames)]
        h=[sample(f,5,1040+i if noise else None) for i,f in enumerate(frames)]
    except InsufficientSupport as exc:
        return dict(selected='R02',poses=prior,R02=prior,R06c=prior,axis=cal,edges=[],
                    tune=None,test=None,warning=str(exc),contact_check_mm=None,contact_test_mm=None,
                    input_views=len(frames),elapsed_s=perf_counter()-t0,prior_s=t1-t0)
    prep=[bmk.prepare_tukey_levels(v['train']) for v in a]
    edges={}
    for i,j in bmk.ring(len(a)):
        init=bmk.rel(prior,i,j)
        if bad_edge and i==0:
            pivot=transform(center[None],np.linalg.inv(prior[j]))[0]
            edges[i,j]=axial(axis,pivot,np.deg2rad(90.))@init,.95
            continue
        try:
            e=bmk.tukey_icp(prep[i],prep[j],init)
            edges[i,j]=e,bmk.support(a[i],a[j],e)
        except (ValueError,RuntimeError):
            continue
    optimized,audit=fit_graph(a,prior,edges,center)
    admitted=sum(x['accepted'] for x in audit)
    tune=raw_fold(frames,tune_tags,anchor,1)
    report=raw_fold(frames,tags,anchor,5)
    r0,_=bmk.summarize(a,tune,prior,prior,axis,center,model)
    r1,_=bmk.summarize(a,tune,optimized,prior,axis,center,model)
    z=contact(frames,optimized,prior,-anchor[:3,2],0)
    use=select(r0,r1,admitted,z)
    e0,_=bmk.summarize(h,report,prior,prior,axis,center,model)
    e1,_=bmk.summarize(h,report,optimized,prior,axis,center,model)
    elapsed=perf_counter()
    return dict(selected='R06c' if use else 'R02',poses=optimized if use else prior,
                R02=prior,R06c=optimized,axis=cal,edges=audit,
                tune={'R02':r0,'R06c':r1},test={'R02':e0,'R06c':e1},
                contact_check_mm=z,contact_test_mm=contact(frames,optimized,prior,-anchor[:3,2],5),
                input_views=len(frames),elapsed_s=elapsed-t0,prior_s=t1-t0)
