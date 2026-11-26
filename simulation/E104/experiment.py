"""E104 measured-source reconstruction, controlled occlusion, and watertight-volume tests."""
import argparse
import csv
import json
from pathlib import Path
from time import perf_counter

import cv2 as cv
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from reconstruct import (HERE,METHODS,tx,transform,cloud,hull_mesh,screened,bpa,scene,oriented,
                         footprint,envelope,rays,grid_box,projective_surface,fit_planes,cylinder,polar_guides,
                         ground_contact,floor_hull)

from deck import exposed_views

ROOT=HERE.parents[1]
CASES={'P1':('三棱锥_小面坐地','pyramid'),'P2':('三棱锥_大面坐地','pyramid'),
       'F1':('坐着人偶','figurine'),'K1':('小南瓜','pumpkin'),'C1':('维生素包装圆柱','cylinder')}
CONTROLS=('G1','G2','G3')
EXPERIMENTS=('T0','T1','T2')


def read(path):
    return json.loads(path.read_text())


def dump(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False,default=lambda a:a.tolist())+'\n')


def ply(path):
    layout=np.dtype([('p','<f4',(3,)),('c','u1',(3,)),('view','u1')])
    with path.open('rb') as f:
        count=None
        for line in f:
            if line.startswith(b'element vertex '):
                count=int(line.split()[-1])
            if line==b'end_header\n':
                break
        assert count is not None and path.stat().st_size-f.tell()==count*layout.itemsize,path
        return np.fromfile(f,layout,count)['p'].astype(float)


def axes_and_floor(board0, object0, roi, branch):
    center=np.array(roi['e101_axis_mm'])
    u=np.array(roi['axis'])
    if u[2]>0:
        u=-u
    right=np.cross([0.,0.,1.],u)
    right/=np.linalg.norm(right)
    fwd=np.cross(u,right)
    B=np.c_[right,fwd,u]
    # Select plate pixels from the E102-only points in camera coordinates without raw caches.
    meta=np.dtype([('x','<f4'),('y','<f4'),('z','<f4')])
    a=board0.astype('float32').view(meta).reshape(-1)
    b=object0.astype('float32').view(meta).reshape(-1)
    keep=~np.isin(a,b)
    deck=(board0[keep]-center)@B
    guess=np.median(deck[:,2])
    good=abs(deck[:,2]-guess)<(3. if branch=='ChArUco' else 5.)
    selected=deck[good]
    q=selected[:,2]
    r=np.median(q)
    # Robust plane via local Huber; ChArUco owns pose and E102 measures the actual floor offset.
    from scipy.optimize import least_squares
    fit=least_squares(lambda p: selected[:,:2]@p[:2]+p[2]-q,[0.,0.,r],loss='soft_l1',f_scale=.6,
                      bounds=([-.12,-.12,r-4.],[.12,.12,r+4.]))
    new_up=u-B[:,:2]@fit.x[:2]
    new_up/=np.linalg.norm(new_up)
    new_right=right-new_up*(right@new_up);new_right/=np.linalg.norm(new_right)
    basis=np.c_[new_right,np.cross(new_up,new_right),new_up]
    origin=center+fit.x[2]*u
    return transform(basis.T,-origin@basis), dict(deck_points=len(q),offset_mm=fit.x[2],
            plane_P50_mm=np.median(abs(selected[:,:2]@fit.x[:2]+fit.x[2]-q)),center=origin,basis=basis)


def real_input(code,branch,pose_source='E103',*,with_board_evidence=False):
    name,shape=CASES[code]
    fusion=ROOT/'algorithm/scene-fusion'
    e2=fusion/'E102'/branch/name
    e3=fusion/'E103'/branch/name
    picked=e3 if pose_source=='E103' else e2
    result=read(picked/'结果.json')
    poses=np.array(result['matrices'][result['chosen']])
    a=[ply(f) for f in sorted((e2/'单视角').glob('*.ply'))]
    b=[ply(f) for f in sorted((e3/'单视角').glob('*.ply'))]
    world,plane=axes_and_floor(a[0],b[0],read(e3/'裁剪.json'),branch)
    c=read(fusion/'E101'/branch/'标定.json')
    cal={k:np.asarray(c[k],float) for k in ('cam_int','cam_dist')}
    cal['shape']=(2048,2448)
    if with_board_evidence:
        evidence=HERE/'场景'/f'{code}-{branch}'/'E102露板索引.npz'
        cal['exposed_board']=exposed_views(name,branch,a,poses,world,evidence)
    def frames(seq):
        v=[]
        for p,T in zip(seq,poses):
            W=world@T
            data=tx(p,W)
            v.append(dict(p=data,camera=W[:3,3].copy(),to_camera=np.linalg.inv(W)))
        return v
    return frames(b),frames(a),cal,plane,None,shape,result['chosen']


def control_mesh(code):
    if code=='G1':
        v=np.array([[-35.,-27.,0.],[37.,-21.,0.],[-9.,33.,0.],[-8.,-5.,62.]])
        m=hull_mesh(v)
        return m,'pyramid',float(abs(np.linalg.det(v[1:]-v[0]))/6.)
    if code=='G2':
        radius,height=19.,75.
        m=o3d.geometry.TriangleMesh.create_cylinder(radius,height,128,14)
        m.translate([0,0,height/2])
        return m,'cylinder',float(np.pi*radius**2*height)
    # Closed freeform surface with known tessellated volume and azimuthal ribs.
    mesh=o3d.geometry.TriangleMesh.create_sphere(1.,resolution=65)
    p=np.asarray(mesh.vertices)
    z=p[:,2].copy()
    az=np.arctan2(p[:,1],p[:,0])
    ridge=1.+.094*np.cos(9*az)*(1-z**2)**.5
    p[:,:2]*=(25*ridge*(1+.12*z))[:,None]
    p[:,2]=(z+1)*15
    return mesh,'pumpkin',float(mesh.get_volume())


def simulated(code):
    m,shape,reference=control_mesh(code)
    base,_,cal,_,_,_,_=real_input('F1','ChArUco')
    sc=scene(m)
    observed=[]
    # Keep the actual viewing directions and optics; center the synthetic object on a separate plate datum.
    for i,d in enumerate(base):
        W=np.linalg.inv(d['to_camera'])
        xx,yy=np.meshgrid(np.arange(250,2100,5),np.arange(160,1900,5))
        px=np.c_[xx.ravel(),yy.ravel()].astype(float)
        u=cv.undistortPoints(px,cal['cam_int'],cal['cam_dist']).reshape(-1,2)
        direct=np.c_[u,np.ones(len(u))]@W[:3,:3].T
        origin=np.broadcast_to(W[:3,3],direct.shape)
        t=sc.cast_rays(o3d.core.Tensor(np.c_[origin,direct].astype('float32')),nthreads=1)['t_hit'].numpy()
        keep=np.isfinite(t)
        rng=np.random.default_rng(770+i)
        good=keep & (rng.uniform(size=len(t))>.10)
        f=origin[good]+direct[good]*(t[good]+rng.normal(0,.22,good.sum()))[:,None]
        observed.append({**d,'p':f})
    return observed,observed,cal,{'deck_points':0,'offset_mm':0.,'plane_P50_mm':0.},(m,reference),shape,'模拟固定真值'


def voxel_hash(p):
    q=np.floor(p/2.2).astype(np.int64)
    return (q[:,0]*131+q[:,1]*23+q[:,2]*47)%11


def split(frames,trial):
    whole=np.vstack([f['p'] for f in frames])
    center=np.median(whole,axis=0)
    lo,hi=np.percentile(whole[:,2],[1.,98.])
    ceiling=lo+.18*(hi-lo)
    out=[];validation=[];occluded=[]
    for frame in frames:
        p=frame['p']
        angle=np.arctan2(p[:,1]-center[1],p[:,0]-center[0])
        if trial=='T1':
            hidden=p[:,2]<ceiling
        elif trial=='T2':
            hidden=(np.abs(np.angle(np.exp(1j*(angle-.58))))<np.deg2rad(36))&(p[:,2]>lo+.19*(hi-lo))
        else:
            hidden=np.zeros(len(p),bool)
        check=(voxel_hash(p)==4) & ~hidden
        out.append({**frame,'p':p[~hidden&~check]})
        validation.append({**frame,'p':p[check]})
        occluded.append({**frame,'p':p[hidden]})
    return out,validation,occluded,dict(ceiling_mm=ceiling,center_xy=center[:2],trial=trial,
        hidden_points=sum(len(r['p']) for r in occluded),visible_holdout_points=sum(len(r['p']) for r in validation),
        measured_training_points=sum(len(r['p']) for r in out))


def sample_frames(frames,each=1050):
    p,c=[],[]
    for f in frames:
        n=len(f['p'])
        if n:
            selected=f['p'][np.linspace(0,n-1,min(each,n)).astype(int)]
            p.append(selected)
            c.append(np.broadcast_to(f['camera'],selected.shape))
    if not p:
        return np.empty((0,3)),np.empty((0,3))
    return np.vstack(p),np.vstack(c)


def metric(m, train, visible, missing, truth=None):
    if m is None or not len(m.triangles):
        return {'status':'no_surface'}
    xyz=np.asarray(m.vertices)
    sc=scene(m)
    loss=ground_contact(m)
    for tag,frames in [('observed',visible),('hidden',missing)]:
        p,c=sample_frames(frames)
        if not len(p):
            continue
        e=sc.compute_distance(o3d.core.Tensor(p.astype('float32')),nthreads=1).numpy()
        loss.update({f'{tag}_n':len(p),f'{tag}_p50':np.median(e),f'{tag}_p95':np.percentile(e,95),
                     f'{tag}_r2':np.mean(e<=2.)})
        if tag=='observed':
            unit=p-c
            length=np.linalg.norm(unit,axis=1)
            unit/=length[:,None]
            t=sc.cast_rays(o3d.core.Tensor(np.c_[c,unit].astype('float32')),nthreads=1)['t_hit'].numpy()
            hit=np.isfinite(t)
            loss['ray_hit_rate']=hit.mean()
            loss['ray_front3_rate']=np.mean(t[hit]<length[hit]-3.) if hit.any() else None
            loss['ray_abs_p95']=np.percentile(abs(t[hit]-length[hit]),95) if hit.any() else None
    o3d.utility.random.seed(771)
    sampled=np.asarray(m.sample_points_uniformly(11000).points)
    loss['unsupported_2p5']=np.mean(cKDTree(train).query(sampled)[0]>2.5)
    loss['below_plane_frac']=np.mean(sampled[:,2]<-1.)
    watertight=m.is_watertight()
    orientable=m.is_orientable() if watertight else False
    closed=watertight and orientable
    volume=m.get_volume() if closed else None
    if truth:
        target,expected=truth
        sc2=scene(target)
        o3d.utility.random.seed(772)
        g=np.asarray(target.sample_points_uniformly(9000).points)
        d1=sc.compute_distance(o3d.core.Tensor(g.astype('float32')),nthreads=1).numpy()
        d2=sc2.compute_distance(o3d.core.Tensor(sampled.astype('float32')),nthreads=1).numpy()
        recall=(d1<=2).mean();prec=(d2<=2).mean()
        loss.update(gt_surface_F1_2=2*recall*prec/max(recall+prec,1e-10),
                    gt_volume_mm3=expected,volume_relative_error=None if volume is None else volume/expected-1.)
    _,comp,_=m.cluster_connected_triangles()
    return dict(status='scored',vertices=len(xyz),triangles=len(m.triangles),components=len(comp),watertight=watertight,
                orientable=orientable,volume_mm3=volume,**loss)


def save(path,m):
    m.compute_vertex_normals()
    m.paint_uniform_color([.94,.94,.94])
    if not o3d.io.write_triangle_mesh(str(path),m):
        raise IOError(path)


def trial_out(folder,frames,cal,trial,shape,truth,modes):
    t,visible,missing,info=split(frames,trial)
    p,n=oriented(t)
    if len(p)>27000:
        take=np.random.default_rng(774).choice(len(p),27000,replace=False)
        p,n=p[take],n[take]
    models={}
    elapsed={}

    def make(key,fn):
        start=perf_counter()
        try:
            models[key]=fn()
        except (RuntimeError,ValueError,ArithmeticError) as exc:
            models[key]=None
            info.setdefault('errors',{})[key]=str(exc)
        elapsed[key]=perf_counter()-start

    if 'S01' in modes:
        make('S01',lambda:screened(p,n))
    if 'S02' in modes:
        make('S02',lambda:bpa(p,n))
    guide,gn=footprint(p)
    if 'S03' in modes:
        make('S03',lambda:floor_hull(p,guide))
    if len(gn):
        gn/=np.linalg.norm(gn,axis=1,keepdims=True)
    guided=None
    if 'S04' in modes or 'S05' in modes and shape in ('pumpkin','figurine'):
        use=np.ones(len(guide),bool)
        if len(guide):
            free,_,_=rays(guide,t,cal)
            use=free<2
        guide,gn=guide[use],gn[use]
        make('S04-guided',lambda:screened(np.r_[p,guide],np.r_[n,gn])[0])
        guided=models.pop('S04-guided')
    polar,pol_norm=polar_guides(p) if shape=='pumpkin' else (np.empty((0,3)),np.empty((0,3)))
    if len(polar):
        vote,_,_=rays(polar,t,cal)
        polar,pol_norm=polar[vote<2],pol_norm[vote<2]
    if 'S04' in modes or 'S06' in modes or ('S07' in modes and len(polar)):
        lo,size,grid=grid_box(np.r_[p,guide])
        free,val,count=rays(grid,t,cal)
    if 'S04' in modes:
        make('S04',lambda:envelope(guided,p,guide,lo,size,grid,free,exposed=cal.get('exposed_board',())) if guided is not None else None)
    if 'S05' in modes:
        if shape=='pyramid':
            make('S05',lambda:fit_planes(p,n))
        elif shape=='cylinder':
            make('S05',lambda:cylinder(p))
        elif shape=='pumpkin' and len(polar):
            make('S05',lambda:screened(np.r_[p,polar],np.r_[n,pol_norm])[0])
        else:
            models['S05']=guided
            elapsed['S05']=elapsed.get('S04-guided',0.)
    if 'S07' in modes and shape=='pumpkin' and len(polar):
        def seal_lobes():
            support,_=screened(np.r_[p,polar],np.r_[n,pol_norm])
            q,m,grid2=grid_box(np.r_[p,polar])
            free2,_,_=rays(grid2,t,cal)
            return envelope(support,p,polar,q,m,grid2,free2,exposed=cal.get('exposed_board',()))
        make('S07',seal_lobes)
    if 'S06' in modes:
        make('S06',lambda:projective_surface(val,count,lo,size))
    summaries={}
    for key,out in models.items():
        if key=='S01' and out is not None:
            out,cut=out
            save(folder/f'{trial}-S01-visible.ply',cut)
        scored=metric(out,p,visible,missing,truth)
        scored['elapsed_s']=elapsed[key]+(elapsed.get('S04-guided',0.) if key in ('S04',) else 0.)
        path=folder/f'{trial}-{key}.ply'
        if out is not None and len(out.triangles):
            save(path,out)
        else:
            path.unlink(missing_ok=True)
        summaries[key]=scored
    for kind,source in [('input',t),('withheld',missing)]:
        q,_=sample_frames(source,each=5000)
        if len(q):
            o3d.io.write_point_cloud(str(folder/f'{trial}-{kind}.ply'),cloud(q))
    info['virtual_guides']=len(guide)
    return dict(split=info,methods=summaries)


def main(code,branch,refresh=False,verify_board=False):
    ctrl=code in CONTROLS
    folder=HERE/'场景'/(code if ctrl else f'{code}-{branch}')
    folder.mkdir(parents=True,exist_ok=True)
    source,board,cal,plane,truth,shape,picked=simulated(code) if ctrl else real_input(code,branch,with_board_evidence=True)
    contact=dict(board_z_mm=0.,maximum_basal_extension_mm=3.75,
                 cap='planar lattice triangulation',
                 exposed_board_points_per_view=[len(p) for p in cal.get('exposed_board',())])
    output=dict(case=code,branch=branch,shape=shape,optical_floor=plane,
                initial_pose=picked,contact=contact,tests={})
    if refresh or verify_board:
        old=read(folder/'结果.json')
        old['contact']=contact
        for t in EXPERIMENTS:
            if verify_board:
                rerun={'S04','S07'} if shape=='pumpkin' else {'S04'}
            else:
                rerun={'S03','S04','S05','S07'} if shape=='pumpkin' else {'S03','S04','S05'}
            row=trial_out(folder,source,cal,t,shape,truth,rerun)
            old['tests'][t]['methods'].update(row['methods'])
            print(code,branch,t,{k:(v.get('watertight'),v.get('touches_floor'),v.get('volume_mm3'))
                   for k,v in row['methods'].items()},flush=True)
        if not ctrl:
            extra,_,_,_,_,_,_=real_input(code,branch,'E102',with_board_evidence=True)
            row=trial_out(folder,extra,cal,'E102-pose',shape,None,{'S04'})
            old['input_controls']['E102-pose']['methods']['S04']=row['methods']['S04']
        dump(folder/'结果.json',old)
        return
    if truth:
        save(folder/'G0-reference.ply',truth[0])
    for trial in EXPERIMENTS:
        output['tests'][trial]=trial_out(folder,source,cal,trial,shape,truth,set(METHODS))
        print(code,branch,trial,{k: (v.get('watertight'),round(v.get('hidden_r2',-1),3),
                round(v.get('observed_p95',-1),2)) for k,v in output['tests'][trial]['methods'].items()},flush=True)
        dump(folder/'结果.json',output)
    if not ctrl:
        extra,b2,_,_,_,_,prior=real_input(code,branch,'E102',with_board_evidence=True)
        # Fixed object measurements, independent alignment comparison for real scans.
        for context,x in [('E102-pose',extra),('E102-direct',board)]:
            keys={'S01','S04'} if context=='E102-pose' else {'S01'}
            # Baseline board contamination: report against the identical object-only held-out test rays.
            ctl=trial_out(folder,x,cal,context,shape,None,keys)
            if context=='E102-direct':
                _,held,_missing,_=split(source,'T0')
                fp=folder/f'{context}-S01.ply'
                m=o3d.io.read_triangle_mesh(str(fp))
                p,_=oriented(source)
                ctl['against_object_only']=metric(m,p,held,[])
            output.setdefault('input_controls',{})[context]=ctl
        output['E102_pose_method']=prior
        dump(folder/'结果.json',output)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('case',choices=(*CASES,*CONTROLS))
    parser.add_argument('branch',choices=('ChArUco','Chessboard','GT'))
    parser.add_argument('--refresh-contact','--refresh-envelope',dest='refresh',action='store_true')
    parser.add_argument('--use-board-footprint',action='store_true')
    a=parser.parse_args()
    cv.setNumThreads(1);cv.ocl.setUseOpenCL(False)
    o3d.utility.random.seed(104)
    main(a.case,a.branch,a.refresh,a.use_board_footprint)
