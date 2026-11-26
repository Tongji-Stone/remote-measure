"""Re-run the E101 method suite on the board crop (E102) and object crop (E103)."""
import argparse
import csv
import json
import os
from pathlib import Path
import sys
from time import perf_counter

import cv2 as cv
import numpy as np
import open3d as o3d
from scipy.optimize import least_squares
from scipy.spatial import cKDTree

from foreground import HERE, CACHE, BRANCHES, SCANS, directory, dump, transform
from geometry import axial, project, read_models, write_scene
import benchmark as old

METHODS=dict(old.METHODS,R20='固定转轴 + 多邻边点到面 + ChArUco 锚定')


def read(path):
    return json.loads(path.read_text())


def matched_analysis(data, masks):
    pairs=[]
    for a,b,ma,mb in zip(data[0],data[1],masks[0],masks[1]):
        _,left,right=np.intersect1d(a['grid_index'],b['grid_index'],assume_unique=True,return_indices=True)
        good=ma['M04'][left]&mb['M04'][right]
        pairs.append((left[good],right[good]))
    return list(zip(*pairs))


def fixed_axis_registration(views, raw, meta, model):
    axis,center,theta = [np.array(meta[k]) for k in ('axis','center','phi')]
    basis=np.linalg.svd(axis[None,:])[2][1:].T
    base=axial(axis,center,-theta)
    n=len(views)
    cache=[]
    for i,(v,T) in enumerate(zip(views,base)):
        p=v['train'][::5]
        normal=v['train_normals'][::5]
        world=transform(p,T)
        nm=normal @ T[:3,:3].T
        cache.append((p,normal,world,nm,cKDTree(world)))
    links=[]
    for step in (1,2):
        for i in range(n):
            j=(i+step)%n
            P,PN,A,AN,AT=cache[i]
            Q,QN,B,BN,BT=cache[j]
            dist,idx=BT.query(A)
            _,rev=AT.query(B)
            good=(dist<=3.) & (rev[idx]==np.arange(len(A))) & (abs(np.sum(AN*BN[idx],axis=1))>=.82)
            if good.sum()<max(25,.07*min(len(A),len(B))):
                continue
            p=np.flatnonzero(good)
            p=p[np.linspace(0,len(p)-1,min(len(p),430)).astype(int)]
            q=idx[p]
            links.append((i,j,P[p],Q[q],PN[p],QN[q],.65 if step==2 else 1.))
    observations=[transform(v['board_xyz'],raw[0]['pose']) for v in raw]

    def poses(x):
        corrected=theta+np.r_[0.,x[2:]/46.]
        pivot=center+basis@x[:2]
        return axial(axis,pivot,-corrected)

    def cost(x):
        matrices=poses(x)
        Tinv=np.linalg.inv(matrices)
        res=[.52*x]
        for i,(p,d) in enumerate(zip(observations,raw)):
            e=project(transform(p,Tinv[i]),model)-d['board_uv']
            res.append((e*(1.25/np.sqrt(len(p)))).ravel())
        for i,j,P,Q,N,M,w in links:
            A,B=matrices[i],matrices[j]
            diff=transform(P,A)-transform(Q,B)
            ni=N@A[:3,:3].T
            nj=M@B[:3,:3].T
            d=np.c_[np.sum(diff*ni,axis=1),np.sum(diff*nj,axis=1)]
            res.append((w*4.1/np.sqrt(len(d))*d).ravel())
        return np.concatenate(res)

    max_change=np.r_[1.5,1.5,np.repeat(1.35,n-1)]
    fit=least_squares(cost,np.zeros(n+1),bounds=(-max_change,max_change),loss='soft_l1',
                      f_scale=.3,max_nfev=23)
    return poses(fit.x),dict(edges=len(links),neighbor_steps=[1,2],expected_n=n,
                            board_anchor_px=1.25,axis_shift_mm=fit.x[:2],
                            max_angle_correction_deg=float(np.rad2deg(abs(fit.x[2:]).max()/46.)))


def bottom_proxy(views, poses, base, raw):
    up=-raw[0]['pose'][:3,2]
    Z=[]
    for xy,a,b in zip(views,poses,base):
        if len(xy):
            Z.extend((transform(xy,a)-transform(xy,b))@up)
    if not len(Z):
        return dict(contact_z_drift_P95_mm=None,contact_z_bias_mm=None,contact_eval_n=0)
    return dict(contact_z_drift_P95_mm=np.percentile(abs(np.asarray(Z)),95),
                contact_z_bias_mm=np.median(Z),contact_eval_n=len(Z))


def main(scan,branch,mode,cc,go,ts):
    start=perf_counter()
    roots=[directory(scan)/b for b in BRANCHES]
    data=[[dict(np.load(p)) for p in sorted(root.glob('*.npz'))] for root in roots]
    folders=[CACHE/scan.name/b for b in BRANCHES]
    masks=[[dict(np.load(p)) for p in sorted(folder.glob('*.npz'))] for folder in folders]
    evaluation=matched_analysis(data,masks)
    idx=BRANCHES.index(branch)
    raw=data[idx]
    mode_mask='e102' if mode=='E102' else 'M04'
    spectra=masks[idx]
    counts=read(folders[idx]/'counts.json')
    meta=read(roots[idx]/'poses.json')
    name=scan.name.split('_',4)[4]
    out=HERE.parent/mode/branch/name
    global_out=out/'整圈'
    model=read_models()[branch]
    axis,center,cmd,phi,pnp = [np.array(meta[k]) for k in ('axis','center','cmd','phi','pnp')]
    base=axial(axis,center,-phi)
    cs=[{**r,'region':np.flatnonzero(m[mode_mask])} for r,m in zip(raw,spectra)]
    es=[{**r,'region':q} for r,q in zip(raw,evaluation[idx])]
    train=[old.sample_view(r) for r in cs]
    test=[old.sample_view(r) for r in es]
    prepared=[old.prepare_pair(v['train'],old.CONFIG,v['gray']) for v in train]
    contact=[]
    for r,m,e in zip(raw,spectra,evaluation[idx]):
        h=m['height'][e]
        g=e[(h>=-.85)&(h<=5.)&~old.training_mask(r['uv'][e])]
        contact.append(r['xyz'][g[::max(1,len(g)//400)]])
    images=sorted(scan.glob('view_*'))
    scratch=Path('/tmp')/mode/'solvers'/branch/scan.name[14:20]/str(os.getpid())
    edges,records=old.solve_edges(train,prepared,images,base,scratch,cc,go,ts)
    poses={'R01':axial(axis,center,-cmd),'R02':base,'R03':pnp}
    for code,pairs in edges.items():
        if pairs:
            poses[code]=old.sequence(pairs,base)
    # R06 preserves common E101 mode. R07 and R20 avoid fitting the board itself as an object facet.
    poses['R06'],n_graph=old.incremental(train,base,edges['R10'],center)
    if '三棱锥' in name:
        poses['R07']=old.turntable_fit(test,raw,axis,center,phi,base,model,planes=True)
    poses['R19']=old.turntable_fit(train,raw,axis,center,phi,base,model)
    poses['R20'],fit=fixed_axis_registration(test,raw,meta,model)
    points=[{k:r[k][m[mode_mask]] for k in ('xyz','gray')} for r,m in zip(raw,spectra)]
    n=len(points)
    for i,r in enumerate(points):
        write_scene(out/'单视角'/f'V{i:02}.ply',[r],[np.eye(4)],[i])
    metrics,detail,missing,full={}, {}, {}, {}
    for code,label in METHODS.items():
        T=poses.get(code)
        if T is None:
            missing[code]='pyramid only' if code=='R07' and '三棱锥' not in name else (
                'all edges rejected' if code in records else 'solver unavailable')
            continue
        row,detail[code]=old.summarize(test,raw,T,base,axis,center,model,edges.get(code))
        row.update(method=label,fallback_edges=sum(x['status']!='accepted' for x in records.get(code,[])),
                   **bottom_proxy(contact,T,base,raw))
        if code=='R06':
            row['graph_admitted_edges']=n_graph
        if code=='R20':
            row['graph_admitted_edges']=fit['edges']
        metrics[code]=row
        full[code]=write_scene(global_out/f'{code}.ply',points,T,range(n))
        write_scene(out/'双视角'/f'{code}.ply',points[:2],T[:2],range(2))
    supported={code:r for code,r in metrics.items() if (r['contact_z_drift_P95_mm'] or 0)<=1.5}
    chosen,eligible=old.prefer(supported)
    n_thin={code:old.thinning_metrics(code,test,raw,poses[code],base,axis,center,model)
            for code in ('R02','R06','R20',chosen)}
    write_scene(global_out/'推荐.ply',points,poses[chosen],range(n))
    write_scene(out/'双视角'/'推荐.ply',points[:2],poses[chosen][:2],range(2))
    dump(out/'裁剪.json',dict(source=counts,
                              registered_vertices=[len(p['xyz']) for p in points],
                              shared_evaluation_points=[len(e) for e in evaluation[idx]],
                              e101_axis_mm=center,axis=axis,model=branch))
    dump(out/'结果.json',dict(chosen=chosen,eligible=eligible,metrics=metrics,missing=missing,
                      per_edge_metrics=detail,edge_solver=records,matrices={k:poses[k] for k in metrics},
                      thinning=n_thin,full_cloud_vertices=full,n_views=n,experiment=mode,
                      R20=fit,elapsed_s=perf_counter()-start))
    fields=sorted(set().union(*(r.keys() for r in metrics.values())))
    with (out/'指标.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=['code',*fields]);w.writeheader()
        w.writerows(dict(code=c,**r) for c,r in metrics.items())
    print(mode,branch,name,chosen,len(metrics),full[chosen],
          round(metrics[chosen]['F1_2'],3),round(metrics[chosen]['depth_abs_P50'],3),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('mode',choices=('E102','E103'))
    p.add_argument('branch',choices=BRANCHES)
    p.add_argument('session')
    p.add_argument('--cloudcompare')
    p.add_argument('--go-icp')
    p.add_argument('--teaser')
    x=p.parse_args()
    cv.ocl.setUseOpenCL(False)
    cv.setNumThreads(1)
    cv.setRNGSeed(101)
    o3d.utility.random.seed(101)
    main(next(s for s in SCANS if x.session in s.name),x.branch,x.mode,x.cloudcompare,x.go_icp,x.teaser)
