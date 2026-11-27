"""Evaluate surface repair on actual held-out views and known-geometry controls."""
import argparse
import csv
from dataclasses import replace
from time import perf_counter

import cv2 as cv
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from core import (Frame, GROUPS, PITCH, ROOT, estimate, get_case, load,
                  one_per_voxel, prepare, transform)
from prepare import dump

QUALIFIED = ('B01-TSDF','B02-MLS','B03-Hybrid')


def world(f):
    return transform(f.camera_xyz,np.linalg.inv(f.world_to_camera))


def pooled(frames):
    x=np.vstack([world(f) for f in frames])
    x=x[one_per_voxel(x,.40)]
    quality=cKDTree(x).query(x,k=7)[0][:,-1]<1.6
    return x[quality]


def points_for(case, name):
    scan=load(name)
    W=np.linalg.inv(scan['P'])@np.linalg.inv(scan['B'])
    from core import HERE, read_ply
    frames=[]
    for j,M in enumerate(W):
        q=read_ply(HERE.parent/'E202'/scan['name']/'单视角'/f'V{j:02}.ply')
        frames.append(Frame(q['xyz'].astype(float),M,0,j))
    return dict(frames=frames,camera=case['camera'])


def metric(x,y):
    d=cKDTree(x).query(y)[0]
    return dict(target_points=len(y),p50_mm=np.median(d),p95_mm=np.percentile(d,95),
                within_05=np.mean(d<=.5),within_075=np.mean(d<=.75),within_1=np.mean(d<=1.0))


def cut(f,centers,radius):
    d=cKDTree(centers).query(world(f))[0]
    return replace(f,camera_xyz=f.camera_xyz[d>radius])


def automatic_centers(frames):
    q=pooled(frames)
    tree=cKDTree(q)
    candidates = q[::23]
    density=tree.query(candidates,k=18)[0][:,-1]
    candidates=candidates[(density<.96)&(density>.42)]
    pos=[]
    index=np.random.default_rng(204).permutation(len(candidates))
    for c in candidates[index]:
        if all(np.linalg.norm(c-p)>9. for p in pos):
            pos.append(c)
        if len(pos)>=11:
            break
    return np.array(pos)


def perturbed(train):
    rng=np.random.default_rng(204)
    out=[]
    for f in train:
        if f.station%7==0:
            continue
        p=f.camera_xyz.copy()
        noise=rng.normal(0,.21,len(p))
        idx=rng.choice(len(p),size=len(p)//70,replace=False)
        noise[idx]+=rng.normal(0,2.,len(idx))
        ray=p/np.maximum(p[:,2:3],1e-8)
        p+=ray*noise[:,None]
        out.append(replace(f,camera_xyz=p))
    return out


def fixtures():
    sphere=o3d.geometry.TriangleMesh.create_sphere(radius=13,resolution=64)
    groove=o3d.geometry.TriangleMesh.create_box(24,16,1.9).translate((-12,-8,-6.4))
    for x0 in (-12,.90):
        groove+=o3d.geometry.TriangleMesh.create_box(11.10 if x0==-12 else 11.10,16,6.5).translate((x0,-8,-4.5))
    return (('球面缺带',sphere),('1.8mm可见沟槽',groove))


def rays_for(mesh,name):
    K=np.array([[590.,0.,307.],[0.,590.,227.],[0.,0.,1.]])
    D=np.zeros(5,float)
    h,w=456,616
    size=(h,w)
    scene=o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    u,v=np.meshgrid(np.arange(0,w,3),np.arange(0,h,3))
    pix=np.c_[(u.ravel()-K[0,2])/K[0,0],(v.ravel()-K[1,2])/K[1,1],np.ones(u.size)]
    frames=[]
    angles=[(i,25) for i in range(0,360,45)]+[(35,75),(155,80),(270,75)]
    for j,(a,e) in enumerate(angles):
        a,e=np.deg2rad([a,e])
        c=78*np.array([np.cos(a)*np.cos(e),np.sin(a)*np.cos(e),np.sin(e)])
        z=-c/np.linalg.norm(c)
        x=np.cross(z,[0.,0.,1.]);x/=np.linalg.norm(x)
        R=np.vstack([x,np.cross(z,x),z])
        rays=np.c_[np.repeat(c[None],len(pix),axis=0),pix@R]
        hit=scene.cast_rays(o3d.core.Tensor(rays.astype('float32')),nthreads=1)['t_hit'].numpy()
        use=np.isfinite(hit)
        P=np.eye(4)
        P[:3,:3],P[:3,3]=R,-R@c
        frames.append(Frame(pix[use]*hit[use,None],P,0,j))
    return dict(frames=frames,camera=(K,D,size))


def synthetic(output,pitch):
    for name,mesh in fixtures():
        case=rays_for(mesh,name)
        f=case['frames']
        true=pooled(f)
        if name=='球面缺带':
            pos=np.array([[6.,.0,np.sqrt(13.**2-6.**2)],[-4.,8.,np.sqrt(13.**2-80.)]])
            radius=1.65
            train=[cut(a,pos,radius) for a in f]
            target=true[cKDTree(pos).query(true)[0]<1.18]
        else:
            train=f
            target=true[(abs(true[:,0])<2.3)&(abs(true[:,1])<6.3)]
        estimates,detail=estimate(train,case['camera'],pitch)
        estimates['B00-measured']=pooled(train)
        for method,x in estimates.items():
            row=metric(x,target)
            if name=='球面缺带':
                extra=x[cKDTree(pos).query(x)[0]<1.20]
                row['surface_p95_mm']=np.percentile(abs(np.linalg.norm(extra,axis=1)-13.),95) if len(extra) else None
                row['inside_mask_points']=len(extra)
            else:
                space=(abs(x[:,0])<.68)&(abs(x[:,1])<6.3)&(x[:,2]>-3.5)
                row['false_free_points']=int(space.sum())
                row['channel_floor_retention']=np.mean(cKDTree(x).query(
                    target[(target[:,2]<-4.42)&(abs(target[:,0])<.75)])[0]<.6)
            output.append(dict(case=name,scenario='已知形状',method=method,**row))
        print(name,[r for r in output if r['case']==name],flush=True)


def real(name,case,output,pitch,scenarios,details):
    f=case['frames']
    train=[a for a in f if a.station not in (2,7)]
    held=pooled([a for a in f if a.station in (2,7)])
    centers=automatic_centers(train)
    sets={
        '留出角站':(train,held),
        '去除小片1.45mm':([cut(a,centers,1.45) for a in train],
                           held[cKDTree(centers).query(held)[0]<1.05]),
        '去除大片2.50mm':([cut(a,centers,2.50) for a in train],
                           held[cKDTree(centers).query(held)[0]<1.69]),
        '缺角站加噪':(perturbed(train),held),
    }
    details[name]=dict(test_stations=[2,7],artificial_patch_centers_mm=centers,
                       training_stations=len(train),all_stations=len(f))
    for k in scenarios:
        if k not in sets:
            continue
        train,y=sets[k]
        t=perf_counter()
        estimates,info=estimate(train,case['camera'],pitch)
        estimates['B00-measured']=pooled(train)
        for method,x in estimates.items():
            output.append(dict(case=name,scenario=k,method=method,**metric(x,y)))
        print(name,k,[(r['method'],round(r['within_075'],4),round(r['p95_mm'],4))
                      for r in output if r['case']==name and r['scenario']==k],
                      info,round(perf_counter()-t,2),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--pitch',type=float,default=PITCH)
    parser.add_argument('--synthetic-only',action='store_true')
    parser.add_argument('--skip-synthetic',action='store_true')
    opts=parser.parse_args()
    cv.setNumThreads(1)
    results=[]
    details={}
    if not opts.skip_synthetic:
        synthetic(results,opts.pitch)
    if not opts.synthetic_only:
        for name in GROUPS:
            cas=get_case(name)
            real(name,cas,results,opts.pitch,['留出角站','去除小片1.45mm','去除大片2.50mm','缺角站加噪'],details)
        real('三棱锥·独立测试',points_for(cas,'021427'),results,opts.pitch,
             ['留出角站','去除小片1.45mm','缺角站加噪'],details)
    fields=list(dict.fromkeys(k for row in results for k in row))
    dest=ROOT/f'实验指标-{opts.pitch:.2f}.csv'
    with dest.open('w',encoding='utf-8-sig',newline='') as file:
        writer=csv.DictWriter(file,fieldnames=fields)
        writer.writeheader()
        writer.writerows(results)
    dump(ROOT/f'实验设置-{opts.pitch:.2f}.json',dict(pitch_mm=opts.pitch,settings=details))
