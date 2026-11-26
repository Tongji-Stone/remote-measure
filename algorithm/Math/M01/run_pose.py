"""Rebuild the same envelope from three pose sets and compare the resulting volumes."""
import json
from pathlib import Path
import sys
from time import perf_counter

import csv
import cv2
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
sys.path.insert(0,str(ROOT/'simulation/E104'))
sys.path.insert(0,str(ROOT/'computing volume/V001'))
from experiment import CASES, real_input, sample_frames, voxel_hash
from reconstruct import tx, screened, footprint, grid_box, rays, envelope, oriented, ground_contact
from run import save_csv
from pose_sync import checks, graph_edges, log, synchronize
from surface_bias import metrics
from volume_math import Surface, checked_volume


def change(frames,mats):
    return [dict(p=tx(f['p'],D), camera=tx(f['camera'][None],D)[0],
                 to_camera=f['to_camera']@np.linalg.inv(D)) for f,D in zip(frames,mats)]


def kernel(frames,cal):
    p,n=oriented(frames)
    p,n=p[p[:,2]>=-.85],n[p[:,2]>=-.85]
    if len(p)>27000:
        take=np.random.default_rng(774).choice(len(p),27000,replace=False)
        p,n=p[take],n[take]
    guide,gn=footprint(p)
    free,_,_=rays(guide,frames,cal)
    keep=free<2
    guide,gn=guide[keep],gn[keep]
    gn=gn/np.linalg.norm(gn,axis=1,keepdims=True)
    m=screened(np.r_[p,guide],np.r_[n,gn])[0]
    lo,shape,grid=grid_box(np.r_[p,guide])
    free,_,_=rays(grid,frames,cal)
    return envelope(m,p,guide,lo,shape,grid,free,exposed=cal['exposed_board'])


def marker_check(code,name,poses,w,cal,folder):
    matches=list(Path('/tmp/E101/cache/object_scans').glob(f'scan_20261125_*_{name}'))
    if len(matches)!=1:
        print('Marker diagnostic requires E101 decoded object scan cache.',flush=True)
        return
    camera=matches[0]/'ChArUco'
    with np.load(camera/'00.npz') as zero:
        board=w@zero['pose']
    records={}
    for method,mats in poses.items():
        errors=[]
        for i,pose in enumerate(mats):
            with np.load(camera/f'{i:02}.npz') as frame:
                if len(frame['held_uv'])==0:
                    continue
                tt=np.linalg.solve(pose,board)
                p=tx(frame['held_xyz'],tt)
                uv=cv2.projectPoints(p,np.zeros(3),np.zeros(3),cal['cam_int'],cal['cam_dist'])[0][:,0]
                errors.extend(np.linalg.norm(uv-frame['held_uv'],axis=1))
        sensors=mats[:,:3,3]
        direction=np.sort(np.degrees(np.arctan2(sensors[:,1],sensors[:,0])))
        gap=np.max(np.diff(np.r_[direction,direction[0]+360]))
        records[method]=dict(held_marker_corners=len(errors),held_marker_p50_px=np.median(errors) if errors else None,
                             measured_axis_azimuth_gap_deg=gap)
    (folder/'pose_meshes'/f'{code}-poses.json').write_text(json.dumps(
            {'frame_to_e102_world':{k:v.tolist() for k,v in poses.items()},'marker_diagnostic':records},indent=2)+'\n')
    return records


def one(code,diagnostics_only=False):
    folder=HERE/'results'
    folder.mkdir(exist_ok=True)
    (folder/'pose_meshes').mkdir(exist_ok=True)
    (folder/'sync-checks.json').write_text(json.dumps(checks(),default=float,indent=2)+'\n')
    name,_=CASES[code]
    fuse=ROOT/'algorithm/scene-fusion'
    file=fuse/'E104/ChArUco'/name/'位姿.json'
    current=json.loads(file.read_text())
    old=json.loads((fuse/'E103/ChArUco'/name/'结果.json').read_text())
    poses=np.array(old['matrices'][old['chosen']])
    frames,_,cal,_,_,_,_=real_input(code,'ChArUco',with_board_evidence=True)
    w=np.linalg.inv(frames[0]['to_camera'])@np.linalg.inv(poses[0])
    old_world=w@poses
    base=w@np.array(current['R02'])
    target=w@np.array(current['R06c'])
    base_delta=base@np.linalg.inv(old_world)
    base_frames=change(frames,base_delta)
    fold=[voxel_hash(f['p'])==4 for f in base_frames]
    split=lambda fs,test: [{**f,'p':f['p'][k if test else ~k]} for f,k in zip(fs,fold)]
    ptrain=split(base_frames,False)
    center=np.median(np.vstack([f['p'][::25] for f in ptrain]),axis=0)
    edges=graph_edges([t['p'] for t in ptrain],center)
    corrections,pre,post=synchronize(len(base_frames),edges,center)
    pos={'R02':base,'R06c':target,'H1':corrections@base}
    markers=marker_check(code,name,pos,w,cal,folder) or {}
    if diagnostics_only:
        source=folder/f'pose_{code}.csv'
        with source.open() as handle:
            rows=list(csv.DictReader(handle))
        assert len(rows)==len(pos)
        for row in rows:
            row.update(markers.get(row['method'],{}))
        save_csv(source,rows)
        print(code,markers,flush=True)
        return
    row=[]
    for method, now in pos.items():
        start=perf_counter()
        delta=now@np.linalg.inv(old_world)
        np.testing.assert_allclose(now[0],base[0],atol=1e-9)
        obs=change(frames,delta)
        train=split(obs,False)
        test=split(obs,True)
        local_cal={**cal,'exposed_board':[tx(p,t) for p,t in zip(cal['exposed_board'],delta)]}
        estimate=kernel(train,local_cal)
        surf=Surface.of(estimate)
        vol=checked_volume(surf)
        clean=estimate.is_watertight()
        board=ground_contact(estimate)
        valid=bool(clean and board['touches_floor'] and board['min_z_mm']>=-1e-7)
        if valid:
            import open3d as o3d
            o3d.io.write_triangle_mesh(str(folder/'pose_meshes'/f'{code}-{method}.ply'),estimate,write_vertex_normals=False)
        sc=surf.scene()
        p,c=sample_frames(test,each=1050)
        points=[tx(f['p'][abs(f['p'][:,2])<1.6][::8],d) for f,d in zip(base_frames,now@np.linalg.inv(base))]
        plane=np.concatenate(points)[:,2]
        original=np.concatenate([f['p'][abs(f['p'][:,2])<1.6][::8] for f in base_frames])[:,2]
        correction=now@np.linalg.inv(base)
        metrics_6=np.array([log(t,center) for t in correction])
        scaled=np.array([40.,40.,40.,1.,1.,1.])
        error=np.array([np.linalg.norm(log(correction[j]@t@np.linalg.inv(correction[i]),center)*scaled)
                        for i,j,t,_ in edges])
        row.append(dict(case=code,method=method,steps=len(frames),pose_edges=len(edges),policy_in_E104=current['selected'],
                        p95_contact_drift_mm=np.percentile(abs(plane-original),95),
                        median_synchronized_edge_mm=np.median(error) if len(error) else None,
                        maximum_shift_in_twist_mm=np.max(np.linalg.norm(metrics_6[:,3:],axis=1)),
                        maximum_rotation_deg=np.rad2deg(np.max(np.linalg.norm(metrics_6[:,:3],axis=1))),
                        valid=valid,volume_ml=vol/1000 if valid else None,**metrics(sc,p,c),**board,
                        seconds=perf_counter()-start,**markers.get(method,{})))
        save_csv(folder/f'pose_{code}.csv',row)
        print(code,row[-1],flush=True)


if __name__=='__main__':
    one(sys.argv[1],'--diagnostics-only' in sys.argv[2:])
