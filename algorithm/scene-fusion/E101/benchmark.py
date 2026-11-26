"""E101 full-ring registration, validation, and exact full-scene exports."""
import argparse
import csv
import os
from pathlib import Path
import subprocess
import sys
from time import perf_counter

import cv2 as cv
import numpy as np
import open3d as o3d
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from acquire import BRANCHES, CACHE, HERE, SCANS, directory, load_frame
from calibrate import dump
from geometry import axial, project, read_models, transform, write_scene

sys.path.insert(0, str(HERE.parent / 'E003'))
from registration_pipeline import cloud, feature_pack, sac_ia
from registration import incremental, edge_closure
from registration_experiment import cloudcompare, fit_pair, go_icp, prepare_pair, rigid_alignment, training_mask
from calibrated_registration_benchmark import pair_metric

REG = o3d.pipelines.registration
CONFIG = dict(training_voxels=[2.4,1.4,.85], training_gates=[7.,4.,2.5], fpfh_voxel=2.5,
              fpfh_ransac_iterations=7000, cloudcompare_overlap=70,
              goicp_trim_fraction=.45, goicp_distance_grid=80, goicp_timeout_seconds=3, seed=101)
METHODS = {
    'R01': '指令角 + 标定轴',
    'R02': '标记实测角 + 共轴',
    'R03': '逐站 ChArUco 标记 PnP',
    'R04': '点到点 ICP',
    'R05': '点到面 ICP',
    'R06': 'Tukey ICP + 门控位姿图',
    'R07': '棱锥侧面 + 共轴',
    'R08': 'GICP',
    'R09': '灰度 ICP',
    'R10': 'Tukey ICP 串联',
    'R11': 'FPFH–RANSAC + ICP',
    'R12': 'FPFH–FGR + ICP',
    'R13': 'ISS–FPFH–SAC-IA + ICP',
    'R14': 'OpenCV PPF + ICP',
    'R15': '白场 SIFT + 三维对应 + ICP',
    'R16': 'Go-ICP + ICP',
    'R17': 'CloudCompare ICP',
    'R18': 'TEASER++ + ICP',
    'R19': '共轴角微调 + 局部匹配',
}
PAIRS = dict(R04='point_to_point', R05='point_to_plane', R08='gicp', R09='gray_icp',
             R10='tukey_icp', R11='fpfh_ransac', R12='fpfh_fgr')


def ring(n):
    return [(i,(i+1)%n) for i in range(n)]


def rel(poses,a,b):
    return np.linalg.inv(poses[b]) @ poses[a]


def sample_view(v):
    inds = v['region']
    p, uv, gray = [v[k][inds] for k in ('xyz','uv','gray')]
    train = training_mask(uv)
    # Stable held-out labels and a deterministic sample order, shared across the optical branches.
    rng = np.random.default_rng(101)
    a = rng.permutation(np.where(train)[0])[:8500]
    b = rng.permutation(np.where(~train)[0])[:1700]
    radius = cKDTree(p[a]).query(p[a], k=12)[0][:,-1]
    keep = radius <= max(2.8, np.median(radius)+4*max(.16,np.median(abs(radius-np.median(radius)))))
    a = a[keep]
    return dict(train=p[a].astype(float), val=p[b].astype(float), uv=uv[a], gray=gray[a]/255,
                val_uv=uv[b], normals=np.asarray(cloud(p[b],4.5).normals),
                train_normals=np.asarray(cloud(p[a],4.5).normals))


def prepare_sift(folder,v):
    im = load_frame(folder, 42)
    small = cv.resize(im,None,fx=.5,fy=.5,interpolation=cv.INTER_AREA)
    mask = np.zeros_like(small)
    xy = (v['uv']/2).astype(int)
    mask[xy[:,1],xy[:,0]] = 255
    mask = cv.dilate(mask, np.ones((5,5), np.uint8))
    sift = cv.SIFT_create(nfeatures=1800,contrastThreshold=.015)
    keys, d = sift.detectAndCompute(small,mask)
    if d is None:
        return np.empty((0,3)),None
    uv = (np.array([k.pt for k in keys])*2 + .5)
    dd, i = cKDTree(v['uv']).query(uv)
    good = (dd <= 6) & training_mask(uv)
    return v['train'][i[good]],d[good]


def fit_matches(A,B,gate=3.5, rng_seed=101):
    if len(A)<5:
        raise ValueError('Too few feature matches')
    best = np.zeros(len(A),bool)
    rng = np.random.default_rng(rng_seed)
    for _ in range(500):
        idx = rng.choice(len(A),3,replace=False)
        if np.linalg.norm(np.cross(A[idx[1]]-A[idx[0]],A[idx[2]]-A[idx[0]])) < 5:
            continue
        T = rigid_alignment(A[idx],B[idx])
        m = np.linalg.norm(transform(A,T)-B,axis=1) <= gate
        if m.sum() > best.sum():
            best = m
    if best.sum()<5:
        raise ValueError('Insufficient feature consensus')
    return rigid_alignment(A[best],B[best])


def sift_pair(A,B):
    x, a = A
    y, b = B
    if a is None or b is None or min(len(a),len(b)) < 4:
        raise ValueError('SIFT descriptors absent')
    bf = cv.BFMatcher()
    back = {m.queryIdx:m.trainIdx for m in bf.match(b,a)}
    matches = [(m.queryIdx,m.trainIdx) for m,n in bf.knnMatch(a,b,k=2)
               if m.distance < .77*n.distance and back.get(m.trainIdx)==m.queryIdx]
    if len(matches)<5:
        raise ValueError('Too few reciprocal SIFT descriptors')
    i,j = np.asarray(matches).T
    return fit_matches(x[i],y[j])


def ppf_sample(prep):
    p = np.c_[np.asarray(prep.points),np.asarray(prep.normals)].astype(np.float32)
    return p[np.linspace(0,len(p)-1,min(len(p),440)).astype(int)]


def ppf_pair(a,b):
    detector = cv.ppf_match_3d_PPF3DDetector(.06,.065,28)
    detector.trainModel(a)
    best = max(detector.match(b,.45,.07)[:5],key=lambda v:v.numVotes,default=None)
    if best is None or best.numVotes < 5:
        raise ValueError('No supported PPF vote')
    return best.pose.copy()


def teaser_pair(src,dst,runner,folder):
    _, a, fa = src
    _, b, fb = dst
    A, B = np.asarray(a.points),np.asarray(b.points)
    da, db = np.asarray(fa.data).T,np.asarray(fb.data).T
    nearest = cKDTree(db).query(da)[1]
    inverse = cKDTree(da).query(db)[1]
    ii = np.flatnonzero(inverse[nearest]==np.arange(len(A)))
    if len(ii) < 5:
        raise ValueError('Too few mutual FPFH correspondences')
    ii = ii[np.linspace(0,len(ii)-1,min(len(ii),300)).astype(int)]
    folder.mkdir(parents=True,exist_ok=True)
    p = folder/'pairs.txt'
    np.savetxt(p,np.c_[A[ii],B[nearest[ii]]])
    result = subprocess.run([runner,str(p)],check=True,timeout=8,text=True,capture_output=True)
    values = np.fromstring(result.stdout.split('E101_TRANSFORM\n')[-1],sep=' ')
    if len(values)!=16:
        raise ValueError('Bad TEASER++ output')
    return values.reshape(4,4)


def support(a,b,T):
    P,Q = a['train'][::3],b['train'][::3]
    P = transform(P,T)
    return min(np.mean(cKDTree(Q).query(P)[0]<=4),np.mean(cKDTree(P).query(Q)[0]<=4))


def solve_edges(v, prep, folders, base, scratch, cc, go, teaser, methods=None):
    n = len(v)
    methods = methods or [*PAIRS,'R13','R14','R15','R16','R17','R18']
    sift = [prepare_sift(f,x) for f,x in zip(folders,v)] if 'R15' in methods else []
    iss = [feature_pack(x['train']) for x in v] if 'R13' in methods else []
    ppf = [ppf_sample(x[1]) for x in prep] if 'R14' in methods else []
    active = [m for m in methods if (m!='R16' or go) and (m!='R17' or cc) and (m!='R18' or teaser)]
    solutions = {m: {} for m in active}
    records = {m:[] for m in active}
    for m in active:
        for a,b in ring(n):
            initial = rel(base,a,b)
            folder = scratch / m / f'{a:02}-{b:02}'
            folder.parent.mkdir(parents=True,exist_ok=True)
            ts = perf_counter()
            try:
                if m=='R05':
                    T = initial
                    for X,Y,gate in zip(prep[a][0],prep[b][0], CONFIG['training_gates']):
                        T = REG.registration_icp(X,Y,gate,T,REG.TransformationEstimationPointToPlane(),
                                                 REG.ICPConvergenceCriteria(max_iteration=50)).transformation
                elif m in PAIRS:
                    T,_ = fit_pair(PAIRS[m],prep[a],prep[b],initial,CONFIG,None,None,101+a)
                elif m=='R17':
                    T = cloudcompare(np.asarray(prep[a][0][-1].points),
                                     np.asarray(prep[b][0][-1].points),initial,CONFIG,folder,cc)
                else:
                    if m=='R13':
                        T,_,_ = sac_ia(iss[a],iss[b],np.random.default_rng(101+a),shots=520)
                        if T is None:
                            raise ValueError('No spaced ISS/FPFH consensus')
                    elif m=='R14':
                        T = ppf_pair(ppf[a],ppf[b])
                    elif m=='R15':
                        T = sift_pair(sift[a],sift[b])
                    elif m=='R16':
                        x,y = v[a]['train'],v[b]['train']
                        T,_ = go_icp(x[:650],y[:900],go,folder,CONFIG)
                    elif m=='R18':
                        T = teaser_pair(prep[a],prep[b],teaser,folder)
                    T,_ = fit_pair('tukey_icp',prep[a],prep[b],T,CONFIG,None,None,101+a)
                amount = support(v[a],v[b],T)
                if amount < .04:
                    raise ValueError(f'Only {amount:.3f} training overlap within 4 mm')
                solutions[m][a,b] = (T,amount)
                records[m].append(dict(source=a,target=b,overlap4=amount,elapsed_s=perf_counter()-ts,
                                       actual_transform=T,status='accepted'))
            except (RuntimeError,ValueError,cv.error,subprocess.SubprocessError,FileNotFoundError) as exc:
                records[m].append(dict(source=a,target=b,error=str(exc),elapsed_s=perf_counter()-ts,
                                       status='fallback_to_R02'))
    return solutions,records


def sequence(edges,base):
    T = [np.eye(4)]
    for i in range(len(base)-1):
        e = (i,i+1)
        left_to_right = edges[e][0] if e in edges else rel(base,*e)
        T.append(T[-1] @ np.linalg.inv(left_to_right))
    return np.array(T)


def cyclic_correspondences(v,base):
    result=[]
    for a,b in ring(len(v)):
        P,Q = v[a]['train'][::4],v[b]['train'][::4]
        W = transform(P,rel(base,a,b))
        d,ids = cKDTree(Q).query(W)
        _,reverse = cKDTree(W).query(Q)
        keep = (d<4.5) & (reverse[ids]==np.arange(len(P)))
        if keep.sum()>=12:
            result.append((a,b,P[keep],Q[ids[keep]]))
    return result


def turntable_fit(v,raw,axis,center,phi,base,model,planes=False):
    corr = cyclic_correspondences(v,base)
    B = np.linalg.svd(axis[None,:])[2][1:].T
    n = len(v)
    surface = []
    if planes:
        world = [transform(d['train'][::2],T) for d,T in zip(v,base)]
        merged = np.concatenate(world)
        for _ in range(3):
            if len(merged)<130:
                break
            eq, ids = cloud(merged).segment_plane(distance_threshold=1.3,ransac_n=3,num_iterations=450)
            if len(ids)<115:
                break
            normal, offset = np.array(eq[:3]),eq[3]
            if abs(normal @ axis)<.94:
                for i,(p,P) in enumerate(zip(v,world)):
                    take = abs(P@normal+offset)<2.0
                    if take.sum()>=22:
                        surface.append((i,p['train'][::2][take],normal,offset))
            merged = np.delete(merged,ids,axis=0)
        if len(surface)<3:
            return None

    def matrices(x):
        pivot = center + B @ x[:2]
        return axial(axis,pivot,-(phi+np.r_[0.,x[2:]/50]))

    def err(x):
        T = matrices(x)
        res = [x*.8]
        for a,b,P,Q in corr:
            res.append(((transform(P,T[a])-transform(Q,T[b]))*(3/np.sqrt(len(P)))).ravel())
        for i,P,normal,offset in surface:
            res.append((transform(P,T[i])@normal+offset)*(2/np.sqrt(len(P))))
        for i,(obs,d) in enumerate(zip(raw,T)):
            board = transform(transform(obs['board_xyz'],raw[0]['pose']),np.linalg.inv(d))
            res.append(((project(board,model)-obs['board_uv'])*(.8/np.sqrt(len(board)))).ravel())
        return np.concatenate(res)

    bounds = np.r_[2.,2.,np.full(n-1,1.4)]
    result = least_squares(err,np.zeros(n+1),bounds=(-bounds,bounds),loss='soft_l1',f_scale=.35,max_nfev=17)
    return matrices(result.x)


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


def prefer(rows):
    # E003 gates apply against physical initialization. Compare absolute validation afterward.
    baseline = rows['R02']
    def compatible(code,row):
        return (row['max_gap'] <= baseline['max_gap'] + 2.5 and
                row['tag_P50'] <= baseline['tag_P50'] + 1 and
                row['depth_supported'] >= baseline['depth_supported'] - .025 and
                row['F1_2'] >= baseline['F1_2'] - .015 and
                row['depth_abs_P50'] is not None and
                row['depth_abs_P50'] < .97 * baseline['depth_abs_P50'] and
                (row['independently_solved_edges'] in (None,baseline['edge_count']) or code=='R06'))
    candidates = [code for code,row in rows.items() if compatible(code,row)]
    key = lambda code: (rows[code]['depth_abs_P50'],-rows[code]['F1_2'],rows[code]['tag_P50'])
    return min(candidates,key=key) if candidates else 'R02',candidates


def benchmark(scan, branch, cc, go, teaser, supplement=False):
    import json
    path = directory(scan)/branch
    meta = json.loads((path/'poses.json').read_text())
    data = [dict(np.load(f)) for f in sorted(path.glob('*.npz'))]
    o = HERE/branch/scan.name.split('_',4)[4]
    scratch = CACHE/'solves'/branch/scan.name[14:20]/str(os.getpid())
    model = read_models()[branch]
    axis,center,phi,cmd,pnp = [np.asarray(meta[k]) for k in ['axis','center','phi','cmd','pnp']]
    base = axial(axis,center,-phi)
    views = [sample_view(d) for d in data]
    prepared = [prepare_pair(v['train'],CONFIG,v['gray']) for v in views]
    n = len(data)
    edges, records = solve_edges(views,prepared,sorted(scan.glob('view_*')),base,scratch,cc,go,teaser,
                                 ['R05','R18'] if supplement else None)
    if supplement:
        result = json.loads((o/'结果.json').read_text())
        for code,pairs in edges.items():
            result['edge_solver'][code] = records[code]
            if not pairs:
                result['missing'][code] = 'all pairs failed'
                continue
            T = sequence(pairs,base)
            row,detail = summarize(views,data,T,base,axis,center,model,pairs)
            row.update(method=METHODS[code],fallback_edges=len(data)-len(pairs))
            result['metrics'][code] = row
            result['matrices'][code] = T
            result['per_edge_metrics'][code] = detail
            result['missing'].pop(code,None)
            result['full_cloud_vertices'][code] = write_scene(o/'整圈'/f'{code}.ply',data,T,range(n))
        old = result['chosen']
        result['chosen'],result['eligible'] = prefer(result['metrics'])
        if result['chosen'] != old:
            chosen = result['chosen']
            T = np.array(result['matrices'][chosen])
            write_scene(o/'整圈'/'推荐.ply',data,T,range(n))
            write_scene(o/'双视角'/f'{chosen}_V00-V01.ply',data[:2],T[:2],range(2))
            (o/'双视角'/f'{old}_V00-V01.ply').unlink()
            result['thinning'][chosen] = thinning_metrics(chosen,views,data,T,base,axis,center,model)
        store(o,result)
        print('supplement', branch, o.name, result['chosen'], {m:len(x) for m,x in edges.items()},flush=True)
        return
    poses = dict(R01=axial(axis,center,-cmd),R02=base,R03=pnp)
    for code,pairs in edges.items():
        if pairs:
            poses[code] = sequence(pairs,base)
    graph, accepted = incremental(views,base,edges['R10'],center)
    poses['R06'] = graph
    if '三棱锥' in meta['name']:
        poses['R07'] = turntable_fit(views,data,axis,center,phi,base,model,planes=True)
    poses['R19'] = turntable_fit(views,data,axis,center,phi,base,model)
    rows, missing, per_edge, counts, thin = {}, {}, {}, {}, {}
    all_points = [{k:d[k] for k in ('xyz','gray')} for d in data]
    for code,label in METHODS.items():
        T = poses.get(code)
        if T is None:
            missing[code] = ('only for pyramids' if code=='R07' and '三棱锥' not in meta['name'] else
                             'all pairs failed' if code in records else 'solver unavailable')
            continue
        rows[code],per_edge[code] = summarize(views,data,T,base,axis,center,model,edges.get(code))
        rows[code].update(method=label, fallback_edges=sum(x['status']!='accepted' for x in records.get(code,[])))
        if code=='R06':
            rows[code]['graph_admitted_edges'] = accepted
        counts[code] = write_scene(o/'整圈'/f'{code}.ply',all_points,T,range(n))
    chosen, eligible = prefer(rows)
    # Fixed-pose thinning preserves the same input and fit. Both interlaced subsets are evaluated.
    for code in ['R01','R02','R03','R06',chosen]:
        thin[code] = thinning_metrics(code,views,data,poses[code],base,axis,center,model)
    write_scene(o/'整圈'/'推荐.ply',all_points,poses[chosen],range(n))
    write_scene(o/'双视角'/f'{chosen}_V00-V01.ply',all_points[:2],poses[chosen][:2],range(2))
    store(o,dict(chosen=chosen,eligible=eligible,metrics=rows,missing=missing,
                 per_edge_metrics=per_edge,edge_solver=records,matrices=poses,thinning=thin,
                 full_cloud_vertices=counts,n_views=n))
    print('complete',branch,meta['name'],chosen,len(rows),rows[chosen],missing,flush=True)


def thinning_metrics(code,views,data,poses,base,axis,center,model):
    result = []
    for phase in range(2):
        ix = np.arange(phase,len(views),2)
        row,_ = summarize([views[i] for i in ix],[data[i] for i in ix],poses[ix],base[ix],axis,center,model)
        result.append({k:row[k] for k in ('F1_2','worst_F1_2','max_gap','depth_abs_P50','depth_supported')})
    return result


def store(out, result):
    rows = result['metrics']
    dump(out/'结果.json',result)
    fields = sorted(set().union(*(row.keys() for row in rows.values())))
    with (out/'指标.csv').open('w',encoding='utf-8-sig',newline='') as file:
        writer = csv.DictWriter(file,fieldnames=['code',*fields])
        writer.writeheader()
        writer.writerows(dict(code=k,**rows[k]) for k in METHODS if k in rows)


if __name__=='__main__':
    args=argparse.ArgumentParser()
    args.add_argument('branch',choices=BRANCHES)
    args.add_argument('scan')
    args.add_argument('--cloudcompare'); args.add_argument('--go-icp'); args.add_argument('--teaser')
    args.add_argument('--supplement',action='store_true')
    p=args.parse_args()
    cv.ocl.setUseOpenCL(False)
    cv.setNumThreads(1)
    cv.setRNGSeed(101)
    o3d.utility.random.seed(101)
    benchmark(next(s for s in SCANS if p.scan in s.name),p.branch,p.cloudcompare,p.go_icp,p.teaser,p.supplement)
