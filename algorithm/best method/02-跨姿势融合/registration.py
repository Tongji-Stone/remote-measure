"""E203 multi-hypothesis rigid registration and anchored three-placement graph."""
from itertools import permutations, product
from time import perf_counter

import numpy as np
import open3d as o3d
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from rgb_matches import rgb_seeds
from scan_io import transform

reg = o3d.pipelines.registration
STAGES = ((1.8, 5.2, 38), (.95, 3.0, 31), (.65, 2.15, 30))


def pca(x):
    mean=np.median(x,axis=0)
    _,_,directions=np.linalg.svd(x-mean,full_matrices=False)
    b=directions.T
    b[:,-1]*=np.sign(np.linalg.det(b))
    return mean,b


def sequence(source,target,inverted):
    a,A=pca(source);b,B=pca(target)
    for idx in permutations(range(3)):
        for signs in product((-1.,1.),repeat=3):
            R=B@np.eye(3)[:,idx]@np.diag(signs)@A.T
            if np.linalg.det(R)<0 or (inverted and R[2,2]>-.32):
                continue
            T=np.eye(4)
            T[:3,:3],T[:3,3]=R,b-R@a
            yield T
    if inverted:
        for yaw in np.arange(0,360,30):
            R=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@np.diag([1.,-1.,-1.])
            T=np.eye(4);T[:3,:3]=R;T[:3,3]=b-R@a
            yield T


def make(points, rgb, voxel):
    p = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    p.colors = o3d.utility.Vector3dVector(rgb)
    p = p.voxel_down_sample(voxel)
    p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=max(2.6, 2.7*voxel), max_nn=36))
    return p


def split(scan):
    base = make(scan['xyz'], scan['rgb']/255., .64)
    p, c, n = map(np.asarray, (base.points, base.colors, base.normals))
    i = np.floor(p / 3.3).astype('int64')
    k = (i[:, 0]*73856093 ^ i[:, 1]*19349663 ^ i[:, 2]*83492791) % 10
    s = dict(points=p, rgb=c, normals=n, tree=cKDTree(p))
    s['mask'] = dict(fit=k>=4, select=(k>=2)&(k<4), test=k<2)
    s['levels'] = [make(p[k>=4], c[k>=4], voxel) for voxel, _, _ in STAGES]
    return s


def one_way(source, target, m, fold):
    k = source['mask'][fold]
    p = transform(source['points'][k], m)
    d, j = target['tree'].query(p)
    delta = p - target['points'][j]
    normal = target['normals'][j]
    seam = np.abs(np.einsum('ij,ij->i', delta, normal))
    ln = source['normals'][k] @ m[:3, :3].T
    use = (d < 2.4) & (np.abs(np.einsum('ij,ij->i', normal, ln)) > .65)
    a, b = source['rgb'][k][use], target['rgb'][j[use]]
    luma = np.mean(a-b, axis=1)
    chroma = np.mean(abs((a-b) - luma[:, None]), axis=1)
    return d, seam[use], chroma


def evaluate(s, t, m, fold, matches):
    ds, se, ce = one_way(s, t, m, fold)
    dd, sf, cf = one_way(t, s, np.linalg.inv(m), fold)
    distance = np.r_[ds, dd]
    seam = np.r_[se, sf]
    hue = np.r_[ce, cf]
    row = dict()
    for cutoff in (1, 2, 3):
        a, b = np.mean(ds<cutoff), np.mean(dd<cutoff)
        row[f'f1_{cutoff}'] = 2*a*b / max(a+b, 1e-6)
        if cutoff == 2:
            row.update(src_to_dst_2=a, dst_to_src_2=b)
    row.update(p50=np.median(distance), p90=np.percentile(distance, 90),
               seam_p95=np.percentile(seam, 95) if len(seam) else 3.,
               chroma_median=np.median(hue) if len(hue) else 1.)
    x, y, info = matches
    id_fold = dict(select=1, test=0)[fold]
    keep = (info[:, 0]*7 + info[:, 1]*3) % 5 == id_fold
    a = np.linalg.norm(transform(x[keep], m)-y[keep], axis=1)
    row.update(photo_matches=len(a), photo_inliers=int(np.sum(a<4.0)),
               photo_inlier_p50=np.median(a[a<4.0]) if np.any(a<4.) else None)
    return row


def rank(row):
    photo_bonus = .035*np.log1p(min(row['photo_inliers'], 60)) / np.log(61)
    return (.26*row['f1_1'] + .50*row['f1_2'] + .15*row['f1_3']
            - .07*row['seam_p95'] - .12*row['chroma_median'] + photo_bonus)


def distance_pose(A, B):
    return Rotation.from_matrix(A[:3, :3]@B[:3, :3].T).magnitude()


def pca_candidates(s, t, inverted):
    a = np.asarray(s['levels'][0].points)
    b = np.asarray(t['levels'][0].points)
    init = list(sequence(a, b, inverted))
    if inverted:
        for angle in range(0, 360, 10):
            R = Rotation.from_euler('z', angle, degrees=True).as_matrix() @ np.diag([1., -1., -1.])
            m = np.eye(4)
            m[:3, :3] = R
            m[:3, 3] = np.median(b, axis=0)-np.median(a, axis=0)@R.T
            init.append(m)
        # Each yaw is tried; a flat-lobed object can have several similar coarse optima.
        return init
    tree = cKDTree(b)
    return sorted(init, key=lambda p: np.median(tree.query(transform(a, p))[0]))[:12]


def fpfh_seeds(s, t):
    a = s['levels'][0]
    b = t['levels'][0]
    search = o3d.geometry.KDTreeSearchParamHybrid(radius=8.8, max_nn=100)
    fa, fb = (reg.compute_fpfh_feature(p, search) for p in (a, b))
    opt = reg.FastGlobalRegistrationOption(maximum_correspondence_distance=3.4, iteration_number=100)
    seeds = [('FPFH-FGR', reg.registration_fgr_based_on_feature_matching(a,b,fa,fb,opt).transformation)]
    for k in range(4):
        o3d.utility.random.seed(203+k)
        m = reg.registration_ransac_based_on_feature_matching(
            a,b,fa,fb,True,3.4,reg.TransformationEstimationPointToPoint(False),3,
            [reg.CorrespondenceCheckerBasedOnEdgeLength(.85),reg.CorrespondenceCheckerBasedOnDistance(3.4)],
            reg.RANSACConvergenceCriteria(35000,.999))
        if m.fitness > .08:
            seeds.append(('FPFH-RANSAC',m.transformation))
    return seeds


def refined(s, t, pose, mode, stages=(0,1,2)):
    for k in stages:
        _, d, n = STAGES[k]
        a, b = s['levels'][k], t['levels'][k]
        term = reg.ICPConvergenceCriteria(relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=n)
        if mode == 'GICP':
            output = reg.registration_generalized_icp(a,b,d,pose,
                         reg.TransformationEstimationForGeneralizedICP(1e-3,reg.TukeyLoss(d)),term)
        elif mode == 'Colored ICP':
            output = reg.registration_colored_icp(a,b,d,pose,
                         reg.TransformationEstimationForColoredICP(.95,reg.TukeyLoss(d)),term)
        else:
            output = reg.registration_icp(a,b,d,pose,
                         reg.TransformationEstimationPointToPlane(reg.TukeyLoss(d*.72)),term)
        if output.fitness > .025 and np.all(np.isfinite(output.transformation)):
            pose = output.transformation
    return pose


def compare(s, t, inverted, matches):
    """Fit several initial poses, rank on selection tiles and score held-out tiles."""
    pool = []
    failures = []
    def trial(method, seed, mode, stages=(0,1,2)):
        start = perf_counter()
        try:
            m = refined(s, t, seed, mode, stages)
        except RuntimeError as exc:
            failures.append(dict(method=method, message=str(exc).splitlines()[-1]))
            return
        val = evaluate(s,t,m,'select',matches)
        pool.append(dict(method=method, pose=m, selection=val, score=rank(val),
                         compatible=bool(not inverted or m[2,2]<-.55), seconds=perf_counter()-start))

    seeds = pca_candidates(s,t,inverted)
    for seed in seeds:
        trial('PCA/翻面搜索 + Tukey ICP',seed,'Tukey')
    for method, seed in fpfh_seeds(s,t):
        trial(method+' + Tukey ICP',seed,'Tukey')
    photo, _ = rgb_seeds(*matches,inverted)
    for seed in photo:
        trial('彩照 SIFT + Tukey ICP',seed,'Tukey')
    initial = sorted((p for p in pool if p['compatible']), key=lambda p:-p['score'])
    distinct = []
    for x in initial:
        if all(distance_pose(x['pose'], y['pose']) > np.deg2rad(8) for y in distinct):
            distinct.append(x)
    for x in distinct[:6]:
        trial('多候选 + GICP',x['pose'],'GICP',(1,2))
        trial('多候选 + Colored ICP',x['pose'],'Colored ICP',(1,2))
    pool.sort(key=lambda p:-p['score'])
    for p in pool:
        p['holdout'] = evaluate(s,t,p['pose'],'test',matches)
    return pool, failures


def relative_distance(a,b, p):
    angle = np.rad2deg(distance_pose(a,b))
    return dict(angle_deg=angle, displacement_mm=np.linalg.norm(transform(p,a)-transform(p,b),axis=1).mean())


def delta(d, center):
    R = Rotation.from_rotvec(d[:3]).as_matrix()
    h = np.eye(4)
    h[:3,:3],h[:3,3] = R, center-R@center+d[3:]
    return h


def graph(nodes, pair, matches):
    """Jointly update the two movable poses, using three edges with trimmed correspondences."""
    order = [(0,1),(0,2),(1,2)]
    start = [np.eye(4), pair[0]['pose'], pair[1]['pose']]
    poses = start.copy()
    cent = np.mean(nodes[0]['points'], axis=0)
    p = [n['points'][n['mask']['fit']] for n in nodes]
    ns = [n['normals'][n['mask']['fit']] for n in nodes]
    for _ in range(5):
        x = [transform(a,m) for a,m in zip(p,poses)]
        nor = [n@m[:3,:3].T for n,m in zip(ns,poses)]
        jobs = []
        for (j,i) in order:
            d,k = cKDTree(x[j]).query(x[i])
            d2,k2 = cKDTree(x[i]).query(x[j])
            idx = np.arange(len(d))
            keep = (d<2.5)&(k2[k]==idx)
            keep &= (abs(np.einsum('ij,ij->i',nor[i],nor[j][k]))>.72)
            a = idx[keep][::2]
            b = k[a]
            if len(a)<60:
                continue
            jobs.append((j,i,x[i][a],x[j][b],nor[j][b]))
        if len(jobs)<3:
            break
        def objective(w):
            U = [np.eye(4),delta(w[:6],cent),delta(w[6:],cent)]
            res = []
            for j,i,a,b,n in jobs:
                e = (transform(a,U[i])-transform(b,U[j]))
                res.append(np.einsum('ij,ij->i',e,n)*np.sqrt(800/len(a)))
                res.append(.24*e.ravel()*np.sqrt(800/len(a)))
            prior = []
            for i in (1,2):
                now=U[i]@poses[i]
                prior.extend(4*(transform(cent[None],now)-transform(cent[None],start[i])).ravel())
                prior.extend(30*Rotation.from_matrix(now[:3,:3]@start[i][:3,:3].T).as_rotvec())
            return np.r_[np.concatenate(res),prior]
        w = least_squares(objective,np.zeros(12),loss='soft_l1',f_scale=.7,max_nfev=22).x
        poses = [np.eye(4),delta(w[:6],cent)@poses[1],delta(w[6:],cent)@poses[2]]
    base = [evaluate(nodes[i],nodes[j],np.linalg.inv(start[j])@start[i],'select',m)
            for (j,i),m in zip(order,matches)]
    new = [evaluate(nodes[i],nodes[j],np.linalg.inv(poses[j])@poses[i],'select',m)
           for (j,i),m in zip(order,matches)]
    accept = np.mean([rank(r) for r in new]) > np.mean([rank(r) for r in base]) and all(
        n['f1_2'] >= b['f1_2']-.012 for n,b in zip(new, base))
    hold = lambda pp: [evaluate(nodes[i],nodes[j],np.linalg.inv(pp[j])@pp[i],'test',m)
                        for (j,i),m in zip(order,matches)]
    diagnostics = dict(accepted=bool(accept), before_selection=base, candidate_selection=new,
                       before_holdout=hold(start), candidate_holdout=hold(poses), candidate=poses,
                       cycle=relative_distance(np.linalg.inv(start[1])@start[2],
                                               pair[2]['pose'],nodes[2]['points'][::11]))
    return (poses if accept else start), diagnostics
