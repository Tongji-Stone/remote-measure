"""Calibrate the post-17:41 turntable and recover a shifted projector using its plane."""
import json
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from prepare_calibrated_benchmark import CALIBRATION, OUTPUT, ROOT, SCANS, decode, load_calibration, triangulate
from stitch_turntable import axis_transform
from registration_experiment import training_mask

TAG_REFERENCE = np.array([[-10,10,0], [10,10,0], [10,-10,0], [-10,-10,0]], float)
PROJECTOR_TRAIN = (1, 2, 4, 5)
TRAIN_AXIS_VIEWS = (0, 2, 4, 6)
# Bounds in units of 1/10 camera resolution, taken from the white-reference images.
ROI = {
    "171640": [(69,97,130,141)]*8,
    "174129": [(70,97,126,142)]*8,
    "174531": [(66,82,134,137)]*8,
    "174747": [(44,87,113,158),(35,87,91,145),(37,82,83,132),(56,79,111,122),
               (81,80,131,122),(99,80,153,131),(105,81,162,146),(83,93,154,158)],
    "175006": [(65,80,138,136)]*8,
}


def estimate_pose(xyz, uv, cal):
    K, D = cal["cam_int"], cal["cam_dist"]
    ok, r, t = cv2.solvePnP(np.ascontiguousarray(xyz, dtype=float),
                             np.ascontiguousarray(uv, dtype=float), K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise ValueError("PnP failed")
    pose = np.eye(4)
    pose[:3,:3] = cv2.Rodrigues(r)[0]
    pose[:3,3] = t.ravel()
    return pose


def image_plane(pixels, pose, cal):
    rays = np.c_[cv2.undistortPoints(np.ascontiguousarray(pixels, dtype=float), cal["cam_int"],
                                    cal["cam_dist"]).reshape(-1,2), np.ones(len(pixels))]
    normal = pose[:3,2]
    return rays * ((normal @ pose[:3,3]) / (rays @ normal))[:,None]


def project(xyz, pose, cal):
    return cv2.projectPoints(np.ascontiguousarray(xyz), pose[:3,:3], pose[:3,3],
                             cal["cam_int"], cal["cam_dist"])[0][:,0,:]


def transform(xyz, T):
    return xyz @ T[:3,:3].T + T[:3,3]


def rig_poses(meta, cal):
    tags = [{int(k):np.array(v) for k,v in d["tags"].items()} for d in meta["views"]]
    candidates = set.intersection(*(set(t) for t in tags[:1]))
    holdout = max(candidates, key=lambda k: (sum(k in t for t in tags[1:]),k))
    anchor = min(k for k in tags[0] if k != holdout)
    p0 = estimate_pose(TAG_REFERENCE, tags[0][anchor], cal)
    layout = {k:transform(image_plane(v,p0,cal), np.linalg.inv(p0)) for k,v in tags[0].items()}
    for xy in layout.values():
        xy[:,2] = 0
    poses = [p0]
    # Build missing IDs from observed joint markers over a few views. Held-out ID is fixed by view 0 only.
    for _ in range(2):
        for t in tags[1:]:
            shared = sorted((layout.keys() & t.keys()) - {holdout})
            if not shared:
                continue
            p = estimate_pose(np.concatenate([layout[k] for k in shared]), np.concatenate([t[k] for k in shared]),cal)
            for k in t.keys() - layout.keys():
                layout[k] = transform(image_plane(t[k],p,cal), np.linalg.inv(p))
                layout[k][:,2] = 0
    for t in tags[1:]:
        shared = sorted((layout.keys() & t.keys()) - {holdout})
        poses.append(estimate_pose(np.concatenate([layout[k] for k in shared]),np.concatenate([t[k] for k in shared]),cal))
    # Joint planar board fit from training IDs only, metric scale fixed by the anchor square.
    fit_ids = sorted(set(layout)-{anchor,holdout})
    def extract(x):
        board = {**layout, **{k:np.c_[x[8*j:8*j+8].reshape(4,2), np.zeros(4)] for j,k in enumerate(fit_ids)}}
        ps = []
        for r in x[8*len(fit_ids):].reshape(-1,6):
            p = np.eye(4); p[:3,:3] = Rotation.from_rotvec(r[:3]).as_matrix(); p[:3,3] = r[3:]; ps.append(p)
        return board, ps

    def loss(x):
        board, ps = extract(x)
        return np.concatenate([(project(board[k], p, cal) - t[k]).ravel() for t,p in zip(tags,ps)
                               for k in sorted(t) if k != holdout and k in board])
    initial = np.r_[np.concatenate([layout[k][:,:2].ravel() for k in fit_ids]),
                    np.concatenate([np.r_[Rotation.from_matrix(p[:3,:3]).as_rotvec(),p[:3,3]] for p in poses])]
    layout, poses = extract(least_squares(loss,initial,loss="soft_l1",f_scale=1.5,max_nfev=90).x)
    layout[holdout] = transform(image_plane(tags[0][holdout],poses[0],cal), np.linalg.inv(poses[0]))
    layout[holdout][:,2] = 0
    to_zero = np.array([poses[0] @ np.linalg.inv(p) for p in poses])
    fit_errors = [np.linalg.norm(project(layout[k],p,cal)-t[k],axis=1).tolist()
                  for t,p in zip(tags,poses) for k in sorted(t) if k != holdout and k in layout]
    ref_xyz = transform(layout[holdout],poses[0])
    heldout = {i:t[holdout].tolist() for i,t in enumerate(tags) if i and holdout in t}
    return poses, to_zero, ref_xyz, holdout, heldout, np.ravel(fit_errors)


def roi_mask(uv, sid, i, grow=0):
    a,b,c,d = np.array(ROI[sid][i])*10
    return (uv[:,0]>=a-grow)&(uv[:,0]<c+grow)&(uv[:,1]>=b-grow)&(uv[:,1]<d+grow)


def table_samples(uv, sid, i):
    # Pixels well away from the manually picked silhouette and the outer rim.
    u,v = uv.T
    disk = ((u-1090)/610)**2 + ((v-1300)/400)**2 < .93
    return disk & ~roi_mask(uv,sid,i,40)


def fit_projector(planes, samples, cal):
    source, image = [], []
    for i in PROJECTOR_TRAIN:
        uv, st = samples[i]
        keep = table_samples(uv,"174129",i) & training_mask(uv)
        source.append(image_plane(uv[keep],planes[i],cal)[::5])
        image.append(st[keep][::5])
    xyz, uv = np.concatenate(source), np.concatenate(image)
    ok, r, t, inlier = cv2.solvePnPRansac(xyz,uv,cal["proj_int"],cal["proj_dist"],
                                         iterationsCount=750,reprojectionError=6,confidence=.999)
    if not ok:
        raise RuntimeError("Unable to recover projector pose")
    def residual(arg):
        pred = cv2.projectPoints(xyz[inlier.ravel()],arg[:3],arg[3:],cal["proj_int"],cal["proj_dist"])[0][:,0]
        return (pred-uv[inlier.ravel()]).ravel()
    result = least_squares(residual,np.r_[r.ravel(),t.ravel()],loss="soft_l1",f_scale=2)
    corrected = {**cal, "rotation":cv2.Rodrigues(result.x[:3])[0],"translation":result.x[3:,None]}
    return corrected, {"samples":len(xyz),"inliers":len(inlier),"median_px":float(np.median(np.linalg.norm(residual(result.x).reshape(-1,2),axis=1)))}



def refine_projector(initial, planes, decoded, cal):
    """Fit projector pose/intrinsics to metric plane + object epipolar observations."""
    xyz_table, st_table, rays_obj, st_obj = [], [], [], []
    for i in PROJECTOR_TRAIN:
        uv, st, _, _ = decoded[i]
        table = table_samples(uv,"174129",i) & training_mask(uv)
        cam = image_plane(uv[table],planes[i],cal)
        expected = cv2.projectPoints(cam,initial["rotation"],initial["translation"],
                                     initial["proj_int"],initial["proj_dist"])[0][:,0]
        clean = np.linalg.norm(expected - st[table],axis=1) < 9
        xyz_table.append(cam[clean][::16]); st_table.append(st[table][clean][::16])
        xyz, e = triangulate(initial, uv, st)
        normal = planes[i][:3,2].copy()
        normal *= -1 if normal[2] > 0 else 1
        h = (xyz-planes[i][:3,3]) @ normal
        mask = training_mask(uv) & roi_mask(uv,"174129",i) & (h>8) & (h<45) & (e.max(1)<40)
        rays = cv2.undistortPoints(uv[mask],cal["cam_int"],cal["cam_dist"]).reshape(-1,2)
        rays_obj.append(np.c_[rays,np.ones(len(rays))][::4]); st_obj.append(st[mask][::4])
    xyz_table, st_table, rays_obj, st_obj = [np.concatenate(m) for m in (xyz_table,st_table,rays_obj,st_obj)]
    L0 = initial["proj_int"]
    D = initial["proj_dist"]
    def unpack(x):
        L=L0.copy()
        L[0,0]*=np.exp(x[6]); L[1,1]*=np.exp(x[7])
        L[:2,2]+=x[8:10]*200
        return x[:3],x[3:6],L
    def residual(x):
        r,t,L=unpack(x)
        plane=cv2.projectPoints(xyz_table,r,t,L,D)[0][:,0]-st_table
        predicted_line=np.cross(rays_obj @ cv2.Rodrigues(r)[0].T, t)
        proj=cv2.undistortPoints(st_obj,L,D).reshape(-1,2)
        epi=(np.sum(proj*predicted_line[:,:2],axis=1)+predicted_line[:,2])
        epi*=L[0,0]/np.maximum(np.linalg.norm(predicted_line[:,:2],axis=1),1e-6)
        return np.r_[plane.ravel(),epi*2, x[6:10]*np.array([15,15,8,8])]
    start=np.r_[Rotation.from_matrix(initial["rotation"]).as_rotvec(), initial["translation"].ravel(), np.zeros(4)]
    low=np.r_[np.full(6,-np.inf),[-.4,-.4,-4.,-2.]]
    high=np.r_[np.full(6,np.inf),[.4,.4,4.,2.]]
    x=least_squares(residual,start,loss="soft_l1",f_scale=2,max_nfev=150,bounds=(low,high)).x
    r,t,L=unpack(x)
    offset=2*len(xyz_table)
    size=len(st_obj)
    details={"plane_samples":len(xyz_table),"object_samples":size,
             "plane_p50_px":[float(np.median(np.linalg.norm(residual(k)[:offset].reshape(-1,2),axis=1))) for k in (start,x)],
             "object_epipolar_p50_px":[float(np.median(np.abs(residual(k)[offset:offset+size]))/2) for k in (start,x)]}
    return {**initial,"rotation":cv2.Rodrigues(r)[0],"translation":t[:,None],"proj_int":L},details


def fit_axis(prior, angles, first):
    ids = list(TRAIN_AXIS_VIEWS[1:])
    normal = -first[:3,2]
    signed = Rotation.from_matrix(prior[1:4,:3,:3]).as_rotvec() @ normal
    normal *= 1 if np.median(signed)>0 else -1
    center = first[:3,3]
    norm = np.linalg.norm
    # Rotations are dimensionless; weight them in object-space millimetres at radius 55.
    def evaluate(x):
        u = x[:3]/norm(x[:3])
        c = x[3:6]
        delta = []
        for j in ids:
            transform = axis_transform(u,c,x[6+ids.index(j)])
            delta.extend(Rotation.from_matrix(transform[:3,:3] @ prior[j,:3,:3].T).as_rotvec()*55)
            delta.extend(transform[:3,3]-prior[j,:3,3])
        delta.extend([(c-center) @ first[:3,2],(norm(x[:3])-1)*55])
        return delta
    rotation_vectors = Rotation.from_matrix(prior[ids,:3,:3]).as_rotvec()
    guess = np.rad2deg(rotation_vectors @ normal)
    a = least_squares(evaluate,np.r_[normal,center,guess],loss="soft_l1",f_scale=2,max_nfev=100).x
    return a[:3]/norm(a[:3]), a[3:6]


def serial(value):
    return value.tolist() if isinstance(value,np.ndarray) else value.item()


def main():
    cv2.setNumThreads(2)
    cv2.setRNGSeed(23)
    raw = OUTPUT/"raw"
    dst = OUTPUT/"recovery"
    dst.mkdir(exist_ok=True)
    cal = load_calibration(CALIBRATION)
    manifest = json.loads((raw/"manifest.json").read_text())
    models = {}
    shared = None
    for sid, meta in manifest.items():
        if sid == "171640":
            continue
        begin = perf_counter()
        scan = ROOT/meta["scan"]
        planes, prior, ref_xyz, holdout, heldout, errors = rig_poses(meta,cal)
        axis, center = fit_axis(prior, [d["command_deg"] for d in meta["views"]], planes[0])
        decoded = [decode(v,recover_msb=True) for v in sorted(scan.glob("view_*"))]
        if shared is None:
            shared, stats = fit_projector(planes,[(u,s) for u,s,_,_ in decoded],cal)
            shared, refinement = refine_projector(shared,planes,decoded,cal)
            normalized = {**shared, "training_views":PROJECTOR_TRAIN, "train":stats, "refinement":refinement, "scan":sid}
            (dst/"projector_recovery.json").write_text(json.dumps(normalized,default=serial,indent=2))
        folder = dst/sid
        folder.mkdir(exist_ok=True)
        stats = []
        for i,(uv, st, gray, _) in enumerate(decoded):
            xyz, e = triangulate(shared,uv,st)
            a, f = triangulate(cal,uv,st)
            table = table_samples(uv,sid,i) & ~training_mask(uv)
            xyz_table = image_plane(uv[table],planes[i],cal)
            before_after = []
            for cm in (cal,shared):
                st_pred = cv2.projectPoints(xyz_table,cm["rotation"],cm["translation"],cm["proj_int"],cm["proj_dist"])[0][:,0]
                before_after.append(np.percentile(np.linalg.norm(st_pred-st[table],axis=1),[50,95]).tolist())
            normal = planes[i][:3,2]
            if normal[2]>0:
                normal=-normal
            xyz_plane = planes[i][:3,3]
            h = (xyz-xyz_plane) @ normal
            keep = np.isfinite(xyz).all(1) & (xyz[:,2]>110) & (xyz[:,2]<650) & (e.max(1)<8)
            obj = keep & roi_mask(uv,sid,i,40) & (h>4) & (h<(62 if sid=="174129" else 118))
            np.savez_compressed(folder/f"{i:02}.npz",xyz=xyz.astype(np.float32),uv=uv.astype(np.uint16),gray=gray,
                                valid=keep,obj=obj,height=h.astype(np.float32),st=st.astype(np.float32),error=e.astype(np.float32))
            stats.append({"view":i,"archive_valid":int(((a[:,2]>110)&(a[:,2]<650)&(f.max(1)<8)).sum()),
                          "recovered_valid":int(keep.sum()),"object":int(obj.sum()),"table_reprojection_p50_p95":before_after,
                          "object_depth_p50":float(np.median(xyz[obj,2])) if obj.any() else None})
            print(sid,i,stats[-1], flush=True)
        models[sid] = {"scan":meta["scan"],"axis_fit_views":TRAIN_AXIS_VIEWS,"axis":axis,"center":center,
                       "planes":planes,"prior":prior,"holdout_xyz":ref_xyz,"heldout_id":holdout,"heldout_image":heldout,
                       "tag_train_reprojection_p50":np.median(errors),"tag_train_reprojection_p95":np.percentile(errors,95),
                       "statistics":stats,"preparation_s":perf_counter()-begin}
        (dst/"model.json").write_text(json.dumps(models,default=serial,ensure_ascii=False,indent=2))


if __name__ == "__main__":
    main()
