"""Estimate rotation from turntable evidence and export every valid scene sample."""
import csv
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np
import open3d as o3d
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from optics import (
    CACHE, OUT, CAL16, CAL20, CALIBRATION, DATA, ROOT, case_inputs, configure, dump, image_plane,
    load, load_calibration, project, roi_mask, table_samples, transform, with_arrays,
)
from calibrate_benchmark_turntable import fit_axis
from calibrated_registration_benchmark import CONFIG, adjacent_from, cloud_sample, pair_metric, training_support
from nov20_object_benchmark import LEGACY_BOX, MINION, rectangle
from registration_experiment import fit_pair, polygon_mask, prepare_pair, training_mask
from stitch_turntable import axis_transform
from box_planes import estimate as rectangular_pose

NATIVE=ROOT/"output/object-benchmarks"
BASELINE=ROOT/"output/calibrated-benchmark"
OUTPUTS=OUT/"clouds"
VIEW_COLORS=np.array([[47,111,196],[227,117,56],[45,162,112],[147,89,174],
                      [214,166,57],[34,158,183],[212,87,124],[105,119,145]])/255.
IDENTITY=np.eye(4)


def basis(n):
    d=np.array([1.,0.,0.]) - n[0]*n
    d/=np.linalg.norm(d)
    return np.column_stack((d,np.cross(n,d)))


def axis_angles(poses,axis,expected):
    r=Rotation.from_matrix(np.asarray(poses)[:,:3,:3]).as_rotvec()@axis
    a=np.rad2deg(r)
    return a+360*np.round((expected-a)/360)


def stage_poses(center,axis,angles):
    return np.array([axis_transform(axis,center,a) for a in angles])


def image_axis(meta,cal):
    nominal=np.array(meta["angles"],float)
    planes=np.array(meta["planes"])
    free=np.array(meta["poses"])
    axis,center=fit_axis(free,nominal,planes[0].copy())
    approximate=axis_angles(free,axis,nominal)
    B=basis(axis)
    tag_maps=[{int(k):np.array(v) for k,v in x["tags"].items()} for x in meta["markers"]]
    hold=meta["holdout_id"]
    groups={}
    for i,tags in enumerate(tag_maps):
        for j,uv in tags.items():
            if j==hold: continue
            xyz=transform(image_plane(uv,planes[i],cal),free[i])
            groups.setdefault(j,[]).append(xyz)
    layout={k:np.median(v,axis=0) for k,v in groups.items()}
    def params(x):
        ax=axis+B@(x[:2]/60.)
        ax/=np.linalg.norm(ax)
        c=center+B@x[2:4]
        angles=approximate+np.r_[0,x[4:]]
        return ax,c,angles
    def residual(x):
        n,c,a=params(x)
        rest=[(project(layout[tag],np.linalg.inv(p),cal)-uv).ravel()
              for p,tags in zip(stage_poses(c,n,a),tag_maps) for tag,uv in tags.items() if tag in layout]
        return np.r_[np.concatenate(rest),x[:4]*.45,x[4:]/2.]
    limit=np.r_[[7.,7.,13.,13.],np.full(len(free)-1,5.)]
    opt=least_squares(residual,np.zeros(len(limit)),bounds=(-limit,limit),loss="soft_l1",f_scale=1.5,max_nfev=70)
    n,c,a=params(opt.x)
    return n,c,a,free,{"stage_fit_training_p50":float(np.median(abs(residual(opt.x)[:-len(limit)]))),"training_layout":layout}


def legacy_disk_points(scene,key):
    u,v=scene["uv"].T/8
    if key=="box_1116":
        return (u>62)&(u<178)&(v>194)&(v<223)&(scene["xyz"][:,2]>345)&(scene["xyz"][:,2]<465)
    rr=((u-151.)/72.)**2+((v-145.)/49.)**2
    return (rr<.94)&(rr>.36)&(v>120)&(scene["xyz"][:,2]<390)


def fit_deck(scene,key,n,center):
    # This mask is only a calibration measurement. Every decoded scene point is exported below.
    p=np.concatenate([s["xyz"][legacy_disk_points(s,key)][::4] for s in scene],axis=0)
    norm=np.linalg.norm
    B=basis(n)
    m=np.median((p-center)@n)
    height=(p-center)@n
    d=height[(height>m-15)&(height<m+15)]
    if len(d)<150:
        return n,center,{"plane_count":len(d),"plane_used":False}
    grid=np.arange(m-15,m+15,1)
    hist,edges=np.histogram(d,bins=grid)
    mode=float(edges[np.argmax(hist)]+.5)
    keep=abs(height-mode)<4.
    if keep.sum()<150:
        return n,center,{"plane_count":int(keep.sum()),"plane_used":False}
    p=p[keep]-center
    def residual(x):
        t=n+B@x[:2]
        return np.r_[(p@t-x[2]),10*x[:2]]
    fit=least_squares(residual,[0.,0.,mode],loss="soft_l1",f_scale=1.4,
                      bounds=([-.18,-.18,mode-4],[.18,.18,mode+4]),max_nfev=50).x
    u=n+B@fit[:2]
    u=u/norm(u)
    p50_old=float(np.median(np.abs(p@n-mode)))
    p50_new=float(np.median(abs(residual(fit)[:-2])))
    changed=p50_new<.90*p50_old
    if changed:
        center=center+u*(fit[2]/norm(n+B@fit[:2]))
    return (u if changed else n),center,{"plane_count":int(keep.sum()),"plane_used":changed,
                                           "plane_median_before":p50_old,"plane_median_after":p50_new}


def legacy_pose(key,meta,scenes,cal):
    radians=np.array(meta["angles"],float)
    original_file=None
    if key=="charger_1716":
        original_file=next((ROOT/meta["path"]).glob("axis_*.json"))
        a=load(original_file)
        n=-np.array(a["axis_direction"],float)
        n/=np.linalg.norm(n)
        c=np.array(a["axis_point_camera_mm"],float)
        return n,c,radians,stage_poses(c,n,radians),{"original_axis_file":str(original_file.relative_to(ROOT))}
    f={"minion":"minion","apple":"apple","box_1116":"legacy-box"}[key]
    d=load(NATIVE/f/"experiment.json")
    n=np.array(d["axis"])
    c=np.array(d["center"])
    prior=stage_poses(c,n,radians)
    original_file=d["axis_file"]
    best_center=np.array(d["rim_center"]) if d["rim_center"] else c
    tilt,best_center,extra=fit_deck(scenes,key,n,best_center)
    obs={int(i):np.array(v) for i,v in d["marker_centers"].items() if radians[int(i)]<360}
    fit_index=[]
    marker_errors={}
    if len(obs)>=3:
        rb=basis(tilt)
        plane=IDENTITY.copy()
        plane[:3,2]=tilt
        plane[:3,3]=best_center
        vi=sorted(obs)
        coords={i:(image_plane(obs[i][None],plane,cal)[0]-best_center)@rb for i in vi}
        ref=vi[0]
        # Hold out alternating strip observations. Use a clipped cyclic interpolation of corrections.
        good=vi[::2]
        x=np.array([radians[i] for i in good])
        delta=[]
        a=coords[ref]
        for i in good:
            b=coords[i]
            angle=np.rad2deg(np.arctan2(b[0]*a[1]-b[1]*a[0],a@b))
            delta.append((angle-(radians[i]-radians[ref])+180)%360-180)
        correction=np.interp(radians,np.r_[x,360.],np.r_[np.clip(delta,-12,12),0.])
        correction[0]=0.
        target=stage_poses(best_center,tilt,radians+correction)
        anchor=image_plane(obs[ref][None],plane,cal)
        def err(poses):
            p=transform(anchor,poses[ref])
            return [float(np.linalg.norm(project(p,np.linalg.inv(poses[i]),cal)[0]-obs[i])) for i in vi if i not in good]
        marker_errors={"frame_ids":[i for i in vi if i not in good],"nominal":err(stage_poses(best_center,tilt,radians)),
                       "corrected":err(target)}
        if np.median(marker_errors["corrected"])<np.median(marker_errors["nominal"]):
            radians=radians+correction
            fit_index=good
    # A two-observation disk strip can validate apple direction, with no stable absolute contour pose fitting.
    elif len(obs)==2:
        i,j=sorted(obs)
        P=IDENTITY.copy()
        P[:3,2]=tilt;P[:3,3]=best_center
        X=image_plane(obs[i][None],P,cal)
        def err(ps):
            return [float(np.linalg.norm(project(transform(X,ps[i]),np.linalg.inv(ps[j]),cal)[0]-obs[j]))]
        marker_errors={"frame_ids":[j],"nominal":err(prior),"corrected":err(stage_poses(best_center,tilt,radians))}
    return tilt,best_center,radians,prior,{"original_axis_file":original_file,"plane":extra,
           "strip_fit_frames":fit_index,"strip_eval":marker_errors}


def objects(key,meta,scenes,axis,center):
    cfg=load(ROOT/"output/registration-experiment/parameters.json")
    views=[]
    for i,s in enumerate(scenes):
        uv,xyz=s["uv"],s["xyz"]
        if key=="minion":
            ix=int(meta["views"][i]["frame"][-1])
            m=rectangle(uv,MINION[ix],8)
        elif key=="apple":
            m=polygon_mask(uv,cfg["apple_polygons_quarter_pixels"][i],(2048,2448))
        elif key=="box_1116":
            m=rectangle(uv,LEGACY_BOX[i],10)
        else:
            m=roi_mask(uv,meta["path"].split("/")[-1][14:20],i,0)
        h=(xyz-center)@axis
        # Quality and object masks are used in the benchmark only.
        if key=="box_1116":
            m&=(xyz[:,2]>275)&(xyz[:,2]<475)
        else:
            m&=(h>3.)&(h<(75 if "charger" in key else 105))
        m&=s["error"]<9.
        views.append(cloud_sample(xyz[m],uv[m],s["gray"][m]))
    return views


def estimate_unconstrained(views,poses,ring):
    n=len(poses)
    pp=poses.copy()
    failed=[]
    available=[i for i in range(n) if len(views[i]["train"])>=80]
    prepared={i:prepare_pair(views[i]["train"],CONFIG) for i in available}
    for a,b in ring[:-1]:
        if a not in prepared or b not in prepared:
            failed.append((a,b))
            continue
        try:
            M,_=fit_pair("point_to_point",prepared[a],prepared[b],adjacent_from(poses,a,b),CONFIG,None,None,71+a)
            if training_support(views,a,b,M)<.03:
                failed.append((a,b))
                continue
            pp[b]=pp[a]@np.linalg.inv(M)
        except RuntimeError:
            failed.append((a,b))
    return pp,failed


def guarded_angles(views,axis,center,angles,ring,limit):
    # Optimize small coupled angular changes and explicitly retain every viewpoint.
    n=len(angles)
    offsets=np.zeros(n)
    samples={i:views[i]["train"][::max(1,len(views[i]["train"])//1300)] for i in range(n)}
    links=[]
    base=stage_poses(center,axis,angles)
    for a,b in ring:
        if min(len(samples[a]),len(samples[b]))<60: continue
        p,q=transform(samples[a],base[a]),transform(samples[b],base[b])
        pa=cKDTree(q).query(p)[0]; pb=cKDTree(p).query(q)[0]
        support=min(np.mean(pa<5),np.mean(pb<5))
        if support>=.15:
            links.append((a,b,float(support)))
    for _ in range(3):
        ps=stage_poses(center,axis,angles+offsets)
        corr=[]
        for a,b,w in links:
            p,q=transform(samples[a],ps[a]),transform(samples[b],ps[b])
            d,j=cKDTree(q).query(p)
            keep=(d<5)&(cKDTree(p).query(q)[1][j]==np.arange(len(p)))
            if keep.sum()>=12:
                corr.append((a,b,samples[a][keep],samples[b][j[keep]],(w/keep.sum())**.5))
        if not corr: break
        def residual(x):
            ps=stage_poses(center,axis,angles+np.r_[0,x])
            d=[((transform(A,ps[a])-transform(B,ps[b]))*w).ravel() for a,b,A,B,w in corr]
            return np.r_[np.concatenate(d),x*.35]
        result=least_squares(residual,offsets[1:],bounds=(-limit,limit),max_nfev=16,loss="soft_l1",f_scale=.3)
        offsets[1:]=result.x
    return stage_poses(center,axis,angles+offsets),offsets,links


def write_scene(path, xyz, colors):
    path.parent.mkdir(parents=True,exist_ok=True)
    cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(xyz,dtype=float)))
    cloud.colors=o3d.utility.Vector3dVector(np.asarray(colors,dtype=float))
    if not o3d.io.write_point_cloud(str(path),cloud):
        raise IOError(path)
    return len(cloud.points)


def save_full(key,scenes,names,poses,repeat):
    folder=OUTPUTS/key
    full=[]; colors=[]; counts=[]
    for i,s in enumerate(scenes):
        # No ROI, rim, disk, height mask, voxel averaging, or object classifier is applied to output.
        color=np.broadcast_to(s["gray"][:,None].astype(float)/255.,(len(s["xyz"]),3))
        counts.append(write_scene(folder/"single"/f"{names[i]}.ply",s["xyz"],color))
        if i!=repeat:
            full.append(transform(s["xyz"],poses[i]))
            shade=(s["gray"].astype(float)/255.*.28+.72)
            colors.append(shade[:,None]*VIEW_COLORS[i%len(VIEW_COLORS)])
    points=np.concatenate(full)
    assert len(points)==sum(n for i,n in enumerate(counts) if i!=repeat)
    write_scene(folder/"full_scene.ply",points,np.concatenate(colors))
    return counts,len(points)


def optical_track_error(key,tracks,poses,cal):
    diff=[]
    for f in tracks:
        if f["session"]!=key: continue
        xyz=np.array(f["xyz"])
        i,j=f["views"]
        Pi,Pj=poses[i],poses[j]
        # Compare the same white image keypoint via camera reprojection under candidate poses.
        qa=project(transform(xyz[0:1],Pi),np.linalg.inv(Pj),cal)[0]
        qb=project(transform(xyz[1:2],Pj),np.linalg.inv(Pi),cal)[0]
        diff.extend((np.linalg.norm(qa-f["camera"][1]),np.linalg.norm(qb-f["camera"][0])))
    return float(np.median(diff)) if diff else None


def evaluate(poses,views,ring,n,c,anchor,meta,cal):
    B=basis(n)
    med=[];support=[]
    for a,b in ring:
        z=pair_metric(views[a],views[b],adjacent_from(poses,a,b))
        if z["status"]=="scored":
            support.append(z["f1_2"])
            med.append(z["seam_p95"])
    centers=np.asarray(poses)[:,:3,3]
    camera=(centers-c)@B
    angle=np.sort(np.arctan2(camera[:,1],camera[:,0]))
    gap=np.rad2deg(np.diff(np.r_[angle,angle[0]+2*np.pi])).max()
    aligned=np.concatenate([transform(v["val"],p) for v,p in zip(views,poses) if len(v["val"])])
    h=(aligned-anchor)@n
    lo,hi=np.percentile(h,[15,92])
    u=(aligned[(h>=lo)&(h<=hi)]-anchor)@B
    count=np.histogram(np.arctan2(u[:,1],u[:,0]),bins=np.linspace(-np.pi,np.pi,37))[0]
    pix=[]
    for i,val in meta.get("heldout",{}).items():
        f=project(np.array(meta["holdout_xyz"]),np.linalg.inv(poses[int(i)]),cal)
        pix.extend(np.linalg.norm(f-val,axis=1))
    return {"f1_2_median":float(np.median(support)) if support else None,
            "f1_2_worst":float(np.min(support)) if support else None,"edges_scored":len(support),
            "azimuth_filled_36":int(np.sum(count>=5)),"azimuth_counts":count,"camera_max_gap_deg":float(gap),
            "heldout_tag_px":float(np.median(pix)) if pix else None,
            "seam_p95":float(np.median([x for x in med if x is not None])) if any(x is not None for x in med) else None}


def train_tag_error(poses,cal,meta,layout):
    errors=[]
    for i,d in enumerate(meta["markers"]):
        for tag,uv in d["tags"].items():
            q=layout.get(int(tag))
            if q is not None:
                errors.extend(np.linalg.norm(project(q,np.linalg.inv(poses[i]),cal)-uv,axis=1))
    return float(np.median(errors))


def main():
    configure()
    audit=load(OUT/"optical_inputs.json")
    proj=load(OUT/"projector_validation.json")
    rows=[]
    for key,meta in audit.items():
        t=perf_counter()
        scenes=[]
        for p in sorted((CACHE/key).glob("*.npz")):
            with np.load(p) as f: scenes.append({k:f[k] for k in f.files})
        era=meta["era"]
        is_new=era=="1123_aruco"
        cal=({"1116":lambda:load_calibration(CAL16),"1120":lambda:load_calibration(CAL20),
              "1123_pre":lambda:load_calibration(CALIBRATION),"1123_aruco":lambda:with_arrays(proj["selected_calibration"])})[era]()
        if is_new:
            n,c,angle,old,extra=image_axis(meta,cal)
        else:
            n,c,angle,old,extra=legacy_pose(key,meta,scenes,cal)
        N=len(scenes)-(1 if meta["angles"][-1]==360 else 0)
        v=objects(key,meta,scenes,n,c)
        ring=[(i,(i+1)%N) for i in range(N)]
        physical=stage_poses(c,n,angle[:N])
        stat=old[:N]
        drift,failed=estimate_unconstrained(v,physical,ring)
        guarded,offset,links=guarded_angles(v,n,c,angle[:N],ring,limit=1.5 if is_new else 3.)
        anchor=np.median(np.concatenate([transform(x["train"],p) for x,p in zip(v,physical) if len(x["train"])]),axis=0)
        hypotheses={"archive_prior":stat,"unconstrained_icp":drift,"physical_axis":physical,"guarded_axis":guarded}
        if key=="box_1116":
            box_pose,box_check=rectangular_pose(v[:N],n,c,basis(n),angle[:N])
            hypotheses["rectangular_face"]=box_pose
            extra["rectangular_face_test"]=box_check
        results={}
        for method,ps in hypotheses.items():
            metric=evaluate(ps,v,ring,n,c,anchor,meta,cal)
            metric["heldout_white_track_px"]=optical_track_error(meta["path"].split("/")[-1][14:20],proj["heldout_tracks"],ps,cal)
            if N<len(scenes):
                rep=pair_metric(v[-1],v[0],IDENTITY)
                metric["repeat_identity_f1_2"]=rep.get("f1_2")
            results[method]=metric
        # Selection is conservative: geometry gets at most the physically bounded degrees of freedom.
        # Keep the calibrated pose if its gated fit has no geometry edges or breaks a held-out stage check.
        selected="guarded_axis" if links else "physical_axis"
        if is_new:
            for name,ps in hypotheses.items():
                results[name]["train_tag_px"]=train_tag_error(ps,cal,meta,extra["training_layout"])
            ref=results["physical_axis"]["train_tag_px"]
            cand=results["guarded_axis"]["train_tag_px"]
            archived=results["archive_prior"]["train_tag_px"]
            if cand is not None and cand>ref+1.0: selected="physical_axis"
            if ref is not None and archived is not None and ref>archived+2.5: selected="archive_prior"
        if key=="box_1116":
            a,b=extra["rectangular_face_test"]["training_residual_p50_before_after"]
            if b<.75*a:
                selected="rectangular_face"
        final=hypotheses[selected]
        if N==8:
            ix=[0,2,4,6]
            thin=[(a,b) for a,b in zip(ix,ix[1:]+ix[:1])]
            extra["controlled_subsample"]={"full_8x45_f1_2":results[selected]["f1_2_median"],
                "4x90_f1_2":evaluate(final,v,thin,n,c,anchor,meta,cal)["f1_2_median"],"indices":ix}
        poses=np.concatenate((final,IDENTITY[None])) if N<len(scenes) else final
        names=[a["frame"]+("_360deg_repeat" if i==N else "") for i,a in enumerate(meta["views"])]
        counts,total=save_full(key,scenes,names,poses,N if N<len(scenes) else None)
        sub=OUTPUTS/key
        # Diagnostic export of the exact same full scene under two alternative poses, preserving identical vertex count.
        def diagnostic(name,ps):
            xyz=np.concatenate([transform(s["xyz"],p) for s,p in zip(scenes[:N],ps)])
            rgb=np.concatenate([np.broadcast_to(VIEW_COLORS[i%8],(len(s["xyz"]),3)) for i,s in enumerate(scenes[:N])])
            write_scene(sub/name,xyz,rgb)
        diagnostic("archive_prior_scene.ply",stat)
        diagnostic("unconstrained_icp_scene.ply",drift)
        merged_for_plot={}
        for method,pp in hypotheses.items():
            positions=[]
            for i,(s,p,obj) in enumerate(zip(scenes[:N],pp,v)):
                positions.append(transform(obj["p"][::3],p))
            merged_for_plot[method]=np.concatenate(positions).astype(np.float32)
        np.savez_compressed(sub/"object_evaluation_points.npz",**merged_for_plot)
        dump(sub/"poses_and_metrics.json",{"case":key,"path":meta["path"],"era":era,"axis":n,"center":c,"object_center":anchor,
           "nominal_angles":meta["angles"],"fitted_angles":angle,"selected":selected,"poses":poses,"pose_variants":hypotheses,
           "guarded_angle_offsets":offset,"guarded_edges":links,"icp_rejected_edges":failed,"diagnostics":results,
           "single_counts":counts,"fused_count":total,"repeat_index":N if N<len(scenes) else None,"reference":extra,
           "method_choice_used_heldout_tag":False,"elapsed_s":perf_counter()-t,"object_eval_counts":[len(a["val"]) for a in v]})
        for m,row in results.items():
            fixed={k:x for k,x in row.items() if k!="azimuth_counts"}
            rows.append({"case":key,"method":m,"selected":m==selected,**fixed,"fused_count":total})
        print(key,selected,total,{k:(round(a["camera_max_gap_deg"]),a["f1_2_median"],a["heldout_tag_px"]) for k,a in results.items()},flush=True)
    fields=list(dict.fromkeys(k for r in rows for k in r))
    with (OUT/"benchmark.csv").open("w",newline="") as f:
        wr=csv.DictWriter(f,fieldnames=fields);wr.writeheader();wr.writerows(rows)


if __name__=="__main__":
    main()
