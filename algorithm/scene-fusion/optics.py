"""Decode full camera frames and calibrate the camera/projector from local data."""
from pathlib import Path
import json
import sys

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"algorithm"))
from prepare_calibrated_benchmark import CALIBRATION, DATA, SCANS, detect, load_calibration, load_frame, triangulate
from calibrate_benchmark_turntable import image_plane, project, rig_poses, roi_mask, table_samples, transform
from registration_experiment import training_mask

OUT=Path(__file__).resolve().parent/"results"
CACHE=OUT/"intermediate"
CAL20=DATA/"procam_calibration/20261120_analysis/calibration_10mm_experimental.xml"
CAL16=DATA/"procam_calibration/calibration_20261116_10mm_experimental.xml"

def configure():
    cv2.ocl.setUseOpenCL(False)
    cv2.setNumThreads(2)
    cv2.setRNGSeed(23)
    CACHE.mkdir(parents=True,exist_ok=True)


def dump(path, obj):
    def encode(x):
        return x.tolist() if isinstance(x,np.ndarray) else x.item()
    path.write_text(json.dumps(obj,default=encode,ensure_ascii=False,allow_nan=False,indent=2)+"\n")


def load(path):
    return json.loads(path.read_text())


def with_arrays(d):
    return {key:(np.array(value,float) if isinstance(value,list) else value) for key,value in d.items()}


def camera_candidates():
    refs=sorted((DATA/"procam_calibration").glob("capture_20261123_171*"))
    obj=np.c_[np.mgrid[0:9,0:7].T.reshape(-1,2)*10,np.zeros(63)].astype(np.float32)
    corners=[]
    for p in refs:
        img=load_frame(p,42)
        small=cv2.resize(img,None,fx=.4,fy=.4,interpolation=cv2.INTER_AREA)
        ok,xy=cv2.findChessboardCornersSB(small,(9,7),cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ok:
            uv=cv2.cornerSubPix(img,(xy/.4).astype(np.float32),(7,7),(-1,-1),(3,40,.01))
            corners.append((p.name,uv))
    if len(corners)<7:
        raise ValueError("Fewer than seven 11/23 camera calibration boards")
    train=[v for i,(_,v) in enumerate(corners) if i%3!=1]
    archived=load_calibration(CALIBRATION)
    base=load_calibration(CAL20)
    configs={"archive_1123":archived,"archive_1120_camera_only":{**archived,**{k:base[k] for k in ("cam_int","cam_dist")}}}
    for name,flag in (("refit_free",0),("refit_square_pixels",cv2.CALIB_FIX_ASPECT_RATIO)):
        K=base["cam_int"].copy()
        if flag: K[0,0]=K[1,1]=np.mean(K.diagonal()[:2])
        rms,K,D,_,_=cv2.calibrateCamera([obj]*len(train),train,(2448,2048),K,np.zeros(5),
                                     flags=cv2.CALIB_USE_INTRINSIC_GUESS|cv2.CALIB_FIX_K3|flag)
        configs[name]={**archived,"cam_int":K,"cam_dist":D,"camera_train_rms":rms}
    rows={}
    for name,cal in configs.items():
        hold=[]
        for i,(f,uv) in enumerate(corners):
            ok,r,t=cv2.solvePnP(obj,uv,cal["cam_int"],cal["cam_dist"])
            if not ok: raise RuntimeError(f)
            e=np.linalg.norm(cv2.projectPoints(obj,r,t,cal["cam_int"],cal["cam_dist"])[0]-uv,axis=2).ravel()
            hold.append({"folder":f,"split":"holdout" if i%3==1 else "train",
                         "rms":float(np.sqrt(np.mean(e**2))),"p95":float(np.percentile(e,95))})
        rows[name]={"fx":cal["cam_int"][0,0],"fy":cal["cam_int"][1,1],
                    "holdout_rms_median":np.median([x["rms"] for x in hold if x["split"]=="holdout"]),"views":hold}
    # Refit only from contemporaneous images. The historical camera is included for diagnosis.
    name=min(("archive_1123","refit_square_pixels","refit_free"),key=lambda k:rows[k]["holdout_rms_median"])
    dump(OUT/"camera_validation.json",{"chosen":name,"sources":"11/23 17:11–17:14, 9 × 7 corners at nominal 10 mm","results":rows,"camera":configs[name]})
    print("camera",name,{k:round(v["holdout_rms_median"],3) for k,v in rows.items()},flush=True)
    return configs[name]


def decoded(view, dynamic_x):
    """Read the sequence once. Keep nested bit-confidence levels and both MSB hypotheses."""
    white,black=(load_frame(view,i)[::4,::4] for i in (42,43))
    m=white.astype(np.int16)-black>=15
    y,x=np.nonzero(m)
    camera=np.column_stack((x*4,y*4)).astype(float)
    deltas=np.empty((21,len(x)),np.int16)
    for i in range(21):
        a,b=(load_frame(view,j)[y*4,x*4] for j in (2*i,2*i+1))
        deltas[i]=a.astype(np.int16)-b
    levels=[]
    for lost in (0,1,2):
        uv=[]
        good=np.ones(len(x),bool)
        for start,n,end in ((0,11,1280),(11,10,720)):
            signs=deltas[start:start+n-lost].copy()
            if dynamic_x and start==0:
                good &= (np.abs(signs[1:])>=5).all(0)
                signs[0]=-1
            else:
                good &= (np.abs(signs)>=5).all(0)
            binary=np.logical_xor.accumulate(signs>0,axis=0)
            u=(binary*2**np.arange(n-1,lost-1,-1)[:,None]).sum(0)+(2**lost-1)/2
            good &= u<end
            uv.append(u)
        levels.append((np.column_stack(uv),good))
    return camera,white[m],levels,deltas[0]


def recover_xyz(cal, data, max_lost=2, recover_msb=False):
    camera,white,levels,msb=data
    xyz=np.full((len(camera),3),np.nan)
    err=np.full(len(camera),np.inf)
    out=np.full((len(camera),2),np.nan)
    bits=np.full(len(camera),-1,dtype=np.int8)
    for i,(st,good) in enumerate(levels[:max_lost+1]):
        take=np.flatnonzero(good & (bits<0))
        if not len(take):
            continue
        uv,p=camera[take],st[take].copy()
        a,e=triangulate(cal,uv,p)
        d=e.max(1)
        valid=(a[:,2]>80)&(a[:,2]<900)&np.isfinite(a).all(1)
        if recover_msb:
            alt=p.copy()
            alt[:,0]=2047-alt[:,0]
            candidates=(alt[:,0]<1280)
            if candidates.any():
                ids=np.where(candidates)[0]
                b,f=triangulate(cal,uv[ids],alt[ids])
                df=f.max(1)
                alt_valid=(b[:,2]>80)&(b[:,2]<900)&np.isfinite(b).all(1)
                pick=alt_valid & ((df<d[ids]-.6) | ~valid[ids] | ((msb[take[ids]]>10)&(df<d[ids]+.5)))
                a[ids[pick]],d[ids[pick]],p[ids[pick]]=b[pick],df[pick],alt[ids[pick]]
                valid[ids[pick]]=True
        keep=valid&(d<8+1.5*i)
        sel=take[keep]
        xyz[sel],err[sel],bits[sel],out[sel]=a[keep],d[keep],i,p[keep]
    m=bits>=0
    return {"xyz":xyz[m],"uv":camera[m],"gray":white[m],"projector":out[m],"bits":bits[m],"error":err[m],
            "raw_samples":len(camera)}


def rays(uv,cal):
    return cv2.undistortPoints(np.asarray(uv,float),cal["cam_int"],cal["cam_dist"]).reshape(-1,2)


def feature_matches(frames, scenes, plans, poses, cal, sid):
    """Triangulate real object keypoints between white images, then locate their Gray correspondences."""
    sift=cv2.SIFT_create(nfeatures=2200,contrastThreshold=.018)
    fs=[]
    for i,p in enumerate(frames):
        im=load_frame(p,42)
        small=cv2.resize(im,None,fx=.5,fy=.5)
        yy,xx=np.indices(small.shape)
        mask=roi_mask(np.c_[xx.ravel()*2,yy.ravel()*2],sid,i).reshape(small.shape)
        k,d=sift.detectAndCompute(small,mask.astype(np.uint8)*255)
        xy=np.array([np.array(e.pt)*2+.5 for e in k])
        fs.append((xy,d))
    bf=cv2.BFMatcher()
    scene_trees=[cKDTree(s["uv"]) for s in scenes]
    records=[]
    for i in range(len(frames)):
        for j in range(i+1,len(frames)):
            if min(j-i,len(frames)-j+i)>2: continue
            if fs[i][1] is None or fs[j][1] is None: continue
            forward=bf.knnMatch(fs[i][1],fs[j][1],k=2)
            backward=bf.match(fs[j][1],fs[i][1])
            rev={m.queryIdx:m.trainIdx for m in backward}
            selected=[p for m in forward if len(m)==2 for p,q in [m] if p.distance<.77*q.distance and rev.get(p.trainIdx)==p.queryIdx]
            if not selected: continue
            a=fs[i][0][[m.queryIdx for m in selected]]
            b=fs[j][0][[m.trainIdx for m in selected]]
            Ti=np.linalg.inv(poses[i])
            Tj=np.linalg.inv(poses[j])
            q=cv2.triangulatePoints(Ti[:3],Tj[:3],rays(a,cal).T,rays(b,cal).T).T
            xyz=q[:,:3]/q[:,3:4]
            ca,cb=transform(xyz,Ti),transform(xyz,Tj)
            ea=np.linalg.norm(project(xyz,Ti,cal)-a,axis=1)
            eb=np.linalg.norm(project(xyz,Tj,cal)-b,axis=1)
            h=(ca-plans[i][:3,3])@plans[i][:3,2]
            if plans[i][2,2]>0: h=-h
            da,ai=scene_trees[i].query(a)
            db,bi=scene_trees[j].query(b)
            h_match=(ea<2.8)&(eb<2.8)&(h>5)&(h<105)&(ca[:,2]>130)&(cb[:,2]>130)&(da<5)&(db<5)
            h_match &= (np.linalg.norm(ca-scenes[i]["xyz"][ai],axis=1)<38)&(np.linalg.norm(cb-scenes[j]["xyz"][bi],axis=1)<38)
            for k in np.where(h_match)[0]:
                records.append({"session":sid,"views":[i,j],"camera":[a[k],b[k]],"xyz":[ca[k],cb[k]],
                                "projector":[scenes[i]["projector"][ai[k]],scenes[j]["projector"][bi[k]]],
                                "camera_max_px":max(float(ea[k]),float(eb[k]))})
    return records


def projector_candidate(base, cal, planes, scenes, features):
    # Table observations determine nominal metric scale. Off-plane SIFT tracks constrain triangulation depth.
    tables=[]
    for i,s in enumerate(scenes):
        m=table_samples(s["uv"],"174129",i)&training_mask(s["uv"])
        xyz=image_plane(s["uv"][m],planes[i],cal)
        st=s["projector"][m]
        pred=cv2.projectPoints(xyz,base["rotation"],base["translation"],base["proj_int"],base["proj_dist"])[0][:,0]
        err=np.linalg.norm(pred-st,axis=1)
        # Recompute the planar PnP even when the camera-only refit changed pixel rays.
        keep=err<np.clip(np.percentile(err,60),7,40)
        tables.append((xyz[keep][::25],st[keep][::25]))
    xyz_t,st_t=[np.concatenate(z) for z in zip(*tables)]
    ok,r,t,inl=cv2.solvePnPRansac(xyz_t,st_t,base["proj_int"],base["proj_dist"],
                                 iterationsCount=1500,reprojectionError=7,confidence=.999)
    if not ok: raise RuntimeError("Planar optical refit failed")
    xyz_t,st_t=xyz_t[inl.ravel()],st_t[inl.ravel()]
    pts=np.concatenate([f["xyz"] for f in features],axis=0) if features else np.empty((0,3))
    pix=np.concatenate([f["projector"] for f in features],axis=0) if features else np.empty((0,2))
    L,D=base["proj_int"],base["proj_dist"]
    def unpack(x):
        K=L.copy()
        K[0,0]*=np.exp(x[6]); K[1,1]*=np.exp(x[7]); K[:2,2]+=x[8:]*150
        return x[:3],x[3:6],K
    def predict(xyz,x):
        r,t,K=unpack(x)
        return cv2.projectPoints(xyz,r,t,K,D)[0][:,0]
    start=np.r_[r.ravel(),t.ravel(),np.zeros(4)]
    def residual(x):
        table=(predict(xyz_t,x)-st_t).ravel()
        residuals=[table,x[6:]*np.array([15,15,8,8])]
        if len(pts): residuals.append((predict(pts,x)-pix).ravel()*min(3.,np.sqrt(len(xyz_t)/len(pts))))
        return np.concatenate(residuals)
    b=np.array([.38,.38,1.8,1.8])
    fit=least_squares(residual,start,loss="soft_l1",f_scale=2.,max_nfev=120,
                      bounds=(np.r_[np.full(6,-np.inf),-b],np.r_[np.full(6,np.inf),b]))
    r,t,K=unpack(fit.x)
    return {**cal,"rotation":cv2.Rodrigues(r)[0],"translation":t[:,None],"proj_int":K,"proj_dist":D}


def examine_projector(cal,features):
    if not features: return None
    xyz=np.concatenate([f["xyz"] for f in features],axis=0)
    uv=np.concatenate([f["projector"] for f in features],axis=0)
    e=np.linalg.norm(cv2.projectPoints(xyz,cal["rotation"],cal["translation"],cal["proj_int"],cal["proj_dist"])[0][:,0]-uv,axis=1)
    return np.percentile(e,[50,95]).tolist()

CASES={
 "minion":("scan_20261120*小黄人","1120"),
 "apple":("scan_20261120*苹果","1120"),
 "box_1116":("scan_20261116_174014*","1116"),
 "charger_1716":("scan_20261123_171640*","1123_pre"),
 "charger":("scan_20261123_174129*","1123_aruco"),
 "box_center":("scan_20261123_174531*","1123_aruco"),
 "box_offset":("scan_20261123_174747*","1123_aruco"),
 "box_1750":("scan_20261123_175006*","1123_aruco"),
}


def case_inputs():
    result={}
    for key,(glob,era) in CASES.items():
        scan=next(SCANS.glob(glob))
        cfg=load(scan/"session.json")
        logged={m["folder"]:m["nominal_angle_degrees"] for m in cfg["views"]}
        if key=="minion":
            logged=load(scan/"corrected_nominal_angles.json")
        frames=[(scan/d,angle) for d,angle in logged.items() if (scan/d).exists()]
        result[key]=(scan,era,frames)
    return result


def save_scene(key,scenes):
    folder=CACHE/key
    folder.mkdir(exist_ok=True)
    for i,s in enumerate(scenes):
        np.savez_compressed(folder/f"{i:02}.npz",**{k:v.astype(np.float32) if isinstance(v,np.ndarray) and v.dtype==np.float64 else v for k,v in s.items()})


def table_eval(model,plans,initial_scenes,sid):
    stats=[]
    for i,s in enumerate(initial_scenes):
        m=table_samples(s["uv"],sid,i)&~training_mask(s["uv"])
        xyz=image_plane(s["uv"][m],plans[i],model)
        if not len(xyz): continue
        pix=cv2.projectPoints(xyz,model["rotation"],model["translation"],model["proj_int"],model["proj_dist"])[0][:,0]
        e=np.linalg.norm(pix-s["projector"][m],axis=1)
        stats.append(np.percentile(e,[50,95]))
    return np.median(stats,axis=0).tolist()


def main():
    configure()
    camera_path=OUT/"camera_validation.json"
    cal=with_arrays(load(camera_path)["camera"]) if camera_path.exists() else camera_candidates()
    base=with_arrays(load(ROOT/"output/calibrated-benchmark/recovery/projector_recovery.json"))
    base={**base,"cam_int":cal["cam_int"],"cam_dist":cal["cam_dist"]}
    audit={}
    modern_cases={}
    for key,(path,era,rows) in case_inputs().items():
        is_new=era=="1123_aruco"
        frames=[v for v,_ in rows]
        optical={"1116":lambda:load_calibration(CAL16),"1120":lambda:load_calibration(CAL20),
                 "1123_pre":lambda:load_calibration(CALIBRATION),"1123_aruco":lambda:base}[era]()
        records=[]; initial=[]; sources=[]
        for view in frames:
            d=decoded(view,is_new)
            scene=recover_xyz(optical,d,recover_msb=is_new)
            initial.append(scene)
            sources.append(d) if is_new else None
            levels=[np.count_nonzero(scene["bits"]<=i) for i in (0,1,2)]
            records.append({"frame":view.name,"samples_with_white_black":scene["raw_samples"],"points_0_1_2":levels,
                            "stereo_residual_p50_p95": np.percentile(scene["error"],[50,95]).tolist()})
        details={"path":str(path.relative_to(ROOT)),"era":era,"angles":[a for _,a in rows],"views":records}
        audit[key]=details
        if is_new:
            m={"views":[{"tags":detect(v)} for v in frames]}
            plans,poses,ref,h,held,err=rig_poses(m,cal)
            features=feature_matches(frames,initial,plans,poses,cal,path.name[14:20])
            details.update(planes=plans,poses=poses,holdout_id=h,holdout_xyz=ref,heldout=held,tag_fit_p50=np.median(err),markers=m["views"])
            modern_cases[key]=(sources,initial,features,details)
            print(key,"features",len(features),flush=True)
        else:
            save_scene(key,initial)
        print(key,[v["points_0_1_2"][-1] for v in records],flush=True)
    a=modern_cases["charger"][3]
    train=[]; val=[]
    for key,(_,_,features,_) in modern_cases.items():
        for f in features:
            if key in ("charger","box_center") and max(f["views"])<5: train.append(f)
            else: val.append(f)
    candidate=projector_candidate(base,cal,a["planes"],modern_cases["charger"][1],train) if len(train)>=12 else base
    p0,p1=[examine_projector(p,val) for p in (base,candidate)]
    t0s=[table_eval(base,r["planes"],s,r["path"].split("/")[-1][14:20]) for _,s,_,r in modern_cases.values()]
    t1s=[table_eval(candidate,r["planes"],s,r["path"].split("/")[-1][14:20]) for _,s,_,r in modern_cases.values()]
    better=len(train)>=12 and len(val)>=12 and p1[0]<.90*p0[0] and np.median(np.array(t1s)[:,0])<np.median(np.array(t0s)[:,0])+1.5
    selected=candidate if better else base
    dump(OUT/"projector_validation.json",
         {"selected":"plane_plus_off_plane_tracks" if better else "archive_plane_epipolar_refit",
          "train_count":len(train),"heldout_count":len(val),"feature_p50_p95_before":p0,"feature_p50_p95_after":p1,
          "table_p50_p95_before":t0s,"table_p50_p95_after":t1s,"case_order":list(modern_cases),
          "original":base,"candidate":candidate,"selected_calibration":selected,"train_tracks":train,"heldout_tracks":val})
    for key,(sources,initial,_,m) in modern_cases.items():
        if better:
            final=[recover_xyz(selected,x,recover_msb=True) for x in sources]
        else:
            final=initial
        save_scene(key,final)
        for s,rec in zip(final,m["views"],strict=True):
            rec["selected_points"]=len(s["xyz"])
            rec["selected_reprojection_p50"]=float(np.median(s["error"]))
    dump(OUT/"optical_inputs.json",audit)


if __name__=="__main__":
    main()
