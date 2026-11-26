"""E002 case-level sweep, leave-one-case-out separation, and numbered PLY exports."""
from pathlib import Path
from time import perf_counter
import csv
import json

import cv2
import numpy as np

from common import HERE, IDS, KEYS, BOX, ROOT, OUTPUT, PREV, NAMES, dump, load, load_case, ply, transform
from segment import separate, evaluate_masks
from registration import run_registration

SEG={"M01":"统一高度阈值","M02":"逐帧盘面与自适应高度","M03":"逐帧盘面与三维连通域",
     "M04":"三维连通域与跨视角支持","M05":"多视角支持与 GrabCut","M06":"图像限制与近底恢复","M07":"近底恢复与偏心门控"}
REGS={"R01":"E001 留存位姿","R02":"同期转轴摆正","R03":"R01 初值 + 串联 P2P","R04":"R01 初值 + 串联 P2Plane",
      "R05":"R01 初值 + 串联 GICP","R06":"法向门控与分批加入的位姿图","R07":"直立长方体与微量位姿联合拟合",
      "R08":"直立长方体 + 训练码约束","R09":"R02 初值 + 串联 P2P","R10":"R02 初值 + 串联 P2Plane",
      "R11":"R02 初值 + 串联 GICP"}
COLORS=np.array([[.19,.44,.77],[.91,.44,.20],[.16,.59,.41],[.58,.37,.70],[.81,.61,.15],[.16,.61,.66],
                 [.78,.34,.52],[.44,.53,.63]])
H=OUTPUT/"cache"


def write_csv(p,rows):
    keys=list(dict.fromkeys(k for x in rows for k in x))
    with p.open("w",newline="",encoding="utf-8-sig") as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)


def select_segment(rows,exclude):
    data=[r for r in rows if r["case"]!=exclude]
    summ={}
    for m in SEG:
        partial=[r for r in data if r["method"]==m]
        rate=np.mean([r["f1"]+.06*r["low_tp"]/max(1,r["low_n"]) for r in partial])
        summ[m]=float(rate)
    return max(summ,key=lambda m:(round(summ[m],10),m))


def choose_reg(metrics):
    base=metrics["R01"]
    approved=[]
    for label in ("R06","R07","R08"):
        z=metrics.get(label)
        if z is None:continue
        # Two adjacent edges can each move by about 2 degrees under the bounded pose graph.
        physics=z["max_gap"]<=base["max_gap"]+4.
        if base["tag_P50"] is not None:physics &= z["tag_P50"]<=base["tag_P50"]+1.5
        if base["depth_supported"] is not None:
            physics &= z["depth_supported"]>=base["depth_supported"]-.04
        err=z["depth_abs_P50"]
        if err is None or base["depth_abs_P50"] is None:continue
        # Conservative development gate. Use depth and shape jointly to avoid a good score from a collapsed single band.
        improvement=err<base["depth_abs_P50"]*.95
        if label in ("R07","R08"):
            improvement |= (z["box_surface_P95"]<base["box_surface_P95"]*.95 and err<=base["depth_abs_P50"]+.6)
        if physics and improvement: approved.append(label)
    return min(approved,key=lambda label:metrics[label]["depth_abs_P50"]) if approved else "R01"


def axis_resampling(meta,previous):
    if "poses" not in meta: return None
    n=np.asarray(previous["axis"])
    base=np.asarray(previous["center"])
    from scipy.spatial.transform import Rotation
    T=np.array(meta["poses"])
    A=T[1:,:3,:3]
    b=T[1:,:3,3]
    estimates=[]
    for i in range(len(A)):
        sel=np.arange(len(A))!=i
        rv=Rotation.from_matrix(A[sel]).as_rotvec()
        unit=np.sign(rv@n)[:,None]*rv/np.linalg.norm(rv,axis=1,keepdims=True)
        ax=np.median(unit,axis=0);ax/=np.linalg.norm(ax)
        M=(np.eye(3)-A[sel]).reshape(-1,3)
        X=np.linalg.lstsq(np.vstack((M,ax)),np.r_[b[sel].ravel(),ax@base],rcond=None)[0]
        estimates.append((X,ax))
    center=np.array([v[0] for v in estimates])
    normals=np.array([v[1] for v in estimates])
    middle=np.median(center,axis=0)
    spread=np.rad2deg(np.arccos(np.clip(normals@n,-1,1)))
    return {"center_jackknife_P95":np.percentile(np.linalg.norm(center-middle,axis=1),95),
            "axis_difference_vs_previous_P95_deg":np.percentile(spread,95),"n_folds":len(estimates)}


def main():
    cv2.ocl.setUseOpenCL(False);cv2.setNumThreads(2);cv2.setRNGSeed(442)
    H.mkdir(parents=True,exist_ok=True)
    seg_data=[];diagnosis=[]
    for key in KEYS:
        t=perf_counter()
        meta,ps,poses,s,cal=load_case(key)
        r=separate(key,s,poses,ps,cal)
        folder=H/IDS[key]
        folder.mkdir(exist_ok=True)
        for i,(a,b) in enumerate(zip(s,r)):
            e=evaluate_masks(key,i,a,b)
            if e:
                for label,scores in e["test"].items():
                    seg_data.append({"case":key,"id":IDS[key],"view":i,"method":label,**scores})
                diagnosis.append({"case":key,"id":IDS[key],"view":i,"valid_depth_inside":e["valid_depth_inside"],
                     "inside_valid_fraction":e["inside_valid_fraction"],"plane_offset":b["plane_offset"],
                     "plane_noise":b["plane_noise"],"decode_error_P50":float(np.median(a["error"])),
                     "low_measurement_candidates":e["test"]["M01"]["low_n"]})
                cv2.imwrite(str(folder/f"V{i:02}-ref.png"),e["gt_preview"])
            np.savez_compressed(folder/f"V{i:02}.npz",**{k:b[k] for k in [*SEG,"h","silhouette","n","c"]})
        print("SEG",IDS[key],round(perf_counter()-t,2),flush=True)
    write_csv(OUTPUT/"E002-01.csv",seg_data)
    write_csv(OUTPUT/"E002-03.csv",diagnosis)
    reg_rows=[];index={}
    for key in KEYS:
        t=perf_counter()
        tag=IDS[key]
        meta,previous,poses,scenes,cal=load_case(key)
        segmented=[]
        for p in sorted((H/tag).glob("V*.npz")):
            with np.load(p) as z:segmented.append({x:z[x] for x in z.files})
        seg=select_segment(seg_data,key)
        out,metrics,shape=run_registration(key,meta,previous,scenes,segmented,seg,cal)
        reg=choose_reg(metrics)
        chosen=out[reg][0]
        count=len(chosen)
        sub=OUTPUT/"clouds"/tag
        full=[];fc=[];objects=[];oc=[]
        tile_counts=[]
        for i,T in enumerate(chosen):
            scan=scenes[i]
            moved=transform(scan["xyz"],T)
            gray=scan["gray"].astype(float)/255.
            color=(gray*.43+.57)[:,None]*COLORS[i%8]
            keep=segmented[i][seg]
            full.append(moved);fc.append(color)
            objects.append(moved[keep]);oc.append(color[keep])
            ply(sub/"views"/f"{tag}-V{i:02}.ply",moved[keep],gray[keep])
            tile_counts.append(int(keep.sum()))
        # E002's scene retains exactly all previously validated measured samples.
        ply(sub/f"{tag}-01.ply",np.concatenate(full),np.concatenate(fc))
        ply(sub/f"{tag}-02.ply",np.concatenate(objects),np.concatenate(oc))
        variants_to_export={"R01"}
        if key in BOX:
            variants_to_export.update({"R06","R07","R08"}&out.keys())
        elif key=="apple":
            variants_to_export.add("R10")
        for variant in sorted(variants_to_export-{reg}):
            clouds=[transform(s["xyz"][m[seg]],p) for s,m,p in zip(scenes,segmented,out[variant][0])]
            ply(sub/f"{tag}-{variant}.ply",np.concatenate(clouds),np.concatenate(oc))
        selected_rows=[x for x in seg_data if x["case"]==key and x["method"]==seg]
        repeat=None
        if previous["repeat_index"] is not None:
            i=previous["repeat_index"]
            keep=segmented[i][seg]
            ply(sub/"views"/f"{tag}-V{i:02}.ply",scenes[i]["xyz"][keep],scenes[i]["gray"][keep]/255.)
            from scipy.spatial import cKDTree
            a=objects[0];b=scenes[i]["xyz"][keep]
            dx=cKDTree(a).query(b)[0];dy=cKDTree(b).query(a)[0]
            repeat={"f1_2":2*(dx<2).mean()*(dy<2).mean()/max(1e-9,(dx<2).mean()+(dy<2).mean()),
                    "nearest_p95":np.percentile(np.r_[dx,dy],95)}
        det=np.linalg.det(chosen[:,:3,:3])
        assert np.allclose(det,1.) and np.isfinite(chosen).all()
        extra={"case":key,"id":tag,"label":NAMES[key],"separation":seg,"registration":reg,
            "selected_poses":chosen,"axis":previous["axis"],"center":previous["center"],
            "source":meta["path"],"shape":shape,"variants":{m:x[0] for m,x in out.items()},
            "metrics":metrics,"label_scores":selected_rows,"repeated_view":repeat,"axis_resampling":axis_resampling(meta,previous),
            "full_count":sum(len(x) for x in full),"object_count":sum(tile_counts),"counts_by_view":tile_counts}
        for name,m in metrics.items():
            reg_rows.append({"case":key,"id":tag,"method":name,"selected":name==reg,**m})
        dump(sub/f"{tag}-03.json",extra)
        index[tag]={"case":key,"label":NAMES[key],"segmentation":seg,"registration":reg,"full_count":extra["full_count"],
          "object_count":extra["object_count"],"index":str((sub/f"{tag}-03.json").relative_to(OUTPUT))}
        print("REG",tag,seg,reg,round(perf_counter()-t,2),metrics[reg]["depth_abs_P50"],flush=True)
    write_csv(OUTPUT/"E002-02.csv",reg_rows)
    dump(OUTPUT/"E002-04.json",{"cases":index,"segmentation":SEG,"registration":REGS})


if __name__=="__main__":
    main()
