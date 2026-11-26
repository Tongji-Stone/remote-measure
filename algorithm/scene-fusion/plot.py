"""Review the full exports and the failure cases without editing the PLYs."""
import csv
from hashlib import sha256
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"algorithm"))
from plot_registration_experiment import camera_basis, read_ply

OUT=Path(__file__).resolve().parent/"results"
CLOUDS=OUT/"clouds"
PLOT=OUT/"figures"
PLOT.mkdir(exist_ok=True)
font_manager.fontManager.addfont("/Library/Fonts/Arial Unicode.ttf")
plt.rcParams.update({"font.family":"Arial Unicode MS","font.size":11,"axes.unicode_minus":False,
                     "axes.spines.top":False,"axes.spines.right":False,"text.color":"#163048"})
CASES={"minion":"小黄人 · 11/20","apple":"苹果 · 11/20","box_1116":"长方体纸盒 · 11/16",
       "charger":"充电头 · 11/23","charger_1716":"充电头 · 17:16",
       "box_center":"包装盒居中 · 11/23","box_offset":"包装盒偏心 · 11/23","box_1750":"包装盒 · 17:50"}
MANIFEST={}


def digest(p):
    return sha256(p.read_bytes()).hexdigest()


def save(fig,name,caption,inputs):
    path=PLOT/f"{name}.png"
    fig.savefig(path,dpi=165,facecolor="white")
    plt.close(fig)
    MANIFEST[name]={"filename":path.name,"sha256":digest(path),"caption":caption,"caption_sha256":sha256(caption.encode()).hexdigest(),
                    "sources":{str(p.relative_to(ROOT)):digest(p) for p in inputs},"code":digest(Path(__file__))}


def get(k):
    return json.loads((CLOUDS/k/"poses_and_metrics.json").read_text())


def orient(n,yaw,elev):
    a=np.array(n,copy=True)
    if a[1]>0: a=-a
    return camera_basis(a,yaw,elev)


def panel(ax,xyz,origin,pose,rgb,extent=None):
    p=(xyz-origin)@pose
    if extent:
        ax.set(xlim=extent[:2],ylim=extent[2:])
    if len(p)>35000:
        ix=np.linspace(0,len(p)-1,35000).astype(int)
        p,rgb=p[ix],rgb[ix]
    order=np.argsort(p[:,2])
    ax.scatter(p[order,0],p[order,1],c=rgb[order],s=1.2,alpha=.65,linewidths=0)
    ax.set_aspect("equal",adjustable="box")
    ax.set_xticks([]);ax.set_yticks([])
    for spine in ax.spines.values(): spine.set_visible(False)


def scenes():
    keys=("minion","apple","charger","box_center","box_offset","box_1116")
    fig,axs=plt.subplots(2,3,figsize=(14,8),layout="constrained")
    data=[]
    for k,ax in zip(keys,axs.flat):
        p=CLOUDS/k/"full_scene.ply"
        d=get(k); data.extend([p,CLOUDS/k/"poses_and_metrics.json"])
        xy,rgb=read_ply(p)
        n,c=d["axis"],np.array(d["center"])
        view=orient(n,-23,34)
        z=(xy-c)@view
        # Quantile framing is used solely to show the stage in the preview.
        bounds=np.quantile(z[:,:2],[.01,.99],axis=0)
        lo,hi=bounds
        padding=(hi-lo)*.085
        ext=[lo[0]-padding[0],hi[0]+padding[0],lo[1]-padding[1],hi[1]+padding[1]]
        panel(ax,xy,c,view,rgb,ext)
        ax.set_title(f"{CASES[k]}\n{len(xy):,} 点",loc="left",fontsize=12)
    fig.suptitle("全场实测点云｜转盘与物体同时保留",fontsize=18)
    save(fig,"full_scenes","Full scene exports, viewed at common oblique orientation per stage. Preview framing is quantile-based; PLY points are unchanged.",data)


def comparison():
    keys=["minion","apple","box_1116","box_center"]
    fig,axs=plt.subplots(3,4,figsize=(15,9),layout="constrained")
    inputs=[]
    for j,k in enumerate(keys):
        d=get(k)
        file=CLOUDS/k/"object_evaluation_points.npz"
        m=np.load(file)
        view=orient(d["axis"],0,90)
        origin=np.asarray(d["object_center"])
        ps=(m[d["selected"]]-origin)@view
        bound=np.max(np.abs(np.quantile(ps[:,:2],[.01,.99],axis=0)))*1.18
        for i,tag in enumerate(("archive_prior","unconstrained_icp",d["selected"])):
            p=m[tag]
            ax=axs[i,j]
            height=(p-origin)@np.asarray(d["axis"])
            lo,hi=np.percentile(height,[2,98]); hue=np.clip((height-lo)/max(hi-lo,1e-5),0,1)
            color=plt.get_cmap("viridis")(hue)[:,:3]
            panel(ax,p,origin,view,color,[-bound,bound,-bound,bound])
            a=d["diagnostics"][tag]
            lab=("档案位姿","自由 ICP","本轮选中")[i]
            ax.set_title(f"{CASES[k]}\n{lab} · 最大方位缺口 {a['camera_max_gap_deg']:.0f}°",fontsize=11,loc="left")
        inputs.extend([file,CLOUDS/k/"poses_and_metrics.json"])
    fig.suptitle("俯视核查｜自由 ICP 可以把相隔几面的点误叠在一起",fontsize=17)
    save(fig,"pose_comparison","Same observed object evaluation subset plotted under three pose hypotheses, with common within-object 2D extent. This diagnostic uses object points; all full PLYs retain the stage.",inputs)


def box_edges():
    d=get("box_1116")
    check=d["reference"]["rectangular_face_test"]
    edges=check["heldout_edge_gap_before_after"]
    fig,ax=plt.subplots(figsize=(7.6,4.1),layout="constrained")
    x=np.arange(4)
    ax.bar(x-.18,edges[0],.36,color="#d09460",label="盘面轴与名义角")
    ax.bar(x+.18,edges[1],.36,color="#238575",label="四面约束后")
    ax.set(xticks=x,xticklabels=["00↔01","01↔02","02↔03","03↔00"],ylabel="留出接缝间隙 / mm*")
    ax.grid(axis="y",alpha=.22)
    ax.legend(frameon=False)
    ax.set_title("11/16 纸盒｜四个相邻面的接缝",loc="left",fontsize=15)
    save(fig,"paper_box_edges","Held-out edge-to-edge horizontal gap on legacy box, before and after rectangular face pose fit.",[CLOUDS/"box_1116/poses_and_metrics.json"])


if __name__=="__main__":
    scenes()
    comparison()
    box_edges()
    (PLOT/"manifest.json").write_text(json.dumps(MANIFEST,indent=2,ensure_ascii=False)+"\n")
