"""Render identical-scale orthographic plots from the exported PLY point clouds."""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.lines import Line2D
import numpy as np

from plot_registration_experiment import camera_basis, read_ply

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/"output/calibrated-benchmark"
FIG=OUT/"figures"
PALETTE=["#2e6fc4","#ec6a35","#249e74","#9c5aac","#d19e32","#1b9db5","#d85389","#647184"]
CAMS=((-22,25),(150,25),(0,90))
CAM_NAMES=("斜前","斜后","俯视")
NAMES={"axis_step":"转轴 + 指令角","aruco":"ArUco PnP","axis_refine":"约束角度配准",
       "point_to_point":"点到点 ICP","point_to_plane":"点到面 ICP","tukey_icp":"Tukey ICP",
       "gicp":"GICP","gray_icp":"灰度 ICP","fpfh_ransac":"FPFH + RANSAC","fpfh_fgr":"FPFH + FGR","pose_graph":"标记锚定图"}
CASES={"174129":"充电头", "174531":"居中盒", "174747":"偏心盒", "175006":"17:50 盒"}
INK="#142b40"
MUTED="#54687e"
font_manager.fontManager.addfont("/Library/Fonts/Arial Unicode.ttf")
plt.rcParams.update({"font.family":"Arial Unicode MS","axes.unicode_minus":False,"font.size":9,"axes.spines.top":False,
                     "axes.spines.right":False,"axes.labelcolor":INK,"text.color":INK,"xtick.color":MUTED,
                     "ytick.color":MUTED,"axes.edgecolor":"#afbbca","pdf.fonttype":42})


def plot_clouds(sid,m,run=None,target=None,methods=None,reference="aruco"):
    run=run or OUT/"runs"/sid
    target=target or FIG/sid
    methods=methods or NAMES
    base,_=read_ply(run/reference/"all_views.ply")
    origin=np.median(base,axis=0)
    up=np.array(m["axis"])
    if up[2]>0:
        up=-up
    cameras=[camera_basis(up,yaw,elev) for yaw,elev in CAMS]
    limits=[]
    for V in cameras:
        p=(base-origin)@V
        robust=np.quantile(p[:,:2],[.015,.985],axis=0)
        center=robust.mean(0)
        height=max(42, np.max((robust[1]-robust[0])/[1.65,1]) * 1.18)
        limits.append((center,np.array([1.65,1]) * height/2))
    for method in methods:
        fig,axs=plt.subplots(2,3,figsize=(9,3.85),gridspec_kw={"wspace":.045,"hspace":.085})
        fig.patch.set_alpha(0)
        fig.subplots_adjust(left=.009,right=.995,bottom=.025,top=.94)
        for r,filename in enumerate(("pair_00_01.ply","all_views.ply")):
            pts,rgb=read_ply(run/method/filename)
            for c,(ax,V,lim) in enumerate(zip(axs[r],cameras,limits)):
                loc=(pts-origin)@V
                center,half=lim
                keep=(np.abs(loc[:,:2]-center)<=half).all(1)
                order=np.where(keep)[0][np.argsort(loc[keep,2])]
                ax.set_facecolor("#f0f4f8")
                ax.scatter(loc[order,0],loc[order,1],c=rgb[order],s=1.85 if r==0 else 1.3,linewidths=0,alpha=.72)
                ax.set(xlim=(center[0]-half[0],center[0]+half[0]),ylim=(center[1]-half[1],center[1]+half[1]),
                       xticks=[],yticks=[],aspect="equal")
                for spine in ax.spines.values():
                    spine.set_visible(False)
                if r==0:
                    ax.set_title(CAM_NAMES[c],fontsize=9,loc="left",pad=4,color=MUTED)
                if keep.mean()<.985:
                    ax.text(.98,.06,f"{100*(1-keep.mean()):.0f}% 框外",transform=ax.transAxes,ha="right",fontsize=6.6,color="#904228")
                if c==0:
                    x=center[0]-half[0]*.86;y=center[1]-half[1]*.8
                    ax.plot([x,x+10],[y,y],color=MUTED,lw=1.05,solid_capstyle="butt")
                    ax.text(x,y+half[1]*.11,"10 mm*",fontsize=6.7,color=MUTED)
        target.mkdir(parents=True,exist_ok=True)
        fig.savefig(target/f"{method}.png",dpi=190,transparent=True)
        plt.close(fig)


def optics(models):
    fig,ax=plt.subplots(figsize=(8.8,2.05))
    colors=["#148474","#3576b4","#c27c3c","#8e60a9"]
    for j,(sid,m) in enumerate(models.items()):
        for k,row in enumerate(m["statistics"]):
            x=j*10+k
            q=row["table_reprojection_p50_p95"]
            ax.plot([x,x],[q[1][0],q[0][0]],lw=.8,color=colors[j],alpha=.24)
            ax.scatter(x,q[0][0],marker="_",s=47,c=colors[j])
            ax.scatter(x,q[1][0],s=13,c=colors[j])
    ax.set(yscale="log",ylim=(.72,860),ylabel="投影残差 P50 / px",
           xticks=np.arange(4)*10+3.5,xticklabels=[f"{sid[0:2]}:{sid[2:4]} {CASES[sid]}" for sid in models])
    ax.grid(axis="y",alpha=.16)
    handles=[Line2D([0],[0],marker="_",c=MUTED,lw=0,markersize=9,label="原外参"),
             Line2D([0],[0],marker="o",c=MUTED,lw=0,markersize=4,label="重估后")]
    ax.legend(handles=handles,loc="center left",bbox_to_anchor=(1,.5),frameon=False,fontsize=8)
    fig.subplots_adjust(left=.095,right=.845,bottom=.19,top=.98)
    fig.savefig(FIG/"optics.png",dpi=185,transparent=True)
    plt.close(fig)


def overview():
    rows=list(csv.DictReader((OUT/"summary.csv").open()))
    by={m:[r for r in rows if r["method"]==m] for m in NAMES}
    fig,ax=plt.subplots(figsize=(4.2,2.5))
    for i,(method,rs) in enumerate(by.items(),1):
        x=np.median([float(r["tag_p50_px"]) for r in rs])
        y=np.median([float(r["f1_2"])*100 for r in rs])
        color="#168674" if method in ("aruco","pose_graph") else ("#466d9b" if i<9 else "#ae6235")
        ax.scatter(x,y,s=32,color=color,alpha=.9)
        offsets={2:(-5,9),11:(6,-11),4:(-3,-12),7:(-6,9),9:(7,9),1:(-6,-10),10:(8,6),5:(-4,-11),8:(-3,9)}
        ax.annotate(f"{i:02}",(x,y),xytext=offsets.get(i,(6,7)),textcoords="offset points",
                    ha="center",va="center",fontsize=7.9,color=color)
    ax.set(xscale="log",ylabel="整圈 F1@2 mm* / %",xlabel="留出码重投影 P50 / px",xlim=(3.2,2200),ylim=(0,58))
    ax.grid(alpha=.19)
    fig.tight_layout()
    fig.savefig(FIG/"tradeoff.png",dpi=185,transparent=True)
    plt.close(fig)


def stress():
    rows=list(csv.DictReader((OUT/"initial_pose_stress.csv").open()))
    fig,ax=plt.subplots(figsize=(4.4,1.75))
    for k,method in enumerate(("point_to_point","point_to_plane","gicp","gray_icp")):
        x=[0,5,12,25]
        y=[]
        for a in x:
            matches=[r for r in rows if abs(int(r["perturbation_deg"]))==a and r["method"]==method]
            y.append(np.median([100*float(r["eval_f1_2"]) if r["status"]=="scored" else 0. for r in matches]))
        ax.plot(x,y,label=NAMES[method],marker="o",ms=3,lw=1.5,color=PALETTE[k])
    ax.set(xlabel="初值绕轴扰动绝对值 / °",ylabel="两视角 F1@2 / %",xticks=[0,5,12,25],ylim=(0,85))
    ax.grid(alpha=.19)
    ax.legend(loc="upper right",frameon=False,fontsize=6,ncol=2)
    fig.tight_layout(pad=.65)
    fig.savefig(FIG/"stress.png",dpi=185,transparent=True)
    plt.close(fig)


def main():
    FIG.mkdir(exist_ok=True)
    models=json.loads((OUT/"recovery/model.json").read_text())
    for sid,m in models.items():
        plot_clouds(sid,m)
    optics(models)
    overview()
    stress()


if __name__ == "__main__":
    main()
