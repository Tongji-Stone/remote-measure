"""Build per-object benchmark visuals using the same point-cloud cameras as the combined report."""
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

from plot_calibrated_benchmark import FIG, OUT as ARUCO, NAMES, PALETTE, plot_clouds, optics
from plot_registration_experiment import camera_basis, read_ply

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/"output/object-benchmarks"
LABELS={**NAMES,"axis_step":"档案转轴","rim_axis":"盘缘修正"}


def rows(p):
    return list(csv.DictReader(p.open()))


def native_figures(key):
    d=OUT/key
    model=json.loads((d/"experiment.json").read_text())
    tab=rows(d/"summary.csv")
    plot_clouds(key,model,run=d/"runs",target=d/"figures",
                methods=[r["method"] for r in tab],reference="rim_axis" if key!="legacy-box" else "axis_step")
    fig, ax = plt.subplots(figsize=(5.5,2.4))
    dataset=rows(d/"initial_pose_stress.csv")
    for j,method in enumerate(("point_to_point","point_to_plane","gicp","gray_icp")):
        x=[0,5,12,25]
        y=[np.median([100*float(v["f1_2"]) for v in dataset if v["method"]==method and abs(int(v["initial_yaw"]))==a]) for a in x]
        ax.plot(x,y,label=LABELS[method],color=PALETTE[j],lw=1.5,marker="o",ms=3)
    ax.set(xlabel="初值绕轴扰动 / °",ylabel="双帧 F1@2 / %",xticks=[0,5,12,25],ylim=(-2,93))
    ax.grid(alpha=.18)
    ax.legend(frameon=False,fontsize=8,ncol=2)
    fig.tight_layout(pad=.65)
    fig.savefig(d/"figures/stress.png",dpi=190)
    plt.close(fig)
    if key=="legacy-box":
        return
    outcome_chart(d,tab)


def outcome_chart(directory, table):
    fig,ax=plt.subplots(figsize=(4.65,2.4))
    by_method={r["method"]:(i,r) for i,r in enumerate(table,1)}
    select=("axis_step","rim_axis","point_to_point","gicp","fpfh_ransac","fpfh_fgr")
    width=.34
    group=np.arange(len(select))
    picked=[by_method[m] for m in select]
    ax.bar(group-width/2,[100*float(r["f1_2"]) for _,r in picked],width,label="全圈邻边",color="#3479a9")
    values=[100*float(r["repeat_f1_2"]) if r["repeat_f1_2"] else np.nan for _,r in picked]
    ax.bar(group+width/2,values,width,label="实拍 360° 预测",color="#219584")
    ax.set(ylabel="F1@2 / %",xlabel="算法编号",xticks=group,xticklabels=[f"{i:02}" for i,_ in picked],ylim=(0,100))
    ax.grid(axis="y",alpha=.16)
    ax.legend(frameon=False,fontsize=9,loc="upper left",ncol=2)
    fig.tight_layout(pad=.55)
    fig.savefig(directory/"figures/outcomes.png",dpi=190)
    plt.close(fig)


def legacy_panel(method,model):
    d=OUT/"legacy-box"
    base,_=read_ply(d/"runs/axis_step/all_views.ply")
    origin=np.median(base,axis=0)
    axis=np.asarray(model["axis"])
    if axis[1]>0:
        axis=-axis
    settings=[(-16,18,"双帧 · 斜前"),(-16,18,"全圈 · 斜前"),(10,90,"全圈 · 俯视")]
    fig,axs=plt.subplots(1,3,figsize=(9,1.6))
    for i,(yaw,elev,name) in enumerate(settings):
        ax=axs[i]
        basis=camera_basis(axis,yaw,elev)
        s=(base-origin)@basis
        lo,hi=np.quantile(s[:,:2],[.015,.985],axis=0)
        center=(hi+lo)/2
        height=max((hi[0]-lo[0])/1.8,hi[1]-lo[1])*1.25
        half=np.array([1.8,1])*height/2
        p,rgb=read_ply(d/"runs"/method/("pair_00_01.ply" if i==0 else "all_views.ply"))
        q=(p-origin)@basis
        keep=(np.abs(q[:,:2]-center)<=half).all(1)
        ind=np.flatnonzero(keep)
        ind=ind[np.argsort(q[ind,2])]
        ax.scatter(q[ind,0],q[ind,1],c=rgb[ind],s=.8,linewidths=0,alpha=.76)
        if keep.mean()<.985:
            ax.text(.98,.08,f"{100*(1-keep.mean()):.0f}% 框外",transform=ax.transAxes,ha="right",fontsize=8,color="#904228")
        ax.set_title(name,fontsize=9,loc="left",pad=3)
        ax.set(xlim=(center[0]-half[0],center[0]+half[0]),ylim=(center[1]-half[1],center[1]+half[1]),aspect="equal",xticks=[],yticks=[])
        for sp in ax.spines.values(): sp.set_visible(False)
    fig.subplots_adjust(left=.006,right=.998,bottom=.035,top=.87,wspace=.035)
    fig.savefig(d/f"figures/strip_{method}.png",dpi=185)
    plt.close(fig)


def box_inputs():
    from PIL import Image, ImageDraw
    modules=json.loads((ARUCO/"recovery/model.json").read_text())
    cells=Image.new("RGB",(1072,232),"#121d2b")
    left=Image.open(OUT/"legacy-box/input.jpg").convert("RGB")
    cells.paste(left.crop((0,0,306,256)).resize((268,224)),(0,0))
    for j,k in enumerate(("174531","174747","175006"),1):
        path=ROOT/modules[k]["scan"]/"view_0000/graycode_42.png"
        im=Image.open(path).convert("RGB").resize((268,224))
        cells.paste(im,(268*j,0))
    draw=ImageDraw.Draw(cells)
    for j,label in enumerate(("11/16 17:40","11/23 17:45","11/23 17:47","11/23 17:50")):
        draw.rectangle((268*j+5,5,268*j+105,25),fill="#18344d")
        draw.text((268*j+10,9),label,fill="white")
    target=OUT/"box"
    target.mkdir(exist_ok=True)
    cells.save(target/"input.jpg",quality=91)


if __name__=="__main__":
    for case in ("minion","apple","legacy-box"):
        native_figures(case)
    model=json.loads((OUT/"legacy-box/experiment.json").read_text())
    for method in [r["method"] for r in rows(OUT/"legacy-box/summary.csv")]:
        legacy_panel(method,model)
    box_inputs()
