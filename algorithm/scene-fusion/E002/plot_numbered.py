"""Render E002 scene, mask and paper-box comparisons from saved inputs."""
import csv
import hashlib
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
from PIL import Image

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
sys.path.insert(0,str(ROOT/"algorithm"))
from plot_registration_experiment import read_ply, camera_basis

OUT=HERE/"results"
FIG=OUT/"figures"
FIG.mkdir(exist_ok=True)
font_manager.fontManager.addfont("/Library/Fonts/Arial Unicode.ttf")
plt.rcParams.update({"font.family":"Arial Unicode MS","font.size":11,"axes.unicode_minus":False,
                     "axes.spines.top":False,"axes.spines.right":False,"text.color":"#163048"})
INDEX=json.loads((OUT/"E002-04.json").read_text())["cases"]
AUDIT=json.loads((HERE.parent/"results"/"optical_inputs.json").read_text())
MANIFEST={}


def digest(path):
    with path.open("rb") as f:return hashlib.file_digest(f,"sha256").hexdigest()


def finish(f,name,caption,sources):
    path=FIG/(name+".png")
    f.savefig(path,dpi=155,facecolor="white")
    plt.close(f)
    MANIFEST[name]={"filename":path.name,"sha256":digest(path),"caption":caption,
       "caption_sha256":hashlib.sha256(caption.encode()).hexdigest(),
       "sources":{str(p.relative_to(ROOT)):digest(p) for p in sources},"script":digest(Path(__file__))}


def scene_panel(ax,p,rgb,origin,n,yaw,elevation,limits=None,bg="#f2f6f9"):
    b=camera_basis(np.array(n),yaw,elevation)
    q=(p-origin)@b
    if limits is None:
        a,c=np.quantile(q[:,:2],[.012,.988],axis=0)
        v=(c-a)*.11+1
        limits=(a-v,c+v)
    ix=np.arange(0,len(q),max(1,len(q)//18000))
    ix=ix[np.argsort(q[ix,2])]
    ax.scatter(q[ix,0],q[ix,1],s=.74,c=rgb[ix],alpha=.78,linewidths=0)
    (a,b),(c,d)=limits
    ax.set(xlim=(a,c),ylim=(b,d),aspect="equal",xticks=[],yticks=[])
    ax.set_facecolor(bg)
    for e in ax.spines.values():e.set_visible(False)


def collect(tag):
    f=OUT/"clouds"/tag
    return f,json.loads((f/f"{tag}-03.json").read_text())


def all_scenes():
    f,axs=plt.subplots(4,4,figsize=(13,11),layout="constrained")
    sources=[]
    for j,tag in enumerate(INDEX):
        folder,d=collect(tag)
        p1,p2=(folder/f"{tag}-{x:02}.ply" for x in (1,2))
        sources.extend((p1,p2))
        for i,p in enumerate((p1,p2)):
            ax=axs.flat[j*2+i]
            xyz,c=read_ply(p)
            scene_panel(ax,xyz,c,np.array(d["center"]),d["axis"],-46,29)
            ax.set_title(f"{tag[-3:]} · "+("全场" if i==0 else "去盘"),fontsize=17,loc="left")
    f.suptitle("E002｜同一批实测点的全场与物体结果",fontsize=19)
    finish(f,"E002-10","All eight sessions. Full calibrated scene and selected object mask. Axes are fit to each panel for preview.",sources)


def seg_scores():
    file=OUT/"E002-01.csv"
    with file.open(encoding="utf-8-sig") as p:z=list(csv.DictReader(p))
    ms=sorted(set(a["method"] for a in z))
    macro=[];low=[]
    for m in ms:
        r=[x for x in z if x["method"]==m]
        macro.append(100*np.mean([float(t["f1"]) for t in r]))
        low.append(100*sum(int(t["low_tp"]) for t in r)/sum(int(t["low_n"]) for t in r))
    f,axs=plt.subplots(1,2,figsize=(11,3.9),layout="constrained")
    for ax,values,title,lim,names in zip(axs,(macro,low),("参考轮廓内外的分割 F1","低高度标签点保留率"),(100,72),("%","%")):
        colors=["#88a9be"]*(len(ms)-1)+["#248473"]
        ax.bar(ms,values,color=colors,width=.68)
        ax.set(ylim=(0,lim),ylabel=names)
        ax.tick_params(labelsize=13)
        ax.set_title(title,loc="left",fontsize=15)
        ax.grid(axis="y",alpha=.15)
        for x,y in enumerate(values):
            ax.text(x,y+1,f"{y:.1f}",ha="center",fontsize=13)
    f.suptitle("E002｜16 张参考图，七套去盘方法",fontsize=17)
    finish(f,"E002-11","Segmentation metrics. F1 is macro averaged across 16 labeled frames; the near-contact retention rate pools reference samples.",[file])


def boxes():
    f,axs=plt.subplots(4,4,figsize=(12.3,11.5),layout="constrained")
    sources=[]
    for j,tag in enumerate(("E002-S03","E002-S06","E002-S07","E002-S08")):
        folder,d=collect(tag)
        origin=np.asarray(d["center"])
        normal=d["axis"]
        main,_=read_ply(folder/f"{tag}-02.ply")
        tr=camera_basis(np.array(normal),-32,29)
        lo,hi=np.quantile((main-origin)@tr[:,:2],[.006,.994],axis=0)
        pad=(hi-lo)*.14+2
        for i,method in enumerate(("R01","R06","R07","R08")):
            a=axs[j,i]
            if method not in d["variants"]:
                a.axis("off")
                a.text(.5,.5,"无同期 ArUco",ha="center",va="center",fontsize=16)
                continue
            selected=method==d["registration"]
            path=folder/f"{tag}-{'02' if selected else method}.ply"
            p,c=read_ply(path)
            scene_panel(a,p,c,origin,normal,-32,29,(lo-pad,hi+pad))
            a.set_title(f"{tag[-3:]} · {method}"+("  采用" if selected else ""),fontsize=18,loc="left")
            sources.append(path)
        sources.append(folder/f"{tag}-03.json")
    f.suptitle("E002｜四组纸盒，同一剔除掩膜下比较位姿",fontsize=17)
    finish(f,"E002-12","Box poses on identical segmented observations and per-row scale. Compare R01 baseline, R06 gated graph, R07 shape only, and R08 shape plus training markers.",sources)


def photo_overlays():
    f,axs=plt.subplots(2,4,figsize=(13,5.8),layout="constrained")
    sources=[]
    for j,(tag,index) in enumerate(INDEX.items()):
        folder,d=collect(tag)
        v0=0
        key=d["case"]
        meta=AUDIT[key]
        source=ROOT/meta["path"]/meta["views"][v0]["frame"]/"graycode_42.png"
        mask_file=OUT/"cache"/tag/"V00.npz"
        data_file=HERE.parent/"results"/"intermediate"/key/"00.npz"
        with np.load(mask_file) as m,np.load(data_file) as pkg:
            good=m[d["separation"]]
            uv=pkg["uv"]
        image=Image.open(source).convert("L").resize((408,342))
        ax=axs.flat[j]
        ax.imshow(image,cmap="gray",extent=(0,2448,2048,0))
        dx=uv[good][::max(1,int(good.sum())//1200)]
        ax.scatter(dx[:,0],dx[:,1],s=1.15,linewidths=0,color="#41dcac",alpha=.78)
        ax.set_xlim(300,1780);ax.set_ylim(1900,530)
        ax.axis("off")
        ax.set_title(f"{tag[-3:]} · {d['separation']}",loc="left",fontsize=16)
        sources.extend((source,mask_file,data_file))
    f.suptitle("E002｜原始白场与剔除后保留的有效点",fontsize=17)
    finish(f,"E002-13","Green points show the retained actual depth pixels overlaying the raw white-field image, frame 0 of eight sessions.",sources)


def rotation_views():
    f,axs=plt.subplots(4,4,figsize=(13,10.4),layout="constrained")
    sources=[]
    for j,tag in enumerate(INDEX):
        folder,d=collect(tag)
        source=folder/f"{tag}-02.ply"
        p,c=read_ply(source)
        for i,(di,el,label) in enumerate(((-46,29,"斜视"),(110,57,"反向俯视"))):
            ax=axs.flat[j*2+i]
            scene_panel(ax,p,c,np.array(d["center"]),d["axis"],di,el)
            ax.set_title(f"{d['label']} · {label}",loc="left")
        sources.extend((source,folder/f"{tag}-03.json"))
    f.suptitle("E002｜八组实测物体的两个观察方向",fontsize=17)
    finish(f,"E002-14","Same selected segmented observations rendered from distinct azimuths and elevations for each of eight sessions.",sources)


if __name__=="__main__":
    all_scenes()
    seg_scores()
    boxes()
    photo_overlays()
    rotation_views()
    (FIG/"E002.json").write_text(json.dumps(MANIFEST,indent=2,ensure_ascii=False)+"\n")
