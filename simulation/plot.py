"""Render volume comparisons and sampled predicted surfaces."""
import csv
import json
from hashlib import sha256
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

HERE=Path(__file__).resolve().parent
OUT=HERE/"results"
PLOT=OUT/"figures"
PLOT.mkdir(exist_ok=True)
font_manager.fontManager.addfont("/Library/Fonts/Arial Unicode.ttf")
plt.rcParams.update({"font.family":"Arial Unicode MS","font.size":11,"axes.unicode_minus":False,
                     "axes.spines.top":False,"axes.spines.right":False,"text.color":"#163048"})
CASES={"A_box":"A · 长方体","B_features":"B · 台阶、槽与孔","C_freeform":"C · 平底自由曲面"}
SCENARIOS={"four_low":"4 低视角 + 阴影","eight_low":"8 低视角","eight_plus_high":"8 低 + 2 高"}
TITLES={"convex_base":"凸包 + 平底","visual_hull":"白场轮廓体","visual_hull_depth":"轮廓 + 深度","heightfield_base":"高度场 + 平底",
        "heightfield_depth":"高度场 + 深度约束","cuboid_fit":"长方体拟合"}
MANIFEST={}


def digest(p):
    return sha256(p.read_bytes()).hexdigest()


def export(fig,stem,description,files):
    path=PLOT/f"{stem}.png"
    fig.savefig(path,dpi=160,facecolor="white")
    plt.close(fig)
    MANIFEST[stem]={"image":path.name,"sha256":digest(path),"caption":description,
        "caption_sha256":sha256(description.encode()).hexdigest(),"inputs":{str(x.relative_to(HERE)):digest(x) for x in files},
        "script_sha256":digest(Path(__file__))}


def read_ply(p):
    types={"double":"<f8","float":"<f4","uchar":"u1","uint8":"u1"}
    with p.open("rb") as f:
        fields=[]
        for line in f:
            if line.startswith(b"property"):
                _,t,k=line.decode().split()
                fields.append((k,types[t]))
            if line.strip()==b"end_header": break
        a=np.fromfile(f,dtype=np.dtype(fields))
    xyz=np.c_[a["x"],a["y"],a["z"]]
    color=np.c_[a["red"],a["green"],a["blue"]]/255. if "red" in a.dtype.names else np.tile([.30,.50,.65],(len(a),1))
    return xyz,color


def score_plot():
    path=OUT/"benchmark.csv"
    with path.open() as f: rows=list(csv.DictReader(f))
    fig,axs=plt.subplots(1,3,figsize=(14,4.6),layout="constrained",sharey=True)
    colors=("#bd724d","#3f7ba5","#228676")
    for i,(case,ax) in enumerate(zip(CASES,axs)):
        options=[k for k in TITLES if any(r["case"]==case and r["method"]==k for r in rows)]
        xx=np.arange(len(options))
        for j,key in enumerate(SCENARIOS):
            vals=[float(next(r["volume_error_pct"] for r in rows if r["case"]==case and r["scenario"]==key and r["method"]==m)) for m in options]
            ax.bar(xx+(j-1)*.23,vals,.21,color=colors[j],label=SCENARIOS[key])
        ax.set(xticks=xx,xticklabels=[TITLES[m] for m in options],ylim=(0,45))
        ax.tick_params(axis="x",rotation=34)
        ax.grid(axis="y",alpha=.16)
        ax.set_title(CASES[case],loc="left",fontsize=15)
        if i==0: ax.set_ylabel("相对体积偏高 / %")
        if i==2: ax.legend(frameon=False,fontsize=10)
    fig.suptitle("遮挡体积模拟｜同一目标的三种采集条件",fontsize=17)
    export(fig,"volume_errors","Signed volume error (all plotted estimates are above reference). Unclosed Poisson meshes have no valid volume and are omitted.",[path,OUT/"parameters.json"])


def basis(elev):
    yaw,a=np.deg2rad([33,elev])
    right=np.array([np.cos(yaw),np.sin(yaw),0])
    depth=np.cross(right,[0,0,1])
    up=np.cos(a)*np.array([0.,0.,1.])-np.sin(a)*depth
    return np.c_[right,up,np.cross(right,up)]


def picture(ax,xyz,c,b,lim):
    p=xyz@b
    if len(p)>16000:
        i=np.linspace(0,len(p)-1,16000).astype(int)
        p,c=p[i],c[i]
    z=np.argsort(p[:,2])
    ax.scatter(p[z,0],p[z,1],c=c[z],s=1.8,linewidths=0,alpha=.82)
    a,d,e,f=lim
    ax.set(xlim=(a,d),ylim=(e,f),aspect="equal",xticks=[],yticks=[])
    for sp in ax.spines.values():sp.set_visible(False)


def prediction_plot():
    fig,axs=plt.subplots(3,3,figsize=(13,9),layout="constrained")
    inputs=[]
    for col,key in enumerate(CASES):
        base=OUT/key/"four_low"
        method="cuboid_fit" if col==0 else "heightfield_depth"
        p1,p2=base/"observed.ply",base/f"{method}_points.ply"
        raw,cr=read_ply(p1); prediction,cp=read_ply(p2)
        inputs.extend([p1,p2])
        for row,(x,c,elev,label) in enumerate(((raw,cr,29,"观测点"),(prediction,cp,29,"推断的完整表面"),(prediction,cp,-30,"底面 / 未测区域"))):
            ax=axs[row,col]
            b=basis(elev)
            values=prediction@b
            l,h=values[:,:2].min(0),values[:,:2].max(0)
            pad=(h-l)*.12+1
            picture(ax,x,c,b,[l[0]-pad[0],h[0]+pad[0],l[1]-pad[1],h[1]+pad[1]])
            ax.set_title(f"{CASES[key]}  ·  {label}",loc="left",fontsize=12)
    fig.suptitle("模拟补全｜橙点离最近实测点超过 2.5 mm",fontsize=17)
    export(fig,"completion_examples","Simulated point clouds for four low camera views with an unilluminated patch. Orange colors label predictions not directly supported by a nearby measured sample.",inputs)


if __name__=="__main__":
    score_plot()
    prediction_plot()
    (PLOT/"manifest.json").write_text(json.dumps(MANIFEST,ensure_ascii=False,indent=2)+"\n")
