"""Visual QA and benchmark plots for E002 real-scan completion."""
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

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path.insert(0,str(HERE.parent))
from plot import read_ply, basis, picture

OUT=HERE/"results"
FIG=OUT/"figures"
FIG.mkdir(exist_ok=True)
display=("小黄人","苹果","纸盒 11/16","充电头 17:16","充电头 17:41","纸盒 17:45","纸盒 17:47","纸盒 17:50")
font_manager.fontManager.addfont("/Library/Fonts/Arial Unicode.ttf")
plt.rcParams.update({"font.family":"Arial Unicode MS","font.size":11})
details={}


def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def save(fig,id,caption,files):
    target=FIG/f"{id}.png"
    fig.savefig(target,dpi=155,facecolor="white")
    plt.close(fig)
    details[id]={"sha256":digest(target),"caption":caption,"caption_sha256":hashlib.sha256(caption.encode()).hexdigest(),
        "inputs":{str(f.relative_to(ROOT)):digest(f) for f in files},"source":digest(Path(__file__))}


def gallery():
    f,axs=plt.subplots(4,4,figsize=(13.8,10.4),layout="constrained")
    inputs=[]
    preferred={"S02":"C06","S03":"C04","S06":"C04","S07":"C04","S08":"C04"}
    for j in range(1,len(display)+1):
        sid=f"S{j:02}"
        tag=f"E002-{sid}"
        df=OUT/tag
        model=preferred.get(sid,"C03")
        a,b=df/f"{tag}-C00.ply",df/f"{tag}-{model}-T0.ply"
        inputs.extend((a,b))
        p,c=read_ply(a); q,d=read_ply(b)
        view=basis(36)
        proj=np.concatenate((p,q))@view
        lo,hi=np.quantile(proj[:,:2],[.005,.995],axis=0)
        expand=.10*(hi-lo)+2
        lo-=expand;hi+=expand
        extent=(lo[0],hi[0],lo[1],hi[1])
        i=(j-1)*2
        picture(axs.flat[i],p,c,view,extent)
        picture(axs.flat[i+1],q,d,view,extent)
        axs.flat[i].set_title(f"{sid} · 实测",fontsize=18,loc="left")
        axs.flat[i+1].set_title(f"{sid} · {model}",fontsize=18,loc="left")
    f.suptitle("E002｜真实扫描补全，橙色表示缺少近邻实测支撑",fontsize=17)
    save(f,"E002-10","Measured object-only point cloud vs one shape candidate per captured session. Orange marks predicted points farther than 2.5 mm* from measured samples.",inputs)


def occlusion():
    p=OUT/"E002-01.csv"
    with p.open(encoding="utf-8-sig") as file:r=list(csv.DictReader(file))
    fig,ax=plt.subplots(figsize=(11,4.0),layout="constrained")
    xx=np.arange(8)
    for j,method in enumerate(("C01","C02","C03","C04","C06")):
        data=[next((float(o["hidden_recall_2"])*100 for o in r if o["test"]=="1" and o["id"]==f"E002-S{s+1:02}" and o["method"]==method and o["hidden_recall_2"]),np.nan) for s in xx]
        ax.bar(xx+(j-2)*.15,np.array(data),.135,label=method,color=("#7190a7","#1e6a9b","#25997d","#b36e46","#8964a3")[j])
    ax.set(xticks=xx,xticklabels=[f"S{i+1:02}" for i in xx],ylim=(0,104))
    ax.set_ylabel("2 mm* 召回 / %",fontsize=18)
    ax.tick_params(labelsize=18)
    ax.grid(axis="y",alpha=.18)
    ax.set_title("遮挡测试｜实测表面恢复",loc="left",fontsize=20)
    ax.legend(frameon=False,ncol=5,fontsize=15,loc="upper right")
    save(fig,"E002-11","Recall on a deliberately withheld measured patch. Void columns represent inapplicable shape priors.",[p])


if __name__=="__main__":
    gallery()
    occlusion()
    (FIG/"E002.json").write_text(json.dumps(details,ensure_ascii=False,indent=2)+"\n")
