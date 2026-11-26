"""Traceable figures for M01 and V001. Run with the scientific Matplotlib environment."""
import csv
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
VOLUME=ROOT/'computing volume/V001'
DATA=HERE/'results'
SCI=Path.home()/'.local/share/codex-science'
sys.path.insert(0,str(SCI/'templates/rectangle/figures/source'))
from figure_qa import export_checked

COLORS={'D0':'#4c647b','D3':'#bf7444','D4':'#137f76',
        'R02':'#4c647b','R06c':'#337bbb','H1':'#bf7444',
        'S04':'#4c647b','S07':'#337bbb','R-pole':'#137f76','R-flat':'#bf7444'}
ORDER=('P1','P2','F1','K1','C1')


def load(path):
    with path.open() as source:
        return list(csv.DictReader(source))


def save(fig,directory,name,caption,paths,semantics):
    stem=directory/name
    qa,_=export_checked(fig,stem,caption=caption,data_paths=paths,
                         source_path=__file__,command=f'{sys.executable} {Path(__file__)}',
                         max_width_mm=180,target_width_mm=180,pdf_python=SCI/'venv/bin/python',
                         semantics=semantics)
    print(name,qa['automatic_status'],
          [(x.get('code'),x.get('text')) for x in qa['issues']],flush=True)
    plt.close(fig)


def style():
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':9,'axes.labelsize':9,
                         'axes.titlesize':10,'axes.spines.top':False,'axes.spines.right':False,
                         'axes.axisbelow':True,'savefig.facecolor':'white','axes.labelcolor':'#263c4b',
                         'axes.edgecolor':'#9eabb5','grid.color':'#dae4eb','grid.linewidth':.55,
                         'pdf.fonttype':42,'legend.fontsize':9,'xtick.labelsize':9,'ytick.labelsize':9})


def grouped(ax,labels,lookup,methods,metric,multiplier=1):
    x=np.arange(len(labels))
    width=.78/len(methods)
    bars=[]
    for i,m in enumerate(methods):
        v=[float(lookup[k,m][metric])*multiplier for k in labels]
        bars.append(ax.bar(x+(i-(len(methods)-1)/2)*width,v,width*.91,color=COLORS[m],label=m))
    ax.set_xticks(x,labels)
    ax.grid(axis='y')
    return bars


def biases():
    f=DATA/'injected_bias.csv'
    raw=load(f)
    order=('SPH','ELL','TOR')
    kinds=('uniform','localized','hidden_cap')
    labels=('Uniform bias','Localized bias','Hidden cap')
    fig,axs=plt.subplots(1,3,figsize=(180/25.4,70/25.4),sharey=True)
    for ax,trial,label in zip(axs,kinds,labels):
        d={(q['case'],q['method']):q for q in raw if q['trial']==trial}
        bars=grouped(ax,order,d,('D0','D3','D4'),'relative_error',100)
        ax.set_title(label)
        ax.set_ylim(-.8,8.3)
        ax.axhline(0,color='#798a95',linewidth=.5)
        ax.set_xlabel('Known shape')
    axs[0].set_ylabel('Signed volume error (%)')
    fig.legend(bars,('D0  baseline','D3  geometry selection','D4  volume selection'),
               loc='upper center',ncol=3,frameon=False,bbox_to_anchor=(.54,.99))
    fig.subplots_adjust(left=.095,right=.99,bottom=.22,top=.76,wspace=.12)
    save(fig,DATA/'figures','M01-bias',
         '同一网格注入偏移后的体积误差。均匀、局部和隐藏底盖分别展示。D3 按几何误差选型，D4 使用可见域有符号面积积分。环体为 TOR。',
         [f],{m:COLORS[m] for m in ('D0','D3','D4')}|{'ordinate':'percent'})


def poses():
    paths=[DATA/f'pose_{c}.csv' for c in ORDER]
    data={(r['case'],r['method']):r for p in paths for r in load(p)}
    for c in ORDER:
        v0=float(data[c,'R02']['volume_ml'])
        for m in ('R02','R06c','H1'):
            data[c,m]['volume_change_pct']=100*(float(data[c,m]['volume_ml'])/v0-1.)
    fig,ax=plt.subplots(1,3,figsize=(180/25.4,77/25.4))
    keys=('R02','R06c','H1')
    bars=grouped(ax[0],ORDER,data,keys,'p95_mm')
    ax[0].set_ylabel('Held-out P95 (mm)')
    ax[0].set_ylim(0,1.18)
    ax[0].set_title('Surface fit')
    grouped(ax[1],ORDER,data,keys,'held_marker_p50_px')
    ax[1].set_ylabel('Marker P50 (px)')
    ax[1].set_ylim(0,8.6)
    ax[1].set_title('Fiducial consistency')
    grouped(ax[2],ORDER,data,keys,'volume_change_pct')
    ax[2].set_ylabel('Change vs. R02 (%)')
    ax[2].set_ylim(-.15,7.2)
    ax[2].set_title('Volume change')
    fig.legend(bars,keys,loc='upper center',ncol=3,frameon=False,bbox_to_anchor=(.54,.99))
    fig.subplots_adjust(left=.093,right=.99,bottom=.16,top=.77,wspace=.58)
    save(fig,DATA/'figures','M01-poses',
         '同一批单站点、 E102 盘面和 S04 重建核。H1 使五组留出表面 P95 下降、留出码 P50 上升，同时使体积相对共轴 R02 上升。实物体积无独立真值。',
         paths,{m:COLORS[m] for m in keys}|{'surface':'mm','volume':'percent'})


def sections():
    f=DATA/'sections.csv'
    control=ROOT/'simulation/E104/场景/G3/结果.json'
    data={(r['trial'],r['method']):r for r in load(f) if r['case']=='G3'}
    g=json.loads(control.read_text())['tests']
    for trial in ('T0','T1','T2'):
        for m in ('S04','S07'):
            data[trial,m]={'relative_error':g[trial]['methods'][m]['volume_relative_error']}
    fig,ax=plt.subplots(figsize=(180/25.4,72/25.4))
    keys=('S04','S07','R-flat','R-pole')
    bars=grouped(ax,('T0','T1','T2'),data,keys,'relative_error',100)
    ax.set_ylabel('Signed volume error (%)')
    ax.set_xlabel('Visibility condition')
    ax.set_ylim(-1.8,53)
    ax.axhline(0,color='#798a95',linewidth=.5)
    fig.legend(bars,keys,loc='upper center',ncol=4,frameon=False,bbox_to_anchor=(.54,.98))
    fig.subplots_adjust(left=.10,right=.98,bottom=.225,top=.83)
    save(fig,DATA/'figures','M01-sections',
         'G3 哑光几何控制的封闭包络与截面积积分。R-pole 和 R-flat 只改变底面形状约束。T1 去低部观测，T2 去侧向扇区观测。G3 构造为圆钝点接触，这一几何先验在实物上需要独立确认。',
         [f,control],{m:COLORS[m] for m in keys}|{'ordinate':'percent'})


def convergence():
    f=VOLUME/'V001-columns.csv'
    data=load(f)
    pitch=np.array([2.,1.,.5,.25])
    kinds=('PYR','TOR','CAV','P2')
    colors=('#337bbb','#137f76','#77629c','#bf7444')
    fig,ax=plt.subplots(figsize=(180/25.4,75/25.4))
    lines=[]
    for case,c in zip(kinds,colors):
        e=[100*max(abs(float(r['relative_to_mesh'])) for r in data
                   if r['case']==case and float(r['pitch_mm'])==h) for h in pitch]
        lines+=ax.plot(range(4),e,'o-',label=case,color=c,linewidth=1.3)
    ax.axhline(.5,color='#687983',linewidth=.7,ls='--').set_gid('qa-ignore-data:target')
    ax.set(xticks=range(4),xticklabels=['2.00','1.00','0.50','0.25'],xlabel='Column spacing (mm)',
           ylabel='Worst error vs. tetrahedra (%)')
    ax.grid(axis='y')
    fig.legend(lines,kinds,loc='upper center',ncol=4,frameon=False,bbox_to_anchor=(.55,.99))
    fig.subplots_adjust(left=.105,right=.975,bottom=.22,top=.80)
    save(fig,VOLUME/'figures','V001-convergence',
         '6 种柱积分方向和网格偏移的最大绝对差。虚线为最细级与三角积分的 0.5% 核验目标。CAV 为有封闭内腔的解析体，P2 为大面摆放棱锥的实测网格。',
         [f],dict(zip(kinds,colors))|{'ordinate':'percent','mesh_spacing':'mm'})


if __name__=='__main__':
    style()
    biases()
    poses()
    sections()
    convergence()
