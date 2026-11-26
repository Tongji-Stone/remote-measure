"""Check final topology, finish the lobed prior, and publish the comparison index."""
import argparse
import csv
from pathlib import Path
import shutil
from time import perf_counter

import cv2 as cv
import numpy as np
import open3d as o3d

from experiment import (HERE,CASES,CONTROLS,EXPERIMENTS,read,dump,real_input,
                        simulated,split,metric,save,trial_out)
from reconstruct import ground_contact, oriented, repair_contact_mesh


def process(folder):
    item=read(folder/'结果.json')
    ctrl=item['case'] in CONTROLS
    which,branch=item['case'],item['branch']
    src,board,cal,plane,truth,shape,picked=simulated(which) if ctrl else real_input(which,branch)
    for case,stats in item['tests'].items():
        if shape=='pumpkin' and 'S07' not in stats['methods']:
            added=trial_out(folder,src,cal,case,shape,truth,{'S05','S07'})
            stats['methods'].update(added['methods'])
        for method in ('S04','S07'):
            if method in stats['methods']:
                attempt(folder,case,stats,src,truth,method)
    if not ctrl:
        src2,_,_,_,_,_,_=real_input(which,branch,'E102')
        key='E102-pose'
        attempt(folder,key,item['input_controls'][key],src2,None,'S04')
    dump(folder/'结果.json',item)
    print(folder.name,[(q,d['methods']['S04'].get('watertight')) for q,d in item['tests'].items()],flush=True)


def attempt(folder,key,record,frames,truth,method):
    original=record['methods'][method]
    path=folder/f'{key}-{method}.ply'
    if original['status']!='scored' or original.get('watertight') or not path.exists():
        return
    start=perf_counter()
    mesh=o3d.io.read_triangle_mesh(str(path))
    simpler=repair_contact_mesh(mesh)
    if simpler is mesh:
        return
    if simpler.is_watertight() and ground_contact(simpler)['touches_floor']:
        t,oc,hidden,_=split(frames,key)
        p,_=oriented(t)
        if len(p)>27000:
            p=p[np.random.default_rng(774).choice(len(p),27000,replace=False)]
        revised=metric(simpler,p,oc,hidden,truth)
        if revised['observed_p95']<=original['observed_p95']+.28:
            revised.update(elapsed_s=original['elapsed_s']+perf_counter()-start,
                           topology_repair='floor_pinned',before_triangles=original['triangles'])
            record['methods'][method]=revised
            save(path,simpler)


def selection(source):
    full=source['tests']['T0']['methods']
    best_obs=min(x['observed_p95'] for x in full.values() if x['status']=='scored')
    candidate={}
    for key,base in full.items():
        if (not base.get('watertight') or not base.get('touches_floor') or base['observed_p95']>best_obs+.8 or
            (base['ray_front3_rate'] or 0)>.08 or base['below_plane_frac']>.016):
            continue
        missing=[]
        for test in ('T1','T2'):
            stat=source['tests'][test]['methods'].get(key,{})
            missing.append(stat.get('hidden_r2') or 0)
        candidate[key]=sum(missing)/len(missing)
    best_phase=max(candidate,key=lambda k:(candidate[k],-full[k]['observed_p95'])) if candidate else None
    best_visible=min((k for k,b in full.items() if b.get('status')=='scored'),
                     key=lambda k:full[k]['observed_p95'])
    if source['case'] in CONTROLS:
        return dict()
    return dict(watertight=best_phase,surface=best_visible,eligible=candidate)


def write_csv(path,rows):
    keys=list(dict.fromkeys(k for v in rows for k in v))
    with path.open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)


def publish(source,target):
    temp=target.with_suffix('.part')
    shutil.copyfile(source,temp)
    temp.replace(target)


def index():
    stats=[]; recommendations={}
    for f in sorted((HERE/'场景').glob('*/结果.json')):
        source=read(f)
        case=f.parent.name
        for key,row in source['tests'].items():
            for k,a in row['methods'].items():
                stats.append(dict(group=case,branch=source['branch'],trial=key,method=k,**a))
        for key,row in source.get('input_controls',{}).items():
            a=row.get('against_object_only')
            for k,b in row['methods'].items():
                stats.append(dict(group=case,branch=source['branch'],trial=key,method=k,**(a or b)))
        if source['case'] not in CONTROLS:
            recommended=selection(source)
            recommendations[case]=recommended
            if recommended['watertight']:
                p=f.parent/f"T0-{recommended['watertight']}.ply"
                publish(p,f.parent/'封闭候选.ply')
            else:
                (f.parent/'封闭候选.ply').unlink(missing_ok=True)
            p=f.parent/f"T0-{recommended['surface']}.ply"
            publish(p,f.parent/'表面优选.ply')
    write_csv(HERE/'补全指标.csv',stats)
    dump(HERE/'推荐.json',recommendations)
    print('indexed',len(stats),len(recommendations))


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('scene')
    a=p.parse_args()
    cv.setNumThreads(1);cv.ocl.setUseOpenCL(False);o3d.utility.random.seed(104)
    if a.scene=='index':
        index()
    else:
        process(HERE/'场景'/a.scene)
