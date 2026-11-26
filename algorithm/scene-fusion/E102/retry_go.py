"""Retry an E102 Go-ICP group that exhausted its E101 time budget on every edge."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import os
from time import perf_counter

import cv2 as cv
import numpy as np

from register_crops import CACHE, HERE, SCANS, BRANCHES, directory, read, matched_analysis, bottom_proxy, dump
from geometry import read_models, axial, write_scene
import benchmark as old
import csv


def main(binary):
    branch='ChArUco'
    scan=next(s for s in SCANS if '043648' in s.name)
    result_dir=HERE/branch/scan.name.split('_',4)[4]
    result=read(result_dir/'结果.json')
    if 'R16' not in result['missing']:
        return
    roots=[directory(scan)/b for b in BRANCHES]
    data=[[dict(np.load(f)) for f in sorted(p.glob('*.npz'))] for p in roots]
    masks=[[dict(np.load(f)) for f in sorted((CACHE/scan.name/b).glob('*.npz'))] for b in BRANCHES]
    d=data[0]
    train=[old.sample_view({**z,'region':np.flatnonzero(m['e102'])}) for z,m in zip(d,masks[0])]
    sample=matched_analysis(data,masks)[0]
    val=[old.sample_view({**z,'region':e}) for z,e in zip(d,sample)]
    prep=[old.prepare_pair(v['train'],old.CONFIG,v['gray']) for v in train]
    meta=read(roots[0]/'poses.json')
    axis,center,phi=[np.array(meta[k]) for k in ('axis','center','phi')]
    base=axial(axis,center,-phi)
    model=read_models()[branch]
    config={**old.CONFIG,'goicp_timeout_seconds':10}
    scratch=Path('/tmp/E102/go-retry')/str(os.getpid())
    scratch.mkdir(parents=True,exist_ok=True)

    def solve(edge):
        a,b=edge
        start=perf_counter()
        try:
            T,_=old.go_icp(train[a]['train'][:650],train[b]['train'][:900],binary,scratch/f'{a:02}-{b:02}',config)
            T,_=old.fit_pair('tukey_icp',prep[a],prep[b],T,config,None,None,101+a)
            overlap=old.support(train[a],train[b],T)
            if overlap<.04:
                raise ValueError(f'Only {overlap:.3f} training overlap within 4 mm')
            return edge,(T,overlap),dict(source=a,target=b,status='accepted',actual_transform=T,
                                         overlap4=overlap,elapsed_s=perf_counter()-start)
        except (RuntimeError,ValueError,cv.error,FileNotFoundError) as exc:
            return edge,None,dict(source=a,target=b,status='fallback_to_R02',error=str(exc),
                                  elapsed_s=perf_counter()-start)

    with ThreadPoolExecutor(max_workers=4) as pool:
        rows=list(pool.map(solve,old.ring(len(d))))
    edges={edge:edge_data for edge,edge_data,_ in rows if edge_data is not None}
    result['R16_retry']=dict(per_edge_budget_s=10,first_run=result['edge_solver']['R16'],
                              retry=[r for _,_,r in rows],accepted=len(edges))
    if edges:
        obj=[{k:z[k][m['e102']] for k in ('xyz','gray')} for z,m in zip(d,masks[0])]
        T=old.sequence(edges,base)
        row,details=old.summarize(val,d,T,base,axis,center,model,edges)
        contacts=[]
        for z,m,e in zip(d,masks[0],sample):
            h=m['height'][e]
            g=e[(h>=-.85)&(h<=5.)&~old.training_mask(z['uv'][e])]
            contacts.append(z['xyz'][g[::max(1,len(g)//400)]])
        row.update(method=old.METHODS['R16'],fallback_edges=len(d)-len(edges),
                   **bottom_proxy(contacts,T,base,d))
        result['metrics']['R16']=row
        result['edge_solver']['R16']=[r for _,_,r in rows]
        result['per_edge_metrics']['R16']=details
        result['matrices']['R16']=T
        result['full_cloud_vertices']['R16']=write_scene(result_dir/'整圈'/'R16.ply',obj,T,range(len(T)))
        write_scene(result_dir/'双视角'/'R16.ply',obj[:2],T[:2],range(2))
        result['missing'].pop('R16')
        eligible={k:r for k,r in result['metrics'].items()
                  if (r['contact_z_drift_P95_mm'] or 0)<=1.5}
        proposed,shortlist=old.prefer(eligible)
        assert proposed==result['chosen'],(proposed,result['chosen'])
        result['eligible']=shortlist
    dump(result_dir/'结果.json',result)
    if edges:
        fields=sorted(set().union(*(r.keys() for r in result['metrics'].values())))
        with (result_dir/'指标.csv').open('w',newline='',encoding='utf-8-sig') as f:
            w=csv.DictWriter(f,fieldnames=['code',*fields]);w.writeheader()
            w.writerows(dict(code=c,**result['metrics'][c]) for c in sorted(result['metrics']))
    print('Go-ICP extended budget',len(edges),'of',len(d),flush=True)


if __name__=='__main__':
    import sys
    cv.setNumThreads(1)
    main(sys.argv[1])
