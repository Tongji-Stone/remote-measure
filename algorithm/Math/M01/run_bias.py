"""Run multi-fold method/shape comparisons and anchored sensitivity calculations."""
import csv
import json
from pathlib import Path
import sys
from time import perf_counter

import numpy as np
import open3d as o3d

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
SIM=ROOT/'simulation/E104'
sys.path.insert(0,str(SIM))
sys.path.insert(0,str(ROOT/'computing volume/V001'))
from experiment import CASES, real_input, sample_frames, simulated, split
from surface_bias import evaluate
from volume_math import Surface, tetra_gradient
from run import analytic, save_csv

OUT=HERE/'results'


def save_mesh(s,path):
    assert o3d.io.write_triangle_mesh(str(path),s.legacy(),write_vertex_normals=False)


def calibration_comparison():
    data=[]
    for code in CASES:
        for branch in ('ChArUco','Chessboard'):
            folder=SIM/'场景'/f'{code}-{branch}'
            s=Surface.load(folder/'T0-S04.ply')
            record=json.loads((folder/'结果.json').read_text())
            m=record['tests']['T0']['methods']['S04']
            data.append(dict(case=code,calibration=branch,recipe='S04',volume_ml=s.signed()/1000.,
                             baseline_pass=m['watertight'],mesh_fit_p95_mm=m['observed_p95'],
                             physical_volume_ml=None))
    return data


def sensitivity(surface,coverage):
    v=surface.signed()
    t,area,n=surface.g()
    on=(np.abs(t[:,:,2])<1e-7).all(axis=1)
    cap=float(area[on].sum())
    grads=tetra_gradient(surface)
    # Distinguish an ideal planar shift from a local warp of the first 2 mm of observed geometry.
    a=surface.vertices.copy()
    datum_weight=np.clip(1-a[:,2]/2.,0.,1.)
    dv=np.zeros_like(a)
    dv[:,2]=datum_weight
    floor_slope=float(np.sum(grads*dv))
    return dict(mesh_ml=v/1000.,surface_mm2=area.sum(),cap_mm2=cap,area_over_volume_per_mm=area.sum()/v,
                scale_minus025_delta_ml=v*((1-.0025)**3-1)/1000.,
                scale_plus025_delta_ml=v*((1+.0025)**3-1)/1000.,
                plane_plus05_first_order_ml=-.5*cap/1000.,
                warped_near_floor_plus05_first_order_ml=.5*floor_slope/1000.,
                supported_normal_plus025_first_order_ml=.25*coverage*area.sum()/1000.,
                whole_surface_normal_plus025_first_order_ml=.25*area.sum()/1000.)


def measured():
    rows,budgets=[],[]
    recommendation=json.loads((SIM/'推荐.json').read_text())
    for code in CASES:
        start=perf_counter()
        key=f'{code}-ChArUco'
        method=recommendation[key]['watertight']
        source=SIM/'场景'/key/f'T0-{method}.ply'
        if code=='F1':
            source=SIM/'网格优化/细网格双边.ply'
        mesh=Surface.load(source)
        frames,_,_,_,_,_,_=real_input(code,'ChArUco')
        _,held,_,_=split(frames,'T0')
        p,cam=sample_frames(held,each=3500)
        rs,out=evaluate(mesh,p,cam,None,code,'T0','real')
        chosen=rs[-1]['selected']
        for r in rs[-2:]:
            if r['selected']!='D0':
                save_mesh(out[r['selected']],OUT/'meshes'/f'{code}-{r["method"]}-{r["selected"]}.ply')
        for r in rs:
            r['mesh_source']=str(source.relative_to(ROOT))
        rows+=rs
        budgets.append(dict(case=code,**sensitivity(mesh,rs[0]['coverage_area_fraction'])))
        save_csv(OUT/'real_bias.csv',rows)
        save_csv(OUT/'volume_sensitivity.csv',budgets)
        print(code,chosen,[(r['method'],r.get('volume_ml'),r.get('test_p95_mm')) for r in rs],
              f'{perf_counter()-start:.1f}s',flush=True)


def scans():
    rows=[]
    for code in ('G1','G2','G3'):
        frames,_,_,_,(_,truth),_,_=simulated(code)
        for trial in ('T0','T1','T2'):
            source=SIM/'场景'/code/f'{trial}-S04.ply'
            mesh=Surface.load(source)
            _,held,missing,_=split(frames,trial)
            p,cam=sample_frames(held,each=2000)
            hid,_=sample_frames(missing,each=1000)
            rs,_=evaluate(mesh,p,cam,truth,code,trial,'synthetic_scan',hidden=hid)
            rows+=rs
            save_csv(OUT/'synthetic_scans.csv',rows)
            print(code,trial,[(r['method'],r.get('relative_error')) for r in rs],flush=True)


def mechanisms():
    rows=[]
    rng=np.random.default_rng(12032)
    for code in ('SPH','ELL','TOR'):
        s,truth,_=analytic()[code]
        t,area,normal=s.g()
        v=s.vertices
        n=np.asarray(s.legacy().compute_vertex_normals().vertex_normals)
        a=np.arctan2(v[:,1],v[:,0])
        z=v[:,2]/np.max(abs(v[:,2]))
        f=np.arange(len(s.faces))
        f=rng.choice(f,21000,p=area/area.sum(),replace=True)
        q=rng.uniform(size=(len(f),2))
        q[q.sum(axis=1)>1]=1-q[q.sum(axis=1)>1]
        p=np.einsum('ij,ijk->ik',np.c_[q,1-q.sum(axis=1)],t[f])
        samples=p+rng.normal(0,.12,(len(p),1))*normal[f]
        for trial,shift,obs in (
                ('uniform',np.full(len(v),.3),np.ones(len(p),bool)),
                ('localized',.55*np.clip((z+.40)/1.4,0.,1.)*(.45+.55*np.cos(a)**2),np.ones(len(p),bool)),
                ('hidden_cap',.65*np.clip((-z-.25)/.75,0.,1.),p[:,2]>-.20*np.max(abs(p[:,2])))):
            deltas=n*shift[:,None]
            model=Surface(v+deltas,s.faces)
            # Idealized independent measurements with isotropic availability on the retained surface.
            rs,_=evaluate(model,samples[obs],None,truth,code,trial,'injected_mesh',
                          hidden=samples[~obs],floor=False)
            rows+=rs
            save_csv(OUT/'injected_bias.csv',rows)
            print(code,trial,[(r['method'],r.get('relative_error')) for r in rs],flush=True)


if __name__=='__main__':
    OUT.mkdir(exist_ok=True)
    (OUT/'meshes').mkdir(exist_ok=True)
    mode=sys.argv[1] if len(sys.argv)>1 else 'all'
    if mode in ('all','real'):
        save_csv(OUT/'calibration_pair.csv',calibration_comparison())
        measured()
    if mode in ('all','scans'):
        scans()
    if mode in ('all','mechanisms'):
        mechanisms()
