"""Run joint registration tests with mutually disjoint ICP/selection/final-test observations."""
import argparse
import csv
import itertools
from pathlib import Path

import cv2 as cv
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

from robust import HERE, axis_prior, axial, fold, load_json, relative_error, stitch, transform
from acquire import BRANCHES, SCANS, directory
from calibrate import dump
from geometry import as_pose, project, read_models, write_scene

SCENARIOS=('clean','half_views','noise_outliers','marker_occlusion','bad_loop')


def compact(result):
    return {key:result[key] for key in ('selected','elapsed_s','prior_s','contact_check_mm','contact_test_mm',
                                         'axis','test','tune','edges','input_views')}


def run(scan,branch):
    mem=directory(scan)/branch
    other='ChArUco' if branch=='Chessboard' else 'Chessboard'
    data=[]
    endpoints=[]
    paths=sorted(scan.glob('view_*'))
    for i,p in enumerate(paths):
        masks=Path('/tmp/E102/masks')/scan.name
        with np.load(mem/f'{i:02}.npz') as r,np.load(directory(scan)/other/f'{i:02}.npz') as v:
            with np.load(masks/branch/f'{i:02}.npz') as m, np.load(masks/other/f'{i:02}.npz') as n:
                _,a,b=np.intersect1d(r['grid_index'],v['grid_index'],return_indices=True,assume_unique=True)
                chosen=a[m['M04'][a]&n['M04'][b]]
                # Match the optical branches for the robustness benchmark. Preserve the full mask for delivery.
                data.append(({**{k:r[k][chosen] for k in ('xyz','uv','gray')},
                              'shape':r['shape'],'height':m['height'][chosen]},
                             {k:r[k][m['M04']] for k in ('xyz','gray')}))
        with np.load(directory(p)/'target.npz') as recovered:
            endpoints.append(dict(recovered))
    cmd=np.array(load_json(mem/'poses.json')['cmd'])
    model=read_models()[branch]
    home=HERE/branch/scan.name.split('_',4)[4]
    metadata={}
    all_metrics=[]
    baseline=None
    for form in SCENARIOS:
        chosen=np.arange(len(data))
        hidden=[]
        if form=='half_views':
            chosen=chosen[::2]
        if form=='marker_occlusion':
            hidden=[0,len(data)//2,len(data)//2+1]
        frames=[data[i][0] for i in chosen]
        targets=[endpoints[i] for i in chosen]
        r=stitch(frames,targets,cmd[chosen],model,noise=form=='noise_outliers',
                 markers_hidden=hidden,bad_edge=form=='bad_loop')
        if baseline is None:
            baseline=r['R02']
        delta=r['poses']@np.linalg.inv(baseline[chosen])
        pivot=np.array(r['axis']['center_mm'])
        error=np.rad2deg(Rotation.from_matrix(delta[:,:3,:3]).magnitude())
        shift=np.linalg.norm(delta[:,:3,:3]@pivot+delta[:,:3,3]-pivot,axis=1)
        assert np.allclose(r['poses'][0],np.eye(4),atol=1e-10)
        assert np.allclose(np.linalg.det(r['poses'][:,:3,:3]),1.,atol=1e-7)
        if form=='bad_loop':
            first=next(x for x in r['edges'] if x['source']==0)
            assert not first['accepted'] and first['delta_deg']>80
        for label in ('R02','R06c','policy'):
            M=r['test'][r['selected'] if label=='policy' else label]
            all_metrics.append(dict(scenario=form,method=label,input_views=len(frames),usable_markers=len(
                r['axis']['marker_stations']),chosen=r['selected'],F1_2=M['F1_2'],worst_F1_2=M['worst_F1_2'],
                depth_p50=M['depth_abs_P50'],depth_support=M['depth_supported'],tag_p50=M['tag_P50'],
                max_gap=M['max_gap'],contact_drift_mm=(r['contact_test_mm'] if label=='R06c' or
                      (label=='policy' and r['selected']=='R06c') else 0.),
                selected_pose_repeat_delta_deg=np.max(error) if label=='policy' else None,
                selected_pivot_repeat_delta_mm=np.max(shift) if label=='policy' else None,
                admitted_edges=sum(e['accepted'] for e in r['edges']),seconds=r['elapsed_s']))
        metadata[form]=compact(r)
        print(branch,home.name,form,r['selected'],round(all_metrics[-1]['F1_2'],3),
              round(all_metrics[-1]['depth_p50'],3),round(all_metrics[-1]['tag_p50'],3),
              round(all_metrics[-1]['max_gap'],2),flush=True)
        if form=='clean':
            full=[v[1] for v in data]
            for code in ('R02','R06c','poses'):
                write_scene(home/('推荐.ply' if code=='poses' else code+'.ply'),full,r[code],range(len(full)))
            dump(home/'位姿.json',{k:r[k] for k in ('poses','R02','R06c','selected')})
    dump(home/'鲁棒性.json',metadata)
    with (home/'指标.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(all_metrics[0]));w.writeheader();w.writerows(all_metrics)


def unit_tests():
    model=read_models()['ChArUco']
    center=np.array([-8.,12.,233.])
    u=np.array([.032,-.93,-.36]);u/=np.linalg.norm(u)
    steps=np.deg2rad(np.arange(0.,360.,36.))
    turns=axial(u,center,steps)
    anchor=as_pose(Rotation.from_euler('x',40,degrees=True).as_matrix(),[15.,-16.,229.])
    board=np.c_[np.mgrid[4.:83.:8.,5.:84.:9.].reshape(2,-1).T,np.zeros(90)]
    observations=[]
    for T in turns@anchor:
        observations.append(dict(pose=T,board_xyz=board,board_uv=project(transform(board,T),model)))
    for hidden in ([0],[2,3,6],[]):
        obs=[o if i not in hidden else None for i,o in enumerate(observations)]
        P,axis,c,A,fit=axis_prior(obs,steps,model)
        ref=np.linalg.inv(turns)
        np.testing.assert_allclose(P,ref,atol=2e-5)
        np.testing.assert_allclose(A,anchor,atol=2e-5)
    def rejects(obs,cmd,expected):
        try:
            axis_prior(obs,cmd,model)
        except ValueError as e:
            assert expected in str(e),e
        else:
            raise AssertionError('Malformed input was accepted')
    rejects([None]*len(observations),steps,'At least four')
    rejects(observations,np.r_[steps[0],steps[0],steps[2:]],'must increase')
    rejects([o if i<4 else None for i,o in enumerate(observations)],steps,'190 degrees')
    # The old target-frame gate used the forward pose on a world pivot.
    base=np.linalg.inv(turns)
    target=1
    offset=np.array([38.,22.,178.])
    pivot=transform(offset[None],np.linalg.inv(base[target]))[0]
    edge=axial(u,pivot,np.deg2rad(7)) @ np.linalg.inv(base[target])
    deg,correct=relative_error(edge,base,0,target,offset)
    wrong_pivot=transform(offset[None],base[target])[0]
    diff=edge@base[target]
    legacy=np.linalg.norm(transform(wrong_pivot[None],diff)[0]-wrong_pivot)
    assert correct<1e-8 and legacy>2.
    a=np.array([[x,y] for x,y in itertools.product(range(0,240,24),range(0,240,24))])
    train=(fold(a)!=0)&(fold(a)!=5)
    assert not ((fold(a)==0)&(fold(a)==5)).any() and not (train&(fold(a)%5==0)).any()
    dump(HERE/'坐标与输入核验.json',dict(known_transform_atol=2e-5,missing_first_accepted=True,
                                       no_markers_rejected=True,bad_angles_rejected=True,
                                       lack_of_angle_coverage_rejected=True,
                                       fixture_edge_angle_deg=deg,correct_pivot_shift_mm=correct,
                                       legacy_pivot_shift_mm=legacy,independent_image_folds=True))
    print('coordinate, reference, missing markers, ordering, and split checks passed',flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('branch',choices=(*BRANCHES,'unit'))
    parser.add_argument('scan',nargs='?')
    args=parser.parse_args()
    cv.setNumThreads(1);cv.ocl.setUseOpenCL(False);cv.setRNGSeed(104);o3d.utility.random.seed(104)
    if args.branch=='unit':
        unit_tests()
    else:
        run(next(s for s in SCANS if args.scan in s.name),args.branch)
