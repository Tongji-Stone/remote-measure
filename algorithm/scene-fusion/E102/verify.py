"""Verify raw subsets, per-view labels, and rigid transforms in the delivered point clouds."""
import csv
import json

import numpy as np
import open3d as o3d

from foreground import HERE, CACHE, BRANCHES, SCANS, CROP_MODES, directory, dump
from geometry import transform

DTYPE=np.dtype([('xyz','<f4',(3,)),('rgb','u1',(3,)),('view','u1')])


def ply(path):
    with path.open('rb') as f:
        assert f.readline()==b'ply\n',path
        assert f.readline()==b'format binary_little_endian 1.0\n',path
        lines=[]
        for line in f:
            if line==b'end_header\n':
                break
            lines.append(line)
        elements=[line for line in lines if line.startswith(b'element ')]
        assert len(elements)==1 and elements[0].startswith(b'element vertex '),path
        count=int(elements[0].split()[-1])
        assert path.stat().st_size-f.tell()==count*DTYPE.itemsize,path
        return np.fromfile(f,DTYPE,count)


def read(path):
    return json.loads(path.read_text())


def check(scan, branch, mode):
    case=scan.name.split('_',4)[4]
    folder=HERE.parent/mode/branch/case
    r=read(folder/'结果.json')
    mask_key='e102' if mode=='E102' else 'M04'
    optical_files=sorted((directory(scan)/branch).glob('*.npz'))
    mask_files=sorted((CACHE/scan.name/branch).glob('*.npz'))
    single=[]
    for i,(opt,msk) in enumerate(zip(optical_files,mask_files)):
        data=np.load(opt)
        masks=np.load(msk)
        use=masks[mask_key]
        assert (masks['M04']<=masks['e102']).all(), (mode,branch,case,i)
        assert not (masks['M04']&masks['board']).any(),(mode,branch,case,i)
        xyz=data['xyz'][use]
        g=data['gray'][use]
        cloud=ply(folder/'单视角'/f'V{i:02}.ply')
        assert np.array_equal(xyz,cloud['xyz']),(mode,branch,case,i)
        assert np.all(cloud['view']==i)
        assert np.all(cloud['rgb']==g[:,None])
        distance=np.linalg.norm(xyz,axis=1)
        assert np.all(distance>=149.999) and np.all(distance<=350.001)
        single.append(cloud)
    lens=[len(s) for s in single]
    assert list(r['full_cloud_vertices'])==list(r['matrices'])
    for method,q in r['matrices'].items():
        matrices=np.array(q)
        assert np.allclose(matrices[0],np.eye(4))
        assert np.allclose(matrices[:,:3,:3] @ matrices[:,:3,:3].transpose(0,2,1),np.eye(3),atol=1e-4)
        assert np.allclose(np.linalg.det(matrices[:,:3,:3]),1.,atol=1e-4)
        all_points=ply(folder/'整圈'/f'{method}.ply')
        assert len(all_points)==r['full_cloud_vertices'][method]==sum(lens)
        left=0
        for i,(s,T,length) in enumerate(zip(single,matrices,lens)):
            stride=max(1,length//90)
            x=all_points[left:left+length:stride]
            assert np.all(x['view']==i)
            assert np.allclose(transform(s['xyz'][::stride],T),x['xyz'],atol=1e-4)
            left += length
        pair=ply(folder/'双视角'/f'{method}.ply')
        assert np.array_equal(pair,all_points[:sum(lens[:2])])
    best = folder/'整圈'/'推荐.ply'
    assert np.array_equal(ply(best),ply(folder/'整圈'/f'{r["chosen"]}.ply'))
    recommended=o3d.io.read_point_cloud(str(best))
    assert len(recommended.points)==sum(lens) and recommended.has_colors()
    assert np.isfinite(np.asarray(recommended.points)).all()
    capture=read(folder/'裁剪.json')['source']['views']
    m=r['metrics'][r['chosen']]
    return dict(experiment=mode,branch=branch,object=case,views=len(single),methods=len(r['matrices']),
                selected=r['chosen'],E101_vertices=sum(v['initial'] for v in capture),
                output_vertices=sum(lens),near_contact_vertices=sum(v['M04_bottom_count'] for v in capture),
                board_ref=input_board(capture,'board_points'),
                kept_board_ref=input_board(capture,'board_pixels_kept_E102') if mode=='E102' else 0,
                F1_at_2=m['F1_2'],worst_F1_at_2=m['worst_F1_2'],depth_P50_mm=m['depth_abs_P50'],
                depth_support=m['depth_supported'],marker_P50_px=m['tag_P50'],angle_gap_deg=m['max_gap'],
                near_contact_drift_P95_mm=m['contact_z_drift_P95_mm'])


def input_board(capture, field):
    return sum(v[field] for v in capture)


if __name__=='__main__':
    for mode in CROP_MODES:
        results=[]
        for b in BRANCHES:
            for s in SCANS:
                row=check(s,b,mode)
                results.append(row)
                print(mode,b,row['object'],row['selected'],row['output_vertices'],flush=True)
        out=HERE.parent/mode
        dump(out/'核验.json',results)
        with (out/'汇总.csv').open('w',newline='',encoding='utf-8-sig') as f:
            w=csv.DictWriter(f,fieldnames=list(results[0]));w.writeheader();w.writerows(results)
