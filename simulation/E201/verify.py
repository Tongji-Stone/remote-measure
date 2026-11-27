"""Check the analytic ray intersection and the saved E201 delivery."""
import csv
import json
import re
from urllib.parse import unquote

import numpy as np
import open3d as o3d

from common import GROUPS,HERE,REPO,Surface,checked_volume,dump,intersection_pairs,column_integral
from splatting import View,load_engine,render


def raster_check():
    discs=dict(p=np.array([[0.,0.,29.],[0.,0.,32.]]), n=np.array([[0.,0.,-1.]]*2),
               u=np.array([[1.,0.,0.]]*2),v=np.array([[0.,1.,0.]]*2),r=np.array([.9,.9]),
               rgb=np.array([[1.,0.,0.],[0.,0.,1.]]))
    view=View(np.eye(4),np.array([84.,84.,9.,9.],np.float32),19,19,(None,None,None))
    d,c,a,s,owner=render(discs,view,1.,.7,load_engine())
    assert abs(d[9,9]-29.)<1e-6
    assert abs(a[9,9]-(1-.3**2))<1e-6
    assert np.allclose(c[9,9],[.7/.91,0,.21/.91],atol=1e-6)
    assert abs(s[9,9]-2*.7*.21*3/(.91**2))<1e-6
    assert owner[9,9]==0
    return dict(front_depth_mm=29,center_opacity=float(a[9,9]),weighted_ray_distortion_mm=float(s[9,9]))


def spatial_intersection_check():
    sphere=o3d.geometry.TriangleMesh.create_sphere(1.,resolution=18)
    test=sphere+o3d.geometry.TriangleMesh(sphere).translate([.9,.1,.13])
    counts=[]
    for mesh in (sphere,test):
        expected=np.asarray(mesh.get_self_intersecting_triangles())
        expected=np.unique(np.sort(expected,axis=1),axis=0)
        observed=intersection_pairs(mesh,100)
        assert np.array_equal(expected,observed)
        counts.append(len(observed))
    return counts


def delivery():
    with (HERE/'体积与几何.csv').open(encoding='utf-8-sig') as f:
        table=list(csv.DictReader(f))
    choices=json.loads((HERE/'体积汇总.json').read_text())
    contents={}
    for name in GROUPS:
        result=[]
        for k in (choices[name]['method'],choices[name]['gaussian_method']):
            target=HERE/name
            rows={}
            for file in ('精确闭合.json','高斯网格.json','旧方法.json'):
                rows.update(json.loads((target/file).read_text()))
            sensitivity=target/'露板补充实验.json'
            if sensitivity.is_file():
                rows.update(json.loads(sensitivity.read_text()))
            m=o3d.io.read_triangle_mesh(str(target/f'{k}.ply'))
            assert m.has_vertex_colors() and m.has_triangles()
            assert rows[k]['self_intersections'] is False
            assert np.isfinite(np.asarray(m.vertex_colors)).all()
            q=Surface.of(m)
            volume=checked_volume(q)/1000
            assert np.isclose(volume,rows[k]['volume_ml'],atol=1e-9)
            checks=rows[k]['column_checks']
            finest=min(s['pitch_mm'] for s in checks)
            assert all(s['odd_rays']==0 for s in checks if s['pitch_mm']==finest)
            assert all(s['relative_difference']<.005 and s['relative_difference']>-.005
                       for s in checks)
            shifted=[]
            for coarse in (s for s in checks if s['odd_rays']):
                test=column_integral(q,q.scene(),coarse['pitch_mm'],'xyz'.index(coarse['axis']),(.57,.29))
                assert test['odd_rays']==0 and abs(test['column_volume_mm3']/(volume*1000)-1)<.005
                shifted.append(dict(original_odd=coarse['odd_rays'],shifted=test))
            assert np.asarray(m.vertices)[:,2].min()>=-1e-6
            result.append(dict(mesh=k,volume_ml=volume,topology='closed and orientable',
                               components=rows[k]['components'],shifted_coarse_ray_checks=shifted))
        contents[name]=result
    for p in [HERE/'README.md',*(REPO/'algorithm/best method').rglob('README.md')]:
        for link in re.findall(r'\]\(([^)]+)\)',p.read_text()):
            if not link.startswith(('https:', 'http:')):
                assert (p.parent/unquote(link)).exists(),(p,link)
    return dict(measured=contents,methods_in_csv=len(table))


if __name__=='__main__':
    dump(HERE/'交付核验.json',dict(analytic_renderer=raster_check(),
                                   spatial_intersection_fixture=spatial_intersection_check(),
                                   **delivery()))
