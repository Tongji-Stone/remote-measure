"""Export comparable coloured meshes, previews, and checks for measured volumes."""
import csv
import json

import cv2 as cv
import numpy as np
import open3d as o3d

from common import GROUPS,HERE,Surface,checked_volume,dump,scene

MODELS=['S04-GuideEnvelope','S08-PrecisePoisson','G01-2DGS-TSDF','G02-2DGS-closed','G03-2DGS-anchored']


def blend_image(m, center, view, width=352,height=300):
    az,el=np.deg2rad(view)
    z=np.array([np.cos(az)*np.cos(el), np.sin(az)*np.cos(el), np.sin(el)])
    x=np.array([-np.sin(az),np.cos(az),0.])
    y=np.cross(z,x)
    focus,scale=center
    s=np.arange(width)-width/2
    t=height/2-np.arange(height)
    u,v=np.meshgrid(s/scale,t/scale)
    points=focus+u.ravel()[:,None]*x+v.ravel()[:,None]*y+130*z
    rays=np.c_[points,np.broadcast_to(-z,points.shape)].astype('float32')
    hit=scene(m).cast_rays(o3d.core.Tensor(rays),nthreads=1)
    tf=hit['t_hit'].numpy()
    pixels=np.flatnonzero(np.isfinite(tf))
    ident=hit['primitive_ids'].numpy()[pixels]
    uv=hit['primitive_uvs'].numpy()[pixels]
    weights=np.c_[1-uv.sum(1),uv]
    faces=np.asarray(m.triangles)[ident]
    color=np.asarray(m.vertex_colors)[faces]
    n=np.asarray(m.vertex_normals)[faces]
    rgb=np.einsum('ij,ijk->ik',weights,color)
    norm=np.einsum('ij,ijk->ik',weights,n)
    light=np.maximum(0.,norm@(.83*z-.28*x+.40*y))
    value=np.rint(255*np.clip(rgb*(.72+.28*light[:,None]),0,1)).astype('uint8')
    image=np.tile(np.array([240,235,229],np.uint8),(height*width,1))
    image[pixels]=value[:,::-1]
    return image.reshape(height,width,3)


def preview(name,root,summary):
    keys=MODELS.copy()
    if 'S05-Planes' in summary:
        keys.insert(0,'S05-Planes')
    if 'S09-ExposedPlate' in summary:
        keys.insert(keys.index('S08-PrecisePoisson')+1,'S09-ExposedPlate')
    meshes={k:o3d.io.read_triangle_mesh(str(root/f'{k}.ply')) for k in keys
            if k in summary and summary[k]['status']!='empty'}
    assert all(m.has_triangles() and m.has_vertex_colors() for m in meshes.values())
    xyz=np.vstack([np.asarray(m.vertices) for m in meshes.values()])
    low,high=np.percentile(xyz,[.20,99.80],axis=0)
    center=(low+high)/2,222./np.max(high-low)
    tiles=[]
    for angles in [(28,16),(-100,26),(34,-65)]:
        row=[]
        for k,m in meshes.items():
            image=blend_image(m,center,angles)
            cv.putText(image,k,(10,24),cv.FONT_HERSHEY_SIMPLEX,.55,(46,48,43),1,cv.LINE_AA)
            vol=summary[k].get('volume_ml')
            s='open / invalid topology' if vol is None else f'{vol:.2f} mL'
            cv.putText(image,s,(10,285),0,.53,(46,48,43),1,cv.LINE_AA)
            row.append(image)
        tiles.append(np.hstack(row))
    cv.imwrite(str(root/'彩色重建对比.png'),np.vstack(tiles))


def summarize():
    cols=[]
    rec={}
    for name in GROUPS:
        root=HERE/name
        models={}
        for f in ('旧方法.json','高斯网格.json','精确闭合.json'):
            models.update(json.loads((root/f).read_text()))
        sensitivity=root/'露板补充实验.json'
        if sensitivity.is_file():
            models.update(json.loads(sensitivity.read_text()))
        for key,row in models.items():
            if row['status']=='empty':
                continue
            path=root/f'{key}.ply'
            m=o3d.io.read_triangle_mesh(str(path))
            assert m.has_triangles() and m.has_vertex_colors()
            col=row['column_checks']
            row['column_fine_max_rel']=max(abs(c['relative_difference']) for c in col if c['pitch_mm']==.4) if col else None
            if row['volume_ml'] is not None:
                v=checked_volume(Surface.of(m))/1000
                assert abs(v-row['volume_ml'])<3e-5
                assert all(r['odd_rays'] < 6 for r in col)
            labels=['volume_ml','observed_P50_mm','observed_P95_mm','boundary','nonmanifold_edges',
                    'components','orientation_errors','self_intersections','plate_cap_area_mm2',
                    'ray_hit_fraction','ray_abs_P95_mm','ray_front_1p5_fraction',
                    'unsupported_fraction_2mm','column_fine_max_rel','triangles']
            cols.append(dict(物体=name,方法=key,**{lab:row.get(lab) for lab in labels}))
        preview(name,root,models)
        key=next(k for k in ('S08-PrecisePoisson','S08-coarse','S04-GuideEnvelope')
                 if models.get(k,{}).get('volume_ml') is not None)
        if sensitivity.is_file():
            plate=json.loads((root/'露板与闭合核验.json').read_text())
            candidate='S09-ExposedPlate'
            if (models[candidate]['volume_ml'] is not None
                    and plate[candidate]['board_hits_cap']<plate.get(key,{}).get('board_hits_cap',0)
                    and models[candidate]['observed_P95_mm']<=models[key]['observed_P95_mm']+.01):
                key=candidate
        gaussian=next(k for k in ('G03-2DGS-anchored','G03-coarse','G02-2DGS-closed')
                      if models.get(k,{}).get('volume_ml') is not None)
        a,b,c=models[key],models.get('S08-coarse',{}),models[gaussian]
        fine=models['S08-PrecisePoisson'].get('volume_ml')
        drift=None if fine is None or b.get('volume_ml') is None else 100*(fine/b['volume_ml']-1)
        rec[name]=dict(method=key,mesh=str((root/f'{key}.ply').relative_to(HERE)),volume_ml=a['volume_ml'],
                        direct_2DGS_Mesh=models['G01-2DGS-TSDF'].get('volume_ml'),
                        gaussian_method=gaussian,
                        gaussian_anchored_ml=c['volume_ml'],
                        fine_coarse_volume_drift_percent=drift,
                        finer_two_family_range_ml=sorted([a['volume_ml'],c['volume_ml']]),
                        volume_estimate_sensitivity_per_half_mm_ml=.5*a['plate_cap_area_mm2']/1000,
                        grid_column_max_relative_difference=a['column_fine_max_rel'])
        if sensitivity.is_file():
            rec[name]['exposed_plate_alternative_ml']=models['S09-ExposedPlate'].get('volume_ml')
            rec[name]['precise_poisson_before_exposed_plate_ml']=fine
    with (HERE/'体积与几何.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,list(cols[0]))
        w.writeheader()
        w.writerows(cols)
    dump(HERE/'体积汇总.json',rec)


if __name__=='__main__':
    cv.setNumThreads(1)
    summarize()
