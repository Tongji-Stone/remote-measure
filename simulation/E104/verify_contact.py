"""Verify the calibrated base, outward caps, and volume of the exported E104 solid meshes."""
import argparse
import csv
from pathlib import Path

import numpy as np
import open3d as o3d

from deck import contact_carve
from experiment import HERE, read
from reconstruct import FLOOR_EPS, ground_contact, projective_surface


def signed_volume(triangles):
    cross = np.cross(triangles[:, 1], triangles[:, 2])
    return np.einsum('ij,ij->', triangles[:, 0], cross) / 6.


def check(mesh):
    vertices = np.asarray(mesh.vertices)
    faces = np.asarray(mesh.triangles)
    triangles = vertices[faces]
    assert mesh.is_watertight() and mesh.is_orientable()
    edges = np.concatenate((faces[:,[0,1]], faces[:,[1,2]], faces[:,[2,0]]))
    keys, index, counts = np.unique(np.sort(edges,axis=1),axis=0,return_inverse=True,return_counts=True)
    signs = np.where(edges[:,0]<edges[:,1],1.,-1.)
    assert np.all(counts==2) and np.all(np.bincount(index,weights=signs,minlength=len(keys))==0)
    contact = ground_contact(mesh)
    assert contact['touches_floor'] and vertices[:, 2].min() >= -FLOOR_EPS
    cap = triangles[np.max(np.abs(triangles[:, :, 2]), axis=1) <= FLOOR_EPS]
    normal = np.cross(cap[:, 1] - cap[:, 0], cap[:, 2] - cap[:, 0])
    assert np.all(normal[:, 2] < 0.), 'Floor triangles must point away from the solid.'
    analytic = signed_volume(triangles)
    volume = mesh.get_volume()
    assert volume > 0. and np.isclose(analytic, volume, rtol=1e-8, atol=1e-6)
    return contact, volume


def regression_test():
    shape = (14, 15, 12)
    x, y, z = np.indices(shape)
    field = np.maximum.reduce([2.5-x, x-9.5, 2.5-y, y-10.5, z-6.5])
    coverage = np.ones(field.size, bool)
    model = projective_surface(field.ravel(), coverage, np.zeros(3), shape, pitch=1., cap_floor=True)
    contact, volume = check(model)
    # The sampled piecewise-linear field rounds a small part of the box's sharp XY corners.
    assert np.isclose(contact['floor_cap_area_mm2'], 7*8, rtol=.01)
    assert np.isclose(volume, 7*8*6.5, rtol=.011)
    x,y=np.meshgrid(np.linspace(0.,5.,11),np.linspace(0.,5.,11))
    plate=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]
    xy=np.array([[2.5,2.5],[4.,4.],[20.,20.]])
    object_base=np.array([[2.5,2.5,.45]])
    assert np.array_equal(contact_carve(xy,[plate,plate],object_base),[False,True,False])
    assert not contact_carve(xy,[plate],object_base).any()


def verify(previous):
    results = []
    references = read(HERE/'推荐.json')
    for group, record in sorted(references.items()):
        folder = HERE/'场景'/group
        data = read(folder/'结果.json')
        method = record['watertight']
        status = 'recommended'
        if method is None:
            # Keep the geometry-only result available in weaker optical branches.
            alternatives = data['tests']['T0']['methods']
            method = next((k for k in ('S04','S07','S05','S03')
                           if alternatives.get(k,{}).get('watertight')
                           and alternatives[k]['touches_floor']),None)
            status = 'geometric_control'
        if method is None:
            raise AssertionError(f'No grounded solid produced in {group}')
        filename = f'T0-{method}.ply'
        model = o3d.io.read_triangle_mesh(str(folder/filename))
        contact, volume = check(model)
        expected = data['tests']['T0']['methods'][method]
        assert np.isclose(expected['volume_mm3'], volume)
        assert np.isclose(expected['floor_cap_area_mm2'], contact['floor_cap_area_mm2'])
        official = folder/'封闭候选.ply'
        if status == 'recommended':
            assert official.read_bytes() == (folder/filename).read_bytes()
        else:
            assert not official.exists(), 'An obsolete recommendation remains in the scene.'
        old = previous.get(group)
        results.append(dict(scene=group,method=method,status=status,file=filename,
                            observed_p95_mm=expected['observed_p95'],
                            watertight=True,**contact,volume_ml=volume/1000.,
                            prior_lowest_1pct_mm=old.get('lowest1pct_mm') if old else None,
                            prior_floor_cap_area_mm2=old.get('cap_area_mm2') if old else None,
                            prior_volume_ml=old.get('volume_ml') if old else None))
        print(group,method,status,f"{contact['floor_cap_area_mm2']:.2f} mm²",f"{volume/1000.:.3f} mL",
              flush=True)
    assert sum(x['status']=='recommended' and x['scene'].endswith('ChArUco') for x in results)==5
    with (HERE/'贴底核验.csv').open('w',newline='',encoding='utf-8-sig') as f:
        out = csv.DictWriter(f,fieldnames=list(results[0]))
        out.writeheader()
        out.writerows(results)


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--prior',type=Path)
    args = p.parse_args()
    regression_test()
    verify(read(args.prior) if args.prior else {})
