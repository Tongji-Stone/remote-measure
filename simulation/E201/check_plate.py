"""Cross-check the closed bottom against positively identified exposed board pixels."""
import json

import numpy as np
import open3d as o3d

from common import GROUPS,HERE,acquisition,dump,tx
from prepare import WORK
from reconstruct import scene


def plate_views(name):
    case,_=acquisition(name)
    s=case['scans'][0]
    root=HERE/name
    A=np.array(json.loads((root/'底面标定.json').read_text())['world_to_plate_mm'])
    cache=WORK/'geometry'/s['folders'][0].parent.name
    data=[]
    for i,P in enumerate(s['P']):
        with np.load(cache/f'{i:02}.npz') as f:
            p=f['xyz'][f['hard_board']]
        data.append(tx(p,A@s['B']@P))
    return data


def plane_witnesses(name):
    q=np.vstack(plate_views(name))
    return q[abs(q[:,2])<1.4]


def inspect(name):
    q=plane_witnesses(name)
    folder=HERE/name
    rows={}
    for key in ('S04-GuideEnvelope','S05-Planes','S08-PrecisePoisson','S09-ExposedPlate',
                'G02-2DGS-closed','G03-2DGS-anchored'):
        model=folder/f'{key}.ply'
        if not model.is_file():
            continue
        m=o3d.io.read_triangle_mesh(str(model))
        pts=np.asarray(m.vertices)
        lo=pts.min(0)[:2]-1.
        hi=pts.max(0)[:2]+1.
        pos=q[((q[:,:2]>lo)&(q[:,:2]<hi)).all(1)]
        cast=np.zeros((len(pos),6),np.float32)
        cast[:,:2]=pos[:,:2]
        cast[:,2]=-.83
        cast[:,5]=1.
        hit=scene(m).cast_rays(o3d.core.Tensor(cast),nthreads=1)['t_hit'].numpy()
        on=np.abs(hit-.83)<.045
        rows[key]=dict(nearby_exposed_board_points=len(pos),board_hits_cap=int(on.sum()),
                       board_hits_cap_fraction=float(on.mean()))
    dump(folder/'露板与闭合核验.json',rows)
    print(name,rows,flush=True)


if __name__=='__main__':
    for name in GROUPS:
        inspect(name)
