"""Register another same-rig scan from a manifest of foreground arrays and board corner observations."""
import argparse
from pathlib import Path

import cv2 as cv
import numpy as np
import open3d as o3d

from robust import load_json, stitch
from geometry import CAL_KEYS, write_scene
from calibrate import dump


def main(manifest,output):
    data=load_json(manifest)
    parent=manifest.parent
    files=data['angles']
    model={k:np.asarray(v,float) for k,v in load_json(parent/data['calibration']).items() if k in CAL_KEYS}
    frames=[]; tags=[]
    for entry in files:
        with np.load(parent/entry['cloud']) as audit:
            f={k:audit[k] for k in ('xyz','uv','gray','shape')}
            if 'height' in audit:
                f['height']=audit['height']
            n=len(f['xyz'])
            if (f['xyz'].shape!=(n,3) or f['uv'].shape!=(n,2) or f['gray'].shape!=(n,)
                or any(not np.isfinite(f[k]).all() for k in ('xyz','uv','gray'))
                or np.shape(f['shape'])!=(2,) or np.any(f['shape']<=0)):
                raise ValueError(f'Unexpected point cloud array shapes in {entry["cloud"]}')
            frames.append(f)
        if entry.get('markers'):
            with np.load(parent/entry['markers']) as m:
                tags.append({k:m[k] for k in ('ids','u','xyz')})
        else:
            tags.append(None)
    theta=np.deg2rad([a['angle_deg'] for a in files])
    result=stitch(frames,tags,theta,model)
    write_scene(output/'推荐.ply',frames,result['poses'],range(len(frames)))
    dump(output/'配准.json',result)
    print(result['selected'],output/'推荐.ply')


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('manifest',type=Path)
    p.add_argument('output',type=Path)
    a=p.parse_args()
    cv.setNumThreads(1);cv.ocl.setUseOpenCL(False);cv.setRNGSeed(104);o3d.utility.random.seed(104)
    main(a.manifest,a.output)
