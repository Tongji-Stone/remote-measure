"""Audit reconstructed metric lengths at paired ChArUco board pixels."""
import json
from pathlib import Path
import sys

import numpy as np
import cv2
from scipy.spatial import cKDTree
from scipy.optimize import least_squares

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
sys.path.insert(0,str(ROOT/'computing volume/V001'))
from run import save_csv

BRANCHES=('ChArUco','Chessboard')
E101=ROOT/'algorithm/scene-fusion/E101'


def rays(uv,model):
    u=cv2.undistortPoints(uv.astype(float),np.array(model['cam_int']),np.array(model['cam_dist']))[:,0]
    return np.c_[u,np.ones(len(u))]


def measure(raw,idx,held,model):
    u=rays(raw['uv'][idx],model)
    measured=raw['xyz'][idx,2].astype(float)
    q=1000./measured
    beta=np.linalg.lstsq(u,q,rcond=None)[0]
    opt=least_squares(lambda a:u@a-q,beta,loss='soft_l1',f_scale=.014)
    b=opt.x
    projection=rays(held,model)
    reconstructed=projection*(1000./(projection@b))[:,None]
    return reconstructed,float(np.median(abs(1000./(u@b)-measured)))


def pairs(xyz,distances):
    i,j=np.triu_indices(len(xyz),1)
    difference=xyz[i]-xyz[j]
    nominal=np.linalg.norm(difference,axis=1)
    for axis,k in (('x',0),('y',1),('diagonal',None)):
        ok=(nominal>=15.)
        if k is None:
            ok &= (abs(difference[:,0])>=7)&(abs(difference[:,1])>=7)
        else:
            ok &= abs(difference[:,1-k])<.03
        if not ok.any():
            continue
        residual=distances[i[ok],j[ok]]/nominal[ok]-1.
        yield axis,int(ok.sum()),float(np.median(residual)),float(np.percentile(abs(residual),95))


def main():
    models={b:json.loads((E101/b/'标定.json').read_text()) for b in BRANCHES}
    out=[]
    base=Path('/tmp/E101/cache/object_scans')
    dirs=sorted(base.glob('scan_20261125_*'))
    if len(dirs)!=5:
        raise FileNotFoundError('Generate the E101 decoded scans and E102 pixel masks before this audit.')
    for scan in dirs:
        masks=Path('/tmp/E102/masks')/scan.name
        for fa in sorted((scan/'ChArUco').glob('*.npz')):
            fb=scan/'Chessboard'/fa.name
            with np.load(fa) as a, np.load(fb) as b, np.load(masks/'ChArUco'/fa.name) as ma, np.load(masks/'Chessboard'/fa.name) as mb:
                held=a['held_uv']
                if len(held)<8:
                    continue
                assert np.allclose(held,b['held_uv'],atol=.05)
                _,ia,ib=np.intersect1d(a['grid_index'],b['grid_index'],return_indices=True,assume_unique=True)
                take=ma['board'][ia]&mb['board'][ib]
                # Use measured exposed board points near held-out corners in the same image pixels.
                take &= cKDTree(held).query(a['uv'][ia])[0]<=24.
                ia,ib=ia[take],ib[take]
                if len(ia)<120:
                    continue
                ids=np.linspace(0,len(ia)-1,min(2500,len(ia))).astype(int)
                for branch,raw,idx in zip(BRANCHES,(a,b),(ia[ids],ib[ids])):
                    coords,plane=measure(raw,idx,held,models[branch])
                    ds=np.linalg.norm(coords[:,None]-coords[None,:],axis=2)
                    for direction,count,bias,p95 in pairs(a['held_xyz'],ds):
                        out.append(dict(object=scan.name.split('_',4)[-1],station=int(fa.stem),calibration=branch,
                                        direction=direction,pair_count=count,plane_point_count=len(idx),
                                        planar_model_residual_mm=plane,median_signed_scale_error=bias,p95_scale_error=p95))
        print(scan.name,len(out),flush=True)
    save_csv(HERE/'results/calibration_plane_scale.csv',out)


if __name__=='__main__':
    cv2.setNumThreads(1)
    main()
