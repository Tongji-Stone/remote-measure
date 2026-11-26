"""Use E102 checkerboard observations as measured free space along the contact plane."""
from pathlib import Path
import hashlib

import numpy as np
from scipy.spatial import cKDTree

from reconstruct import tx

HERE=Path(__file__).resolve().parent
CACHE=Path('/tmp/E102/masks')
RAW=Path('/tmp/E101/cache/object_scans')


def exposed_views(name,branch,e102,poses,world,evidence_path=None):
    """Recover pixels marked as bare board by E102, validating their exported coordinate order."""
    hashes=np.array([hashlib.sha256(np.asarray(p,dtype='<f4').tobytes()).hexdigest() for p in e102])
    if evidence_path is not None and evidence_path.is_file():
        with np.load(evidence_path) as evidence:
            if np.array_equal(evidence['sha256_xyz'],hashes):
                return [tx(points[evidence[f'v{i:02}']],world@T) for i,(points,T) in enumerate(zip(e102,poses))]
    scans=list(CACHE.glob(f'scan_*_{name}'))
    if len(scans)!=1:
        raise FileNotFoundError(f'Run E101/E102 preprocessing for {name} to recover board-pixel masks.')
    raw=RAW/scans[0].name/branch
    mask=scans[0]/branch
    points=[]
    evidence={'sha256_xyz':hashes,'e102_scan':scans[0].name,'branch':branch}
    for i,(ply,T) in enumerate(zip(e102,poses)):
        with np.load(raw/f'{i:02}.npz') as record, np.load(mask/f'{i:02}.npz') as region:
            source=record['xyz']
            full=region['e102']
            valid=region['board'][full]
            if not np.array_equal(source[full],ply):
                raise ValueError(f'E102 mask provenance mismatch in {name}/{branch}/V{i:02}')
            idx=np.flatnonzero(valid).astype('int32')
            points.append(tx(ply[idx],world@T))
            evidence[f'v{i:02}']=idx
    if evidence_path is not None:
        temp=evidence_path.with_suffix('.part.npz')
        np.savez_compressed(temp,**evidence)
        temp.replace(evidence_path)
    return points


def conservative_votes(xy,observations,radius=1.05):
    votes=np.zeros(len(xy),np.uint8)
    for candidate in observations:
        if len(candidate)<50:
            continue
        z=candidate[:,2]
        m=np.median(z)
        scale=1.4826*np.median(abs(z-m))
        valid=abs(z-m)<=max(.75, min(2.,3.*scale))
        if valid.sum()>=50:
            votes+=(cKDTree(candidate[valid,:2]).query(xy)[0]<=radius)
    return votes


def contact_carve(xy,exposed,p):
    """Carve confidently exposed XY cells, except where the split contains direct object support."""
    empty=conservative_votes(xy,exposed)>=2
    measured=p[(p[:,2]>=-.7)&(p[:,2]<=1.3)]
    if len(measured):
        empty&=cKDTree(measured[:,:2]).query(xy)[0]>.8
    return empty
