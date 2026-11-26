"""Compare reconstructed deck depths at the same held-out marker pixels."""
import cv2 as cv
import numpy as np

from acquire import BRANCHES, HERE, SCANS, directory
from calibrate import dump


def marker_mask(u, polygons):
    keep = np.zeros(len(u),bool)
    for corners in polygons:
        lo, hi = np.floor(corners.min(0)-2),np.ceil(corners.max(0)+2)
        ix = np.where(np.all((u>=lo)&(u<=hi),axis=1))[0]
        H = cv.getPerspectiveTransform(corners.astype('float32'),
                                       np.array([[0,0],[1,0],[1,1],[0,1]],'float32'))
        q = cv.perspectiveTransform(u[ix].astype(float)[None],H)[0]
        keep[ix] |= ((q>=-.035)&(q<=1.035)).all(1)
    return keep


def main():
    rows = []
    for scan in SCANS:
        roots = directory(scan)
        folders = [sorted((roots/b).glob('*.npz')) for b in BRANCHES]
        for i,(row_a,row_b) in enumerate(zip(*folders)):
            data = [dict(np.load(f)) for f in (row_a,row_b)]
            common,j,k = np.intersect1d(*[x['grid_index'] for x in data],return_indices=True)
            marks = data[0]['held_uv'].reshape(-1,4,2)
            r = dict(object=scan.name.split('_',4)[4],view=i,marker_count=len(marks))
            good = marker_mask(data[0]['uv'][j],marks)
            r['shared_plane_points']=int(good.sum())
            if good.any():
                for branch,a,idx in zip(BRANCHES,data,[j,k]):
                    center,normal = a['pose'][:3,3],a['pose'][:3,2]
                    d = (a['xyz'][idx[good]]-center) @ normal
                    r[branch] = dict(abs_P50_mm=np.median(np.abs(d)),abs_P95_mm=np.percentile(np.abs(d),95),
                                     signed_P50_mm=np.median(d))
            rows.append(r)
    dump(HERE/'E101光学验证.json',rows)
    for b in BRANCHES:
        usable=[r for r in rows if b in r]
        print(b,len(usable),sum(v['shared_plane_points'] for v in usable),
              np.median([v[b]['abs_P50_mm'] for v in usable]),
              np.median([v[b]['abs_P95_mm'] for v in usable]))


if __name__=='__main__':
    cv.setNumThreads(1)
    main()
