"""Validate foreground masks against independently drawn original white-frame outlines."""
import csv
import json

import cv2 as cv
import numpy as np

from foreground import HERE, CACHE, BRANCHES, SCANS, METHODS, directory, dump


def silhouette_mask(uv, polygon, shape, boundary):
    # Rasterize locally to make distance-to-boundary inexpensive.
    around=polygon.min(0)-boundary-30
    end=polygon.max(0)+boundary+30
    area=np.zeros(tuple((end-around+1)[::-1]),np.uint8)
    cv.fillPoly(area,[polygon-around],1)
    large=cv.getStructuringElement(cv.MORPH_ELLIPSE,(2*boundary+1,)*2)
    inner=cv.erode(area,large)
    outer=cv.dilate(area,large)
    u=uv.astype(int)-around
    inside=((u>=0)&(u<np.array(area.shape)[::-1])).all(1)
    positive=np.zeros(len(u),bool)
    negative=np.ones(len(u),bool)
    xy=u[inside]
    positive[inside]=inner[xy[:,1],xy[:,0]]>0
    negative[inside]=outer[xy[:,1],xy[:,0]]==0
    # The top of the cylinder is outside the annotated window in these frames.
    visible=uv[:,1]> shape
    return positive&visible,negative&visible


def main():
    manual=json.loads((HERE/'白场抽查轮廓.json').read_text())
    rows=[]
    for scan in SCANS:
        name=scan.name.split('_',4)[4]
        for b in BRANCHES:
            mdir=CACHE/scan.name/b
            for frame,outline in manual['polygons'][scan.name[14:20]].items():
                d=np.load(directory(scan)/b/f'{int(frame):02}.npz')
                seg=np.load(mdir/f'{int(frame):02}.npz')
                polygon=np.array(outline,np.int32)*manual['camera_px_per_coordinate']+np.array(manual['origin_camera_px'],np.int32)
                pos,neg=silhouette_mask(d['uv'],polygon,manual['origin_camera_px'][1]+15,manual['boundary_ignore_camera_px'])
                near=pos&(seg['height']>=-.85)&(seg['height']<=5.)
                for m in (*METHODS,'e102'):
                    keep=seg[m]
                    tp,fp,fn=(keep&pos).sum(),(keep&neg).sum(),(~keep&pos).sum()
                    rows.append(dict(branch=b,object=name,view=int(frame),method=m,
                                     TP=int(tp),FP=int(fp),FN=int(fn),
                                     F1=float(2*tp/max(2*tp+fp+fn,1)),
                                     low_kept=int((keep&near).sum()),low_input=int(near.sum()),
                                     board_removed=int((~keep&seg['board']).sum()),board_input=int(seg['board'].sum())))
    dst=HERE.parent/'E103'
    dst.mkdir(exist_ok=True)
    dump(dst/'分割验证.json',rows)
    with (dst/'分割验证.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0])); w.writeheader();w.writerows(rows)
    for b in BRANCHES:
        for m in METHODS:
            a=[x for x in rows if x['branch']==b and x['method']==m]
            low=sum(x['low_kept'] for x in a)
            print(b,m,np.median([x['F1'] for x in a]),low,sum(x['low_input'] for x in a),
                  sum(x['board_removed'] for x in a),sum(x['board_input'] for x in a))


if __name__=='__main__':
    cv.setNumThreads(1)
    main()
