"""Reproduce the measured E201 scenarios."""
import argparse

import cv2 as cv

from common import GROUPS,HERE,PYRAMIDS,as_mesh,audit,dump,measured,save_mesh
from complete_splats import finish
from plate_closure import run as precise
from splatting import run_splat
from traditional import traditional


def execute(name, phase):
    case=measured(name)
    root=HERE/name
    root.mkdir(exist_ok=True)
    dump(root/'底面标定.json',case['datum'])
    dump(root/'输入.json',dict(point_cloud=case['source'],stations=len(case['raw'])))
    if phase in ('all','classic'):
        results,times=traditional(case)
        stats={}
        for code,m in results.items():
            if as_mesh(m):
                save_mesh(root/f'{code}.ply',m,case['p'],case['rgb'],neutral_missing=name in PYRAMIDS)
            stats[code]=dict(**audit(m,case),elapsed_s=times.get(code,0))
        for code in times.keys()-results.keys():
            stats[code]=dict(status='empty',elapsed_s=times[code])
        dump(root/'旧方法.json',stats)
    if phase in ('all','gaussian'):
        info,_=run_splat(case,root)
        dump(root/'高斯方法.json',info)
        finish(case,root)
    if phase in ('all','precision'):
        precise(case,root)


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--object',choices=GROUPS)
    p.add_argument('--phase',choices=('all','classic','gaussian','precision'),default='all')
    args=p.parse_args()
    cv.setNumThreads(1)
    for name in ([args.object] if args.object else GROUPS):
        execute(name,args.phase)
