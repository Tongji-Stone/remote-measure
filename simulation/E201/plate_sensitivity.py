"""Quantify the effect of multi-view exposed-board evidence on each pyramid."""
import cv2 as cv
import open3d as o3d

from check_plate import inspect, plate_views
from common import HERE,PYRAMIDS,audit,dump,measured,save_mesh
from plate_closure import precise_closure


def run(name):
    folder=HERE/name
    case=measured(name)
    scan=o3d.io.read_triangle_mesh(str(folder/'S01-Poisson.ply'))
    result=precise_closure(scan,case,.55,exposed=plate_views(name))
    key='S09-ExposedPlate'
    save_mesh(folder/f'{key}.ply',result,case['p'],case['rgb'],neutral_missing=True)
    record=audit(result,case)
    dump(folder/'露板补充实验.json',{key:record})
    inspect(name)
    print(name,record['volume_ml'],record['observed_P95_mm'],record['boundary'],flush=True)


if __name__=='__main__':
    cv.setNumThreads(1)
    for name in PYRAMIDS:
        run(name)
