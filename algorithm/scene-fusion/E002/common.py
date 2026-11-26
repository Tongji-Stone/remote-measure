"""Common data I/O for E002. E001's optical inputs remain immutable."""
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
from optics import ROOT, OUT as PREV, CACHE, CALIBRATION, CAL16, CAL20, load, dump, load_calibration, with_arrays, transform

import cv2
import numpy as np
import open3d as o3d

OUTPUT=HERE/"results"
KEYS=("minion","apple","box_1116","charger_1716","charger","box_center","box_offset","box_1750")
NAMES={"minion":"小黄人","apple":"苹果","box_1116":"纸盒 11/16","charger_1716":"充电头 17:16",
       "charger":"充电头 17:41","box_center":"纸盒 17:45","box_offset":"纸盒 17:47","box_1750":"纸盒 17:50"}
IDS={k:f"E002-S{i:02}" for i,k in enumerate(KEYS,1)}
BOX={"box_1116","box_center","box_offset","box_1750"}
ANGLESIZE=(245,205)


def load_case(key):
    optics=load(PREV/"optical_inputs.json")[key]
    previous=load(PREV/"clouds"/key/"poses_and_metrics.json")
    poses=np.asarray(previous["poses"])
    scenes=[]
    for view,p in zip(optics["views"],sorted((CACHE/key).glob("*.npz")),strict=True):
        with np.load(p) as d:
            s={k:d[k] for k in d.files}
        s["image"]=cv2.imread(str(ROOT/optics["path"]/view["frame"]/"graycode_42.png"),0)
        scenes.append(s)
    match optics["era"]:
        case "1123_aruco":
            cal=with_arrays(load(PREV/"projector_validation.json")["selected_calibration"])
        case "1123_pre": cal=load_calibration(CALIBRATION)
        case "1120": cal=load_calibration(CAL20)
        case "1116": cal=load_calibration(CAL16)
    return optics,previous,poses,scenes,cal


def table_basis(up):
    a=np.array([1.,0.,0.])-up[0]*up
    a/=np.linalg.norm(a)
    return np.column_stack((a,np.cross(up,a),up))


def ply(path,xyz,gray=None):
    path.parent.mkdir(parents=True,exist_ok=True)
    p=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(xyz,float)))
    if gray is not None:
        gray=np.asarray(gray,float)
        color=np.broadcast_to(gray[:,None],(len(xyz),3)) if gray.ndim==1 else gray
        p.colors=o3d.utility.Vector3dVector(color)
    if not o3d.io.write_point_cloud(str(path),p):
        raise IOError(path)


def xyzread(path):
    p=o3d.io.read_point_cloud(str(path))
    return np.asarray(p.points).copy()
