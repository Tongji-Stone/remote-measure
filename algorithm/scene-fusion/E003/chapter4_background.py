"""Test the thesis's voxel, RANSAC background and SOR sequence on our turntable."""
import csv

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from registration_pipeline import (KEYS, OUT, PRIOR, cloud, csv_write, evaluate_masks, load, load_case,
                                   ply, table_basis, transform, VIEW_COLORS)


def filter_scan(s,original,pose,axis,center):
    world=transform(s["xyz"],pose)
    B=table_basis(axis)
    local=(world-center)@B
    radial=np.linalg.norm(local[:,:2],axis=1)
    annulus=(radial>26)&(radial<78)&(abs(local[:,2])<12)&(s["error"]<7.)
    disk=cloud(local[annulus],voxel=1.8)
    active=original.copy()
    plane_count=0
    if len(disk.points)>=40:
        coeff,inds=disk.segment_plane(distance_threshold=1.35,ransac_n=3,num_iterations=230)
        if abs(coeff[2])>=.975 and abs(coeff[3])<10:
            active&=(abs(local@coeff[:3]+coeff[3])>1.55)
            plane_count=len(inds)
    p=local[active]
    if len(p)>=35:
        reduced=cloud(p,voxel=1.2)
        reduced,inds=reduced.remove_statistical_outlier(nb_neighbors=16,std_ratio=2.0)
        if len(inds)>=12:
            active[active]=cKDTree(np.asarray(reduced.points)).query(p)[0]<1.78
    return active,plane_count


def main():
    o3d.utility.random.seed(441)
    file=OUT/"E003-01.csv"
    with file.open(encoding="utf-8-sig") as f:
        rows=[r for r in csv.DictReader(f) if r["filter"]!="D03"]
    for j,key in enumerate(KEYS,1):
        tag=f"E003-S{j:02d}"
        e2=tag.replace("E003","E002")
        e2_meta=load(PRIOR/"clouds"/e2/f"{e2}-03.json")
        entry=load(OUT/tag/f"{tag}-01.json")
        poses=np.asarray(entry["poses"])
        _,_,_,scans,_=load_case(key)
        xyz=[];colors=[]
        for i,s in enumerate(scans):
            T=poses[i] if i<len(poses) else poses[0]
            with np.load(PRIOR/"cache"/e2/f"V{i:02d}.npz") as f:
                active,n=filter_scan(s,f[e2_meta["separation"]],T,np.array(entry["axis"]),np.array(entry["center"]))
                stats=evaluate_masks(key,i,s,{**dict.fromkeys([f"M{k:02d}" for k in range(1,8)],active),"h":f["h"]})
            row={"id":tag,"view":i,"filter":"D03","point_count":int(active.sum()),"ransac_deck_voxel_inliers":n}
            if stats:
                row.update(stats["test"]["M01"])
            rows.append(row)
            if i<len(poses):
                xyz.append(transform(s["xyz"][active],T))
                colors.append(np.broadcast_to(VIEW_COLORS[i%len(VIEW_COLORS)],(active.sum(),3)))
        ply(OUT/tag/f"{tag}-D03.ply",np.concatenate(xyz),np.concatenate(colors))
    csv_write(file,rows)


if __name__=="__main__":
    main()
