"""Construct two blind-hole depths with equal unshadowed camera observations."""
import json

import numpy as np
import open3d as o3d

from run import GRID, OUT, camera, height, scene, voxel_mesh


def main():
    x,y=GRID[:,:2].T
    hole=(x+16)**2+(y-13)**2<8.5**2
    h=height("B_features",GRID[:,:2])
    h_shallow=h.copy()
    h_shallow[hole]+=3.
    a=scene(voxel_mesh(GRID[:,2]<h_shallow))
    b=scene(voxel_mesh(GRID[:,2]<h))
    comparisons=[]
    for angle in range(0,360,45):
        cam=camera(angle,25)
        rays=cam.rays()
        o,d=rays[...,:3],rays[...,3:]
        mask=np.zeros(rays.shape[:2],bool)
        # Block the entire ray bundle that can observe the hole, independently of either depth.
        for z in np.linspace(43.,49.,13):
            t=(z-o[...,2])/d[...,2]
            xy=o[...,:2]+t[...,None]*d[...,:2]
            mask |= (t>0)&(((xy[...,0]+16)**2+(xy[...,1]-13)**2)<11.0**2)
        values=[q.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy() for q in (a,b)]
        sa,sb=[np.isfinite(q) for q in values]
        both=sa&sb&~mask
        differences=abs(values[0][both]-values[1][both])
        comparisons.append({"camera_deg":angle,"depth_masked_pixels":int(mask.sum()),
                            "raw_silhouette_differing_pixels":int((sa^sb).sum()),
                            "visible_unmasked_depth_differing_pixels":int((differences>1e-3).sum()),
                            "visible_unmasked_depth_max_delta_mm":float(differences.max())})
    result={"hole_radius_mm":8.5,"depths_mm":[1.5,4.5],
            "analytic_volume_difference_mm3":float(np.pi*8.5**2*3),
            "observations":comparisons}
    (OUT/"hidden_hole_ambiguity.json").write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))

if __name__=="__main__":
    main()
