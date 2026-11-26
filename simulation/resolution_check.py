"""Compare reconstruction voxel counts at two pitches on the same high-elevation captures."""
import json

import numpy as np

import run


def main():
    shapes=("A_box","B_features","C_freeform")
    all_views={}
    setup=[(angle,25) for angle in range(0,360,45)]+[(30,76),(190,76)]
    rng=np.random.default_rng(331)
    for case in shapes:
        truth=run.voxel_mesh(run.GRID[:,2]<run.height(case,run.GRID[:,:2]))
        cast=run.scene(truth)
        all_views[case]=[run.observe(cast,run.camera(a,e),run.camera(a+17,e+3),False,rng) for a,e in setup]
    out=[]
    for pitch in (1.5,.75):
        run.PITCH=pitch
        run.SIZE=tuple(np.rint(np.array([120,108,76.5])/pitch).astype(int))
        run.GRID=run.ORIGIN+(np.indices(run.SIZE).reshape(3,-1).T+.5)*pitch
        for k,scans in all_views.items():
            obs=np.concatenate([v["observed"] for v in scans])
            d=run.carve(scans,True)
            hx=run.infer_height(obs,d.reshape(run.SIZE).any(2).ravel())&d
            volume=float(hx.sum()*pitch**3)
            truth=run.analytic_volume(k)
            out.append({"case":k,"pitch_mm":pitch,"occupied_volume_mm3":volume,
                        "volume_error_pct":100*(volume/truth-1)})
    (run.OUT/"resolution_sensitivity.json").write_text(json.dumps({"method":"heightfield_depth",
        "estimator":"occupied voxel volume (not mesh); same raw depth across resolutions","cases":out},indent=2)+"\n")
    print(out)

if __name__=="__main__":
    main()
