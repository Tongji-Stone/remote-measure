"""Keep the pre-17:41 reconstruction separate from the later ArUco benchmark."""
import json
from pathlib import Path

import numpy as np

from calibrated_registration_benchmark import OUTPUT, dump, save_cloud, pair_metric, RING, cloud_sample
from calibrate_benchmark_turntable import roi_mask
from prepare_calibrated_benchmark import SCANS
from stitch_turntable import axis_transform


def main():
    raw=OUTPUT/"raw/171640"
    scan=next(SCANS.glob("scan_20261123_171640*"))
    source=next(scan.glob("axis_*.json"))
    c=json.loads(source.read_text())
    n=-np.array(c["axis_direction"],float)
    n/=np.linalg.norm(n)
    center=np.array(c["axis_point_camera_mm"])
    views=[]
    for i in range(8):
        a=np.load(raw/f"{i:02}.npz")
        h=a["xyz"]@n-c["plane"][3]
        keep=roi_mask(a["uv"],"171640",i,60)&(h>6)&(h<75)
        views.append(cloud_sample(a["xyz"][keep],a["uv"][keep],a["gray"][keep]))
    dst=OUTPUT/"history"
    dst.mkdir(exist_ok=True)
    estimates={}
    for sign in (1,-1):
        poses=np.array([axis_transform(n,center,45*i*sign) for i in range(8)])
        edges=[pair_metric(views[a],views[b],np.linalg.inv(poses[b])@poses[a]) for a,b in RING]
        estimates[str(sign)]={"f1_2_median":float(np.median([x["f1_2"] for x in edges if "f1_2" in x])),
                              "scored_edges":sum("f1_2" in x for x in edges)}
        save_cloud(dst/f"171640_nominal_sign_{sign:+d}.ply",views,poses,range(8))
    repeat=next(SCANS.rglob("loop_0000_0004_analysis.json"))
    rr=json.loads(repeat.read_text())
    dump(dst/"history_diagnostics.json",{"session":scan.name,"axis_source":source.name,"count":[len(v["p"]) for v in views],
           "sign_check":estimates,"registered_marker_reference":False,
           "20261116_repeat_archive":{"source":str(repeat.relative_to(SCANS)),"before_forward_p50":rr["holdout_0004_to_0000_before"]["median_mm"],
                                      "before_backward_p50":rr["holdout_0000_to_0004_before"]["median_mm"]}})
    

if __name__ == "__main__":
    main()
