"""Rebuild camera-frame point clouds and detect turntable fiducials from raw PNGs."""
import argparse
import hashlib
import json
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data"
SCANS = DATA / "object_scans"
OUTPUT = ROOT / "output/calibrated-benchmark"
CALIBRATION = DATA / "procam_calibration/20261123_1711_analysis/calibration_20261123_10mm_experimental.xml"
KEYS = ["cam_int", "cam_dist", "proj_int", "proj_dist", "rotation", "translation"]


def load_calibration(path):
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise FileNotFoundError(path)
    m = {key: fs.getNode(key).mat() for key in KEYS}
    m["rms"] = fs.getNode("rms").real()
    fs.release()
    return m


def load_frame(view, index):
    path = view / f"graycode_{index:02}.png"
    frame = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if frame is None:
        raise ValueError(f"Cannot read {path}")
    return frame


def decode(view, step=4, ignore=1, recover_msb=False, camera_roi=(310,680,1770,1900)):
    # Limit the camera region to the disk and all object silhouettes.
    w, b = (load_frame(view, i) for i in (42, 43))
    left, top, right, bottom = camera_roi
    yy, xx = np.mgrid[top:bottom:step, left:right:step]
    white, black = w[yy, xx], b[yy, xx]
    mask = (white.astype(np.int16) - black >= 15) & (black < 240)
    x, y = xx[mask], yy[mask]
    pixels = np.column_stack((x, y)).astype(np.float64)
    valid = np.ones(len(x), bool)
    codes = []
    for offset, n, extent in ((0, 11, 1280), (11, 10, 720)):
        value = np.zeros(len(x), np.int32)
        bit = np.zeros(len(x), bool)
        for i in range(n - ignore):
            if recover_msb and offset == 0 and i == 0:
                continue
            a, b = (load_frame(view, j)[y, x] for j in (2*(offset+i), 2*(offset+i)+1))
            delta = a.astype(np.int16) - b
            valid &= np.abs(delta) >= 5
            bit ^= delta > 0
            value = value * 2 + bit
        value = ((value << ignore) + (2**ignore-1)/2)
        valid &= value < extent
        codes.append(value)
    return pixels[valid], np.column_stack(codes)[valid], white[mask][valid], int(mask.sum())


def triangulate(cal, uv, st):
    K, D, L, E, R, t = (cal[k] for k in KEYS)
    a = cv2.undistortPoints(uv, K, D).reshape(-1, 2)
    b = cv2.undistortPoints(st, L, E).reshape(-1, 2)
    q = cv2.triangulatePoints(np.eye(3, 4), np.c_[R, t], a.T, b.T).T
    xyz = q[:, :3] / q[:, 3:4]
    c = cv2.projectPoints(xyz, np.zeros(3), np.zeros(3), K, D)[0][:, 0]
    p = cv2.projectPoints(xyz, R, t, L, E)[0][:, 0]
    error = np.stack((np.linalg.norm(c-uv, axis=1), np.linalg.norm(p-st, axis=1)), 1)
    return xyz, error


def detect(view):
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), params)
    choices = {}
    for frame in (42, 43):
        im = load_frame(view, frame)
        small = cv2.resize(im, None, fx=.5, fy=.5, interpolation=cv2.INTER_AREA)
        corners, ids, _ = detector.detectMarkers(small)
        if ids is None:
            continue
        for tag, corner in zip(ids.ravel(), corners):
            if int(tag) not in (0, 1, 2, 3) or int(tag) in choices:
                continue
            refined = cv2.cornerSubPix(im, (corner * 2 + .5).astype(np.float32), (5, 5), (-1,-1),
                                       (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, .01))
            choices[int(tag)] = refined.reshape(4, 2).tolist()
    return choices


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--calibration", type=Path, default=CALIBRATION)
    p.add_argument("--capture", default="20261123")
    p.add_argument("--out", type=Path, default=OUTPUT/"raw")
    args = p.parse_args()
    cal = load_calibration(args.calibration)
    cv2.setNumThreads(2)
    manifest = {}
    for scan in sorted(SCANS.glob(f"scan_{args.capture}*")):
        sid = scan.name[14:20]
        dst = args.out / sid
        dst.mkdir(parents=True, exist_ok=True)
        session = json.loads((scan/"session.json").read_text())
        meta = {"scan": str(scan.relative_to(ROOT)), "calibration": str(args.calibration.resolve().relative_to(ROOT)),
                "calibration_sha256": hashlib.sha256(args.calibration.read_bytes()).hexdigest(),
                "stereo_rms_px": cal["rms"], "baseline_nominal_mm": float(np.linalg.norm(cal["translation"])),
                "views": []}
        for idx, view in enumerate(sorted(scan.glob("view_*"))):
            start = perf_counter()
            uv, st, intensity, n = decode(view)
            xyz, err = triangulate(cal, uv, st)
            good = np.isfinite(xyz).all(1) & (xyz[:,2]>100) & (xyz[:,2]<650) & (err.max(1)<10)
            name = f"{idx:02}.npz"
            np.savez_compressed(dst/name, xyz=xyz[good].astype(np.float32), uv=uv[good].astype(np.uint16),
                                gray=intensity[good], error=err[good].astype(np.float32), projector=st[good].astype(np.float32))
            tags = detect(view)
            meta["views"].append(dict(view=idx, filename=name,
                                      command_deg=session["views"][idx]["nominal_angle_degrees"],
                                      illuminated_samples=n, decoded_samples=len(uv),
                                      triangulated_samples=int(good.sum()),
                                      stereo_p50_px=float(np.median(err[good].max(1))),
                                      tags=tags, preprocess_s=perf_counter()-start))
            print(sid, idx, n, len(uv), good.sum(), sorted(tags), flush=True)
        (dst/"meta.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2))
        manifest[sid] = meta
    (args.out/"manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
