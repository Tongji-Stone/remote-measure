"""Plot identical-scale point-cloud views and the controlled recovery test."""

import argparse
from collections import Counter
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np

NAMES = {
    "nominal": "Old axis",
    "axis_refined": "Axis + ICP",
    "historical_angles": "Archive angles",
    "rim_center": "Rim center",
    "rim_center_refined": "Rim + axis ICP",
    "coarse_face": "Coarse face",
    "point_to_plane": "Point-to-plane",
    "point_to_point": "Point-to-point",
    "tukey_icp": "Tukey ICP",
    "gray_icp": "Gray + ICP",
    "gicp": "GICP",
    "cloudcompare_icp": "CloudCompare",
    "fpfh_ransac": "FPFH-RANSAC",
    "fpfh_fgr": "FPFH-FGR",
    "sift_image": "SIFT image",
    "go_icp": "Go-ICP",
}
PALETTE = np.array([[.10, .48, .71], [.89, .48, .19], [.19, .63, .47], [.65, .32, .65]])
WIDTH = 180 / 25.4
plt.rcParams.update({"font.size": 9, "axes.titlesize": 9, "axes.labelsize": 9, "axes.spines.top": False,
                     "axes.spines.right": False, "font.family": "DejaVu Sans",
                     "text.color": "#163746", "axes.labelcolor": "#163746"})


def read_ply(path):
    formats = {"double": "<f8", "float": "<f4", "uchar": "u1"}
    with path.open("rb") as stream:
        fields = []
        for line in stream:
            if line.startswith(b"property"):
                _, kind, key = line.decode().split()
                fields.append((key, formats[kind]))
            if line.strip() == b"end_header":
                break
        data = np.fromfile(stream, dtype=np.dtype(fields))
    xyz = np.column_stack([data[k] for k in ("x", "y", "z")])
    rgb = np.column_stack([data[k] for k in ("red", "green", "blue")]) / 255
    return xyz, rgb


def camera_basis(vertical, yaw, elevation):
    vertical = np.asarray(vertical, dtype=float)
    vertical /= np.linalg.norm(vertical)
    side = np.array([1., 0, 0]) - vertical * vertical[0]
    side /= np.linalg.norm(side)
    deep = np.cross(vertical, side)
    angle = np.deg2rad(yaw)
    h = np.cos(angle) * side + np.sin(angle) * deep
    view = np.cross(h, vertical)
    e = np.deg2rad(elevation)
    return np.column_stack([h, np.cos(e) * vertical - np.sin(e) * view,
                            np.sin(e) * vertical + np.cos(e) * view])


def clouds(run, item, methods, basis, titles, view_names, output):
    columns = 4
    rows = (len(methods) + columns - 1) // columns
    fig, panels = plt.subplots(rows, columns, figsize=(WIDTH, rows * 1.38 + .34))
    panels = panels.ravel()
    dataset = []
    for method in methods:
        p = run / f"{item}_{method}.ply"
        if p.exists():
            xyz, rgb = read_ply(p)
            dataset.append((xyz @ basis, rgb))
        else:
            dataset.append(None)
    merged = np.vstack([d[0] for d in dataset if d is not None])
    # Every plotted panel uses an identical camera, scale and bounding box.
    if item == "box":
        lo, hi = np.quantile(merged[:, :2], [.003, .997], axis=0)
    else:
        lo, hi = np.min(merged[:, :2], axis=0), np.max(merged[:, :2], axis=0)
    margin = (hi - lo) * .04
    for ax, (method, data, title) in zip(panels, zip(methods, dataset, titles)):
        ax.set_title(NAMES[method] + "\n" + title, pad=4)
        ax.set(xlim=(lo[0] - margin[0], hi[0]+margin[0]),
               ylim=(lo[1] - margin[1], hi[1]+margin[1]), aspect="equal")
        ax.set_xticks([])
        ax.set_yticks([])
        ax.patch.set_facecolor("#f2f7f7")
        for spine in ax.spines.values():
            spine.set_visible(False)
        if data is None:
            ax.text(.5, .5, "No connected result", ha="center", va="center", transform=ax.transAxes)
            continue
        p, rgba = data
        step = 3 if item == "box" else 1
        indexes = np.argsort(p[:, 2])[::step]
        ax.scatter(p[indexes, 0], p[indexes, 1], c=rgba[indexes], linewidth=0,
                   s=.68 if item == "box" else 1.2, alpha=.64, rasterized=True)
    for i, ax in enumerate(panels[len(methods):]):
        ax.axis("off")
        if i == 0:
            ax.text(.04, .55, "Same projection\nand scale", va="center", transform=ax.transAxes)
    fig.legend([Patch(facecolor=c) for c in PALETTE[:len(view_names)]], view_names,
               loc="lower center", ncol=len(view_names), frameon=False, bbox_to_anchor=(.5, .00), fontsize=9)
    fig.subplots_adjust(left=.012, right=.988, bottom=.072, top=.91, wspace=.14, hspace=.55)
    fig.savefig(output, dpi=290)
    plt.close(fig)


def plot_counts(run, output):
    with (run / "known_pose_trials.csv").open() as f:
        rows = list(csv.DictReader(f))
    counts = Counter((r["shape"], r["method"]) for r in rows if r["success"] == "True")
    methods = ["point_to_plane", "point_to_point", "tukey_icp", "gicp", "fpfh_ransac", "fpfh_fgr"]
    assert all(sum(r["shape"] == shape and r["method"] == m for r in rows) == 8
               for shape in ("single_face", "two_faces") for m in methods)
    fig, ax = plt.subplots(figsize=(WIDTH, 2.27))
    y = np.arange(len(methods))
    both_zero = [counts["single_face", m] == counts["two_faces", m] == 0 for m in methods]
    for dy, shape, color, label in [(-.17, "single_face", "#bd773a", "Single face"),
                                     (.17, "two_faces", "#167c75", "Two faces")]:
        results = [counts[shape, method] for method in methods]
        ax.barh(y + dy, results, height=.28, color=color, label=label)
        for i, value in enumerate(results):
            if not both_zero[i]:
                ax.text(value + .15, i + dy, f"{value}/8", va="center", fontsize=9)
    for i, empty in enumerate(both_zero):
        if empty:
            ax.text(.15, i, "0/8 both", va="center", fontsize=9)
    ax.set(yticks=y, yticklabels=[NAMES[m] for m in methods], xlim=(0, 9),
           xticks=range(0, 9, 2), xlabel="Successful recoveries")
    ax.invert_yaxis()
    ax.spines["left"].set_visible(False)
    ax.tick_params(axis="y", length=0, pad=6)
    ax.xaxis.grid(True, alpha=.22)
    ax.set_axisbelow(True)
    ax.legend(loc="upper center", bbox_to_anchor=(.56, 1.21), ncols=2, frameon=False)
    fig.subplots_adjust(left=.20, right=.975, bottom=.20, top=.83)
    fig.savefig(output,dpi=290)
    plt.close(fig)


def main():
    args = argparse.ArgumentParser()
    args.add_argument("run", type=Path)
    args.add_argument("out", type=Path)
    options = args.parse_args()
    out = options.out
    out.mkdir(parents=True, exist_ok=True)
    apple = json.loads((options.run/"apple.json").read_text())
    box = json.loads((options.run/"box.json").read_text())
    apple_methods, box_methods = list(apple["methods"]), list(box["methods"])
    apple_titles = [f'{100 * a["mean_ring_coverage_2"]:.1f}% / {max(a["nominal_correction_deg"]):.1f}°'
                    if a["state"] == "complete" else "Failed" for a in apple["methods"].values()]
    box_titles = [f'{b["image_method_holdout"]["median"]:.2f} U median'
                  if "image_method_holdout" in b else "No result" for b in box["methods"].values()]
    clouds(options.run, "apple", apple_methods, camera_basis(-np.asarray(apple["axis"]), 5, 56),
           apple_titles, ("0 deg", "90 deg", "180 deg", "270 deg"), out/"apple-alignment.png")
    clouds(options.run, "box", box_methods, camera_basis([0, -1, 0], 21, 20),
           box_titles, ("124618", "124643"), out/"box-alignment.png")
    plot_counts(options.run, out/"known-pose.png")


if __name__ == "__main__":
    main()
