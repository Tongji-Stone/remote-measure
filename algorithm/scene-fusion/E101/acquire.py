"""Decode E101 captures on a shared, full camera grid."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import sys

import cv2 as cv
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
CACHE = Path('/tmp/E101/cache')
BRANCHES = ('ChArUco', 'Chessboard')
CAL_DIR = ROOT / 'data/procam_calibration'
SCAN_DIR = ROOT / 'data/object_scans'
CALIBRATIONS = dict(zip(BRANCHES, sorted(CAL_DIR.glob('calibration_20261125_*'))))
SCANS = sorted(SCAN_DIR.glob('scan_20261125_*'))
STEP = 3
sys.path.insert(0, str(ROOT / 'algorithm'))
from prepare_calibrated_benchmark import load_frame, triangulate


def directory(folder):
    return CACHE / folder.parent.name / folder.name


def inputs():
    for station in [*CALIBRATIONS.values(), *SCANS]:
        yield from sorted(station.glob('capture_*' if station in CALIBRATIONS.values() else 'view_*'))


def decode(folder):
    path = directory(folder) / 'gray.npz'
    if path.is_file():
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    white = load_frame(folder, 42)
    black = load_frame(folder, 43)
    contrast = white[::STEP, ::STEP].astype(np.int16) - black[::STEP, ::STEP]
    y, x = (a * STEP for a in np.nonzero(contrast >= 14))
    uv = np.c_[x, y].astype(np.uint16)
    signals = []
    for i in range(21):
        a = load_frame(folder, 2 * i)[y, x].astype(np.int16)
        b = load_frame(folder, 2 * i + 1)[y, x]
        signals.append(a - b)
    codes, quality = [], []
    for ignored in (0, 1, 2):
        axes = []
        good = np.ones(len(x), bool)
        for start, width, extent in ((0, 11, 1280), (11, 10, 720)):
            values = np.array(signals[start:start + width - ignored])
            good &= (np.abs(values) >= 5).all(0)
            binary = np.logical_xor.accumulate(values > 0, axis=0)
            coord = (binary * (1 << np.arange(width - 1, ignored - 1, -1))[:, None]).sum(0)
            coord = 2 * coord + (1 << ignored) - 1
            good &= coord < 2 * extent
            axes.append(coord)
        codes.append(np.c_[*axes].astype(np.uint16))
        quality.append(good)
    np.savez(path.with_suffix('.part.npz'), uv=uv, gray=white[y, x],
             shape=white.shape, codes=codes, good=quality)
    path.with_suffix('.part.npz').replace(path)
    print(folder.parent.name[14:20], folder.name, len(x), np.count_nonzero(quality, axis=1), flush=True)


if __name__ == '__main__':
    cv.setNumThreads(1)
    with ThreadPoolExecutor(max_workers=4) as pool:
        list(pool.map(decode, inputs()))
