"""Use a JSON manifest to stitch the prepared camera-space views of one turn."""
import argparse
import json
from pathlib import Path

import cv2 as cv
import numpy as np
import open3d as o3d

from camera_geometry import write_scene
from turntable import stitch


def read_view(path):
    with np.load(path, allow_pickle=False) as data:
        frame = {key: data[key] for key in ('xyz', 'uv', 'gray', 'shape')}
        frame.update({key: data[key] for key in ('height', 'rgb') if key in data})
    points, pixels, gray, shape = (frame[key] for key in ('xyz', 'uv', 'gray', 'shape'))
    count = len(points)
    if (not count or points.shape != (count, 3) or pixels.shape != (count, 2)
            or gray.shape != (count,) or shape.shape != (2,) or np.any(shape <= 0)
            or np.any(pixels < 0) or np.any(pixels >= shape[::-1])
            or not all(np.isfinite(array).all() for array in frame.values())):
        raise ValueError(f'Invalid xyz, uv, gray or image shape in {path}')
    if 'height' in frame and frame['height'].shape != (count,):
        raise ValueError(f'height must be an N-vector in {path}')
    if 'rgb' in frame and (frame['rgb'].shape != (count, 3) or frame['rgb'].dtype != np.uint8):
        raise ValueError(f'rgb must be an N-by-3 uint8 array in {path}')
    return frame


def read_markers(path):
    with np.load(path, allow_pickle=False) as data:
        markers = {key: data[key] for key in ('ids', 'u', 'xyz')}
    count = len(markers['ids'])
    if (markers['ids'].shape != (count,) or markers['ids'].dtype.kind not in 'iu'
            or markers['u'].shape != (count, 4, 2) or markers['xyz'].shape != (count, 4, 3)
            or not all(np.isfinite(array).all() for array in markers.values())):
        raise ValueError(f'Invalid marker corners in {path}')
    return markers


def run(manifest, output):
    data = json.loads(manifest.read_text(encoding='utf-8'))
    indir = manifest.parent
    calibration = json.loads((indir / data['calibration']).read_text(encoding='utf-8'))
    camera = calibration.get('hik_projector', calibration)
    model = {key: np.asarray(camera[key], dtype=float) for key in ('cam_int', 'cam_dist')}
    frames = [read_view(indir / entry['cloud']) for entry in data['angles']]
    if not frames or any(not np.array_equal(f['shape'], frames[0]['shape']) for f in frames):
        raise ValueError('All stations must use the same calibrated image dimensions.')
    tags = [read_markers(indir / entry['markers']) if entry.get('markers') else None
            for entry in data['angles']]
    angles = np.deg2rad([entry['angle_deg'] for entry in data['angles']])
    result = stitch(frames, tags, angles, model)
    output.mkdir(parents=True, exist_ok=True)
    count = write_scene(output / 'registered.ply', frames, result['poses'], range(len(frames)))
    (output / 'registration.json').write_text(
        json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2,
                   default=lambda x: x.tolist()) + '\n', encoding='utf-8')
    print(f'{result["selected"]}: {count:,} vertices in {output / "registered.ply"}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path, help='Input JSON with calibration and per-angle observations')
    parser.add_argument('output', type=Path, help='Directory for registered.ply and registration.json')
    args = parser.parse_args()
    cv.setNumThreads(1)
    cv.setRNGSeed(104)
    o3d.utility.random.seed(104)
    run(args.manifest, args.output)
