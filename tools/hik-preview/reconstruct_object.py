"""Experimental single-view Gray-code triangulation; calibration is unverified."""
import argparse
import math
from pathlib import Path

import cv2
import numpy as np


def image(path):
    frame = cv2.imdecode(np.fromfile(str(path), np.uint8), cv2.IMREAD_GRAYSCALE)
    if frame is None:
        raise ValueError(f'Cannot read {path}')
    return frame


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('capture', type=Path)
    parser.add_argument('calibration', type=Path)
    parser.add_argument('--step', type=int, default=2)
    parser.add_argument('--ignore-finest-bits', type=int, choices=(0, 1, 2), default=0)
    parser.add_argument('--black-threshold', type=int, default=40)
    parser.add_argument('--white-threshold', type=int, default=5)
    parser.add_argument('--roi', type=int, nargs=4, metavar=('X1', 'Y1', 'X2', 'Y2'))
    parser.add_argument('--output', default='experimental_point_cloud.ply')
    parser.add_argument('--pixel-map', help='Optional output NPZ mapping camera pixels to reconstructed XYZ')
    args = parser.parse_args()
    if args.step < 1:
        parser.error('step must be positive')
    files = sorted(args.capture.glob('graycode_*.png'))
    w, h = 1280, 720
    count = 2 * (math.ceil(math.log2(w)) + math.ceil(math.log2(h))) + 2
    if len(files) != count:
        raise ValueError(f'Expected {count} frames, found {len(files)}')
    xml = args.calibration.read_text(encoding='utf-8')
    fs = cv2.FileStorage(xml, cv2.FILE_STORAGE_READ | cv2.FILE_STORAGE_MEMORY)
    if not fs.isOpened():
        raise ValueError('Calibration file could not be opened')
    camera, cd = fs.getNode('cam_int').mat(), fs.getNode('cam_dist').mat()
    projector, pd = fs.getNode('proj_int').mat(), fs.getNode('proj_dist').mat()
    rot, trans = fs.getNode('rotation').mat(), fs.getNode('translation').mat()
    fs.release()
    if any(x is None for x in (camera, cd, projector, pd, rot, trans)):
        raise ValueError('Missing calibration matrices')
    white, black = image(files[-2])[::args.step, ::args.step], image(files[-1])[::args.step, ::args.step]
    valid = white.astype(np.int16) - black.astype(np.int16) > args.black_threshold
    if args.roi:
        x1, y1, x2, y2 = args.roi
        if x1 >= x2 or y1 >= y2:
            parser.error('invalid ROI')
        rows, cols = np.indices(valid.shape)
        valid &= (cols * args.step >= x1) & (cols * args.step < x2) & (rows * args.step >= y1) & (rows * args.step < y2)
    grays = []
    idx = 0
    for nbits in (math.ceil(math.log2(w)), math.ceil(math.log2(h))):
        code = np.zeros(white.shape, np.uint16)
        for bit in range(nbits):
            if bit >= nbits - args.ignore_finest_bits:
                code <<= 1
                idx += 2
                continue
            a = image(files[idx])[::args.step, ::args.step].astype(np.int16)
            b = image(files[idx+1])[::args.step, ::args.step].astype(np.int16)
            delta = a-b
            valid &= np.abs(delta) > args.white_threshold
            code = (code << 1) | (delta > 0).astype(np.uint16)
            idx += 2
        binary = code.copy()
        for shift in (1, 2, 4, 8):
            binary ^= binary >> shift
        block = 1 << args.ignore_finest_bits
        binary = (binary // block) * block
        grays.append(binary)
    px, py = grays
    valid &= (px < w) & (py < h)
    yy, xx = np.nonzero(valid)
    print(f'Valid decoded camera pixels: {len(xx)}', flush=True)
    if not len(xx):
        raise ValueError('No valid decoded pixels')
    cam_points = np.column_stack((xx * args.step, yy * args.step)).astype(np.float64)
    proj_points = np.column_stack((px[valid], py[valid])).astype(np.float64)
    proj_points += ((1 << args.ignore_finest_bits) - 1) / 2.0
    cam_norm = cv2.undistortPoints(cam_points.reshape(-1, 1, 2), camera, cd).reshape(-1, 2)
    proj_norm = cv2.undistortPoints(proj_points.reshape(-1, 1, 2), projector, pd).reshape(-1, 2)
    proj0 = np.hstack((np.eye(3), np.zeros((3, 1))))
    proj1 = np.hstack((rot, trans.reshape(3, 1)))
    homogeneous = cv2.triangulatePoints(proj0, proj1, cam_norm.T, proj_norm.T)
    with np.errstate(divide='ignore', invalid='ignore'):
        points = (homogeneous[:3] / homogeneous[3]).T
    projector_z = (points @ rot.T + trans.reshape(1, 3))[:, 2]
    keep = np.isfinite(points).all(axis=1) & (points[:, 2] > 0) & (projector_z > 0)
    points = points[keep].astype('<f4')
    if not len(points):
        raise ValueError('No points remain in front of both camera and projector; check calibration, Gray-code ordering, projector mapping and capture synchronization')
    if args.pixel_map:
        np.savez_compressed(args.capture / args.pixel_map, camera_pixels=cam_points[keep].astype(np.int32), xyz=points)
    brightness = white[valid][keep]
    out = args.capture / args.output
    records = np.empty(len(points), dtype=[('xyz', '<f4', (3,)), ('rgb', 'u1', (3,))])
    records['xyz'] = points
    records['rgb'] = brightness[:, None]
    header = (f'ply\nformat binary_little_endian 1.0\ncomment Experimental unverified calibration; scale follows calibration board square size\nelement vertex {len(points)}\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
    with out.open('wb') as f:
        f.write(header.encode('ascii'))
        records.tofile(f)
    print(f'Saved {len(points)} points to {out}', flush=True)
    if len(points):
        print('Depth percentiles (mm):', np.percentile(points[:, 2], [1, 50, 99]), flush=True)


if __name__ == '__main__':
    main()
