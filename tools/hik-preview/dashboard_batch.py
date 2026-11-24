"""Reconstruct a scan session and run the experimental fixed-axis merge."""
import argparse
import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
ROOT = SCRIPTS.parents[1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('scan', type=Path)
    ap.add_argument('calibration', type=Path)
    ap.add_argument('axis', type=Path)
    ap.add_argument('--bits', type=int, choices=[0, 1, 2], default=0)
    ap.add_argument('--step', type=int, default=4)
    ap.add_argument('--no-crop', action='store_true')
    ap.add_argument('--angles', type=Path)
    ap.add_argument('--radius', type=float, default=55)
    ap.add_argument('--clearance', type=float, default=3)
    args = ap.parse_args()
    scan = args.scan.resolve(strict=True)
    calibration = args.calibration.resolve(strict=True)
    axis = args.axis.resolve(strict=True)
    session = json.loads((scan / 'session.json').read_text(encoding='utf8'))
    angles = json.loads(args.angles.read_text(encoding='utf8')) if args.angles else None
    entries = []
    for item in session.get('views', []):
        name = item['folder']
        angle = angles.get(name) if angles is not None else item.get('nominal_angle_degrees')
        folder = scan / name
        if angle is not None and abs(angle) < 360 and folder.is_dir():
            if len(list(folder.glob('graycode_*.png'))) != 44:
                raise ValueError(f'图像不完整: {folder}')
            entries.append(folder)
    if len(entries) < 2:
        raise ValueError('需要至少两个角度在 0 到 360 度之间的完整视角；请检查 session.json 或角度修正文件')
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    pattern = f'dashboard_{stamp}_ignore{{bits}}.ply'
    suffix = f'_dashboard_{stamp}'
    for folder in entries:
        print(f'重建 {folder.name}', flush=True)
        subprocess.run([sys.executable, str(SCRIPTS / 'reconstruct_object.py'), str(folder), str(calibration),
                        '--step', str(args.step), '--ignore-finest-bits', str(args.bits),
                        '--output', pattern.format(bits=args.bits)], cwd=ROOT, check=True)
    command = [sys.executable, str(SCRIPTS / 'merge_turntable_experiment.py'), str(scan), str(axis), '--bits', str(args.bits),
               '--input-pattern', pattern, '--suffix', suffix, '--radius', str(args.radius), '--base-clearance', str(args.clearance)]
    if args.no_crop:
        command.append('--no-crop')
    if args.angles:
        command.extend(['--angles', str(args.angles.resolve(strict=True))])
    subprocess.run(command, cwd=ROOT, check=True)
    print('拼接结果:', scan / f'merged_ignore{args.bits}{suffix}_experimental.ply', flush=True)


if __name__ == '__main__':
    main()
