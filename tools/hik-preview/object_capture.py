"""Capture stationary object views while the turntable is moved manually."""
import sys
import argparse
from calibration_capture import main

if __name__ == '__main__':
    try:
        print('通过 ESP32 网页手动转动，转停后再采集。', flush=True)
        parser = argparse.ArgumentParser()
        parser.add_argument('--step', type=float, help='每次旋转的名义角度')
        parser.add_argument('--auto-exposure', action='store_true')
        parser.add_argument('--exposure-us', type=float)
        parser.add_argument('--gain-db', type=float)
        args = parser.parse_args()
        if args.step is None:
            raw = input('每次计划转多少度？输入正数或负数，直接回车表示不记录角度：').strip()
            step = float(raw) if raw else None
        else:
            step = args.step
        if step is not None and (not -360 <= step <= 360 or step == 0):
            raise ValueError('角度应为 -360 到 360 之间的非零数值')
        main(mode='object', nominal_step_degrees=step, exposure_us=args.exposure_us, gain_db=args.gain_db, auto_exposure=args.auto_exposure)
    except Exception as exc:
        print('Error:', exc, flush=True)
        sys.exit(1)
