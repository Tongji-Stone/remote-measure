"""Capture stationary object views while the turntable is moved manually."""
import sys
from calibration_capture import main

if __name__ == '__main__':
    try:
        print('通过 ESP32 网页手动转动，转停后再采集。', flush=True)
        raw = input('每次计划转多少度？输入正数或负数，直接回车表示不记录角度：').strip()
        step = float(raw) if raw else None
        if step is not None and (not -360 <= step <= 360 or step == 0):
            raise ValueError('角度应为 -360 到 360 之间的非零数值')
        main(mode='object', nominal_step_degrees=step)
    except Exception as exc:
        print('Error:', exc, flush=True)
        sys.exit(1)
