"""Run a structured-light scan with an ESP32 turntable."""
import argparse
import sys
from turntable_control import Turntable


def main():
    parser = argparse.ArgumentParser(description='Capture automatically after each 512-half-step turn.')
    parser.add_argument('--exposure-us', type=float)
    parser.add_argument('--gain-db', type=float)
    parser.add_argument('--url', default='http://192.168.4.1', help='ESP32 address (default: http://192.168.4.1)')
    parser.add_argument('--views', type=int, default=8, help='Number of non-repeat views around one turn')
    parser.add_argument('--auto-exposure', action='store_true')
    parser.add_argument('--loop', action='store_true', help='Capture an additional 360-degree repeat view')
    parser.add_argument('--settle', type=float, default=1.0, help='Seconds to wait after movement stops')
    args = parser.parse_args()
    url = args.url.strip()
    if url and '://' not in url:
        url = 'http://' + url
    if not 1 <= args.views <= 128:
        raise ValueError('采集组数应为 1～128')
    steps = round(4096 / args.views)
    angle = steps * 360.0 / 4096.0
    controller = Turntable(url, views=args.views + int(args.loop), steps=steps, settle_seconds=args.settle)
    print('检查转台状态……', flush=True)
    controller.check_idle()
    print(f'将采集 {args.views} 个非回环视角，回环 {int(args.loop)} 组；每次正转 {steps} 半步、间隔 3 ms，名义 {angle:.3f}°。', flush=True)
    print('请保持物体固定并关闭其他控制操作；在预览窗口按空格开始，转动时可按 Q 停止。', flush=True)
    from calibration_capture import main as capture
    capture(mode='object', nominal_step_degrees=angle, auto_turntable=controller, exposure_us=args.exposure_us, gain_db=args.gain_db, auto_exposure=args.auto_exposure)


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('Error:', exc, flush=True)
        sys.exit(1)
