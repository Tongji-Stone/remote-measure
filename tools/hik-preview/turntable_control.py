"""Control the ESP32-CAM turntable and wait for reported movement completion."""
import json
import time
from urllib.parse import urlencode, urlsplit
from urllib.request import ProxyHandler, Request, build_opener


class Turntable:
    def __init__(self, url, views=8, steps=512, interval_ms=3, settle_seconds=1.0, timeout=30.0):
        parts = urlsplit(url.rstrip('/'))
        if parts.scheme != 'http' or not parts.netloc or parts.path or parts.query or parts.fragment:
            raise ValueError('ESP32 address must be like http://192.168.4.1')
        if not 1 <= views <= 129 or not 1 <= steps <= 500000 or not 1 <= interval_ms <= 50 or settle_seconds < 0 or timeout <= 0:
            raise ValueError('Invalid turntable settings')
        self.url = url.rstrip('/')
        self.views, self.steps, self.interval_ms = views, steps, interval_ms
        self.settle_seconds, self.timeout = settle_seconds, timeout
        self.opener = build_opener(ProxyHandler({}))

    def settings(self):
        return {'controller_url': self.url, 'views': self.views, 'steps_per_move': self.steps,
                'interval_ms': self.interval_ms, 'settle_seconds': self.settle_seconds,
                'nominal_degrees_per_move': 360.0 * self.steps / 4096.0, 'angle_is_measured': False}

    def request(self, path, post=False):
        req = Request(self.url + path, data=b'' if post else None, method='POST' if post else 'GET',
                      headers={'Cache-Control': 'no-cache'})
        with self.opener.open(req, timeout=3) as response:
            data = json.load(response)
        if not isinstance(data, dict) or not isinstance(data.get('running'), bool) or not isinstance(data.get('remaining'), int) or data.get('mode') not in ('idle', 'move', 'continuous'):
            raise RuntimeError('Controller response has unexpected status fields')
        return data

    def check_idle(self):
        status = self.request('/api/status')
        if status['running'] or status['mode'] != 'idle' or status['remaining'] != 0:
            raise RuntimeError('Turntable is not idle; stop it before starting')
        if status.get('ledOn') is not False:
            raise RuntimeError('Turn off the shared white LED in the controller first')
        return status

    def stop(self):
        try:
            self.request('/api/stop', post=True)
        except Exception as exc:
            print('Could not confirm motor stop:', exc, flush=True)

    def move_and_wait(self, cancelled=lambda: False):
        self.check_idle()
        commanded = time.monotonic()
        deadline = commanded + self.timeout
        try:
            # Do not retry: if the response is lost, the first command may have moved it.
            status = self.request('/api/move?' + urlencode({'steps': self.steps, 'interval': self.interval_ms}), post=True)
            if (status['mode'] != 'move' or not status['running'] or
                not 0 < status['remaining'] <= self.steps or status.get('direction') != 1 or
                status.get('intervalMs') != self.interval_ms):
                raise RuntimeError('Move was not acknowledged as the expected forward movement')
            previous = status['remaining']
            while True:
                if cancelled():
                    raise RuntimeError('Stopped by user')
                if time.monotonic() > deadline:
                    raise TimeoutError('Timed out waiting for turntable')
                status = self.request('/api/status')
                if status['mode'] == 'idle' and not status['running'] and status['remaining'] == 0:
                    break
                if (status['mode'] != 'move' or not status['running'] or status.get('direction') != 1 or
                    status.get('intervalMs') != self.interval_ms or not 0 < status['remaining'] <= previous):
                    raise RuntimeError('Turntable status changed unexpectedly')
                previous = status['remaining']
                time.sleep(0.1)
            idle = time.monotonic()
            if idle - commanded < (self.steps - 1) * self.interval_ms / 1000:
                raise RuntimeError('Turntable reported idle before the commanded steps could finish')
            until = idle + self.settle_seconds
            while time.monotonic() < until:
                if cancelled():
                    raise RuntimeError('Stopped by user')
                time.sleep(min(0.05, max(0, until-time.monotonic())))
            self.check_idle()
            return {'controller_motion_seconds': idle - commanded,
                    'settle_wait_seconds': time.monotonic() - idle}
        except Exception:
            self.stop()
            raise
