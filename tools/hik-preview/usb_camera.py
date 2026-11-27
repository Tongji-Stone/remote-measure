"""DirectShow discovery and preview for the FHD C3 USB camera."""

import ctypes as ct
import math
import time
from contextlib import contextmanager
from ctypes import wintypes as wt
from uuid import UUID

import cv2
from camera_metrics import exposure_score, roi_brightness


NAME = 'FHD C3 Camera'
DEVICE_ID = 'vid_32e6&pid_9251&mi_00'


class GUID(ct.Structure):
    _fields_ = [('bytes', ct.c_ubyte * 16)]

    @classmethod
    def parse(cls, value):
        return cls.from_buffer_copy(UUID(value).bytes_le)


class Variant(ct.Structure):
    _fields_ = [('type', ct.c_ushort), ('reserved', ct.c_ushort * 3),
                ('data', ct.c_void_p * 2)]


def method(obj, slot, result, *args):
    vtable = ct.cast(obj, ct.POINTER(ct.POINTER(ct.c_void_p))).contents
    return ct.WINFUNCTYPE(result, ct.c_void_p, *args)(vtable[slot])


def release(obj):
    if obj:
        method(obj, 2, ct.c_ulong)(obj)


@contextmanager
def discover():
    """Find the USB video interface and keep its COM apartment alive for capture."""
    ole = ct.WinDLL('ole32')
    oleaut = ct.WinDLL('oleaut32')
    ptr = ct.c_void_p
    ref = ct.byref
    guid_ptr = ct.POINTER(GUID)
    ptr_out = ct.POINTER(ptr)
    enum_guid = GUID.parse('29840822-5b84-11d0-bd3b-00a0c911ce86')
    dev_enum_guid = GUID.parse('62be5d10-60eb-11d0-bd3b-00a0c911ce86')
    video_guid = GUID.parse('860bb310-5d01-11d0-bd3b-00a0c911ce86')
    bag_guid = GUID.parse('55272a00-42cb-11ce-8135-00aa004bb851')
    filter_guid = GUID.parse('56a86895-0ad4-11ce-b03a-0020af0ba770')
    control_guid = GUID.parse('c6e13370-30ac-11d0-a18c-00a0c9118956')
    proc_amp_guid = GUID.parse('c6e13360-30ac-11d0-a18c-00a0c9118956')
    device_enum, devices, moniker = ptr(), ptr(), ptr()
    initialized = ole.CoInitializeEx(None, 2) >= 0
    try:
        if ole.CoCreateInstance(ref(dev_enum_guid), None, 1, ref(enum_guid), ref(device_enum)) < 0:
            raise RuntimeError('无法初始化 DirectShow 视频设备枚举')
        if method(device_enum, 3, ct.c_long, guid_ptr, ptr_out, ct.c_ulong)(
                device_enum, ref(video_guid), ref(devices), 0) != 0:
            raise RuntimeError(f'未找到 {NAME}')
        next_device = method(devices, 3, ct.c_long, ct.c_ulong, ptr_out, ct.c_void_p)
        index = 0
        while next_device(devices, 1, ref(moniker), None) == 0:
            bag = ptr()
            path = Variant()
            try:
                bind_storage = method(moniker, 9, ct.c_long, ptr, ptr, guid_ptr, ptr_out)
                if bind_storage(moniker, None, None, ref(bag_guid), ref(bag)) >= 0:
                    read = method(bag, 3, ct.c_long, wt.LPCWSTR, ct.POINTER(Variant), ptr)
                    if read(bag, 'DevicePath', ref(path), None) >= 0 and path.type == 8:
                        if DEVICE_ID in ct.wstring_at(path.data[0]).lower():
                            with (camera_control(moniker, filter_guid, control_guid) as control,
                                  camera_control(moniker, filter_guid, proc_amp_guid) as proc_amp):
                                yield index, control, proc_amp
                            return
            finally:
                oleaut.VariantClear(ref(path))
                release(bag)
                release(moniker)
                moniker = ptr()
            index += 1
        raise RuntimeError(f'未找到 {NAME}')
    finally:
        release(devices)
        release(device_enum)
        if initialized:
            ole.CoUninitialize()


@contextmanager
def camera_control(moniker, filter_guid, control_guid):
    ptr = ct.c_void_p
    video_filter, control = ptr(), ptr()
    ref = ct.byref
    guid_ptr, ptr_out = ct.POINTER(GUID), ct.POINTER(ptr)
    try:
        bind = method(moniker, 8, ct.c_long, ptr, ptr, guid_ptr, ptr_out)
        if bind(moniker, None, None, ref(filter_guid), ref(video_filter)) >= 0:
            query = method(video_filter, 0, ct.c_long, guid_ptr, ptr_out)
            query(video_filter, ref(control_guid), ref(control))
        yield control
    finally:
        release(control)
        release(video_filter)


EXPOSURE = 4  # IAMCameraControl exposure values are log2(seconds).
WHITE_BALANCE = 7  # IAMVideoProcAmp properties.
GAIN = 9
MANUAL = 2
LONG_OUT = ct.POINTER(ct.c_long)


def control_values(control, property_id):
    if control:
        values = [ct.c_long() for _ in range(5)]
        get_range = method(control, 3, ct.c_long, ct.c_long, *([LONG_OUT] * 5))
        if get_range(control, property_id, *(ct.byref(value) for value in values)) >= 0:
            return tuple(value.value for value in values)
    return None


def exposure_levels(control):
    values = control_values(control, EXPOSURE)
    if values is not None:
        lower, upper, step, _, flags = values
        if flags & MANUAL and step > 0:
            return range(lower, upper + 1, step)
    return range(0)


def read_control(control, property_id):
    value, mode = ct.c_long(), ct.c_long()
    get = method(control, 5, ct.c_long, ct.c_long, LONG_OUT, LONG_OUT)
    if get(control, property_id, ct.byref(value), ct.byref(mode)) < 0:
        raise RuntimeError(f'无法读取 USB 相机控制项 {property_id}')
    return value.value, mode.value


def set_control(control, property_id, value, mode=MANUAL):
    set_value = method(control, 4, ct.c_long, ct.c_long, ct.c_long, ct.c_long)
    if set_value(control, property_id, value, mode) < 0:
        raise RuntimeError(f'USB 相机拒绝控制项 {property_id} 的设置 {value}')


def read_exposure(control):
    return read_control(control, EXPOSURE)


def set_exposure(control, value, mode=MANUAL):
    set_control(control, EXPOSURE, value, mode)


def lock_texture_settings(control, proc_amp, levels, restore, requested_us=None):
    restore.callback(set_exposure, control, *read_exposure(control))
    actual_us = set_manual_exposure(control, levels, 977 if requested_us is None else requested_us)
    settings = {'exposure_us': actual_us}
    for property_id, key, name in ((GAIN, 'gain', '增益'), (WHITE_BALANCE, 'white_balance_k', '白平衡')):
        values = control_values(proc_amp, property_id)
        if values is None or not values[-1] & MANUAL:
            raise RuntimeError(f'USB 相机无法锁定{name}')
        value = values[0] if property_id == GAIN else values[3]
        original = read_control(proc_amp, property_id)
        restore.callback(set_control, proc_amp, property_id, *original)
        set_control(proc_amp, property_id, value)
        actual, flags = read_control(proc_amp, property_id)
        if flags != MANUAL or actual != value:
            raise RuntimeError(f'USB 相机未确认锁定{name}')
        settings[key] = actual
    return settings


def exposure_us(level):
    return 1e6 * 2 ** level


def set_manual_exposure(control, levels, requested_us):
    if not levels:
        raise RuntimeError('USB 相机未提供手动档位')
    requested_level = math.log2(requested_us) - math.log2(1e6)
    level = min(levels, key=lambda value: abs(value - requested_level))
    original = read_exposure(control)
    try:
        set_exposure(control, level)
        if read_exposure(control) != (level, MANUAL):
            raise RuntimeError('USB 相机未确认手动曝光锁定')
    except Exception:
        set_exposure(control, *original)
        raise
    return exposure_us(level)


def settled_frame(capture, stop):
    # This camera continues to deliver frames from the previous shutter setting
    # for about a second at 1080p. Keep consuming them during the transition.
    deadline = time.monotonic() + 1.3
    frame = read_frame(capture)
    while time.monotonic() < deadline:
        if stop.is_set():
            raise InterruptedError('用户停止区域测光')
        frame = read_frame(capture)
    return frame


def projected_frame(capture, stop):
    """Read after the projector hold, discarding the first buffered USB frame."""
    if stop.is_set():
        raise InterruptedError('用户停止采集')
    read_frame(capture)
    if stop.is_set():
        raise InterruptedError('用户停止采集')
    return read_frame(capture)


def meter(capture, control, levels, roi, stop, compensation_ev=0.0):
    """Search supported shutter steps and retain the measured best setting."""
    if not math.isfinite(compensation_ev) or not -2 <= compensation_ev <= 2:
        raise ValueError('曝光补偿应在 -2 到 +2 EV 之间')
    original, mode = read_exposure(control)
    low, high = 0, len(levels) - 1
    index = min(range(len(levels)), key=lambda i: abs(levels[i] - original))
    candidates = []
    status = '已锁定最接近目标的曝光档位'
    try:
        while low <= high:
            if stop.is_set():
                raise InterruptedError('用户停止区域测光')
            value = levels[index]
            set_exposure(control, value)
            frame = settled_frame(capture, stop)
            p90, clipped = roi_brightness(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), roi)
            candidates.append((exposure_score(p90, clipped), value, p90, clipped))
            if 165 <= p90 <= 210 and clipped <= 0.05:
                status = '已达到目标'
                break
            if p90 > 210 or clipped > 0.05:
                high = index - 1
            else:
                low = index + 1
            index = (low + high) // 2
        if status != '已达到目标':
            _, best_value, p90, clipped = min(candidates)
            if best_value != value:
                set_exposure(control, best_value)
                frame = settled_frame(capture, stop)
                p90, clipped = roi_brightness(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), roi)
            value = best_value
        if compensation_ev:
            target = value + compensation_ev
            compensated = min(levels, key=lambda level: abs(level - target))
            if compensated != value:
                set_exposure(control, compensated)
                frame = settled_frame(capture, stop)
                p90, clipped = roi_brightness(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), roi)
            applied = compensated - value
            value = compensated
            status = f'请求 {compensation_ev:+g} EV，实际档位 {applied:+g} EV'
            if clipped > .05:status += '；警告：选区过亮像素超过 5%'
        if read_exposure(control) != (value, MANUAL):
            raise RuntimeError('USB 相机未确认手动曝光锁定')
        return {'exposure_us': exposure_us(value), 'p90': p90, 'clipped': clipped, 'status': status, 'compensation_ev': compensation_ev}
    except Exception:
        set_exposure(control, original, mode)
        raise


def open_camera(index):
    capture = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    try:
        if not capture.isOpened():
            raise RuntimeError(f'无法打开 {NAME}，请检查 USB 连接及相机占用')
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        return capture, read_frame(capture)
    except Exception:
        capture.release()
        raise


def read_frame(capture):
    ok, frame = capture.read()
    if not ok or frame is None:
        raise RuntimeError(f'{NAME} 未返回画面')
    return frame
