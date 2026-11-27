"""Structured light experiment control panel for acquisition and reconstruction."""
import os
import ctypes
import json

# Must be set before Tk creates any window; Windows otherwise bitmap-scales it.
if os.name == 'nt':
    try:
        ctypes.windll.shcore.SetProcessDpiAwareness(1)  # system DPI aware
    except (AttributeError, OSError):
        try:
            ctypes.windll.user32.SetProcessDPIAware()
        except (AttributeError, OSError):
            pass
import math
import queue
import subprocess
import sys
import threading
import tkinter as tk
from concurrent.futures import Future
from contextlib import ExitStack
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from camera_preview import CameraPreview

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = Path(__file__).resolve().parent
DATA = ROOT / 'data'
DEFAULT_CAL = DATA / 'procam_calibration' / '20261120_analysis' / 'all11' / 'calibration_20261116_unit_provisional.xml'
DEFAULT_AXIS = DATA / 'procam_calibration' / '20261120_analysis' / 'axis_approx.json'
HIK_EXPOSURE_MAX_US = 200_000


def put_latest(frames, frame):
    try:
        frames.get_nowait()
    except queue.Empty:
        pass
    frames.put_nowait(frame)


class EmbeddedCaptureUI:
    """Thread-safe bridge between the acquisition worker and the Tk event loop."""
    def __init__(self, app): self.app=app
    def cancelled(self): return self.app.capture_stop_event.is_set()
    def poll_key(self):
        try: return self.app.capture_commands.get_nowait()
        except queue.Empty: return -1
    def show_frame(self, frame):
        put_latest(self.app.preview_frames, frame)
    def set_capturing(self,busy): self.app.preview_events.put(('capture_busy',busy))
    def pattern_progress(self, completed, total):
        self.app.preview_events.put(('pattern_progress', (completed, total)))
    def calibration_session(self, folder):
        self.app.preview_events.put(('calibration_session', folder))
    def auto_exposure_result(self,value): self.app.preview_events.put(('auto_result',value))
    def roi_exposure_result(self,result): self.app.preview_events.put(('roi_auto_result',result))
    def manual_exposure_result(self,result): self.app.preview_events.put(('manual_result',('海康',result)))
    def focus_status(self,mode,score,best,ratio,saturation):
        self.app.preview_events.put(('focus_status',(mode,score,best,ratio,saturation)))
    def log(self,*args,**kwargs):
        self.app.preview_events.put(('capture_log',' '.join(str(x) for x in args)))

    def usb_capture_frame(self, purpose, wait_for_camera=False, defer=False):
        import time
        import cv2
        from calibration_capture import CaptureInterrupted

        deadline = time.monotonic() + 4
        while wait_for_camera and not self.app.usb_stream_ready.is_set() and time.monotonic() < deadline:
            if self.cancelled():
                raise CaptureInterrupted('用户停止采集')
            cv2.waitKey(15)
        if not self.app.usb_stream_ready.is_set():
            raise RuntimeError('FHD C3 Camera 未连接或没有画面')
        request = Future()
        self.app.usb_commands.put((purpose, request))
        deadline = time.monotonic() + 8
        def result():
            try:
                while not request.done():
                    if self.cancelled():
                        raise CaptureInterrupted('用户停止采集')
                    if not self.app.usb_stream_ready.is_set():
                        raise RuntimeError('FHD C3 Camera 已停止取流')
                    if time.monotonic() >= deadline:
                        raise TimeoutError('等待 FHD C3 Camera 图像超时')
                    cv2.waitKey(10)
                return request.result()
            finally:
                request.cancel()

        return result if defer else result()

    def texture_frame(self):
        return self.usb_capture_frame('texture')

    def calibration_frame(self, wait_for_camera=False, defer=False):
        return self.usb_capture_frame('calibration', wait_for_camera, defer)


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('结构光实验操作台')
        self.geometry('1200x900')
        self.minsize(900, 700)
        if os.name == 'nt': self.state('zoomed')
        self.log_path = DATA / 'logs' / f"scan_dashboard_{datetime.now():%Y%m%d_%H%M%S_%f}.log"
        self.process = None
        self.preview_thread = None
        self.preview_stop_event = threading.Event()
        self.preview_frames = queue.Queue(maxsize=1)
        self.preview_events = queue.Queue()
        self.preview_next = None
        self.preview_active = False
        self.usb_thread = None
        self.usb_stop = threading.Event()
        self.usb_stream_ready = threading.Event()
        self.usb_frames = queue.Queue(maxsize=1)
        self.usb_commands = queue.Queue()
        self.usb_meter_ready = False
        self.usb_restart = None
        self.close_requested = False
        self.usb_focus_text = tk.StringVar(value='')
        self.usb_focus_history = []
        self.usb_focus_best = 0.0
        self.focus_mode = None
        self.usb_focus_auto = False
        self.focus_tone = None
        self.focus_audio_view = None
        self.focus_audio_text = tk.StringVar(value='')
        self.capture_thread = None
        self.capture_stop_event = threading.Event()
        self.capture_commands = queue.Queue()
        self.capture_kind = None
        self.capture_buttons = {}
        self.preview_commands = queue.Queue()
        self.roi_pending = set()
        self.focus_text = tk.StringVar(value='')
        self.roi_hint = tk.StringVar(value='分别在两台画面左键拖框，蓝框为各自的曝光区域。')
        self.task_has_warnings = False
        self.messages = queue.Queue()
        self.status = tk.StringVar(value='就绪')
        self.calibration = tk.StringVar(value=str(DEFAULT_CAL) if DEFAULT_CAL.exists() else '')
        self.axis = tk.StringVar(value=str(DEFAULT_AXIS) if DEFAULT_AXIS.exists() else '')
        self.exposure = tk.StringVar(value='50000')
        self.exposure_camera = tk.StringVar(value='海康相机')
        self.usb_exposure = tk.StringVar(value='')
        self.usb_manual_exposure = None
        self.usb_exposure_values = ()
        self.manual_pending = set()
        self.gain = tk.StringVar(value='0')
        self.auto_exposure = tk.BooleanVar(value=False)
        self.roi_compensation_ev = tk.StringVar(value='0')
        self.view_count = tk.StringVar(value='8')
        self.marker_size = tk.StringVar(value='20')
        self.board_size = tk.StringVar(value='10')
        self.board_columns = tk.StringVar(value='9')
        self.board_rows = tk.StringVar(value='7')
        self.marker_calibration = tk.StringVar(value='')
        self.calibration_capture_folder = tk.StringVar(value='')
        self.calibration_board = tk.StringVar(value='普通棋盘格')
        self.calibration_columns = tk.StringVar(value='9')
        self.calibration_rows = tk.StringVar(value='7')
        self.calibration_square = tk.StringVar(value='10')
        self.calibration_marker = tk.StringVar(value='7')
        self.esp = tk.StringVar(value='http://192.168.4.1')
        self.settle = tk.StringVar(value='1.0')
        self.loop = tk.BooleanVar(value=False)
        self.manual_step = tk.StringVar(value='45')
        self.view_folder = tk.StringVar(value='')
        self.scan_folder = tk.StringVar(value='')
        self.angles = tk.StringVar(value='')
        self.bits = tk.StringVar(value='0')
        self.step = tk.StringVar(value='4')
        self.radius = tk.StringVar(value='55')
        self.clearance = tk.StringVar(value='3')
        self.no_crop = tk.BooleanVar(value=False)
        self.ply = tk.StringVar(value='')
        self.keep_colors = tk.BooleanVar(value=False)
        self.roi = tk.StringVar(value='720 1010 1370 1590')
        self.depth = tk.StringVar(value='345 390')
        self.fig_label = tk.StringVar(value='')
        self.fig_params = tk.StringVar(value='试验结果；请填写参数')
        self._build()
        self.protocol('WM_DELETE_WINDOW', self.close_window)
        self.after(100, self._poll)

    def _build(self):
        title = ttk.Frame(self, padding=(16, 12, 16, 5)); title.pack(fill='x')
        ttk.Label(title, text='结构光实验操作台', font=('Microsoft YaHei UI', 17, 'bold')).pack(anchor='w')
        ttk.Label(title, text='相机采集 · 物体重建 · 转台拼接 · 点云查看', foreground='#555').pack(anchor='w', pady=(3,0))
        notebook = ttk.Notebook(self); notebook.pack(fill='both', expand=True, padx=16, pady=6)
        capture = ttk.Frame(notebook, padding=14); reconstruction = ttk.Frame(notebook, padding=14)
        merging = ttk.Frame(notebook, padding=14); viewer = ttk.Frame(notebook, padding=14); advanced = ttk.Frame(notebook, padding=14)
        for frame, title in [(capture,'① 采集'),(reconstruction,'② 单视角重建'),(merging,'③ 转台拼接'),(viewer,'④ 查看与绘图'),(advanced,'其他试验')]:
            notebook.add(frame, text=title)
        self._build_capture(capture); self._build_reconstruction(reconstruction); self._build_merge(merging); self._build_viewer(viewer); self._build_advanced(advanced)
        logs = ttk.LabelFrame(self, text='运行日志', padding=8); logs.pack(fill='x', padx=16, pady=(0, 8))
        progress = ttk.Frame(logs)
        progress.pack(fill='x', pady=(0, 5))
        ttk.Label(progress, text='图案采集').pack(side='left', padx=(0, 8))
        self.pattern_bar = ttk.Progressbar(progress, mode='determinate')
        self.pattern_bar.pack(side='left', fill='x', expand=True)
        self.pattern_label = ttk.Label(progress, text='待采集', width=12, anchor='e')
        self.pattern_label.pack(side='right')
        transcript = ttk.Frame(logs)
        transcript.pack(fill='both', expand=True)
        self.log = tk.Text(transcript, height=5, wrap='word', font=('Consolas', 9)); self.log.pack(side='left', fill='both', expand=True)
        bar = ttk.Scrollbar(transcript, command=self.log.yview); bar.pack(side='right', fill='y'); self.log.configure(yscrollcommand=bar.set)
        footer = ttk.Frame(self, padding=(16, 0, 16, 10)); footer.pack(fill='x')
        ttk.Label(footer, textvariable=self.status).pack(side='left')
        ttk.Button(footer, text='清空日志', command=lambda: self.log.delete('1.0','end')).pack(side='right')
        ttk.Button(footer, text='打开日志文件', command=self.open_log_file).pack(side='right', padx=(0, 8))
        self.append(f'本次运行日志文件：{self.log_path}')
        self.append('提示：预览和采集均显示在右侧；再次点击相同的开始按钮可停止。')

    def pathrow(self, parent, label, variable, kind='file', filetypes=None):
        row=ttk.Frame(parent); row.pack(fill='x', pady=5)
        ttk.Label(row, text=label, width=15).pack(side='left')
        ttk.Entry(row, textvariable=variable).pack(side='left', fill='x', expand=True, padx=(0,6))
        def choose():
            initial=str(DATA if DATA.exists() else ROOT)
            value=(filedialog.askdirectory(initialdir=initial) if kind=='dir' else filedialog.askopenfilename(initialdir=initial, filetypes=filetypes or [('所有文件','*.*')]))
            if value: variable.set(value)
        ttk.Button(row,text='选择…',command=choose).pack(side='left')

    def _build_capture(self, f):
        split=ttk.Panedwindow(f,orient='horizontal');split.pack(fill='both',expand=True)
        sidebar=ttk.Frame(split);right=ttk.Frame(split)
        split.add(sidebar,weight=0);split.add(right,weight=1)
        scroll=tk.Canvas(sidebar,width=520,highlightthickness=0)
        scrollbar=ttk.Scrollbar(sidebar,orient='vertical',command=scroll.yview)
        scroll.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side='right',fill='y');scroll.pack(side='left',fill='both',expand=True)
        left=ttk.Frame(scroll,padding=(0,0,10,0))
        left_window=scroll.create_window(0,0,anchor='nw',window=left)
        left.bind('<Configure>',lambda event:scroll.configure(scrollregion=scroll.bbox('all')))
        scroll.bind('<Configure>',lambda event:scroll.itemconfigure(left_window,width=event.width))
        ttk.Label(left,text='相机参数',font=('Microsoft YaHei UI',12,'bold')).pack(anchor='w')
        r=ttk.Frame(left);r.pack(fill='x',pady=4)
        ttk.Label(r,text='手动曝光').pack(side='left')
        selector = ttk.Combobox(r, textvariable=self.exposure_camera, width=21, state='readonly',
                                values=('海康相机', 'FHD C3 Camera（彩色）'))
        selector.pack(side='left', padx=5)
        selector.bind('<<ComboboxSelected>>', lambda event: self._update_exposure_options())
        r=ttk.Frame(left);r.pack(fill='x',pady=4)
        ttk.Label(r,text='曝光（微秒）').pack(side='left')
        self.manual_exposure_box = ttk.Combobox(r, width=10)
        self.manual_exposure_box.pack(side='left', padx=5)
        ttk.Button(r, text='应用曝光', command=lambda: self.checked(self.apply_manual_exposure)).pack(side='left')
        r=ttk.Frame(left);r.pack(fill='x',pady=4)
        ttk.Label(r,text='海康增益 dB').pack(side='left')
        ttk.Combobox(r,textvariable=self.gain,width=5,values=['0','1','2','3','4','6','8','10','12','16','20','24']).pack(side='left',padx=5)
        self._update_exposure_options()
        ttk.Checkbutton(left,text='自动曝光一次后锁定',variable=self.auto_exposure).pack(anchor='w',pady=4)
        self.preview_button=ttk.Button(left,text='启动实时预览',command=self.preview);self.preview_button.pack(fill='x',pady=3)
        r=ttk.Frame(left);r.pack(fill='x',pady=2)
        ttk.Label(r,text='框选测光补偿 EV').pack(side='left')
        ttk.Combobox(r,textvariable=self.roi_compensation_ev,width=7,state='readonly',values=['-1','-0.5','0','0.3','0.5','1','1.5','2']).pack(side='left',padx=5)
        ttk.Label(r,text='正值更亮').pack(side='left')
        self.roi_button=ttk.Button(left,text='按框选区自动曝光',command=self.meter_selected_roi,state='disabled');self.roi_button.pack(fill='x',pady=3)
        ttk.Label(left,textvariable=self.roi_hint,wraplength=420).pack(anchor='w')
        self.capture_buttons['focus']=ttk.Button(left,text='开始手动调焦',command=self.focus_guide)
        self.capture_buttons['focus'].pack(fill='x',pady=3)
        r=ttk.Frame(left);r.pack(fill='x')
        self.focus_mode_button=ttk.Button(r,text='切换白光 / 横竖条纹',command=lambda:self.focus_command(ord('m')),state='disabled')
        self.focus_mode_button.pack(side='left',fill='x',expand=True)
        self.focus_reset_button=ttk.Button(r,text='重置最高分',command=lambda:self.focus_command(ord('b')),state='disabled')
        self.focus_reset_button.pack(side='left',fill='x',expand=True)
        self.focus_label = ttk.Label(left, textvariable=self.focus_text, wraplength=440, justify='left')
        self.focus_label.pack(anchor='w', pady=3)
        self.usb_focus_label = ttk.Label(left, textvariable=self.usb_focus_text, wraplength=440, justify='left')
        self.focus_audio_label = ttk.Label(left, textvariable=self.focus_audio_text, wraplength=440, justify='left')
        ttk.Separator(left).pack(fill='x',pady=8)
        ttk.Label(left,text='采集',font=('Microsoft YaHei UI',12,'bold')).pack(anchor='w')
        self.capture_buttons['calibration']=ttk.Button(left,text='开始标定采集',command=self.cal_capture)
        self.capture_buttons['calibration'].pack(fill='x',pady=3)
        self.capture_buttons['manual']=ttk.Button(left,text='开始手动转台采集',command=self.manual_capture)
        self.capture_buttons['manual'].pack(fill='x',pady=3)
        r=ttk.Frame(left);r.pack(fill='x',pady=3)
        ttk.Label(r,text='手动每步名义角度').pack(side='left')
        ttk.Entry(r,textvariable=self.manual_step,width=7).pack(side='left',padx=7)
        self.capture_one_button=ttk.Button(left,text='采集当前视角',command=self.capture_one,state='disabled');self.capture_one_button.pack(fill='x',pady=4)
        ttk.Label(left,text='启动标定或手动采集后，摆稳并点击“采集当前视角”；再次点击开始按钮可停止。',wraplength=320,justify='left').pack(anchor='w',pady=3)
        box=ttk.LabelFrame(left,text='ESP32 自动旋转采集',padding=8);box.pack(fill='x',pady=8)
        r=ttk.Frame(box);r.pack(fill='x',pady=3)
        ttk.Label(r,text='控制地址').pack(side='left');ttk.Entry(r,textvariable=self.esp,width=22).pack(side='left',padx=4)
        r=ttk.Frame(box);r.pack(fill='x',pady=3)
        ttk.Label(r,text='采集组数').pack(side='left');ttk.Entry(r,textvariable=self.view_count,width=5).pack(side='left',padx=4)
        ttk.Label(r,text='稳定等待秒').pack(side='left');ttk.Entry(r,textvariable=self.settle,width=5).pack(side='left',padx=4)
        ttk.Checkbutton(box,text='额外采集 360° 回环',variable=self.loop).pack(anchor='w',pady=3)
        self.capture_buttons['automatic']=ttk.Button(box,text='开始自动旋转采集',command=self.auto_capture)
        self.capture_buttons['automatic'].pack(fill='x',pady=3)
        ttk.Label(box,text='再次点击可停止；每圈按 4096 半步计算，间隔 3 ms。',wraplength=300,justify='left').pack(anchor='w')
        ttk.Label(right,text='相机实时预览',font=('Microsoft YaHei UI',11,'bold')).pack(anchor='w',pady=(0,5))
        previews = ttk.Frame(right)
        previews.pack(fill='both', expand=True)
        previews.rowconfigure(0, weight=1)
        self.hik_view = CameraPreview(previews, '海康相机', '启动实时预览或采集')
        self.usb_view = CameraPreview(previews, 'FHD C3 Camera', '启动实时预览或采集')
        self.preview_ratios = {self.hik_view: 2448 / 2048, self.usb_view: 16 / 9}
        for column, view in enumerate(self.preview_ratios):
            view.grid(row=0, column=column, sticky='nsew', padx=(0, 8) if column == 0 else 0)
            previews.columnconfigure(column, weight=round(1000 * self.preview_ratios[view]), uniform='cameras')
            view.canvas.bind('<Configure>', lambda event, pane=view: self._render_preview(pane))
            for button in (1, 3):
                view.canvas.bind(f'<ButtonPress-{button}>', lambda event, pane=view, key=button: self._roi_start(pane, event, key))
                view.canvas.bind(f'<B{button}-Motion>', lambda event, pane=view, key=button: self._roi_move(pane, event, key))
                view.canvas.bind(f'<ButtonRelease-{button}>', lambda event, pane=view, key=button: self._roi_end(pane, event, key))
        ttk.Label(right,text='结构光采集使用海康相机，投影仪显示结构光图案。',wraplength=800).pack(anchor='w',pady=5)

    def _show_frame(self, view, frame):
        ratio = view.set_frame(frame)
        if ratio != self.preview_ratios[view]:
            self.preview_ratios[view] = ratio
            column = 0 if view is self.hik_view else 1
            view.master.columnconfigure(column, weight=round(1000 * ratio))
        self._render_preview(view)

    def _render_preview(self, view):
        view.render(focusing=self.capture_kind == 'focus')

    def _roi_kind(self, button):
        if self.capture_kind == 'focus' and not self.capture_stop_event.is_set():
            return 'focus' if button == 1 else 'exposure'
        if self.preview_active and button == 1:
            return 'exposure'
        return None

    def _roi_start(self, view, event, button=1):
        kind = self._roi_kind(button)
        if kind is None or view.drag_kind is not None:
            return
        view.drag_start = view.point(event.x, event.y)
        if view.drag_start is not None:
            view.drag_kind = kind
            if kind == 'focus':
                self._select_focus_audio(view)

    def _roi_move(self, view, event, button=1):
        if view.drag_kind is not None and view.drag_kind == self._roi_kind(button):
            view.draw_box(view.drag_kind, view.drag_box(event.x, event.y))

    def _roi_end(self, view, event, button=1):
        kind = self._roi_kind(button)
        if kind is None or view.drag_kind != kind:
            return
        candidate = view.drag_box(event.x, event.y)
        view.drag_start = None
        view.drag_kind = None
        if candidate is None:
            return
        x0, y0, x1, y1 = candidate
        if x1 - x0 > .01 and y1 - y0 > .01:
            if kind == 'exposure':
                view.exposure_roi = candidate
                self._render_preview(view)
                return
            if view.frame is not None:
                h, w = view.frame.shape[:2]
                scale = min(1., 1100 / w, 750 / h)
                if (x1 - x0) * w * scale <= 11 or (y1 - y0) * h * scale <= 11:
                    self.append('选区太小，请选大一些的区域。')
                    self._render_preview(view)
                    return
            view.focus_roi = candidate
            view.exposure_roi = candidate
            view.set_focus_zoom(True)
            if view is self.hik_view:
                self.capture_commands.put(('roi', *candidate))
            else:
                self._reset_usb_focus()
        self._render_preview(view)

    def _meter_targets(self):
        targets = {}
        focus = self.capture_kind == 'focus' and not self.capture_stop_event.is_set()
        if self.preview_active or focus:
            if self.hik_view.frame is not None:
                targets['海康'] = self.capture_commands if focus else self.preview_commands
            if self.usb_meter_ready and not self.usb_stop.is_set() and self.usb_view.frame is not None:
                targets['USB'] = self.usb_commands
        return targets

    def _update_exposure_options(self):
        usb = self.exposure_camera.get() == 'FHD C3 Camera（彩色）'
        self.manual_exposure_box.configure(
            textvariable=self.usb_exposure if usb else self.exposure,
            values=self.usb_exposure_values if usb else
            ('500', '1000', '2000', '3000', '5000', '7500', '10000', '15000', '20000',
             '25000', '30000', '40000', '50000', '100000', str(HIK_EXPOSURE_MAX_US)),
        )

    def _manual_exposure_us(self, source):
        entry = self.exposure if source == '海康' else self.usb_exposure
        try:
            exposure = float(entry.get())
        except ValueError:
            raise ValueError('请输入有效的曝光时间（微秒）') from None
        if not math.isfinite(exposure) or exposure <= 0:
            raise ValueError('曝光时间必须大于 0')
        if source == '海康' and exposure > HIK_EXPOSURE_MAX_US:
            raise ValueError(f'海康曝光不能超过 {HIK_EXPOSURE_MAX_US} 微秒（200 ms）')
        return exposure

    def apply_manual_exposure(self):
        source = '海康' if self.exposure_camera.get() == '海康相机' else 'USB'
        exposure = self._manual_exposure_us(source)
        if self.roi_pending or source in self.manual_pending:
            raise ValueError('请等当前曝光调整完成')
        commands = self._meter_targets().get(source)
        if commands is not None:
            self.manual_pending.add(source)
            commands.put(('manual', exposure))
        else:
            if source == 'USB':
                if (self.preview_active or self.capture_kind == 'focus') and self.usb_thread is not None and self.usb_view.frame is None:
                    raise ValueError('请等待 USB 相机画面出现后再应用曝光')
                if self.usb_view.frame is not None and not self.usb_meter_ready:
                    raise ValueError('当前 USB 相机未提供手动曝光控制')
                self.usb_manual_exposure = exposure
            else:
                self.auto_exposure.set(False)
            self.append(f'{source} 手动曝光已设为 {exposure:g} 微秒，下次启动时应用。')

    def meter_selected_roi(self):
        targets = self._meter_targets()
        if self.roi_pending or self.manual_pending or not targets:
            return
        regions = {'海康': self.hik_view.exposure_roi, 'USB': self.usb_view.exposure_roi}
        selected = {source: commands for source, commands in targets.items() if regions[source] is not None}
        if not selected:
            if any(region is not None for region in regions.values()):
                messagebox.showinfo('相机暂不能测光', '已框选的相机当前没有可用的曝光控制。')
            else:
                messagebox.showinfo('请先框选', '请先在右侧画面上拖框选择测光区域。')
            return
        try:compensation=float(self.roi_compensation_ev.get())
        except ValueError:
            messagebox.showerror('参数错误','请输入有效曝光补偿');return
        if not math.isfinite(compensation) or not -2<=compensation<=2:
            messagebox.showerror('参数错误','曝光补偿应在 -2 到 +2 EV 之间');return
        self.roi_pending = set(selected)
        self.roi_button.state(['disabled'])
        self.status.set('正在按框选区域自动曝光…')
        for source, commands in selected.items():
            commands.put(('auto_roi', (regions[source],compensation)))

    def focus_command(self,key):
        if self.capture_kind == 'focus':
            if key == ord('m') and (self.roi_pending or self.manual_pending):
                return
            if key == ord('b'):
                self._reset_usb_focus()
            self.capture_commands.put(key)

    def _reset_usb_focus(self):
        self.usb_focus_history.clear()
        self.usb_focus_best = 0.0

    def _select_focus_audio(self, view):
        self.focus_audio_view = view
        self.focus_audio_text.set(f'提示音：{view.name}（点击画面切换）')

    def _update_usb_focus(self, frame):
        import cv2
        from statistics import median
        from camera_metrics import focus_score

        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        height, width = gray.shape
        scale = min(1., 1100 / width, 750 / height)
        gray = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        height, width = gray.shape
        x0, y0, x1, y1 = self.usb_view.focus_roi or (.2, .2, .8, .8)
        score, saturation = focus_score(gray[int(y0 * height):int(y1 * height), int(x0 * width):int(x1 * width)])
        self.usb_focus_history = (self.usb_focus_history + [score])[-5:]
        score = median(self.usb_focus_history)
        self.usb_focus_best = max(score, self.usb_focus_best)
        ratio = score / self.usb_focus_best if self.usb_focus_best else 0.
        self.usb_focus_text.set(f'USB | 清晰度 {score:.1f} / 最高 {self.usb_focus_best:.1f} | 相对 {ratio:.0%} | 极暗或过亮 {saturation:.0%}')
        if self.focus_audio_view is self.usb_view:
            self.focus_tone.update(score, ratio, saturation)

    def _build_reconstruction(self,f):
        self.pathrow(f,'采集文件夹',self.view_folder,'dir')
        self.pathrow(f,'标定参数 XML',self.calibration,filetypes=[('标定 XML','*.xml'),('所有文件','*.*')])
        r=ttk.Frame(f);r.pack(fill='x',pady=12)
        ttk.Label(r,text='忽略最细编码位（每方向）').pack(side='left');ttk.Combobox(r,textvariable=self.bits,values=['0','1','2'],state='readonly',width=5).pack(side='left',padx=8)
        ttk.Label(r,text='相机采样间隔（像素）').pack(side='left');ttk.Combobox(r,textvariable=self.step,values=['1','2','4','8'],width=6).pack(side='left',padx=8)
        ttk.Button(f,text='生成单视角点云',command=self.reconstruct).pack(anchor='w',pady=8)
        ttk.Label(f,text='选择包含 44 张 graycode 图片的文件夹。新结果使用时间命名，不覆盖旧点云。',wraplength=900).pack(anchor='w')
        box = ttk.LabelFrame(f, text='相机与投影仪标定', padding=8)
        box.pack(fill='x', pady=10)
        self.pathrow(box,'标定采集文件夹',self.calibration_capture_folder,'dir')
        options = ttk.Frame(box)
        options.pack(fill='x', pady=6)
        board = ttk.Combobox(options, textvariable=self.calibration_board,
                             values=['普通棋盘格','ChArUco'], state='readonly', width=12)
        board.pack(side='left', padx=(0,8))
        board.bind('<<ComboboxSelected>>', self._calibration_board_changed)
        for label, variable in [('列', self.calibration_columns), ('行', self.calibration_rows),
                                ('方格 mm', self.calibration_square), ('编码 mm', self.calibration_marker)]:
            ttk.Label(options, text=label).pack(side='left')
            ttk.Entry(options, textvariable=variable, width=5).pack(side='left', padx=(3,10))
        ttk.Label(box,text='普通棋盘格填写内角点数。ChArUco 填写方格数和编码黑框边长，字典为 DICT_5X5_50。').pack(anchor='w')
        ttk.Button(box,text='求内外参',command=self.calibrate).pack(anchor='w',pady=6)

    def _build_merge(self,f):
        ttk.Label(f,text='二维码标记自动重建与配准',font=('Microsoft YaHei UI',12,'bold')).pack(anchor='w',pady=5)
        self.pathrow(f,'扫描会话文件夹',self.scan_folder,'dir')
        self.pathrow(f,'标定 XML 文件',self.marker_calibration,filetypes=[('标定 XML','*.xml'),('所有文件','*.*')])
        r=ttk.Frame(f);r.pack(fill='x',pady=5)
        ttk.Label(r,text='或选择标定采集/结果文件夹').pack(side='left')
        ttk.Button(r,text='选择标定文件夹…',command=lambda:self.choose_cal_dir()).pack(side='left',padx=8)
        r=ttk.Frame(f);r.pack(fill='x',pady=7)
        ttk.Label(r,text='棋盘每格 mm').pack(side='left');ttk.Entry(r,textvariable=self.board_size,width=5).pack(side='left',padx=5)
        ttk.Label(r,text='内角点 列×行').pack(side='left');ttk.Entry(r,textvariable=self.board_columns,width=3).pack(side='left');ttk.Label(r,text='×').pack(side='left');ttk.Entry(r,textvariable=self.board_rows,width=3).pack(side='left',padx=5)
        ttk.Label(r,text='标记黑框 mm').pack(side='left');ttk.Combobox(r,textvariable=self.marker_size,values=['20','15'],width=6).pack(side='left',padx=6)
        ttk.Label(r,text='忽略最细位').pack(side='left');ttk.Combobox(r,textvariable=self.bits,values=['0','1','2'],state='readonly',width=4).pack(side='left',padx=6)
        ttk.Label(r,text='采样间隔').pack(side='left');ttk.Entry(r,textvariable=self.step,width=5).pack(side='left',padx=6)
        ttk.Button(f,text='自动重建并拼接',command=self.marker_batch).pack(anchor='w',pady=8)
        ttk.Label(f,text='先选当前光学布局对应的标定：XML 或标定采集文件夹。ArUco DICT_4X4_50；各视角与首帧至少共享两个标记。输出保留转盘和背景，结果需验证。',wraplength=900).pack(anchor='w')
        ttk.Button(f,text='ChArUco 板转台拼接',command=self.charuco_batch).pack(anchor='w',pady=(8,2))
        ttk.Label(f,text='使用 XML、10×10 方格、10/7 mm、DICT_5X5_50，输出各帧和合并点云。').pack(anchor='w')
        box=ttk.LabelFrame(f,text='旧版：使用已知转轴试验拼接',padding=8);box.pack(fill='x',pady=10)
        self.pathrow(box,'转轴参数 JSON',self.axis,filetypes=[('JSON','*.json'),('所有文件','*.*')])
        self.pathrow(box,'角度修正 JSON（可空）',self.angles,filetypes=[('JSON','*.json'),('所有文件','*.*')])
        r=ttk.Frame(box);r.pack(fill='x',pady=4)
        ttk.Label(r,text='裁剪半径 mm').pack(side='left');ttk.Entry(r,textvariable=self.radius,width=6).pack(side='left',padx=6)
        ttk.Label(r,text='离盘面 mm').pack(side='left');ttk.Entry(r,textvariable=self.clearance,width=6).pack(side='left',padx=6)
        ttk.Checkbutton(r,text='保留转盘和背景',variable=self.no_crop).pack(side='left',padx=8)
        ttk.Button(r,text='使用转轴拼接',command=self.batch).pack(side='left',padx=6)

    def choose_cal_dir(self):
        value=filedialog.askdirectory(initialdir=str(DATA))
        if value:self.marker_calibration.set(value)

    def _build_viewer(self,f):
        self.pathrow(f,'PLY 文件',self.ply,filetypes=[('PLY 点云','*.ply'),('所有文件','*.*')])
        r=ttk.Frame(f);r.pack(fill='x',pady=8)
        ttk.Button(r,text='打开点云',command=self.view).pack(side='left',padx=(0,10))
        ttk.Checkbutton(r,text='保留文件颜色（不勾选则按深度上色）',variable=self.keep_colors).pack(side='left')
        self.pathrow(f,'标定参数 XML',self.calibration,filetypes=[('标定 XML','*.xml'),('所有文件','*.*')])
        r=ttk.Frame(f);r.pack(fill='x',pady=7);ttk.Label(r,text='物体名称').pack(side='left');ttk.Entry(r,textvariable=self.fig_label,width=18).pack(side='left',padx=8);ttk.Label(r,text='图中参数说明').pack(side='left');ttk.Entry(r,textvariable=self.fig_params).pack(side='left',fill='x',expand=True,padx=8)
        ttk.Button(f,text='生成带相机位置的图片',command=self.draw_scene).pack(anchor='w',pady=8)
        ttk.Button(f,text='打开扫描数据文件夹',command=lambda: os.startfile(str(DATA / 'object_scans'))).pack(anchor='w')
        ttk.Label(f,text='点云可在新窗口旋转、缩放。生成相机位置图时会使用所选标定文件。',wraplength=900).pack(anchor='w',pady=8)

    def _build_advanced(self,f):
        ttk.Label(f,text='以下为针对既有数据开发的专用试验，不是通用自动标定工具。',wraplength=900).pack(anchor='w',pady=6)
        r=ttk.Frame(f);r.pack(fill='x',pady=8)
        ttk.Button(r,text='11 月 16 日标定留出验证',command=lambda: self.run('标定留出验证','validate_calibration_split.py',[])).pack(side='left',padx=(0,8))
        ttk.Button(r,text='旧长方体固定轴拟合',command=lambda: self.run('旧固定轴拟合','尝试固定转轴拼接.py',[])).pack(side='left')
        self.pathrow(f,'回环扫描文件夹',self.scan_folder,'dir')
        r=ttk.Frame(f);r.pack(fill='x',pady=5);ttk.Label(r,text='ROI x1 y1 x2 y2').pack(side='left');ttk.Entry(r,textvariable=self.roi,width=25).pack(side='left',padx=6);ttk.Label(r,text='深度下限 上限 mm').pack(side='left');ttk.Entry(r,textvariable=self.depth,width=15).pack(side='left',padx=6)
        ttk.Button(f,text='比较旧长方体 0000 / 0004',command=self.compare).pack(anchor='w',pady=8)
        ttk.Label(f,text='该比较脚本依赖 view_0000 / view_0004 内的 box_view_10mm.ply，并会覆盖既有比较结果。',wraplength=900).pack(anchor='w')

    def append(self,text):
        self.log.insert('end', text + '\n')
        if self.log_path is not None:
            try:
                self.log_path.parent.mkdir(parents=True, exist_ok=True)
                with self.log_path.open('a', encoding='utf-8') as stream:
                    stream.write(f'[{datetime.now():%Y-%m-%d %H:%M:%S.%f}] {text}\n')
            except OSError as exc:
                self.log.insert('end', f'日志文件无法写入：{exc}\n')
                self.log_path = None
        self.log.see('end')

    def open_log_file(self):
        if self.log_path is None:
            messagebox.showerror('日志文件不可用', '当前会话的运行日志未能写入文件。')
            return
        try:
            os.startfile(str(self.log_path))
        except OSError as exc:
            messagebox.showerror('打开日志失败', str(exc))

    def run(self,title,script,args,on_success=None):
        if self.capture_thread is not None:
            messagebox.showinfo('正在采集','请先点击当前采集按钮停止。');return
        if self.preview_next is not None:
            messagebox.showinfo('正在释放相机','请稍候。');return
        if self.preview_active or self.preview_thread is not None:
            self.preview_next = lambda: self.run(title,script,args,on_success)
            self.stop_preview()
            self._stop_usb()
            return
        if self.process is not None:
            messagebox.showinfo('正在运行','请先结束当前任务。相机预览或采集请在其窗口按 Q 退出。');return
        cmd=[sys.executable,'-u',str(SCRIPTS/script),*map(str,args)]
        try:
            self.process=subprocess.Popen(cmd,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8',errors='replace',env={**os.environ,'PYTHONUTF8':'1','PYTHONUNBUFFERED':'1'},creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        except OSError as exc:
            messagebox.showerror('启动失败',str(exc));return
        process=self.process;self.task_has_warnings=False;self.status.set('运行中：'+title);self.append('▶ '+title)
        def read():
            for line in process.stdout: self.messages.put(('line',line.rstrip()))
            code=process.wait();self.messages.put(('done',(title,code,on_success)))
        threading.Thread(target=read,daemon=True).start()

    def close_window(self):
        if self.capture_thread is not None:
            self.capture_stop_event.set()
            self.status.set('正在停止采集，完成后请再次关闭窗口')
            return
        if self.process is not None:
            messagebox.showinfo('任务正在运行', '请先在预览或采集窗口按 Q 退出；点云查看窗口请直接关闭。任务结束后再关闭操作台。')
            return
        self.close_requested = True
        self.preview_next = None
        self.stop_preview()
        if self.preview_thread is None and self.usb_thread is None:
            self.destroy()

    def _report_roi(self, source, payload, startup=False):
        if not startup:
            self.roi_pending.discard(source)
        if isinstance(payload, dict):
            if source == '海康':
                self.exposure.set(str(round(payload['exposure_us'])))
                self.auto_exposure.set(False)
            else:
                self.usb_exposure.set(str(round(payload['exposure_us'])))
                self._reset_usb_focus()
            exposure_label = '当前曝光' if payload.get('no_usable_signal') else '曝光已锁定为'
            self.append(f"{source} {exposure_label} {payload['exposure_us']:.0f} 微秒，"
                        f"P90={payload['p90']:.0f}，过亮比例={payload['clipped']:.1%}。{payload['status']}。")
        else:
            self.append(f'{source} 测光失败：{payload}')
        if not startup and not self.roi_pending:
            self.status.set('区域测光结束，请查看运行日志')

    def _report_manual_exposure(self, source, result):
        self.manual_pending.discard(source)
        if isinstance(result, str):
            self.append(f'{source} 手动曝光设置失败：{result}')
            return
        if source == 'USB':
            self.usb_manual_exposure = result
            self.usb_exposure.set(str(round(result)))
            self._reset_usb_focus()
        else:
            self.exposure.set(f'{result:g}')
            self.auto_exposure.set(False)
        self.append(f'{source} 手动曝光已应用：{result:.0f} 微秒。')

    def _poll(self):
        try:
            while True:
                event,payload=self.preview_events.get_nowait()
                if event == 'info':
                    self.append(payload)
                    self.status.set(payload)
                elif event == 'auto_result':
                    self.exposure.set(str(round(payload)))
                    self.append(f'自动曝光锁定：{payload:.0f} 微秒。请检查主体亮度及高光是否过曝。')
                    if payload >= 199000:
                        self.append('警告：已接近自动曝光上限 200000 微秒，画面仍可能偏暗；请增加照明或手动调整。')
                elif event == 'roi_auto_result':
                    self._report_roi('海康', payload)
                elif event == 'usb_roi_result':
                    result, startup = payload
                    self._report_roi('USB', result, startup)
                elif event == 'manual_result':
                    self._report_manual_exposure(*payload)
                elif event == 'usb_ready':
                    self.usb_meter_ready = bool(payload)
                    self.usb_exposure_values = payload
                    self._update_exposure_options()
                    self.append('FHD C3 Camera 实时预览中')
                    if not payload:
                        self.append('FHD C3 Camera 未提供手动曝光控制，区域测光不可用。')
                elif event == 'usb_done':
                    self.usb_thread = None
                    self.usb_meter_ready = False
                    self.roi_pending.discard('USB')
                    self.manual_pending.discard('USB')
                    if payload:
                        self.append(f'FHD C3 Camera 预览失败：{payload}')
                    self.usb_view.show_message(payload or 'USB 预览已停止')
                    self.usb_focus_text.set('USB | 无可用画面')
                    if self.capture_kind == 'focus' and self.focus_audio_view is self.usb_view:
                        self._select_focus_audio(self.hik_view)
                    restart, self.usb_restart = self.usb_restart, None
                    if restart is not None:
                        self._start_usb(restart)
                elif event == 'focus_status':
                    mode,score,best,ratio,saturation=payload
                    names=['白光：调整相机镜头','竖条纹：调整投影仪','横条纹：调整投影仪']
                    if mode != self.focus_mode:
                        self.focus_mode = mode
                        self._reset_usb_focus()
                    self.focus_text.set(f'{names[mode]}\n海康 | 清晰度 {score:.1f} / 最高 {best:.1f} | 相对 {ratio:.0%} | 极暗或过亮 {saturation:.0%}')
                    if (self.focus_audio_view is self.hik_view and not self.roi_pending
                            and not self.capture_stop_event.is_set()):
                        self.focus_tone.update(score, ratio, saturation)
                elif event == 'capture_busy':
                    self.capture_one_button.configure(state='disabled' if payload else 'normal')
                elif event == 'pattern_progress':
                    completed, total = payload
                    self.pattern_bar.configure(maximum=total, value=completed)
                    self.pattern_label.configure(text=f'{completed}/{total}')
                elif event == 'calibration_session':
                    self.calibration_capture_folder.set(str(payload))
                elif event == 'capture_log':
                    self.append(payload)
                elif event == 'capture_done':
                    kind,err=payload
                    self.capture_thread=None;self.capture_kind=None
                    labels={'calibration':'开始标定采集','manual':'开始手动转台采集','automatic':'开始自动旋转采集','focus':'开始手动调焦'}
                    for name,button in self.capture_buttons.items():button.configure(text=labels[name],state='normal')
                    self.capture_one_button.configure(state='disabled')
                    if 0 < self.pattern_bar['value'] < self.pattern_bar['maximum']:
                        self.pattern_label.configure(text='已停止')
                    self.focus_mode_button.configure(state='disabled');self.focus_reset_button.configure(state='disabled')
                    for view in self.preview_ratios:
                        view.set_focus_mode(False)
                    self.roi_pending.discard('海康')
                    self.manual_pending.discard('海康')
                    self._stop_usb()
                    self.usb_focus_label.pack_forget()
                    self.focus_audio_label.pack_forget()
                    self.focus_text.set('')
                    self.roi_hint.set('分别在两台画面左键拖框，蓝框为各自的曝光区域。')
                    self.preview_button.configure(state='normal')
                    self.status.set('采集已结束' if not err else '采集已停止或失败：'+err)
                    self.append(self.status.get())
                    self.hik_view.show_message('采集已结束，可启动实时预览')
                elif event == 'error':
                    self.append('海康预览失败：'+payload)
                    self.status.set('海康预览失败')
                    self.hik_view.show_message('海康预览失败：'+payload)
                elif event == 'done':
                    self.preview_thread=None
                    self.roi_pending.discard('海康')
                    self.manual_pending.discard('海康')
                    if not self.preview_active:
                        self._finish_preview()
        except queue.Empty: pass
        try:
            frame=self.preview_frames.get_nowait()
            if ((self.preview_active and self.preview_thread is not None) or
                    (self.capture_thread is not None and not self.capture_stop_event.is_set())):
                self._show_frame(self.hik_view, frame)
        except queue.Empty: pass
        try:
            frame = self.usb_frames.get_nowait()
            if self.usb_thread is not None and not self.usb_stop.is_set():
                self._show_frame(self.usb_view, frame)
                if (self.capture_kind == 'focus' and self.focus_mode is not None
                        and not self.capture_stop_event.is_set() and not (self.roi_pending or self.manual_pending)):
                    self._update_usb_focus(frame)
        except queue.Empty: pass
        if (self.capture_kind == 'focus' and self.focus_mode is not None and
                self.usb_focus_auto and self.usb_meter_ready and self.usb_view.frame is not None):
            self.usb_focus_auto = False
            self.usb_commands.put(('startup', (0., 0., 1., 1.)))
        busy = self.roi_pending or self.manual_pending
        self.roi_button.state(['!disabled'] if not busy and self._meter_targets() else ['disabled'])
        if self.capture_kind == 'focus':
            self.focus_mode_button.state(['disabled'] if busy else ['!disabled'])
        if self.close_requested and self.preview_thread is None and self.usb_thread is None:
            self.destroy()
            return
        try:
            while True:
                kind,payload=self.messages.get_nowait()
                if kind=='line':
                    self.append(payload)
                    if payload.startswith('警告'): self.task_has_warnings=True
                    if payload.startswith('拼接结果: '): self.ply.set(payload[len('拼接结果: '):].strip())
                else:
                    title,code,callback=payload;self.process=None;self.status.set(f'{title}：'+(('完成（有质量警告，请查看日志）' if self.task_has_warnings else '完成') if code==0 else f'失败（{code}）'));self.append(self.status.get())
                    if code==0 and callback: callback()
        except queue.Empty: pass
        self.after(100,self._poll)

    def camera_args(self):
        gain=float(self.gain.get())
        if not math.isfinite(gain) or gain<0: raise ValueError('增益不能小于 0')
        if self.auto_exposure.get(): return ['--gain-db',str(gain),'--auto-exposure']
        exposure=self._manual_exposure_us('海康')
        return ['--gain-db',str(gain),'--exposure-us',str(exposure)]

    def checked(self,action):
        try: action()
        except (ValueError, OSError) as exc: messagebox.showerror('参数错误',str(exc))

    def preview(self):
        def go():
            if self.process is not None: raise ValueError('请先结束正在运行的采集或其他任务')
            if self.preview_active or self.preview_thread is not None:
                self.stop_preview();return
            if self.capture_thread is not None: raise ValueError('请先停止采集')
            self.camera_args()
            while not self.preview_frames.empty():
                try:self.preview_frames.get_nowait()
                except queue.Empty:break
            exposure=None if self.auto_exposure.get() else float(self.exposure.get())
            gain=float(self.gain.get()); automatic=self.auto_exposure.get()
            self.preview_stop_event.clear()
            while not self.preview_commands.empty():
                try:self.preview_commands.get_nowait()
                except queue.Empty:break
            self.preview_active = True
            self.hik_view.show_message('正在连接海康相机…')
            self.preview_button.configure(text='停止实时预览')
            self.preview_thread=threading.Thread(target=self._preview_worker,args=(exposure,gain,automatic),daemon=True)
            self.preview_thread.start()
            self._start_usb(automatic)
        self.checked(go)

    def stop_preview(self):
        self.preview_active = False
        self.preview_stop_event.set()
        if self.preview_next is None:
            self._stop_usb()
        self.status.set('正在停止相机预览…')
        if self.preview_thread is None:
            self._finish_preview()

    def _finish_preview(self):
        self.preview_button.configure(text='启动实时预览')
        self.hik_view.show_message('海康预览已停止')
        if self.preview_next is not None:
            callback, self.preview_next = self.preview_next, None
            self.after_idle(callback)
        else:
            self.status.set('海康预览已停止')

    def _preview_worker(self,exposure,gain,automatic):
        import time
        saved={};created=opened=grabbing=initialized=False
        try:
            from preview import (MvCamera, MV_CC_DEVICE_INFO_LIST, MV_CC_DEVICE_INFO,
                MV_USB_DEVICE, MV_ACCESS_Exclusive, MVCC_ENUMVALUE,
                check, set_float_parameter, run_auto_exposure_once, run_roi_auto_exposure,
                cast, POINTER)
            from calibration_capture import get_frame
            camera=MvCamera()
            check(MvCamera.MV_CC_Initialize(),'初始化相机 SDK');initialized=True
            devices=MV_CC_DEVICE_INFO_LIST()
            check(MvCamera.MV_CC_EnumDevices(MV_USB_DEVICE,devices),'搜索 USB 相机')
            if devices.nDeviceNum != 1: raise RuntimeError(f'发现 {devices.nDeviceNum} 台海康相机，请只连接一台')
            info=cast(devices.pDeviceInfo[0],POINTER(MV_CC_DEVICE_INFO)).contents
            check(camera.MV_CC_CreateHandle(info),'创建相机句柄');created=True
            check(camera.MV_CC_OpenDevice(MV_ACCESS_Exclusive,0),'打开相机（请关闭其他相机软件）');opened=True
            for name in ('TriggerMode','ExposureAuto','GainAuto'):
                state=MVCC_ENUMVALUE();check(camera.MV_CC_GetEnumValue(name,state),'读取 '+name)
                saved[name]=state.nCurValue
                check(camera.MV_CC_SetEnumValue(name,0),'设置 '+name)
            if exposure is not None:set_float_parameter(camera,'ExposureTime',exposure)
            set_float_parameter(camera,'Gain',gain)
            check(camera.MV_CC_StartGrabbing(),'启动取流');grabbing=True
            if automatic:
                result=run_auto_exposure_once(camera)
                self.preview_events.put(('auto_result',result))
            self.preview_events.put(('info','海康相机实时预览中'))
            while not self.preview_stop_event.is_set():
                try:command=self.preview_commands.get_nowait()
                except queue.Empty:command=None
                if command and command[0]=='auto_roi':
                    try:
                        result = run_roi_auto_exposure(camera, command[1][0], cancelled=self.preview_stop_event.is_set,
                                                       compensation_ev=command[1][1],
                                                       diagnostic=lambda text: self.preview_events.put(('capture_log', text)))
                    except Exception as exc: result=str(exc)
                    self.preview_events.put(('roi_auto_result',result))
                elif command and command[0] == 'manual':
                    try:
                        set_float_parameter(camera, 'ExposureTime', command[1])
                        result = command[1]
                    except Exception as exc:
                        result = str(exc)
                    self.preview_events.put(('manual_result', ('海康', result)))
                img=get_frame(camera)
                put_latest(self.preview_frames, img)
                time.sleep(0.03)
        except Exception as exc:
            self.preview_events.put(('error',str(exc)))
        finally:
            if grabbing:camera.MV_CC_StopGrabbing()
            if opened:
                for name,value in saved.items():camera.MV_CC_SetEnumValue(name,value)
                camera.MV_CC_CloseDevice()
            if created:camera.MV_CC_DestroyHandle()
            if initialized:MvCamera.MV_CC_Finalize()
            self.preview_events.put(('done', None))

    def _start_usb(self, automatic=False):
        if self.usb_thread is not None:
            if self.usb_stop.is_set():
                self.usb_restart = automatic
            return
        self.usb_stop.clear()
        self.usb_meter_ready = False
        self.usb_commands = queue.Queue()
        self.usb_frames = queue.Queue(maxsize=1)
        self.usb_view.show_message('正在连接 FHD C3 Camera…')
        self.usb_thread = threading.Thread(target=self._usb_worker, args=(automatic, self.usb_manual_exposure), daemon=True)
        self.usb_thread.start()

    def _stop_usb(self):
        self.usb_restart = None
        self.usb_stop.set()
        self.usb_meter_ready = False
        self.usb_stream_ready.clear()

    def _usb_worker(self, automatic, manual_us):
        error = None
        try:
            import cv2
            from usb_camera import (
                discover, exposure_levels, exposure_us, meter, open_camera, read_exposure,
                read_frame, set_exposure, set_manual_exposure, settled_frame, lock_texture_settings, projected_frame,
            )

            with discover() as (index, control, proc_amp):
                capture, frame = open_camera(index)
                original = None
                texture_settings = None
                texture_restore = ExitStack()
                try:
                    levels = exposure_levels(control)
                    if levels:
                        try:
                            original = read_exposure(control)
                        except RuntimeError:
                            levels = range(0)
                    if automatic and levels:
                        self.usb_commands.put(('startup', (0., 0., 1., 1.)))
                    elif manual_us is not None and levels:
                        self.usb_commands.put(('manual', manual_us))
                    self.preview_events.put(('usb_ready', tuple(str(round(exposure_us(level))) for level in levels)))
                    self.usb_stream_ready.set()
                    while not self.usb_stop.is_set():
                        try:
                            command, value = self.usb_commands.get_nowait()
                        except queue.Empty:
                            pass
                        else:
                            if command in ('texture', 'calibration'):
                                request = value
                                if request.set_running_or_notify_cancel():
                                    try:
                                        first = texture_settings is None
                                        if texture_settings is None:
                                            if not levels:
                                                raise RuntimeError('FHD C3 Camera 不支持锁定曝光')
                                            with ExitStack() as rollback:
                                                requested = self.usb_manual_exposure if command == 'calibration' else None
                                                texture_settings = lock_texture_settings(control, proc_amp, levels, rollback, requested)
                                                texture_restore.enter_context(rollback.pop_all())
                                        read_projected = projected_frame if command == 'calibration' and not first else settled_frame
                                        frame = read_projected(capture, self.capture_stop_event)
                                    except Exception as exc:
                                        request.set_exception(exc)
                                    else:
                                        request.set_result((frame, texture_settings))
                                continue
                            try:
                                if command == 'manual':
                                    result = set_manual_exposure(control, levels, value)
                                else:
                                    roi,compensation = value if command == 'auto_roi' else (value,0.0)
                                    result = meter(capture, control, levels, roi, self.usb_stop, compensation_ev=compensation)
                            except Exception as exc:
                                result = str(exc)
                            if command == 'manual':
                                self.preview_events.put(('manual_result', ('USB', result)))
                            else:
                                self.preview_events.put(('usb_roi_result', (result, command == 'startup')))
                            if self.usb_stop.is_set():
                                break
                            frame = settled_frame(capture, self.usb_stop) if command == 'manual' else read_frame(capture)
                        put_latest(self.usb_frames, cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                        if self.usb_stop.wait(.03):
                            break
                        frame = read_frame(capture)
                finally:
                    try:
                        try:
                            texture_restore.close()
                        finally:
                            if original is not None:
                                set_exposure(control, *original)
                    finally:
                        capture.release()
        except Exception as exc:
            if not self.usb_stop.is_set():
                error = str(exc)
        finally:
            self.usb_stream_ready.clear()
            self.preview_events.put(('usb_done', error))

    def focus_guide(self):
        if self.capture_kind=='focus':self.capture_stop_event.set();self.status.set('正在停止调焦…');return
        self.checked(lambda:self._toggle_capture('focus',self._capture_camera_config()))
    def capture_one(self):
        if self.capture_thread is not None and self.capture_kind in ('calibration','manual'):
            if self.capture_commands.empty():self.capture_commands.put(32)
            self.append('已请求采集当前视角；采集过程中请保持物体或标定板不动。')

    def _toggle_capture(self,kind,config):
        if self.capture_thread is not None:
            if self.capture_kind==kind:
                self.capture_stop_event.set();self.status.set('正在停止采集…')
            else:messagebox.showinfo('正在采集','请先停止当前采集。')
            return
        if self.process is not None:raise ValueError('请先结束当前任务')
        if self.preview_active or self.preview_thread is not None:
            if self.preview_next is not None:raise ValueError('正在释放相机，请稍候')
            self.preview_next=lambda:self._toggle_capture(kind,config)
            self.stop_preview();return
        self.capture_stop_event.clear()
        while not self.capture_commands.empty():
            try:self.capture_commands.get_nowait()
            except queue.Empty:break
        self.capture_kind=kind
        if kind == 'focus':
            from camera_metrics import FocusTone

            self.roi_hint.set('左键拖框用于调焦和曝光，右键只调整曝光框。')
            self.focus_mode = None
            self.usb_focus_auto = config['auto']
            self._reset_usb_focus()
            self.focus_tone = FocusTone()
            self._select_focus_audio(self.hik_view)
            self.usb_focus_text.set('USB | 等待调焦画面')
            self.usb_focus_label.pack(after=self.focus_label, anchor='w', pady=3)
            self.focus_audio_label.pack(after=self.usb_focus_label, anchor='w', pady=3)
            for view in self.preview_ratios:
                view.set_focus_mode(True)
            if self.hik_view.focus_roi is not None:
                self.capture_commands.put(('roi', *self.hik_view.focus_roi))
        labels={'calibration':'停止标定采集','manual':'停止手动采集','automatic':'停止自动采集','focus':'停止手动调焦'}
        for name,button in self.capture_buttons.items():
            button.configure(text=labels[name] if name==kind else button.cget('text'),state='normal' if name==kind else 'disabled')
        self.capture_one_button.configure(state='normal' if kind in ('manual','calibration') else 'disabled')
        self.focus_mode_button.configure(state='normal' if kind=='focus' else 'disabled');self.focus_reset_button.configure(state='normal' if kind=='focus' else 'disabled')
        self.preview_button.configure(state='disabled')
        self.hik_view.show_message('正在启动采集…')
        self.status.set('正在启动采集…')
        self.pattern_bar.configure(value=0)
        self.pattern_label.configure(text='待采集')
        self.capture_thread=threading.Thread(target=self._capture_worker,args=(kind,config),daemon=True)
        self._start_usb()
        self.capture_thread.start()

    def _capture_worker(self,kind,config):
        error=None
        try:
            from calibration_capture import main, CaptureInterrupted
            from turntable_control import Turntable
            if kind=='focus':
                from focus_assistant import main as focus_main
                focus_main(ui=EmbeddedCaptureUI(self),exposure_us=config['exposure'],gain_db=config['gain'],auto_exposure=config['auto'])
                return
            controller=None;angle=config.get('angle')
            if kind=='automatic':
                n=config['views'];steps=round(4096/n);angle=steps*360/4096
                controller=Turntable(config['url'],views=n+int(config['loop']),steps=steps,settle_seconds=config['settle'])
                controller.check_idle()
                self.capture_commands.put(32)
            ui=EmbeddedCaptureUI(self)
            main(mode='calibration' if kind=='calibration' else 'object',nominal_step_degrees=angle,
                 auto_turntable=controller,exposure_us=config['exposure'],gain_db=config['gain'],
                 auto_exposure=config['auto'],ui=ui,
                 read_texture_frame=ui.texture_frame if kind in ('manual', 'automatic') else None,
                 read_calibration_frame=ui.calibration_frame if kind == 'calibration' else None)
        except Exception as exc:
            error=str(exc)
            if not self.capture_stop_event.is_set():self.preview_events.put(('capture_log','采集失败：'+error))
        finally:
            self.preview_events.put(('capture_done',(kind,error)))

    def _capture_camera_config(self):
        self.camera_args()
        return {'exposure':None if self.auto_exposure.get() else float(self.exposure.get()),
                'gain':float(self.gain.get()),'auto':self.auto_exposure.get()}

    def cal_capture(self):
        if self.capture_kind=='calibration':self.capture_stop_event.set();self.status.set('正在停止采集…');return
        self.checked(lambda:self._toggle_capture('calibration',self._capture_camera_config()))
    def manual_capture(self):
        if self.capture_kind=='manual':self.capture_stop_event.set();self.status.set('正在停止采集…');return
        def go():
            angle=float(self.manual_step.get())
            if not -360 <= angle <= 360 or angle==0:raise ValueError('名义角度须为 -360 到 360 的非零数')
            config=self._capture_camera_config();config['angle']=angle
            self._toggle_capture('manual',config)
        self.checked(go)
    def auto_capture(self):
        if self.capture_kind=='automatic':self.capture_stop_event.set();self.status.set('正在停止采集…');return
        def go():
            url=self.esp.get().strip();settle=float(self.settle.get());views=int(self.view_count.get())
            if not url:raise ValueError('请输入 ESP32 地址')
            if '://' not in url:url='http://'+url
            if not math.isfinite(settle) or settle<0:raise ValueError('稳定等待时间不能为负数')
            if not 1<=views<=128:raise ValueError('采集组数应为 1～128')
            config=self._capture_camera_config();config.update(url=url,settle=settle,views=views,loop=self.loop.get())
            self._toggle_capture('automatic',config)
        self.checked(go)
    def require(self,var,kind='file'):
        raw=var.get().strip()
        if not raw:raise ValueError('请选择所需的文件或文件夹')
        p=Path(raw)
        if not (p.is_dir() if kind=='dir' else p.is_file()):raise ValueError('路径不存在：'+str(p))
        return p.resolve()
    def reconstruct(self):
        def go():
            p=self.require(self.view_folder,'dir');cal=self.require(self.calibration)
            if len(list(p.glob('graycode_*.png')))!=44:raise ValueError('采集文件夹应包含 44 张 graycode 图片')
            step=int(self.step.get());bits=int(self.bits.get())
            if step<1:raise ValueError('采样间隔必须大于零')
            name='dashboard_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+f'_ignore{bits}.ply'
            self.run('单视角重建','reconstruct_object.py',[p,cal,'--step',step,'--ignore-finest-bits',bits,'--output',name],lambda:self.ply.set(str(p/name)))
        self.checked(go)
    def marker_batch(self):
        def go():
            scan=self.require(self.scan_folder,'dir')
            raw=self.marker_calibration.get().strip()
            if not raw or not Path(raw).exists():raise ValueError('请选择标定文件或标定结果文件夹')
            cal=Path(raw).resolve();size=float(self.marker_size.get());board=float(self.board_size.get());cols=int(self.board_columns.get());rows=int(self.board_rows.get());step=int(self.step.get());bits=int(self.bits.get())
            if not math.isfinite(size) or not math.isfinite(board) or min(size,board)<=0 or min(cols,rows)<3 or step<1:raise ValueError('标记、棋盘格尺寸和采样间隔不合理')
            self.run('二维码自动拼接','automatic_marker_merge.py',[cal,scan,'--marker-size',size,'--square-size',board,'--columns',cols,'--rows',rows,'--step',step,'--bits',bits])
        self.checked(go)

    def _calibration_board_changed(self, _event):
        charuco = self.calibration_board.get() == 'ChArUco'
        self.calibration_columns.set('10' if charuco else '9')
        self.calibration_rows.set('10' if charuco else '7')

    def calibrate(self):
        def go():
            folder = self.require(self.calibration_capture_folder, 'dir')
            board = 'charuco' if self.calibration_board.get() == 'ChArUco' else 'chessboard'
            name = f"calibration_{board}_{datetime.now():%Y%m%d_%H%M%S_%f}.xml"
            output = folder / name
            args = [folder, '--output', output, '--board', board, '--square-size', self.calibration_square.get(),
                    '--columns', self.calibration_columns.get(), '--rows', self.calibration_rows.get(),
                    '--marker-size', self.calibration_marker.get()]
            def done():
                metadata = json.loads(output.with_suffix('.json').read_text(encoding='utf-8'))
                if metadata.get('camera') == 'usb':
                    self.append(f'FHD C3 单机标定已保存到 {output}。海康重建参数保留原设置。')
                else:
                    self.calibration.set(str(output))
                    self.marker_calibration.set(str(output))
            self.run('相机与投影仪标定', 'automatic_calibration.py', args, done)
        self.checked(go)

    def charuco_batch(self):
        def go():
            scan = self.require(self.scan_folder, 'dir')
            cal = self.require(self.marker_calibration)
            output = scan / f"paper_turntable_{datetime.now():%Y%m%d_%H%M%S_%f}"
            args = [scan, cal, '--output', output, '--step', self.step.get(), '--ignore-finest-bits',
                    self.bits.get(), '--radius', self.radius.get(), '--clearance', self.clearance.get(), '--icp']
            self.run('ChArUco 板转台拼接', 'paper_turntable.py', args,
                     lambda: self.ply.set(str(output / cal.stem / 'merged_coarse.ply')))
        self.checked(go)

    def batch(self):
        def go():
            scan=self.require(self.scan_folder,'dir');cal=self.require(self.calibration);axis=self.require(self.axis)
            if not (scan/'session.json').is_file():raise ValueError('扫描文件夹中没有 session.json')
            bits=int(self.bits.get());step=int(self.step.get());radius=float(self.radius.get());clearance=float(self.clearance.get())
            if step<1 or not math.isfinite(radius) or not math.isfinite(clearance) or radius<=0 or clearance<0:raise ValueError('采样间隔、半径和离盘面距离不合适')
            args=[scan,cal,axis,'--bits',bits,'--step',step,'--radius',radius,'--clearance',clearance]
            if self.no_crop.get():args.append('--no-crop')
            if self.angles.get().strip():args.extend(['--angles',self.require(self.angles)])
            self.run('批量重建与拼接','dashboard_batch.py',args)
        self.checked(go)
    def view(self):
        def go():
            args=[self.require(self.ply)]
            if self.keep_colors.get():args.append('--preserve-colors')
            self.run('点云查看','view_point_cloud.py',args)
        self.checked(go)
    def draw_scene(self):
        def go():
            path=self.require(self.ply);cal=self.require(self.calibration);out=path.with_name(path.stem+'_camera_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.png')
            self.run('相机位置绘图','plot_camera_scene.py',[path,'--calibration',cal,'--output',out,'--label',self.fig_label.get(),'--parameters',self.fig_params.get()])
        self.checked(go)
    def compare(self):
        def go():
            scan=self.require(self.scan_folder,'dir');cal=self.require(self.calibration);roi=[int(x) for x in self.roi.get().split()];depth=[float(x) for x in self.depth.get().split()]
            if len(roi)!=4 or len(depth)!=2:raise ValueError('ROI 需要 4 个整数，深度需要 2 个数字')
            if not messagebox.askyesno('覆盖已有结果','该专用试验会覆盖同一目录中已有的回环比较文件。继续吗？'):return
            self.run('回环比较','compare_loop_repeat.py',[scan,cal,'--roi',*roi,'--depth',*depth])
        self.checked(go)


if __name__ == '__main__':
    App().mainloop()
