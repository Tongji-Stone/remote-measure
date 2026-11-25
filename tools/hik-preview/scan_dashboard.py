"""Structured light experiment control panel for acquisition and reconstruction."""
import os
import ctypes

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
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = Path(__file__).resolve().parent
DATA = ROOT / 'data'
DEFAULT_CAL = DATA / 'procam_calibration' / '20261120_analysis' / 'all11' / 'calibration_20261116_unit_provisional.xml'
DEFAULT_AXIS = DATA / 'procam_calibration' / '20261120_analysis' / 'axis_approx.json'


class EmbeddedCaptureUI:
    """Thread-safe bridge between the acquisition worker and the Tk event loop."""
    def __init__(self, app): self.app=app
    def cancelled(self): return self.app.capture_stop_event.is_set()
    def poll_key(self):
        try: return self.app.capture_commands.get_nowait()
        except queue.Empty: return -1
    def show_frame(self, frame):
        q=self.app.preview_frames
        try:
            while True:q.get_nowait()
        except queue.Empty:pass
        try:q.put_nowait(frame)
        except queue.Full:pass
    def set_capturing(self,busy): self.app.preview_events.put(('capture_busy',busy))
    def auto_exposure_result(self,value): self.app.preview_events.put(('auto_result',value))
    def roi_exposure_result(self,result): self.app.preview_events.put(('roi_auto_result',result))
    def focus_status(self,mode,score,best,ratio,saturation):
        self.app.preview_events.put(('focus_status',(mode,score,best,ratio,saturation)))
    def log(self,*args,**kwargs):
        self.app.preview_events.put(('capture_log',' '.join(str(x) for x in args)))


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('结构光实验操作台')
        self.geometry('1200x900')
        self.minsize(900, 700)
        if os.name == 'nt': self.state('zoomed')
        self.process = None
        self.preview_thread = None
        self.preview_stop_event = threading.Event()
        self.preview_frames = queue.Queue(maxsize=1)
        self.preview_events = queue.Queue()
        self.preview_next = None
        self.preview_photo = None
        self.preview_current_frame = None
        self.capture_thread = None
        self.capture_stop_event = threading.Event()
        self.capture_commands = queue.Queue()
        self.capture_kind = None
        self.capture_buttons = {}
        self.focus_start_point = None
        self.selected_roi = None
        self.focus_zoom = False
        self.preview_commands = queue.Queue()
        self.roi_metering = False
        self.focus_text = tk.StringVar(value='调焦：启动后在画面拖框选择区域')
        self.task_has_warnings = False
        self.messages = queue.Queue()
        self.status = tk.StringVar(value='就绪')
        self.calibration = tk.StringVar(value=str(DEFAULT_CAL) if DEFAULT_CAL.exists() else '')
        self.axis = tk.StringVar(value=str(DEFAULT_AXIS) if DEFAULT_AXIS.exists() else '')
        self.exposure = tk.StringVar(value='50000')
        self.gain = tk.StringVar(value='0')
        self.auto_exposure = tk.BooleanVar(value=False)
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
        self.log = tk.Text(logs, height=5, wrap='word', font=('Consolas', 9)); self.log.pack(side='left', fill='both', expand=True)
        bar = ttk.Scrollbar(logs, command=self.log.yview); bar.pack(side='right', fill='y'); self.log.configure(yscrollcommand=bar.set)
        footer = ttk.Frame(self, padding=(16, 0, 16, 10)); footer.pack(fill='x')
        ttk.Label(footer, textvariable=self.status).pack(side='left')
        ttk.Button(footer, text='清空日志', command=lambda: self.log.delete('1.0','end')).pack(side='right')
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
        ttk.Label(r,text='曝光（微秒）').pack(side='left')
        ttk.Combobox(r,textvariable=self.exposure,width=9,values=['500','1000','2000','3000','5000','7500','10000','15000','20000','25000','30000','40000','50000','100000']).pack(side='left',padx=5)
        ttk.Label(r,text='增益 dB').pack(side='left')
        ttk.Combobox(r,textvariable=self.gain,width=5,values=['0','1','2','3','4','6','8','10','12','16','20','24']).pack(side='left',padx=5)
        ttk.Checkbutton(left,text='自动曝光一次后锁定',variable=self.auto_exposure).pack(anchor='w',pady=4)
        self.preview_button=ttk.Button(left,text='启动实时预览',command=self.preview);self.preview_button.pack(fill='x',pady=3)
        self.roi_button=ttk.Button(left,text='按框选区自动曝光',command=self.meter_selected_roi,state='disabled');self.roi_button.pack(fill='x',pady=3)
        ttk.Label(left,text='在右侧画面拖框；该框用于测光和调焦。',wraplength=420).pack(anchor='w')
        self.capture_buttons['focus']=ttk.Button(left,text='开始手动调焦',command=self.focus_guide)
        self.capture_buttons['focus'].pack(fill='x',pady=3)
        r=ttk.Frame(left);r.pack(fill='x')
        self.focus_mode_button=ttk.Button(r,text='切换白光 / 横竖条纹',command=lambda:self.focus_command(ord('m')),state='disabled')
        self.focus_mode_button.pack(side='left',fill='x',expand=True)
        self.focus_reset_button=ttk.Button(r,text='重置最高分',command=lambda:self.focus_command(ord('b')),state='disabled')
        self.focus_reset_button.pack(side='left',fill='x',expand=True)
        self.focus_zoom_button=ttk.Button(left,text='显示选区放大',command=self.toggle_focus_zoom,state='disabled')
        self.focus_zoom_button.pack(fill='x',pady=2)
        ttk.Label(left,textvariable=self.focus_text,wraplength=440,justify='left').pack(anchor='w',pady=3)
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
        self.preview_canvas=tk.Canvas(right,bg='#1c222b',highlightthickness=0,width=400,height=260)
        self.preview_canvas.pack(fill='both',expand=True)
        self.preview_image_id=self.preview_canvas.create_image(0,0,anchor='center')
        self.preview_text_id=self.preview_canvas.create_text(0,0,text='启动实时预览或采集',fill='#d7dee8')
        self.preview_canvas.bind('<Configure>',lambda event:self._render_preview())
        self.preview_canvas.bind('<ButtonPress-1>',self._focus_drag_start)
        self.preview_canvas.bind('<B1-Motion>',self._focus_drag_move)
        self.preview_canvas.bind('<ButtonRelease-1>',self._focus_drag_end)
        self.focus_rect=self.preview_canvas.create_rectangle(0,0,0,0,outline='#00ff80',width=2,state='hidden')
        ttk.Label(right,text='采集期间画面显示在这里；投影仪另显示结构光图案。',wraplength=800).pack(anchor='w',pady=5)

    def _render_preview(self):
        canvas=self.preview_canvas
        width=max(1,canvas.winfo_width());height=max(1,canvas.winfo_height())
        canvas.coords(self.preview_image_id,width//2,height//2)
        canvas.coords(self.preview_text_id,width//2,height//2)
        if self.preview_current_frame is not None and width>2 and height>2:
            from PIL import Image, ImageTk
            image=Image.fromarray(self.preview_current_frame)
            if self.capture_kind=='focus' and self.focus_zoom and self.selected_roi is not None:
                iw,ih=image.size;x0,y0,x1,y1=self.selected_roi
                ax=max(0,min(iw-1,int(x0*iw)));ay=max(0,min(ih-1,int(y0*ih)))
                bounds=(ax,ay,min(iw,max(ax+1,int(x1*iw))),min(ih,max(ay+1,int(y1*ih))))
                image=image.crop(bounds)
                scale=min((width-4)/image.width,(height-4)/image.height)
                image=image.resize((max(1,round(image.width*scale)),max(1,round(image.height*scale))),Image.Resampling.LANCZOS)
            else:
                image.thumbnail((max(1,width-4),max(1,height-4)),Image.Resampling.LANCZOS)
            self.preview_photo=ImageTk.PhotoImage(image)
            canvas.itemconfigure(self.preview_image_id,image=self.preview_photo)
            canvas.itemconfigure(self.preview_text_id,text='')
            if self.focus_start_point is None:self._draw_selected_roi()

    def _draw_selected_roi(self):
        if self.selected_roi is None or self.preview_photo is None or (self.capture_kind=='focus' and self.focus_zoom):
            self.preview_canvas.itemconfigure(self.focus_rect,state='hidden');return
        w,h=self.preview_photo.width(),self.preview_photo.height()
        left=(self.preview_canvas.winfo_width()-w)/2;top=(self.preview_canvas.winfo_height()-h)/2
        x0,y0,x1,y1=self.selected_roi
        self.preview_canvas.coords(self.focus_rect,left+x0*w,top+y0*h,left+x1*w,top+y1*h)
        self.preview_canvas.itemconfigure(self.focus_rect,state='normal');self.preview_canvas.tag_raise(self.focus_rect)

    def _preview_message(self,text):
        self.preview_current_frame=None;self.preview_photo=None
        self.preview_canvas.itemconfigure(self.preview_image_id,image='')
        self.preview_canvas.itemconfigure(self.preview_text_id,text=text)
        self.preview_canvas.itemconfigure(self.focus_rect,state='hidden')

    def _focus_drag_start(self,event):
        if (self.capture_kind != 'focus' and self.preview_thread is None) or self.preview_photo is None:return
        self.focus_start_point=(event.x,event.y)
        self.preview_canvas.coords(self.focus_rect,event.x,event.y,event.x,event.y)
        self.preview_canvas.itemconfigure(self.focus_rect,state='normal')
        self.preview_canvas.tag_raise(self.focus_rect)

    def _focus_drag_move(self,event):
        if self.focus_start_point:
            self.preview_canvas.coords(self.focus_rect,*self.focus_start_point,event.x,event.y)

    def _focus_drag_end(self,event):
        if self.focus_start_point is None or self.preview_photo is None:return
        x0,y0=self.focus_start_point;self.focus_start_point=None
        w,h=self.preview_photo.width(),self.preview_photo.height()
        left=(self.preview_canvas.winfo_width()-w)/2;top=(self.preview_canvas.winfo_height()-h)/2
        xs=sorted((max(0,min(1,(x0-left)/w)),max(0,min(1,(event.x-left)/w))))
        ys=sorted((max(0,min(1,(y0-top)/h)),max(0,min(1,(event.y-top)/h))))
        if xs[1]-xs[0]>.01 and ys[1]-ys[0]>.01:
            if self.capture_kind=='focus' and self.focus_zoom and self.selected_roi is not None:
                bx0,by0,bx1,by1=self.selected_roi
                candidate=(bx0+xs[0]*(bx1-bx0),by0+ys[0]*(by1-by0),bx0+xs[1]*(bx1-bx0),by0+ys[1]*(by1-by0))
            else:
                candidate=(xs[0],ys[0],xs[1],ys[1])
            if self.capture_kind=='focus' and self.preview_current_frame is not None:
                h0,w0=self.preview_current_frame.shape[:2]
                scale=min(1.,1100/w0,750/h0)
                if (candidate[2]-candidate[0])*w0*scale<=11 or (candidate[3]-candidate[1])*h0*scale<=11:
                    self.append('选区太小，请选大一些的区域。')
                    self._render_preview();return
            self.selected_roi=candidate
            if self.capture_kind=='focus':
                self.focus_zoom=True;self.focus_zoom_button.configure(text='返回全图重新选框')
                self.capture_commands.put(('roi',*self.selected_roi))
            self.append('已选择共用区域：可用于调焦和自动曝光。')
        self._render_preview()

    def toggle_focus_zoom(self):
        if self.capture_kind!='focus':return
        if self.focus_zoom:
            self.focus_zoom=False;self.focus_zoom_button.configure(text='显示选区放大')
        elif self.selected_roi is not None:
            self.focus_zoom=True;self.focus_zoom_button.configure(text='返回全图重新选框')
        else:
            messagebox.showinfo('请先框选','请在画面上拖框选择区域。');return
        self._render_preview()

    def meter_selected_roi(self):
        if self.roi_metering:return
        if self.selected_roi is None:
            messagebox.showinfo('请先框选','请先在右侧画面上拖框选择测光区域。');return
        if self.capture_kind=='focus':commands=self.capture_commands
        elif self.preview_thread is not None:commands=self.preview_commands
        else:
            messagebox.showinfo('请先预览','请先启动实时预览或手动调焦。');return
        self.roi_metering=True;self.roi_button.configure(state='disabled')
        self.status.set('正在按框选区域自动曝光…')
        commands.put(('auto_roi',self.selected_roi))

    def focus_command(self,key):
        if self.capture_kind=='focus':self.capture_commands.put(key)

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
        self.log.insert('end',text+'\n');self.log.see('end')

    def run(self,title,script,args,on_success=None):
        if self.capture_thread is not None:
            messagebox.showinfo('正在采集','请先点击当前采集按钮停止。');return
        if self.preview_next is not None:
            messagebox.showinfo('正在释放相机','请稍候。');return
        if self.preview_thread is not None:
            self.preview_next=lambda:self.run(title,script,args,on_success)
            self.stop_preview()
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
        if self.preview_thread is not None:
            self.preview_next=self.destroy
            self.stop_preview()
            return
        if self.process is not None:
            messagebox.showinfo('任务正在运行', '请先在预览或采集窗口按 Q 退出；点云查看窗口请直接关闭。任务结束后再关闭操作台。')
            return
        self.destroy()

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
                    self.roi_metering=False
                    if isinstance(payload,dict):
                        self.exposure.set(str(round(payload['exposure_us'])))
                        self.auto_exposure.set(False)
                        self.append(f"区域曝光锁定：{payload['exposure_us']:.0f} 微秒；区域亮度 P90={payload['p90']:.0f}，过亮比例={payload['clipped']:.1%}；{payload['status']}。已切换手动锁定，供采集使用。")
                        self.status.set('区域测光完成：'+payload['status'])
                    else:
                        self.append('区域测光失败：'+str(payload));self.status.set('区域测光失败')
                    if self.preview_thread is not None or self.capture_kind=='focus':self.roi_button.configure(state='normal')
                elif event == 'focus_status':
                    mode,score,best,ratio,saturation=payload
                    names=['白光：调整相机镜头','竖条纹：调整投影仪','横条纹：调整投影仪']
                    self.focus_text.set(f'{names[mode]} | 清晰度 {score:.1f} / 最高 {best:.1f} | 相对 {ratio:.0%} | 极暗或过亮 {saturation:.0%}')
                elif event == 'capture_busy':
                    self.capture_one_button.configure(state='disabled' if payload else 'normal')
                elif event == 'capture_log':
                    self.append(payload)
                elif event == 'capture_done':
                    kind,err=payload
                    self.capture_thread=None;self.capture_kind=None
                    labels={'calibration':'开始标定采集','manual':'开始手动转台采集','automatic':'开始自动旋转采集','focus':'开始手动调焦'}
                    for name,button in self.capture_buttons.items():button.configure(text=labels[name],state='normal')
                    self.capture_one_button.configure(state='disabled')
                    self.focus_mode_button.configure(state='disabled');self.focus_reset_button.configure(state='disabled')
                    self.focus_zoom=False;self.focus_zoom_button.configure(state='disabled',text='显示选区放大')
                    self.roi_metering=False;self.roi_button.configure(state='disabled')
                    self.focus_text.set('调焦：启动后在画面拖框选择区域')
                    self.preview_button.configure(state='normal')
                    self.status.set('采集已结束' if not err else '采集已停止或失败：'+err)
                    self.append(self.status.get())
                    self._preview_message('采集已结束，可启动实时预览')
                elif event == 'error':
                    self.append('相机预览失败：'+payload)
                    self.status.set('相机预览失败')
                    self._preview_message('相机预览失败：'+payload)
                elif event == 'done':
                    self.preview_thread=None
                    self.roi_metering=False;self.roi_button.configure(state='disabled')
                    self.preview_button.configure(text='启动实时预览')
                    if self.preview_next is not None:
                        callback=self.preview_next;self.preview_next=None
                        self.after_idle(callback)
                    elif payload:
                        self.status.set('相机预览已停止')
                        self._preview_message('预览已停止')
        except queue.Empty: pass
        try:
            frame=self.preview_frames.get_nowait()
            if ((self.preview_thread is not None and not self.preview_stop_event.is_set()) or (self.capture_thread is not None and not self.capture_stop_event.is_set())):
                self.preview_current_frame=frame
                self._render_preview()
        except queue.Empty: pass
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
        exposure=float(self.exposure.get())
        if not math.isfinite(exposure) or exposure<=0: raise ValueError('曝光时间必须大于 0')
        return ['--gain-db',str(gain),'--exposure-us',str(exposure)]

    def checked(self,action):
        try: action()
        except (ValueError, OSError) as exc: messagebox.showerror('参数错误',str(exc))

    def preview(self):
        def go():
            if self.process is not None: raise ValueError('请先结束正在运行的采集或其他任务')
            if self.preview_thread is not None:
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
            self.roi_button.configure(state='normal')
            self._preview_message('正在连接相机…')
            self.preview_button.configure(text='停止实时预览')
            self.preview_thread=threading.Thread(target=self._preview_worker,args=(exposure,gain,automatic),daemon=True)
            self.preview_thread.start()
        self.checked(go)

    def stop_preview(self):
        if self.preview_thread is not None:
            self.preview_stop_event.set()
            self.status.set('正在停止相机预览…')

    def _preview_worker(self,exposure,gain,automatic):
        import time
        saved={};created=opened=grabbing=initialized=False; success=False
        try:
            from preview import (MvCamera, MV_CC_DEVICE_INFO_LIST, MV_CC_DEVICE_INFO,
                MV_USB_DEVICE, MV_GIGE_DEVICE, MV_ACCESS_Exclusive, MVCC_ENUMVALUE,
                MV_TRIGGER_MODE_OFF, check, set_float_parameter, run_auto_exposure_once, run_roi_auto_exposure,
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
            self.preview_events.put(('info','相机实时预览中'))
            while not self.preview_stop_event.is_set():
                try:command=self.preview_commands.get_nowait()
                except queue.Empty:command=None
                if command and command[0]=='auto_roi':
                    try: result=run_roi_auto_exposure(camera,command[1],cancelled=self.preview_stop_event.is_set)
                    except Exception as exc: result=str(exc)
                    self.preview_events.put(('roi_auto_result',result))
                img=get_frame(camera)
                if self.preview_frames.full():
                    try:self.preview_frames.get_nowait()
                    except queue.Empty:pass
                try:self.preview_frames.put_nowait(img)
                except queue.Full:pass
                time.sleep(0.03)
            success=True
        except Exception as exc:
            self.preview_events.put(('error',str(exc)))
        finally:
            if grabbing:camera.MV_CC_StopGrabbing()
            if opened:
                for name,value in saved.items():camera.MV_CC_SetEnumValue(name,value)
                camera.MV_CC_CloseDevice()
            if created:camera.MV_CC_DestroyHandle()
            if initialized:MvCamera.MV_CC_Finalize()
            self.preview_events.put(('done',success))
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
        if self.preview_thread is not None:
            if self.preview_next is not None:raise ValueError('正在释放相机，请稍候')
            self.preview_next=lambda:self._toggle_capture(kind,config)
            self.stop_preview();return
        self.capture_stop_event.clear()
        while not self.capture_commands.empty():
            try:self.capture_commands.get_nowait()
            except queue.Empty:break
        self.capture_kind=kind
        if kind=='focus' and self.selected_roi is not None:self.capture_commands.put(('roi',*self.selected_roi))
        labels={'calibration':'停止标定采集','manual':'停止手动采集','automatic':'停止自动采集','focus':'停止手动调焦'}
        for name,button in self.capture_buttons.items():
            button.configure(text=labels[name] if name==kind else button.cget('text'),state='normal' if name==kind else 'disabled')
        self.capture_one_button.configure(state='normal' if kind in ('manual','calibration') else 'disabled')
        self.focus_mode_button.configure(state='normal' if kind=='focus' else 'disabled');self.focus_reset_button.configure(state='normal' if kind=='focus' else 'disabled')
        self.focus_zoom=(kind=='focus' and self.selected_roi is not None)
        self.focus_zoom_button.configure(state='normal' if kind=='focus' else 'disabled',text='返回全图重新选框' if self.focus_zoom else '显示选区放大')
        self.roi_button.configure(state='normal' if kind=='focus' else 'disabled')
        self.preview_button.configure(state='disabled')
        self._preview_message('正在启动采集…')
        self.status.set('正在启动采集…')
        self.capture_thread=threading.Thread(target=self._capture_worker,args=(kind,config),daemon=True)
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
                 auto_exposure=config['auto'],ui=ui)
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
