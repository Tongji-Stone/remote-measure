"""Lightweight local launcher for experimental structured-light reconstruction."""
import os
import subprocess
import sys
import threading
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / 'data' / 'procam_calibration'
CALIB = DATA / 'calibration_result_初步未验证.xml'
SCRIPTS = Path(__file__).resolve().parent
OUTPUT = 'experimental_point_cloud.ply'


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('结构光扫描操作台（试验版）')
        self.geometry('850x570')
        self.minsize(700, 480)
        self.busy = False
        self.status = tk.StringVar(value='请选择采集组')
        ttk.Label(self, text='结构光扫描操作台', font=('Microsoft YaHei UI', 17, 'bold')).pack(anchor='w', padx=18, pady=(15, 3))
        ttk.Label(self, text='单视角与拼接点云 · 当前标定尚未验证，请勿用于尺寸或体积测量').pack(anchor='w', padx=18)
        pane = ttk.Frame(self, padding=18)
        pane.pack(fill='both', expand=True)
        ttk.Label(pane, text='采集组（需要 44 张图片）').pack(anchor='w')
        row = ttk.Frame(pane)
        row.pack(fill='both', expand=True, pady=(5, 8))
        self.listbox = tk.Listbox(row, font=('Microsoft YaHei UI', 10), exportselection=False)
        self.listbox.pack(side='left', fill='both', expand=True)
        scroll = ttk.Scrollbar(row, orient='vertical', command=self.listbox.yview)
        scroll.pack(side='right', fill='y')
        self.listbox.configure(yscrollcommand=scroll.set)
        self.listbox.bind('<<ListboxSelect>>', self.on_select)
        self.listbox.bind('<Double-Button-1>', lambda event: self.view())
        buttons = ttk.Frame(pane)
        buttons.pack(fill='x')
        for label, command in [('刷新列表', self.refresh), ('生成点云', self.reconstruct), ('查看彩色点云', self.view), ('打开文件夹', self.open_folder)]:
            ttk.Button(buttons, text=label, command=command).pack(side='left', padx=(0, 8))
        more = ttk.Frame(pane)
        more.pack(fill='x', pady=(8, 0))
        ttk.Button(more, text='查看固定轴拼接（试验）', command=self.view_fixed_axis).pack(side='left', padx=(0, 8))
        ttk.Button(more, text='打开其他 PLY', command=self.open_ply).pack(side='left')
        ttk.Label(pane, textvariable=self.status, wraplength=790).pack(anchor='w', pady=(12, 5))
        self.log = tk.Text(pane, height=7, wrap='word', font=('Consolas', 9))
        self.log.pack(fill='x')
        self.log.insert('end', '选中采集组后，点击“生成点云”，完成后点击“查看彩色点云”。\n')
        self.refresh()

    def refresh(self):
        self.folders = sorted((p for p in DATA.glob('capture_*') if p.is_dir()), key=lambda p: p.name, reverse=True) if DATA.exists() else []
        self.listbox.delete(0, 'end')
        for p in self.folders:
            count = len(list(p.glob('graycode_*.png')))
            exists = '  [已有点云]' if (p / OUTPUT).exists() else ''
            self.listbox.insert('end', f'{p.name}  ({count}/44){exists}')
        self.status.set(f'找到 {len(self.folders)} 组图片，请选择一组。')

    def selected(self):
        selected = self.listbox.curselection()
        if not selected:
            messagebox.showinfo('请选择采集组', '请先在列表中选择一组图片。')
            return None
        return self.folders[selected[0]]

    def on_select(self, event=None):
        if self.listbox.curselection():
            p = self.folders[self.listbox.curselection()[0]]
            self.status.set(f'已选择：{p.name}')

    def append(self, value):
        self.log.insert('end', value + '\n')
        self.log.see('end')

    def reconstruct(self):
        p = self.selected()
        if p is None or self.busy:
            return
        if len(list(p.glob('graycode_*.png'))) != 44:
            messagebox.showerror('图片不完整', '该组图片不是 44 张，请先检查采集结果。')
            return
        if not CALIB.exists():
            messagebox.showerror('找不到标定文件', f'找不到试验标定文件：\n{CALIB}')
            return
        if (p / OUTPUT).exists() and not messagebox.askyesno('覆盖点云', '该组已有点云，重新生成会覆盖它。继续吗？'):
            return
        self.busy = True
        self.status.set('正在生成点云，请稍候……')
        self.append('开始处理：' + p.name)
        def work():
            try:
                result = subprocess.run([sys.executable, str(SCRIPTS / 'reconstruct_object.py'), str(p), str(CALIB)], cwd=str(ROOT), capture_output=True, text=True, encoding='utf-8', errors='replace', env={**os.environ, 'PYTHONUTF8': '1'}, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
                output = (result.stdout + result.stderr).strip()
                self.after(0, lambda: self.finish(p, result.returncode, output))
            except Exception as exc:
                error = str(exc)
                self.after(0, lambda: self.finish(p, 1, error))
        threading.Thread(target=work, daemon=True).start()

    def finish(self, p, code, output):
        self.busy = False
        self.append(output or ('完成' if code == 0 else '生成失败'))
        self.status.set(('已生成：' + str(p / OUTPUT)) if code == 0 else '生成失败，请查看下方信息。')
        if code != 0:
            messagebox.showerror('生成失败', output or '请检查图片和标定文件。')

    def view(self):
        p = self.selected()
        if p is None:
            return
        target = p / OUTPUT
        colored = p / 'experimental_depth_color.ply'
        use_colored = colored.exists() and (not target.exists() or colored.stat().st_mtime >= target.stat().st_mtime)
        if not target.exists() and not use_colored:
            messagebox.showinfo('没有点云', '请先点击“生成点云”。')
            return
        try:
            command = [sys.executable, str(SCRIPTS / 'view_point_cloud.py'), str(colored if use_colored else target)]
            if use_colored:
                command.append('--preserve-colors')
            subprocess.Popen(command, cwd=str(ROOT), creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            self.status.set('已启动点云窗口：左键旋转，滚轮缩放；颜色表示深度。')
        except OSError as exc:
            messagebox.showerror('无法打开点云', str(exc))

    def open_folder(self):
        p = self.selected()
        if p is not None:
            os.startfile(str(p))

    def view_ply(self, path):
        if not path.is_file():
            messagebox.showerror('找不到点云', f'文件不存在：\n{path}')
            return
        try:
            subprocess.Popen([sys.executable, str(SCRIPTS / 'view_point_cloud.py'), str(path), '--preserve-colors'], cwd=str(ROOT), creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            self.status.set(f'已启动点云窗口：{path.name}；左键旋转，滚轮缩放。')
        except OSError as exc:
            messagebox.showerror('无法打开点云', str(exc))

    def view_fixed_axis(self):
        self.view_ply(DATA / 'box_fixed_axis_experimental.ply')

    def open_ply(self):
        name = filedialog.askopenfilename(title='选择点云', initialdir=str(DATA), filetypes=[('PLY 点云', '*.ply'), ('所有文件', '*.*')])
        if name:
            self.view_ply(Path(name))


if __name__ == '__main__':
    App().mainloop()
