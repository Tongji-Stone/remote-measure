"""Aspect-preserving preview pane with normalized selection coordinates."""

import tkinter as tk
from tkinter import ttk


class CameraPreview(ttk.Frame):
    def __init__(self, parent, name, placeholder):
        super().__init__(parent)
        self.name = name
        header = ttk.Frame(self)
        header.pack(fill='x', pady=(0, 4))
        self.title = ttk.Label(header, text=name)
        self.title.pack(side='left')
        self.zoom_button = ttk.Button(header, text='放大选区', command=self.toggle_focus_zoom)
        self.canvas = tk.Canvas(self, bg='#1c222b', highlightthickness=0, width=1, height=1)
        self.canvas.pack(fill='both', expand=True)
        self.image_id = self.canvas.create_image(0, 0, anchor='nw')
        self.text_id = self.canvas.create_text(0, 0, text=placeholder, fill='#d7dee8', justify='center')
        self.rects = {
            'focus': self.canvas.create_rectangle(0, 0, 0, 0, outline='#00ff80', width=2, state='hidden'),
            'exposure': self.canvas.create_rectangle(0, 0, 0, 0, outline='#57caff', width=2, dash=(6, 3), state='hidden'),
        }
        self.exposure_roi = None
        self.focus_roi = None
        self.focus_zoom = False
        self.frame = None
        self.photo = None
        self.image_box = None
        self.source_box = (0., 0., 1., 1.)
        self.drag_start = None
        self.drag_kind = None

    def set_frame(self, frame):
        self.frame = frame
        height, width = frame.shape[:2]
        title = f'{self.name}  {width} × {height}'
        if self.title.cget('text') != title:
            self.title.configure(text=title)
        return width / height

    def show_message(self, message):
        self.frame = self.photo = self.image_box = self.drag_start = None
        self.drag_kind = None
        self.title.configure(text=self.name)
        self.canvas.itemconfigure(self.image_id, image='')
        self.canvas.itemconfigure(self.text_id, text=message)
        for rect in self.rects.values():
            self.canvas.itemconfigure(rect, state='hidden')

    def set_focus_mode(self, enabled):
        self.set_focus_zoom(enabled and self.focus_roi is not None)
        if enabled:
            self.zoom_button.pack(side='right')
        else:
            self.zoom_button.pack_forget()

    def set_focus_zoom(self, zoom):
        self.focus_zoom = zoom
        self.zoom_button.configure(text='返回全图' if zoom else '放大选区',
                                   state='normal' if self.focus_roi is not None else 'disabled')

    def toggle_focus_zoom(self):
        self.set_focus_zoom(not self.focus_zoom)
        self.render(focusing=True)

    def render(self, focusing=False):
        canvas = self.canvas
        width, height = canvas.winfo_width(), canvas.winfo_height()
        canvas.coords(self.text_id, width // 2, height // 2)
        canvas.itemconfigure(self.text_id, width=max(1, width - 16))
        if self.frame is None or min(width, height) < 5:
            return
        from PIL import Image, ImageTk

        roi = self.focus_roi if focusing else None
        zoom = roi is not None and self.focus_zoom
        image = Image.fromarray(self.frame)
        self.source_box = (0., 0., 1., 1.)
        if zoom:
            iw, ih = image.size
            x0, y0, x1, y1 = roi
            left, top = int(x0 * iw), int(y0 * ih)
            right = max(left + 1, int(x1 * iw))
            bottom = max(top + 1, int(y1 * ih))
            image = image.crop((left, top, right, bottom))
            self.source_box = (left / iw, top / ih, right / iw, bottom / ih)
        scale = min((width - 4) / image.width, (height - 4) / image.height)
        display_width = max(1, round(image.width * scale))
        display_height = max(1, round(image.height * scale))
        image = image.resize((display_width, display_height), Image.Resampling.BILINEAR)
        self.photo = ImageTk.PhotoImage(image)
        left, top = (width - display_width) // 2, (height - display_height) // 2
        self.image_box = (left, top, display_width, display_height)
        canvas.coords(self.image_id, left, top)
        canvas.itemconfigure(self.image_id, image=self.photo)
        canvas.itemconfigure(self.text_id, text='')
        for kind, box in (('focus', None if zoom else roi), ('exposure', self.exposure_roi)):
            if self.drag_kind != kind:
                self.draw_box(kind, box)

    def point(self, x, y, clamp=False):
        """Map a canvas point to the full image, accounting for padding and zoom."""
        if self.image_box is None:
            return None
        left, top, width, height = self.image_box
        x, y = (x - left) / width, (y - top) / height
        if not clamp and not (0 <= x <= 1 and 0 <= y <= 1):
            return None
        x0, y0, x1, y1 = self.source_box
        return (x0 + max(0, min(1, x)) * (x1 - x0),
                y0 + max(0, min(1, y)) * (y1 - y0))

    def drag_box(self, x, y):
        if self.drag_start is None or self.image_box is None:
            return None
        start_x, start_y = self.drag_start
        end_x, end_y = self.point(x, y, clamp=True)
        return min(start_x, end_x), min(start_y, end_y), max(start_x, end_x), max(start_y, end_y)

    def draw_box(self, kind, roi):
        rect = self.rects[kind]
        if roi is None or self.image_box is None:
            self.canvas.itemconfigure(rect, state='hidden')
            return
        left, top, width, height = self.image_box
        x0, y0, x1, y1 = self.source_box
        a, b, c, d = roi
        a, b, c, d = max(a, x0), max(b, y0), min(c, x1), min(d, y1)
        if a > c or b > d:
            self.canvas.itemconfigure(rect, state='hidden')
            return
        self.canvas.coords(rect,
                           left + (a - x0) * width / (x1 - x0),
                           top + (b - y0) * height / (y1 - y0),
                           left + (c - x0) * width / (x1 - x0),
                           top + (d - y0) * height / (y1 - y0))
        self.canvas.itemconfigure(rect, state='normal')
