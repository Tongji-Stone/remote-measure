"""Generate vector ArUco markers for a turntable, printed at 100%."""
from pathlib import Path
import cv2
import numpy as np
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas

OUTPUT = Path(__file__).with_name("转台ArUco标记_A4_20mm与15mm.pdf")
pdfmetrics.registerFont(TTFont("YaHei", r"C:\Windows\Fonts\msyh.ttc", subfontIndex=0))
D = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
W, H = A4
c = canvas.Canvas(str(OUTPUT), pagesize=A4)
c.setTitle("Turntable ArUco markers 4x4_50")
def label(x,y,text,size=10):
    c.setFont("YaHei",size);c.drawString(x*mm,y*mm,text)
label(15,279,"转台 ArUco 标记 · A4 实际尺寸打印",15)
label(15,268,"4×4_50 字典；每组 ID 0～3，不要混用两种尺寸。",10)
label(15,259,"以黑色外框边长为标记尺寸；保留周围白边。",10)
for row,(side,y) in enumerate([(20,213),(15,158)]):
    label(15,y+side+9, f"{side} mm 黑框边长（ID 0、1、2、3）",11)
    for marker_id in range(4):
        x=25+marker_id*42
        bits=cv2.aruco.generateImageMarker(D,marker_id,60,borderBits=1)
        # 4x4 data + one black border cell on each side = 6x6 modules.
        c.setFillColorRGB(1,1,1);c.rect((x-4)*mm,(y-4)*mm,(side+8)*mm,(side+8)*mm,fill=1,stroke=0)
        c.setFillColorRGB(0,0,0)
        for rr in range(6):
            for cc in range(6):
                if bits[rr*10+5,cc*10+5] < 128:
                    c.rect((x+cc*side/6)*mm,(y+(5-rr)*side/6)*mm,side/6*mm,side/6*mm,fill=1,stroke=0)
        label(x,y-9,f"ID {marker_id}",9)
label(15,143,"打印：A4、实际大小 / 100%，关闭适合页面。",10)
label(15,135,"标记粘贴在转台，不粘贴在待测物体；保持平整且留出白边。",10)
label(15,127,"下方标尺长度应为 50 mm，黑框边长也应实际测量。",10)
c.setLineWidth(.4);c.line(20*mm,110*mm,70*mm,110*mm)
for x in [20,70]: c.line(x*mm,107*mm,x*mm,113*mm)
label(37,101,"50 mm",10)
c.save();print(OUTPUT)
