"""Generate the numbered benchmark report from the archived E002 outputs."""
import csv
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.utils import ImageReader
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen.canvas import Canvas
from reportlab.platypus import Paragraph

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
OUT = HERE / "results"
SIM = ROOT / "simulation/E002/results"
pdfmetrics.registerFont(TTFont("Report", "/Library/Fonts/Arial Unicode.ttf"))
W, H = landscape(A4)
N = 5
INK = "#153046"
SUB = "#42586B"
ACCENT = "#08796E"
PALE = "#E9F4F1"
LINE = "#D7E2E9"

def readcsv(path):
    with path.open(encoding="utf-8-sig") as f:
        return list(csv.DictReader(f))

reg = readcsv(OUT / "E002-02.csv")
seg = readcsv(OUT / "E002-01.csv")
sim = readcsv(SIM / "E002-01.csv")
c = Canvas(str(HERE / "E002.pdf"), pagesize=(W, H), pageCompression=1)
c.setTitle("E002 多视角配准、去盘与实测补全")
c.setSubject("旋转台结构光扫描 | 八组配准与补全对照")

def text(x, top, s, size=11, color=INK):
    c.setFillColor(colors.HexColor(color))
    c.setFont("Report", size)
    c.drawString(x, H-top-size, s)

def para(x, top, width, s, size=11, color=SUB, leading=None):
    p = Paragraph(s, ParagraphStyle("body",fontName="Report",fontSize=size,
        textColor=colors.HexColor(color),wordWrap="CJK",leading=leading or size*1.55))
    _, height = p.wrap(width, H)
    p.drawOn(c, x, H-top-height)
    return height

def line(x, top, end):
    c.setStrokeColor(colors.HexColor(LINE))
    c.setLineWidth(.65)
    c.line(x,H-top,end,H-top)

def panel(x, top, w, h, fill=PALE):
    c.setFillColor(colors.HexColor(fill))
    c.roundRect(x,H-top-h,w,h,8,stroke=0,fill=1)

def frame(number, title, note):
    text(34,21,"旋转台结构光扫描    /    E002",10,ACCENT)
    text(34,40,title,28)
    text(35,81,note,11,SUB)
    line(34,106,W-34)
    line(34,H-34,W-34)
    text(35,H-28,"E002   •   同期标定尺度 mm*   •   2026.11.25",8.5,SUB)
    text(W-58,H-28,f"{number} / {N}",8.5,SUB)

def fig(path, x, top, w, h):
    img = ImageReader(str(path))
    iw, ih = img.getSize()
    scale = min(w/iw,h/ih)
    ww, hh = iw*scale, ih*scale
    c.drawImage(img,x+(w-ww)/2,H-top-hh,ww,hh,mask="auto")

def item(x, top, width, title, desc):
    text(x,top,title,12,INK)
    return 19+para(x,top+19,width,desc,10.5)

def table(x,top,widths,headers,rows,rh=26):
    total=sum(widths)
    panel(x,top,total,rh,"#EAF1F6")
    for i,row in enumerate([headers,*rows]):
        y=top+i*rh
        if i:
            line(x,y+rh,x+total)
        offset=x
        for j,value in enumerate(row):
            width=widths[j]
            para(offset+6,y+7,width-11,str(value),9.25,INK if i==0 or j==0 else SUB,10.5)
            offset+=width

def get(rows, sid, method, test=None):
    return next(v for v in rows if v["id"]==f"E002-S{sid:02d}"
                and v["method"]==method and (test is None or v["test"]==str(test)))

def percent(number):
    return f"{100*float(number):.1f}%"

def val(row,key,precision=2):
    return f"{float(row[key]):.{precision}f}" if row.get(key) else "—"

# 1. Dataset and result overview
frame(1,"八组全部复跑","小黄人、苹果、充电头与纸盒   /   原始单视角、同期标定、完整扫描场景")
panel(34,122,305,74)
text(48,134,"7 种去盘    ·    11 种位姿路线",16,ACCENT)
para(48,161,278,f"覆盖 59 个单视角。完成 {len(reg)} 项配准及 {len(sim)} 项补全对照。",10.5)
labels=["小黄人","苹果","纸盒 11/16","充电头 17:16","充电头 17:41","纸盒 17:45","纸盒 17:47","纸盒 17:50"]
notes=["F1 40.5% → 52.7%","0°/360° F1 99.2%","回零 F1 28.4%","深度 1.72 → 1.49","深度 P50 1.35","轴心留一角 P95 4.51","深度 6.37 → 3.44","深度 4.26 → 3.54"]
rows=[]
for i,(label,note) in enumerate(zip(labels,notes),1):
    r=next(v for v in reg if v["id"]==f"E002-S{i:02d}" and v["selected"]=="True")
    rows.append((f"S{i:02d}",label,r["method"],note))
table(34,210,[39,83,43,140],["编号","对象","位姿","关键实测"],rows,27)
para(34,465,303,"F1 的匹配阈值为 2 mm*。表中深度是留出视角误差 P50。回零依靠重复实拍直接核验。",10.4)
para(34,506,303,"小黄人每约 45° 一帧，眼镜和手臂提供可辨认的对应。苹果和 11/16 纸盒约每 90° 一帧。",10.35)
fig(OUT/"figures/E002-10.png",353,119,454,389)
para(359,510,439,"相邻两幅依次是全场与去盘后实测点。颜色区分角度，各窗分别取景。",9.7)
c.showPage()

# 2. Protocol and paper evidence
frame(2,"Benchmark｜先校验标定，再评价拼接","固定 Gray code 光学输入与去盘掩膜，比较位姿；用另外的留出数据检查误对齐")
left=[
("01  标定与覆盖","检查最大方位缺口、邻边是否齐全和独立边的闭环。现代组用留出 ArUco 重投影 P50 与转轴留一角，早期组用 0°/360° 回零实拍。"),
("02  相邻视角","每帧按 24 px 图块分训练和留出；报告 1/2/4 mm* 的双向 F1 和最差邻边、内点 RMSE、接缝法向 P95、双向近邻均值和 P95。"),
("03  整圈一致性","逐一留出整个参考帧，用别的视角预测 8×8 px 栅格深度。联合报告支持率、绝对误差 P50、前方冲突、后方遮挡和有效一致比例。"),
("04  去盘与补全","16 张白场轮廓验证前景 F1、近盘带召回。补全用人工遮挡区的已测点验证召回和 P95；体积同时报告闭合性与形状假设敏感度。"),
]
for i,(title,desc) in enumerate(left):
    item(35,126+86*i,374,title,desc)
line(35,465,406)
apple_icp=get(reg,2,"R10")
para(35,474,367,f"苹果串联点到面 ICP（R10）的深度 P50 为 {val(apple_icp,'depth_abs_P50')} mm*，方位缺口由 90° 扩至 {val(apple_icp,'max_gap',0)}°。把整圈覆盖加进评分能识别这种塌缩。",10)
para(35,513,367,"同批开发集上选用路线。留出误差表示重复或视图一致性。",9.5)
panel(425,119,382,417,"#F1F5F7")
sources=[
("秦琴等  ·  2024","第 53–54 页的圆环平面与轨迹求轴，第 55 页式 12 与第 56 页表 3–4。对应 R02 转轴基线及 R09–R11 三种 ICP。论文 fitness 分母与本轮双向 F1 定义各自保留。"),
("朱贺勇  ·  2022","第 38–44 页的转轴初值及翻转补扫，第 45–47 页的 Poisson 与参考 Chamfer，第 65 页的 ADD。对应 C05 实测 Poisson 对照和遮挡区验证。"),
("李怀泽  ·  旋转平台多视角三维重建","第 35–37 页的棋盘角点轨迹与重复标定，第 43–44 页的强重叠边优先及平面退化，第 45–47 页有初值对照。对应 R06 法向门控的分批位姿图。"),
]
for i,(title,desc) in enumerate(sources):
    item(443,137+99*i,344,title,desc)
line(443,432,790)
para(443,441,342,'<link href="https://qikan.cmes.org/zzyzdh/CN/10.3969/j.issn.1009-0134.2024.09.008" color="#08796E">秦等原文 DOI</link>　·　<link href="https://www.open3d.org/docs/latest/tutorial/pipelines/multiway_registration.html" color="#08796E">Open3D 多视角配准</link>　·　<link href="https://docs.opencv.org/4.13.0/df/d4a/tutorial_charuco_detection.html" color="#08796E">OpenCV ChArUco</link>',10)
para(443,475,343,"朱、李为用户提供全文。页码按原文页脚。脚本和指标字典随实验同目录交付。",10)
c.showPage()

# 3. Segmentation
frame(3,"去盘｜保留贴近盘面的物体点","先拟合每帧盘面，再使用连通性、跨视角支持和白场边界，最后处理贴盘区域")
fig(OUT/"figures/E002-11.png",34,124,505,192)
fig(OUT/"figures/E002-13.png",34,322,505,221)
panel(557,124,250,181)
text(572,136,"M07 综合选用",16,ACCENT)
fg=[r for r in seg if r["method"]=="M07"]
f1=sum(float(r["f1"]) for r in fg)/len(fg)
recall=sum(int(r["low_tp"]) for r in fg)/sum(int(r["low_n"]) for r in fg)
para(572,162,218,f"""16 帧前景 F1 为 {percent(f1)}，近盘带召回为 {percent(recall)}。
    统一高度阈值 M01 分别为 91.5% 和 28.7%。""",11)
para(572,224,219,"M06 在近盘带微强。偏心纸盒启用 M04 跨视角支持，改善这一组的盘面泄漏。",11)
table(557,320,[47,203],["编号","方法"],[
    ("M01","统一高度阈值"),("M02","逐帧盘面和自适应高度"),("M03","加三维连通域"),
    ("M04","加跨视角支持"),("M05","加白场 GrabCut"),("M06","恢复近盘点"),("M07","恢复近盘点，偏心时切换 M04")],24)
c.showPage()

# 4. Pose diagnosis
frame(4,"纸盒｜形状贴合和标定一致性要一起看","重复平面容易滑移，低覆盖视角会放大误差。独立码点可识别部分只改善外观的候选")
fig(OUT/"figures/E002-12.png",34,120,370,382)
para(34,508,371,"每行用同一批实测点和取景范围。R07 仅用盒面，R08 加训练 ArUco，留出码独立评估。",9.8)
rows=[]
for sid,m in [(3,"R01"),(6,"R01"),(6,"R07"),(7,"R01"),(7,"R06"),(7,"R07"),(8,"R01"),(8,"R08")]:
    r=get(reg,sid,m)
    rows.append((f"S{sid:02d} {m}",val(r,"depth_abs_P50"),val(r,"box_surface_P95"),val(r,"tag_P50")))
table(418,122,[109,93,93,94],["组别","深度 P50","盒面 P95","留出码 px"],rows,23)
item(418,341,386,"S03｜回零一致性低","四视角间隔约 90°，0°/360° 匹配 28.4%。42.07 mm* 的投影误差含跨面及遮挡。")
item(418,395,386,"S06 / S07｜依据留出码选位姿","S06 的轴心留一角 P95 为 4.51 mm*，R07 增加留出码偏差。S07 偏心约 49 mm*，R06 降低深度误差，留出码稳定在约 10.2 px。")
item(418,467,386,"S08｜R08 的深度与标记折中","V00 与 V05 轮廓内有效深度是 80.0% 与 26.9%。R08 深度 P50 降 17%，留出码从 4.97 升至 6.13 px。")
c.showPage()

# 5. Completion
frame(5,"补全｜估计看不见的面","八组都生成候选模型。橙色采样点在 2.5 mm* 内缺少实测近邻")
fig(SIM/"figures/E002-10.png",34,124,448,344)
fig(SIM/"figures/E002-11.png",492,122,315,127)
text(505,256,"遮掉已测片区后，再检查恢复",13)
box_ids=(3,6,7,8)
box_wins=sum(float(get(sim,s,"C04",1)["hidden_P95"])<float(get(sim,s,"C03",1)["hidden_P95"]) for s in box_ids)
off_base=val(get(sim,7,"C03",1),"hidden_P95")
off_shape=val(get(sim,7,"C04",1),"hidden_P95")
para(505,280,292,f"{box_wins}/4 组纸盒用长方体可降低遮挡区 P95。偏心组 C03 / C04 的 P95 分别是 {off_base} / {off_shape} mm*。苹果椭球恢复 54.2%，P95 为 13.61 mm*。",10.35)
table(499,351,[114,92,101],["完整输入 T0","形状先验","约束高度场"],[
    ("S02 苹果","329.0","438.9"),("S08 纸盒","260.1","265.5")],24)
para(505,432,284,"体积单位 cm*³。按每 1,000 mm*³ 换算。真实底面依赖补扫或标准件核验。",10)
para(38,482,445,"C01 平底凸包　C02 平底高度场　C03 轮廓与深度约束<br/>C04 长方体　C05 补法向 Poisson　C06 苹果椭球",10.5)
para(505,477,285,"58/66 项通过闭合检查。Poisson 的八个网格保留供查看，体积栏留空。",10.35)
para(38,532,435,'<link href="https://cs233.stanford.edu/papers/volrange/" color="#08796E">深度射线约束参考｜Curless 与 Levoy</link>',8.5)
c.showPage()
c.save()
