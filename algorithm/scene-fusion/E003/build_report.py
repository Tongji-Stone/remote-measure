"""Build the six-page E003 experiment report from measured CSV and rendered meshes."""
import csv
import json
from pathlib import Path
from statistics import mean

from reportlab.lib import colors
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.utils import ImageReader
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen.canvas import Canvas
from reportlab.platypus import Paragraph

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
AG=HERE/"results"
SG=ROOT/"simulation/E003/results"
AF=AG/"figures"
SF=SG/"figures"
pdfmetrics.registerFont(TTFont("cn","/Library/Fonts/Arial Unicode.ttf"))
W,H=landscape(A4)
c=Canvas(str(HERE/"E003.pdf"),pagesize=(W,H),pageCompression=1)
c.setTitle("E003 多视角拼接、白色网格与局部补全")
c.setAuthor("remote-measure")
Ink="#173347"
Muted="#466073"
Teal="#117569"
Sky="#EAF2F6"
Tint="#E7F3F0"
Rule="#D6E3EA"


def rd(path):
    with path.open(encoding="utf-8-sig") as f:
        return list(csv.DictReader(f))


index=json.loads((AG/"E003-05.json").read_text())
reg=rd(AG/"E003-02.csv")
noise=rd(AG/"E003-01.csv")
edges=rd(AG/"E003-03.csv")
ppf=rd(AG/"E003-04.csv")
model=rd(SG/"E003-01.csv")
diag=json.loads((SG/"E003-02.json").read_text())
feet=json.loads((SG/"E003-S01/E003-S01-02.json").read_text())["local_completion"]["0"]["photo_support"]


def label(x,y,s,size=11,color=Ink):
    c.setFillColor(colors.HexColor(color))
    c.setFont("cn",size)
    c.drawString(x,H-y-size,s)


def paragraph(x,y,w,s,size=10.75,color=Muted,leading=None):
    p=Paragraph(s,ParagraphStyle("p",fontName="cn",fontSize=size,wordWrap="CJK",
                 leading=leading or size*1.55,textColor=colors.HexColor(color)))
    _,h=p.wrap(w,H)
    p.drawOn(c,x,H-y-h)
    return h


def rule(x,y,end):
    c.setStrokeColor(colors.HexColor(Rule))
    c.setLineWidth(.65)
    c.line(x,H-y,end,H-y)


def block(x,y,w,h,fill=Sky):
    c.setFillColor(colors.HexColor(fill))
    c.roundRect(x,H-y-h,w,h,7,stroke=False,fill=True)


def header(i,title,summary):
    label(34,17,"旋转台结构光扫描   /   E003",10,Teal)
    label(34,36,title,27)
    label(35,78,summary,11,Muted)
    rule(34,104,W-34)
    rule(34,H-32,W-34)
    label(35,H-25,"E003   •   同期标定尺度 mm*   •   2026.11.25",8.5,Muted)
    label(W-62,H-25,f"{i} / 6",8.5,Muted)


def table(x,y,widths,head,rows,step=24):
    block(x,y,sum(widths),step)
    for i,row in enumerate([head,*rows]):
        start=x
        for j,val in enumerate(row):
            paragraph(start+6,y+i*step+6,widths[j]-10,str(val),9.4,Ink if i==0 or j==0 else Muted,11)
            start+=widths[j]
        if i:
            rule(x,y+(i+1)*step,x+sum(widths))


def image(path,x,y,w,h):
    i=ImageReader(str(path))
    a,b=i.getSize()
    s=min(w/a,h/b)
    c.drawImage(i,x+(w-a*s)/2,H-y-b*s,a*s,b*s,mask="auto")


def titled(x,y,w,name,desc):
    label(x,y,name,12)
    return paragraph(x,y+20,w,desc,10.7)


def regrow(tag,route=None):
    return next(r for r in reg if r["id"]==tag and ((route is None and r["selected"]=="True") or r["route"]==route))


def meshrow(tag,test,kind):
    return next(r for r in model if r["id"]==tag and r["test"]==str(test) and r["method"]==kind)


def ft(x,n=2):
    return f"{float(x):.{n}f}" if x not in ("",None) else "—"


def pct(x,n=1):
    return f"{100*float(x):.{n}f}%"


header(1,"从原始视角到白色表面","八组实测数据，59 个单视角。配准、三角网格、缺失面估计逐段保存。")
phases=[
 ("01 光学输入","Gray code 深度<br/>同期轴与相机"),
 ("02 前处理","去盘<br/>体素与离群去噪"),
 ("03 粗配准","转台初值<br/>特征候选另测"),
 ("04 精配准","三尺度点到面<br/>约束位姿图"),
 ("05 白色网格","相机定向法向<br/>screened Poisson"),
 ("06 局部模型","补底与规则化<br/>独立模拟输出"),
]
for i,(name,description) in enumerate(phases):
    x=34+131*i
    block(x,121,118,64,Tint if i in (2,3) else Sky)
    label(x+7,126,name,11.2,Teal)
    paragraph(x+7,147,106,description,9.3,Muted,12)
rows=[]
for t,v in index.items():
    only=regrow(t);base=regrow(t,"R01")
    depth=ft(base["depth_abs_P50"],3)
    if only["route"]!="R01":
        depth+=" → "+ft(only["depth_abs_P50"],3)
    rows.append((t[-3:],v["label"],only["route"],depth,ft(only["max_gap"],1)+"°"))
table(34,211,[39,113,52,131,59],["编号","采集组","选用","留出深度 P50","方位缺口"],rows,29)
titled(459,207,339,"照论文区分两种配准",
    "同一摆放时转台给可靠初值。人工翻转后相对姿态未知，再用 ISS、FPFH 和 SAC-IA 找初值。E003 现有八组是单次摆放，特征路线在实际邻角上做对照。")
titled(459,293,339,"评价同时检查深度和位姿",
    "使用固定 24 px 留出图块、整帧深度投影、最大角度缺口、法向接缝和独立码点。八组共 "+str(len(reg))+" 项配准候选。点云导出保持刚体。")
titled(459,380,339,"白色网格与后处理各有文件",
    "M01 显示有实测支持的表面。纸盒规则化、局部充电头平面与脚蹼等补全从同一批去噪点启动。")
rule(459,469,802)
paragraph(459,478,341,"论文入口｜图 3.1、§3.4–3.7、§4.2–4.6。报告使用印刷页码，流程映射与式号见实验目录。",10)
c.showPage()

header(2,"去噪｜小成分与贴盘点","同一融合位姿下比较四种掩膜。验证使用 16 张手工轮廓与已测深度。")
image(AF/"E003-10.png",34,119,520,295)
stat=[]
for method,title in (("D00","E002 去盘"),("D01","统计过滤"),("D02","密度 + 连通"),("D03","RANSAC 盘 + 统计")):
    data=[r for r in noise if r["filter"]==method and r["f1"]]
    stat.append((method,title,pct(mean(float(r["f1"]) for r in data),2),
        f'{sum(int(r["fp"]) for r in data):,}',
        pct(sum(int(r["low_tp"]) for r in data)/sum(int(r["low_n"]) for r in data),2)))
panel_rows=[(s[0],s[2],s[3],s[4]) for s in stat]
table(557,121,[42,65,75,67],["路线","前景 F1","背景点","近盘召回"],panel_rows,31)
titled(557,290,246,"D02 用于精配准与白色网格",
   "D02 从 59 帧去掉 3,734 点。它按跨视角连通性剔除远处和很小的成分，贴盘召回比统计过滤高约 2.1 个百分点。")
titled(35,436,497,"D03 跑了论文第 4.2 节的基本顺序",
   "原始近盘环点体素化 → RANSAC 求盘面 → 在现有去盘点内加盘距判断 → 统计过滤。D03 压低背景残留，同时减少较多贴盘细节。")
titled(557,436,245,"标注覆盖的背景点只报实数",
   "背景点是 16 张参考中的保留数。近盘召回统计离盘 −4 至 +8 mm* 的实测前景。")
c.showPage()

header(3,"粗配准｜ISS、FPFH、SAC-IA 与 PPF","以真实相邻视角测试特征初值。带真实转台约束的求姿控制错误重叠。")
image(AF/"E003-12.png",34,116,773,265)
e=[r for r in edges if r["feature_accepted"]]
n=len(e);h=sum(bool(r["sac_score"]) for r in e); q=sum(r["feature_accepted"]=="True" for r in e)
positive=[r for r in ppf if r["test"]=="known_transform"]
small=sum(r["ADD_match"] and float(r["ADD_match"])<.1 for r in positive)
block(35,408,372,120,Tint)
titled(48,420,344,"ISS + FPFH + SAC-IA 式抽样",
    f"54 对里 {h} 对有粗配准候选，{q} 对通过物理门控。每次采三个有间距的 ISS 点，从相近的 FPFH 候选取配对，再用双向截断距离评价及 ICP 精配准。")
block(429,408,377,120,Sky)
titled(442,420,348,"第 4 章 PPF 投票和聚类",
    f"已知变换控制里 {small}/8 组精配准 ADD 小于 0.1 mm*。真实邻角仍可能观测不同平面，11/16 实例相对标定位姿差约 92°。")
c.showPage()

header(4,"纸盒｜十字突出怎样形成","理想几何验证了内移会出十字。本批 11/16 实测还存在宽度、面法向与位姿的共同误差。")
image(AF/"E003-11.png",34,114,772,304)
old=diag["E003-S03"]
d=old["axis_sensitivity_fixed_geometry"]
best=min(float(v["envelope_side_P95"]) for k,v in d.items() if k!="base")
block(34,437,375,103,Sky)
titled(46,447,350,"11/16｜固定几何下的轴心平移试验",
    f"角端突出 P95 为 {ft(old['heldout_E003']['max_face_protrusion_P95'])} mm*。轴心平移 ±4 mm*，最佳整体侧壁 P95 仍是 {best:.2f} mm*，基准 {d['base']['envelope_side_P95']:.2f} mm*。侧面也有倾斜。")
block(427,437,380,103,Tint)
titled(439,447,356,"17:47｜真实配准有改善",
    "R07 联合试共享轴心和平滑转角微调。留出深度 P50 3.436 → 3.226 mm*，F1@2 mm* 21.7% → 26.0%。平整盒面与锐角由模拟网格 M04 单独输出。")
c.showPage()

header(5,"白色三角网格｜实测面与完整候选","首列 Poisson 实测面。中列为局部补全或纸盒规则化，末列展示同一模型的底面。")
image(SF/"E003-10.png",34,113,774,403)
paragraph(35,504,765,"8 个完整候选通过闭合校验。M01 保持数据支持，适合检查扫描接缝、丢面和端口。纸盒提供硬边 OBJ；形状保持的补全模型用白色 PLY。",10.35)
c.showPage()

header(6,"缺失面｜保留实物特征再补局部","留出点来自真实扫描。照片指引补全另用一张未参与拟合的白场图验证。")
image(SF/"E003-11.png",34,116,480,219)
rows=[]
for j,tag in enumerate(index,1):
    choice="M04" if j in (3,6,7,8) else "M03"
    ref=meshrow(tag,1,"M01")
    alt=meshrow(tag,1,choice)
    rows.append((tag[-3:],choice,pct(ref["hidden_recall_2"]),pct(alt["hidden_recall_2"]),ft(alt["hidden_P95"])))
table(533,120,[42,49,54,63,60],["采集","补全","M01 召回","补全召回","P95 mm*"],rows,26)
titled(35,350,478,"小黄人｜原照片支持脚蹼轮廓",
    f"V00/V01 给两个脚蹼近盘投影，V07 可见区域覆盖 {pct(feet['heldout_visible_area_agreement'])}。留出切片 IoU 为 M03 51.5%、照片引导 M05 51.2%、硬扣轮廓 M06 44.6%。")
titled(35,427,478,"苹果｜沿实测轮廓向下续接",
    "按方位追踪较低的曲面边界，经射线冲突检查后补面。保留上部果梗凹窝。")
titled(533,376,270,"充电头｜只修有支撑的对向面",
    "两组留出局部角差 5.48° → 0.03°、17.01° → 0.64°。输入点修正限 2.3 mm*，配准点云照原坐标保存。")
para='留出召回以点到三角面 ≤2 mm* 计。小黄人 T1 用 M03，白场脚蹼另做验证。'
paragraph(533,478,270,para,10,Muted)
text='<link href="https://gfx.cs.princeton.edu/pubs/Kazhdan_2020_PSR/index.php" color="#117569">Poisson 包络</link>　·　<link href="https://graphics.stanford.edu/~niloy/research/globFit/globFit_sigg11.html" color="#117569">GlobFit</link>　·　<link href="https://docs.opencv.org/4.13.0/d9/d25/group__surface__matching.html" color="#117569">OpenCV PPF</link>'
paragraph(35,526,465,text,9.5)
c.showPage()
c.save()
