"""Build the Chinese benchmark report from the experimental outputs."""
import csv
import json
from pathlib import Path
import re
from statistics import median

from reportlab.lib.colors import HexColor, white
from reportlab.lib.utils import ImageReader
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/"output/calibrated-benchmark"
FIG=OUT/"figures"
PDF=ROOT/"output/pdf/多视角点云拼接_标定联合Benchmark.pdf"
W,H=1000,635
INK="#132943"
MUTED="#516981"
BLUE="#28587f"
TEAL="#0a796c"
LINE="#dce6ed"
BG="#f3f7fb"
ORANGE="#9b4e29"
COLORS=["#2e6fc4","#ec6a35","#249e74","#9c5aac","#d19e32","#1b9db5","#d85389","#647184"]
SOURCES={
    "icp":("Open3D · ICP 与指标","https://www.open3d.org/docs/latest/tutorial/pipelines/icp_registration.html"),
    "cv":("OpenCV · 标定、PnP 与重投影","https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html"),
    "aruco":("OpenCV · ArUco","https://docs.opencv.org/4.13.0/d5/dae/tutorial_aruco_detection.html"),
    "gray":("OpenCV · 格雷码","https://docs.opencv.org/4.13.0/d1/dec/classcv_1_1structured__light_1_1GrayCodePattern.html"),
    "charuco":("OpenCV · ChArUco","https://docs.opencv.org/4.13.0/df/d4a/tutorial_charuco_detection.html"),
    "global":("Open3D · 全局配准","https://www.open3d.org/docs/latest/tutorial/pipelines/global_registration.html"),
    "robust":("Open3D · 鲁棒核","https://www.open3d.org/docs/latest/tutorial/pipelines/robust_kernels.html"),
    "gicp":("Segal 等 · RSS 2009","https://www.roboticsproceedings.org/rss05/p21.html"),
    "colored":("Park 等 · ICCV 2017","https://openaccess.thecvf.com/content_ICCV_2017/papers/Park_Colored_Point_Cloud_ICCV_2017_paper.pdf"),
    "fgr":("Zhou 等 · ECCV 2016","https://vladlen.info/papers/fast-global-registration.pdf"),
    "graph":("Open3D · 多视角配准","https://www.open3d.org/docs/latest/tutorial/pipelines/multiway_registration.html"),
}
METHODS = {
 "axis_step":("转轴 + 指令角","软硬件","固定转轴，按每次下发的 45° 将视角逆转回第 0 帧。",
              "从 0、2、4、6 帧的 ArUco 位姿拟合公共轴及轴心。各训练帧角度一起拟合，融合时使用电机指令角。",
              "每帧直接回到参考帧，运行快，便于定位传动问题。",
              "指令角与实际位置有偏差。充电头第 1 帧的标记估角约 35°，指令为 45°。","cv"),
 "aruco":("ArUco 多标记 PnP","软硬件","从相机图像直接恢复盘面对相机的位姿。",
           "提取白场 ArUco 角点。用一个 20 mm 黑框定标，联合拟合其余可见标记的盘面坐标与各帧位姿。通过共享盘面把各帧转到第 0 帧。",
           "充电头和三组盒子都能给出独立于物体形状的位姿。测试码误差小。",
           "依赖角点可见性与相机内参。三角化深度畸变仍会在点云上留下接缝。","aruco"),
 "axis_refine":("固定轴 · 一维角度配准","软硬件","锁定旋转轴与轴心，只给每个视角一个角度自由度。",
                "从指令角起步，搜索 ±18°。在相邻帧重叠区用最接近 65% 的双向距离优化，加入角度偏移惩罚。全圈交替更新 3 轮。",
                "限制不合理的轴向位移，较易解释每一帧为何移动。",
                "真实缺陷包含深度变形和台面姿态差，单靠改角度会把几何误差吸收到转角。","icp"),
 "point_to_point":("点到点 ICP","软硬件","由 ArUco 给初值，寻找最近点并最小化点间欧氏距离。",
                   "三个体素尺度依次收紧匹配门限。全圈按采集顺序累计相邻变换，另估末帧到首帧的边做闭环诊断。",
                   "四组数据合看，环上 F1@2 位居前列，充电头接缝距离明显减小。",
                   "平面、缺面和深度畸变会诱导切向滑动。偏心盒的留出码偏差达千像素量级。","icp"),
 "point_to_plane":("点到面 ICP · Huber","软硬件","以 ArUco 为初值，把点拉向目标局部切平面。",
                   "法线半径约为当前体素的 3 倍，细层至少 2.4 mm*。沿法线最小化残差，Huber 核临界值是当前距离门限的 1/3。",
                   "充电头有效边较可靠，独立整圈边的角度漂移比点到点小。",
                   "平行平面难以约束面内滑动。盒子缺面时，有些边连训练支持门槛也达不到。","icp"),
 "tukey_icp":("点到面 ICP · Tukey","软硬件","在点到面 ICP 上把大残差的权重逐渐压到零。",
              "沿用 ArUco、点到面法线与三尺度流程。每层 Tukey 截止设为当前距离门限的 0.75 倍。",
              "充电头局部重影得到压缩，孤点和深度飞点对解的拉扯更小。",
              "真实重叠很少或深度偏差大时，大残差也可能来自真实表面，鲁棒核会丢掉这些约束。","robust"),
 "gicp":("广义 ICP · GICP","软硬件","利用两侧局部表面协方差对距离加权。",
         "以 ArUco 定位，三个分辨率逐步优化。协方差正则 ε=0.001，使用 Huber 核降低离群点权重。",
         "接缝与环上 F1 接近点到点，充电头独立闭环角小。",
         "低重叠和错误平面匹配仍会拉动位姿。四组留出码误差普遍远高于 ArUco。","gicp"),
 "gray_icp":("灰度 ICP","软硬件","用几何和白场灰度一起估计局部变换。",
             "以 ArUco 为初值，白场强度复制到 Open3D 的三通道。使用 Colored ICP，几何权重 0.968。",
             "有稳定字样或纹理时，可给平面内滑动增加约束。",
             "单色投影、阴影和反光随视角变动。充电头与偏心盒均出现过低支持边。","colored"),
 "fpfh_ransac":("FPFH + RANSAC","软件配准","用局部 3D 形状特征找初值，再用点到面 ICP 精配准。",
                "在 2.2 mm* 体素上提取 FPFH。抽 3 对特征估计刚体，检查边长和距离，最多 24,000 次。成功边从几何初值精配准。",
                "即便物体初始姿态未知，也能在有辨识度的几何上给出候选。",
                "相似侧面能给出高几何分但朝向错误。充电头独立闭环角达数十度。失败边才使用标记位姿。","global"),
 "fpfh_fgr":("FPFH + FGR","软件配准","用快速全局优化同时求刚体变换、压低错配权重。",
             "复用 FPFH 和 3.74 mm* 匹配半径。FGR 输出作为点到面 ICP 初值，失败时回退到 ArUco。",
             "全局候选生成快，适合作为有可靠外部约束时的候选路径。",
             "本批数据的局部特征重复且缺面，失败边多，部分整圈结果实际由回退位姿主导。","fgr"),
 "pose_graph":("ArUco 锚定的位姿图","软硬件","以每帧 ArUco 为锚，同时使用通过门控的局部配准边。",
               "取间隔 1 或 2 帧的点到面边。训练双向支持至少 16%，相对锚点的改动小于 15° 和物体中心 12 mm* 才入图。锁定第 0 帧，小步联合调整。",
               "能在充电头上略降接缝，同时守住留出码。差边多时，图自然停留在 ArUco 位姿附近。",
               "居中、偏心盒没有通过门控的边。硬件位姿可以维持，单帧缺失与畸变仍应从采集解决。","graph"),
}
SIDS=["174129","174531","174747","175006"]
CASES=["充电头","居中盒","偏心盒","17:50 盒"]
pdfmetrics.registerFont(TTFont("ZH","/Library/Fonts/Arial Unicode.ttf"))
C=None


def t(x,y,s,size=10,col=INK,font="ZH",right=False):
    C.setFont(font,size)
    C.setFillColor(HexColor(col))
    if right: C.drawRightString(x,H-y,str(s))
    else: C.drawString(x,H-y,str(s))


def box(x,y,w,h,fill=BG,stroke=None,r=0):
    C.setFillColor(HexColor(fill))
    if stroke: C.setStrokeColor(HexColor(stroke))
    if r: C.roundRect(x,H-y-h,w,h,r,fill=1,stroke=bool(stroke))
    else: C.rect(x,H-y-h,w,h,fill=1,stroke=bool(stroke))


def rule(x,y,w):
    C.setStrokeColor(HexColor(LINE)); C.setLineWidth(.65); C.line(x,H-y,x+w,H-y)


def para(x,y,s,w,size=11,leading=18,col=MUTED):
    tokens=re.findall(r"[A-Za-z0-9.+@/*%=°±_-]+|.",s)
    cur=""
    for tok in tokens:
        test=cur+tok
        if pdfmetrics.stringWidth(test,"ZH",size)>w and cur:
            t(x,y,cur.rstrip(),size,col)
            y+=leading
            cur=tok.lstrip()
        else:
            cur=test
    if cur:
        t(x,y,cur.rstrip(),size,col)
    return y+leading


def picture(path,x,y,width,height=None):
    im=ImageReader(str(path))
    w,h=im.getSize()
    height=height if height else width*h/w
    C.drawImage(im,x,H-y-height,width,height,mask="auto")
    return height


def footer(n,keys=()):
    rule(44,601,912)
    t(44,619,"远距离结构光测量  /  多视角拼接实测",9,MUTED)
    x=350
    for key in keys:
        label,url=SOURCES[key]
        t(x,619,label,9,BLUE)
        ww=pdfmetrics.stringWidth(label,"ZH",9)
        C.linkURL(url,(x,H-622,x+ww,H-609),relative=0,thickness=0)
        x+=ww+18
    t(955,619,f"{n:02} / 14",9,MUTED,right=True)
    C.showPage()


def heading(k,title,sub):
    t(44,38,k,10,TEAL)
    t(43,79,title,31)
    t(44,105,sub,11,MUTED)


def card(x,y,w,label,value,sub):
    box(x,y,w,66,BG,r=8)
    t(x+13,y+17,label,10,MUTED)
    t(x+13,y+42,value,22,TEAL)
    t(x+13,y+58,sub,8.8,MUTED)


def val(r,k,scale=1,digits=1,empty="—"):
    return f"{scale*float(r[k]):.{digits}f}" if r.get(k) not in (None,"") else empty


def cover(models,rows):
    held=[x for sid,m in models.items() for x in m["statistics"] if sid!="174129" or x["view"] not in (1,2,4,5)]
    original,after=[median(x["table_reprojection_p50_p95"][i][0] for x in held) for i in (0,1)]
    heading("01  数据与标定","多视角点云拼接实测","把光学标定、转台位姿和配准放在同一套 Benchmark 里")
    card(44,123,291,"原始视角","32 / 32","四组 ArUco 扫描，重建并融合全部唯一视角")
    card(355,123,291,"整圈比较","11 套","2 视角 + 8 视角，统一输入与留出评测")
    card(666,123,290,"盘面投影残差",f"{original:.0f} → {after:.1f} px","28 个未参与投影拟合视角的视角内 P50 再取中位数")
    t(44,224,"匹配项目时间线",16)
    history=[
        ("11/16","格雷码弯贴转台","早期手动 90°。同位 0°/360° 档案的双向中位距离为 3.43 / 3.55 mm*。"),
        ("11/20","手动多物体","另一套光学参数，单独用于历史检查。小黄人有 45° 系列与 360° 重复。"),
        ("11/23 17:16","棋盘格贴转台","用当次 17:11 光学参数和 SIFT 试验轴复跑。合并 8 帧，1 帧仅 17 个点，6 条邻边可打分。"),
        ("11/23 17:41–17:50","ArUco + 自动 45°","由 17:41 组恢复投影模型，然后冻结参数应用到其余 3 组。每组重新从盘面角点估位姿。"),
        ("11/24","采集开发测试","目录中只有测试记录或少量采集，没有可用于回填 11/23 的成套标定参数。")
    ]
    for i,(day,name,desc) in enumerate(history):
        yy=251+42*i
        t(44,yy,day,10,TEAL); t(157,yy,name,11,INK)
        para(157,yy+16,desc,408,9.5,14)
        if i!=4: rule(44,yy+23,525)
    t(590,224,"同一批解码像素的投影残差",15)
    picture(FIG/"optics.png",583,237,371,105)
    para(590,369,"发现  17:41 起盘面对应点的投影横坐标突变，部分最高位正反图几乎相同。对这批扫描按低半区解码，舍去 x/y 最低位，以编码区间中心三角化。",354,11,17)
    para(590,436,"恢复  用充电头 01、02、04、05 四帧的盘面与物体极线约束重估投影参数，相机内参沿用 17:11 XML。原 XML 联合标定 RMS 4.25 px。",354,11,17)
    box(44,500,912,67,"#eaf5f3",r=8)
    t(59,520,"本次结论",10,TEAL)
    para(59,542,"优先保留 ArUco 位姿。几何 ICP 能提高表面重合，但全圈会偏离留出角点。下一轮先提高单帧深度一致性，再让点云细调通过位姿约束。",876,13,18,INK)
    footer(1,("cv","gray"))


def benchmark():
    heading("02  评测协议","Benchmark 怎么看","同批原图、同一套裁剪，统一第 0 帧坐标。四组分别打分，再比较跨组中位数。")
    data=[
      ("01","输入质量","逐视角记录白黑场通过量、解码量、三角化通过量与物体有效点。每 4 像素采样，白黑差 ≥15、有效位正反差 ≥5，两端重投影误差 <8 px。"),
      ("02","重叠与接缝","距离门限 1/2/4 mm*。双向最近点命中比例取调和平均得 F1。再报 2 mm* 内点 RMSE、全距离 P50/P95、互为最近点的法向接缝 P95。"),
      ("03","位姿独立复核","每组留出一个 ArUco ID。只用第 0 帧及其余码恢复测试码的盘面坐标，预测其余视角四角位置。报告 P50/P95 像素误差，附与 PnP 的转角与角点位移。"),
      ("04","最差边与闭环","整圈检查 8 条相邻边，含 07→00。报边 F1 的中位和最差值。可独立估满 8 条边时，连乘得到闭环角及轴心处位移。缺边记空。"),
      ("05","稳定性与耗时","记录失败边、回退、有效视角和配准耗时。两视角再测试绕轴 ±5°、±12°、±25° 的初值扰动。耗时不含共用预处理和写文件。"),
    ]
    for i,(n,label,body) in enumerate(data):
        yy=145+77*i
        box(44,yy-16,27,21,"#e7f3ef",r=4)
        t(51,yy-1,n,9,TEAL)
        t(83,yy,label,14)
        final=para(83,yy+19,body,460,10.5,15)
        assert final<=yy+65
        rule(44,yy+51,499)
    x=571
    box(x,128,385,186,BG,r=8)
    t(x+15,153,"公平比较的三个固定项",14,TEAL)
    para(x+15,178,"同一输入  每帧先按白黑场、格雷码置信度及重投影筛选，再做物体框、盘面高度和稀疏点剔除。",353,10.5,16)
    para(x+15,221,"留出像素  24 px 图块固定留 20%。光学参数优化与点云配准都用训练块，几何指标用留出块。",353,10.5,16)
    para(x+15,265,"统一设置  双视角 01→00；全圈 00–07。ICP 的体素为 2.4 / 1.25 / 0.75 mm*，配准门限为 8 / 4 / 2.5 mm*。",353,10.5,16)
    t(x,340,"初值扰动对两视角配准的影响",14)
    picture(FIG/"stress.png",x,351,385,141)
    para(x,510,"图中每点取四组两视角实验的中位数，合并正负扰动。在 ArUco 初值上施加角度扰动。未通过的配对仍保留在明细中。",381,9.4,14)
    box(44,544,912,35,"#fff6eb",r=6)
    t(56,559,"读分顺序  先看留出码与失效边，再比较 F1 与接缝。F1 衡量相邻点云重合。整物覆盖率需有独立参考表面。",10.1,ORANGE)
    t(56,573,"mm* 为档案 20 mm 码边定义的标定尺度。07→00 是跨圈相邻边。",9.7,ORANGE)
    footer(2,("icp","cv","graph"))


def algorithm_page(n,key,descriptor,table):
    name,category,principle,operation,advantage,limit,source=descriptor
    r=table["174129",key]
    heading(f"算法 {n-2:02}  /  11",name,"共同输入  ·  17:41 充电头原始 8 视角  /  重估后的相机–投影仪模型")
    box(843,47,113,29,"#e6f4f0",r=6)
    t(858,66,category,10,TEAL)
    card(44,124,291,"两视角 F1@2 mm*",val(r,"pair_f1_2",100)+"%","01 → 00，蓝橙两色" + (" · 回退到标记" if r["pair_fallback"]=="True" else ""))
    card(355,124,291,"全圈 F1@2 mm*",val(r,"f1_2",100)+"%",f"{r['views']} / 8 帧，8 条相邻边取中位")
    card(666,124,290,"留出码 P50",val(r,"tag_p50_px")+" px","越低越贴近实拍标记角点")
    t(44,218,"融合点云",15)
    t(44,237,"上排：两视角    下排：8 视角    同方向使用同一比例",9.4,MUTED)
    picture(FIG/"174129"/f"{key}.png",44,244,605,259)
    for i,color in enumerate(COLORS):
        xx=47+i*71
        box(xx,514,8,8,color,r=2)
        t(xx+14,522,f"{i:02} / {i*45}°",8.5,MUTED)
    box(668,205,288,320,BG,r=8)
    cursor=225
    for title,body in (("原理",principle),("操作",operation),("优势",advantage),("不足",limit)):
        t(683,cursor,title,11,TEAL)
        cursor=para(683,cursor+16,body,256,10.5,15)+11
    if key=="pose_graph":
        t(683,509,"双视角图为入图前的局部边，整圈图为锚定结果。",9,MUTED)
    rule(44,537,912)
    t(44,556,"充电头 · 整圈诊断",10,TEAL)
    closure=val(r,"closure_deg",empty="—")
    closure=closure+"°" if closure!="—" else closure
    t(44,574,f"F1@1/4  {val(r,'f1_1',100,0)}% / {val(r,'f1_4',100,0)}%    RMSE@2 {val(r,'rmse_2',1,2)} mm*    接缝 P95 {val(r,'seam_p95',1,2)} mm*    独立闭环角 {closure}",9.3,MUTED)
    t(955,556,"另外三组 · 整圈 F1@2 mm*",10,TEAL,right=True)
    t(955,574,"    ".join(f"{case} {val(table[sid,key],'f1_2',100,0)}%" for case,sid in zip(CASES[1:],SIDS[1:])),9.3,MUTED,right=True)
    footer(n,(source,))


def summary(table):
    heading("14  结果总览","先核对位姿，再比点云贴合","四组都使用标定后的同一批单视角点云。每组单独打分，表中再对四组取中位数。")
    cols=[44,74,287,358,427,507,578,650]
    headings=["#","配准方案","双帧 F1","全圈 F1","最差边 F1","测试码","失败边","配准耗时"]
    for x,label in zip(cols,headings):
        t(x,145,label,9,TEAL)
    rule(44,152,676)
    for i,(key,(name,*_)) in enumerate(METHODS.items()):
        yy=171+22*i
        rows=[table[sid,key] for sid in SIDS]
        if key in ("aruco","pose_graph"): box(44,yy-14,677,22,"#e8f4f0",r=3)
        mm=lambda k:median(float(r[k]) for r in rows)
        columns=[f"{i+1:02}",name,f"{mm('pair_f1_2')*100:.1f}%",f"{mm('f1_2')*100:.1f}%",
                 f"{mm('f1_2_worst')*100:.1f}%",f"{mm('tag_p50_px'):.1f} px",
                 str(sum(int(r['edge_failures']) for r in rows)),f"{mm('seconds'):.2f} s"]
        for x,entry in zip(cols,columns): t(x+2,yy,entry,9.5)
        if key not in ("aruco","pose_graph"): rule(44,yy+7,676)
    t(740,144,"两个指标一起看",13)
    picture(FIG/"tradeoff.png",723,151,242,140)
    para(740,319,"横轴越靠左，留出码越准。纵轴越高，点云越贴合。圆点编号对应左侧方案。",209,10,15)
    para(740,374,"串接算法在训练双向支持 <3% 或求解失败时回退。位姿图另有门控，四组入图边数为 5 / 0 / 0 / 1。",209,9.3,14)
    box(44,421,912,146,BG,r=8)
    points=[
        ("现阶段基线","保留每视角 ArUco 位姿。充电头留出码 P50 为 5.7 px。锚定位姿图把 F1 从 28.0% 提至 29.4%，码偏差为 5.4 px。"),
        ("几何分不能单独决定方案","点到点 ICP 与 FPFH + RANSAC 的跨组全圈 F1 都约 44%，留出码分别约 283 和 716 px。优先检查相似平面上的错配。"),
        ("下一轮采集的判定条件","固定相机与投影仪后，用实测格距的 ChArUco 做多倾角标定。盘上保留可见码，补拍同位 0°/360° 和独立尺寸件，逐层核对深度尺度、重复性和整圈位姿。")
    ]
    for i,(title,body) in enumerate(points):
        yy=446+i*40
        t(57,yy,title,11,TEAL)
        para(229,yy,body,705,11,16)
    t(44,586,"F1 门限为 2 mm*。耗时为 Python 配准阶段，未计共同的解码、光学校正和盘面标定。",9,MUTED)
    footer(14,("charuco",))


def main():
    global C
    rows=list(csv.DictReader((OUT/"summary.csv").open()))
    table={(r["session"],r["method"]):r for r in rows}
    models=json.loads((OUT/"recovery/model.json").read_text())
    C=canvas.Canvas(str(PDF),pagesize=(W,H),pageCompression=1)
    C.setTitle("多视角点云拼接 · 标定联合 Benchmark")
    C.setAuthor("结构光测量实验")
    cover(models,rows)
    benchmark()
    for i,(key,descriptor) in enumerate(METHODS.items(),3):
        algorithm_page(i,key,descriptor,table)
    summary(table)
    C.save()
    print(PDF)


if __name__=="__main__":
    main()
