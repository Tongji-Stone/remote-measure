"""Generate separate reports for the Minion, apple, and time-matched carton datasets."""
import csv
import json
from pathlib import Path
from statistics import median

from reportlab.pdfgen import canvas
import build_calibrated_benchmark_pdf as pdf

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/"output/object-benchmarks"
MODERN=ROOT/"output/calibrated-benchmark"
BASE=pdf.METHODS
TITLES={"minion":"小黄人","apple":"苹果","box":"纸盒"}
SESSIONS=("174531","174747","175006")
BOX_NAMES=("居中 · 17:45","偏心 · 17:47","17:50 组")
NEW={
 "axis_step":("档案转轴 + 名义角","软硬件","用同一上午的转台轴与录入角度，把每个相机点云逆转回基准帧。",
              "从标定目录读取轴和轴心。绕轴旋转时加上保持轴心不动的平移 c−Rc。以 00 为共同原点。","cv"),
 "rim_axis":("盘缘修正转轴","软硬件","盘面转动时外圆保持不动，可用外圆几何修正轴心。",
             "继承 11/20 的轴方向。在档案盘面 ±10 mm* 范围内分扇区提取外缘点，鲁棒拟合圆心。各视角的圆心取中位数。融合仍使用名义角。","cv"),
 "axis_refine":("固定轴 · 一维角度配准","软硬件","锁住档案轴和轴心，只调整每帧的绕轴角。",
               "名义角附近搜索 ±18°。用邻帧双向距离中较近的 65% 优化角度，对大偏移加惩罚。整圈交替更新 3 轮。","icp"),
 "pose_graph":("转台锚定的位姿图","软硬件","同时使用转台预测和筛选后的局部点到面边。",
               "相隔一或两帧的邻边先做点到面 ICP。双向支持达 16%，相对先验小于 15° 和 12 mm* 才入图。锁住第 0 帧，仅小步改动位姿。","graph"),
}
NATIVE_TEXT={
 "minion":{
 "axis_step":("保持八帧角度顺序，给出可复核的基线。","轴心偏差放大为整圈边缘重影，最差邻边贴合很少。"),
 "rim_axis":("盘缘修正后最差边和整圈重合都有明显提升，点云仍保持台面坐标约束。","盘面转轴方向沿用档案，逐帧实际角仍未单独测得。"),
 "axis_refine":("只调一个角度，容易排查转台角误差。","档案轴心已有偏差，调整角度对整圈改善很小。"),
 "point_to_point":("有最高的整圈中位 F1，局部表面容易被拉近。","最差边几乎失配。走完 360° 后只剩约 1% 重合。"),
 "point_to_plane":("接缝收紧，整圈最差边仍能维持一定重合。","眼镜、手臂及暗区局部缺点，跨圈重复帧仍有可见残差。"),
 "tukey_icp":("轻微提高最差边和跨圈重复的重合，削弱零散点的影响。","降权依赖距离门限，强反光和暗区缺点仍留在输入。"),
 "gicp":("整圈重合、最差边和跨圈复测都表现较稳。独立八边连乘的角漂移较小。","跨圈预测仍低于 0°/360° 原位直接对照，精度还受档案轴与深度影响。"),
 "gray_icp":("眼镜和身体的灰度边界提供了一部分面内约束。","灰度来自投影白场，暗色腰带与高亮眼镜导致跨视角外观变化。"),
 "fpfh_ransac":("在多处凸起和弧面可以找到有用的匹配初值。","独立闭环角偏大，邻边证据不足以支撑全圈使用。"),
 "fpfh_fgr":("首对可以找到较好的局部解，耗时短。","三条整圈边失败，回退后的最差边仍接近零。"),
 "pose_graph":("有 11 条边通过门控，整圈位姿保留在档案转台附近。","强锚把不准确的轴心带入结果，闭环复测表现弱。双帧图展示入图前的局部边。"),
 },
 "apple":{
 "axis_step":("保持 90° 的物理视角顺序，便于定位哪两帧缺重叠。","视角间隔大，档案轴下邻边重叠很低。"),
 "rim_axis":("调整盘心后双帧和整圈重合均提升，同时维持名义视角顺序。","苹果曲面和反光造成缺点，90° 邻边仍分离，当前结果适合做采集基线。"),
 "axis_refine":("角度与位移的耦合受固定轴约束，偏移可以直接读出。","几何角度优化把真实外观差吸收入角度，留出盘面码边偏差增大。"),
 "point_to_point":("能明显增加表面重合，首对匹配较多。","近似圆的果体允许错误朝向。跨圈预测和留出盘面码显示较大漂移。"),
 "point_to_plane":("部分相邻面被拉近，可减少局部间隙。","首对回退；约 90° 的取样间隔让局部切面缺少方向证据。"),
 "tukey_icp":("降权会压制较远的伪对应。","相邻可用点少，真实的几何约束也会被降权。首对回退。"),
 "gicp":("协方差可区分不同方向的点位置不确定性。","近圆曲面可沿切向滑动，独立闭环角与重复帧验证都较弱。"),
 "gray_icp":("在灰度稳定区域能补充几何约束。","白场亮斑位置变化，首对回退。光照差异使灰度难以作可靠的位姿参照。"),
 "fpfh_ransac":("整圈 F1 与跨圈重合较高，能找出几何上相似的果面。","盘面码位置偏差很大，圆润果面之间有朝向歧义。"),
 "fpfh_fgr":("首对有可用的几何候选。","整圈多条候选边失效，保留的低覆盖率反映回退后的全局结果。"),
 "pose_graph":("不满足位姿门限的边保持在图外。","没有边通过门控，全圈仍靠档案转轴。双帧图展示入图前的局部边。"),
 }}
MODERN_TEXT={
 "axis_step":"转台标定给出清晰的物理初值。偏心盒会放大角度误差，17:50 组的留出码误差尤其明显。",
 "aruco":"三组测试码的 P50 为 5.0–10.6 px。固定物体坐标的依据最清楚，单帧缺面与翘曲仍保留在融合结果。",
 "axis_refine":"只调角度可提高部分组的邻边重合。纸盒的平面残差、偏心放置和遮挡会把角度拉偏。",
 "point_to_point":"居中盒表面重合明显升高。偏心盒留出码超过 1,000 px，典型的平面错配。",
 "point_to_plane":"有足够几何支持的边可以压低法向误差。偏心盒许多边只看见同类平面，得分低或回退。",
 "tukey_icp":"对少量离群点有抑制作用。弱平面重叠下，硬截止会一起压掉一部分真实对应。",
 "gicp":"三组环上 F1 较高。纸盒平面间的错配仍可产生高分，留出码复核显示位姿漂移。",
 "gray_icp":"印刷文字提供部分区分信息。纸盒阴影与白场反射随视角变化，几何与亮度组合仍会失败。",
 "fpfh_ransac":"不同朝向的相似侧面形成几何高分，三组留出码均显示较大偏差。",
 "fpfh_fgr":"搜索快。重复面和缺面导致较多配对退回标记初值，应逐边查看回退记录。",
 "pose_graph":"居中和偏心组均无边通过门控，保留 ArUco 位姿。17:50 组有一条边通过，对全圈重合改变很小。",
}


def load(p):
    with p.open() as f: return {r["method"]:r for r in csv.DictReader(f)}


def line(x,y,label,body,w=866):
    pdf.t(x,y,label,11,pdf.TEAL)
    pdf.para(x+124,y,body,w-124,10.6,16)


def pct(row,key):
    if row.get(key) in ("",None): return "—"
    return pdf.val(row,key,100)+"%"


def unit(row,key,suffix):
    return pdf.val(row,key)+suffix if row.get(key) not in ("",None) else "—"


def native_cover(key,meta):
    d=OUT/key
    item=TITLES[key]
    count=len(meta["angles"])-1
    step=int(meta["angles"][1])
    pdf.heading("01  数据与标定",f"{item} · 多视角拼接",
               f"11 月 20 日 {meta['capture'][11:16]} 采集 / 逐视角原始格雷码 / {step}° 名义间隔")
    pdf.card(44,124,291,"独立融合视角",f"{count} 帧","另有一帧实拍 360° 同位复测")
    pdf.card(355,124,291,"11/20 相机–投影仪联合标定",f"{meta['rms_px']:.2f} px","09:44–09:48 棋盘格采集的档案 RMS")
    r=meta["repeat_same_place"]
    pdf.card(666,124,290,"0° 与 360° 原位重合",f"{r['f1_2']*100:.1f}%","F1@2 mm*，未用配准调整")
    pdf.t(44,224,"原始白场与物体区域",16)
    pdf.picture(d/"input.jpg",44,239,584,195 if key=="minion" else 113)
    y=460 if key=="minion" else 376
    pdf.para(44,y,"黄色为物体有效点，蓝点为盘面棋盘条带中心。每个视角先解码和三角化，再进行统一裁剪。",584,10.5,16)
    if key=="minion":
        pdf.para(44,y+47,"角度按 corrected_nominal_angles.json 配对。0003 的目录缺失，0004–0008 顺移对应 135°–315°，0009 对应 360°。",584,11,16)
    else:
        pdf.para(44,y+48,"0000–0003 对应 0°、90°、180°、270°。0004 是单独采集的 360° 重复，留作闭环验证。",584,11,16)
        pdf.para(44,y+96,"果面接近旋转对称，且局部高光明显。后面的页面用盘面条带和同位复测一起检查几何匹配。",584,11,16)
    pdf.box(652,210,304,345,pdf.BG,r=8)
    y=236
    for label,content in [
        ("光学标定", "用 20261120_analysis 目录的 10 mm 标定 XML。采集配置为 1280×720 投影与 44 帧格雷码。"),
        ("转台标定", "用当天 09:35–09:40 的条带中心和圆拟合档案估算转轴。采集角度为录入值。"),
        ("原位复测", f"0° / 360° 双向最近距离 P95 为 {r['p95']:.2f} mm*。该值反映同一名义方位下的实测可重复性。"),
        ("尺度", "mm* 为档案棋盘格设定的标定尺度。镜面与遮挡会影响单帧有效点。")]:
        pdf.t(666,y,label,12,pdf.TEAL)
        y=pdf.para(666,y+19,content,275,10.5,16)+12
    pdf.footer(1,("cv","gray"))


def native_bench(key,meta):
    d=OUT/key
    n=len(meta["angles"])-1
    checks=max(0,len(meta["marker_centers"])-1-(int(str(n) in meta["marker_centers"])))
    pdf.heading("02  评测协议","Benchmark 怎么打分",f"双帧 01→00，整圈 {n} 个不同角度。每种方法共享同一批点云和像素划分。")
    explanations=[
       ("光学输入","每 4 像素采样。白黑场差 ≥15、格雷码有效位正反差 ≥5，忽略每轴最低位。三角化两端重投影残差 <8 px。"),
       ("表面贴合","每 5 个固定的 24 px 图块留 1 块。双向最近点统计 F1@1/2/4 mm*、RMSE@2、全距离 P50/P95 和接缝法向 P95。"),
       ("整圈稳定性",f"汇总 {n} 条邻边含末帧→首帧，取中位数与最差值。独立边齐全时，连乘得到闭环旋转及轴心处位移。"),
       ("盘面独立检查",f"物体优化避开盘面条带。用白场可见的棋盘条带中心复核 {checks} 个视角间的投影偏差。单点中心是近似刻度。"),
       ("实拍 360° 检查","将最后一视角到 360° 的独立边接在估计路径末端，预测 360° 位姿；对留出块与 00 打分。360° 从融合点云中留出。")]
    for i,(title,body) in enumerate(explanations):
        y=149+76*i
        pdf.t(44,y,f"{i+1:02}",11,pdf.TEAL)
        pdf.t(76,y,title,14)
        pdf.para(76,y+20,body,463,10.7,16)
        pdf.rule(44,y+51,482)
    pdf.box(556,130,400,166,pdf.BG,r=8)
    pdf.t(571,156,"固定设置",14,pdf.TEAL)
    pdf.para(571,179,"体素 2.4 / 1.25 / 0.75 mm*，匹配门限 8 / 4 / 2.5 mm*。训练双向支持低于 3% 的边触发回退。",369,10.6,16)
    pdf.para(571,226,"原位 00↔360 基准独立统计。由轴模型直接给出的整圈中心位姿，无法提供独立的跨圈边估计，表中用横线表示。",369,10.6,16)
    pdf.t(556,330,"对绕轴初值扰动的敏感度",14)
    pdf.picture(d/"figures/stress.png",556,342,400,177)
    pdf.para(556,535,"正负扰动合并。求解失败或边支持过低记 0，配准耗时另计。",396,9.6,15)
    pdf.footer(2,("icp","cv","graph"))


def common_native_description(method):
    if method in NEW:
        return NEW[method]
    title,cat,principle,operation,_,_,source=BASE[method]
    principle=principle.replace("由 ArUco 给初值","由转台模型给初值").replace("以 ArUco 为初值","以转台模型为初值").replace("以 ArUco 定位","由转台模型定位")
    operation=operation.replace("沿用 ArUco、","沿用档案转台初值、").replace("以 ArUco 为初值","以转台模型为初值").replace("回退到 ArUco","回退到转台预测")
    return title,cat,principle,operation,source


def method_page_native(key,n,method,row,meta):
    d=OUT/key
    name,cat,principle,operation,source=common_native_description(method)
    plus,minus=NATIVE_TEXT[key][method]
    pdf.heading(f"算法 {n-2:02}  /  11",name,f"{TITLES[key]} · 11/20 原始数据 / {row['frames']} 个唯一视角")
    pdf.box(837,48,119,29,"#e5f3ef",r=6)
    pdf.t(852,67,cat,10,pdf.TEAL)
    pdf.card(44,122,291,"双帧 F1@2 mm*",pct(row,"pair_f1_2"),
             "01→00 · 已回退到转台" if row["pair_fallback"]=="True" else "01→00 · 留出块")
    pdf.card(355,122,291,"整圈 F1@2 mm*",pct(row,"f1_2"),f"{row['frames']} 条邻边取中位")
    pdf.card(666,122,290,"跨圈预测 F1@2 mm*",pct(row,"repeat_f1_2"),"用末帧到实拍 360° 的独立边")
    pdf.t(44,216,"融合点云",16)
    pdf.t(44,235,f"上排两视角，下排全部 {row['frames']} 视角。同方向使用同一比例。",9.6,pdf.MUTED)
    pdf.picture(d/"figures"/f"{method}.png",44,242,605,259)
    small_legend(meta["angles"][:-1],513)
    pdf.box(669,203,287,330,pdf.BG,r=8)
    cursor=224
    for tag,body in (("原理",principle),("操作",operation),("优势",plus),("不足",minus)):
        pdf.t(683,cursor,tag,11,pdf.TEAL)
        cursor=pdf.para(683,cursor+17,body,257,10.8,16)+12
    pdf.rule(44,543,912)
    pdf.t(44,562,"整圈诊断",10,pdf.TEAL)
    pdf.t(44,581,f"最差边 {pct(row,'worst_2')}    内点 RMSE {unit(row,'rmse_2',' mm*')}    接缝 P95 {unit(row,'seam_p95',' mm*')}    盘面码 {unit(row,'marker_px',' px')}    闭环角 {unit(row,'ring_closure_deg','°')}",10,pdf.MUTED)
    pdf.footer(n,(source,))


def small_legend(angles,y):
    for k,a in enumerate(angles):
        x=48+70*k
        pdf.box(x,y,8,8,pdf.COLORS[k],r=2)
        pdf.t(x+14,y+8,f"{int(a)}°",9,pdf.MUTED)


def native_summary(key,data,meta):
    d=OUT/key
    item=TITLES[key]
    pdf.heading("14  结果总览",f"{item} · 拼接结果对比","整圈分数、最差边与独立复核共同判断。所有距离用相同标定尺度。")
    head=("#","配准方案","双帧 F1","整圈 F1","最差边","360° F1","盘面码","失败边")
    xs=(44,70,268,332,397,456,515,579)
    for x,l in zip(xs,head): pdf.t(x,145,l,9.3,pdf.TEAL)
    for i,(m,r) in enumerate(data.items()):
        y=171+22*i
        if m in ("rim_axis","gicp") if key=="minion" else m=="rim_axis":
            pdf.box(44,y-15,604,22,"#e9f5f1",r=4)
        line_values=(f"{i+1:02}",common_native_description(m)[0],pct(r,"pair_f1_2"),pct(r,"f1_2"),
                     pct(r,"worst_2"),pct(r,"repeat_f1_2"),unit(r,"marker_px"," px"),r["failures"])
        for x,l in zip(xs,line_values): pdf.t(x+1,y,l,9.7)
        pdf.rule(44,y+7,604)
    pdf.t(674,147,"整圈与实拍复测",14)
    pdf.picture(d/"figures/outcomes.png",664,155,292,153)
    p=meta["repeat_same_place"]
    pdf.para(674,333,f"同位直比 0° / 360°\u3000 F1 {100*p['f1_2']:.1f}%，最近距离 P95 {p['p95']:.2f} mm*。表中 360° F1 来自沿整圈传播的位姿。",282,10.3,15)
    notes=[
       ("最佳折中","盘缘修正作为台面基线。若要收紧形状，GICP 在最差边和 360° 复测上更稳。"),
       ("关键证据","GICP 整圈 F1 为 52.6%，最差边 34.6%，360° F1 为 57.5%。点到点 ICP 的一条邻边几乎完全错开。"),
       ("下一轮采集","保留 45° 分度和 360° 重复，给转台加可见标记。尽量补低视角，以覆盖眼镜下沿、手臂和脚。")
    ] if key=="minion" else [
       ("当前基线","用盘缘修正维持四个视角的物理顺序。现有 90° 邻边缺少足够约束，建议优先增加 45° 中间视角。"),
       ("关键证据",f"FPFH + RANSAC 整圈 F1 为 {pct(data['fpfh_ransac'],'f1_2')}，盘面条带偏差为 {unit(data['fpfh_ransac'],'marker_px',' px')}。近圆曲面仍有朝向歧义。"),
       ("下一轮采集","保留条带或增加多 ID 标记，补拍果梗和另一高度的视角。以同位置 0° / 360° 验证动作重复性。")
    ]
    pdf.box(44,420,912,144,pdf.BG,r=8)
    for i,(title,body) in enumerate(notes): line(58,447+i*41,title,body)
    pdf.t(44,585,"F1 门限统一为 2 mm*。留出数据、位姿、计时和失败边见同目录明细。",9,pdf.MUTED)
    pdf.footer(14,("icp",))


def modern_data():
    pack={}
    with (MODERN/"summary.csv").open() as f:
        for r in csv.DictReader(f):
            if r["session"] in SESSIONS:
                pack[r["session"],r["method"]]=r
    return pack


def box_cover(old,models):
    pdf.heading("01  数据与标定","纸盒 · 多视角拼接","11/16 长方体与 11/23 三组包装盒按各自的标定和转台记录配对")
    pdf.card(44,122,291,"11/16 长方体","4 + 1","四个唯一视角与一帧实拍 360°")
    pdf.card(355,122,291,"11/23 包装盒","3 × 8","居中、偏心及 17:50，三组均融合全部 8 帧")
    pdf.card(666,122,290,"11/23 转台配置","ArUco","每组重新估盘面位姿，步进指令间隔 45°")
    pdf.t(44,223,"四组原始白场",16)
    pdf.picture(OUT/"box/input.jpg",44,239,912,194)
    for i,(title,body) in enumerate((
       ("11/16 17:40","配 11/16 16:09 起的棋盘格采集所得 10 mm XML，联合 RMS 1.27 px。转轴配 17:35 格雷码条带标定，档案假设轴平行相机 Y。"),
       ("11/23 17:45–17:50","配 11/23 17:11 的相机内参和 17:41 充电头重估的投影模型。投影参数随后冻结。各组 ArUco 重新解算。"),
       ("采集配对规则","11/20 的 0.96 px 标定用于小黄人和苹果。两个纸盒时期按各自设备状态处理，分别保留独立评分。"))):
        line(44,468+41*i,title,body,910)
    pdf.footer(1,("cv","aruco","gray"))


def box_bench(old,models):
    pdf.heading("02  评测协议","Benchmark 与时间配对","先检验单帧光学输入，再验位姿与整圈贴合。两个时期分别打分。")
    pdf.box(44,130,440,204,pdf.BG,r=8)
    pdf.box(500,130,456,204,"#eaf6f3",r=8)
    pdf.t(60,158,"11/16 长方体 · 四帧整圈",14,pdf.TEAL)
    pdf.para(60,183,"由 44 张原始图重建每帧，裁取箱体。0°/360° 原位重合 F1@2 为 26.1%，距离 P95 为 8.11 mm*。",406,11,17)
    pdf.para(60,236,"独立 360° 帧留出。转轴用历史参数，选用训练边支持更好的旋向。缺少可稳定追踪的留出码。",406,11,17)
    pdf.t(516,158,"11/23 包装盒 · 每组八帧整圈",14,pdf.TEAL)
    vals=[median(v["table_reprojection_p50_p95"][1][0] for v in models[s]["statistics"]) for s in SESSIONS]
    pdf.para(516,183,f"重估后的盘面投影残差中位数，居中 / 偏心 / 17:50 组分别为 {vals[0]:.1f} / {vals[1]:.1f} / {vals[2]:.1f} px。",420,11,17)
    pdf.para(516,236,"每组留出一个 ArUco ID，预测它在其余帧的角点位置，报告 P50/P95。07→00 参与邻边诊断。",420,11,17)
    for i,(title,body) in enumerate((
       ("双帧与全圈","统一展示 01→00 及所有唯一视角。11/16 为 00–03 四帧，11/23 每组为 00–07 八帧。"),
       ("统一几何指标","固定留出 20% 的 24 px 图块。双向 F1@1/2/4 mm*，内点 RMSE、P50/P95、法向接缝、失败边与最差边。"),
       ("闭环及位置复核","独立边齐全时报告连乘的闭环角与轴心位移。11/16 用实拍重复帧作独立验证，11/23 用留出 ArUco 复核每帧。"),
       ("尺度与耗时","mm* 为同期档案标定尺度。配准时间单独统计。整物覆盖率需要参考表面，各组指标明细保留到边。"))):
        line(44,369+49*i,title,body,912)
    pdf.footer(2,("icp","cv","graph"))


def box_method(n,method,md,old):
    name,cat,principle,operation,_,_,source=BASE[method]
    if method=="axis_step":
        operation="11/23 用 0、2、4、6 帧标记拟合轴，再按电机指令角融合。11/16 用当日格雷码条带轴与手动名义角。"
    if method=="pose_graph":
        operation="将同时间段的位姿设为锚，加入通过训练支持和位姿门控的点到面边。锁住 00 帧。双帧图展示入图前的单独边。"
    r=md["174531",method]
    previous=old.get(method)
    pdf.heading(f"算法 {n-2:02}  /  11",name,"11/23 居中盒 · 17:45 为主图 / 11/16 长方体独立对照")
    pdf.box(837,48,119,29,"#e5f3ef",r=6)
    pdf.t(852,67,cat,10,pdf.TEAL)
    pdf.card(44,119,291,"居中 · 双帧 F1@2",pct(r,"pair_f1_2"),
             "01→00 · 回退到 ArUco" if r["pair_fallback"]=="True" else "01→00")
    pdf.card(355,119,291,"居中 · 8 帧 F1@2",pct(r,"f1_2"),"8 条邻边取中位数")
    pdf.card(666,119,290,"居中 · 留出码 P50",unit(r,"tag_p50_px"," px"),"与真实拍到的角点比对")
    pdf.t(44,209,"11/23 · 上排两帧，下排 8 帧",13)
    pdf.picture(MODERN/"figures/174531"/f"{method}.png",44,217,599,247)
    pdf.rule(44,471,599)
    pdf.t(44,489,"11/16 · 单独重建",12)
    pdf.t(642,489,"局部配准用该次档案转轴作初值",9,pdf.MUTED,right=True)
    if previous:
        pdf.picture(OUT/"legacy-box/figures"/f"strip_{method}.png",44,499,599,94)
    else:
        pdf.para(44,519,"该组为格雷码条带标定，未采集适用于多 ArUco PnP 的盘面码。",585,11,17)
    pdf.box(662,199,294,392,pdf.BG,r=8)
    cursor=222
    for title,body in (("原理",principle),("操作",operation),("表现与限制",MODERN_TEXT[method])):
        pdf.t(677,cursor,title,11,pdf.TEAL)
        cursor=pdf.para(677,cursor+17,body,265,10.6,16)+13
    pdf.rule(677,435,264)
    pdf.t(677,453,"其他 11/23 组 · 全圈 F1 / 留出码",10.5,pdf.TEAL)
    for j,(sid,label) in enumerate(zip(SESSIONS[1:],BOX_NAMES[1:])):
        q=md[sid,method]
        pdf.t(677,472+j*19,f"{label}    {pct(q,'f1_2')} / {unit(q,'tag_p50_px',' px')}",10.1)
    pdf.t(677,525,"11/16 长方体 · 独立统计",10.5,pdf.TEAL)
    if previous:
        pdf.t(677,546,f"双帧 {pct(previous,'pair_f1_2')}   整圈 {pct(previous,'f1_2')}   360° {pct(previous,'repeat_f1_2')}",10.1)
        pdf.t(677,565,f"闭环角 {unit(previous,'ring_closure_deg','°')}   失败边 {previous['failures']}",10.1,pdf.MUTED)
    else:
        pdf.t(677,547,"缺少本方法的采集输入",10.1,pdf.MUTED)
    pdf.footer(n,(source,))


def box_summary(md,older):
    pdf.heading("14  结果总览","纸盒 · 时间分开，指标一起看",
               "每组显示整圈邻边 F1@2 mm*。11/23 同列显示留出码 P50，11/16 另有 360° 复核。")
    cols=[44,71,285,435,585,735,862]
    for x,h in zip(cols,("#","算法",*BOX_NAMES,"11/16 F1","11/16  360°")): pdf.t(x,145,h,9.5,pdf.TEAL)
    for i,(m,info) in enumerate(BASE.items()):
        y=170+22*i
        if m in ("aruco","pose_graph"): pdf.box(44,y-15,912,22,"#e9f5f1",r=4)
        metrics=[]
        for sid in SESSIONS:
            r=md[sid,m]
            metrics.append(f"{pct(r,'f1_2')} / {unit(r,'tag_p50_px',' px')}")
        o=older.get(m,{})
        for x,v in zip(cols,(f"{i+1:02}",info[0],*metrics,pct(o,"f1_2"),pct(o,"repeat_f1_2"))):
            pdf.t(x+1,y,v,9.7)
        pdf.rule(44,y+7,912)
    pdf.box(44,415,912,154,pdf.BG,r=8)
    for j,(title,body) in enumerate((
       ("11/23 推荐","保留 ArUco 逐视角位姿。居中与偏心组的位姿图没有候选边通过门控；表面 F1 较高的算法往往同时移动了纸盒方向。"),
       ("11/16 结果","0° / 360° 原位 F1 只有 26.1%。点到点能提高邻边重合，但独立整圈闭合角接近 179°。旧档案未提供逐帧绝对位姿。"),
       ("优先调整","固定光学支架，用多倾角标定及已知长宽的尺寸件检查深度。纸盒转台上保留 ArUco，优先补光照不足的侧面与重复回零帧。"))):
        line(58,443+j*43,title,body,877)
    pdf.t(44,588,"11/23 偏心组与 17:50 组的两帧、全圈三方向图和 PLY 单独保存在结果目录。",9,pdf.MUTED)
    pdf.footer(14,("charuco",))


def main():
    for key in ("minion","apple","box"):
        destination=ROOT/"output/pdf"/f"{TITLES[key]}_多视角拼接Benchmark.pdf"
        pdf.C=canvas.Canvas(str(destination),pagesize=(pdf.W,pdf.H),pageCompression=1)
        pdf.C.setTitle(f"{TITLES[key]} · 多视角拼接 Benchmark")
        if key!="box":
            data=load(OUT/key/"summary.csv")
            model=json.loads((OUT/key/"experiment.json").read_text())
            native_cover(key,model)
            native_bench(key,model)
            for i,(m,r) in enumerate(data.items(),3):
                method_page_native(key,i,m,r,model)
            native_summary(key,data,model)
        else:
            older=load(OUT/"legacy-box/summary.csv")
            models=json.loads((MODERN/"recovery/model.json").read_text())
            modern=modern_data()
            box_cover(older,models)
            box_bench(older,models)
            for i,m in enumerate(BASE,3):
                box_method(i,m,modern,older)
            box_summary(modern,older)
        pdf.C.save()
        print(destination,flush=True)


if __name__=="__main__":
    main()
