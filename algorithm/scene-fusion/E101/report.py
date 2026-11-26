"""Create and verify the E101 report, metric index, and registration previews."""
import csv
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image
from reportlab.lib.colors import HexColor, Color, white
from reportlab.lib.utils import ImageReader
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen.canvas import Canvas

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
BRANCHES = ('ChArUco','Chessboard')
CASES = ('三棱锥_小面坐地','三棱锥_大面坐地','坐着人偶','小南瓜','维生素包装圆柱')
FONT = 'ArialUnicode'
pdfmetrics.registerFont(TTFont(FONT,'/Library/Fonts/Arial Unicode.ttf'))
INK = '#152838'
MUTED = '#536877'
LINE = '#DCE6ED'
PALE = '#F1F6F9'
TEAL = '#00796F'
BLUE = '#2A64AC'
ORANGE = '#AC5527'
CLOUD = np.array([[242,163,98],[98,200,232],[98,213,160],[208,153,241],
                  [252,213,100],[117,161,248],[246,142,174],[161,211,103],
                  [124,209,214],[221,156,111],[188,186,244],[124,180,134]])
DESCRIPTIONS = {
'R01': '指令角 + 标定轴', 'R02': '标记实测角 + 共轴', 'R03': '逐站标记 PnP',
'R04': '点到点 ICP', 'R05': '点到面 ICP', 'R06': 'Tukey ICP + 门控位姿图',
'R07': '棱锥侧面 + 共轴','R08': 'GICP','R09': '灰度 ICP','R10': 'Tukey ICP 串联',
'R11': 'FPFH–RANSAC + ICP','R12': 'FPFH–FGR + ICP','R13': 'ISS–FPFH–SAC-IA + ICP',
'R14': 'OpenCV PPF + ICP','R15': '白场 SIFT + 三维对应 + ICP','R16': 'Go-ICP + ICP',
'R17': 'CloudCompare ICP','R18': 'TEASER++ + ICP','R19': '共轴角微调 + 局部匹配',
}
RATIONALE = (
('小面坐地时部分站仅露出板边。共轴先验先固定整圈，再用图优化削弱侧面接缝。',
 '小面组的角度比指令值早约 1.4°–2.9°，单靠步数不能充分对齐。'),
('大面姿态使可见面更宽，标记角共轴结果已经稳定。',
 'R07 使用实测棱锥面，保持点坐标。该轮的留出深度改善未超过选用门槛。'),
('头部、脸和手臂提供局部约束。ChArUco 的门控图保留这些约束，同时限制串联漂移。',
 '照片中的高光和背面弱纹理会影响灰度 ICP 与 SIFT。'),
('点云的重复沟槽容易连到相邻沟槽。共轴路线保留各站真正的绕圈次序。',
 '只按局部几何取最优时，多个站会聚到相近方位。'),
('瓶身近似旋转对称，点到面 ICP 可沿圆柱面滑动。盘上码提供与瓶身形状无关的方位。',
 '整圈评估同时读取方位缺口、留出码与物体深度，可识别高 F1 的叠站。'),
)


def read(path):
    return json.loads(path.read_text())


def rows():
    for name in CASES:
        for branch in BRANCHES:
            case = HERE/branch/name
            result = read(case/'结果.json')
            acq = read(case/'采集.json')
            yield (name,branch,case,result,acq)


def ply(path):
    dtype = [('p','<f4',(3,)),('rgb','u1',(3,)),('view','u1')]
    with path.open('rb') as file:
        header = []
        for line in file:
            header.append(line)
            if line.strip() == b'end_header':
                break
        n = int(next(v for v in header if v.startswith(b'element vertex')).split()[-1])
        assert path.stat().st_size-file.tell() == 16*n, str(path)
        v = np.fromfile(file,dtype,count=n)
    return v


def scene_image(vertices, center, axis, angle, elevation, bounds, size=(800,542)):
    width,height = size
    up = axis if axis[2]<0 else -axis
    right = np.cross([0.,0.,1.],up)
    right /= np.linalg.norm(right)
    back = np.cross(up,right)
    a,e = np.deg2rad([angle,elevation])
    horizontal = right*np.cos(a)+back*np.sin(a)
    bearing = np.cross(up,horizontal)
    vertical = up*np.cos(e)+bearing*np.sin(e)
    depth = np.cross(horizontal,vertical)
    p = vertices['p'].astype(float)-center
    view = p @ np.c_[horizontal,vertical,depth]
    x = np.floor(width*.5+view[:,0]*width/bounds).astype(int)
    y = np.floor(height*.52-view[:,1]*width/bounds).astype(int)
    k = (x>=0)&(x<width)&(y>=0)&(y<height)
    order = np.argsort(view[k,2])
    i,j = y[k][order],x[k][order]
    gray = vertices['rgb'][k][order,0]
    shade = .53+.47*np.sqrt(gray/255)
    colors = (CLOUD[vertices['view'][k][order]]*shade[:,None]).astype('uint8')
    canvas = np.full((height,width,3),[17,31,43],np.uint8)
    canvas[i,j] = colors
    return Image.fromarray(canvas)


def preview(case, result, acquisition, code=None):
    code = code or result['chosen']
    p = ply(case/'整圈'/f'{code}.ply')
    center = np.array(acquisition['center_mm'])
    axis = np.array(acquisition['axis'])
    pair = p[p['view']<2]
    # Equal display scale and camera orientation across branches.
    images = [scene_image(pair,center,axis,-14,23,230), scene_image(p,center,axis,-14,23,230),
              scene_image(p,center,axis,3,78,230)]
    return images


def text(c, x,y, value, size=11, color=INK, align='left'):
    c.setFillColor(HexColor(color)); c.setFont(FONT,size)
    options = {'left':c.drawString,'right':c.drawRightString,'center':c.drawCentredString}
    options[align](x,y,str(value))


def rule(c,x,y,w):
    c.setStrokeColor(HexColor(LINE)); c.setLineWidth(.65); c.line(x,y,x+w,y)


def rect(c,x,y,w,h,color):
    c.setFillColor(HexColor(color)); c.rect(x,y,w,h,stroke=0,fill=1)


def paragraph(c,x,y,value,width=746,size=11,leading=19,color=INK):
    for p in value.split('\n'):
        line = ''
        for char in p:
            new = line+char
            if pdfmetrics.stringWidth(new,FONT,size)>width and line:
                text(c,x,y,line,size,color)
                y-=leading
                line=char
            else:
                line=new
        if line:
            text(c,x,y,line,size,color);y-=leading
    return y


def header(c,p,title,subtitle):
    text(c,42,792,'E101    /    单目结构光转台',11,TEAL)
    text(c,799,792,'2026.11.25    /    多视角点云配准',10,MUTED,'right')
    text(c,42,747,title,32)
    text(c,42,717,subtitle,11,MUTED)
    rule(c,42,699,757)
    rule(c,42,47,757)
    text(c,42,29,'E101     摄像机 + 投影仪 + 步进转台',9,MUTED)
    text(c,799,29,f'{p:02d}',9,MUTED,'right')


def table(c,x,y,cols,data,rh=27,font=10):
    starts=np.r_[0,np.cumsum([w for _,w in cols])]+x
    total=starts[-1]-x
    rect(c,x,y-rh,total,rh,PALE)
    for i,(name,_) in enumerate(cols):
        text(c,starts[i]+8,y-rh+9,name,font,MUTED)
    y-=rh
    for i,row in enumerate(data):
        if i%2==1: rect(c,x,y-rh,total,rh,'#F9FBFD')
        for j,value in enumerate(row):
            text(c,starts[j]+8,y-rh+9,value,font)
        y-=rh
        rule(c,x,y,total)
    return y


def pct(x):
    return '—' if x is None else f'{100*x:.1f}%'


def num(x, digits=2):
    return '—' if x is None else f'{x:.{digits}f}'


def group_map(records):
    return {(n,b):(f,r,a) for n,b,f,r,a in records}


def chosen(record):
    return record[1]['metrics'][record[1]['chosen']]


def outcome_page(c, records):
    header(c,1,'两套标定，五组实测','同一批 54 个原始视角分别重建。整圈 PLY 保留转台、标定板和其余有效扫描点。')
    idx=group_map(records)
    best = [[chosen(idx[name,b]) for name in CASES] for b in BRANCHES]
    cards=[
        ('54', '个原始角站', '12 + 12 + 10 + 10 + 10'),
        (f'{np.median([r["F1_2"] for r in best[0]])*100:.1f}%', 'ChArUco · F1@2 中位', f'棋盘格 {np.median([r["F1_2"] for r in best[1]])*100:.1f}%'),
        (f'{np.median([r["depth_abs_P50"] for r in best[0]]):.2f} mm', 'ChArUco · 深度 P50 中位',
         f'棋盘格 {np.median([r["depth_abs_P50"] for r in best[1]]):.2f} mm'),
    ]
    for i,(large,label,small) in enumerate(cards):
        x=42+255*i
        rect(c,x,563,247,104,PALE)
        text(c,x+12,628,large,30,TEAL)
        text(c,x+12,601,label,13)
        text(c,x+12,579,small,10,MUTED)
    rr=[]
    for name in CASES:
        _,L,A=idx[name,BRANCHES[0]]; lc=chosen(idx[name,BRANCHES[0]])
        _,R,_=idx[name,BRANCHES[1]]; rc=chosen(idx[name,BRANCHES[1]])
        rr.append((name.replace('_',' · '),L['n_views'],L['chosen'],pct(lc['F1_2']),
                   num(lc['depth_abs_P50']),R['chosen'],pct(rc['F1_2']),num(rc['depth_abs_P50'])))
    table(c,42,524,[('物体',162),('视角',48),('ChArUco',80),('F1@2',78),('深度 mm',78),
                   ('Chessboard',104),('F1@2',92),('深度 mm',105)],rr,35,11)
    text(c,42,283,'结论',17)
    paragraph(c,42,258,'本批五组的 ChArUco 标定均取得更高推荐方案 F1@2 和更低留出深度误差。'
              '标记角共轴 R02 是默认起点。小面棱锥和坐着人偶采用门控图 R06，另外三组保留 R02。'
              '棋盘格支路主要保留 R02，小面棱锥采用逐站 PnP R03。',748,12,20)
    rule(c,42,192,757)
    text(c,42,169,'目录',14)
    text(c,42,144,'02  标定与采集      03  评估基准      04–08  五组点云      09  方法全集      10  方位诊断与结论',11,MUTED)
    text(c,42,108,'实验文件每组含单视角、各方法整圈、推荐双视角、位姿矩阵和逐邻边指标。',11,MUTED)
    c.showPage()


def optical_page(c, records):
    header(c,2,'从步进脉冲到三维点','两支路分别标定摄像机与投影仪。五组扫描的盘上均放置同一规格 ChArUco 板。')
    flow = [('步进停转','341 或 410 脉冲'),('灰码采集','42 张编码 + 白场 / 黑场'),
            ('光学重建','局部同态 + 双射线三角化'),('盘面位姿','留码 PnP + 共享转轴')]
    for i,(t,s) in enumerate(flow):
        x=42+i*192
        rect(c,x,583,174,75,PALE)
        text(c,x+10,630,t,14,TEAL)
        text(c,x+10,606,s,10)
        if i<3: text(c,x+184,615,'→',13,MUTED,'center')
    data=[]
    for b in BRANCHES:
        m=read(HERE/b/'标定.json')
        v=m['validation']
        same=[r for r in v if r['target']==b and r['holdout']]
        pooled=[r for r in v if r['holdout']]
        data.append((b,str(len(m['training'])),str(len(same)),num(m['stereo_train_rms']),
                     num(np.median([q['projector_p50'] for q in same])),
                     num(np.median([q['projector_p50'] for q in pooled])),
                     num(m['baseline_mm'])))
    text(c,42,539,'光学标定',16)
    y=table(c,42,519,[('标定靶',110),('训练站',75),('同靶留出站',97),('双目标定 RMS px',118),
                     ('同靶投影 P50 px',120),('共 9 站 P50 px',130),('基线 mm',107)],data,30,9)
    paragraph(c,42,y-23,'ChArUco 为 10×10 格、方格 10 mm、码边 7 mm、DICT_5X5_50。旧棋盘为 9×7 内角点，格边 10 mm。'
              '两套模型各自以整站留出。共 9 站为两批留出图的并集，在每张图上重新做相机 PnP 后比较投影误差。',748,11,19)
    rule(c,42,355,757)
    text(c,42,328,'扫描与导出',16)
    paragraph(c,42,304,'控制脚本按 4096 步 / 圈换算，12 站与 10 站的指令步距为 29.97° 和 36.04°。'
              '控制器报告停转后等待 1 秒。每幅图案至少等待 0.1 秒加曝光时间，清缓存并丢弃首帧。'
              '记录中的角度为指令值。R02 从白场码的像素观测拟合共轴角度。',748,11,19)
    paragraph(c,42,234,'全画幅每 3 px 采样。白黑差 14，位对差 5，先用全位解码，再容许缺 1–2 个低位。'
              '保留两设备前方且通过重投影门槛的 3D 点。每支路单站有效顶点全部进入整圈文件，导出无分割、无点云降采样。',748,11,19)
    plane=read(HERE/'E101光学验证.json')
    deck=[np.median([r[b]['abs_P50_mm'] for r in plane if b in r]) for b in BRANCHES]
    paragraph(c,42,174,f'盘面独立检查    在 {sum(r["shared_plane_points"] for r in plane):,} 个共同留出码像素上，'
              f'结构光点到盘面距离的站间中位为 ChArUco {deck[0]:.2f} mm、Chessboard {deck[1]:.2f} mm。',750,10,16)
    for i,b in enumerate(BRANCHES):
        count=sum(sum(x['all_points'] for x in a['views']) for n,k,_,r,a in records if k==b)
        text(c,42+380*i,123,f'{b}     {count:,} 个全画幅有效顶点',13,TEAL if i==0 else BLUE)
    text(c,42,88,'采集批次、标定靶、摆位与样本数共同变化。本表比较这两次采集得到的完整标定。',10,MUTED)
    c.showPage()


def benchmark_page(c, records):
    header(c,3,'评估基准','所有方法共用相邻站序，首尾计入配准边。两标定支路在相同摄像机像素取物体分数。')
    cols=[('层级',111),('指标',217),('定义与读数',429)]
    data=[('局部贴合','双向 F1@1 / 2 / 4 mm','双向最近邻命中率的调和平均，取邻边中位、P10 与最差边。'),
          ('局部贴合','内点 RMSE 与法向接缝 P95','2 mm 内点双向 RMSE。4 mm 内互近邻沿目标法向的绝对误差。'),
          ('深度一致性','留出支持率、深度 P50、前后冲突','其余帧训练点按最近深度投影到 8×8 px 栅格，查看目标留出点。'),
          ('整圈位姿','最大 / 最小方位间隙与改动','按相机关于轴心的方位排序，含首尾；相对 R02 另报角度与轴心位移。'),
          ('外部观测','留出码重投影、相邻边闭环','码角按 ID 留出。独立邻边全部成功时合成闭环旋角与轴心位移。')]
    y=table(c,42,666,cols,data,35,10)
    paragraph(c,42,y-20,'训练/验证以 24 px 图块固定分组，每五块留一块。ID ≡ 1 (mod 4) 的码不用于拟合。'
              '基准仅用两支路都重建、都落在板上方物体区域的同像素点。训练点先去稀疏噪声，再做多尺度降采样。',750,11,18)
    text(c,42,397,'角站密度对照',16)
    text(c,42,376,'全量与奇偶交错减半，固定原位姿，各组分别计算。下图为两支路推荐方法的 F1@2。',10,MUTED)
    idx=group_map(records)
    for i,name in enumerate(CASES):
        x=42+i*152
        text(c,x,346,name.replace('三棱锥_','').replace('维生素包装圆柱','维生素圆柱'),11)
        for k,b in enumerate(BRANCHES):
            _,r,_=idx[name,b]; q=r['chosen']; full=r['metrics'][q]['F1_2']
            thin=np.mean([p['F1_2'] for p in r['thinning'][q]])
            y0=319-k*40
            rect(c,x,y0,126,7,PALE)
            rect(c,x,y0,126*full,7,TEAL if k==0 else BLUE)
            rect(c,x,y0-10,126*thin,4,'#8FBAB8' if k==0 else '#AABFE0')
            text(c,x,y0-23,f'{full*100:.0f}% / {thin*100:.0f}%',9,MUTED)
    text(c,42,244,'上行 ChArUco   下行 Chessboard   粗线全量 / 细线减半',10,MUTED)
    text(c,42,198,'推荐规则',16)
    paragraph(c,42,172,'相对共轴基线 R02，最大方位缺口增量 ≤ 2.5°，留出码 P50 增量 ≤ 1 px，'
              '深度支持降幅 ≤ 2.5 个百分点，F1@2 降幅 ≤ 1.5 个百分点，深度 P50 改善 > 3%。合格方案按深度 P50 选。'
              '全圈几何法有回退边时只作为混合对照。',751,11,19)
    text(c,42,91,'减半使用同一组位姿。方位覆盖描述角站分布。本批没有独立 360° 回零帧。',10,MUTED)
    c.showPage()


def case_page(c, ix, records):
    name=CASES[ix]; idx=group_map(records)
    case=idx[name,BRANCHES[0]]
    header(c,ix+4,name.replace('_',' · '),f'{case[1]["n_views"]} 个角站   /   同一原始灰码序列   /   按角站着色')
    for i,label in enumerate(('V00 + V01','全部角站 · 斜视','全部角站 · 俯视')):
        text(c,42+i*255,679,label,12)
    rr=[]
    for k,b in enumerate(BRANCHES):
        path,r,a=idx[name,b]
        q=r['chosen']; m=r['metrics'][q]
        images=preview(path,r,a)
        ybase=487-k*205
        for j,im in enumerate(images):
            c.drawImage(ImageReader(im),42+j*255,ybase,247,167,mask='auto')
        text(c,42,ybase-17,f'{b}  ·  {q}  {DESCRIPTIONS[q]}',11,TEAL if k==0 else BLUE)
        cdoc=Image.new('RGB',(2400,542),(17,31,43))
        for j,im in enumerate(images): cdoc.paste(im,(800*j,0))
        cdoc.save(path/'预览.jpg',quality=91)
        rr.append((b,q,pct(m['F1_2']),pct(m['worst_F1_2']),num(m['depth_abs_P50']),
                   num(m['tag_P50']),num(m['max_gap'],1),f'{r["full_cloud_vertices"][q]:,}'))
    table(c,42,247,[('标定',102),('方案',49),('F1@2',87),('最差邻边',92),('深度 P50 mm',128),
                   ('留出码 px',101),('最大缺口 °',99),('整圈点数',99)],rr,28,10)
    paragraph(c,42,144,'\n'.join(RATIONALE[ix]),745,11,20)
    text(c,42,86,'图中显示转台附近的固定视窗。全部有效扫描点见对应整圈 PLY。',9,MUTED)
    c.showPage()


def algorithms_page(c, records):
    header(c,9,'方法全集','方法级统计取各分支、各物体的中位或最差值。横向比较以单组指标为准。')
    data=[]
    for code,name in DESCRIPTIONS.items():
        r=[a['metrics'][code] for _,_,_,a,_ in records if code in a['metrics']]
        data.append((code,name,len(r),sum(q['fallback_edges']==0 for q in r),
                     pct(np.median([q['F1_2'] for q in r])),num(np.median([q['tag_P50'] for q in r])),
                     num(max(q['max_gap'] for q in r),1)))
    table(c,42,668,[('编号',52),('算法',242),('有结果组',79),('无回退组',82),('中位 F1@2',99),
                   ('中位留出码 px',105),('最大缺口 °',98)],data,26,10)
    paragraph(c,42,121,'R07 用棱锥侧面与转轴先验，适用于两种棱锥姿态。'
              '特征或全局法无法得到有重叠的邻边时回退到该边 R02，JSON 逐边标记。'
              'FlexScan3D 为商业软件文献对照，本机无运行环境。',751,10,17)
    c.showPage()


def coverage_page(c, records):
    header(c,10,'局部贴合高分与方位失真','图中使用同一批 ChArUco 圆柱扫描。F1@2 是邻边指标，方位缺口评价整个角站环。')
    idx=group_map(records)
    case,r,a=idx[CASES[-1],'ChArUco']
    box=(65,363,477,266)
    x,y,w,h=box
    rect(c,x,y,w,h,PALE)
    def px(v): return x+w*(v-25)/(360-25)
    def py(v): return y+h*(v-.28)/(.98-.28)
    for value in [30,90,150,210,270,330]:
        u=px(value)
        c.setStrokeColor(HexColor(LINE)); c.line(u,y,u,y+h)
        text(c,u,y-17,value,9,MUTED,'center')
    for value in [.3,.5,.7,.9]:
        v=py(value)
        rule(c,x,v,w);text(c,x-7,v-4,f'{value*100:.0f}%',9,MUTED,'right')
    for code,m in r['metrics'].items():
        em=code in ('R02','R05')
        c.setFillColor(HexColor(TEAL if code=='R02' else ORANGE if code=='R05' else '#9FADBD'))
        c.circle(px(m['max_gap']),py(m['F1_2']),5 if em else 3,fill=1,stroke=0)
    for code,dy,dx in (('R02',10,11),('R05',-14,-9)):
        m=r['metrics'][code]
        text(c,px(m['max_gap'])+dx,py(m['F1_2'])+dy,code,11,TEAL if code=='R02' else ORANGE)
    text(c,x+w,y-35,'最大方位缺口 (°)',10,MUTED,'right')
    text(c,x,652,'相邻 F1@2',10,MUTED)
    for i,code in enumerate(('R02','R05')):
        m=r['metrics'][code]
        text(c,569,604-i*130,f'{code}   {DESCRIPTIONS[code]}',13,TEAL if i==0 else ORANGE)
        text(c,569,576-i*130,f'F1@2 {pct(m["F1_2"])}    缺口 {num(m["max_gap"],1)}°',11)
        text(c,569,551-i*130,f'留出码 P50 {num(m["tag_P50"])} px',11)
        text(c,569,526-i*130,f'留出深度 P50 {num(m["depth_abs_P50"])} mm',11)
    images=[preview(case,r,a,q)[2] for q in ('R02','R05')]
    for i,im in enumerate(images):
        c.drawImage(ImageReader(im),42+252*i,147,244,165)
        text(c,42+252*i,129,('R02 · 转台与整圈','R05 · 多站叠到同侧')[i],10,MUTED)
    paragraph(c,563,299,'R02 保留物理角站。R06 在训练邻边有可靠重叠时，只做小幅全局修正。'
              '近回转体优先保留共轴角和留出码约束。',235,11,19)
    paragraph(c,563,206,'增加角站能提高本批重叠。棋盘格支路在相同像素下仍有较大深度差，角站密度与光学标定应分别比较。',235,11,19)
    for x,label,url in [(42,'OpenCV ChArUco','https://docs.opencv.org/4.13.0/df/d4a/tutorial_charuco_detection.html'),
                        (188,'Open3D 位姿图','https://www.open3d.org/docs/release/tutorial/pipelines/multiway_registration.html'),
                        (340,'TEASER++','https://github.com/MIT-SPARK/TEASER-plusplus')]:
        text(c,x,80,label,10,TEAL); c.linkURL(url,(x,77,x+127,94))
    c.showPage()


def validate_and_index(records):
    results=[]
    for name,b,path,result,acq in records:
        vertices=[ply(f) for f in sorted((path/'单视角').glob('*.ply'))]
        one=[len(p) for p in vertices]
        step=[max(1,n//64) for n in one]
        samples=[v[::g]['p'] for v,g in zip(vertices,step)]
        assert len(one)==result['n_views']
        assert one==[v['all_points'] for v in acq['views']]
        full=sum(one)
        ordered=np.array(list(result['matrices'].values()))
        assert np.allclose(np.linalg.det(ordered[:,:,:3,:3]),1.,atol=1e-4)
        assert np.allclose(ordered[:,0],np.eye(4),atol=1e-8)
        for variant,n in result['full_cloud_vertices'].items():
            assert n==full
            full_scene=ply(path/'整圈'/f'{variant}.ply')
            assert np.bincount(full_scene['view']).tolist()==one
            first=0
            for i,(length,stride,src,T) in enumerate(zip(one,step,samples,result['matrices'][variant])):
                last=first+length
                T=np.asarray(T)
                ref=src@T[:3,:3].T+T[:3,3]
                assert np.allclose(full_scene['p'][first:last:stride],ref,atol=1e-4)
                assert (full_scene['view'][first:last:stride]==i).all()
                first=last
        actual=ply(path/'整圈'/'推荐.ply')
        assert np.array_equal(actual,ply(path/'整圈'/f'{result["chosen"]}.ply'))

        assert len(ply(next((path/'双视角').glob('*.ply'))))==sum(one[:2])
        results.append(dict(branch=b,object=name,views=len(one),methods=len(ordered),
                            full_vertices=full,recommended=result['chosen']))
    with (HERE/'E101索引.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=list(results[0]))
        w.writeheader();w.writerows(results)
    (HERE/'E101核验.json').write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n')


def main():
    record=list(rows())
    validate_and_index(record)
    output=ROOT/'output/pdf/E101.pdf'
    c=Canvas(str(output),pagesize=(841.89,841.89),pageCompression=1)
    c.setTitle('E101 多视角点云拼接 · 两套标定实测')
    c.setAuthor('结构光测量项目')
    outcome_page(c,record)
    optical_page(c,record)
    benchmark_page(c,record)
    for ix in range(len(CASES)):
        case_page(c,ix,record)
    algorithms_page(c,record)
    coverage_page(c,record)
    c.save()
    print(output)


if __name__=='__main__':
    main()
