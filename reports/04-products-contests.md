# 赛题二 · 现有方案调研（04）：市面产品、开源三维扫描仪、历届竞赛与合规红线

> 调研范围：行业在用的体积测量设备、商用三维扫描仪与深度相机的精度/价格水平线、开源 DIY 三维扫描项目、历届同类竞赛题目、以及"哪些做法会被判违规"。
> 说明：所有价格与精度数字均附来源链接（见第七节）。凡未亲自访问核实的内容，一律标注"未找到可靠来源"或"不确定"。**我没有找到任何一届光电设计竞赛的获奖队伍公开方案细节，也没有找到任何因使用商用设备被取消成绩的报道**——这两项是空白，请勿在后续材料里编造。

---

## 一、现状与标杆：别人是怎么测体积的？

| 行业 | 技术路线 | 精度水平 | 价格 / 规模 |
|---|---|---|---|
| **物流快递 DWS 一体机**（量方 Dimension + 称重 Weight + 扫码 Scanning 三合一） | 超声波量方（只适合规则纸箱）/ 光学光幕（一排对射光栅，"用很多条看不见的线量尺寸"）/ 激光扫描 / 3D 视觉（双目、结构光、ToF） | 动态 **±1mm**、3000 件/小时；静态 **±1mm**、称重 ±1g；托盘大货 **±5mm**（墨家科技官方科普页）；德马科技称称重最高 ±10 g | 2025 年全球约 **6.51 亿美元 / 2.6 万套 / 均价约 2.5 万美元一套**；"标准静态包裹设备的价格通常为数千美元"，取得计量认证的托盘/动态线体"即可达到数万美元"（QYResearch） |
| **农产品 / 水果分级** | 产业界主流是**称重分级**（法国 FACHAUX）或外观品质分级（陶朗 Spectrim），**不是体积**。学术上最接近本题的是《基于双目视觉的海参体积测量方法》（农业工程学报 2024, 40(21):165-174）：双目相机拍（靠左右视差算深度）→ 深度学习分割 → 点云 → 泊松表面重建（把散点"糊"成连续曲面）→ 体素化（切成小方块数格子）算体积 | 100 cm 处体积准确率 **94%**（相对误差约 6%），比"最小包围框法"高 22 个百分点；标准值**用排水法**测得 | — |
| **文物数字化 / 医疗辅助** | 文物 = 采集表面形貌 + 颜色 → 纹理模型；医疗（先临三维 EinScan Medixa 案例）= 扫描肢体/缺损部位做支具、假体 | 营销文称"0.01mm、5 分钟"（**厂商口径，不确定**）；两者**通常都不出体积数字** | — |

**三条结论**：
1. DWS 的 1mm 是**包裹级**（几十厘米箱子）精度，不是 5–20 cm 小物体的精度；且全是成套商用设备，赛题禁止作核心测量单元。
2. 农产品论文给出的水平线是**约 5–10% 体积相对误差，且测的是近似凸、无凹槽通孔的单个物体**——赛题的凹槽/台阶/通孔难一个量级。
3. 文物与医疗追求"长得像"和贴合精度，用的正是赛题禁止的商用结构光扫描仪。

---

## 二、商用产品与精度水平线（判断红线的基础）

| 产品 | 技术路线 | 标称精度 | 价格 | 来源 |
|---|---|---|---|---|
| Revopoint POP 3 | 双目红外结构光 | 单帧准确度 ≤0.1 mm，单帧重复性 ≤0.05 mm | 官网首页"From $495"（POP 3 Plus 起步）；All3DP 称 POP 3 Plus 约 $620 | revopoint3d.com；all3dp.com |
| Revopoint MIRACO | 一体式结构光 | 官网未在摘要页给出 | 官网售价 $1,274（原价 $1,499） | revopoint3d.com |
| Creality CR-Scan Otter | 双光源手持 | 官方店标"Up to 0.02mm Accuracy" | 官方店 $899 | store.creality.com |
| EinScan Rigil（先临三维） | 蓝激光 50 线+7 线 / 红外 VCSEL 双光源 | 基础精度最高 0.04 mm；**体积精度 0.04 mm + 0.06 mm/m** | 官网未标价 | shining3d.cn |
| 国产工业三维扫描仪价格带 | — | 入门级 0.05–0.1 mm；工业级手持 0.02–0.04 mm；计量级 0.01–0.02 mm | 1万–5万 / 8万–30万 / 20万–60万元 | insvision3d.com（**厂商博客，仅供参考**） |
| 基恩士 LJ-X8000 线激光轮廓仪 | 线激光三角法 | 3200 点/轮廓，线宽最大 720 mm；未标 μm 级精度 | 需询价 | keyence.com.cn |
| iPhone LiDAR | ToF（飞行时间） | 大于 10 cm 的物体，绝对精度约 ±1 cm | 随机型 | 3dmag.com 转述 Scientific Reports 评测 |
| Azure Kinect / Orbbec Femto Bolt | ToF | 随机误差标准差 ≤17 mm，典型系统误差 <11 mm + 距离的 0.1% | — | orbbec.com 产品对比文档 |
| Intel RealSense D435 | 主动双目 | 深度精度 <2% @ 2 m；最小工作距离约 28 cm（848×480 下约 19.5 cm） | — | realsenseai.com |
| Intel RealSense D405 | 短距被动双目 | 理想距离 7–50 cm，7 cm 处最小可测物体 0.1 mm | 发布价 $259 | therobotreport.com |

**一个容易被忽略的关键点**：D405 这类短距深度相机在"深度（Z）方向"确实能到亚毫米级，看着很能打。但第三方评测（PMC10284995，12–20 cm 距离、3D 打印台阶/立方体试件，测 D405、D415、ZED-Mini）的结论是：D405 综合最好，**但"台阶边缘无法分辨"**（2.5 mm 级台阶只能"粗略重建"），对反光金属件和细结构（缝线）**直接重建失败**。这正好命中赛题的复杂特征——**台阶、凹槽、通孔、盲孔全是"边缘"**。所以深度相机不只是"规则上只能当辅助"，技术上也不适合当主数据源。

---

## 三、开源 DIY 三维扫描仪项目清单（"自己搭"最直接的参考）

| 项目 | 原理 | 硬件清单 | 成本 | 精度 | 代码/资料 | 有体积功能吗 |
|---|---|---|---|---|---|---|
| **OpenScan Mini / Classic** | 摄影测量（拍一圈照片，用照片反算三维） | 树莓派 + Arducam IMX519 16MP 相机 + 环形灯 + **交叉偏振片（消反光）** + NEMA17 步进电机 + 3D 打印件 | 整机套件 **$266 起**；零件：打印件 $63 起、IMX519 $45、NEMA17 $19 | 官网称"可低于 0.02 mm（Mini）/0.01 mm（Classic）"，但明确注明"高度依赖物体准备和操作水平"→ **营销口径，不确定** | 全开源：GitHub `OpenScan-org/OpenScan-Doc`、`OpenScan-org/OpenScan-Design` | 只出模型，**体积要自己算**（有免费云处理） |
| **FreeLSS + ATLAS 3D（Murobo）** | 激光三角法（转台转 + 线激光扫，相机拍激光条纹） | 树莓派 + 5MP Pi 相机 + 线激光 + 转台 | ATLAS 3D 走 Kickstarter 众筹，**未找到可靠官方定价** | 未找到可靠数据 | GitHub `hairu/freelss`（开源） | 出点云/模型，体积自算 |
| **BQ Ciclop** | 激光三角法 + 转台，双激光 | 开源硬件 + 3D 打印件；配套开源软件 Horus | 上市价"低于 $500"（论坛）；DIY 复刻可压到 $100 级 | **0.5 mm** | LulzBot 介绍页；Horus 开源 | 出模型 |
| **Sardauscan** | 激光三角法 + 转台 | Arduino Nano($4) + 步进电机及驱动($5) + 线激光(每个 $2.5) + 罗技摄像头($15) | **$26.5–35** | 作者自述"仍在开发中，达不到工业扫描仪效果，应视作学习项目" | GitHub `Sardau/Sardauscan` | 出点云 |
| **《Sensors》开源激光扫描仪**（PMC12788225） | 摆镜扫线激光 + 相机三角法 | 树莓派 4($70) + Pi 相机(<$30) + Arduino UNO 兼容板(<$10) + 28BYJ-48 步进电机+ULN2003($3) + 线激光(<$2) | **约 $115** | **4 个方块试件实测：高度平均误差最大 2.56 mm、长度 1.48 mm、宽度 1.30 mm**；约 4 秒/激光线 | GitLab `gitlab.cs.pub.ro/andrei.mateescu0612/3d-scanner`（含 Arduino/Python 代码） | 出点云 |
| **SLStudio** | 结构光（相机 + 投影仪投条纹图案） | 单相机 + 投影仪；实时需投影仪给相机硬触发，推荐 LightCrafter 4500（普通投影仪最多 60 图案/秒） | 取决于相机/投影仪 | 2+3 步相移（6 张图）可达 10 Hz 点云 | GitHub `jakobwilm/slstudio`，GPLv3 或商业许可 | 出点云 |
| **SL Sensor（ETH）** | 结构光 ROS 实时传感器 | 相机 + 投影仪 | — | 5 Hz 实时点云 | GitHub `ethz-asl/sl_sensor` | 出点云 |
| **scAnt** | 摄影测量（小物体/昆虫） | 3D 打印 + 激光切割结构件 + 步进电机 + 可选单反/工业相机 | 硬件约 **€200**，加相机镜头 €700 ≈ €900；同类商用系统 €5,000–8,000 | 论文未给单一误差数字 | GitHub `evo-biomech/scAnt`（MIT） | 论文提到可用于面积/体积测量，但**不直接输出体积** |

**精度水平线小结（这条最重要）**：
- $30–150 级 DIY 扫描仪 → **毫米级（1–3 mm）**形貌误差；
- $500 级开源转台扫描仪（Ciclop）→ **约 0.5 mm**；
- 商用结构光手持机 → **0.02–0.05 mm**。

**为什么毫米级形貌误差要紧张**（这是我基于误差传播的**估算**，不是实测数据）：体积相对误差约等于三个方向相对误差之和。50 mm 的物体若有 1 mm 线性误差，单方向约 2%，三方向叠加后体积误差量级 **5–10%**；这与 1.2 节海参论文实测的 ~6% 相互印证。想做到 2% 以内，需要**单方向误差压到 0.3 mm 级**。

---

## 四、历届竞赛与同类题目线索

### 4.1 第四届全国大学生光电设计竞赛（2014-08-16，国防科技大学，长沙）——**同一道题的"前身"**

竞赛主题"光与测量"，两个赛题。**赛题二就是本题目的前身：**

> **竞赛题目2：复杂表面物体体积的非接触光学测量**
> - 被测物：**橡皮泥捏的棱锥**，底面为平面，各棱面均为**缓变表面，无深度凹陷**；各维尺度**约 5 cm**；形状由同组其他参赛队现场制作。
> - **体积测量精度以被测模型排开水的重量为标准进行评判**（排水法只用于裁判定标准值，不是参赛手段）。
> - 每队可测两次，**每次不超过 10 分钟**；"测量过程中，不可将被测物体放入其他容器中"。

来源：复旦大学光科学与工程系 2014-02-27 通知页；中国光学学会光学教育专业委员会文件（山东大学物理学院 PDF）；科学网报道；中南大学物理学院《第四届全国大学生光电设计竞赛总结》。

**难度跃升**：从"单件、缓变表面、5 cm、10 分钟"变成"3 件、曲面/凹槽/台阶/通孔/盲孔/局部遮挡、5–20 cm、60 秒、全自动"。往届简易方案**不足以直接套用**。

> **未找到可靠来源**：未能检索到第四届全国赛任何获奖队伍的公开方案、答辩材料或论文。唯一线索是复旦页面提到"郑渊中、徐月晗和陈梁将参赛成果总结成的论文已发表在《中国科技纵横》"，但该页说的是**"上届"（第三届）**的成果，与体积测量题目**未必相关**，我未核实全文，**不可引用为依据**。

### 4.2 本届（第十五届）官方口径

- 竞赛官网 `gd.moocollege.com` 显示 **2025-07-21 发布《关于公布第十五届全国大学生光电设计竞赛赛题和优秀应征赛题奖的通知》**。
- 北京理工大学教务部网站可下载《第十五届全国大学生光电设计竞赛赛题》PDF，"赛题 2"与本任务书完全一致，评分细则原文：
  > "总分 100 分，其中现场测量成绩占 70 分，提交海报等展示材料函评占 30 分。**使用被禁止的商用成熟测量仪器或直接输出三维测量结果的成套设备，取消现场测量成绩。**测量过程中接触、移动、夹持、称重、浸没或改造待测目标，该目标成绩无效。人工描边、人工读取关键尺寸、人工修正体积结果，视情节扣分；严重者取消比赛成绩。"
- 关于"自制 vs 商用"的一贯态度：本届是**首次把这条禁令写得如此明确并挂上取消现场成绩的后果**。第四届时是"现场捏橡皮泥 + 裁判用排水法定标准"，商用设备事实上很难用上——**但不能据此认为往届就允许商用设备**。另，竞赛官网/公众号未检索到关于"自制 vs 商用"的历史性专门表述，也未检索到任何违规取消成绩的通报；唯一"取消资格"条款来自四川大学转发的第十四届通知，针对的是**抄袭、盗用、虚假材料**，与商用设备无关。（**未找到可靠来源**）

### 4.3 全国大学生电子设计竞赛——"用普通相机自己搭测量装置"的先例

从 GitHub 题库 `CCBP/NUEDC_Topic`（收录 1994 年起全部电赛题目）可见，电赛长期出这类题：**2025 年 C 题"基于单目视觉的目标物测量装置"**、**2020 年 G 题"非接触物体尺寸形态测量"**、2021 年 D 题"基于互联网的摄像测量系统"、2017 年 M 题"管道内钢珠运动测量装置"。

**意义**：说明"普通相机 + 自研算法做非接触测量"是这类赛事的常规套路，不是本届的新花样；可去找这些题目的优秀作品报告作为**算法与标定**参考。

### 4.4 挑战杯 / 机械创新设计大赛

**未找到可靠来源**：检索"挑战杯 体积测量"只返回赛事介绍页，没有可核实的获奖作品与方案。

---

## 五、合规红线清单

### A. 绝对不能用（用了取消现场测量成绩）

1. **商用三维扫描仪**：EinScan（先临三维）、Revopoint（知象光电）、Creality（创想三维）、Artec、Scantech 等手持/桌面机。
2. **商用结构光扫描仪**（工业级蓝光/白光）。
3. **激光轮廓仪 / 线激光测量仪**：基恩士 LJ-X8000 / LJ-S8000、LMI Gocator、Micro-Epsilon scanCONTROL 等。
4. **工业视觉测量仪、成套体积测量仪**：物流 DWS 一体机、静态量方仪。
5. **能直接输出深度图 / 点云 / 三维模型 / 体积结果的成熟商用测量模块**作核心测量单元。
6. **任何接触**：接触、夹持、按压、浸没、称重、破坏；贴标记、喷涂、加辅助结构。
7. **排水法 / 称重法当测量手段**：第四届时排水法只是裁判定标准值的办法；本届明确禁止"浸没、称重"。**"称重除以密度"直接违规。**
8. **人工介入**：人工描边、手工读关键尺寸、人工修正结果。

### B. 可以作为零件用（自己搭系统的组成部分）

普通相机模组、CMOS/CCD 相机、镜头（"眼睛"）；LED、投影器 / DLP 光机（"投图案的灯"）；低功率线激光器（"一条会变形的高亮线"）；滤光片（只让某波长通过）、偏振片（滤掉镜面反光）；单片机、开发板（Arduino / 树莓派 / STM32）；电机、**普通**机械转台；3D 打印结构件。

> **边界提醒（不确定）**：赛题写的是"**普通**机械转台"。带高精度角度编码器、能直接读出角度的精密电动转台算不算"普通"，赛题没说。建议用普通步进电机 + 自己数步数，避免争议。

> **采购提醒**：工业相机、DLP 光机这类**通用裸零件**合规（赛题明文允许）。但不要买"能直接吐出点云的结构光扫描仪开发套件"——"成品属性 + 直接输出点云"叠加，很可能被判红线。

### C. 可作辅助，但**不可作主数据源**

**深度相机**（Intel RealSense D435 / D405、奥比中光 Astra / Femto、Microsoft Azure Kinect）与 **TOF 相机**。

赛题原话："若使用深度相机、TOF 相机等设备，**仅可作为辅助定位或演示，不得作为体积测量的主要数据来源**。"

**允许**：判断物体是否放好、粗定位、演示展示。**越线**：把深度图/点云当真值算体积，或拿它做主配准基准。

**技术上也不划算**：D435 最小工作距离 19.5–28 cm，对 5 cm 物体视场占比太小；Kinect/TOF 系统误差 11 mm 级；D405 虽 7 cm 起、Z 向亚毫米，但**分不清 2.5 mm 台阶边缘、对反光件和细结构直接失败**——赛题的复杂目标恰恰全是边缘。所以这不只是规则问题，技术上也不该接进主流程。

### D. 其他高风险动作

- **成本清单漏项/失实**：赛题明确"提交成本清单明显失实，取消成本相关得分；情节严重者取消评奖资格"。电脑、显示器、通用电源适配器不计入尺寸与成本，**但核心光电测量硬件必须计入**——相机、镜头、激光器、投影光机、转台、开发板一个都不能漏。
- **尺寸/重量超限**：整机 ≤50×50×50 cm、≤10 kg。**激光安全**：需标明安全等级、遮光防护。**超时**：单件 ≤60 秒，从"启动测量"到"显示体积结果"全程计时。

---

## 六、自己搭一套大概多少钱

| 档次 | 方案 | 成本 | 预期水平 |
|---|---|---|---|
| 极简 | Sardauscan | **$26.5–35** | 学习级，无公开可靠精度 |
| 入门可用 | 《Sensors》开源激光扫描仪 | **约 $115** | 实测 1.3–2.6 mm 形貌误差 |
| 转台成品件 | OpenScan Mini 套件 | **$266 起** | 9×9×9 cm 扫描体积（摄影测量） |
| 复刻经典 | BQ Ciclop 开源件 | 复刻 ~$100 级；原成品"低于 $500" | 0.5 mm |
| 小物体摄影测量 | scAnt | €900（含 €700 相机镜头） | 昆虫级细节 |

**相机升级会显著抬价**：CSDN 上的"slm3D 单目结构光 3D 标定及重建开发套件"用 DLP2010 投影光机 + 海康机器人工业相机，主打"低成本教学级"，但**该文未给总价**，我也没核实到工业相机 + DLP 光机的可靠报价——**不确定**，请以实际询价为准并如实写入成本清单。

---

## 七、参考链接（均为本次调研实际访问）

**行业设备**
- DWS 分类与选购（墨家科技，±1mm / ±1g / ±5mm）：https://mohisttech.com/cn/news/%E7%89%A9%E6%B5%81DWS%E8%AE%BE%E5%A4%87%E6%9C%89%E4%BB%80%E4%B9%88%E7%B1%BB%E5%88%AB%EF%BC%9F%E5%A6%82%E4%BD%95%E9%80%89%E8%B4%AD%EF%BC%9F
- DWS 全球市场规模与均价（QYResearch 转载）：https://caifuhao.eastmoney.com/news/20260803144010962657360
- 德马科技 DWS 系列：https://www.damon-group.cn/intelligent-driven-system-124
- 梅特勒-托利多 DWS 组合系统：https://www.mt.com/cn/zh/home/products/Transport_and_Logistics_Solutions/dimensioning/dynamic-weighing-scanning-systems.html
- 知乎 DWS 精度（1mm / ±1g）：https://zhuanlan.zhihu.com/p/11089628495
- 海参体积测量（双目视觉，94% 准确率）：https://www.sciopen.com/local/article_pdf/10.11975/j.issn.1002-6819.202407228.pdf
- 基于机器视觉的水果尺寸检测误差分析：http://tcsae.org/cn/article/pdf/preview/20030131.pdf
- 故宫博物院 文物三维数字化技术：https://www.dpm.org.cn/ydyl/detail/258786.html
- Basler 体积测量应用（3D ToF 装箱）：https://www.baslerweb.com/zh-cn/use-cases/3d-warehouse-robotics-volume-measurement/

**商用扫描仪与深度相机**
- EinScan Rigil（0.04 mm 基础精度、体积精度 0.04 mm + 0.06 mm/m）：https://www.shining3d.cn/dm/shouchi/EinScanRigil.html
- 国产工业三维扫描仪价格区间（厂商博客，仅供参考）：https://www.insvision3d.com/cn/news/articles/512.html
- Revopoint POP 3（0.1 mm 准确度 / 0.05 mm 重复性）：https://www.revopoint3d.com/pages/portable-3d-scanner-pop3
- Revopoint MIRACO 售价：https://www.revopoint3d.com/products/standalone-3d-scanner-miraco
- Creality CR-Scan Otter（$899，0.02mm）：https://store.creality.com/products/cr-scan-otter-3d-scanner
- 基恩士 LJ-X8000：https://www.keyence.com.cn/products/measure/laser-2d/lj-x8000/
- RealSense D435 规格（Min-Z 28 cm，<2% @ 2 m）：https://www.realsenseai.com/products/stereo-depth-camera-d435/
- RealSense 深度相机调优（848×480 下 minZ 19.5 cm）：https://dev.realsenseai.com/docs/tuning-depth-cameras-for-best-performance/
- RealSense D405 短距（$259，7–50 cm）：https://www.therobotreport.com/intel-adds-short-range-realsense-d405-depth-camera/
- D405 官方页：https://www.realsenseai.com/products/stereo-depth-camera-d405/
- D405/D415/ZED-Mini 12–20 cm 第三方评测（台阶边缘无法分辨）：https://pmc.ncbi.nlm.nih.gov/articles/PMC10284995/
- Azure Kinect / Kinect v2 精度评测：https://pmc.ncbi.nlm.nih.gov/articles/PMC9002889/
- Orbbec Femto Bolt 与 Azure Kinect 参数对比（误差 17 mm / 11 mm）：https://www.orbbec.com/documentation/comparison-with-azure-kinect-dk/
- iPhone LiDAR 精度（>10 cm 物体 ±1 cm）：https://www.3dmag.com/3d-wikipedia/phone-3d-scanning-lidar-iphone-3d-apps-guide/

**开源 DIY 项目**
- 《An Open-Source, Low-Cost Solution for 3D Scanning》（$115，实测 1.3–2.56 mm）：https://pmc.ncbi.nlm.nih.gov/articles/PMC12788225/
- OpenScan 官网：https://openscan.eu/
- OpenScan Mini（$266 起、9×9×9 cm、0.02 mm 标称）：https://openscan.eu/pages/openscan-mini
- FreeLSS 开源扫描系统（含 ATLAS 3D）：https://3dprint.com/36628/freelss-open-source-scanner/
- FreeLSS 代码：https://github.com/hairu/freelss/
- BQ Ciclop（0.5 mm、250 mm×205 mm 扫描区）：https://lulzbot.com/learn/meet-the-bq-ciclop-3d-scanner
- Sardauscan（$30 DIY，含 BOM）：https://3dprinting.com/how-to/sardauscan-30-usd-diy-3d-scanner-3d-print/
- Sardauscan 代码：https://github.com/Sardau/Sardauscan
- SLStudio 结构光软件：https://github.com/jakobwilm/slstudio
- SL Sensor（ETH）：https://github.com/ethz-asl/sl_sensor
- scAnt（开源摄影测量，€900 vs €5,000–8,000）：https://pmc.ncbi.nlm.nih.gov/articles/PMC8048404/
- 结构光相机投影设备分类（知乎）：https://zhuanlan.zhihu.com/p/1894343056540558986
- slm3D 单目结构光教学套件（DLP2010 + 海康相机）：https://blog.csdn.net/slm3D/article/details/134098125

**竞赛资料**
- 竞赛官网（第十五届赛题公布 2025-07-21）：https://gd.moocollege.com/
- 《第十五届全国大学生光电设计竞赛赛题》PDF（含本题评分细则）：https://jwb.bit.edu.cn/docs//2026-09/c1fde5a1a91e44a09562c34324feec15.pdf
- 复旦大学：第四届全国大学生光电设计竞赛赛题公布（2014-02-27）：https://optics.fudan.edu.cn/2c/ef/c39848a470255/page.htm
- 中国光学学会光学教育专业委员会文件（第四届全国赛正式题目）：https://www.phy.sdu.edu.cn/__local/7/63/42/9ADE24081078E8876894DF9FF00_A2B950B5_24B60.pdf
- 科学网：第四届全国大学生光电设计竞赛在国防科大举行：https://news.sciencenet.cn/sbhtmlnews/2014/8/290706.shtm
- 中南大学：第四届全国大学生光电设计竞赛总结：https://wl.csu.edu.cn/info/1096/2312.htm
- 电赛历年题目题库（含 2025 C 题、2020 G 题）：https://github.com/CCBP/NUEDC_Topic
- 四川大学：第十四届全国赛通知（含"取消参赛权益"条款，针对抄袭/虚假材料）：https://eie.scu.edu.cn/info/1024/14858.htm
