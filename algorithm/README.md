# 单相机 Gray code 多视角拼接

**现阶段建议用 ChArUco 或多枚 ArUco 板确定机械位姿，以实测转轴和角度刚性回转，先验收拼接，再给通过验收的重叠区做小幅、绕轴约束的 ICP。** 每一帧直接变换到参考角，便于量化误差和定位缺测。优先拿哑光不对称标准块做整圈，六棱柱做解码压力测试，鸡蛋做对称性测试。后续体积线路仍可并行推进。

## 实验与结果入口

这部分覆盖从 Gray code 三角化和转轴标定，到去背景、去板和多视角配准。完整实验顺序和图表见 [E001–E104 汇报](../output/pdf/多视角点云拼接与缺面重建_E104拼接与网格优化.pdf)，配准后的补全、网格和体积结果见 [simulation](../simulation/README.md)。

| 实验 | 做了什么 | 从哪里看 |
| --- | --- | --- |
| E001 | 在八组早期扫描上复跑全场拼接，比较角站减半、同批轴标定和独立回零。 | [全场预览](scene-fusion/E001/figures/full_scenes.png)、[配准评分](scene-fusion/E001/benchmark.csv)、[各组全场点云](scene-fusion/E001/clouds) |
| E002 | 在去转盘的点云上比较轴先验与多种 ICP，检查局部重叠改善时整圈方位是否仍成立。 | [实验说明](scene-fusion/E002/README.md)、[评分与点云](scene-fusion/E002/results) |
| E003 | 对照去噪、特征粗配准和约束精修，用留出码与深度检查偏心纸盒等案例。 | [实验说明](scene-fusion/E003/README.md)、[评分与点云](scene-fusion/E003/results) |
| E101 | 用同一套五组、54 个相机视角比较 ChArUco 与棋盘格光学标定，保留整幅有效点。 | [目录与复跑说明](scene-fusion/E101/README.md)、[索引](scene-fusion/E101/E101索引.csv) |
| E102 | 剔除外部背景，保留物体和棋盘格。露出的板面供后续估计贴底高度。 | [实验说明](scene-fusion/E102/README.md)、[汇总](scene-fusion/E102/汇总.csv) |
| E103 | 从 E102 分割物体，保留可见底部，对照分割方法和配准结果。 | [实验说明](scene-fusion/E103/README.md)、[汇总](scene-fusion/E103/汇总.csv) |
| E104 | 从共轴位姿开始做稳健精修，评估角站减半、噪声、遮码和虚假环边。 | [方法与新扫描入口](scene-fusion/E104/README.md)、[按标定和物体查看结果](scene-fusion/E104) |
| M01 | 比较标定尺度、位姿同步和表面偏差对体积的影响，并测试截面积积分。 | [M01 说明](Math/M01/README.md)、[结果与图](Math/M01/results) |

找早期物体时，先在[四类物体索引](point%20cloud/README.md)按采集场次定位 `单视角/` 与 `拼接/`。找新批次的推荐结果时，在 E104 的 `ChArUco/` 或 `Chessboard/` 下选择物体，同组对照 `推荐.ply`、`指标.csv` 和 `鲁棒性.json`。可先看预览图，再把 PLY 下载到 CloudCompare 或 MeshLab 查看。配准评分结合邻角 F1、留出深度和方位缺口来读。

完整复跑需要各实验说明所列的原始图像、标定档案或本地中间产物。原始采集的共享位置见[仓库说明](../README.md#结构光实验数据)。只查看本分支的点云、图和指标表时，可直接使用上述目录。

## 1. 赛题对方案的影响

依据 2026-11-17 第一轮通知，赛题 2 在 PDF 物理页 9–13。布设 10 分钟，正式测量另计 10 分钟，A/B/C 各一件并提交一次结果。目标为 50–120 mm 喷砂哑光铝件，C/B/A 权重分别为 0.5/0.3/0.2。B 类槽口与孔口尺寸至少 15 mm，深度不大于 5 mm，特征从上方可见。B 和 C 有平底，C 上表面连续且无垂向内倾遮挡。通孔、凹槽和盲孔的空缺须扣除。评分使用现场排水标准体积，提交值保留到 0.1 mm³。比赛现场离线，整机 500 mm 立方包络和 10 kg 上限按细则规定计入设备。物体表面不可贴标、喷涂或加辅助结构，测量端不得接触目标，测量期间不得触碰目标或手动调整其姿态。

商用三维扫描仪及可直接输出深度、点云、三维模型或体积的成熟商用模块受赛题限制。现有普通相机与投影仪可作为自研光学头的组成器件，最终系统以赛区检录为准。

- **实验室**用现有步进转台把采集、轴标定和融合跑通。标记装在台面或独立标定件上。
- **赛场布局**优先考虑目标固定，刚性相机—投影仪光学头沿弧线移动或在可复现位置锁定，支撑面外布置固定标记板，或逐站预标定位姿。自动旋转载物台是否可作为承载台面，需要赛区书面确认，规则没有明确点名该机构。先给高位视角，测 B 类凹槽和孔底。用低一些的斜视补外侧壁。两者都要检查可用视场、遮光、机构与线缆的运动包络。
- **体积方向**可同步做 A 类的稳健几何拟合，以及 B/C 类的高位深度加已标定底面。对有有效投影域且每条竖线实体为连续一段的区域，可积分厚度。B 类通孔、竖壁或高度不连续区域需单独分区，区分真实孔洞与无回波，并用斜视或明确的边界模型验证。高位视角对最终体积的贡献可先用已知体积试块量化。

## 2. 现有点云审计

检查了历史结构光结果的 17 个 PLY 与各组预览。归档文件见 [results/structured-light](../results/structured-light)。11 月 20 日各组 PLY 头写明使用假定轴线和名义角度，文件已合并成按颜色区分的实验点云。逐文件点数和色组数量见 [result-audit.tsv](result-audit.tsv)。四种物体的点数如下。

| 物体 | 忽略 0 级 | 忽略 1 级 | 忽略 2 级 | 观察 |
|---|---:|---:|---:|---|
| 六棱柱 | 138 | 758 | 2,248 | 拼图四个角度，0 级各组只有 10–57 点。首先补单视图有效点 |
| 鸡蛋 | 4,731 | 12,903 | 19,445 | 最稀疏色组只有 20/36/45 点，整体加密也补不回这个视角 |
| 苹果 | 3,947 | 18,155 | 60,754 | 点数对档位敏感，需对比重复扫描厚度和离群率 |
| 小黄人 | 24,508 | 44,599 | 63,905 | 拼图标出八个角度，文件名从 0002 跳到 0004，PLY 每档各有七个色组。下次保存帧号、角度、RGB 与拒收记录 |

11 月 13 日的盒子预览可见平面错层。11 月 16 日的轴拟合预览已有较清楚的多个盒面，同时还有边缘散点和缺口。把 `loop_0000_0004_before.ply` 和 `loop_0000_0004_fitted.ply` 分别按 RGB 分成 30,771 / 36,187 两组，合并两个方向的最近邻距离，得到以下结果。坐标单位沿用配套图标注的 mm。

| 重复采集点云 | P50 (mm) | P90 (mm) | P95 (mm) |
|---|---:|---:|---:|
| 拟合前 | 3.259 | 6.297 | 6.826 |
| 拟合后 | 0.398 | 0.682 | 0.800 |

该对数据说明首尾两次扫描的重合得到改善。相同数据已经用于历史拟合，当前缺少可独立评价的原始单视角 PLY、角度日志、原始 Gray code 帧与标定参数，同批结果无法给出绝对精度、体积误差、缺点根因或全圈配准精度。颜色仅用作该批导出的分组线索，无法反解原始相机位姿。

### 怎样分辨缺点来自采集还是拼接

1. 先固定一个站位、固定曝光/增益，保存全白、全黑、正反 Gray code 帧和相机时间戳。对目标 ROI 分别画全白减全黑、各正反图差值的最小值、饱和掩膜、解码有效掩膜、三角化保留掩膜。筛选门槛由重复数据和试块确定。海康相机若支持软触发先用 SDK 软触发，型号支持时再考虑 Line 触发。澳典 M8 的分辨率、显示延迟和是否支持同步接口需在实机测量。用黑白交替实测画面变化并确认图案已经稳定，留出捕获余量，排除切帧混拍。可关闭的自动梯形修正、缩放、动态曝光和动态增益应在标定前锁定。
2. 对光学状态相同的哑光平板、已知尺寸台阶和六棱柱做 0/1/2 级对照。记录各视角点数、目标有效覆盖、平面 P95、边缘位置和台阶深度。如果“忽略级”实际指丢最低有效码位，忽略 $k$ 位相当于把相应轴的投影地址分到 $2^k$ 像素宽的码组，具体要查自有解码实现。用同拍正反图的亮度差、相机与投影仪重投影误差来验证。可用高位 Gray code 解绝对条纹序号，再用 3/4 步相移求亚像素投影坐标。做相移前先验证定帧、亮度响应及是否饱和。
3. 台面和夹具先用空台标定平面、测量 ROI 或空台掩膜剔除。按视角保存相机系 PLY、视角号、有效掩膜、深度图或有组织点云、投影设置、相机模型与单位、位姿和标记图，留到融合之后再体素降采样。把假彩色标成视角索引。调机时用哑光平板和喷砂铝试块模拟规则材料。

[OpenCV GrayCodePattern 4.13 文档](https://docs.opencv.org/4.13.0/d1/dec/classcv_1_1structured__light_1_1GrayCodePattern.html) · [Brown 投影仪—相机标定](https://mesh.brown.edu/calibration/) · [HIKROBOT 官方下载中心](https://www.hikrobotics.com/en/machinevision/service/download/?module=0)

## 3. 步进转台与轴心标定

### 电机与投影序列

- 现阶段用 **回零 → 同向转到绝对步数 → 减速停稳 → 拍标记 → 完成 Gray code 正反帧及黑白参考 → 下一站**。让 MCU 定时器或步进驱动生成 STEP/DIR 脉冲，根据电机带载情况设置梯形或 S 曲线加减速。台面有间隙时始终同向接近目标角。光电或霍尔开关提供重复零位，编码器若添加应尽量放在台面输出轴端，便于看见传动回差与漏步。微步主要用于平滑，定位精度按光学验收。
- 设 $N$ 为每台面一圈的有效脉冲数（电机整步数 × 微步倍率 × 电机转数/台面转数）。目标角 $\theta_i$ 的绝对步数用 `round(N * theta_i / 360)`。相邻目标取两个绝对步数之差。采用 200 步/转、1/16 细分和直驱时是 3200 脉冲/圈，30° 对应累计 267 步，60° 对应 533 步，整圈 3200。假定 30° 扫描用 12 个有效角，再采一帧 360° 独立比较，名义角保留在日志中，拼接优先用视觉实测角。重叠不足时再试 15° 或加一圈高位。
- 匀速段脉冲频率 $f$ 与台面转速的关系为 `rpm = 60 * f / N`，例如 3200 脉冲/圈下 5 rpm 对应约 267 Hz。用定时器或驱动器保证脉冲间隔和最短高低电平宽度，按驱动器手册设 STEP/DIR 时序。以多次带载角度复测评估是否漏步，再逐档增速。若将来考虑连续旋转，序列中目标转过的角度约为 $\omega T_{\text{seq}}$，需要整组图案与相机精确同步或进行运动补偿。
- 用贴在转台边缘的标记试测不同速度和载荷。每一站停后连续拍 3–5 张，记录角点相对位移何时趋稳、PnP 实测角是否稳定，取大于复测停稳时间的等待值。先以 200–500 ms 做试验，最终参数由台面振动和采样频率定。跑多圈同向回零，比较逐站实际角度、径向位移、端跳和 0°/360° 差。逆向一圈只用于诊断回差。同圈转速的波动主要影响节拍和失步概率，整组曝光期间的位姿稳定才直接决定 Gray code 一致性。
- 角度误差在半径 $r$ 的一阶位移约为 $r|\delta\theta|$，角度用弧度。半径 60 mm、角差 0.2° 时约 0.21 mm，可按你们的尺寸误差预算反推容许角差。若半径方向的轴心估计误差为 $e$，按 180° 回转会造成最多约 $2e$ 的偏移。用时间戳计时。举例 1024×768 投影时双向 Gray code 含正反帧 40 张，加白黑至少 42 张。若实测每张稳定和抓取共 80 ms，12 站约 40 s/件，仅含投影抓取，另计转动、停止、计算和提交。实机测得的 M8 输入像素与稳定时间应代入。

[AccelStepper `moveTo` 和 `run` 参考](https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html) · [OpenCV Gray code 采集教程](https://docs.opencv.org/3.4.18/db/d56/tutorial_capture_graycode_pattern.html)

### 标记先验与固定轴

1. **准备板和坐标。** 锁定相机对焦、分辨率与工作距离。用独立多姿态校准图确认 $K,D$。单相机、单投影仪标定把投影仪作为逆相机，在多姿态平面板上取得对应投影像素，用 `calibrateCamera`/ `stereoCalibrate` 定相对外参，用去畸变射线或配套投影矩阵做三角化。转轴标定只需相机与实体标记板。把已量过方格大小和姿态的硬质板固定在空转台，板平放且让角点覆盖多个离轴半径，或在台面边缘刚性安装多个标记。若用纸板，检验平整度、打印缩放和实测间距。
2. **求每站刚体位姿。** 在稳定均匀照明下采 12 个测试角、另存首尾重复帧。在 OpenCV 用 `cv2.aruco.CharucoDetector(board).detectBoard(image)` 返回角点和 ID，再用 `board.matchImagePoints(corners, ids)` 和 `cv2.solvePnP` 求板到相机 $T_{C\leftarrow B_i}$。板至少需要多点且不共线，角点要铺开。平面姿态有歧义时可用 `solvePnPGeneric(..., flags=cv2.SOLVEPNP_IPPE)` 比较候选，用正深度、重投影、相邻站的角度连续性及标定靶三维方位筛选。保留每站角点数量、重投影 P50/P95 和位置。若每次正式扫描时都能看到台面标记，可先用直接位姿法 $T_{C\leftarrow B_0}(T_{C\leftarrow B_i})^{-1}$ 拼同一轮的数据，作为固定轴法的独立对照。相机旋转而目标及标记板固定时，同样的矩阵组合可把相机系点对齐到首站。
3. **拟合轴线。** 把每个固定角点在不同角度的板坐标 $b_j$ 变换成 $p_{ij}=R_i b_j+t_i$。在相机系，每个离轴角点沿与轴正交的平面画圆。对轨迹先用 SVD 拟合平面，在平面中拟合圆，使用覆盖角范围大、半径大、重投影好的轨迹。各圆的法向共线，圆心都在同一轴线上。可把各轨迹中心化并按轨迹归一化后合并，再用加权 SVD 估共同法向。圆心落到同一条线才算通过。明显离群的轨迹回看原始角点。进阶可用 Huber 联合拟合所有图像角点、共享轴和各站角度，用同一份未参与拟合的图像对照。轴符号与电机正转约定一致，用 30° 和 90° 的观察确定正负方向。每站实测转角可用校正后离轴向量的 `atan2(u·(v0×vi), v0·vi)`，多个角点做圆周统计。这里 $v_i$ 是相对轴的径向向量。
4. **定义转台中心。** 轴是直线 $c+s u$，沿轴任一点都能用于回转。要报告台面中心需用空台标定的台面 $n^T x+d=0$ 与轴求交，得到 $c-rac{n^T c+d}{n^T u}u$。用已知板厚、实际安装垫高和深度单位还原台面高度。平面与轴近似垂直是正常几何，若分母接近零要回查平面或轴。按各位置标记轨迹的轴向起伏报告端跳，径向残差的 P95 可提示径向摆动。结合重复角及独立圆柱确认轴线可复现。
5. **把视角回到参考位姿。** $x_0=c+R(u,-(\theta_i-\theta_0))(x_i-c)$。这一步要求各 PLY 都还在同一台固定相机坐标系。标定后的第 1 圈可用 0/360 度验证，第二圈至少留出几个未参与定轴的角度和首尾重复。移动相机—投影仪支架后重做相对位姿检查，尤其复核镜头与投影焦距改动后的投影标定。

[Sadlo 等的相机—投影仪—转台实验论文](https://cgl.ethz.ch/Downloads/Publications/Papers/2005/Sad05/Sad05.pdf) · [Man 等的转台标定](https://doi.org/10.1587/transinf.2019edp7043) · [Zhu 等的解析标定](https://doi.org/10.1109/AIM.2019.8868360) · [OpenCV ChArUco 检测与姿态](https://docs.opencv.org/4.13.0/df/d4a/tutorial_charuco_detection.html)

现有 ArUco 码若已经装成刚性板，可先用 `ArucoDetector.detectMarkers` 和 `Board.matchImagePoints` 接同一 PnP 流程。使用棋盘格的批次可以用 `findChessboardCornersSB`，并核对每个角度的角点编号方向一致。可在现场采集前先保存一张实际标定板的正视图、板型、字典/方格数和毫米尺寸，避免把不同板的尺寸混用。

## 4. 算法比较与落地

| 方法 | 对你们装置的适用性 | 开源实现 |
|---|---|---|
| 每站标记直接变换 | 最快做出可对照的拼接。可覆盖有轻微摆动的实际姿态，要求各站标记角点足够、PnP 稳定 | [OpenCV](https://github.com/opencv/opencv) |
| 固定轴与实测角 | 当前主线。转轴和角度一起锁住全圈尺度与姿态，先判断硬件和原始深度的误差 | [OpenCV](https://github.com/opencv/opencv) · [Open3D](https://github.com/isl-org/Open3D) |
| 从目标点云反推固定轴 | Pang 等先粗配准，再用整圈配准约束估轴。可以处理没有标记的旧数据，要求有未拼接序列且目标形貌足以估计相对运动 | [Pang 等原论文](https://doi.org/10.1109/MCG.2014.83) · 可用 Open3D 组装配准步骤 |
| 轴约束的稳健 point-to-plane ICP | 轴法通过后用于小幅修角。需要可观测的正常重叠，圆柱和近球的角度要依靠标记 | [Open3D 稳健 ICP](https://github.com/isl-org/Open3D/blob/v0.20.0/examples/python/pipelines/robust_icp.py) · [SciPy](https://github.com/scipy/scipy) |
| GICP / 彩色 ICP | GICP 用两侧局部协方差，适合对照斜视导致的局部噪声方向差。彩色 ICP 需要实际一致的纹理。本批 RGB 是视角标签，且赛题件灰色喷砂 | [GICP 原作者实现](https://github.com/avsegal/gicp) · [Open3D 彩色 ICP 示例](https://github.com/isl-org/Open3D/blob/v0.20.0/examples/python/pipelines/colored_icp_registration.py) |
| FPFH + RANSAC、FGR | 缺位姿的补拍或多个安装工位对齐时做对照。先采法向、提 FPFH，再做粗配准与局部优化。弱几何或重复棱边时容易找到多个角度，需用机械角或标记判别 | [Open3D 全局配准](https://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html) |
| TEASER++ | 特征对应中有大量离群时可作为脱离机械先验的备用方案。依赖对应点和合理的噪声尺度，几何对称仍有位姿歧义 | [MIT-SPARK/TEASER-plusplus](https://github.com/MIT-SPARK/TEASER-plusplus) · [原论文](https://arxiv.org/abs/2001.07715) |
| 位姿图优化 | 扩展到跨仰角、跨工位时有用。节点由外部位姿初始化。只加入通过双向验证的边。首站固定。首尾重复边单独校验。将机械或标记先验直接加入目标函数需要自行扩展或使用轴约束优化 | [Open3D 多视角](https://www.open3d.org/docs/latest/tutorial/pipelines/multiway_registration.html) · [Choi 等 CVPR 2015](https://openaccess.thecvf.com/content_cvpr_2015/html/Choi_Robust_Reconstruction_of_2015_CVPR_paper.html) |

### 实际配准顺序

1. 每站先用相机系 ROI 和台面掩膜清掉背景，在小体素上估计法向。用物理轴或标记位姿得到各站到参考系的 4×4 初值。对 50–120 mm 试块可先比较 0.5 mm 和 1.0 mm 体素，法向半径从约 2–4 倍体素起试，根据邻点数与试块边缘调整。用首尾重复和相邻两帧各跑一遍，导出覆盖、双向匹配比例和固定阈值下的 P50/P95。
2. 做 Open3D 对照时，原始源→目标初值是 `inv(T_target_to_ref) @ T_source_to_ref`。如果两帧已经变换到公共坐标，则传 `np.eye(4)`。对双向近邻已证实有重叠的边，先粗后细试 `registration_icp` 与 `TransformationEstimationPointToPlane(HuberLoss(k))`。最大对应距离可从粗 2–3 倍体素起试，结合首尾重复的误差水平收紧。稳健核参数用同一单位，并由配准残差标定。对照结果同时保留 `fitness`、`inlier_rmse`、双向覆盖和与先验位姿的差。厚度、棱边或已知尺寸变差就回到先验。标准 Open3D 这个调用会估计六自由度，适用于基线对照。
3. 实际优先用 **单自由度精修**。保持固定轴与轴心，每次用已对齐的重叠区做近邻匹配与法向门控，只拟合每站的小角修正 `delta_i`。在共同参考系将每站的角修正写成绕轴的逆旋转。最小化点到面的 Huber 残差并加 `(delta_i / sigma_angle)^2` 先验，锁定第一站，加入经过验收的首尾及相邻边。每轮先固定对应和参与点，再用 `scipy.optimize.least_squares(loss="huber", bounds=...)` 求修正后更新对应。如需先验项保持严格二次，可在外层做 IRLS 并仅对几何残差加稳健权重。修正界限由视觉角复测确定。只有单一平面、光滑旋转体或匹配比例太低时，保留外部位姿，由未参与拟合的重复帧或试块边缘评价。
4. 融合时保留各视角贡献，按 Gray 置信度、入射几何与局部一致性筛选后出一版统一点云。保留原始深度、内参和相机位姿时可做 TSDF。若每站融合到物体参考系的初始相机变换是相机到世界，使用 Open3D TSDF 时传其逆作为世界到相机外参。独立点云可先以定向法向做 Poisson 预览，检查低密度外插与孔/槽被封口的风险。B 类优先保留开孔边界、孔壁和底面，应用明确拓扑约束。计算网格体积前检查闭合、流形、法向一致和自交。用已知尺寸或体积的试块验证不同封闭方式的体积差。

[Open3D 稳健核](https://open3d.org/docs/latest/tutorial/t_pipelines/t_robust_kernel.html) · [Open3D 表面重建](https://www.open3d.org/docs/latest/tutorial/geometry/surface_reconstruction.html) · [PoissonRecon](https://github.com/mkazhdan/PoissonRecon) · [Curless–Levoy 体融合](https://graphics.stanford.edu/papers/volrange/)

## 5. 源码和运行办法

只克隆了当前入口所需的三个 Git 仓库，使用浅克隆和 sparse checkout。它们可用于查 API、源码和例程。执行 Python 时用已发布的二进制包。环境建议使用 Python 3.14。版本和克隆内容如下。

| 本地目录 | 版本与提交 | 先看 |
|---|---|---|
| `Open3D` | v0.20.0 · `b6c5e196384ad71e75b6e6f9c5da22d046221f1d` | [robust_icp.py](https://github.com/isl-org/Open3D/blob/b6c5e196384ad71e75b6e6f9c5da22d046221f1d/examples/python/pipelines/robust_icp.py) · [multiway_registration.py](https://github.com/isl-org/Open3D/blob/b6c5e196384ad71e75b6e6f9c5da22d046221f1d/examples/python/pipelines/multiway_registration.py) · [Poisson 示例](https://github.com/isl-org/Open3D/blob/b6c5e196384ad71e75b6e6f9c5da22d046221f1d/examples/python/geometry/triangle_mesh_from_point_cloud_poisson.py) |
| `opencv` | 4.13.0 · `fe38fc608f6acb8b68953438a62305d8318f4fcd` | [ChArUco 教程](https://github.com/opencv/opencv/blob/fe38fc608f6acb8b68953438a62305d8318f4fcd/doc/tutorials/objdetect/charuco_detection/charuco_detection.markdown) |
| `opencv_contrib` | 4.13.0 · `d99ad2a188210cc35067c2e60076eed7c2442bc3` | [Gray code 实现](https://github.com/opencv/opencv_contrib/blob/d99ad2a188210cc35067c2e60076eed7c2442bc3/modules/structured_light/src/graycodepattern.cpp) · [投影仪标定样例](https://github.com/opencv/opencv_contrib/blob/d99ad2a188210cc35067c2e60076eed7c2442bc3/modules/structured_light/samples/projectorcalibration.cpp) |

单目加投影仪可继续使用自有解码器，也可参考 contrib 的 `GrayCodePattern.getProjPixel()` 产出相机像素到投影像素对应。4.13 源码中其布尔返回值表示是否出错，白图减黑图的阴影判定需由单目流程应用。官方教程中 3DUNDERWORLD 的 `decode()` 示例采用双相机视差。你们用相机和已标定投影仪射线三角化。更多可对照入口见 [Moreno–Taubin 原始项目](https://mesh.brown.edu/calibration/) 和 [INESCTEC 单相机采集与标定开源实现](https://github.com/inesc-tec-robotics/carlos_projector_calibration)。

三个上游目录由各自的 Git 仓库管理。其他工作站可从项目根目录用以下命令复现。

```bash
git clone --depth 1 --filter=blob:none --sparse --branch v0.20.0 \
  https://github.com/isl-org/Open3D.git algorithm/Open3D
git -C algorithm/Open3D sparse-checkout set examples/python/pipelines \
  examples/python/geometry docs/jupyter/pipelines docs/jupyter/geometry cpp/open3d/pipelines/registration
git clone --depth 1 --filter=blob:none --sparse --branch 4.13.0 \
  https://github.com/opencv/opencv.git algorithm/opencv
git -C algorithm/opencv sparse-checkout set modules/objdetect/src/aruco \
  modules/objdetect/include/opencv2/objdetect modules/calib3d/src modules/calib3d/include/opencv2 doc/tutorials/objdetect
git clone --depth 1 --filter=blob:none --sparse --branch 4.13.0 \
  https://github.com/opencv/opencv_contrib.git algorithm/opencv_contrib
git -C algorithm/opencv_contrib sparse-checkout set modules/structured_light
```

从项目根目录复查 PLY 或安装环境。该工作区已建 `algorithm/.venv`。

```bash
python3.14 -m venv algorithm/.venv
algorithm/.venv/bin/python -m pip install -r algorithm/requirements.txt
algorithm/.venv/bin/python algorithm/audit_result.py results/structured-light
```

Apple Silicon 上 Open3D 0.20.0 的 wheel 还依赖 `libusb`。本机已在项目的 `.venv` 内补入 Homebrew 1.0.30 的 `libusb` 并改为相对路径加载。其他 Mac 如果在导入时报相同错误，可在本机用 `brew install libusb` 提供 wheel 所需的依赖。相关平台问题见 [Open3D issue #7586](https://github.com/isl-org/Open3D/issues/7586)。

每站导出已经裁掉台面和标记的原始单视角 PLY。把以下示例存为一份 `scan.json`。数值只表示字段格式，替换为实际标定的相机坐标。相对路径以清单目录为基准。按扫描顺序列出视角，第一个是参考。主融合保留 0° 到 330°，可另建只含 0°/360° 两帧的清单比较重复性。

```json
{
  "unit": "mm",
  "axis": [0, 1, 0],
  "center_mm": [12, 57, 310],
  "views": [
    {"file": "views/000.ply", "angle_deg": 0.0},
    {"file": "views/001.ply", "angle_deg": 29.97}
  ]
}
```

```bash
algorithm/.venv/bin/python algorithm/stitch_turntable.py scan.json /tmp/scan-aligned \
  --voxel-mm 0.5 --gate-mm 1.5
```

输出为 `merged.ply` 和 `overlap.json`。JSON 给出各站刚体变换和相邻、首尾两个方向的阈内覆盖比例与距离分位。距离分位是阈内统计，同读覆盖比例才有意义。脚本实现固定轴基线，局部 ICP 可以用前一节的源码入口与约束设计做下一组对照。正式比赛前在离线机器实测整个流程，预下载依赖和试验用数据。

本机运行验证使用 Open3D 0.20.0、OpenCV 4.13.0 和 SciPy 1.18.1。合成的三个倾斜轴视角恢复到同一参考系，融合 1,600 个顶点，边覆盖率分别为 50%、50%、0%，与生成数据设定一致。另在生成的 ChArUco 图上运行了检测、点匹配与 PnP，以及在生成的 Gray code 序列上跑通 `getProjPixel`。这些检查验证坐标变换和 API 调用，实物的多视角精度需要独立采集。

## 6. 视频与课程

| 平台 | 视频或课程 | 对应任务 |
|---|---|---|
| CV 课程 | [RPI CVFX 课程](https://sites.ecse.rpi.edu/~rjradke/cvfxcourse.html) · [第 24 讲 Structured light scanning](https://www.youtube.com/watch?v=ZC7CIlAVkIA) · [第 26 讲 3D features and registration](https://www.youtube.com/watch?v=7b2OupI6gKw) | 先过相机—投影仪三角化和编码，再过 ICP、局部特征 |
| CV 实验教材 | [MIT CBA Active sensing / Scanning](https://fab.cba.mit.edu/classes/865.21/topics/scanning/03_active.html) | 结构光、采样和光机布置备查 |
| B 站 | [结构光系统标定（上篇）](https://www.bilibili.com/video/BV1NK4y1X7eG/) · [结构光系统标定实战（下篇）](https://www.bilibili.com/video/BV1anPszSEzc/) | 按编码和标定流程检查你们的设备输入与采图 |
| B 站 | [从零搭建一套结构光 3D 重建系统](https://www.bilibili.com/video/BV1V64y1h7LW/) · [高精度面结构光三维扫描仪 硬件+源码](https://www.bilibili.com/video/BV1EA4m15776/) | 参考自制光学头与支架的布局演示 |
| B 站 | [点云配准教程第九讲 ICP（上）](https://www.bilibili.com/video/BV1UM4m127D8/) · [基于 ICP 工件位姿估计 Open3D](https://www.bilibili.com/video/BV1wr4y1S7UB/) | 结合先验和最近邻看两帧示例 |
| YouTube | [From Point Clouds to 3D Poses](https://www.youtube.com/watch?v=CzbETzWgFrc) | Python + Open3D 的 ICP 演示 |
| YouTube | [Arduino controlled photogrammetry 3D scanner](https://www.youtube.com/watch?v=SAgiv4o8rxQ) | 参考步进转台、停走与采集节拍。视频的测量原理是摄影测量 |
| YouTube | [A Single-shot-per-pose Camera-Projector Calibration System](https://www.youtube.com/watch?v=fnrVDOhcu7I) · [作者源码](https://github.com/BingyaoHuang/single-shot-pro-cam-calib) | 参考相机—投影仪标定采集设计 |
| YouTube | [RangeVision PRO 的标定与转台演示](https://www.youtube.com/watch?v=08v_wrdQY4c) | 参考轴标定和靶板摆位。赛题对商用扫描仪的限制见第 1 节 |

视频链接按课程页或公开索引核对标题和主题。部分 B 站与 YouTube 页面限制自动访问。标定与配准的实现以论文、官方文档和本地源码为准。

## 7. 纸面证据

| 原始资料 | 查阅位置 | PDF SHA-256 |
|---|---|---|
| 第十五届竞赛第一轮通知 | 物理页 2–3 与 9–13，赛题 2 §1、§3、§5–6 | `919c641486199541b51d6dd6730e8eda9e628bc282546174f0a0550a1456e754` |
| [Sadlo 等 2005](https://cgl.ethz.ch/Downloads/Publications/Papers/2005/Sad05/Sad05.pdf) | 物理页 4，§3.5 用水平棋盘、整圈采样及虚拟相机位置拟合圆。本文采用同一几何的固定标记点轨迹。§4.1 和图 3 说明 Gray code 与相移组合 | `6be5cb7591cf3d7cc843751a4b8637cced359f1f41866f1892163573824f5e17` |
| [Man 等 2019 · DOI 10.1587/transinf.2019edp7043](https://doi.org/10.1587/transinf.2019edp7043) | 物理页 5–6，印刷页 1837–1838，§5.2–5.3、式 (16)–(19) 与图 8、10 给出圆与轴线拟合 | `d5fc3559d398bf91908879331819211d680d0d8af371e0336db13322d397ae51` |
| [Choi 等 CVPR 2015 原文](https://openaccess.thecvf.com/content_cvpr_2015/papers/Choi_Robust_Reconstruction_of_2015_CVPR_paper.pdf) | 物理页 4–5，§5 式 (1) 提出相邻与闭环边的位姿图目标 | `c8cd24a7e16e0e81b801bd7581b771bb446e9527646dc0c64b86faf365cbcccb` |
