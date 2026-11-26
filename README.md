# 单目结构光转台的多视角拼接与缺面重建

本分支记录赛题二的单相机、投影仪与步进转台实验。`algorithm/` 保存从标定、单视角点云到整圈拼接的代码和实测结果。`simulation/` 保存遮挡模拟、实测缺面补全、网格优化和体积评估。

## 从哪里看

- **看完整汇报**：先读 [E001–E104 实验汇报（PDF）](output/pdf/多视角点云拼接与缺面重建_E104拼接与网格优化.pdf)。第 2 页是目录，第 8–11 页看早期配准结果和反例，第 13–21 页看新标定、裁剪与拼接，第 22–27 页看缺面重建、体积和人偶网格对照。
- **看实际点云**：从[四类物体的点云与报告](algorithm/point%20cloud/README.md)找早期样本。新批次从 [E104 拼接说明](algorithm/scene-fusion/E104/README.md)进入，在 `ChArUco/` 或 `Chessboard/` 下按物体找 `推荐.ply`、`指标.csv` 和 `鲁棒性.json`。E101 的 `预览.jpg` 可以在 GitHub 上直接看。下载 PLY 后可用 CloudCompare 或 MeshLab 旋转查看。
- **看补全和网格**：在 [E104 补全说明](simulation/E104/README.md)查场景和编号，从[推荐索引](simulation/E104/推荐.json)定位 `场景/` 下的 PLY，用[逐模型指标](simulation/E104/补全指标.csv)核对留出误差、闭合和体积。从[坐姿人偶网格优化](simulation/E104/网格优化/README.md)查看原候选、接底和去阶梯对照。
- **看代码与复跑方法**：读[拼接总说明](algorithm/README.md)、[全场景复跑说明](algorithm/scene-fusion/README.md)和[补全仿真说明](simulation/README.md)。参考论文存放在 [`algorithm/paper/`](algorithm/paper/)，体积误差专项见 [M01](algorithm/Math/M01/README.md)。

## 实验做了什么

| 实验 | 拼接与采集 | 仿真、补全与体积 |
| --- | --- | --- |
| E001 | [早期八组扫描](algorithm/scene-fusion/E001)保留完整场景和回零记录。通过八角、四角对照检查重叠不足对拼接的影响。 | [合成物体基准](simulation/README.md)用已知体积的 A 类长方体、B 类带孔槽组合体和 C 类自由曲面对照视角、遮挡与体积偏差。 |
| E002 | [去转盘与配准](algorithm/scene-fusion/E002/README.md)比较转轴先验和多种 ICP，用方位缺口识别苹果局部贴合但整圈塌缩的情况。 | [实测点云补全](simulation/E002/README.md)对比凸包、高度场、Poisson 和形状先验。通过人工遮挡已测区域检验恢复。 |
| E003 | [多视角拼接](algorithm/scene-fusion/E003/README.md)比较去噪、特征粗配准与约束精修，以留出标记和深度核对偏心纸盒等案例。 | [局部补全与规则化](simulation/E003/README.md)按物体补脚蹼、曲面、局部平面或盒面，并保留实测点、引导点和网格。 |
| E101 | [新批次全场拼接](algorithm/scene-fusion/E101/README.md)在同一套五组、54 个视角上比较 ChArUco 与棋盘格光学标定。 | |
| E102 | [前景裁剪](algorithm/scene-fusion/E102/README.md)剔除外部背景，保留棋盘格、物体和露板数据。 | 露板数据用于 E104 的底面与支撑核验。 |
| E103 | [物体分割与配准](algorithm/scene-fusion/E103/README.md)去棋盘格并保留可见底部，对照分割及配准效果。 | 物体点云进入 E104 补全。 |
| E104 | [共轴初始化与稳健精修](algorithm/scene-fusion/E104/README.md)比较角站减半、噪声、遮码和虚假环边等压力条件，根据留出证据选位姿。 | [缺面重建](simulation/E104/README.md)比较 Poisson、TSDF、局部补全与形状先验，核验贴底和体积。[人偶网格实验](simulation/E104/网格优化/README.md)比较局部换底、双边滤波和 Taubin 平滑。 |
| M01 | [体积误差专项](algorithm/Math/M01/README.md)考察标定尺度、位姿同步和表面偏差。 | 使用已知体积控制件和实测样本，比较截面积积分与封闭网格体积。 |

## 阅读结果与复跑

- `PLY` 视文件可能是实测点云、补全点云或三角网格。各实验 README 说明编号和盘面坐标约定。旧批次的 `mm*` 沿用对应档案的标定尺度。
- 配准时结合方位缺口、邻角 F1 和留出深度判断。补全时结合留出表面误差与网格拓扑判断。实物体积随候选表面和形状假设而变，已知体积的仿真组用于对照误差。
- 阅读已有结果可直接使用本分支文件。复跑时在仓库根目录按[环境说明](algorithm/README.md#5-源码和运行办法)重建 `algorithm/.venv`。依赖原始图像、标定档案或其他本地中间结果的阶段需先按各实验 README 准备输入。原始 `data/` 的共享位置沿用下方原仓库说明。Open3D、OpenCV 与 OpenCV Contrib 的本地参考检出可按环境说明重建。

## 原仓库说明

1.to-do：联系顿雄，阅读其论文方向

2.学习B站三维结构光/三维重建基础

3.盘点器材，组装

## 结构光实验数据

原始采集图像和完整试验数据存放在[同济云盘](https://yunpan.tongji.edu.cn/link/AAAE071E5B2DF34896A8C3A9EE4046624D)，不提交到 Git。链接允许预览和下载，有效期至 **2026 年 12 月 15 日 15:54（北京时间）**，到期后需要重新分享。较小的试验参数、点云及预览图保存在 [results/structured-light](results/structured-light/README.md)。

压缩包解压到项目的 `data/` 目录后，会合并到 `data/procam_calibration/` 或 `data/object_scans/`：

| 文件 | 内容 | SHA-256 |
| --- | --- | --- |
| `01_早期采集.zip` | `之前的/` 中的早期采集 | `996cddf14930a87295a656aa0780dbc2b171eee62fd5c29de8558e357a1e545d` |
| `02_标定图片.zip` | 11 组 12:12～12:19 标定板采集 | `f8fe2fe3e448a5950945ee17017a5d5d4de04c2f136eeb914e5ca193ad3dd4ee` |
| `03_物体与结果.zip` | 物体采集、点云和试验结果，包括未验证的标定参数 | `90f9e6bf0bd8af6cb4772d875a61d19c4508d9c504a3aadfc192ebbb7ef4d35b` |
| `20261116_calibration.zip` | 11 月 16 日的 16 组棋盘格采集、标定参数及验证记录 | `125ec0813ce11713329bff2a0e37db5a6caf5fc6a39827effa7b509276e6ed64` |
| `20261116_object_scans.zip` | 11 月 16 日的 6 组物体和转台采集及其试验结果 | `a65469832303abd033069e4d696a9722db5e38860b70b33926f2d50f1fea01dd` |

这些数据和标定结果属于试验记录，不能直接作为已验证的尺寸或体积测量结果。操作步骤见 [结构光扫描使用说明](reports/07-结构光扫描使用说明.md)，标定与误差分析见 [技术说明](reports/08-结构光标定与误差技术说明.md)。
