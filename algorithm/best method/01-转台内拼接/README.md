# 01 · 同一摆放下的转台配准

使用同一轮采集的 ChArUco 四角、名义转角和相机坐标点云，先求 **R02 共轴位姿**，再尝试 **R06c 稳健精修**。所有输出统一到首站 V00 的海康相机坐标，单位 mm。

## 运行示例

从 `best method` 目录执行。先按[依赖说明](../README.md)安装环境。

```sh
python "01-转台内拼接/stitch.py" "01-转台内拼接/cases/pumpkin-turntable/scan.json" "runs/turntable-pumpkin"
```

[南瓜正放清单](cases/pumpkin-turntable/scan.json) 包含 10 个实际角站。[标定参数](cases/pumpkin-turntable/camera.json)、[第 0 站点云与像素](cases/pumpkin-turntable/stations/00.npz)、[第 0 站码点](cases/pumpkin-turntable/stations/00-tags.npz) 与清单放在一起。[参考整圈 PLY](cases/pumpkin-turntable/reference.ply) 来自 E202。复跑选择 R06c，导出 90,118 个彩色点。

| 源码 | 负责的步骤 |
|---|---|
| [stitch.py](stitch.py) | 读清单、校验数组、运行配准并导出 PLY 和 JSON |
| [camera_geometry.py](camera_geometry.py) | ChArUco 平面 PnP、相机投影、绕轴刚体矩阵与点云导出 |
| [turntable.py](turntable.py) | 共轴拟合、缺码角插值、位姿图与 R02/R06c 选择 |
| [registration_metrics.py](registration_metrics.py) | 多尺度 Tukey ICP、重叠 F1、码重投影、接缝和深度核验 |

## 输入格式

- 清单 `calibration` 指向相机 JSON，包含 `cam_int` 和 `cam_dist`。路径以清单位置为基准。
- `angles` 按实际采集顺序列出 `angle_deg`、`cloud` 和可选 `markers`。至少四站有有效码，码站最大缺口小于 190°。采一圈只用半开区间 ([0,360°))。
- 单站 `cloud` 为 NPZ，含 `xyz` N×3 相机坐标、`uv` N×2 像素坐标、`gray` N 个白场灰度、`shape` 为 `[高, 宽]`。可选 `height` 为盘面相对高度，`rgb` 为 N×3 的 8 位颜色。
- `markers` 为 NPZ，含 marker `ids` M 个、像素角点 `u` M×4×2、板面坐标 `xyz` M×4×3。每个标记的三维与像素四角次序相同。

## 方法和验收

设 (A_i) 将标定板坐标变到第 (i) 站相机。逐站的初始映射为 (M_i=A_0A_i^{-1})。共轴优化以单位轴 (u) 和轴上一点 (c) 约束所有站，绕轴转动 (R) 的平移为 (c-Rc)。

R06c 用 2.4、1.4、0.85 mm 三个尺度做 Tukey 点到面 ICP。门控检查相邻帧互近邻与法向、位姿对物理先验的改动、留出的码点和深度、整圈角覆盖以及近底高度。精修通过门槛时才采用。输出 `registration.json` 含选型、每站刚体矩阵、曲面与标记指标。所有测点的 `view_id` 在 `registered.ply` 中保留。

示例为已经解码、去板和上色的采集点。完整装置实验的格雷码三角化、双机标定与上色分别来自 E201 的 `prepare.py`、`geometry_scan.py` 和 `export_color.py`。本目录的配准核心来自 E104，配准指标沿用 E101/E003 的实现。
