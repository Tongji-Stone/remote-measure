# 四个物体的点云与报告

每个物体的 PDF 位于物体目录。充电头沿用以 17:41 充电头为主图的标定联合 Benchmark。

- `单视角/` 为实验裁剪后的相机坐标系点云，文件名保留采集帧号与名义角。
- `拼接/<算法>/all_views.ply` 为全视角拼接，`pair_00_01.ply` 为双视角拼接。
- 标注为 `360deg_复测` 的单视角用于独立复核。小黄人沿用校正后的帧号与角度配对。
- 17:16 充电头保存两种旋向的历史拼接对照，最稀疏视角含 17 点。
- 坐标单位采用各采集组的标定尺度。每组的逐帧点数在 `帧索引.csv`。

| 物体 | 采集组 | 单视角 PLY | 拼接 PLY |
|---|---|---:|---:|
| 充电头 | 2026-11-23_171640_棋盘格 | 8 | 2 |
| 充电头 | 2026-11-23_174129_ArUco | 8 | 22 |
| 小黄人 | 2026-11-20_095133 | 9 | 22 |
| 纸盒 | 2026-11-16_174014_长方体 | 5 | 20 |
| 纸盒 | 2026-11-23_174531_居中 | 8 | 22 |
| 纸盒 | 2026-11-23_174747_偏心 | 8 | 22 |
| 纸盒 | 2026-11-23_175006 | 8 | 22 |
| 苹果 | 2026-11-20_095754 | 5 | 22 |

## 标定与采集来源

### 充电头 / 2026-11-23_171640_棋盘格

- 原始采集 `data/object_scans/scan_20261123_171640_110748_充电头_棋盘格贴转台`
- 光学参数 `data/procam_calibration/20261123_1711_analysis/calibration_20261123_10mm_experimental.xml`
- 转台参数 `data/object_scans/scan_20261123_171640_110748_充电头_棋盘格贴转台/axis_1711_sift_experimental.json`

### 充电头 / 2026-11-23_174129_ArUco

- 原始采集 `data/object_scans/scan_20261123_174129_740645_充电头_二维码_中心`
- 光学参数 `data/procam_calibration/20261123_1711_analysis/calibration_20261123_10mm_experimental.xml + output/calibrated-benchmark/recovery/projector_recovery.json`
- 转台参数 `该会话 ArUco 标定`

### 小黄人 / 2026-11-20_095133

- 原始采集 `data/object_scans/scan_20261120_095133_815739_小黄人`
- 光学参数 `data/procam_calibration/20261120_analysis/calibration_10mm_experimental.xml`
- 转台参数 `data/procam_calibration/20261120_analysis/axis_approx.json`

### 纸盒 / 2026-11-16_174014_长方体

- 原始采集 `data/object_scans/scan_20261116_174014_915028用格雷码弯曲转台标定测试的长方体`
- 光学参数 `data/procam_calibration/calibration_20261116_10mm_experimental.xml`
- 转台参数 `data/object_scans/scan_20261116_174014_915028用格雷码弯曲转台标定测试的长方体/axis_fit_box_metadata.json`

### 纸盒 / 2026-11-23_174531_居中

- 原始采集 `data/object_scans/scan_20261123_174531_873516包装盒_转盘中心`
- 光学参数 `data/procam_calibration/20261123_1711_analysis/calibration_20261123_10mm_experimental.xml + output/calibrated-benchmark/recovery/projector_recovery.json`
- 转台参数 `该会话 ArUco 标定`

### 纸盒 / 2026-11-23_174747_偏心

- 原始采集 `data/object_scans/scan_20261123_174747_132362包装盒转盘边缘`
- 光学参数 `data/procam_calibration/20261123_1711_analysis/calibration_20261123_10mm_experimental.xml + output/calibrated-benchmark/recovery/projector_recovery.json`
- 转台参数 `该会话 ArUco 标定`

### 纸盒 / 2026-11-23_175006

- 原始采集 `data/object_scans/scan_20261123_175006_029822`
- 光学参数 `data/procam_calibration/20261123_1711_analysis/calibration_20261123_10mm_experimental.xml + output/calibrated-benchmark/recovery/projector_recovery.json`
- 转台参数 `该会话 ArUco 标定`

### 苹果 / 2026-11-20_095754

- 原始采集 `data/object_scans/scan_20261120_095754_604145_苹果`
- 光学参数 `data/procam_calibration/20261120_analysis/calibration_10mm_experimental.xml`
- 转台参数 `data/procam_calibration/20261120_analysis/axis_approx.json`
