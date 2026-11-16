# 结构光试验结果

本目录下的文件来自 2026-11-13 的相机—投影仪试验，用于查看和排查；标定和拼接均未验证，不能用于尺寸或体积测量。

| 文件 | 内容 |
| --- | --- |
| `calibration-unverified.xml` | 六组标定板图片算出的初步标定参数；原程序尺寸顺序需修正，联合误差约 5.5 像素。 |
| `box_views.png`、`cup_views.png`、`solder_views.png` | 长方体、纸杯、焊锡卷的单视角点云预览。预览图为了显示而裁去少量极端坐标。 |
| `box_alignment_experimental.ply` 和 `.png` | 长方体多视角初次配准试验，存在明显错配。 |
| `box_fixed_axis_experimental.ply` 和 `.png` | 限制同轴同向、相邻角度不超过 70° 的拼接试验，假设轴平行相机 Y 轴；结果未验证且仍有错位。 |

PLY 文件可以使用 CloudCompare、MeshLab 等工具旋转查看；颜色表示不同采集视角。完整原始采集照片、单视角点云与本地程序使用的参数文件见项目根目录 README 中的同济云盘链接。

操作台目前读取的是 `data/procam_calibration/calibration_result_初步未验证.xml`，它不会自动使用本目录的副本。若要恢复实验，按根目录 README 下载并解压数据包。

2026-11-16 的标定、转台与长方体拼接试验见 [2026-11-16 结果说明](2026-11-16/README.md)。
