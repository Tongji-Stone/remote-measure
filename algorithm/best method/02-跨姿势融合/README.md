# 02 · 同一物体不同摆放的配准与接缝融合

输入同一刚体的两个或三个摆放，先求跨摆放位姿，再按 **F03** 取舍实测点。有逐站深度时，再用 **B05** 修复有观测支持的接缝。输出在主摆放 V00 相机坐标，单位 mm。

## 运行示例

从 `best method` 目录执行，使用[安装好的依赖](../README.md)。

```sh
python "02-跨姿势融合/fuse_placements.py" "02-跨姿势融合/cases/seated-doll/scan.json" "runs/doll"
python "02-跨姿势融合/fuse_placements.py" "02-跨姿势融合/cases/pumpkin/scan.json" "runs/pumpkin" --poses "02-跨姿势融合/cases/pumpkin/poses_reference.json"
```

| 案例 | 输入与对照 |
|---|---|
| 坐着人偶 | [清单与相机标定](cases/seated-doll/scan.json) · [正放 PLY](cases/seated-doll/upright/ring.ply) · [背部 PLY](cases/seated-doll/back/ring.ply) · [面部 PLY](cases/seated-doll/face/ring.ply) · [实验位姿](cases/seated-doll/poses_reference.json) · [E204 参考点云](cases/seated-doll/reference.ply) |
| 小南瓜 | [清单与相机标定](cases/pumpkin/scan.json) · [正放 PLY](cases/pumpkin/upright/ring.ply) · [翻面 PLY](cases/pumpkin/inverted/ring.ply) · [实验位姿](cases/pumpkin/poses_reference.json) · [E204 参考点云](cases/pumpkin/reference.ply) |

每个摆放都含 10 站实测 PLY，清单逐一列出相对路径和矩阵。两个案例保留了点云颜色。南瓜棱纹近似周期对称，正文示例用 E203 已结合果梗证据选定的位姿。省略 `--poses` 可检查几何候选和可能的对称解。人偶示例直接从三组点云估计位姿。

## 代码

| 源码 | 负责的步骤 |
|---|---|
| [fuse_placements.py](fuse_placements.py) | 读取清单、组织两摆位或三摆位、投影深度、输出点云和核验记录 |
| [registration.py](registration.py) | 主轴和翻面候选、FPFH-FGR/RANSAC、Tukey ICP/GICP/Colored ICP、三边图与留出评分 |
| [rgb_matches.py](rgb_matches.py) | 可选彩照 SIFT、射线与局部实测面的交点、刚体种子 |
| [measured_fusion.py](measured_fusion.py) | F03 的跨摆放射线投票、法向与密度约束、去重和颜色处理 |
| [seam_repair.py](seam_repair.py) | B05 的加权 TSDF、局部 MLS、窄带插值、空域检查与锚点微调 |
| [scan_io.py](scan_io.py) | PLY、标定与位姿矩阵检查，投影和变换 |

## 自有数据清单

- `camera` 包含 `cam_int`、`cam_dist`、`shape=[高, 宽]`。每个 `placements` 项含名称 `name`、整圈 `ring` 和 V00 到该摆放盘面坐标的 4×4 矩阵 `camera_v00_to_placement`。第一项为主摆放。已知相对主摆放翻面时，设置 `flipped: true`。所有摆放需使用一致的坐标系手性。
- 逐站 `views` 含相机原坐标下的 `cloud` 和 `station_to_v00`。保留逐站 PLY 即可启用深度空域约束和 B05。只有整圈 PLY 时可以试跑几何配准及 F03 的密度与法向筛选。
- 若有彩照，可选填 `views[*].texture`。同时提供 `rgb_camera` 的 `cam_int`、`cam_dist`、`shape`、`hik_to_color`。彩照 SIFT 可提供位姿初值。随附样例使用彩色点云和逐站深度，现场彩照保存在原实验目录。
- 若给定 `--poses`，位姿 JSON 的 `names` 与清单同序，`placement_to_primary` 给出各摆放盘面到主盘面的刚体矩阵，第一项为单位矩阵。两份样例位姿 JSON 均可作格式参考。

设 (B_i) 为 V00 到当地盘面、(Q_i) 为该盘面到主盘面、(M_{ij}) 为单站到本次 V00。单站相机点经 (B_0^{-1}Q_iB_iM_{ij}) 进入主摆放 V00。配准用拟合、选型、留出三个空间块集，另外记录姿态候选间的差异。

输出 `F03-measured.ply` 和 `B05-repaired.ply` 为纯点云。对应的 provenance NPZ 记录 F03 的采集序号与顶点索引、B05 插值标记与法向位移。`fusion.json` 记录矩阵、留出候选指标和表面覆盖。本目录代码来自 E203/E204。原实验另外使用果梗、棱纹和实拍重投影核验特定样本。随附的参考位姿和参考点云保留了该实验的结果。
