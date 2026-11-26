# 遮挡补全与体积预测

[体积误差图](results/figures/volume_errors.png) · [补全点云示例](results/figures/completion_examples.png) · [完整评分 CSV](results/benchmark.csv) · [仿真配置与真值](results/parameters.json)

## 怎么模拟

构造三类平底试件。A 是旋转放置的长方体，B 含台阶、槽、盲孔、薄翼通孔，C 是具有连续起伏上表面的自由曲面。表面由离散真值构建，体积用独立的 0.25 mm 高度积分计算。相机与投影仪分别投射光线。相机看见且投影仪能照到的交点才成为深度输入，另保留白场的二值轮廓。深度加入 0.18 mm 标准差噪声。三种采集配置分别为四个低角且加一片投影盲区、八个低角、八个低角加两个高角。第一种同时改变遮挡与视角数，独立的视角数实验见全场景拼接报告。

每个试件目录保存真值表面与网格，每个配置保存 observed.ply、各方法的预测点云和网格。预测点云中，距最近观测点超过 2.5 mm 的部分标成橙色。用相对体积误差、体素 IoU、整体表面 F1 及隐藏表面的 2 mm 召回率评分。每次都有闭合和可定向检查。

## 实测结果

下表采用 **八个低视角加两个高视角**，数字为相对真值的体积偏差。三类的大小及槽孔都在仿真参数中完整指定。

| 方法 | A 长方体 | B 台阶、槽与孔 | C 自由曲面 |
|---|---:|---:|---:|
| 凸包 + 平底 | +11.51% | +26.03% | +16.45% |
| 白场视觉轮廓体 | +22.45% | +28.88% | +17.72% |
| 视觉轮廓体 + 深度剔除 | +4.13% | +4.77% | +4.41% |
| 高度场 + 平底 | +7.54% | +9.58% | +6.44% |
| 高度场 + 平底 + 深度约束 | +3.38% | **+3.85%** | **+3.36%** |
| 长方体拟合 | **+2.84%** | — | — |

这批无底面观测的普通 Open3D Poisson 产物均未通过闭合检查，保留输出供观察。体素方案使用共享面的 marching tetrahedra 提取网格，48 个可算体积的重建均通过闭合检查。B 的遮挡低视角场景中，带深度约束的高度场误差为 **+9.28%**。加密和补高视角一起把它降到 **+3.85%**。高度场仍受盲孔观测影响。

[网格分辨率试验](results/resolution_sensitivity.json) 固定同一批光线输入，从 1.5 mm 改为 0.75 mm。高度场与深度约束的占据体积误差在 A 为 3.42%→3.04%，在 B 为 3.95%→4.18%，在 C 为 3.38%→3.17%。这组结果提示，补盲区比单纯加密网格更值得优先做。

## 看不见的区域能推到哪里

- **底部**由转台曝光点估盘面。在正式赛题的平底假设下用盘面封底。封底网格可与输入点云分开保存。
- **A 类**按可能的尺寸模型做鲁棒拟合。长方体用正交平面和边界，其他 A 类可换成圆柱、圆锥或球冠候选，利用留出视角选择。当前验证的是长方体。
- **B 类**用白场轮廓限制外部空间，再根据深度把已观测的孔槽空缺从体积中扣除。优先拍高视角，让光到孔底。 [Open3D 轮廓与深度体素裁切](https://www.open3d.org/docs/release/tutorial/geometry/voxelization.html)
- **C 类**用平底和上表面连续的约束，在盘面坐标内做 z=h(x,y)，让实测点限制高度，白场轮廓限制 XY 支撑域，自由空间限制上界。补全缺点后对高度积分，也能输出闭合网格。如还有侧向不确定，体积同时报告不同模型和遮挡假设下的敏感度。

[盲孔反例](results/hidden_hole_ambiguity.json) 实跑了同一个半径 8.5 mm 的孔，深度分别为 1.5 和 4.5 mm。遮住孔对应的视线后，八视角的轮廓和仍可观测的深度一项不差。两件实体体积相差约 **681 mm³**。这里应让高视角看到孔底，或按可支持的深度范围给出体积敏感度。表面上大面积补齐也可能漏掉体积敏感的小结构。

[真实纸盒预测演示](results/real_box_1116_prior) 从 11/16 长方体的实测四面边界估计外包长方体，导出闭合顶底与点云。档案尺度下约为 **4.80 × 10⁵ mm*³**。仅改变侧面高度分位数，得到约 **4.71–4.83 × 10⁵ mm*³**。这是外包模型的估计，宽度、比例标定和开口内部实体都还需要实物基准。全景实测 PLY 仍留在 algorithm，预测文件独立保存。

## 公开算法依据

| 来源 | 对本项目的用法 |
|---|---|
| [Laurentini，Visual Hull，TPAMI](https://doi.org/10.1109/34.273735) | 多视角轮廓交集限定外轮廓。凹槽与盲孔深度由深度数据确定。 |
| [Curless–Levoy，SIGGRAPH 1996](https://graphics.stanford.edu/papers/volrange/paper_1_level/gamma-corrected/paper.html) | 多视角深度可在体素里累计，TSDF 是可比较的表面融合分支。本轮采用显式轮廓与自由空间约束。 |
| [Screened Poisson，ACM TOG](https://doi.org/10.1145/2487228.2487237) · [Open3D 实现](https://www.open3d.org/docs/release/tutorial/geometry/surface_reconstruction.html) · [ImFusion 公开实现说明](https://docs.imfusion.com/suite/Mesh/Algorithms/Mesh.PoissonReconstruction.html) | 从带法向点得到连续曲面，看数据密度、边界与闭合。商业软件文档用于调研实现。 |
| [Kazhdan 等，SGP 2020](https://hhoppe.com/proj/envelope/) | 在 Poisson 中加入轮廓或深度包络，适合后续把平底、剪影与自由空间边界一起加入平滑求解。 |
| [VRCNet，CVPR 2021](https://openaccess.thecvf.com/content/CVPR2021/html/Pan_Variational_Relational_Point_Completion_Network_CVPR_2021_paper.html) · [PoinTr，ICCV 2021](https://openaccess.thecvf.com/content/ICCV2021/html/Yu_PoinTr_Diverse_Point_Cloud_Completion_With_Geometry-Aware_Transformers_ICCV_2021_paper.html) | 学习方法可以生成多种缺失形状的候选。应用前用机加工类别、尺寸、孔槽实测与轮廓约束验证。本轮进行了几何方法的实验。 |

队伍提供的赛题规则规定正式测量自动完成，体积按实体计算，孔槽空缺扣除。正式 C 类下表面为单一平面，上表面连续，约束适合高度场。商用算法公开文档用于方法参考，实测和本仿真均使用本地自己的代码与开源库。

## 复现

从仓库根目录运行 simulation/run.py、simulation/ambiguity.py、simulation/resolution_check.py。前述三项用 algorithm/.venv/bin/python，建议 OMP_NUM_THREADS=2 和 OPENBLAS_NUM_THREADS=1。11/16 实例执行 simulation/real_box_example.py，引用本轮全场拼接。作图执行 simulation/plot.py，使用科研 Matplotlib 环境。
