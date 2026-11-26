# E104 人偶网格优化

坐姿人偶使用 E103/ChArUco 的物体点和 E102 实测露板。世界坐标由 E102 V00 拟合的盘面定义，底盖位于 z=0。E003 小黄人使用其历史盘面坐标。全部文件以网格 PLY 保存。

- **接底局部修复.ply**。从封闭 S04 取 0–5.25 mm，从 S01 表面取 6.45 mm 以上，取向一致的边界环按弧长缝合。只对上部自交面一环做局部松弛，通过共边、无自交及有向体积核验。
- **细网格双边.ply**。0.75 mm 射线约束占据格，配合 12 次法向双边滤波和局部拓扑回退。0–1.5 mm 处固定顶点，4 mm 以上作用完整。用于后续展示和体积试验候选。
- **双边-8.ply**。原 1.25 mm S04 网格上的保底双边滤波，保留原 S04 底盖坐标。Taubin、双边-20 和细网格用于强度和网格间距对照。
- **坐姿人偶.csv**。每个变体的留出距离、头部与鞋部法向结果、射线冲突、闭合、相交对及体积。裸接底的三个试验网格仍有自交，适合查验环匹配和拓扑问题。CSV 同时保留双边局部回退前的诊断行。
- **边界.csv / 复核.json**。源网格的边界分组及 E102 盘面与支撑证据核验。露板侵入占比用各角站三取一样点，剔除底盖边界 1.5 mm 内的点。
- **E003-S01.csv**。历史小黄人 M05 的轻度和较强双边平滑，统计原有输入点到网格距离，用于跨物体检查。

头部法向统计以同一网格上探针偏移 0.8 mm 后的两个点为起点，将点投影到各候选面并插值得到两处法向，记录转角 P50/P95。坐姿人偶误差使用 10,500 个未参与 T0 重建的实际深度点。模型体积由重新读取的 PLY 计算，贴盘候选通过有向面和闭合双重检查。

在项目根目录执行 `OMP_NUM_THREADS=1 algorithm/.venv/bin/python simulation/E104/refine_mesh.py --fine` 可从 E102/E103 重新计算。省略 `--fine` 可重用已导出的 0.75 mm 对照网格。双边与 Taubin 的论文链接见最终汇报。

## 平滑参数

双边更新的局部高度为 `h_ij = n_i · (v_j − v_i)`，权重为 `exp(−d_ij²/(2σ_d²) − h_ij²/(2σ_h²))`。顶点沿当前法向移动带权高度均值的 0.8 倍，再乘盘面渐变权重。细网格候选的 σ_d=0.8024 mm、σ_h=0.3672 mm、12 次迭代、单顶点累计位移上限 0.60 mm。Taubin 对照每轮做 0.50 和 −0.53 两步逆距离加权移动，共 12 轮。旧小黄人的轻度方案做 3 次，累计位移上限 0.25 mm，拟合 P95 增幅以 0.10 mm 为本轮工程筛选值。

- [Taubin，A Signal Processing Approach to Fair Surface Design](https://graphics.stanford.edu/courses/cs468-01-fall/Papers/taubin-fair-surface.pdf)
- [Fleishman 等，Bilateral Mesh Denoising](https://cris.tau.ac.il/en/publications/bilateral-mesh-denoising-3/)
- [Liepa，Filling Holes in Meshes](https://diglib.eg.org/items/bd5c3658-4de6-4336-a7bd-0baf4f5618ac)
