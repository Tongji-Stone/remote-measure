# 05 · 后半段共用调研：点云 → 体积的算法、标定计量与成本

> 覆盖：赛题二所有技术路线共用的"后半段"（点云处理 → 曲面重建 → 体积积分 → 标定与误差 → 硬件成本）。
> 名词第一次出现都跟一句大白话。价格标来源类型；查不到写"未找到可靠来源"，不确定处标"⚠️不确定"。

## 0. 三条硬约束（决定算法取舍）

1. **体积 = 实体部分体积，通孔/凹槽/缺口不计入** → 凸包、包围盒直接出局；重建必须"该空的地方保持空"。
2. **单件 ≤60 s 全自动**；允许赛前标定，不得接触/夹持/贴标记 → 必须能自动分割、自动去测试台、自动定参、自动重试。

---

## 1. 算法链清单与对比表

### 1.1 点云预处理

| 环节 | 干什么（大白话） | 开源实现（实际函数） | 复杂度 | 对凹槽/通孔 | 适合本题？ |
|---|---|---|---|---|---|
| 无效点清理 | 删掉深度算不出、坐标是 NaN 的点 | `PointCloud.remove_non_finite_points()` | 极低 | 无影响 | ✅ 第一步 |
| 体素降采样 | 空间切小方格，每格只留一个代表点（点云变稀） | `PointCloud.voxel_down_sample(voxel_size)`；PCL `pcl::VoxelGrid` | 低 | ⚠️ 方格大时窄缝细孔被"抹平" | ✅ 但体素 ≤ 孔宽 1/3 |
| 统计滤波 | 看每点与邻居的平均距离，太"孤僻"的点删掉 | `PointCloud.remove_statistical_outlier(nb_neighbors, std_ratio)`；PCL `pcl::StatisticalOutlierRemoval` | 低 | 中性；参数过激会啃掉孔口真实数据 | ✅ 文献同款 K=20、σ=3.0 |
| 半径滤波 | 规定半径内邻居不足 N 个就删 | `PointCloud.remove_radius_outlier(nb_points, radius)`；PCL `pcl::RadiusOutlierRemoval` | 低 | 对稀疏的孔内壁杀伤大 | ⚠️ 孔洞区慎用 |
| 直通/裁剪 | 只留坐标范围内的点（如台面上方 25 cm 盒子内） | PCL `pcl::PassThrough` / `pcl::CropBox` | 极低 | 无影响 | ✅ 便宜的去背景 |
| 法向估计 | 算每个点"朝哪边"（局部曲面法线） | `PointCloud.estimate_normals(KDTreeSearchParamHybrid(radius, max_nn))` | 低 | 半径太大会被周围平面带偏 | ✅ 泊松/BPA 必须 |
| 法向定向 | 把法向统一朝外 | `orient_normals_consistent_tangent_plane(k)` / `orient_normals_towards_camera_location()` | 低 | ⚠️ 有通孔时"朝外"是双义的，易定向错 | ✅ 配 TSDF 时用"朝相机" |

### 1.2 分割、配准与融合

| 环节 | 干什么 | 开源实现 | 复杂度 | 对凹槽/通孔 | 适合？ |
|---|---|---|---|---|---|
| 测试台去除 | 用 RANSAC 找"最大的一块平面"剔除（RANSAC＝随机抽几点评模型，抽很多次，被支持最多者胜） | `PointCloud.segment_plane(distance_threshold, ransac_n, num_iterations)` | 低 | 物体与台面粘连处会切出洞，需补底 | ✅ 必做 |
| 物体分割 | 把散点按"挨得近不近"分堆，取最大的一堆 | `PointCloud.cluster_dbscan(eps, min_points)` | 低 | 好：凹槽不会把物体切成两半（除非槽宽>eps） | ✅ 必做 |
| 已知角度粗配准 | 转台转多少度就先把点云转多少度 | 直接构造旋转矩阵 | 极低 | — | ✅ **本题最优初值** |
| ICP 精配准 | 两张点云像拼图，"找最近点→微调位姿"反复贴合（ICP＝迭代最近点） | `registration_icp(src,dst,max_corr_dist,init,TransformationEstimationPointToPoint()/PointToPlane(),ICPConvergenceCriteria(max_iteration=...))` | 中 | 对应距离太大会把孔洞两侧错配 | ✅ 点对面更快更准 |
| 彩色 ICP | 除形状外用颜色帮忙对齐 | `registration_colored_icp()` | 中 | — | ❌ 物体无纹理 |
| 盲配准 | 不知初始角度时用局部特征粗对齐 | `registration_ransac_based_on_feature_matching()` + `compute_fpfh_feature()` | 高 | — | ⚠️ 仅兜底 |
| 位姿图优化 | 多片点云的相对关系当一张网统一平差，消累积误差 | `PoseGraph`/`PoseGraphNode`/`PoseGraphEdge`/`global_optimization()` | 中高 | 中性 | ✅ 视角≥8 收益明显 |
| 多片融合 | 把配准好的多片合并 | `PointCloud + PointCloud` | 极低 | 重叠区点密度不均 | ✅ 更推荐 TSDF 加权融合 |

### 1.3 曲面重建

| 方法 | 一句话原理 | 开源实现 | 复杂度 | 对凹槽/通孔 | 适合？ |
|---|---|---|---|---|---|
| **泊松重建** | 给散点糊一层光滑的皮，让它们变成封闭表面（解一个梯度场拟合的泊松方程） | `TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)`，返回 `(mesh, densities)` | 中（depth 越大越细越慢） | ❌ **危险**。官方原文：泊松"会在低点密度区造三角形，甚至会向某些区域**外插**"→ 孔内壁没采到点就封盖 | ⚠️ 只出外壳 |
| **TSDF 体素融合** | 空间切小格，每格记"表面在格子前还是后、差多远"，多视角加权平均（TSDF＝截断符号距离场） | `ScalableTSDFVolume(voxel_length, sdf_trunc, color_type)` + `integrate(rgbd,intrinsic,extrinsic)` + `extract_triangle_mesh()` | 中 | ✅ **最好**。体素融合有两个核心概念：**TSDF 加权平均**（抗高斯噪声）与 **carving**（Open3D 文档原文：carving 善于去掉悬浮噪点和结构边缘的鼓包）。同一机制也意味着"射线穿过却没打到东西"的区域不会被判成实体 → 只要孔内壁被任一视角看到，通孔/凹槽就不容易被填 | ✅✅ **首选重建通道** |
| 滚球法 BPA | 小球在点云上滚，同时碰到三个点就贴一个三角片 | `TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, radii)` | 中 | ⚠️ 不主动封盖也不补洞，输出常不水密 | ⚠️ 需配水密检查 |
| **Alpha Shape** | 凸包的"可调松紧版"：半径 α 的球钻不进去的地方不算实体 | `TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)`；CGAL `Alpha_shape_3`/`Fixed_alpha_shape_3` | 中 | ✅ 好。**孔径 ≫ α 时通孔保留**，凹槽也保留。α 太小→碎裂偏小；太大→退化凸包偏大 | ✅✅ **最佳交叉验证** |
| 网格体检 | 查网格是否水密（无开口）、可定向、是流形（每条边最多两面共用） | `is_watertight()`/`is_orientable()`/`is_edge_manifold()`；trimesh `is_watertight`/`is_volume`/`is_winding_consistent` | 极低 | — | ✅ 出数前必做 |
| 网格补洞 | 把网格开口用三角片封上，好让体积公式可用 | `trimesh.repair.fill_holes(mesh)`；`open3d.t.geometry.TriangleMesh.fill_holes(hole_size)` | 低 | ❌ **本题里是错误操作**：会把真通孔真凹槽一起封上，体积偏大 | ❌ 除非确认是采集缺口 |

**自动选 α**：CGAL 官方给了可编程判据——"从数据点穿出实体的**最小 α**"和"使 alpha shape 只有**一个连通分量的最优 α**"。用后者起步，再扫邻域取体积的稳定平台值，即可免人工调参。

---

## 2. 体积计算方法对比（核心）

### 2.1 一把尺子：表面整体"胖"了 δ 毫米，体积差多少

**ΔV ≈ A·δ**（A 为表面积）。近似等轴、尺寸 L 的物体 A/V ≈ 6/L，故 **ΔV/V ≈ 6δ/L**：

| L | δ | 体积相对误差 |
|---|---|---|
| 50 mm | 0.10 mm | **1.2 %** |
| 50 mm | 0.05 mm | 0.6 % |
| 50 mm | 0.20 mm | 2.4 % |
| 200 mm | 0.10 mm | 0.3 % |

**推论（本报告推导）**：5 cm 小件做到 1%，表面偏差须压到 **0.08 mm 以内**；做到 2% 要压到 0.17 mm。**这是本赛题最硬的设计约束，比"点云多密"重要得多。**

### 2.2 第二个结论：随机噪声几乎不影响体积

零均值随机深度噪声、空间相关长度 ℓ 时，**σ_ΔV ≈ σ_z·ℓ·√A**（相邻区域偏差互相抵消）。取 σ_z=0.05 mm、ℓ=1 mm、A=60000 mm²（10 cm 立方体）：σ_ΔV ≈ 12 mm³，约 **0.001%**，可忽略。
→ **力气花在消除系统性偏差（标定、重建偏置），不是"多拍几张降噪"。**（本报告推导）

### 2.3 方法对比表

| 方法 | 一句话原理 | 误差量级 | 凹槽 | 通孔 | 需水密？ | 现成实现 | 推荐场景 |
|---|---|---|---|---|---|---|---|
| **凸包体积** | 拿紧绷塑料膜把所有点包起来 | 只偏大；凹槽全填平，可偏大 5–50% ⚠️不确定 | ❌ | ❌ | 否 | `PointCloud.compute_convex_hull()`／`TriangleMesh.compute_convex_hull()`；trimesh `convex_hull`；`scipy.spatial.ConvexHull` | 只当体积上界做异常检测 |
| **有向包围盒 OBB** | 用最小盒子套住 | 极大偏大，仅规则件有意义 | ❌ | ❌ | 否 | `get_minimal_oriented_bounding_box()` | 只做量级检查 |
| **体素占用计数** | 空间切小方块，数"被占方块"×方块体积 | **有实测**：苹果体积 MAPE **1.09%**、RMSE 6.95 cm³（0.5 mm 体素）；原文指出重建体积系统性**偏低** | ⭕ 凹槽被采到就是空的 | ⭕ 取决于占用判定，易糊掉 | 不严格，但判定规则决定成败 | 无单一函数；`Trimesh.voxelized(pitch)`→`VoxelGrid` | 现场鲁棒，但要知道它系统性偏低 |
| **等高线切片积分（层析）** | 像切面包切薄片，每片量面积再乘厚度累加 | 片内用凸包→偏大；用带孔多边形/2D alpha shape→接近真实。层厚 0.2 mm 时梯形法误差通常 **<0.3%** | ✅ 凹槽＝多边形缺口，天然保留 | ✅ 孔轴穿过时切片是"多边形带孔"，面积可精确算 | 否（只需每层轮廓闭合） | 自写（numpy + `shapely` 或 `trimesh.intersections.mesh_plane`）；片内凸包用 `scipy.spatial.ConvexHull` | ✅ **性价比最高**；难点是保证每层轮廓闭合 |
| **网格有符号体积（散度定理）** | 表面拆成小三角，每个三角对原点张一个锥体，所有锥体带正负号相加 | 网格正确时**算法本身零误差**，误差全来自网格几何 | 取决于网格 | 取决于网格 | ✅ **必须**（水密＋法向一致＋体积为正） | `TriangleMesh.get_volume()`（官方要求 watertight 且 orientable）；trimesh `Trimesh.volume`（先查 `is_volume`） | ✅ **最终出数主通道** |
| **泊松重建后积分** | 先糊皮成封闭面，再套散度定理 | 随 depth 与点密度变化；边角磨圆、低密度区鼓包 | ❌ 易糊平 | ❌ 两端易封盖→偏大 | 是（泊松天然封闭） | `create_from_point_cloud_poisson()` → `get_volume()`；用 `densities` 删 0.01 分位以下低支持顶点 | 出外壳/完整性检查 |
| **TSDF 重建后积分** | 多视角把"表面在哪"投票进体素，再提网格算体积 | 体素 0.5 mm 时量化偏差可达 ±0.25 mm → L=50 mm 时约 **3%**！须用 0.2–0.3 mm 体素＋标准件校准 | ✅ 自由空间雕刻 | ✅ **最可靠的通孔保真方案** | 提取的网格未必水密，需检查＋补底 | `ScalableTSDFVolume` + `integrate()` + `extract_triangle_mesh()` → `get_volume()` | ✅✅ **首选重建通道** |
| **Alpha Shape 实体体积** | 半径 α 的球钻不进去的地方不算实体；实体是四面体集合，体积直接可算 | α 太小→碎裂偏小；太大→退化凸包偏大 | ✅ | ✅ 孔径≫α 时保留 | alpha shape 本身封闭可算体积 | `create_from_point_cloud_alpha_shape(pcd, alpha)`；CGAL `Alpha_shape_3` | ✅✅ **最佳交叉验证** |

### 2.4 「封盖」会不会发生、往哪偏（最容易踩的坑）

关键不在算法名字，而在**孔内壁有没有被看到**：

- **被看到 → 都保得住**：孔内壁只要被任一视角采到点，TSDF（自由空间雕刻）和 Alpha Shape（球钻不进去）都能保留孔洞。
- **没被看到 → 一定猜错，而且往大偏**：泊松默认"低密度区外插补全"会直接封盖；体素占用法也会把孔判成实体。
- **量级**：直径 10 mm、穿过 30 mm 厚壁的通孔 = π×5²×30 ≈ 2360 mm³ = **2.36 cm³**。
  - 标准体积 500 cm³：一个孔封盖 **+0.47%**；标准体积 125 cm³（5 cm 小件）：**+1.9%**；3 个这样的孔：**+1.4%～+5.7%**。
- **反向风险是偏小**：配准把表面"收缩"、或漏采凸起，体积会偏小。已发表的单目结构光苹果工作就发现重建体积系统性偏低。
- → `trimesh.repair.fill_holes()` / `open3d.t...fill_holes()` 这类"补洞让网格水密"的操作**在本题是错误操作**；只有确认开口是"采集缺口"（如底面与台面粘连处）才能用，且须有物理依据（§5）。

### 2.5 推荐链（可直接写进方案）

1. ≥12 视角采集 → 已知转角粗配准 → ICP 点对面精配准 → 位姿图优化。
2. RANSAC 去台面 → DBSCAN 取最大簇 → **把底面边界投影到台面补出封底**（物理依据：物体底面本就平放台上，这不是"封盖作弊"）。
3. **同时**跑 TSDF（体素 0.2–0.3 mm）与 Alpha Shape（自动选 α），各算一次 `get_volume()`。
4. **一致性判据**：两者相对偏差 <1.5% 才输出均值；偏差大就自动换参重跑（程序行为，非"人工修正"）。
5. 套用赛前标定的"体积比例＋加性残差"模型修正后输出。

---

## 3. 标定与误差分析建议

### 3.1 三类标定

| 标定项 | 推荐做法 | 判据 / 可对比实测 |
|---|---|---|
| **相机内参/畸变** | **张正友平面棋盘格法**：已知格距棋盘在多个姿态拍十几张解算；OpenCV `findChessboardCorners`→`cornerSubPix`→`calibrateCamera` | **平均重投影误差 0.1–0.3 px 算良好**。（重投影误差＝把三维角点用标定参数投回图像、与实际角点的像素距离；它把内外参与畸变误差全"卷"在一起） |
| **系统几何（相机–投影仪/双目）** | 结构光可**绕开投影仪内参标定**：改用"**相位—深度**"经验模型，相移＋格雷码解绝对相位后直接拟合相位→深度 | 实测：某单目条纹投影对半径 40 mm 标准球重建，绝对误差 **0.293 mm**、相对误差 **0.37%**，球面拟合 RMSE **≤0.095 mm**，平面误差 0.387 mm |
| **运动轴（转台）** | 用多视角点云自标定；或台面放棋盘格标定转角 | 《光学学报》2024 判据：**角度分辨率×被测半径 < 传感器空间分辨率**；角度定位误差×半径、端面跳动×半径都须小于传感器测量误差。该文实测转台角度误差 **<0.003°**、离轴 5 mm 处端面跳动 **<2 μm** |

**易错点（本报告推理）**：物体偏离转台轴心（偏心）**本身不产生误差**——刚体绕固定轴旋转，只要用真实的轴位置，多视角就自洽。真正的误差是"**你用的轴位置与真实轴不符**"。故赛前必须标出轴，且**不能指望裁判把物体摆正**（规则也不允许你动它）。

### 3.2 深度噪声的量级与缩放律

引用 *Light: Science & Applications* 的条纹投影（FPP）精度建模（PMC10613632）：
- 实测系统在 **900 mm** 工作距离下 σ_z ≈ **70 μm**；按模型 **σ_z ∝ z²** 外推，缩到 **240 mm** 时 σ_z ≈ **5 μm**（代价是视场变小）。
- → **把工作距离压到 250–350 mm 是本题最便宜的精度提升手段。**
- 两个易忽略的工程细节：(a) 投影 LED 要**预热到热平衡**，否则相位漂移，同时损害准确度与精密度；(b) 条纹周期越小精度越高但受 MTF 限制，存在最优周期（该文系统约 5 px，离焦时约 20 px）。
- 反直觉结论：2×2 像素合并成"超像素"取平均，噪声方差只降到 1/3.5 而非理论的 1/4——相邻像素噪声不独立，**"多平均"收益有上限**。

把 σ_z≈20–70 μm 代入 §2.1：L=50 mm、δ=0.05 mm → **0.6%**。这解释了 1% 可达，但前提是**短工作距离＋消除系统偏差**。

### 3.3 用标准件做「体积比例＋加性残差」校准（强烈建议）

思路：先用几个**已知体积**的东西量一遍，看系统偏多少，再反过来修正。赛题明确允许（"允许在测量前完成系统标定"）。

- **器件**：陶瓷标准球、0 级陶瓷量块、标称半径 5 mm 陶瓷球等。
- **可参考水平**：某线结构光系统对 5 mm 0 级陶瓷量块 10 次重复测量，**平均误差 2.0 μm**；标称半径 5 mm 陶瓷标准球的**半径测量误差 7.3 μm**（《光学学报》2024, 44(21): 2112001）。
- **模型**：对 N 个标准件测出 `V_meas,i`/`V_true,i`，最小二乘拟合 **`V_meas = a·V_true + b`**，出数用 **`V_out = (V_meas − b)/a`**。`a` 修与体积成比例的误差（深度尺度错）；`b` 修固定偏置（如表面整体外扩 δ 带来的 A·δ）。
- 因 A 与 V 非线性，**单一 b 只在同尺寸段有效**：建议按小/中/大各 1 件分档拟合，或直接用面积修正 `V_out = V_meas − δ̂·A_meas`（`A_meas` 由 `TriangleMesh.get_surface_area()` 得到）。
- **合规边界**：标准件属**标定器具**，不是被禁的商用三维测量设备；但须在成本清单与测量说明中如实申报，且**不能拿已知答案手动改输出**（规则禁止"人工修正体积结果"）。整条校准流程应脚本化，现场一键运行。
- **重复性**：同一标准件重复装夹测量 n≥10 次，报"均值±标准差"与极差；换一天/换操作者再做一轮得再现性。**判据：重复性标准差 < 目标误差的 1/3。**

### 3.4 不确定度分解表模板（可直接抄进报告）

| # | 误差源 | 符号 | A/B 类 | 估计标准不确定度 | 灵敏度（对体积） | 对体积的相对贡献 | 评定方法 |
|---|---|---|---|---|---|---|---|
| 1 | 相机内参/畸变标定残差 | u₁ | B | 重投影 0.1–0.3 px | 间接（转成深度后并入 u₂） | 0.1–0.3% | 标定残差统计＋留出集验证 |
| 2 | 深度/相位**系统性偏差** | u₂ | B | 0.02–0.15 mm | ≈ 表面积 A | **0.3–1.5%（主瓶颈）** | 平面/标准球多点拟合比对真值 |
| 3 | 深度**随机噪声** | u₃ | A | 0.02–0.10 mm | ≈ ℓ·√A（自抵消 ≈0） | **<0.05%** | 同一平面重复扫描，取面拟合 RMS |
| 4 | 转台角度/轴位误差 | u₄ | B | 角度 0.01–0.1°；轴位 0.05–0.2 mm | r·Δθ | 0.1–0.5% | 多视角重配准残差＋转角标定 |
| 5 | 多视配准残差 | u₅ | A | 0.05–0.2 mm | 等效表面偏移 | 0.1–0.6% | ICP 对应点距离 RMSE |
| 6 | 曲面重建（封盖/磨圆/收缩） | u₆ | B | 通孔体积 0.5–3% | 直接 | **0.5–3%（第二大瓶颈）** | 已知孔件比对；TSDF vs Alpha Shape 一致性 |
| 7 | 体积积分/离散化 | u₇ | B | 体素 0.2–0.5 mm | ≈ A·voxel/2 | 0.2–1.0% | 加密体素/网格看收敛性 |
| 8 | 装夹与姿态重复性 | u₈ | A | — | — | 0.1–0.5% | n≥10 次重复测量 |
| | **合成（方和根）u_c** | | | | | **约 0.8–3.5%** | |
| | 经标准件比例＋加性校准后 | | | | | **目标 0.5–1.5%** | |

> 表中数值是本报告基于 §2.1、§2.2、§3.2、§3.3 的量级推导与文献锚点给出的**估计区间**，非实测值；正式报告须替换为自己系统的实测值。

**瓶颈结论**：第 2 项（深度系统性偏差）与第 6 项（重建封盖/收缩）占绝大部分；第 3 项随机噪声基本可忽略。**钱花在"标定＋不封盖的重建"上，比花在"更高分辨率相机"上回报高得多。**

### 3.5 目标定多少才有竞争力

| 参照系 | 数值 | 来源 |
|---|---|---|
| 研究级：单目条纹投影＋转台＋ICP，苹果体积相对误差 | **1.09%（MAPE）** | 已发表论文（《农业工程学报》2025） |
| 研究级：双目结构光类球形水果，外形尺寸相对误差 | 3.32% 以内 | 已发表论文 |
| 商用扫描仪自报体积精度（仅作参照，不能交差） | ±0.02 mm + 0.04 mm×L(m) | 厂商博客 |
| 商用扫描仪自报单帧精度 | 最高 0.01–0.03 mm | 厂商博客 |

**建议：现场目标 ≤1.5%（争取 ≤1.0%），底线 ≤3%。** 理由：组委会标准体积本身可能带 0.1–0.3% 基准不确定度；研究级方案已到 1.09%。⚠️**不确定**：赛题官方**没有公开评分曲线或相对误差分档细则**，"1% 是分水岭"为本报告推测。

---

## 4. 硬件选型与成本清单表

赛题口径：计算机、显示器、通用电源适配器**不计入**尺寸与成本；**核心光电测量硬件必须计入**；成本清单明显失实会取消成本相关得分。
价格标注：**【快照】**＝本次检索到的淘宝/1688/京东/微雪页面文字快照，非实时报价；**【官网】**；**【估算】**。

| 类别 | 型号/规格 | 价格（¥） | 计入？ | 备注 |
|---|---|---|---|---|
| 相机模组（首选） | **OV9281** 1MP 黑白**全局快门** USB/UVC（全局快门＝整幅同时曝光，转台转动时不像卷帘快门把物体"扭歪"） | **55–150** | 计入 | 【快照】单目 UVC ¥55；420 fps 工业模组 ¥138；双目同步 ¥298 |
| 相机模组（备选） | 索尼 **IMX477** 1230 万像素模组（C/CS 口可换镜头） | 250–500 | 计入 | ⚠️**不确定**：微雪页面价格显示占位 ¥0.00，未取到可核实标价 |
| 工业相机（备选） | 海康机器人 **CS/CE 系列** GigE/USB3 | 约 **988 起** | 计入 | 【快照】淘宝专题页 ¥988；官网无公开报价，需询价 |
| 镜头 | 500 万像素 C 口 FA 镜头 5/8/12/16 mm | **135 起** | 计入 | 【快照】淘宝 ¥135 |
| 结构光投影（低成本） | 迷你 **DLP 投影仪**（消费级投影器，赛题明确允许"投影器"） | **300–1500** | 计入 | 【快照】淘宝迷你 DLP ¥50–¥2400 跨度极大；须实测图形刷新能力 |
| 结构光投影（工业） | **DLP3010** 微型光机 + DLP3478 控制板（0.3" 720p DMD） | 约 **6500** | 计入 | 【快照】淘宝商品页 ¥6500 |
| DMD 芯片（自研） | TI **DLP3310** ≈$73.49 / **DLP4710** ≈$148（仅芯片） | 约 530 / 1060 | 计入 | 【快照】知乎引 TI 价；不含驱动与光学，自研风险高，**不建议** |
| 线激光模组 | 650 nm 5 mW **一字线**（12 mm 壳） | **6.5–35** | 计入 | 【快照】淘宝 ¥6.53；1688 ¥35 |
| 窄带滤光片 | 850 nm / 808 nm 红外窄带 | **38–110**；实验室级 25.4 mm/半带宽 10 nm 约 **611** | 计入 | 【快照】淘宝 ¥38/¥110/¥611；多数厂商标"电议"，**具体型号报价不确定** |
| 偏振片 | PL 线偏振片 / 偏振膜 | **7–50** | 计入 | 【快照】淘宝 ¥7/¥18/¥39/¥50 |
| 步进电机＋驱动器 | 42 步进 + **TB6600** 套装 | **17 起**（DM542 驱动另约 80–150） | 计入 | 【快照】淘宝 ¥16.80 |
| 旋转台（成品） | 精密中空旋转平台（步进/伺服，60/85 型） | **305–650** | 计入 | 【快照】淘宝 ¥305/¥400/¥480/¥650 |
| 旋转台（自制） | 3D 打印转盘 + 大直径轴承 + 同步带 + 42 步进 | 60–200 | 计入 | 【估算】 |
| 主控/算力 | 树莓派 5（4/8 GB）/ **Jetson Orin Nano Super** 开发套件 | Pi 5 官方 $80 量级；Jetson 官方 **$249**，微雪 **¥3999** | ⚠️ **边界情况，需确认** | 【官网/快照】赛题把"处理单元"列为系统组成、又把"计算机"排除在外，**存在解释空间**；建议成本清单里单列并附说明交由裁判判定 |
| 3D 打印结构件 | 支架、机壳、转盘 | 50–300 | 计入 | 【估算】 |
| 标定器具 | 陶瓷标准球 25 mm / 0 级陶瓷量块 | 球 **59–550**；量块按等级另计 | **建议申报** | 【快照】淘宝 25 mm 陶瓷校正球 ¥59.4/¥388/¥498/¥550；红宝石校准球 ¥2000–8000 |
| **不计入** | 台式计算机、显示器、通用电源适配器 | — | **不计入** | 规则第 1 条明确排除 |

**整机预算量级（【估算】）**：线激光＋单目＋自制转台＋树莓派 **¥800–2000**；结构光（消费级 DLP）＋成品转台＋工控机 **¥3000–8000**；工业 DLP 光机＋工业相机＋精密转台＋Jetson **¥12000–20000**。
**低预算 ≠ 低精度**：§2、§3 表明卡住 1% 的瓶颈是标定与不封盖的重建，不是器件档次。省下的钱投到标准件和标定上回报更高。

---

## 5. 赛前标定与现场全自动（7 条可落地清单）

1. **自动去背景/去台面**：`segment_plane()` 拟合最大平面剔除 → `cluster_dbscan()` 取最大簇。底部开口则把边界投影到台面补封底。
2. **自动定尺度**：全部阈值由包围盒对角线 D 生成，如 `voxel=D/300`、`alpha=D/40`、滤波半径 `D/100`——5 cm 与 20 cm 物体无需改参数。
3. **自动初配准**：转台转角已知 → 旋转矩阵作初值 → ICP；RANSAC+FPFH 只作兜底（无纹理物体上不稳）。
4. **自动失败检测与重试**：出数前查 (a) `is_watertight()`/`is_orientable()`；(b) 覆盖率＝点数÷表面积；(c) `V_TSDF` 与 `V_alpha` 一致性；(d) `V/V_凸包` 是否落在合理区间。不通过就自动换参重跑（程序行为，不违反"禁止人工修正"）。
5. **自动补偿热漂移**：投影 LED 预热到热平衡再开始（§3.2）；建议整机开机预热 10–20 min，或现场先空跑一次"热身件"。
6. **时间预算（60 s 硬约束）**：12–18 视角采集（0.5–1 s/视角）约 10–20 s；配准＋重建＋体积 10–25 s；合计 **20–45 s**，留重试余量。内置"**快速模式**"（8 视角＋TSDF）作超时兜底。
7. **不确定度自评**：程序顺带输出"本次预估相对不确定度"，写在体积结果旁——函评（30 分）的加分项。

---

## 6. 参考链接（均为本次实际检索/抓取到的页面）

**赛题与规则**
- [第十五届全国大学生光电设计竞赛赛题（北京理工大学教务处 PDF，与本地 PDF 一致）](https://jwb.bit.edu.cn/docs//2026-09/c1fde5a1a91e44a09562c34324feec15.pdf)
- [第四届全国大学生光电设计竞赛赛题（山东大学 PDF，历届对照）](https://www.phy.sdu.edu.cn/__local/7/63/42/9ADE24081078E8876894DF9FF00_A2B950B5_24B60.pdf)

**开源库官方文档**
- [Open3D TriangleMesh API：`get_volume` / `is_watertight` / `is_orientable` / `compute_convex_hull` / 包围盒 / 表面积](https://www.open3d.org/docs/release/python_api/open3d.geometry.TriangleMesh.html)
- [Open3D PointCloud API：统计与半径滤波 / 体素降采样 / 法向 / 分割 / 聚类](https://www.open3d.org/docs/release/python_api/open3d.geometry.PointCloud.html)
- [Open3D 曲面重建教程：Alpha Shape / 滚球法 / 泊松（含"低密度区外插"原文与 densities 低支持剔除）](https://www.open3d.org/docs/release/tutorial/geometry/surface_reconstruction.html)
- [Open3D ScalableTSDFVolume API：构造参数 / `integrate` / `extract_triangle_mesh` / 自由空间雕刻](https://www.open3d.org/docs/release/python_api/open3d.pipelines.integration.ScalableTSDFVolume.html)
- [Open3D 多视角配准教程：`registration_icp` / `PoseGraph` / `global_optimization`](https://www.open3d.org/docs/release/tutorial/pipelines/multiway_registration.html)
- [Open3D ICP 配准教程：`ICPConvergenceCriteria` / 鲁棒核](https://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html)
- [PCL Filters 模块：`StatisticalOutlierRemoval` / `RadiusOutlierRemoval` / `VoxelGrid` / `PassThrough` / `CropBox` / `BilateralFilter` / `VoxelGridOcclusionEstimation`](https://pointclouds.org/documentation/group__filters.html)
- [CGAL 3D Alpha Shapes 手册：general/regularized 模式、自动挑 α 判据](https://doc.cgal.org/latest/Alpha_shapes_3/index.html)
- [trimesh.repair：`fill_holes` / `fix_normals` / `fix_winding` / `broken_faces` / `stitch`](https://trimesh.org/trimesh.repair.html)
- [trimesh Trimesh：`volume` / `is_volume` / `is_watertight` / `is_winding_consistent` / `convex_hull` / `voxelized`](https://trimesh.org/trimesh.html)
- [SciPy ConvexHull](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.ConvexHull.html)
- [Ball-Pivoting Algorithm 原始论文（Bernardini et al., TVCG 1999）](http://mesh.brown.edu/taubin/pdfs/bernardini-etal-tvcg99.pdf)

**标定与误差分析文献**
- [Sources of Errors in Structured Light 3D Scanners（NIST）：VDI/VDE 2634 质量参数、重投影误差、三角测量角度与深度不确定度、环境光/温度/凹面难扫](https://tsapps.nist.gov/publication/get_pdf.cfm?pub_id=927473)
- [Modeling the measurement precision of Fringe Projection Profilometry（Light: Sci & Appl, PMC10613632）：σ_z≈70 μm@900 mm、σ_z∝z²、预热与最优条纹周期、超像素收益上限](https://pmc.ncbi.nlm.nih.gov/articles/PMC10613632/)
- [线结构光三维形貌测量系统精度影响因素分析（《光学学报》2024, 44(21): 2112001）：转台选型判据、量块 2.0 μm 与标准球半径 7.3 μm 实测](https://www.opticsjournal.net/Articles/OJ47a1d8ca507c2398/FullText)
- [Camera Calibration（OpenCV 官方教程，张正友法）](https://docs.opencv.org/4.13.0/dc/dbb/tutorial_py_calibration.html)
- [基于单目结构光的苹果三维重建与形态学参数测量（《农业工程学报》2025）：标准球绝对误差 0.293 mm/相对 0.37%；体积 MAPE 1.09%、RMSE 6.95 cm³；统计滤波 K=20 σ=3.0；体素 0.15 mm 降采样；体积用 0.5 mm 体素统计法；重建体积系统性偏低](https://www.sciopen.com/local/article_pdf/10.11975/j.issn.1002-6819.202505078.pdf)
- [Understanding Volumetric Accuracy（Revopoint 厂商博客）：商用扫描仪自报 ±0.02+0.04L mm](https://www.revopoint3d.com/blogs/blog/understanding-volumetric-accuracy)

**开源结构光实现（算法骨架参考）**
- [jiayuzhang128/FourStepPhaseShifting：互补格雷码＋四步相移单目结构光重建，含标定](https://github.com/jiayuzhang128/FourStepPhaseShifting)
- [SHU-FLYMAN/3dStructureLight：从零搭建结构光 3D 重建系统](https://github.com/SHU-FLYMAN/3dStructureLight)

**硬件价格来源（均为检索快照，采购前须重新询价）**
- [微雪 OV9281 1MP 黑白 USB 相机模块](https://www.waveshare.net/shop/OV9281-1MP-USB-Camera-A.htm)｜[淘宝 OV9281 全局快门列表页](https://www.taobao.com/list/product/ov9281%E5%85%A8%E5%B1%80%E5%BF%AB%E9%97%A8.htm)｜[淘宝 42 步进电机驱动列表页](https://www.taobao.com/list/product/wap/42%E6%AD%A5%E8%BF%9B%E7%94%B5%E6%9C%BA%E9%A9%B1%E5%8A%A8.htm)｜[淘宝工业视觉镜头 16mm 列表页](https://www.taobao.com/list/product/%E5%B7%A5%E4%B8%9A%E8%A7%86%E8%A7%89%E9%95%9C%E5%A4%B416mm.htm)｜[淘宝陶瓷校正球列表页](https://www.taobao.com/list/product/%E9%99%B6%E7%93%B7%E6%A0%A1%E6%AD%A3%E7%90%83.htm)｜[淘宝光学偏振片列表页](https://www.taobao.com/list/product/%E5%85%89%E5%AD%A6%E5%81%8F%E6%8C%AF%E7%89%87.htm)｜[淘宝滤光片列表页](https://www.taobao.com/list/product/wap/%E6%BB%A4%E5%85%89%E7%89%87.htm)
- [微雪 Raspberry Pi 5 8GB](https://www.waveshare.net/shop/Raspberry-Pi-5-8GB.htm)｜[微雪 Jetson Orin Nano Super 开发套件](https://www.waveshare.net/shop/Jetson-Orin-Nano-Super-Developer-Kit.htm)｜[NVIDIA Jetson Orin Nano Super 官方页](https://www.nvidia.cn/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/)｜[海康机器人 CS 系列工业面阵相机](https://www.hikrobotics.com/cn/machinevision/visionproduct/?typeId=27&id=249)｜[TI DLPDLCR2000EVM](https://www.ti.com.cn/tool/cn/DLPDLCR2000EVM)｜[TI DLP2010](https://www.ti.com.cn/product/zh-cn/DLP2010)

---

## 7. 「未找到可靠来源」与「不确定」清单

| 条目 | 状态 |
|---|---|
| 赛题**官方评分曲线 / 相对误差分档细则** | **未找到可靠来源**；§3.5 的"1% 是分水岭"为推测 |
| 组委会标准体积的**基准不确定度** | **未找到可靠来源** |
| **IMX477 模组国内实时标价** | **不确定**：微雪页面显示占位 ¥0.00 |
| **海康 CS/CE 系列官方定价** | **未找到可靠来源**：官网需询价；仅淘宝专题页快照 ¥988 |
| **DLP2000EVM 国内售价** | **未找到可靠来源**：TI 页面无价格 |
| **窄带滤光片具体型号报价** | **不确定**：多数厂商标"电议"；仅电商列表快照 ¥38/¥110/¥611 |
| 转台/配准误差 → 体积误差的**定量传递系数** | **未找到可靠来源**；§3.4 为量级估计 |
| 检索快照中出现的"标准球半径相对误差约 6%" | **未采信**：与《光学学报》原文不符（原文为半径 5 mm 标准球测半径误差 7.3 μm），疑为搜索摘要失真 |
| 所有标注【快照】的价格 | **非实时报价**，仅供量级参考 |
