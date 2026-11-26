# M01 文献与数学依据

## 从装置限制到实验

| 方向 | 来源中的方法 | M01 实际用法 |
| --- | --- | --- |
| 平面标定与尺度 | 利用多姿态平面单应恢复内参和外参，随后优化像点重投影 | 从两套已解码的结构光点重建同一组露板附近的空间长度，按站统计 x、y、斜向尺度。用同 S04 核比较标定切换的下游影响 |
| 刚体与图同步 | 李群指数映射、图上单位元闭合、边 1-形式的恰当分量 | H1 将边差移至同一锚定坐标上的李代数，再用带权图 Poisson 同步，和 R02/R06c 比较 |
| PDE 与有符号偏差 | Screened Poisson 重建定向场的等值面；形状微积分把体积一阶变化写成边界积分 | S04 原解作起点，D1/D2 校正可见面，D4 将面积加权有符号残差作为选型目标 |
| 截面积与形貌正则 | Fubini 分层积分，周期 Fourier 基表达角向截面 | R-pole／R-flat 拟合星形截面的半径平方，同时积分并生成待核验网格 |
| 拓扑与计量 | 边界拓扑作为形状约束；按输入的联合分布传播不确定度 | V001 核对 Euler 示性数、方向、贯穿孔与内腔。M01 输出物理扰动的灵敏度，采集重复和参考计量用于下一步估计输入分布 |

## 已核查的原始来源

1. **Zhang, A Flexible New Technique for Camera Calibration.** TPAMI 2000，§2–3，平面约束与最大似然精修。对平面，归一化相机光线 $q=(x,y,1)$，由 $n^\top(Zq)+d=0$ 可得 $1/Z=-(n^\top q)/d$。M01 的平面距离诊断由此导出。 [Microsoft Research 原文与 DOI](https://www.microsoft.com/en-us/research/publication/a-flexible-new-technique-for-camera-calibration/)
2. **Solà, Deray, Atchuthan, A micro Lie theory for state estimation in robotics.** arXiv v9，PDF/印刷 p.16–17，Appendix D，SE(3) exp/log 及式 (181–183)；作者坐标写作 $(\rho,\theta)$，程序参数为 $(\omega,v)$。验证 PDF SHA256 为 `93190a60ad5f390006970f684ba8590bf55ce21adc3c1cb675adc55ead17f869`。 [v9 原文](https://arxiv.org/pdf/1812.01537v9)
3. **Ren, Jiang, Chen, Lei, Gu, Global Consistent Point Cloud Registration Based on Lie-algebraic Cohomology.** 已逐页核查用户所存版本 `note/2208.07103v1.pdf`，PDF/印刷 p.6–8。式 (5) 将边修正延到嵌入的曲面，式 (6–9) 将 1-形式分解并用 Poisson 方程取恰当分量。H1 只使用其核心投影思想，直接在采集图上用关联矩阵，权重来自重叠率和鲁棒残差，不做原论文的闭曲面嵌入与余切权。固定规范后，边上的精确差分由节点位姿生成。PDF SHA256 为 `bf59fd6b2af892c99d0d7651351b536b692104d11d8df113c55f8f1b0c3df9c0`。 [v1 原文](https://arxiv.org/pdf/2208.07103v1)
4. **de Almeida, Giraldi, Vieira, Applying Lie Groups Approaches for Rigid Registration of Point Clouds.** 其 Lie 群建模对象包含局部高斯张量，论文也指出直接张量比较的旋转不变性限制。该方向对后续基于曲率建立边有帮助。M01 用已标定转台与 Tukey 点到面建边。 [预印本](https://arxiv.org/abs/2006.13341v1)
5. **Kazhdan, Hoppe, Screened Poisson Surface Reconstruction.** SIGGRAPH 2013。稀疏插值约束增强原 Poisson 的数据约束。M01 保留 E104 S04 的接触面、射线自由空间和露板掩膜，以便把位姿和补偿影响分别核对。 [作者论文与实现](https://hhoppe.com/proj/screenedpoisson/) · [包络约束扩展](https://gfx.cs.princeton.edu/pubs/Kazhdan_2020_PSR/index.php)
6. **Sokołowski, Zolésio；Laurain, Sturm。** 前者为形状灵敏度教材；后者说明 Hadamard–Zolésio 结构导数和边界表达的条件。取域积分常函数 1，从 (x\mapsto x+\varepsilon u(x)) 的雅可比得 (\frac{dV}{d\varepsilon}|_0=\int\operatorname{div}u=\int_{\partial\Omega}u\cdot n)。M01 另外从三角面体积多项式直接微分，适用于多面网格。 [Springer 原书](https://doi.org/10.1007/978-3-642-58106-9) · [Laurain–Sturm v3](https://arxiv.org/abs/1509.01816v3)
7. **Brüel-Gabrielsson, Ganapathi-Subramanian, Skraba, Guibas, Topology-Aware Surface Reconstruction for Point Clouds.** CGF 39(5)，197–207，§3。用持续图约束基函数构成的似然场。V001 先验证贯穿孔和空壳，M01 的局部形变固定网格连通并做自交和方向检查。在未照到的区域，预设同调可能与真物体冲突。 [DOI](https://doi.org/10.1111/cgf.14079) · [出版 PDF](https://diglib.eg.org/bitstream/handle/10.1111/cgf14079/v39i5pp197-207.pdf)
8. **Niyogi, Smale, Weinberger, Finding the Homology of Submanifolds with High Confidence from Random Samples.** 结论依赖流形几何与采样条件。现有结构光点在底部和暗面有非随机成片缺失，孔洞的稳定性需要补扫证据。 [作者原稿](https://math.uchicago.edu/~shmuel/NSW1.pdf) · [大学技术报告条目](https://newtraell.cs.uchicago.edu/research/publications/techreports/TR-2004-08)
9. **Mirtich, Fast and Accurate Computation of Polyhedral Mass Properties.** JGT 1996，用散度和投影把边界矩积分化简。V001 用较直接的有向四面体和，另用多段柱积分复核。 [作者论文与源码](https://people.eecs.berkeley.edu/~jfc/mirtich/massProps.html)
10. **JCGM 100／101。** GUM 及联合输入分布的 Monte Carlo 传播。对跨站共模尺度，独立地给每点加噪声会漏掉系统项。M01 给出确定性敏感度。重复实验和独立真值可用于建立有数据支持的不确定度。 [JCGM 100](https://doi.org/10.59161/JCGM100-2008E) · [JCGM 101](https://www.bipm.org/en/doi/10.59161/jcgm101-2008)

### 研究边界

图中闭合回路的代数乘积由节点矩阵构造，恒等只校验实现与坐标约定。判定配准是否更真实用留出物体点、留出码、盘面和体积对照。Gauss–Bonnet 可解释小的全表面平行偏移的三阶项；真实局部补片、棱角与拓扑变化需直接处理。M01 统计闭合网格的 Euler 示性数，并精确计算有限变形体积。持续同调损失及位姿—隐式场的全联合优化留待独立物理参照和遮挡数据到齐后量化。
