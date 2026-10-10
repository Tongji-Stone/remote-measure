# 方向一：结构光 / 条纹投影轮廓术（FPP）方案调研

> **来源标注**：✅ 实际抓取全文；⚠️ 仅见检索摘要未核实；"未找到可靠来源"=真没查到。

## 一、原理（人话）

投影仪往物体打明暗平滑渐变的条纹，相机侧看被表面"压歪"的条纹，反推每点深度。

- **相移法**：同一条纹平移 3~4 次各拍一张，反正切算相位——等于用几把尺子量同一段长度取平均，把噪声平均掉。
- **包裹相位**：反正切只吐 −180°~+180°，超了绕回一圈，像时钟只显示 0~59 分，于是相位图是一圈圈锯齿。
- **相位解包裹**：把折叠的条纹重新拉直。**这步是命门。**
- **格雷码**：额外投黑白"二维码"图，告诉相机"你在第几圈"。靠多张图随时间定位=**时间编码**，不怕台阶；靠邻像素猜圈数=**空间编码**，只投一张但**遇台阶必崩**。
- **多频外差**：两组频率略不同的条纹相减合成超长周期，像游标卡尺。
- **相位—高度模型**：相位差换算高度的关系式；有一种只需一块平板当参照，**不用标定投影仪**。

✅[MDPI Sensors 2025 综述](https://www.mdpi.com/1424-8220/25/20/6296)、✅[张宗华综述（红外与激光工程 2023, 52(8)）](https://www.researching.cn/ArticlePdf/m00018/2023/52/8/20230126.pdf)：时间展开精度高、可测不连续面但图案多效率低；空间展开高效但不适用非连续面。本题有台阶通孔 → **只能走时间编码**。

## 二、方案清单

| # | 方案 | 原理一句话 | 精度/速度 | 硬件与成本 | 优点 | 缺点 | 合规 |
|---|---|---|---|---|---|---|---|
| 1 | **单目相移+格雷码** ★主力 | 4 张相移算包裹相位 + 4~6 张格雷码定级次 | 实测**体积平均相对误差 1.09%**、标准球绝对误差 0.29 mm ✅；8~10 张图，60~120 Hz 下单视角 0.1~0.2 s | 投影仪 ¥1000~4700⚠️ + 黑白工业相机 ¥1080~1400⚠️ + 镜头/转台/打印件估 ¥500~1000（镜头与电机**未找到可靠来源**）。**整机估 ¥2500~7000** | 开源代码现成；能处理台阶；精度够；器件全在允许清单 | 格雷码错一位→整段相位错一整个条纹周期（周期 5 mm 就错 5 mm），体积大幅失真；位数多了条纹太细相机分不出 | ✅ |
| 2 | 单目相移+多频外差 | 投 2~3 组不同频率条纹，低频帮高频"数圈" | 与格雷码同档，图案数可更少 | 同上 | 周期误判容限更宽，不需解码图 | 频率比选不好有"合成波长"漏洞 | ✅ |
| 3 | 双目结构光（**自搭**） | 不标定投影仪，两相机立体匹配，投影仪只给白表面加纹理 | 与单目同档 | 多一台相机 +¥1000~1400 | 跳过投影仪标定；有一致性检查能剔错匹配 | 基线和双相机更占地方，50 cm 立方体内要算清；标定翻倍 | ✅ |
| 4 | 纯格雷码/二值编码 | 只投黑白编码图，解出投影仪列/行号再三角测量 | **仅投影仪像素级**，无亚像素；✅NIST 表给 0.032 mm（⚠️未标量程，不能直接搬到 5–20 cm） | 同 1 | 简单 | OpenCV 实现里 1024×768 投影仪要投 **40 张**✅，又慢又不准 | ✅ 但不做主力 |
| 5 | 傅里叶变换轮廓术（单张） | 单张图在频域滤出变形信息 | 极快 | 同 1 | 只要一张 | **不能测台阶断层**（频谱串扰） | ❌ **本题不可用** |
| 6 | 单目+转台多视角+封闭网格体积 | 8~12 角度各扫一次，ICP（把两片点云来回挪到最贴合）拼成封闭模型再算体积 | ✅ 苹果论文正是这套：体积平均相对误差 1.09%、R²=0.98、体积 RMS 6.95 cm³ | 42 步进电机 + TB6600 驱动器 **$14~20**⚠️[DFRobot](https://www.dfrobot.com/product-1547.html)，电机本体**未找到可靠来源** | 解决遮挡；体积是最终目的 | 拼接误差让体积"虚胖"（见难点 4） | ✅ |
| 7 | **商用结构光扫描仪** | — | — | — | — | — | ❌ **禁用**，仅作对照 |

方案 1 来源：✅[fpp_tools](https://github.com/nzhagen/fpp_tools)、✅[kamino410/phase-shifting](https://github.com/kamino410/phase-shifting)、✅[苹果体积论文](https://www.sciopen.com/article/10.11975/j.issn.1002-6819.202505078)

**投影端**：`DLP LightCrafter 4500 EVM` **4225 Hz 二值 / 120 Hz 8-bit 灰度、带触发输出** ✅[TI](https://www.ti.com/tool/DLPLCR4500EVM)，但 EKB 报 **$1119**⚠️、淘宝 DLP4500 光机 **¥10600**⚠️，且 DigiKey 页已标 Discontinued⚠️——**采购风险最大，必须确认现货**。`DLP3010EVM-LC` 0.3" 720p、**2 输入触发 + 1 输出触发** ✅[TI](https://www.ti.com/tool/DLP3010EVM-LC)，约 **$916~964**⚠️。普通 1080p 家用 DLP **¥999~4700**⚠️，但**最多 60 图/秒、无触发口**。自制 LED 投影/掩模无法快速切图案，**不可行**。✅[SLStudio README](https://github.com/jakobwilm/slstudio) 原话大意：很少有投影仪提供触发输出，LightCrafter4500 有、是推荐设备。

**投影仪为什么不踩禁用清单**：允许清单里明确写了"投影器"。它只是**可控显示屏**，自己不算深度、不吐点云。危险的是成品结构光 3D 相机、商用扫描仪、深度相机/TOF——它们能直接输出深度图/点云，**只能辅助定位，不能作体积主数据源**。

## 三、相机端与同步

- **必须用黑白相机**：彩色相机的滤镜阵列让每像素只收 1/3 光，去马赛克还会在细条纹上造假彩色。
- **全局快门 vs 卷帘快门**：全局快门=所有像素同一瞬间曝光；卷帘=逐行扫描曝光。本题物体静止、相机不动、只有投影仪换图案——只要"图案换完、画面稳定后再曝光"，卷帘**也未必致命** ⚠️（原理推理，**未找到验证该条件的文献，不确定**）。
- **同步**：最稳是硬件触发（投影仪出脉冲→相机曝光）。✅SLStudio 明确要"从投影仪到相机的触发信号线"，并做了 **Arduino 触发电路**取脉冲生成相机时序。无触发口的普通投影仪只能软件同步（投→等→拍），降到 60 图/秒以下但仍够用。
- **必须关掉自动曝光/增益/白平衡**，否则亮度在图案间乱变，直接毁掉相移法。
- **分辨率算术（估算非实测）**：1/1.8" 传感器 + 8 mm 镜头 + 工作距离 500 mm → 视场约 200 mm、2000 像素宽 → **约 0.1 mm/像素**；条纹取 8 像素一周期 → 周期 0.8 mm；相位精度约周期的 1/50~1/100 → 深度分辨约 **0.01~0.05 mm**。

## 四、标定

- **相机标定（张正友标定法）**：拿棋盘格板从十几个角度拍，反推相机内参和畸变；OpenCV `calibrateCamera`、MATLAB Camera Calibrator 都能做。
- **投影仪标定难在**：投影仪只能投不能看，主流做法是当"反向相机"——棋盘格投到平板上、用已标定相机拍下来建立投影仪像素与三维点的对应 ✅（MDPI 综述：投垂直+水平两组条纹，相移+时间解包裹得两方向绝对相位，再算投影仪像素坐标）。
- **推荐替代**：**完全跳过投影仪标定**，用"相位—高度模型"。✅[fpp_tools](https://github.com/nzhagen/fpp_tools)：先在**没放物体时拍一组条纹**当参考平面，放上物体再拍一组，两者相位差 Δφ 换算高度，只需条纹空间频率 f₀、相机到参考面距离 L₀、投影仪与相机间距 d。**不需平移台、不需标定投影仪，成本最低**；代价是旁轴近似，量程大时精度下降。
- **标定板**：SLStudio 建议**印在平板上的灰白棋盘格**，在整个视场里带倾斜角拍很多位置 ✅。但**纸上打印的棋盘格其实不是平的**（纸会翘）会引入系统误差；玻璃/陶瓷/铝基板更好（价格**未找到可靠来源**）。
- **标定误差对体积的影响**：体积约正比尺寸三次方，**dV/V ≈ 3×dL/L**。边长 100 mm 的长方体、边长误差 0.2 mm（0.2%）→ 体积误差约 0.6%。⚠️**标准误差传播推导，非实测数据。**

## 五、开源项目与代码

| 项目 | 语言/许可 | 能力 | 评价 |
|---|---|---|---|
| ✅[fpp_tools](https://github.com/nzhagen/fpp_tools) | Python, MIT | 4 步/N 步/非等间距/未知相移；含**相位—高度公式**与参考平面差分法 | **最推荐入门** |
| ✅[slstudio](https://github.com/jakobwilm/slstudio) | C++/Qt | 2×3 相移（6 张图）、多种编码、三角测量、Arduino 触发器、Matlab 伽马测量 | 工程最完整、实时 20 Hz；依赖 Qt + 特定相机 SDK，**上手陡** |
| ✅[phase-shifting](https://github.com/kamino410/phase-shifting) | Python, MIT | 3 步相移 + 格雷码 + 两组不同频率（step/3、step/2），带边界插值与掩模 | **最贴本题**，代码量小 |
| ✅[OpenCV `structured_light`](https://docs.opencv.org/4.13.0/d1/dec/classcv_1_1structured__light_1_1GrayCodePattern.html) | C++/Python | `GrayCodePattern` + `SinusoidalPattern` | **有硬伤，见下** |
| ✅[OSI 低成本扫描仪](https://www.opensourceimaging.org/project/structured-light-3d-scanner/) | Python, CC | 普通投影仪（原生 1024×768）+ 两摄像头 | 证明便宜投影仪也能扫，精度不高 |

**OpenCV 硬伤** ✅：① 生成**传统黑白格雷码、无相移**，只有投影仪像素级精度；② 图案爆炸——官方写明 1024×768 投影仪 Ncols=10、Nrows=10，加**反色图**共 10×2+10×2=**40 张**；③ 内建 **3DUNDERWORLD** 靠"两台相机视角"算阴影掩模 → **本质双目，不配套单目**；④ 只吐**视差图**，三角测量要自己写；⑤ 不做解包裹（`SinusoidalPattern` 支持 FTP/PSP/FAPS，展开要另配 `cv::phase_unwrapping`）。社区反馈"单目+格雷码三角测量得到扭曲点云"⚠️[SO](https://stackoverflow.com/questions/78844136/structured-light-3d-scan-triangulation-using-opencv-in-python-delivers-warped-r)——**照抄会翻车**。

**算体积**：Open3D 的 `TriangleMesh.get_volume()`（把封闭曲面切成很多小三角形，逐块体积贡献求和）是常用做法，**但坑深**：官方 issue 报告"形状相近的网格算出的体积差异巨大"⚠️[#4710](https://github.com/isl-org/open3d/issues/4710)，根因是 `create_from_point_cloud_alpha_shape` 生成的网格**不水密**（有缝有洞）。**必须先保证网格水密**，或改在深度图上做体素积分。

## 六、精度基准

- **最好的公开对照**：✅[单目条纹投影测苹果（农业工程学报 2025）](https://www.sciopen.com/article/10.11975/j.issn.1002-6819.202505078)：相移+格雷码+单目+转台+ICP；标准球绝对误差 0.29 mm/相对 0.37%；球拟合 RMS <0.095 mm；平面重建误差 0.387 mm；40 个红富士实测**体积平均相对误差 1.09%**、RMS 6.95 cm³、R²=0.98。**物体尺寸与本题高度重合**（苹果直径 7~9 cm、体积约 200~400 cm³）→ **建议把"体积误差 ≤2%、力争 ≤1%"作为本题指标。**
- ✅[NIST《Sources of Errors in Structured Light 3D Scanners》](https://tsapps.nist.gov/publication/get_pdf.cfm?pub_id=927473)：混合编码 <0.050 mm、Gray/二值编码 0.032 mm（⚠️未标量程）；明确指出"**大多数光学扫描仪在高反光和/或暗表面上表现很差；三角测量角大时凹面也难测**"。误差来源排序：标定与畸变→标定件质量→景深→**三角测量角/构型**→编码方式→光波长。
- ⚠️**未找到**"5–20 cm 零件/组合体、无喷粉无标记"条件下可直接引用的体积误差论文数据。苹果数据是最接近的参照。

## 七、关键难点与坑（按杀伤力排序）

1. **相位跳变（最致命）**：台阶/通孔边缘格雷码解错一位→整段相位错一整个条纹周期→体积大块失真。✅[Applied Optics 2026](https://opg.optica.org/ao/fulltext.cfm?uri=ao-65-8-2530)：**互补格雷码**与**三分相位展开**属"事前规避"，但仍解决不了级次解码错误引起的跳变。→ 对策：互补格雷码（多投反色图）／2~3 组多频外差互校／相位分区纠错。**条纹周期要小于最小台阶、又要大到相机能分辨——本方案最核心的参数取舍。**
2. **阴影与遮挡**：投影仪与相机不同位，台阶背面、深孔内壁、凹槽底**收不到条纹**，这些像素没有相位。→ 转台多视角（苹果论文靠旋转+ICP"把被遮挡区域几何形状有效恢复"✅）；或两方向轮流投影。**但深盲孔内壁、通孔内壁本质上测不到，这是本路线的原理性极限。**
3. **通孔/凹槽体积扣除**：单视角看不到通孔→把孔当实体→**体积系统性偏大**。→ 多视角+网格水密；宁可用体素法（把空间切成小方块逐个判断内外），也别盲目用 Poisson 重建（会在孔洞上"糊一层皮"把孔堵死）。
4. **拼接让体积"虚胖"**：ICP 配准误差使两片点云错开，重叠区成两层薄壳。→ 标定转台轴线；用重叠区均方误差做质检；必要时取平均而非叠加。
5. **投影仪伽马非线性**：投出的"灰度 128"实际不是线性一半亮度→条纹变成带谐波的非正弦→相位出现周期波纹，表面浮起规则小鼓包。→ 用 8 步以上相移（✅fpp_tools 明确说 N=8 可在不标定投影仪的情况下压低该误差）或伽马反校正。
6. **环境光**：压低条纹对比度、降低点云质量 ✅[PMC 环境光研究](https://pmc.ncbi.nlm.nih.gov/articles/PMC9400401/)。→ 投**蓝光**+相机前加**窄带滤光片**（只放投影仪那个颜色进来）。滤光片、偏振片都在允许清单。⚠️蓝光抗环境光主要来自厂商博客 [3D ScanTech](https://www.3d-scantech.com/blue-light-or-white-light-3d-scanners/)，学术依据待核。**最省事一招：拉窗帘遮光。**
7. **高反光/暗表面**：本题写明物体**漫反射**，风险**低**。万一抽到抛光金属件：多次曝光 HDR（同图案拍多档曝光，取各自不饱和的像素拼起来）✅[Optica HDR-FPP](https://opg.optica.org/oe/fulltext.cfm?uri=oe-33-5-10965)、✅[高反光表面测量](https://pmc.ncbi.nlm.nih.gov/articles/PMC11945723/)；或偏振片交叉消反光 ⚠️[MDPI Photonics](https://www.mdpi.com/2304-6732/12/1/27)。**赛题禁止喷粉贴标记，"喷显影剂"不能用。**
8. **运动敏感**：物体静止→风险极低，但**转台必须停稳再拍**。⚠️[两频相移 vs 格雷码对比](https://www.sciencedirect.com/science/article/pii/S0143816622000501)（仅摘要）指出互补格雷码在运动量达条纹周期 1/18 时就出现解包裹错误。
9. **时间预算完全够**：8~12 视角 × 8~12 张 = 64~144 帧，60~120 Hz → 采集 1~3 s，加转台定位总采集 <10 s。**60 秒瓶颈不在采集，在解包裹、拼接和体积计算**，算力要留足。

## 八、结论：能不能用在这道题上

| 方案 | 结论 | 踩禁用清单？ |
|---|---|---|
| 单目相移+格雷码+转台多视角+水密网格/体素体积 | ✅**首选**，1% 量级精度可期，成本低，器件全在允许清单 | 不踩 |
| 单目相移+多频外差 | ✅ 可用，作备选或交叉校验 | 不踩 |
| 双目结构光（**自搭**） | ✅ 可用，但占空间、标定翻倍 | 不踩 |
| 买成品双目结构光 3D 相机 | ❌ **禁用** | **踩**（直接输出深度图/点云的成熟商用模块） |
| 纯格雷码/二值编码 | ⚠️ 只有像素级精度，OpenCV 要 40 张图，不做主力 | 不踩 |
| 傅里叶变换轮廓术（单张） | ❌ **不可用**，测不了台阶断层 | 不踩 |
| 普通 1080p 家用投影仪 | ✅ 可用，成本最低；无触发口、≤60 图/秒，需软件同步 | 不踩 |
| TI DLP LightCrafter 4500 EVM | ✅ 性能最好（4225 Hz 二值/120 Hz 8-bit，带触发输出），但 **$1100+ 且可能已停产** | 不踩（可编程投影器≠扫描仪） |
| 深度相机/TOF（奥比中光、RealSense） | ⚠️ 只能辅助定位或演示，**不能作体积主数据源** | **踩**（若当主数据源） |
| 商用三维扫描仪 | ❌ **禁用** | **踩** |

**一句话推荐**：单目相移 + **互补格雷码**做主链路、**多频外差**交叉校验；投影端用**普通 1080p DLP 或 DLP3010EVM-LC**（避开可能停产的 DLP4500）；黑白工业相机 + 8~12 mm 定焦镜头；42 步进电机转台做多视角；用**相位—高度模型**跳过投影仪标定；最后用**体素法或水密网格**算实体体积并显式扣除通孔。

## 九、参考链接

**实际抓取全文（✅）**
1. [MDPI Sensors 25(20):6296, 2025 — FPP/PMD 综述](https://www.mdpi.com/1424-8220/25/20/6296)
2. [张宗华等，红外与激光工程 2023, 52(8) — 相位展开技术综述](https://www.researching.cn/ArticlePdf/m00018/2023/52/8/20230126.pdf)
3. [nzhagen/fpp_tools](https://github.com/nzhagen/fpp_tools)
4. [jakobwilm/slstudio](https://github.com/jakobwilm/slstudio)
5. [kamino410/phase-shifting](https://github.com/kamino410/phase-shifting)
6. [OpenCV GrayCodePattern 文档](https://docs.opencv.org/4.13.0/d1/dec/classcv_1_1structured__light_1_1GrayCodePattern.html)
7. [OpenCV Structured Light 教程](https://docs.opencv.org/5.0/tutorials_contrib/structured_light/structured_light.html)
8. [Open Source Imaging — 低成本结构光扫描仪](https://www.opensourceimaging.org/project/structured-light-3d-scanner/)
9. [TI DLPLCR4500EVM](https://www.ti.com/tool/DLPLCR4500EVM)
10. [TI DLP3010EVM-LC](https://www.ti.com/tool/DLP3010EVM-LC)
11. [农业工程学报 2025 — 单目条纹投影苹果体积测量](https://www.sciopen.com/article/10.11975/j.issn.1002-6819.202505078)
12. [NIST — Sources of Errors in Structured Light 3D Scanners](https://tsapps.nist.gov/publication/get_pdf.cfm?pub_id=927473)
13. [Applied Optics 65:2530-2538, 2026 — 格雷码相位跳变纠正](https://opg.optica.org/ao/fulltext.cfm?uri=ao-65-8-2530)
14. [PMC — 高反光表面条纹投影测量](https://pmc.ncbi.nlm.nih.gov/articles/PMC11945723/)
15. [Open3D issue #4710 — 网格体积计算不稳定](https://github.com/isl-org/open3d/issues/4710)
16. [Stack Overflow — Open3D 网格体积计算](https://stackoverflow.com/questions/76752991/open3d-how-to-calculate-the-volume-of-a-mesh-created-by-point-cloud)

**仅见检索摘要、未逐页核实（⚠️）**
17. [EKB — DLP LightCrafter 4500 报价 1119 美元](https://www.ekbtechnologies.com/e-store/dlp-lightcrafter-4500)
18. [DigiKey — DLPLCR4500EVM（标 Discontinued）](https://www.digikey.com/en/products/detail/texas-instruments/DLPLCR4500EVM/4288515)
19. [DFRobot — TB6600 驱动器 20 美元](https://www.dfrobot.com/product-1547.html)
20. [投影网 — 千元 DLP 投影仪推荐](https://www.touying.com/t-55285-1.html)
21. [Opt. Lasers Eng. 153:106995, 2022 — 两频相移 vs 格雷码](https://www.sciencedirect.com/science/article/pii/S0143816622000501)
22. [Optics Express 33(5):10965 — HDR-FPP](https://opg.optica.org/oe/fulltext.cfm?uri=oe-33-5-10965)
23. [MDPI Photonics 12(1):27 — 偏振法 FPP](https://www.mdpi.com/2304-6732/12/1/27)
24. [PMC — 环境光对三维测量精度的影响](https://pmc.ncbi.nlm.nih.gov/articles/PMC9400401/)
25. [3D ScanTech — 蓝光 vs 白光（厂商博客）](https://www.3d-scantech.com/blue-light-or-white-light-3d-scanners/)
26. [Wikipedia — Structured light](https://en.wikipedia.org/wiki/Structured_light)
27. [Stack Overflow — 单目格雷码三角测量点云扭曲](https://stackoverflow.com/questions/78844136/structured-light-3d-scan-triangulation-using-opencv-in-python-delivers-warped-r)
28. [guillaume-charron/3DScanner-GrayCode](https://github.com/guillaume-charron/3DScanner-GrayCode)
