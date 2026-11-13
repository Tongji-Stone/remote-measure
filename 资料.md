# 赛题二 原理书籍清单

书目信息经出版社页面、图书馆书目、WorldCat、Google Books 核实。

---

## 怎么用这份书单

三条：

1. **不要从第一页读到最后一页。** 这些书大多是研究生/工程师级别的，通读一年的时间都未必够。**按赛题难点反查章节**才是正确用法——本文末尾有"按难点查书"的速查表。
2. **优先级最高的是"误差理论"和"相位展开"这两块。** 前面调研的结论很清楚：决定成败的是**系统性偏差**（`ΔV/V ≈ 6δ/L`，5 cm 小件要做到 1%，表面偏差得压到 0.08 mm）和**台阶/孔的相位跳变**，不是相机的像素数。
3. **中文教材打基础，英文专著查细节。** 中文教材便宜、体系完整、术语对得上国内竞赛的语境；英文专著在相位展开、多视图几何、光学计量这些具体问题上更深、更新。

---

## 第一梯队：直接对着赛题来的

### 1. 结构光 / 条纹投影的"总纲"

**《High-Speed 3D Imaging with Digital Fringe Projection Techniques》**
- 作者 Song Zhang（张松，普渡大学）｜CRC Press
- 第 1 版 2016，ISBN 0367869721；**第 2 版** ISBN 9781041191391
- 链接：[Routledge（第2版）](https://www.routledge.com/High-Speed-3D-Imaging-with-Digital-Fringe-Projection-Techniques/Zhang/p/book/9781041191391) ｜ [Taylor & Francis](https://www.taylorfrancis.com/books/mono/10.1201/9781003710240/high-speed-3d-imaging-digital-fringe-projection-techniques-song-zhang)

**一句话**：条纹投影（往物体上投一片会随表面起伏弯掉的条纹，相机拍下来反算高度）这门技术的**标准教科书**，作者是这个方向最活跃的团队之一。

**对应赛题**：路线 A（结构光）的全部——投影什么图案、怎么解出高度、为什么会有误差。

**为什么排第一**：`reports/01` 里那个"像素错一位、整块体积失真"的**相位跳变**问题，以及"深孔内壁收不到条纹"的**原理性极限**，这本书里都有系统讨论。前面报告引用的 Purdue 实验室那篇《Absolute phase retrieval methods for digital fringe projection profilometry》综述，就是这本书作者的团队写的。

**怎么读**：先读讲条纹分析与相位解包裹的章节，再看高速成像部分（我们的物体静止，高速不是重点）。

---

**《Fringe Pattern Analysis for Optical Metrology: Theory, Algorithms, and Applications》**
- 作者 Manuel Servin、J. Antonio Quiroga、J. Moisés Padilla｜Wiley
- ISBN 9783527681105 ｜ DOI 10.1002/9783527681075
- 链接：[Wiley](https://www.wiley.com/en-us/fringe-pattern-analysis-for-optical-metrology-theory-algorithms-and-applications-p-9783527681105) ｜ [Wiley Online Library](https://onlinelibrary.wiley.com/doi/book/10.1002/9783527681075)

**一句话**：专门讲"**怎么从一片弯条纹里把相位算出来**"的算法书——从最基础的相移法（拍几张条纹、每张挪一点，联立算出相位）到傅里叶变换法、再到各种去噪与误差补偿。

**对应赛题**：路线 A 的算法核心。

**和上一本的分工**：张松那本偏"整套系统"，Servin 这本偏"**条纹分析这一步的算法细节与数学**"。如果你要把相移步数、条纹频率、解包裹策略这些参数定下来，这本更顶用。

---

**《Two-Dimensional Phase Unwrapping: Theory, Algorithms, and Software》**
- 作者 Dennis C. Ghiglia、Mark D. Pritt｜Wiley-Interscience
- ISBN 0471249351
- 链接：[Amazon UK](https://www.amazon.co.uk/Phase-Unwrapping-Algorithms-Software-Studies/dp/0471249351) ｜ [Semantic Scholar](https://www.semanticscholar.org/paper/Two-Dimensional-Phase-Unwrapping%3A-Theory%2C-and-Ghiglia-Pritt/c5ed4b0485b535ff60eac93f51eb3837117eb546)

**一句话**：条纹法测出来的是**被折成一圈一圈的相位**（就像把一根长绳子绕在柱子上），这本书专讲"**怎么把它重新拉直、并且处理拉错的地方**"。

**对应赛题**：**这是路线 A 最容易翻车的一环**。`reports/01` 明确指出：台阶和通孔边缘上，"编号"解错一位就会让**整段相位错一整个条纹周期**，体积大块失真；而且这类算法普遍**对不连续区域（也就是台阶、孔）特别敏感**。

**为什么必读**：本书讲的是**通用的**二维相位展开（雷达、干涉测量、条纹投影共用），比单纯看条纹投影的书更能让你理解"为什么孔边上一定会错"。虽然是 1998 年的书，但算法根基没过时。

---

### 2. 双目 / 多视图几何

**《计算机视觉中的多视图几何》（第2版）**
- 原版：*Multiple View Geometry in Computer Vision*, 2nd ed., Richard Hartley & Andrew Zisserman, Cambridge University Press, 2004, ISBN 0521540518
- 中译：**韦穗、章权兵 译，机械工业出版社，2020，ISBN 978-7-111-63288-7，¥169**
- 链接：[牛津 VGG 官方书页（含勘误）](https://www.robots.ox.ac.uk/~vgg/hzbook/) ｜ [豆瓣（中译）](https://book.douban.com/works/1053883?order=time)

**一句话**：**"两台或多台相机拍到的画面之间，几何关系是怎样的"**这门学问的圣经——相机怎么标定、两张图上的同一点怎么对应、能不能从多张图反推出三维结构。

**对应赛题**：路线 C（双目/多目）的全部；以及**所有路线的相机标定环节**。

**注意**：这本书**很数学、很难**（大量射影几何）。不要试图通读。对这道题，重点看：相机模型与标定、对极几何（两张图之间点的约束关系）、三角测量（从对应点算三维坐标）。

**中译本说明**：中译本由机械工业出版社 2020 年出版，译者韦穗、章权兵——对中文读者省很多事。

---

**《计算机视觉：计算理论与算法基础》**
- 马颂德、张正友 著｜科学出版社｜1998
- 百度百科：[链接](https://baike.baidu.com/item/%E8%AE%A1%E7%AE%97%E6%9C%BA%E8%A7%86%E8%A7%89%20%3A%20%E8%AE%A1%E7%AE%97%E7%90%86%E8%AE%BA%E4%B8%8E%E7%AE%97%E6%B3%95%E5%9F%BA%E7%A1%80/53129404)

**一句话**：国内计算机视觉的**经典中文教材**，张正友就是那个提出"张正友标定法"的人（用一块黑白格子板标定相机，你后面一定会用到）。

**对应赛题**：相机标定的中文入门；双目视觉的中文叙述。

**为什么值得看**：年代久（1998），但**相机标定和双目三角测量这两块的基本内容没变**，而且中文讲得清楚、符号不吓人。适合作为英文专著的"翻译层"先垫一下。

---

### 3. 近景摄影测量（"用照片量尺寸"的上位学科）

**《Close-Range Photogrammetry and 3D Imaging》**（第 4 版）
- 作者 Thomas Luhmann、Stuart Robson、Stephen Kyle、Jan Boehm｜De Gruyter
- 第 4 版 2023，ISBN 3111029352 ｜ DOI 10.1515/9783111029672
- 链接：[De Gruyter（第4版）](https://www.degruyterbrill.com/document/doi/10.1515/9783111029672/html) ｜ [De Gruyter（第3版）](https://www.degruyterbrill.com/document/doi/10.1515/9783110607253/html)

**一句话**：**"怎么用相机当量具"**的工程手册——不是"拍好看的照片"，而是"拍完能报出毫米级尺寸"。涵盖相机标定、多视角布设、精度如何估算。

**对应赛题**：**所有路线共用的"标定 + 多视角布置 + 精度评估"**这块。

**为什么值得看**：这本书的视角和竞赛非常契合——它关心的是**" misurabile"（可测量）而不是"好看"**，会讲怎么布置相机的角度和数量才能让误差最小、怎么用标准件验证。这正是 `reports/05` 强调的"用标准球/量块做体积比例校准"的上位方法论。

---

## 第二梯队：决定"1% 能不能做到"的

**这一梯队和上面同等重要，但最容易被队伍忽略。** 前面调研反复指出：**瓶颈是系统性偏差，不是分辨率**。

### 4. 误差理论

**《误差理论与数据处理》（第 7 版）**
- 费业泰 主编（合肥工业大学）｜机械工业出版社
- ISBN 9787111495246（第 7 版）；第 6 版 2010 年出版，初版 1981 年
- 全国高校"测控技术与仪器"专业审定教材
- 链接：[机械工业出版社（第7版）](http://www.cmpedu.com/books/book/2044461.htm) ｜ [百度百科（第6版）](https://baike.baidu.com/item/%E8%AF%AF%E5%B7%AE%E7%90%86%E8%AE%BA%E4%B8%8E%E6%95%B0%E6%8D%AE%E5%A4%84%E7%90%86%EF%BC%88%E7%AC%AC%E5%85%AD%E7%89%88%EF%BC%89/5119400)

**一句话**：国内**测控专业的标准误差课教材**——什么叫系统误差、什么叫随机误差、误差怎么合成、怎么判断结果可不可信。

**对应赛题**：写**误差分析**和**不确定度报告**（海报那 30 分要讲的东西）；也是理解 `ΔV/V ≈ 6δ/L` 为什么成立的基础。

**为什么必读**：这道题**明着考误差分析**（赛题说明里列了"误差分析"）。而且它和别的科目不一样——**花两周读这本书，可能比多买一个相机更能提分**，因为前面调研证明了随机噪声几乎不影响体积（约 0.001%），真正吃分的是系统性偏差。

**价格**：第 7 版淘宝约 ¥39。

---

### 5. 测量不确定度（国家计量技术规范）

**JJF 1059.1—2012《测量不确定度评定与表示》**
- 国家计量技术规范（**是规范，不是书**，但写报告绕不开）
- 另有 **JJF 1059.2—2012**（蒙特卡洛法评定不确定度）
- 链接：[JJF 1059.1 介绍](https://www.yiqifuwu.com/standard/335.html) ｜ [JJF 1059.2 PDF](https://img1.17img.cn/17img/files/201608/ueattachment/1b76afd4-a44c-416c-81cc-1cdeab684350.pdf)

**一句话**：中国**官方规定的"误差该怎么算、怎么报"的规矩**——A 类不确定度（重复测量统计出来的）、B 类不确定度（从仪器说明书等推出来的）、怎么合成。

**对应赛题**：`reports/05` 给了一张"8 项不确定度分解表模板"，那张表的**填法和术语就该照这个规范来**。评委里如果有搞计量的，一眼就能看出你专不专业。

**和上一本的关系**：费业泰那本是"教材"（讲原理、带例题），这份规范是"**标准**"（讲报法、术语必须对齐）。**两份合起来用**：教材学方法，规范对术语。

---

## 第三梯队：共用后半段与基础

### 6. 点云处理

**《点云库 PCL 从入门到精通》**
- 郭浩、苏伟、朱德海 等 编著｜机械工业出版社
- ISBN 978-7-111-61552-1
- 链接：[Amazon（书目信息）](https://www.amazon.es/%E7%82%B9%E4%BA%91%E5%BA%93PCL%E4%BB%8E%E5%85%A5%E9%97%A8%E5%88%B0%E7%B2%BE%E9%80%9A-%E8%8B%8F%E4%BC%9F-%E6%9C%B1%E5%BE%B7%E6%B5%B7%E7%AD%89-%E9%83%AD%E6%B5%A9/dp/7111615522) ｜ [PCL 官网](https://pointclouds.org/)

**一句话**：**中文的 PCL（点云库）实操书**——PCL 是 3D 点云处理的"瑞士军刀"库。

**对应赛题**：`reports/05` 讲的**点云预处理、配准、分割**这一整段。

**⚠️ 提醒**：前面研究的结论是 **Open3D 在 Python/python 生态里比 PCL 更好上手**（报告 05 引用的函数名大多来自 Open3D）。所以这本书的价值是**理解概念和算法**（滤波、配准、分割到底在干嘛），**不是必须照着 PCL 写代码**。如果你的队伍用 C++，它直接就是手册；如果用 Python，可以当"概念字典"翻。

另可参考：[PCL 学习资料整理（知乎）](https://zhuanlan.zhihu.com/p/268524083) ｜ [pcl-learning（GitHub）](https://github.com/HuangCongQing/pcl-learning)

---

### 7. 光学计量 / 光学测量（总论）

**《Handbook of Optical Metrology: Principles and Applications》**（第 2 版）
- 编者 Toru Yoshizawa｜CRC Press
- 第 2 版含 34 章
- 链接：[Taylor & Francis](https://www.taylorfrancis.com/books/edit/10.1201/b18328/handbook-optical-metrology-toru-yoshizawa) ｜ [Optica 书评](https://www.optica-opn.org/home/book_reviews/2018/0218/handbook_of_optical_metrology_principles_and_appl/)

**一句话**：**光学测量的百科全书**——把各种"用光量尺寸"的方法（干涉、条纹、三角测量、散斑……）放在一起讲原理。

**对应赛题**：选路线之前的"**横向比较**"；以及海报里讲"我们这个方法的原理"。适合当字典查，不适合通读。

---

**《Handbook of 3D Machine Vision: Optical Metrology and Imaging》**
- 编者 Song Zhang｜CRC Press
- ISBN 9781138199576 ｜ DOI 10.1201/b13856
- 链接：[Routledge](https://www.routledge.com/Handbook-of-3D-Machine-Vision-Optical-Metrology-and-Imaging/Zhang/p/book/9781138199576) ｜ [Taylor & Francis](https://www.taylorfrancis.com/books/edit/10.1201/b13856/handbook-3d-machine-vision-song-zhang)

**一句话**：**"三维机器视觉有哪几种做法"的横向综述合集**——每种主流三维成像技术一章。

**对应赛题**：**这份书单里最适合"开题前读一章就够"的书**。赛题点名的那些路线（结构光、双目、激光三角法、光切法、多视角、光度立体）基本都有对应章节，正好帮你把 `reports/00-总览.md` 里那张对比表看出所以然来。

---

**《Optical Shop Testing》**（第 3 版）
- 编者 Daniel Malacara｜Wiley-Interscience｜ISBN 0471484040 ｜ DOI 10.1002/9780470135976
- 链接：[Wiley Online Library](https://onlinelibrary.wiley.com/doi/book/10.1002/9780470135976)

**一句话**：**光学车间检测的经典手册**，主要讲干涉测量。

**对应赛题**：**相关性较弱**，除非你走"干涉/相位"路线。列在这里是因为它的**相位与波前测量**章节和相位展开是相通的。**优先级低，可以不读。**

---

### 8. 图像处理与机器视觉基础

**《数字图像处理》（第四版）**
- Rafael C. Gonzalez、Richard E. Woods 著
- 中文版：电子工业出版社，ISBN 9787121377471（定价约 ¥172）
- 英文影印版：电子工业出版社，定价 ¥198
- 链接：[电子工业出版社（英文版）](https://www.phei.com.cn/module/goods/wssd_content_comment.jsp?bookid=65198)

**一句话**：**图像处理的全球标准教材**。

**对应赛题**：**激光光条中心提取**（`reports/02` 讲的极值法、灰度重心法、Steger 亚像素法，本质都是图像处理）、**剪影的边缘提取与阈值分割**（`reports/03` 提到剪影误差超过约 2.5% 就严重退化）。

**怎么读**：只看你用得上的那几章——**图像增强、边缘检测、二值化与形态学、亚像素定位**。别通读。

---

**《机器视觉》**
- 张广军 著｜科学出版社｜2005
- 百度百科：[链接](https://baike.baidu.com/item/%E6%9C%BA%E5%99%A8%E8%A7%86%E8%A7%89/59117690) ｜ [豆瓣](https://m.douban.com/book/subject/1323691/)

**一句话**：中文的**机器视觉系统级教材**，讲机器视觉系统怎么组成、空间几何变换、标定、三维重建。

**对应赛题**：中文语境下的"整机怎么搭、标定怎么做"。可作为 Hartley 那本英文专著的中文过渡。

---

### 9. 光电技术基础

**《光电技术》（第 4 版 / 第 5 版）**
- 王庆有 编著｜电子工业出版社
- 第 4 版 2018 年 6 月，ISBN 9787121195419；另有第 5 版（定价约 ¥60.72）
- "十一五"国家级规划教材
- 链接：[百度百科（第4版）](https://baike.baidu.com/item/%E5%85%89%E7%94%B5%E6%8A%80%E6%9C%AF%EF%BC%88%E7%AC%AC4%E7%89%88%EF%BC%89/49763524) ｜ [电子工业出版社](https://www.phei.com.cn/module/goods/jsjts_list.jsp?Page=19&btid=1181&desc=0&desc1=0&desc2=1&sort=jg&goodtypename=%E6%9C%AC%E7%A7%91%E7%A0%94%E7%A9%B6%E7%94%9F)

**一句话**：光电信息专业的**基础课教材**——光源、探测器（CCD/CMOS）、光电信号怎么处理。

**对应赛题**：**打底**用。选相机、理解感光原理、算信噪比时翻。

---

**《光电检测技术与系统》（第 3 版）**
- 王霞 编著｜电子工业出版社｜2015 年 3 月｜ISBN 978-7-121-25458-1
- 链接：[中科院半导体所书目](https://bdt.semi.ac.cn/library/contents/762/22144.html)

**一句话**：**光电检测系统**怎么设计、组成、抗干扰。

**对应赛题**：同上，打底用；讲"系统"比上一本更贴近工程。

---

**《信息光学》（第三版）**
- 曹益平、苏显渝 主编，张启灿 等参编｜科学出版社｜2024 年 8 月｜ISBN 9787030793355｜¥79
- 第二版（2011）：苏显渝、李继陶、曹益平、张启灿，ISBN 9787030307378，含"三维面形测量"内容
- 链接：[百度百科（第三版）](https://baike.baidu.com/item/%E4%BF%A1%E6%81%AF%E5%85%89%E5%AD%A6%EF%BC%88%E7%AC%AC%E4%B8%89%E7%89%88%EF%BC%89/65095370) ｜ [四川大学三维传感与机器视觉实验室](https://eie.scu.edu.cn/info/1041/7006.htm)

**一句话**：**苏显渝团队**（四川大学，国内结构光三维传感的重镇）主编的信息光学教材，第二版里就含"三维面形测量"。

**对应赛题**：**中文语境下理解结构光/条纹投影的最佳入口**。傅里叶变换轮廓术（FTP）这类方法在中国的发展就与这个团队密切相关。

**为什么值得看**：`reports/01` 提到的"傅里叶变换轮廓术（单张图）、多频外差"等方法，中文的原始论述在这类教材里比英文综述更容易啃。

---

## 按赛题难点查书（速查表）

前面调研收敛出的几个真问题，直接对应到书：

| 赛题难点（来自 reports/） | 看哪几本 |
|---|---|
| **通孔/凹槽被算法"填平"，体积系统性偏大** | 《Handbook of 3D Machine Vision》（横向看哪些方法能保孔）；`reports/05` 是主依据，书里补原理 |
| **台阶/孔边缘条纹"错一位"，整块失真**（相位跳变） | 《High-Speed 3D Imaging…》＋《Two-Dimensional Phase Unwrapping》← **这两本最对症** |
| **相位怎么从弯条纹里解出来** | 《Fringe Pattern Analysis for Optical Metrology》＋《信息光学》 |
| **相机怎么标定、投影仪要不要标定** | 《计算机视觉中的多视图几何》＋《计算机视觉：计算理论与算法基础》＋《Close-Range Photogrammetry and 3D Imaging》 |
| **多视角怎么拼、误差会不会累积** | 《Close-Range Photogrammetry and 3D Imaging》（讲相机布置与精度） |
| **体积怎么从网格/点云算出来** | `reports/05` 为主；《点云库 PCL 从入门到精通》补概念 |
| **激光线中心的亚像素提取** | 《数字图像处理》（边缘/亚像素定位章节） |
| **误差怎么分解、不确定度怎么写** | 《误差理论与数据处理》＋**JJF 1059.1—2012** ← **写海报必看** |
| **为什么"提高分辨率"不解决问题** | 《误差理论与数据处理》（系统误差 vs 随机误差）+ `reports/05` 的 `ΔV/V ≈ 6δ/L` 推导 |
| **相机、光源、探测器基础** | 《光电技术》《光电检测技术与系统》 |

---

## 建议的读书计划（按角色分工）

如果队伍 3~4 人，可以这样分，**别让每个人都从头读到尾**：

| 角色 | 主读 | 目标 |
|---|---|---|
| **光学/结构光负责人** | 《High-Speed 3D Imaging…》选章 ＋《Fringe Pattern Analysis》＋《信息光学》 | 定下条纹频率、相移步数、解包裹策略 |
| **算法/三维重建负责人** | 《计算机视觉中的多视图几何》选章 ＋《Two-Dimensional Phase Unwrapping》＋ `reports/05` | 标定、配准、重建、体积积分 |
| **误差/计量负责人** | 《误差理论与数据处理》＋**JJF 1059.1—2012** ＋《Close-Range Photogrammetry》 | 写不确定度分解表，定标定方案、标准件校准 |
| **机械/电控负责人** | 《光电技术》＋《光电检测技术与系统》 | 选相机、光源、转台、同步触发 |
| **全员**（开题前） | 《Handbook of 3D Machine Vision》对应章节 | 半天时间，建立"有几条路、各自代价"的全局观 |

**时间分配建议**：《误差理论与数据处理》和相位展开那两本**至少各花一周**；其余按需查。**不要把时间全花在相机和结构光的实现细节上而跳过误差分析**——前置调研的结论是，这道题的分数更容易在误差分析上拿，而不是在器件档次上。

---

## 备注

- 《Close-Range Photogrammetry and 3D Imaging》第 4 版：De Gruyter 官方页用 DOI `10.1515/9783111029672`，零售商页给的 ISBN 是 3111029352，**买书以出版社页面为准**。
- 《High-Speed 3D Imaging…》第 2 版出版年份未取到；第 1 版为 2016 年。
- 书中价格（¥39 / ¥79 / ¥169 / ¥172 / ¥198）是页面标价快照，非实时报价。

---

## 附：前面调研的报告（与本书单配套）

- [00-总览.md](reports/00-总览.md) —— 五条路线横向对比、分歧裁决、推荐路线
- [01-structure-light.md](reports/01-structure-light.md) —— 结构光 / 条纹投影
- [02-laser-triangulation.md](reports/02-laser-triangulation.md) —— 激光三角法 / 光切法
- [03-stereo-multiview.md](reports/03-stereo-multiview.md) —— 双目 / 多视角 / 光度 / 偏振
- [04-products-contests.md](reports/04-products-contests.md) —— 市面产品 / 开源项目 / 历届竞赛 / 合规红线
- [05-algorithm-metrology-cost.md](reports/05-algorithm-metrology-cost.md) —— 算法 / 计量 / 成本
