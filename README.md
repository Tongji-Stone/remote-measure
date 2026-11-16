1.to-do：联系顿雄，阅读其论文方向

2.学习B站三维结构光/三维重建基础

3.盘点器材，组装

## 结构光实验数据

原始采集图像和完整试验数据存放在[同济云盘](https://yunpan.tongji.edu.cn/link/AAAE071E5B2DF34896A8C3A9EE4046624D)，不提交到 Git。链接允许预览和下载，有效期至 **2026 年 12 月 15 日 15:54（北京时间）**，到期后需要重新分享。较小的试验参数、点云及预览图保存在 [results/structured-light](results/structured-light/README.md)。

压缩包解压到项目的 `data/` 目录后，会合并到 `data/procam_calibration/` 或 `data/object_scans/`：

| 文件 | 内容 | SHA-256 |
| --- | --- | --- |
| `01_早期采集.zip` | `之前的/` 中的早期采集 | `996cddf14930a87295a656aa0780dbc2b171eee62fd5c29de8558e357a1e545d` |
| `02_标定图片.zip` | 11 组 12:12～12:19 标定板采集 | `f8fe2fe3e448a5950945ee17017a5d5d4de04c2f136eeb914e5ca193ad3dd4ee` |
| `03_物体与结果.zip` | 物体采集、点云和试验结果，包括未验证的标定参数 | `90f9e6bf0bd8af6cb4772d875a61d19c4508d9c504a3aadfc192ebbb7ef4d35b` |
| `20261116_calibration.zip` | 11 月 16 日的 16 组棋盘格采集、标定参数及验证记录 | `125ec0813ce11713329bff2a0e37db5a6caf5fc6a39827effa7b509276e6ed64` |
| `20261116_object_scans.zip` | 11 月 16 日的 6 组物体和转台采集及其试验结果 | `a65469832303abd033069e4d696a9722db5e38860b70b33926f2d50f1fea01dd` |

这些数据和标定结果属于试验记录，不能直接作为已验证的尺寸或体积测量结果。操作步骤见 [结构光扫描使用说明](reports/07-结构光扫描使用说明.md)，标定与误差分析见 [技术说明](reports/08-结构光标定与误差技术说明.md)。
