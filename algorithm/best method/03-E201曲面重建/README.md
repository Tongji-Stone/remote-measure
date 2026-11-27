# Simulation E201 优选曲面重建

| 算法 | 用法 | 小南瓜 | 坐着人偶 | 三棱锥大面 | 三棱锥小面 |
|---|---|---:|---:|---:|---:|
| **[S08-PrecisePoisson](S08-PrecisePoisson/S08-PrecisePoisson.py)** | 用 Poisson 拟合表面，按多角站实测空域裁剪，沿实测盘面截线封口。南瓜和人偶的计量主线。 | **20.01** | **44.63** | 131.51 | 125.89 |
| **[S09-ExposedPlate](S09-ExposedPlate/S09-ExposedPlate.py)** | 在 S08 上增加同期露板证据，修正约 1.2 mm 的贴盘薄层。两组三棱锥的计量主线。 | — | — | **131.47** | **125.88** |
| **[G03-2DGS-anchored](G03-2DGS-anchored/G03-2DGS-anchored.py)** | 用二维高斯盘重建的局部表面指导实测点沿法线微调，单点最多 0.15 mm，再用 Poisson 和 S08 封口。用于高斯路线对照。 | 20.43 | 45.44 | 132.27 | 126.74 |

体积单位为 mL。南瓜、人偶使用 E204 跨姿势融合点云，两组三棱锥使用各自的 E202 整圈点云。三个方法的闭合网格均以实测盘面为 (z=0)，坐标单位为 mm。实物体积误差需用独立参考测量验证。

## 文件

各方法目录内的入口 `.py`、指标 `.json`、各物体的预览 `.png` 和彩色三角网格 `.ply` 均使用同一算法名。每张预览从左到右为 RGB 输入点云、同角度重建网格、重建底视。网格 PLY 含面片，可用于体积积分。JSON 记录原始输入点云路径、盘面、拓扑、体积、射线误差和独立积分核验。

- **S08** · [指标](S08-PrecisePoisson/S08-PrecisePoisson.json) · [南瓜](S08-PrecisePoisson/小南瓜/S08-PrecisePoisson.png) · [人偶](S08-PrecisePoisson/坐着人偶/S08-PrecisePoisson.png) · [棱锥大面](S08-PrecisePoisson/三棱锥_大面坐地/S08-PrecisePoisson.png) · [棱锥小面](S08-PrecisePoisson/三棱锥_小面坐地/S08-PrecisePoisson.png)
- **S09** · [指标](S09-ExposedPlate/S09-ExposedPlate.json) · [棱锥大面](S09-ExposedPlate/三棱锥_大面坐地/S09-ExposedPlate.png) · [棱锥小面](S09-ExposedPlate/三棱锥_小面坐地/S09-ExposedPlate.png)
- **G03** · [指标](G03-2DGS-anchored/G03-2DGS-anchored.json) · [南瓜](G03-2DGS-anchored/小南瓜/G03-2DGS-anchored.png) · [人偶](G03-2DGS-anchored/坐着人偶/G03-2DGS-anchored.png) · [棱锥大面](G03-2DGS-anchored/三棱锥_大面坐地/G03-2DGS-anchored.png) · [棱锥小面](G03-2DGS-anchored/三棱锥_小面坐地/G03-2DGS-anchored.png)

## 源码与复跑

[Poisson 和精确封口](源码/plate_closure.py) · [露板约束](源码/deck.py) · [二维高斯拟合](源码/splatting.py) · [高斯盘 CPU 渲染](源码/splat.cpp) · [高斯 TSDF](源码/complete_splats.py)。`源码` 中这些文件链接到项目现有实现。

从项目根目录运行。三个入口都接收 `--object` 和 `--output`，输出至指定目录。S09 直接读取本目录保存的各角站露板坐标。G03 复用 [Simulation E201](../../../simulation/E201/README.md) 中已拟合的盘数据和 G01 网格。如需重新拟合高斯盘，可用原实验入口的 `--phase gaussian`。

```sh
base='algorithm/best method/03-E201曲面重建'
algorithm/.venv/bin/python "$base/S08-PrecisePoisson/S08-PrecisePoisson.py" --object 小南瓜 --output /tmp/S08-pumpkin
algorithm/.venv/bin/python "$base/S09-ExposedPlate/S09-ExposedPlate.py" --object 三棱锥_小面坐地 --output /tmp/S09-pyramid
algorithm/.venv/bin/python "$base/G03-2DGS-anchored/G03-2DGS-anchored.py" --object 小南瓜 --output /tmp/G03-pumpkin
```
