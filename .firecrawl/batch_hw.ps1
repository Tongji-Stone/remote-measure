$queries = @(
  @{ n = "c01_ov9281"; q = "OV9281 双目 相机模组 价格 淘宝 全局快门" },
  @{ n = "c02_imx"; q = "IMX477 IMX219 树莓派 相机模组 价格 京东" },
  @{ n = "c03_dlp"; q = "DLP2000 DLP2010 投影 光机 模组 价格 淘宝 结构光" },
  @{ n = "c04_laser"; q = "650nm 线激光模组 一字线 激光器 价格 淘宝" },
  @{ n = "c05_filter"; q = "窄带滤光片 850nm 808nm 价格 淘宝 光学" },
  @{ n = "c06_stepper"; q = "42步进电机 驱动器 价格 淘宝 57步进电机" },
  @{ n = "c07_turntable"; q = "电动旋转台 精密转台 价格 淘宝 步进电机 转台" },
  @{ n = "c08_sbc"; q = "树莓派5 价格 Jetson Orin Nano 价格 京东 2024" },
  @{ n = "c09_indcam"; q = "海康机器人 工业相机 MV-CS 价格 大恒图像 工业相机 报价" },
  @{ n = "c10_openstruct"; q = "低成本 结构光 三维重建 开源 光栅投影 格雷码 相移法" }
)
foreach ($item in $queries) {
  $out = ".firecrawl/$($item.n).json"
  Write-Output "=== $($item.n) ==="
  firecrawl search $item.q -o $out --json 2>&1 | Select-Object -First 3
}
Write-Output "DONE_HW"
