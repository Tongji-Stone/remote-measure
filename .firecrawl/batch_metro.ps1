$queries = @(
  @{ n = "b01_vol_uncert"; q = "uncertainty budget structured light 3D scanner volume measurement relative error percent" },
  @{ n = "b02_calib_prop"; q = "camera calibration error propagation to 3D reconstruction volume measurement uncertainty" },
  @{ n = "b03_zhang"; q = "Zhang Zhengyou camera calibration reprojection error checkerboard accuracy OpenCV" },
  @{ n = "b04_sphere_cal"; q = "calibration sphere standard artifact scale correction 3D scanner volumetric accuracy" },
  @{ n = "b05_drain"; q = "排水法 测量 体积 精度 误差 标准球 计量" },
  @{ n = "b06_competition"; q = "全国大学生光电设计竞赛 体积测量 赛题二 结构光 方案 报告" },
  @{ n = "b07_structured_precision"; q = "fringe projection profilometry depth accuracy micrometer phase error calibration" },
  @{ n = "b08_hole_scan"; q = "3D scanning blind hole through hole measurement limitation occluded features" }
)
foreach ($item in $queries) {
  $out = ".firecrawl/$($item.n).json"
  Write-Output "=== $($item.n) ==="
  firecrawl search $item.q -o $out --json 2>&1 | Select-Object -First 3
}
Write-Output "DONE_METRO"
