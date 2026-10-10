$queries = @(
  @{ n = "a01_open3d_volume"; q = "Open3D compute point cloud volume convex hull alpha shape Python example" },
  @{ n = "a02_tsdf"; q = "Open3D ScalableTSDFVolume integrate extract triangle mesh volume" },
  @{ n = "a03_poisson"; q = "Open3D create_from_point_cloud_poisson depth parameter hole filling artifacts" },
  @{ n = "a04_pcl_filter"; q = "PCL statistical outlier removal radius outlier removal voxel grid filter functions" },
  @{ n = "a05_registration"; q = "Open3D ICP registration point-to-plane colored ICP turntable multiway registration" },
  @{ n = "a06_alpha_shape"; q = "alpha shape 3D volume computation Python implementation concave hull" },
  @{ n = "a07_trimesh"; q = "trimesh mesh volume divergence theorem watertight fill_holes Python" },
  @{ n = "a08_cgal"; q = "CGAL 3D alpha shape volume Poisson surface reconstruction documentation" },
  @{ n = "a09_bpa"; q = "ball pivoting algorithm Delaunay triangulation greedy triangulation point cloud surface reconstruction" },
  @{ n = "a10_slice"; q = "volume estimation cross section slice integration point cloud layer area summation" }
)
foreach ($item in $queries) {
  $out = ".firecrawl/$($item.n).json"
  Write-Output "=== $($item.n) ==="
  firecrawl search $item.q -o $out --json 2>&1 | Select-Object -First 3
}
Write-Output "DONE_ALGO"
