param([string]$Tag = "x")
$all = Get-Content -Raw -Encoding UTF8 ".firecrawl\scrapes.json" | ConvertFrom-Json
foreach ($item in $all) {
  $out = ".firecrawl\$($item.n).md"
  Write-Output "=== $($item.n) ==="
  firecrawl scrape $item.u -o $out 2>&1 | Out-String -Width 200 | Select-Object -First 1
}
Write-Output "DONE_$Tag"
