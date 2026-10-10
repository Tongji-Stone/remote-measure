param([string]$Filter = "", [string]$Tag = "x")
$all = Get-Content -Raw -Encoding UTF8 ".firecrawl\queries.json" | ConvertFrom-Json
foreach ($item in $all) {
  if ($Filter -ne "" -and $item.n -notlike "$Filter*") { continue }
  $out = ".firecrawl\$($item.n).json"
  Write-Output "=== $($item.n) ==="
  firecrawl search $item.q -o $out --json 2>&1 | Out-String -Width 200 | Select-Object -First 1
}
Write-Output "DONE_$Tag"
