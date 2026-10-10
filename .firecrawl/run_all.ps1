param([string]$Mode = "search", [string]$Filter = "", [string]$Tag = "x", [int]$Delay = 6)
$file = if ($Mode -eq "search") { ".firecrawl\queries.json" } else { ".firecrawl\scrapes.json" }
$all = Get-Content -Raw -Encoding UTF8 $file | ConvertFrom-Json
foreach ($item in $all) {
  if ($Filter -ne "" -and $item.n -notlike "$Filter*") { continue }
  Write-Output "=== $($item.n) ==="
  $ok = $false
  foreach ($attempt in 1..3) {
    if ($Mode -eq "search") {
      $out = ".firecrawl\$($item.n).json"
      $res = firecrawl search $item.q -o $out --json 2>&1 | Out-String
    } else {
      $out = ".firecrawl\$($item.n).md"
      $res = firecrawl scrape $item.u -o $out 2>&1 | Out-String
    }
    if ($res -notmatch "Rate limit exceeded") { $ok = $true; break }
    Write-Output "  retry $attempt after rate limit"
    Start-Sleep -Seconds 25
  }
  if (-not $ok) { Write-Output "  FAILED after retries" }
  Start-Sleep -Seconds $Delay
}
Write-Output "DONE_$Tag"
