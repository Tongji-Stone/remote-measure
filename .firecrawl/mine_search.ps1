$ErrorActionPreference = "Continue"
$raw = Get-Content -Raw -Encoding UTF8 ".firecrawl/mine-queries.json"
$obj = $raw | ConvertFrom-Json
foreach ($p in $obj.PSObject.Properties) {
  $k = $p.Name
  $q = $p.Value
  $out = ".firecrawl/$k.json"
  if ((Test-Path $out) -and ((Get-Item $out).Length -gt 300)) { Write-Output "SKIP $k"; continue }
  $tries = 0
  $ok = $false
  while ($tries -lt 5) {
    firecrawl search $q -o $out --json 2>&1 | Out-Null
    if ((Test-Path $out) -and ((Get-Item $out).Length -gt 300)) { Write-Output "OK $k $((Get-Item $out).Length)"; $ok = $true; break }
    $tries++
    Start-Sleep -Seconds 16
  }
  if (-not $ok) { Write-Output "FAIL $k" }
  Start-Sleep -Seconds 12
}
Write-Output "ALLDONE"
