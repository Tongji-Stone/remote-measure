# 启动 Tavily -> Anthropic 翻译桥（DSH 网页搜索用）
#
# 用法：
#   pwsh -File tools\start-shim.ps1            # 前台启动，Ctrl+C 停止
#   pwsh -File tools\start-shim.ps1 -Detached  # 后台启动，日志写到 tools\shim.log
#
# 启动后，在 DSH 的 侧栏 -> 插件 -> 官方分组 -> 网页搜索 里，
# 把「接口地址」改成 http://127.0.0.1:8787/v1 并保存。API Key 不用动。
#
# 为什么需要它：DSH 的搜索插件只会说 DeepSeek 的 Anthropic 兼容协议，
# 而你手上的是 Tavily 的钥匙。这个小服务负责两边翻译。

param(
  [switch]$Detached,
  [int]$Port = 8787
)

$ErrorActionPreference = 'Stop'

$node = Join-Path $env:LOCALAPPDATA 'pi-node\current\node.exe'
if (-not (Test-Path $node)) {
  $cmd = Get-Command node -ErrorAction SilentlyContinue
  if ($cmd) { $node = $cmd.Source } else { throw "找不到 node.exe（找过 $node 和 PATH）" }
}

$shim = Join-Path $PSScriptRoot 'tavily-anthropic-shim.mjs'
if (-not (Test-Path $shim)) { throw "找不到 $shim" }

$env:SHIM_PORT = "$Port"

if (-not $Detached) {
  Write-Host "启动翻译桥: http://127.0.0.1:$Port/v1  (Ctrl+C 停止)"
  & $node $shim
  exit $LASTEXITCODE
}

# ---- 后台模式 ----
$log = Join-Path $PSScriptRoot 'shim.log'

# 端口已被占用就别重复起
$inUse = $false
try {
  $c = New-Object System.Net.Sockets.TcpClient
  $c.Connect('127.0.0.1', $Port)
  $inUse = $true
  $c.Close()
} catch { }

if ($inUse) {
  try {
    $h = Invoke-RestMethod -Uri "http://127.0.0.1:$Port/health" -TimeoutSec 5
    Write-Host "翻译桥已经在跑了（$($h.service)，端口 $Port），不用重复启动。"
    exit 0
  } catch {
    throw "端口 $Port 被别的程序占用了，换个端口：-Port 8788"
  }
}

Start-Process -FilePath $node -ArgumentList @($shim) -WindowStyle Hidden `
  -RedirectStandardOutput $log -RedirectStandardError "$log.err"

# 等它就绪
for ($i = 0; $i -lt 20; $i++) {
  Start-Sleep -Milliseconds 300
  try {
    $h = Invoke-RestMethod -Uri "http://127.0.0.1:$Port/health" -TimeoutSec 3
    Write-Host "翻译桥已就绪: http://127.0.0.1:$Port/v1"
    Write-Host "日志: $log"
    Write-Host ""
    Write-Host "下一步（只需做一次）:"
    Write-Host "  DSH 侧栏 -> 插件 -> 官方分组 -> 网页搜索 -> 接口地址填 http://127.0.0.1:$Port/v1 -> 保存"
    exit 0
  } catch { }
}

Write-Warning "翻译桥没能在 6 秒内就绪，看看日志：$log"
exit 1
