# Switches between offline MOD mode and original (multiplayer) mode.
$ErrorActionPreference = 'Stop'
$game = Split-Path -Parent $MyInvocation.MyCommand.Path
. (Join-Path $game 'AC8MouseAim-Common.ps1')
if (-not (Test-AC8GameDirectory $game)) { throw '这个工具不在 ACE COMBAT 8 根目录中。' }
if (Get-Process -Name 'AceCombat8' -ErrorAction SilentlyContinue) { throw '请先关闭 ACE COMBAT 8。' }
$loader = Join-Path $game 'Game\Binaries\Win64\dwmapi.dll'
$disabled = "$loader.disabled"
$launch = Get-AC8LaunchOptionState

if (Test-Path -LiteralPath $loader) {
    Write-Host '当前：离线 MOD 模式（MouseFlight 已启用）' -ForegroundColor Green
    if ((Read-Host '切换到联机原版模式？(Y/N)') -notmatch '^[Yy]') { return }
    Move-Item -LiteralPath $loader -Destination $disabled -Force
    Write-Host ''
    Write-Host '已切换到联机原版模式：MOD 加载器已停用。' -ForegroundColor Green
    if ($launch -ne 'missing') {
        Write-Host '联机前还需要：Steam 库 → 右键 ACE COMBAT 8 → 属性 → 通用 → 启动选项，清空本 MOD 的离线启动参数。' -ForegroundColor Yellow
    }
} elseif (Test-Path -LiteralPath $disabled) {
    Write-Host '当前：联机原版模式（MouseFlight 已停用）' -ForegroundColor Green
    if ((Read-Host '切换到离线 MOD 模式？(Y/N)') -notmatch '^[Yy]') { return }
    Move-Item -LiteralPath $disabled -Destination $loader -Force
    Write-Host ''
    Write-Host '已切换到离线 MOD 模式：MOD 加载器已启用。' -ForegroundColor Green
    if ($launch -eq 'set') { Write-Host 'Steam 启动参数已经设置好，直接从 Steam 启动游戏即可。' }
    else { Show-AC8LaunchOptionHelp }
} else {
    throw '没有找到 MOD 加载器，请重新运行安装包中的 Install.cmd。'
}
