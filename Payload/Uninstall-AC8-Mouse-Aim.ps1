$ErrorActionPreference = 'Stop'
$game = Split-Path -Parent $MyInvocation.MyCommand.Path
$exe = Join-Path $game 'Game\Binaries\Win64\AceCombat8.exe'
if (-not (Test-Path -LiteralPath $exe)) { throw '这个脚本不在 ACE COMBAT 8 根目录中。' }
if (Get-Process -Name 'AceCombat8' -ErrorAction SilentlyContinue) { throw '请先关闭 ACE COMBAT 8。' }
if ((Read-Host '确定卸载 AC8 MouseFlight？(Y/N)') -notmatch '^[Yy]') { exit 0 }

# Checked before the helper script itself is removed.
$launch = 'unknown'
$common = Join-Path $game 'AC8MouseAim-Common.ps1'
if (Test-Path -LiteralPath $common) { . $common; $launch = Get-AC8LaunchOptionState }

$targets = @(
    'Game\Binaries\Win64\dwmapi.dll',
    'Game\Binaries\Win64\dwmapi.dll.disabled',
    'Game\Binaries\Win64\override.txt',
    'Game\Binaries\Win64\UE4SS\Mods\AC8MouseAim',
    'EasyAntiCheat\AC8MouseAim_Offline.json',
    'Launch-AC8-Mouse-Aim.cmd',
    'Disable-Mod-For-Multiplayer.cmd',
    'MouseFlight-Mode.cmd',
    'MouseFlight-Mode.ps1',
    'AC8MouseAim-Common.ps1',
    'Uninstall-AC8-Mouse-Aim.cmd',
    'Uninstall-AC8-Mouse-Aim.ps1'
)
foreach ($relative in $targets) {
    $target = Join-Path $game $relative
    if (Test-Path -LiteralPath $target) { Remove-Item -LiteralPath $target -Recurse -Force }
}
Write-Host 'AC8 MouseFlight 及其加载器入口已移除。' -ForegroundColor Green
Write-Host '不再被加载的 UE4SS 框架目录已保留，以免影响其他离线 MOD。'
if ($launch -ne 'missing') {
    Write-Host '最后一步：Steam 库 → 右键 ACE COMBAT 8 → 属性 → 通用 → 启动选项，清空本 MOD 的离线启动参数。' -ForegroundColor Yellow
}
Read-Host '按回车键关闭' | Out-Null
