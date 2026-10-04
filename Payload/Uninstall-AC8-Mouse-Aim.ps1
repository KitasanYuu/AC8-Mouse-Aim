param([switch]$NoPause)
$ErrorActionPreference = 'Stop'
$game = $PSScriptRoot
. (Join-Path $game 'AC8MouseAim-ModList.ps1')
if (-not (Test-Path -LiteralPath (Join-Path $game 'Game\Binaries\Win64\AceCombat8.exe'))) { throw '这个脚本不在 ACE COMBAT 8 根目录中。' }
if (Get-Process -Name AceCombat8 -ErrorAction SilentlyContinue) { throw '请先关闭 ACE COMBAT 8。' }
if (-not $NoPause -and (Read-Host '确定卸载 AC8 MouseFlight？(Y/N)') -notmatch '^[Yy]') { exit 0 }
# Checked before the helper script itself is moved away.
$launch = 'unknown'
$common = Join-Path $game 'AC8MouseAim-Common.ps1'
if (Test-Path -LiteralPath $common) { . $common; $launch = Get-AC8LaunchOptionState }
$mods = Join-Path $game 'Game\Binaries\Win64\UE4SS\Mods'
$mod = Join-Path $mods 'AC8MouseAim'
$updates = Get-ModListUpdates $mods $false
$backup = Join-Path $game ('AC8MouseAim-Backups\uninstall-' + (Get-Date -Format 'yyyyMMdd-HHmmss') + '-' + [guid]::NewGuid().ToString('N').Substring(0,8))
$ownFiles = @('EasyAntiCheat\AC8MouseAim_Offline.json','Launch-AC8-Mouse-Aim.cmd','Disable-Mod-For-Multiplayer.cmd',
    'MouseFlight-Mode.cmd','MouseFlight-Mode.ps1','AC8MouseAim-Common.ps1','Uninstall-AC8-Mouse-Aim.cmd',
    'Uninstall-AC8-Mouse-Aim.ps1','AC8MouseAim-ModList.ps1')
$targets = @($mod) + @($ownFiles | ForEach-Object { Join-Path $game $_ })
foreach ($path in $targets + @($updates.Keys) + @($backup)) {
    $resolved = [IO.Path]::GetFullPath($path)
    if (-not $resolved.StartsWith([IO.Path]::GetFullPath($game).TrimEnd('\')+'\',[StringComparison]::OrdinalIgnoreCase)) { throw '卸载路径超出了游戏目录。' }
    Assert-NoReparsePath $resolved
}
New-Item -ItemType Directory -Path $backup -Force | Out-Null
foreach ($path in $updates.Keys) {
    if (Test-Path -LiteralPath $path) { Copy-Item -LiteralPath $path -Destination (Join-Path $backup (Split-Path -Leaf $path)) }
}
Write-ModListUpdates $updates
foreach ($path in $targets) {
    if (Test-Path -LiteralPath $path) { Move-Item -LiteralPath $path -Destination (Join-Path $backup (Split-Path -Leaf $path)) }
}
Write-Host 'AC8 MouseFlight 已卸载。UE4SS、共用加载器、框架设置和其他 MOD 均已保留。' -ForegroundColor Green
Write-Host "移走的文件可从这里恢复：$backup"
if ($launch -ne 'missing') {
    Write-Host '最后一步：Steam 库 → 右键 ACE COMBAT 8 → 属性 → 通用 → 启动选项，清空本 MOD 的离线启动参数。' -ForegroundColor Yellow
}
Write-Host '其他 MOD 仍可能被加载，这不等于干净的联机环境。'
if (-not $NoPause) { Read-Host '按回车键关闭' | Out-Null }
