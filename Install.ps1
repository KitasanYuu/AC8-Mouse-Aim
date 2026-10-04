param(
    [string]$GamePath,
    [switch]$NoPause
)
$ErrorActionPreference = 'Stop'
Import-Module Microsoft.PowerShell.Utility -ErrorAction Stop
$source = Join-Path $PSScriptRoot 'Payload'
. (Join-Path $source 'AC8MouseAim-Common.ps1')
function Finish([int]$Code) {
    if (-not $NoPause) { Read-Host '按回车键关闭' | Out-Null }
    exit $Code
}

# Game folder: from Steam's library records; ask only when it cannot be found.
if ([string]::IsNullOrWhiteSpace($GamePath)) {
    $GamePath = Find-AC8GamePath
    if ($GamePath) {
        Write-Host "已通过 Steam 找到游戏：$GamePath"
    } else {
        Write-Host '没有自动找到游戏，请手动选择 ACE COMBAT 8 的根目录（包含 Game 和 EasyAntiCheat）。'
        Add-Type -AssemblyName System.Windows.Forms
        $picker = New-Object System.Windows.Forms.FolderBrowserDialog
        $picker.Description = '选择 ACE COMBAT 8 根目录（包含 Game 和 EasyAntiCheat）'
        $picker.ShowNewFolderButton = $false
        try {
            while ($true) {
                if ($picker.ShowDialog() -ne [System.Windows.Forms.DialogResult]::OK) {
                    Write-Host '已取消安装，没有修改任何游戏文件。'
                    Finish 0
                }
                if (Test-AC8GameDirectory $picker.SelectedPath) { $GamePath = $picker.SelectedPath; break }
                [void][System.Windows.Forms.MessageBox]::Show(
                    '这不是游戏根目录。请选择 ACE COMBAT 8 文件夹本身，而不是 Game 或 Win64 子文件夹。',
                    '未找到游戏', 'OK', 'Warning')
            }
        } finally { $picker.Dispose() }
    }
}
if (-not (Test-AC8GameDirectory $GamePath)) { throw "不是有效的 ACE COMBAT 8 目录：$GamePath" }
$GamePath = (Get-Item -LiteralPath $GamePath).FullName
if (Get-Process -Name 'AceCombat8' -ErrorAction SilentlyContinue) { throw '请先关闭 ACE COMBAT 8。' }
Write-Host "安装到：$GamePath"

$loader = Join-Path $GamePath 'Game\Binaries\Win64\dwmapi.dll'
$disabledLoader = "$loader.disabled"
$ue4ss = Join-Path $GamePath 'Game\Binaries\Win64\UE4SS'
$mouseAim = Join-Path $ue4ss 'Mods\AC8MouseAim'
$isUpgrade = Test-Path -LiteralPath $mouseAim
$hasLoader = (Test-Path -LiteralPath $loader) -or (Test-Path -LiteralPath $disabledLoader) -or
    (Test-Path -LiteralPath $ue4ss)
if ($hasLoader -and -not $isUpgrade) {
    throw '发现其他来源的 UE4SS/dwmapi 安装，为避免冲突没有覆盖。'
}

# Settings the player chose in [control] survive an upgrade; version adaptation
# keys (input slots, axis signs) and diagnostics come from the new package.
$configPath = Join-Path $mouseAim 'config.ini'
$keptSettings = [ordered]@{}
if ($isUpgrade -and (Test-Path -LiteralPath $configPath)) {
    $section = ''
    foreach ($line in Get-Content -LiteralPath $configPath) {
        if ($line -match '^\s*\[(.+)\]\s*$') { $section = $Matches[1].Trim(); continue }
        if ($section -eq 'control' -and $line -match '^\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.*?)\s*$') {
            if ($Matches[1] -notin @('pitch_slot', 'roll_slot', 'pitch_sign', 'roll_sign', 'yaw_sign', 'input_probe')) {
                $keptSettings[$Matches[1]] = $Matches[2]
            }
        }
    }
    Copy-Item -LiteralPath $configPath -Destination "$configPath.bak" -Force
}

if ($isUpgrade) {
    $knownLoaderHashes = @(
        'C5D2AB9F9B89BD94460B0A283EEFB113085105014011CAC961F36787376DB744',
        'CF440B9EB8643BB7C434ACFDA696AEE57FD981D185DCA5E57FB8DBB18F8FC1CD'
    )
    $installedLoader = if (Test-Path -LiteralPath $loader) { $loader } else { $disabledLoader }
    if (-not (Test-Path -LiteralPath $installedLoader)) {
        throw '现有的 AC8MouseAim 安装缺少可识别的加载器，无法修复。'
    }
    $installedHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $installedLoader).Hash
    if ($installedHash -notin $knownLoaderHashes) {
        throw '现有加载器已被修改或属于其他安装包，没有覆盖。'
    }
    $backup = Join-Path $GamePath 'AC8MouseAim-Loader-Backup-0.1.0'
    if (-not (Test-Path -LiteralPath $backup)) {
        New-Item -ItemType Directory -Path $backup | Out-Null
        Copy-Item -LiteralPath $installedLoader -Destination (Join-Path $backup 'dwmapi.dll')
        $oldCore = Join-Path $ue4ss 'UE4SS.dll'
        if (Test-Path -LiteralPath $oldCore) { Copy-Item -LiteralPath $oldCore -Destination (Join-Path $backup 'UE4SS.dll') }
    }
    if (Test-Path -LiteralPath $disabledLoader) { Remove-Item -LiteralPath $disabledLoader -Force }
}

Copy-Item -Path (Join-Path $source 'Game\*') -Destination (Join-Path $GamePath 'Game') -Recurse -Force
Copy-Item -LiteralPath (Join-Path $source 'EasyAntiCheat\AC8MouseAim_Offline.json') `
    -Destination (Join-Path $GamePath 'EasyAntiCheat\AC8MouseAim_Offline.json')
$tools = @('Launch-AC8-Mouse-Aim.cmd', 'MouseFlight-Mode.cmd', 'MouseFlight-Mode.ps1',
    'Uninstall-AC8-Mouse-Aim.cmd', 'Uninstall-AC8-Mouse-Aim.ps1', 'AC8MouseAim-Common.ps1')
foreach ($name in $tools) {
    Copy-Item -LiteralPath (Join-Path $source $name) -Destination (Join-Path $GamePath $name)
}
# Replaced by MouseFlight-Mode.cmd.
$oldDisable = Join-Path $GamePath 'Disable-Mod-For-Multiplayer.cmd'
if (Test-Path -LiteralPath $oldDisable) { Remove-Item -LiteralPath $oldDisable -Force }

# Verify the actual files, not just whether Copy-Item returned successfully.
foreach ($relative in @(
    'Game\Binaries\Win64\UE4SS\UE4SS.dll',
    'Game\Binaries\Win64\UE4SS\Mods\AC8MouseAim\Scripts\ac8_mouse_aim_010.dll',
    'Game\Binaries\Win64\UE4SS\Mods\AC8MouseAim\Scripts\main.lua',
    'Game\Binaries\Win64\UE4SS\Mods\AC8MouseAim\Scripts\camera.lua',
    'Game\Binaries\Win64\UE4SS\Mods\AC8MouseAim\Scripts\gaze_probe.lua',
    'Game\Binaries\Win64\UE4SS\Mods\AC8MouseAim\Scripts\gaze.lua',
    'Game\Binaries\Win64\UE4SS\Mods\AC8MouseAim\Scripts\rig_math.lua',
    'Game\Binaries\Win64\UE4SS\Mods\AC8MouseAim\config.ini'
)) {
    $expected = (Get-FileHash -LiteralPath (Join-Path $source $relative) -Algorithm SHA256).Hash
    $actual = (Get-FileHash -LiteralPath (Join-Path $GamePath $relative) -Algorithm SHA256).Hash
    if ($actual -ne $expected) { throw "安装校验失败：$relative" }
}

if ($keptSettings.Count -gt 0) {
    $section = ''; $restored = 0
    $lines = foreach ($line in Get-Content -LiteralPath $configPath) {
        if ($line -match '^\s*\[(.+)\]\s*$') { $section = $Matches[1].Trim(); $line; continue }
        if ($section -eq 'control' -and $line -match '^(\s*)([A-Za-z_][A-Za-z0-9_]*)(\s*=\s*)(.*)$' -and
            $keptSettings.Contains($Matches[2]) -and $Matches[4].Trim() -ne $keptSettings[$Matches[2]]) {
            $restored++
            "$($Matches[1])$($Matches[2])$($Matches[3])$($keptSettings[$Matches[2]])"
        } else { $line }
    }
    if ($restored -gt 0) {
        [IO.File]::WriteAllLines($configPath, [string[]]$lines, (New-Object Text.UTF8Encoding($false)))
        Write-Host "已保留你之前修改过的 $restored 项设置（旧配置备份为 config.ini.bak）。"
    }
}

Write-Host ''
Write-Host '安装完成，文件校验通过。' -ForegroundColor Green
if ($isUpgrade) { Write-Host '这是升级安装，加载器已修复，旧文件已备份。' }
Write-Host '游戏目录中的 MouseFlight-Mode.cmd 可在「离线 MOD」和「联机原版」之间切换。'
if (-not $NoPause) {
    switch (Get-AC8LaunchOptionState) {
        'set' { Write-Host 'Steam 启动参数已经设置好，直接从 Steam 启动游戏即可。' -ForegroundColor Green }
        default { Show-AC8LaunchOptionHelp }
    }
}
Finish 0
