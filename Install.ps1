param([string]$GamePath, [switch]$NoPause)
$ErrorActionPreference = 'Stop'
$source = Join-Path $PSScriptRoot 'Payload'
. (Join-Path $PSScriptRoot 'ModList.ps1')
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
if (Get-Process -Name AceCombat8 -ErrorAction SilentlyContinue) { throw '请先关闭 ACE COMBAT 8。' }
$GamePath = (Get-Item -LiteralPath $GamePath).FullName
Write-Host "安装到：$GamePath"

$bin = Join-Path $GamePath 'Game\Binaries\Win64'
$ue4ss = Join-Path $bin 'UE4SS'
$mods = Join-Path $ue4ss 'Mods'
$mod = Join-Path $mods 'AC8MouseAim'
$core = Join-Path $ue4ss 'UE4SS.dll'
$coreHash = '680A026890ABB4D0DF2211251F8DEFC1681A584275F1521DCC0FE30AF480006F'
$sourceCore = Join-Path $source 'Game\Binaries\Win64\UE4SS\UE4SS.dll'
if ((Get-FileHash -LiteralPath $sourceCore).Hash -ne $coreHash) { throw '安装包内的 UE4SS 与本 MOD 不匹配。' }
# Complete preflight before any writes. Custom layouts remain manual, not overwritten.
foreach ($path in @($bin,$ue4ss,$mods,$mod,(Join-Path $GamePath 'EasyAntiCheat'),(Join-Path $GamePath 'AC8MouseAim-Backups'))) {
    Assert-NoReparsePath $path
}
if (Test-Path -LiteralPath $mod) {
    if (Get-ChildItem -LiteralPath $mod -Recurse -Force | Where-Object { $_.Attributes -band [IO.FileAttributes]::ReparsePoint }) {
        throw '现有 MOD 目录中包含链接文件，请手动安装。'
    }
}
$reuse = Test-Path -LiteralPath $core -PathType Leaf
if ($reuse) {
    if ((Get-FileHash -LiteralPath $core).Hash -ne $coreHash) {
        throw '已有的 UE4SS 不是本 MOD 验证过的版本，没有替换任何文件。请使用兼容的 MOD 版本，或另行升级框架。'
    }
    if (Test-Path -LiteralPath (Join-Path $bin 'UE4SS.dll')) {
        throw '发现多套 UE4SS 运行库布局，请手动选择正在使用的框架；没有替换任何文件。'
    }
    $overridePath = Join-Path $bin 'override.txt'
    if ((Test-Path -LiteralPath $overridePath) -and [IO.File]::ReadAllText($overridePath).Trim() -ne 'UE4SS') {
        throw 'override.txt 中有自定义加载器重定向，请手动整合；没有替换任何文件。'
    }
    $settingsPath = Join-Path $ue4ss 'UE4SS-settings.ini'
    if (Test-Path -LiteralPath $settingsPath) {
        $settings = [IO.File]::ReadAllText($settingsPath)
        if ($settings -match '(?im)^\s*[+\-]?(ModsFolderPath|ModsFolderPaths|ControllingModsTxt)\s*=[\t ]*[^\s;\r\n]') {
            throw '检测到自定义 MOD 搜索路径，请手动安装；已有设置未改动。'
        }
        if ($settings -match '(?im)^\s*HookEngineTick\s*=[\t ]*(0|false)[\t ]*(?:;.*)?\r?$') {
            throw '本 MOD 需要 HookEngineTick，已有设置未改动。'
        }
    }
} else {
    foreach ($relative in @('UE4SS','UE4SS.dll','dwmapi.dll','dwmapi.dll.disabled','override.txt','winhttp.dll','version.dll','dinput8.dll','xinput1_3.dll','dxgi.dll')) {
        if (Test-Path -LiteralPath (Join-Path $bin $relative)) {
            throw "发现无法识别的框架/加载器（$relative），没有覆盖，需要手动整合。"
        }
    }
}
$updates = Get-ModListUpdates $mods $true
$modSource = Join-Path $source 'Game\Binaries\Win64\UE4SS\Mods\AC8MouseAim'
$configPath = Join-Path $mod 'config.ini'
$rootFiles = @('Launch-AC8-Mouse-Aim.cmd','MouseFlight-Mode.cmd','MouseFlight-Mode.ps1','Uninstall-AC8-Mouse-Aim.cmd',
    'Uninstall-AC8-Mouse-Aim.ps1','AC8MouseAim-Common.ps1','AC8MouseAim-ModList.ps1')
$retired = @('Disable-Mod-For-Multiplayer.cmd')  # replaced by MouseFlight-Mode.cmd
function Get-RootSource([string]$Name) {
    if ($Name -eq 'AC8MouseAim-ModList.ps1') { return (Join-Path $PSScriptRoot 'ModList.ps1') }
    return (Join-Path $source $Name)
}
foreach ($name in $rootFiles) {
    if (-not (Test-Path -LiteralPath (Get-RootSource $name) -PathType Leaf)) { throw "安装包不完整：$name" }
}
foreach ($path in @($updates.Keys) + @((Join-Path $GamePath 'EasyAntiCheat\AC8MouseAim_Offline.json')) +
                  @(($rootFiles + $retired) | ForEach-Object { Join-Path $GamePath $_ })) {
    Assert-NoReparsePath $path
}

# Settings the player chose in [control] survive an upgrade; version adaptation
# keys (input slots, axis signs) and diagnostics come from the new package.
$keptSettings = [ordered]@{}
if (Test-Path -LiteralPath $configPath) {
    $section = ''
    foreach ($line in Get-Content -LiteralPath $configPath) {
        if ($line -match '^\s*\[(.+)\]\s*$') { $section = $Matches[1].Trim(); continue }
        if ($section -eq 'control' -and $line -match '^\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.*?)\s*$') {
            if ($Matches[1] -notin @('pitch_slot', 'roll_slot', 'pitch_sign', 'roll_sign', 'yaw_sign', 'input_probe')) {
                $keptSettings[$Matches[1]] = $Matches[2]
            }
        }
    }
}

$backup = Join-Path $GamePath ('AC8MouseAim-Backups\install-' + (Get-Date -Format 'yyyyMMdd-HHmmss') + '-' + [guid]::NewGuid().ToString('N').Substring(0,8))
New-Item -ItemType Directory -Path $backup -Force | Out-Null
if (Test-Path -LiteralPath $mod) { Copy-Item -LiteralPath $mod -Destination (Join-Path $backup 'AC8MouseAim') -Recurse }
foreach ($path in $updates.Keys) {
    if (Test-Path -LiteralPath $path) { Copy-Item -LiteralPath $path -Destination (Join-Path $backup (Split-Path -Leaf $path)) }
}
foreach ($name in $rootFiles + $retired) {
    $path = Join-Path $GamePath $name
    if (Test-Path -LiteralPath $path) { Copy-Item -LiteralPath $path -Destination $backup }
}
$offline = Join-Path $GamePath 'EasyAntiCheat\AC8MouseAim_Offline.json'
if (Test-Path -LiteralPath $offline) { Copy-Item -LiteralPath $offline -Destination $backup }
if (-not $reuse) {
    Copy-Item -Path (Join-Path $source 'Game\*') -Destination (Join-Path $GamePath 'Game') -Recurse -Force
} else {
    New-Item -ItemType Directory -Path $mod -Force | Out-Null
    foreach ($item in Get-ChildItem -LiteralPath $modSource -Force) {
        Copy-Item -LiteralPath $item.FullName -Destination $mod -Recurse -Force
    }
}
Write-ModListUpdates $updates
Copy-Item -LiteralPath (Join-Path $source 'EasyAntiCheat\AC8MouseAim_Offline.json') -Destination $offline -Force
foreach ($name in $rootFiles) { Copy-Item -LiteralPath (Get-RootSource $name) -Destination (Join-Path $GamePath $name) -Force }
foreach ($name in $retired) {
    $path = Join-Path $GamePath $name
    if (Test-Path -LiteralPath $path) { Remove-Item -LiteralPath $path -Force }
}
# Verify the actual files, not just whether Copy-Item returned successfully.
foreach ($file in Get-ChildItem -LiteralPath $modSource -Recurse -File) {
    $relative = $file.FullName.Substring($modSource.Length).TrimStart('\')
    if ((Get-FileHash -LiteralPath $file.FullName).Hash -ne (Get-FileHash -LiteralPath (Join-Path $mod $relative)).Hash) { throw "MOD 文件校验失败：$relative" }
}
if ((Get-FileHash -LiteralPath $core).Hash -ne $coreHash) { throw '运行库校验失败。' }

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
        Write-Host "已保留你之前修改过的 $restored 项设置。"
    }
}

Write-Host ''
Write-Host '安装完成，文件校验通过。' -ForegroundColor Green
if ($reuse) { Write-Host '已复用兼容的 UE4SS：加载器、框架设置和其他 MOD 均未改动。' }
else { Write-Host '已安装随包的 UE4SS 框架。' }
Write-Host "旧文件备份在：$backup"
Write-Host '游戏目录中的 MouseFlight-Mode.cmd 可在「离线 MOD」和「联机原版」之间切换。'
Write-Host '安装层面的共存不代表与其他输入、相机类 MOD 功能上不冲突。仅限离线单人使用。'
if (Test-Path -LiteralPath (Join-Path $bin 'dwmapi.dll.disabled')) {
    Write-Host '当前处于联机原版模式（加载器已停用），要使用 MOD 请运行 MouseFlight-Mode.cmd 切回。' -ForegroundColor Yellow
}
if (-not $NoPause) {
    if ((Get-AC8LaunchOptionState) -eq 'set') {
        Write-Host 'Steam 启动参数已经设置好，直接从 Steam 启动游戏即可。' -ForegroundColor Green
    } else { Show-AC8LaunchOptionHelp }
}
Finish 0
