# AC8 MouseFlight: install / update, mode switch, launch option help and uninstall.
# Run through AC8MouseFlight.cmd in the repository root (menu), or with -Action for scripts.
# Nothing is left in the game root: backups go to the repository's backups folder.
param(
    [ValidateSet('Menu', 'Install', 'Switch', 'Uninstall', 'CopyLaunch')]
    [string]$Action = 'Menu',
    [string]$GamePath,
    [switch]$NoPause
)
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$source = Join-Path $repo 'Payload'
$backupRoot = Join-Path $repo 'backups'
. (Join-Path $PSScriptRoot 'ModList.ps1')

$AppId = '2288340'
$LaunchOption = 'cmd /d /c "set EOS_USE_ANTICHEATCLIENTNULL=1&& %command% -anticheat_settings=AC8MouseAim_Offline.json"'
$CoreHash = '680A026890ABB4D0DF2211251F8DEFC1681A584275F1521DCC0FE30AF480006F'
$LoaderHashes = @('C5D2AB9F9B89BD94460B0A283EEFB113085105014011CAC961F36787376DB744',
                  'CF440B9EB8643BB7C434ACFDA696AEE57FD981D185DCA5E57FB8DBB18F8FC1CD')
# Helper scripts and backup folders earlier versions left in the game root.
$LegacyRootFiles = @('Launch-AC8-Mouse-Aim.cmd', 'Disable-Mod-For-Multiplayer.cmd', 'MouseFlight-Mode.cmd',
    'MouseFlight-Mode.ps1', 'Uninstall-AC8-Mouse-Aim.cmd', 'Uninstall-AC8-Mouse-Aim.ps1',
    'AC8MouseAim-Common.ps1', 'AC8MouseAim-ModList.ps1')
$LegacyRootDirs = @('AC8MouseAim-Backups', 'AC8MouseAim-Loader-Backup-*', 'AC8MouseAim-Uninstall-Backup-*')

function Say([string]$Text, [string]$Color) {
    if ($Color) { Write-Host $Text -ForegroundColor $Color } else { Write-Host $Text }
}
function Confirm-Choice([string]$Question) {
    if ($NoPause) { return $true }
    return ((Read-Host "$Question (Y/N)") -match '^[Yy]')
}

# ---------- game and Steam ----------
function Test-GameDirectory([string]$Path) {
    if ([string]::IsNullOrWhiteSpace($Path)) { return $false }
    return ((Test-Path -LiteralPath (Join-Path $Path 'Game\Binaries\Win64\AceCombat8.exe') -PathType Leaf) -and
        (Test-Path -LiteralPath (Join-Path $Path 'EasyAntiCheat') -PathType Container))
}
function Get-SteamRoot {
    foreach ($key in @('HKCU:\Software\Valve\Steam', 'HKLM:\SOFTWARE\WOW6432Node\Valve\Steam', 'HKLM:\SOFTWARE\Valve\Steam')) {
        try {
            $item = Get-ItemProperty -LiteralPath $key -ErrorAction Stop
            foreach ($name in @('SteamPath', 'InstallPath')) {
                $value = $item.$name
                if ($value -and (Test-Path -LiteralPath $value -PathType Container)) {
                    return (Get-Item -LiteralPath ($value -replace '/', '\')).FullName
                }
            }
        } catch { }
    }
    return $null
}
# The game folder from Steam's own library records (libraryfolders.vdf + app manifest).
function Find-GamePath {
    $steam = Get-SteamRoot
    if (-not $steam) { return $null }
    $libraries = New-Object System.Collections.Generic.List[string]
    $libraries.Add($steam)
    $vdf = Join-Path $steam 'steamapps\libraryfolders.vdf'
    if (Test-Path -LiteralPath $vdf) {
        foreach ($match in [regex]::Matches((Get-Content -LiteralPath $vdf -Raw), '"path"\s+"((?:[^"\\]|\\.)*)"')) {
            $libraries.Add(($match.Groups[1].Value -replace '\\\\', '\'))
        }
    }
    foreach ($library in $libraries) {
        $manifest = Join-Path $library "steamapps\appmanifest_$AppId.acf"
        if (-not (Test-Path -LiteralPath $manifest)) { continue }
        $dir = [regex]::Match((Get-Content -LiteralPath $manifest -Raw), '"installdir"\s+"([^"]+)"')
        if (-not $dir.Success) { continue }
        $candidate = Join-Path $library ("steamapps\common\" + $dir.Groups[1].Value)
        if (Test-GameDirectory $candidate) { return (Get-Item -LiteralPath $candidate).FullName }
    }
    return $null
}
function Resolve-GamePath {
    if (-not [string]::IsNullOrWhiteSpace($GamePath)) {
        if (-not (Test-GameDirectory $GamePath)) { throw "不是有效的 ACE COMBAT 8 目录：$GamePath" }
        return (Get-Item -LiteralPath $GamePath).FullName
    }
    $found = Find-GamePath
    if ($found) { return $found }
    if ($NoPause) { throw '没有通过 Steam 找到游戏，请用 -GamePath 指定。' }
    Say '没有通过 Steam 自动找到游戏，请手动选择 ACE COMBAT 8 根目录（包含 Game 和 EasyAntiCheat）。'
    Add-Type -AssemblyName System.Windows.Forms
    $picker = New-Object System.Windows.Forms.FolderBrowserDialog
    $picker.Description = '选择 ACE COMBAT 8 根目录（包含 Game 和 EasyAntiCheat）'
    $picker.ShowNewFolderButton = $false
    try {
        while ($true) {
            if ($picker.ShowDialog() -ne [System.Windows.Forms.DialogResult]::OK) { return $null }
            if (Test-GameDirectory $picker.SelectedPath) { return (Get-Item -LiteralPath $picker.SelectedPath).FullName }
            [void][System.Windows.Forms.MessageBox]::Show('这不是游戏根目录。请选择 ACE COMBAT 8 文件夹本身，而不是 Game 或 Win64 子文件夹。',
                '未找到游戏', 'OK', 'Warning')
        }
    } finally { $picker.Dispose() }
}
# 'set' when any Steam user has the offline launch option for the game, 'missing' when
# Steam's settings were readable but none has it, 'unknown' otherwise. Read-only.
function Get-LaunchOptionState {
    $steam = Get-SteamRoot
    if (-not $steam) { return 'unknown' }
    $configs = Get-ChildItem -LiteralPath (Join-Path $steam 'userdata') -Directory -ErrorAction SilentlyContinue |
        ForEach-Object { Join-Path $_.FullName 'config\localconfig.vdf' } |
        Where-Object { Test-Path -LiteralPath $_ }
    if (-not $configs) { return 'unknown' }
    foreach ($config in $configs) {
        $text = Get-Content -LiteralPath $config -Raw -Encoding UTF8
        foreach ($match in [regex]::Matches($text, '"' + $AppId + '"\s*\{')) {
            # The app's block, by brace matching (quoted strings may contain braces).
            $depth = 0; $inString = $false; $end = $text.Length
            for ($i = $match.Index + $match.Length - 1; $i -lt $text.Length; $i++) {
                $c = $text[$i]
                if ($inString) {
                    if ($c -eq '\') { $i++ } elseif ($c -eq '"') { $inString = $false }
                } elseif ($c -eq '"') { $inString = $true }
                elseif ($c -eq '{') { $depth++ }
                elseif ($c -eq '}') { $depth--; if ($depth -eq 0) { $end = $i; break } }
            }
            $block = $text.Substring($match.Index, $end - $match.Index)
            if ($block -match '"LaunchOptions"\s*"((?:[^"\\]|\\.)*)"' -and $Matches[1] -like '*AC8MouseAim_Offline.json*') {
                return 'set'
            }
        }
    }
    return 'missing'
}
function Show-LaunchOptionHelp {
    $copied = $false
    try { Set-Clipboard -Value $LaunchOption; $copied = $true } catch { }
    Say ''
    Say '在 Steam 中设置启动参数：' Yellow
    if ($copied) { Say '  启动参数已复制到剪贴板。' }
    Say '  Steam 库 → 右键 ACE COMBAT 8 → 属性 → 通用 → 启动选项，粘贴：'
    Say "  $LaunchOption" Cyan
    Say '  如果原来已有其他启动选项，请先备份。'
}
function Assert-GameClosed {
    if (Get-Process -Name AceCombat8 -ErrorAction SilentlyContinue) { throw '请先关闭 ACE COMBAT 8。' }
}

# ---------- paths in the game ----------
function Get-Layout([string]$Game) {
    $bin = Join-Path $Game 'Game\Binaries\Win64'
    $ue4ss = Join-Path $bin 'UE4SS'
    $mods = Join-Path $ue4ss 'Mods'
    [pscustomobject]@{
        Game = $Game; Bin = $bin; UE4SS = $ue4ss; Mods = $mods
        Mod = Join-Path $mods 'AC8MouseAim'
        Core = Join-Path $ue4ss 'UE4SS.dll'
        Loader = Join-Path $bin 'dwmapi.dll'
        DisabledLoader = Join-Path $bin 'dwmapi.dll.disabled'
        Offline = Join-Path $Game 'EasyAntiCheat\AC8MouseAim_Offline.json'
    }
}
function Get-Mode($L) {
    $on = Test-Path -LiteralPath $L.Loader; $off = Test-Path -LiteralPath $L.DisabledLoader
    if ($on -and $off) { return 'conflict' }
    if ($on) { return 'mod' }
    if ($off) { return 'original' }
    return 'none'
}
function Get-LegacyRootItems([string]$Game) {
    $items = @()
    foreach ($name in $LegacyRootFiles) {
        $path = Join-Path $Game $name
        if (Test-Path -LiteralPath $path) { $items += $path }
    }
    foreach ($pattern in $LegacyRootDirs) {
        $items += @(Get-ChildItem -LiteralPath $Game -Directory -Filter $pattern -ErrorAction SilentlyContinue | ForEach-Object FullName)
    }
    return $items
}
function New-Backup([string]$Kind) {
    $path = Join-Path $backupRoot ("$Kind-" + (Get-Date -Format 'yyyyMMdd-HHmmss') + '-' + [guid]::NewGuid().ToString('N').Substring(0, 8))
    New-Item -ItemType Directory -Path $path -Force | Out-Null
    return $path
}
# Moves a file or folder into the backup (copy then delete when across drives).
function Move-ToBackup([string]$Path, [string]$Destination) {
    New-Item -ItemType Directory -Path (Split-Path -Parent $Destination) -Force | Out-Null
    if ([IO.Path]::GetPathRoot($Path) -eq [IO.Path]::GetPathRoot($Destination)) {
        Move-Item -LiteralPath $Path -Destination $Destination
    } else {
        Copy-Item -LiteralPath $Path -Destination $Destination -Recurse
        Remove-Item -LiteralPath $Path -Recurse -Force
    }
}

# ---------- install / update ----------
function Invoke-Install([string]$Game) {
    Assert-GameClosed
    $L = Get-Layout $Game
    Say "安装到：$Game"
    $sourceCore = Join-Path $source 'Game\Binaries\Win64\UE4SS\UE4SS.dll'
    if ((Get-FileHash -LiteralPath $sourceCore).Hash -ne $CoreHash) { throw '仓库里的 UE4SS 与本 MOD 不匹配。' }
    # Complete preflight before any writes. Custom layouts remain manual, not overwritten.
    foreach ($path in @($L.Bin, $L.UE4SS, $L.Mods, $L.Mod, (Join-Path $Game 'EasyAntiCheat'))) { Assert-NoReparsePath $path }
    if (Test-Path -LiteralPath $L.Mod) {
        if (Get-ChildItem -LiteralPath $L.Mod -Recurse -Force | Where-Object { $_.Attributes -band [IO.FileAttributes]::ReparsePoint }) {
            throw '现有 MOD 目录中包含链接文件，请手动安装。'
        }
    }
    if ((Get-Mode $L) -eq 'conflict') { throw '启用和停用的加载器文件同时存在，请手动处理；没有改动任何文件。' }
    $reuse = Test-Path -LiteralPath $L.Core -PathType Leaf
    if ($reuse) {
        if ((Get-FileHash -LiteralPath $L.Core).Hash -ne $CoreHash) {
            throw '已有的 UE4SS 不是本 MOD 验证过的版本，没有替换任何文件。请使用兼容的 MOD 版本，或另行升级框架。'
        }
        if (Test-Path -LiteralPath (Join-Path $L.Bin 'UE4SS.dll')) { throw '发现多套 UE4SS 运行库布局，请手动选择正在使用的框架；没有替换任何文件。' }
        $overridePath = Join-Path $L.Bin 'override.txt'
        if ((Test-Path -LiteralPath $overridePath) -and [IO.File]::ReadAllText($overridePath).Trim() -ne 'UE4SS') {
            throw 'override.txt 中有自定义加载器重定向，请手动整合；没有替换任何文件。'
        }
        $settingsPath = Join-Path $L.UE4SS 'UE4SS-settings.ini'
        if (Test-Path -LiteralPath $settingsPath) {
            $settings = [IO.File]::ReadAllText($settingsPath)
            if ($settings -match '(?im)^\s*[+\-]?(ModsFolderPath|ModsFolderPaths|ControllingModsTxt)\s*=[\t ]*[^\s;\r\n]') {
                throw '检测到自定义 MOD 搜索路径，请手动安装；已有设置未改动。'
            }
            if ($settings -match '(?im)^\s*HookEngineTick\s*=[\t ]*(0|false)[\t ]*(?:;.*)?\r?$') { throw '本 MOD 需要 HookEngineTick，已有设置未改动。' }
        }
    } else {
        foreach ($relative in @('UE4SS', 'UE4SS.dll', 'dwmapi.dll', 'dwmapi.dll.disabled', 'override.txt', 'winhttp.dll', 'version.dll', 'dinput8.dll', 'xinput1_3.dll', 'dxgi.dll')) {
            if (Test-Path -LiteralPath (Join-Path $L.Bin $relative)) { throw "发现无法识别的框架/加载器（$relative），没有覆盖，需要手动整合。" }
        }
    }
    $updates = Get-ModListUpdates $L.Mods $true
    foreach ($path in @($updates.Keys) + @($L.Offline)) { Assert-NoReparsePath $path }
    $modSource = Join-Path $source 'Game\Binaries\Win64\UE4SS\Mods\AC8MouseAim'
    $configPath = Join-Path $L.Mod 'config.ini'

    # Settings the player chose in [control] survive an update; version adaptation
    # keys (input slots, axis signs) and diagnostics come from the new files.
    $kept = [ordered]@{}
    if (Test-Path -LiteralPath $configPath) {
        $section = ''
        foreach ($line in Get-Content -LiteralPath $configPath) {
            if ($line -match '^\s*\[(.+)\]\s*$') { $section = $Matches[1].Trim(); continue }
            if ($section -eq 'control' -and $line -match '^\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.*?)\s*$' -and
                $Matches[1] -notin @('pitch_slot', 'roll_slot', 'pitch_sign', 'roll_sign', 'yaw_sign', 'input_probe')) {
                $kept[$Matches[1]] = $Matches[2]
            }
        }
    }

    $backup = New-Backup 'install'
    if (Test-Path -LiteralPath $L.Mod) { Copy-Item -LiteralPath $L.Mod -Destination (Join-Path $backup 'AC8MouseAim') -Recurse }
    foreach ($path in $updates.Keys) {
        if (Test-Path -LiteralPath $path) { Copy-Item -LiteralPath $path -Destination (Join-Path $backup (Split-Path -Leaf $path)) }
    }
    if (Test-Path -LiteralPath $L.Offline) { Copy-Item -LiteralPath $L.Offline -Destination $backup }
    if (-not $reuse) {
        Copy-Item -Path (Join-Path $source 'Game\*') -Destination (Join-Path $Game 'Game') -Recurse -Force
    } else {
        New-Item -ItemType Directory -Path $L.Mod -Force | Out-Null
        foreach ($item in Get-ChildItem -LiteralPath $modSource -Force) { Copy-Item -LiteralPath $item.FullName -Destination $L.Mod -Recurse -Force }
    }
    Write-ModListUpdates $updates
    Copy-Item -LiteralPath (Join-Path $source 'EasyAntiCheat\AC8MouseAim_Offline.json') -Destination $L.Offline -Force
    # Earlier versions left helper scripts and backups in the game root: move them out.
    $legacy = Get-LegacyRootItems $Game
    foreach ($path in $legacy) { Move-ToBackup $path (Join-Path $backup ('game-root\' + (Split-Path -Leaf $path))) }
    # Verify the actual files, not just whether Copy-Item returned successfully.
    foreach ($file in Get-ChildItem -LiteralPath $modSource -Recurse -File) {
        $relative = $file.FullName.Substring($modSource.Length).TrimStart('\')
        if ((Get-FileHash -LiteralPath $file.FullName).Hash -ne (Get-FileHash -LiteralPath (Join-Path $L.Mod $relative)).Hash) { throw "MOD 文件校验失败：$relative" }
    }
    if ((Get-FileHash -LiteralPath $L.Core).Hash -ne $CoreHash) { throw '运行库校验失败。' }

    if ($kept.Count -gt 0) {
        $section = ''; $restored = 0
        $lines = foreach ($line in Get-Content -LiteralPath $configPath) {
            if ($line -match '^\s*\[(.+)\]\s*$') { $section = $Matches[1].Trim(); $line; continue }
            if ($section -eq 'control' -and $line -match '^(\s*)([A-Za-z_][A-Za-z0-9_]*)(\s*=\s*)(.*)$' -and
                $kept.Contains($Matches[2]) -and $Matches[4].Trim() -ne $kept[$Matches[2]]) {
                $restored++
                "$($Matches[1])$($Matches[2])$($Matches[3])$($kept[$Matches[2]])"
            } else { $line }
        }
        if ($restored -gt 0) {
            [IO.File]::WriteAllLines($configPath, [string[]]$lines, (New-Object Text.UTF8Encoding($false)))
            Say "已保留你之前修改过的 $restored 项设置。"
        }
    }
    Say ''
    Say '安装完成，文件校验通过。' Green
    if ($reuse) { Say '已复用兼容的 UE4SS：加载器、框架设置和其他 MOD 均未改动。' } else { Say '已安装随附的 UE4SS 框架。' }
    if ($legacy.Count) { Say "已把旧版本留在游戏根目录的 $($legacy.Count) 个脚本/备份移出游戏目录。" }
    Say "备份：$backup"
    Say '安装层面的共存不代表与其他输入、相机类 MOD 功能上不冲突。仅限离线单人使用。'
    if ((Get-Mode $L) -eq 'original') { Say '当前是联机原版模式（加载器已停用），要使用 MOD 请在菜单中切换回离线 MOD 模式。' Yellow }
    if (-not $NoPause) {
        if ((Get-LaunchOptionState) -eq 'set') { Say 'Steam 启动参数已经设置好，直接从 Steam 启动游戏即可。' Green }
        else { Show-LaunchOptionHelp }
    }
}

# ---------- offline MOD mode / original mode ----------
function Invoke-Switch([string]$Game) {
    Assert-GameClosed
    $L = Get-Layout $Game
    $launch = Get-LaunchOptionState
    switch (Get-Mode $L) {
        'conflict' { throw '启用和停用的加载器文件同时存在，请手动处理；没有改动任何文件。' }
        'none' { throw '没有找到 dwmapi 加载器。如果你使用其他加载器，请沿用它自己的启动方式。' }
        'mod' {
            Say '这会停用共用的 dwmapi 加载器，所有通过它加载的 MOD 都会一起停用。'
            if (-not (Confirm-Choice '切换到联机原版模式？')) { return }
            Move-Item -LiteralPath $L.Loader -Destination $L.DisabledLoader
            Say '已切换到联机原版模式：MOD 加载器已停用。' Green
            if ($launch -ne 'missing') { Say '联机前还需要：Steam 库 → 右键 ACE COMBAT 8 → 属性 → 通用 → 启动选项，清空本 MOD 的离线启动参数。' Yellow }
            Say '其他类型的加载器不在处理范围内，联机前请自行确认已全部停用。'
        }
        'original' {
            Say '这会启用共用的 dwmapi 加载器，以及所有通过它加载的已启用 MOD。'
            if (-not (Confirm-Choice '切换到离线 MOD 模式？')) { return }
            Move-Item -LiteralPath $L.DisabledLoader -Destination $L.Loader
            Say '已切换到离线 MOD 模式：MOD 加载器已启用。' Green
            if ($launch -eq 'set') { Say 'Steam 启动参数已经设置好，直接从 Steam 启动游戏即可。' } else { Show-LaunchOptionHelp }
        }
    }
}

# ---------- uninstall ----------
function Invoke-Uninstall([string]$Game) {
    Assert-GameClosed
    $L = Get-Layout $Game
    $legacy = Get-LegacyRootItems $Game
    if (-not (Test-Path -LiteralPath $L.Mod) -and -not (Test-Path -LiteralPath $L.Offline) -and -not $legacy.Count) {
        Say '没有发现已安装的 AC8 MouseFlight。'
        return
    }
    if (-not (Confirm-Choice '确定卸载 AC8 MouseFlight？')) { return }
    $launch = Get-LaunchOptionState
    # Other mods sharing the framework (the framework and loader stay for them).
    $others = @()
    if (Test-Path -LiteralPath $L.Mods) {
        $others += @(Get-ChildItem -LiteralPath $L.Mods -Force | Where-Object { $_.Name -notin @('AC8MouseAim', 'mods.txt', 'mods.json') } | ForEach-Object Name)
        $txt = Join-Path $L.Mods 'mods.txt'
        if (Test-Path -LiteralPath $txt) {
            foreach ($line in Get-Content -LiteralPath $txt) {
                $line = ($line -split ';', 2)[0].Trim()
                if ($line -and -not $line.StartsWith('#') -and $line -notmatch '^AC8MouseAim\s*:') { $others += $line }
            }
        }
        $json = Join-Path $L.Mods 'mods.json'
        if (Test-Path -LiteralPath $json) {
            foreach ($entry in (Get-Content -LiteralPath $json -Raw | ConvertFrom-Json).mods) {
                if ($entry.mod_name -ne 'AC8MouseAim') { $others += $entry.mod_name }
            }
        }
    }
    $settings = Join-Path $L.UE4SS 'UE4SS-settings.ini'
    if ((Test-Path -LiteralPath $settings) -and
        [IO.File]::ReadAllText($settings) -match '(?im)^\s*[+\-]?(ModsFolderPath|ModsFolderPaths|ControllingModsTxt)\s*=[\t ]*[^\s;\r\n]') {
        $others += 'custom mod folders'
    }
    $updates = Get-ModListUpdates $L.Mods $false
    $targets = @($L.Mod, $L.Offline) + $legacy
    foreach ($path in $targets + @($updates.Keys)) { Assert-NoReparsePath $path }

    $backup = New-Backup 'uninstall'
    foreach ($path in $updates.Keys) {
        if (Test-Path -LiteralPath $path) { Copy-Item -LiteralPath $path -Destination (Join-Path $backup (Split-Path -Leaf $path)) }
    }
    Write-ModListUpdates $updates
    foreach ($path in $targets) {
        if (Test-Path -LiteralPath $path) { Move-ToBackup $path (Join-Path $backup (Split-Path -Leaf $path)) }
    }
    Say 'AC8 MouseFlight 已卸载。' Green

    # With no other mods, the framework and loader this mod installed can go too, leaving
    # the game folder as it was. Only the exact files this mod ships are recognized.
    $frameworkOurs = (Test-Path -LiteralPath $L.Core) -and (Get-FileHash -LiteralPath $L.Core).Hash -eq $CoreHash
    $override = Join-Path $L.Bin 'override.txt'
    if ((Test-Path -LiteralPath $override) -and [IO.File]::ReadAllText($override).Trim() -ne 'UE4SS') { $frameworkOurs = $false }
    $loaders = @($L.Loader, $L.DisabledLoader) | Where-Object { Test-Path -LiteralPath $_ }
    foreach ($path in $loaders) { if ((Get-FileHash -LiteralPath $path).Hash -notin $LoaderHashes) { $frameworkOurs = $false } }
    if ($others.Count -eq 0 -and $frameworkOurs) {
        if (Confirm-Choice '没有发现其他 MOD。是否同时移除 UE4SS 框架和加载器，让游戏目录恢复原样？') {
            foreach ($path in @($L.UE4SS) + @($loaders) + @($override)) {
                if (Test-Path -LiteralPath $path) { Move-ToBackup $path (Join-Path $backup ('framework\' + (Split-Path -Leaf $path))) }
            }
            Say 'UE4SS 框架和加载器已移除，游戏目录已恢复原样。' Green
        }
    } elseif ($others.Count) {
        Say "发现其他 MOD（$(($others | Select-Object -Unique) -join '、')），UE4SS 框架和加载器已保留。其他 MOD 仍可能被加载，这不等于干净的联机环境。"
    }
    Say "备份：$backup"
    if ($launch -ne 'missing') { Say '最后一步：Steam 库 → 右键 ACE COMBAT 8 → 属性 → 通用 → 启动选项，清空本 MOD 的离线启动参数。' Yellow }
}

# ---------- menu ----------
function Show-Menu {
    $game = Resolve-GamePath
    if (-not $game) { Say '已取消。'; return }
    while ($true) {
        Clear-Host
        $L = Get-Layout $game
        $installed = Test-Path -LiteralPath (Join-Path $L.Mod 'Scripts\main.lua')
        $mode = Get-Mode $L
        $launch = Get-LaunchOptionState
        Say '==== AC8 MouseFlight ====' Cyan
        Say "游戏目录：$game"
        if (-not $installed) { Say '状态：未安装' Yellow }
        else {
            switch ($mode) {
                'mod' { Say '状态：已安装，离线 MOD 模式' Green }
                'original' { Say '状态：已安装，联机原版模式（MOD 已停用）' Yellow }
                'conflict' { Say '状态：已安装，但加载器文件冲突，需要手动处理' Red }
                default { Say '状态：已安装，未找到 dwmapi 加载器' Yellow }
            }
        }
        switch ($launch) {
            'set' { Say 'Steam 启动参数：已设置' Green }
            'missing' { Say 'Steam 启动参数：未设置' Yellow }
            default { Say 'Steam 启动参数：无法读取 Steam 设置' }
        }
        $legacy = Get-LegacyRootItems $game
        if ($legacy.Count) { Say "游戏根目录里有 $($legacy.Count) 个旧版本留下的脚本/备份，安装或更新时会自动移走。" Yellow }
        Say ''
        Say ('  1  ' + $(if ($installed) { '更新 / 修复' } else { '安装' }))
        if ($installed -and $mode -eq 'mod') { Say '  2  切换到联机原版模式' }
        elseif ($installed -and $mode -eq 'original') { Say '  2  切换到离线 MOD 模式' }
        Say '  3  复制 Steam 启动参数'
        if ($installed -or $legacy.Count) { Say '  4  卸载' }
        Say '  0  退出'
        Say ''
        $choice = (Read-Host '请选择').Trim()
        if ($choice -eq '0' -or $choice -eq '') { return }
        Say ''
        try {
            switch ($choice) {
                '1' { Invoke-Install $game }
                '2' { if ($installed) { Invoke-Switch $game } }
                '3' { Show-LaunchOptionHelp }
                '4' { Invoke-Uninstall $game }
                default { Say '无效的选项。' }
            }
        } catch {
            Say "出错：$($_.Exception.Message)" Red
        }
        Say ''
        Read-Host '按回车返回菜单' | Out-Null
    }
}

try {
    switch ($Action) {
        'Menu' { Show-Menu }
        'CopyLaunch' { Show-LaunchOptionHelp }
        default {
            $game = Resolve-GamePath
            if (-not $game) { Say '已取消。'; exit 0 }
            switch ($Action) {
                'Install' { Invoke-Install $game }
                'Switch' { Invoke-Switch $game }
                'Uninstall' { Invoke-Uninstall $game }
            }
        }
    }
} catch {
    Say "出错：$($_.Exception.Message)" Red
    if (-not $NoPause -and $Action -ne 'Menu') { Read-Host '按回车键关闭' | Out-Null }
    exit 1
}
exit 0
