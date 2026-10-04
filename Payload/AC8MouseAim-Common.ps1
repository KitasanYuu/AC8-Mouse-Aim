# Shared helpers for the AC8 MouseFlight installer and game-folder tools.
# Read-only toward Steam: Steam's own settings are never modified.

$AC8AppId = '2288340'
$AC8LaunchOption = 'cmd /d /c "set EOS_USE_ANTICHEATCLIENTNULL=1&& %command% -anticheat_settings=AC8MouseAim_Offline.json"'

function Test-AC8GameDirectory([string]$Path) {
    if ([string]::IsNullOrWhiteSpace($Path)) { return $false }
    return ((Test-Path -LiteralPath (Join-Path $Path 'Game\Binaries\Win64\AceCombat8.exe') -PathType Leaf) -and
        (Test-Path -LiteralPath (Join-Path $Path 'EasyAntiCheat') -PathType Container))
}

function Get-AC8SteamRoot {
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
function Find-AC8GamePath {
    $steam = Get-AC8SteamRoot
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
        $manifest = Join-Path $library "steamapps\appmanifest_$AC8AppId.acf"
        if (-not (Test-Path -LiteralPath $manifest)) { continue }
        $dir = [regex]::Match((Get-Content -LiteralPath $manifest -Raw), '"installdir"\s+"([^"]+)"')
        if (-not $dir.Success) { continue }
        $candidate = Join-Path $library ("steamapps\common\" + $dir.Groups[1].Value)
        if (Test-AC8GameDirectory $candidate) { return (Get-Item -LiteralPath $candidate).FullName }
    }
    return $null
}

# 'set' when any Steam user has the offline launch option for the game, 'missing' when
# Steam's settings were readable but none has it, 'unknown' otherwise.
function Get-AC8LaunchOptionState {
    $steam = Get-AC8SteamRoot
    if (-not $steam) { return 'unknown' }
    $configs = Get-ChildItem -LiteralPath (Join-Path $steam 'userdata') -Directory -ErrorAction SilentlyContinue |
        ForEach-Object { Join-Path $_.FullName 'config\localconfig.vdf' } |
        Where-Object { Test-Path -LiteralPath $_ }
    if (-not $configs) { return 'unknown' }
    foreach ($config in $configs) {
        $text = Get-Content -LiteralPath $config -Raw -Encoding UTF8
        foreach ($match in [regex]::Matches($text, '"' + $AC8AppId + '"\s*\{')) {
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

# Copies the launch option to the clipboard and says where to paste it.
function Show-AC8LaunchOptionHelp {
    $copied = $false
    try { Set-Clipboard -Value $AC8LaunchOption; $copied = $true } catch { }
    Write-Host ''
    Write-Host '还需要一步：在 Steam 中设置启动参数' -ForegroundColor Yellow
    if ($copied) { Write-Host '  启动参数已复制到剪贴板。' }
    Write-Host '  Steam 库 → 右键 ACE COMBAT 8 → 属性 → 通用 → 启动选项，粘贴：'
    Write-Host "  $AC8LaunchOption" -ForegroundColor Cyan
    Write-Host '  如果原来已有其他启动选项，请先备份。'
}
