[CmdletBinding()]
param(
    [string]$GamePath,
    [switch]$NoBuild,
    [switch]$IncludeConfig,
    [switch]$Rollback,
    [switch]$Preview,
    [switch]$Live
)
$ErrorActionPreference = 'Stop'
Import-Module Microsoft.PowerShell.Utility -ErrorAction Stop
$repo = Split-Path -Parent $PSScriptRoot
$cache = Join-Path $repo '.dev-game-path'
$backupRoot = Join-Path $repo 'build\dev-backups'
$modRelative = 'Game\Binaries\Win64\UE4SS\Mods\AC8MouseAim'
$runtimeHash = '680A026890ABB4D0DF2211251F8DEFC1681A584275F1521DCC0FE30AF480006F'

if ([string]::IsNullOrWhiteSpace($GamePath) -and (Test-Path -LiteralPath $cache)) {
    $GamePath = (Get-Content -LiteralPath $cache -Raw).Trim()
}
if ([string]::IsNullOrWhiteSpace($GamePath)) {
    Add-Type -AssemblyName System.Windows.Forms
    $picker = New-Object System.Windows.Forms.FolderBrowserDialog
    $picker.Description = 'Select the installed ACE COMBAT 8 game root.'
    $picker.ShowNewFolderButton = $false
    try {
        if ($picker.ShowDialog() -ne [System.Windows.Forms.DialogResult]::OK) { exit 0 }
        $GamePath = $picker.SelectedPath
    } finally { $picker.Dispose() }
}
if (-not (Test-Path -LiteralPath $GamePath -PathType Container)) { throw "Game directory not found: $GamePath" }
$game = (Get-Item -LiteralPath $GamePath).FullName.TrimEnd('\')
$gamePrefix = $game + [IO.Path]::DirectorySeparatorChar
if (-not (Test-Path -LiteralPath (Join-Path $game 'Game\Binaries\Win64\AceCombat8.exe') -PathType Leaf) -or
    -not (Test-Path -LiteralPath (Join-Path $game 'EasyAntiCheat') -PathType Container)) {
    throw 'Select the ACE COMBAT 8 root folder containing Game and EasyAntiCheat.'
}
if ((Get-Item -LiteralPath $game -Force).Attributes -band [IO.FileAttributes]::ReparsePoint) {
    throw 'A linked game root is not supported by Dev-Deploy.'
}

function Assert-GamePath([string]$Path) {
    $full = [IO.Path]::GetFullPath($Path)
    if (-not $full.StartsWith($gamePrefix,[StringComparison]::OrdinalIgnoreCase)) {
        throw "Path outside game directory: $full"
    }
    $cursor = $full
    while ($cursor.StartsWith($gamePrefix,[StringComparison]::OrdinalIgnoreCase)) {
        if (Test-Path -LiteralPath $cursor) {
            if ((Get-Item -LiteralPath $cursor -Force).Attributes -band [IO.FileAttributes]::ReparsePoint) {
                throw "Link or junction refused: $cursor"
            }
        }
        $cursor = Split-Path -Parent $cursor
    }
}
$mod = Join-Path $game $modRelative
$runtime = Join-Path $game 'Game\Binaries\Win64\UE4SS\UE4SS.dll'
foreach ($path in @($mod,$runtime)) { Assert-GamePath $path }

# Live mode: replace only the development flight-logic module. The running game
# reloads it within a second; hooks, loader and scripts are untouched.
if ($Live) {
    if (-not (Test-Path -LiteralPath $mod -PathType Container)) { throw 'Install the mod once before using -Live.' }
    if (-not $NoBuild) {
        & (Join-Path $PSScriptRoot 'build.cmd')
        if ($LASTEXITCODE -ne 0) { throw 'Build failed. No game files were changed.' }
    }
    $source = Join-Path $repo 'build\ac8_flight_logic.dll'
    $destination = Join-Path $mod 'Scripts\ac8_flight_logic.dll'
    Assert-GamePath $destination
    if (-not (Test-Path -LiteralPath $source -PathType Leaf)) { throw "Missing build file: $source" }
    if ($Preview) { Write-Host "Would update $destination"; return }
    # Write beside the target, then rename, so the game never sees a partial file.
    $staging = $destination + '.staging'
    Copy-Item -LiteralPath $source -Destination $staging -Force
    Move-Item -LiteralPath $staging -Destination $destination -Force
    Set-Content -LiteralPath $cache -Value $game -Encoding UTF8
    Write-Host 'Flight logic updated; the running game loads it within a second.'
    return
}
if (Get-Process -Name 'AceCombat8' -ErrorAction SilentlyContinue) {
    throw 'Close ACE COMBAT 8 before deploying or rolling back (use -Live for flight logic).'
}
if (-not (Test-Path -LiteralPath $mod -PathType Container)) {
    if ($Rollback) { throw 'AC8MouseAim is not installed; there is nothing to roll back.' }
    if ($Preview) {
        Write-Host "Would perform the initial installation in $game"
        return
    }
    if (-not $NoBuild) {
        & (Join-Path $PSScriptRoot 'build.cmd')
        if ($LASTEXITCODE -ne 0) { throw 'Build failed. No game files were changed.' }
    }
    & (Join-Path $repo 'tools\AC8MouseFlight.ps1') -Action Install -GamePath $game -NoPause
    Set-Content -LiteralPath $cache -Value $game -Encoding UTF8
    Write-Host 'Initial installation complete. Future Dev-Deploy runs will sync only changed files.'
    return
}
if (-not (Test-Path -LiteralPath $runtime -PathType Leaf) -or
    (Get-FileHash -LiteralPath $runtime -Algorithm SHA256).Hash -ne $runtimeHash) {
    throw 'Installed UE4SS.dll differs from the pinned runtime. Nothing was changed.'
}
$knownLoaderHashes = @(
    'C5D2AB9F9B89BD94460B0A283EEFB113085105014011CAC961F36787376DB744',
    'CF440B9EB8643BB7C434ACFDA696AEE57FD981D185DCA5E57FB8DBB18F8FC1CD'
)
$loader = Join-Path $game 'Game\Binaries\Win64\dwmapi.dll'
if (-not (Test-Path -LiteralPath $loader)) { $loader += '.disabled' }
Assert-GamePath $loader
if (-not (Test-Path -LiteralPath $loader -PathType Leaf) -or
    (Get-FileHash -LiteralPath $loader -Algorithm SHA256).Hash -notin $knownLoaderHashes) {
    throw 'Installed loader is missing or unrecognized. Nothing was changed.'
}

if ($Rollback) {
    $snapshot = Get-ChildItem -LiteralPath $backupRoot -Directory -ErrorAction SilentlyContinue |
        Sort-Object Name -Descending | Where-Object {
            $file = Join-Path $_.FullName 'manifest.json'
            (Test-Path -LiteralPath $file -PathType Leaf) -and
                ((Get-Content -LiteralPath $file -Raw | ConvertFrom-Json).GamePath -eq $game)
        } | Select-Object -First 1
    if (-not $snapshot) { throw 'No development backup is available.' }
    $manifest = Get-Content -LiteralPath (Join-Path $snapshot.FullName 'manifest.json') -Raw | ConvertFrom-Json
    foreach ($relative in $manifest.Files) {
        $source = Join-Path $snapshot.FullName $relative
        $destination = Join-Path $game $relative
        Assert-GamePath $destination
        if (-not (Test-Path -LiteralPath $source -PathType Leaf)) { throw "Backup is incomplete: $relative" }
    }
    foreach ($relative in $manifest.Files) {
        $source = Join-Path $snapshot.FullName $relative
        $destination = Join-Path $game $relative
        if ($Preview) { Write-Host "Would restore $relative" }
        else {
            Copy-Item -LiteralPath $source -Destination $destination -Force
            if ((Get-FileHash -LiteralPath $source).Hash -ne (Get-FileHash -LiteralPath $destination).Hash) {
                throw "Rollback verification failed: $relative"
            }
            Write-Host "Restored $relative"
        }
    }
    return
}

if (-not $NoBuild -and -not $Preview) {
    & (Join-Path $PSScriptRoot 'build.cmd')
    if ($LASTEXITCODE -ne 0) { throw 'Build failed. No game files were changed.' }
}
$files = @(
    'Scripts\ac8_mouse_aim_010.dll',
    'Scripts\main.lua',
    'Scripts\camera.lua',
    'Scripts\gaze.lua',
    'Scripts\gaze_probe.lua',
    'Scripts\rig_math.lua',
    'Scripts\contacts.lua'
)
if ($IncludeConfig) { $files += 'config.ini' }
$changes = @()
foreach ($file in $files) {
    $relative = Join-Path $modRelative $file
    $source = Join-Path (Join-Path $repo 'Payload') $relative
    $destination = Join-Path $game $relative
    Assert-GamePath $destination
    if (-not (Test-Path -LiteralPath $source -PathType Leaf)) { throw "Missing build file: $source" }
    # A file new in this version is added (there is nothing to back up for it).
    $isNew = -not (Test-Path -LiteralPath $destination -PathType Leaf)
    if ($isNew -or (Get-FileHash -LiteralPath $source).Hash -ne (Get-FileHash -LiteralPath $destination).Hash) {
        $changes += [pscustomobject]@{ Relative=$relative; Source=$source; Destination=$destination; New=$isNew }
    }
}
if (-not $changes.Count) {
    if (-not $Preview) { Set-Content -LiteralPath $cache -Value $game -Encoding UTF8 }
    Write-Host 'Development files are already current.'
    return
}
if ($Preview) {
    foreach ($change in $changes) { Write-Host "Would update $($change.Relative)" }
    return
}

$stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$snapshot = Join-Path $backupRoot ($stamp + '-' + [guid]::NewGuid().ToString('N').Substring(0,8))
New-Item -ItemType Directory -Path $snapshot -Force | Out-Null
$started = @()
try {
    foreach ($change in $changes) {
        $backup = Join-Path $snapshot $change.Relative
        if ($change.New) {
            $started += [pscustomobject]@{ Destination=$change.Destination; Backup=$null }
        } else {
            New-Item -ItemType Directory -Path (Split-Path -Parent $backup) -Force | Out-Null
            Copy-Item -LiteralPath $change.Destination -Destination $backup
            $started += [pscustomobject]@{ Destination=$change.Destination; Backup=$backup }
        }
        Copy-Item -LiteralPath $change.Source -Destination $change.Destination -Force
        if ((Get-FileHash -LiteralPath $change.Source).Hash -ne
            (Get-FileHash -LiteralPath $change.Destination).Hash) {
            throw "Deployment verification failed: $($change.Relative)"
        }
        Write-Host "$(if ($change.New) { 'Added' } else { 'Updated' }) $($change.Relative)"
    }
    [pscustomobject]@{ GamePath=$game; Files=@($changes | Where-Object { -not $_.New } | ForEach-Object Relative) } |
        ConvertTo-Json -Depth 3 | Set-Content -LiteralPath (Join-Path $snapshot 'manifest.json') -Encoding UTF8
    Set-Content -LiteralPath $cache -Value $game -Encoding UTF8
    Write-Host "Backup for rollback: $snapshot"
} catch {
    foreach ($entry in $started) {
        if ($entry.Backup) { Copy-Item -LiteralPath $entry.Backup -Destination $entry.Destination -Force }
        elseif (Test-Path -LiteralPath $entry.Destination) { Remove-Item -LiteralPath $entry.Destination -Force }
    }
    throw
}
