$ErrorActionPreference = 'Stop'
Import-Module Microsoft.PowerShell.Utility -ErrorAction Stop
$commit = 'e3ba1016562d6c0868c410d0a71e88bfcdbf691b'
$repository = 'https://github.com/UE4SS-RE/RE-UE4SS.git'
$repo = Split-Path -Parent $PSScriptRoot
$source = Join-Path $repo 'deps\ue4ss-source'
$gitArgs = @('-c', "safe.directory=$($source.Replace('\','/'))", '-C', $source)
$runtime = Join-Path $repo 'Payload\Game\Binaries\Win64\UE4SS\UE4SS.dll'
$runtimeHash = '680A026890ABB4D0DF2211251F8DEFC1681A584275F1521DCC0FE30AF480006F'
if (-not (Test-Path -LiteralPath $runtime -PathType Leaf) -or
    (Get-FileHash -LiteralPath $runtime -Algorithm SHA256).Hash -ne $runtimeHash) {
    throw 'The bundled UE4SS.dll does not match the pinned build. Restore the release package before building.'
}

if (-not (Test-Path -LiteralPath (Join-Path $source '.git'))) {
    New-Item -ItemType Directory -Path $source -Force | Out-Null
    & git @gitArgs init -q
    if ($LASTEXITCODE -ne 0) { throw 'Could not initialize the UE4SS source checkout.' }
    & git @gitArgs remote add origin $repository
    if ($LASTEXITCODE -ne 0) { throw 'Could not set the UE4SS upstream.' }
}

$current = & git @gitArgs rev-parse HEAD 2>$null
if ($LASTEXITCODE -ne 0 -or $current -ne $commit) {
    $changes = & git @gitArgs status --porcelain 2>$null
    if ($LASTEXITCODE -eq 0 -and $changes) {
        throw 'The local UE4SS source has changes. Preserve them before changing its revision.'
    }
    & git @gitArgs fetch --depth 1 origin $commit
    if ($LASTEXITCODE -ne 0) { throw 'Could not download the pinned UE4SS commit.' }
    & git @gitArgs checkout --detach $commit
    if ($LASTEXITCODE -ne 0) { throw 'Could not check out the pinned UE4SS commit.' }
}

foreach ($relative in @(
    'deps\first\LuaMadeSimple\include\LuaMadeSimple\Common.hpp',
    'deps\first\LuaRaw\include\lua.hpp'
)) {
    if (-not (Test-Path -LiteralPath (Join-Path $source $relative) -PathType Leaf)) {
        throw "Pinned UE4SS source is incomplete: $relative"
    }
}
Write-Host "UE4SS source ready at $commit"
