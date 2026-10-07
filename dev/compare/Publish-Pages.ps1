# The bench page for GitHub Pages: the viewer (index.html, models) and the last run's results
# (results/, which are not committed on main) as the one commit of the local branch gh-pages.
# Each run replaces that commit instead of adding to it, so the results (hundreds of MB) never
# pile up in the history. Nothing is pushed: push it yourself with
#   git push -f origin gh-pages
# GitHub serves the branch once Settings > Pages has "Deploy from a branch", gh-pages, / (root).
param([string]$Branch = 'gh-pages')
$ErrorActionPreference = 'Stop'
$compare = $PSScriptRoot
$repo = Split-Path -Parent (Split-Path -Parent $compare)

if (-not (Test-Path -LiteralPath (Join-Path $compare 'results\index.js'))) {
    throw 'No results: run dev\compare\compare.cmd first.'
}
$files = @('index.html', 'models.js')
$files += Get-ChildItem -LiteralPath (Join-Path $compare 'models') -Filter '*.js' -File | ForEach-Object { "models/$($_.Name)" }
$files += Get-ChildItem -LiteralPath (Join-Path $compare 'results') -Filter '*.js' -File | ForEach-Object { "results/$($_.Name)" }

# A tree built in a scratch index, so the working tree and its index are left alone.
$index = Join-Path $repo 'build\gh-pages.index'
New-Item -ItemType Directory -Path (Split-Path -Parent $index) -Force | Out-Null
if (Test-Path -LiteralPath $index) { Remove-Item -LiteralPath $index }
$env:GIT_INDEX_FILE = $index
try {
    foreach ($file in $files) {
        $blob = git -C $repo hash-object -w --no-filters -- (Join-Path $compare $file)
        if ($LASTEXITCODE -ne 0) { throw "Could not store $file" }
        git -C $repo update-index --add --cacheinfo "100644,$blob,$file"
        if ($LASTEXITCODE -ne 0) { throw "Could not stage $file" }
    }
    # no Jekyll: serve the files as they are
    $nothing = Join-Path $repo 'build\gh-pages.nojekyll'
    [IO.File]::WriteAllBytes($nothing, [byte[]]@())
    $empty = git -C $repo hash-object -w -- $nothing
    Remove-Item -LiteralPath $nothing
    git -C $repo update-index --add --cacheinfo "100644,$empty,.nojekyll"
    $tree = git -C $repo write-tree
    if ($LASTEXITCODE -ne 0) { throw 'Could not write the tree.' }
} finally {
    Remove-Item Env:GIT_INDEX_FILE
    if (Test-Path -LiteralPath $index) { Remove-Item -LiteralPath $index }
}

$label = (Select-String -LiteralPath (Join-Path $compare 'results\index.js') -Pattern '"id":"ours","label":"([^"]*)"').Matches[0].Groups[1].Value
$head = git -C $repo rev-parse --short HEAD
$dirty = if (git -C $repo status --porcelain -- src Payload dev/compare/harness.cpp) { ', uncommitted changes' } else { '' }
$message = "Bench results: $label (main $head$dirty)"
$commit = git -C $repo commit-tree $tree -m $message
if ($LASTEXITCODE -ne 0) { throw 'Could not commit.' }
git -C $repo update-ref "refs/heads/$Branch" $commit
if ($LASTEXITCODE -ne 0) { throw "Could not move $Branch." }
Write-Host "$Branch -> $($commit.Substring(0,7)): $message ($($files.Count + 1) files)"
Write-Host "Publish with: git push -f origin $Branch"
