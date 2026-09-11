param(
    [string]$Python = "python"
)

$ErrorActionPreference = "Stop"

# Discover files instead of keeping a hand-written list: a new module file must
# never be silently skipped by the syntax check.
$files = @()
$files += Get-ChildItem -Path . -Filter *.py -File | ForEach-Object { $_.Name }
foreach ($dir in @("examples", "tests", "scripts")) {
    if (Test-Path $dir) {
        $files += Get-ChildItem -Path $dir -Filter *.py -File |
            ForEach-Object { Join-Path $dir $_.Name }
    }
}
$files = $files | Sort-Object -Unique

Write-Output "py_compile: $($files.Count) files"
& $Python -m py_compile @files
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& $Python -m unittest discover -s tests -v
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& $Python -m examples.offline_takeover
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& $Python scripts\check_module.py --all --quiet
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Output "OFFLINE_CHECK_OK"
