param(
    [string]$Python = "python"
)

$ErrorActionPreference = "Stop"
$files = @(
    "__init__.py",
    "config.py",
    "models.py",
    "camera_source.py",
    "line_detector.py",
    "controller.py",
    "runtime.py",
    "motion_output.py",
    "main.py",
    "examples\__init__.py",
    "examples\offline_takeover.py",
    "tests\__init__.py",
    "tests\test_offline.py"
)

& $Python -m py_compile @files
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& $Python -m unittest discover -s tests -v
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& $Python -m examples.offline_takeover
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Output "OFFLINE_CHECK_OK"
