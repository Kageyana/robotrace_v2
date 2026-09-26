$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$testExecutable = Join-Path $env:TEMP "robotrace-auto-run-tests-$PID.exe"

& gcc -std=c11 -Wall -Wextra -Werror `
    -I (Join-Path $projectRoot 'Core/Inc') `
    (Join-Path $PSScriptRoot 'auto_run_test.c') `
    (Join-Path $projectRoot 'Core/Src/autoRun.c') `
    (Join-Path $projectRoot 'Core/Src/courseLogCsv.c') `
    -o $testExecutable
if ($LASTEXITCODE -ne 0) {
    throw "Host test compilation failed with exit code $LASTEXITCODE."
}

try {
    & $testExecutable
    if ($LASTEXITCODE -ne 0) {
        throw "Host tests failed with exit code $LASTEXITCODE."
    }
}
finally {
    Remove-Item -LiteralPath $testExecutable -ErrorAction SilentlyContinue
}
