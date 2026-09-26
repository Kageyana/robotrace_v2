$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$testExecutable = Join-Path $env:TEMP "robotrace-auto-run-tests-$PID.exe"
$pathFollowerObject = Join-Path $env:TEMP "robotrace-path-follower-tests-$PID.o"
$includeArgs = @(
    '-I', (Join-Path $projectRoot 'Core/Inc'),
    '-I', (Join-Path $projectRoot 'FATFS/Target'),
    '-I', (Join-Path $projectRoot 'FATFS/App'),
    '-I', (Join-Path $projectRoot 'Drivers/STM32F4xx_HAL_Driver/Inc'),
    '-I', (Join-Path $projectRoot 'Drivers/STM32F4xx_HAL_Driver/Inc/Legacy'),
    '-I', (Join-Path $projectRoot 'Middlewares/Third_Party/FatFs/src'),
    '-I', (Join-Path $projectRoot 'Drivers/CMSIS/Device/ST/STM32F4xx/Include'),
    '-I', (Join-Path $projectRoot 'Drivers/CMSIS/Include')
)

& gcc -std=gnu11 -w -ffunction-sections -fdata-sections `
    -DSTM32F446xx -DUSE_HAL_DRIVER @includeArgs `
    -c (Join-Path $projectRoot 'Core/Src/pathFollower.c') `
    -o $pathFollowerObject
if ($LASTEXITCODE -ne 0) {
    throw "Host PATH firmware compilation failed with exit code $LASTEXITCODE."
}

& gcc -std=c11 -Wall -Wextra -Werror -ffunction-sections -fdata-sections `
    -DSTM32F446xx -DUSE_HAL_DRIVER -I (Join-Path $projectRoot 'Core/Inc') @includeArgs `
    (Join-Path $PSScriptRoot 'auto_run_test.c') `
    (Join-Path $PSScriptRoot 'path_policy_test.c') `
    (Join-Path $PSScriptRoot 'path_route_builder_test.c') `
    (Join-Path $projectRoot 'Core/Src/autoRun.c') `
    (Join-Path $projectRoot 'Core/Src/courseLogCsv.c') `
    $pathFollowerObject `
    '-Wl,--gc-sections' -lm `
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
    Remove-Item -LiteralPath $pathFollowerObject -ErrorAction SilentlyContinue
}
