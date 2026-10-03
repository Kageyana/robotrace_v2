"""Build/archive both firmware variants and replay identical primary logs on the host."""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
PROJECT = ROOT / 'robotrace_v2'
OUT = ROOT / 'analysis/fpu-float-audit'
INCLUDES = ['Core/Inc', 'FATFS/Target', 'FATFS/App',
            'Drivers/STM32F4xx_HAL_Driver/Inc', 'Drivers/STM32F4xx_HAL_Driver/Inc/Legacy',
            'Middlewares/Third_Party/FatFs/src', 'Drivers/CMSIS/Device/ST/STM32F4xx/Include',
            'Drivers/CMSIS/Include']


def run(args, log=None, cwd=PROJECT):
    result = subprocess.run([str(a) for a in args], cwd=cwd, capture_output=True, text=True,
                            encoding='utf-8', errors='replace')
    output = result.stdout + result.stderr
    if log:
        Path(log).write_text(output, encoding='utf-8')
    if result.returncode:
        raise RuntimeError(f'Command failed ({result.returncode}): {args}\n{output[-4000:]}')
    return output


def function_body(source, name):
    match = re.search(r'^(?:static )?void ' + name + r'\([^;]*?\)\s*\{', source, re.M)
    if not match:
        raise ValueError(f'Function not found: {name}')
    end = source.index('{', match.start()) + 1
    depth = 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


def host_flags():
    return ['-std=gnu11', '-DSTM32F446xx', '-DUSE_HAL_DRIVER',
            '-ffunction-sections', '-fdata-sections'] + [f'-I{PROJECT / d}' for d in INCLUDES]


def timing_test(directory):
    timer = (PROJECT / 'Core/Src/timer.c').read_text(encoding='utf-8-sig')
    source = directory / 'timing.c'
    source.write_text('#include "timer.h"\n#include <assert.h>\n'
                      'uint32_t SystemCoreClock = 180000000U;\n'
                      'volatile IsrTimingStats isrTimingStats[ISR_TIMING_MODE_COUNT] = {0};\n'
                      + function_body(timer, 'recordIsrTiming') + r'''
int main(void) {
    recordIsrTiming(2, 50); recordIsrTiming(2, 180000); recordIsrTiming(2, 200000);
    assert(isrTimingStats[2].samples == 3 && isrTimingStats[2].totalCycles == 380050);
    assert(isrTimingStats[2].minCycles == 50 && isrTimingStats[2].maxCycles == 200000);
    assert(isrTimingStats[2].over1ms == 2 && isrTimingStats[0].samples == 0);
    recordIsrTiming(5, UINT32_MAX);
    isrTimingStats[1].totalCycles = UINT32_MAX; recordIsrTiming(1, 100);
    assert(isrTimingStats[1].totalCycles == (uint64_t)UINT32_MAX + 100);
    isrTimingStats[1].samples = UINT32_MAX; recordIsrTiming(1, 100);
    assert(isrTimingStats[1].totalCycles == (uint64_t)UINT32_MAX + 100);
    return 0;
}
''', encoding='utf-8')
    executable = directory / 'timing.exe'
    run(['gcc', *host_flags(), '-DROBOTRACE_ISR_TIMING=1', '-Wall', '-Wextra', '-Werror',
         source, '-o', executable])
    run([executable])


def snapshots(directory, destination, primary):
    # Reuse the existing FatFs stubs; do not duplicate production geometry or XY formulas.
    fixture = (PROJECT / 'tests/path_route_builder_test.c').read_text(encoding='utf-8-sig')
    fixture = fixture.replace('#define MOCK_CSV_CAPACITY 120000U', '#define MOCK_CSV_CAPACITY 4000000U')
    fixture += r'''
void auditDumpRoute(void);
int main(int argc, char **argv) {
    CHECK(argc == 2);
    FILE *input = fopen(argv[1], "rb"); CHECK(input != NULL);
    mockCsvLength = fread(mockCsv, 1, MOCK_CSV_CAPACITY - 1, input);
    CHECK(!ferror(input) && feof(input)); fclose(input); mockCsv[mockCsvLength] = '\0';
    for (uint8_t level = 0; level <= 1; level++) {
        prepareRouteSettings();
        int result = routeBuildFromLog(1, level);
        printf("result,%u,%d,%u,%u,%u,%u\n", level, result, optimalTrace,
               pathRouteCount(), pathRouteShortcutLevel(), pathRouteShortcutBuildStatus());
        if (result > 0) auditDumpRoute();
    }
    return 0;
}
'''
    route_source = (PROJECT / 'Core/Src/pathFollower.c').read_text(encoding='utf-8-sig')
    route_source += r'''
#include <stdio.h>
void auditDumpRoute(void) {
    for (uint16_t i = 0; i < routeCount; i++)
        printf("point,%u,%d,%d,%d,%u,%d,%d,%d,%u,%u\n", i,
            lineRoute[i].x_mm, lineRoute[i].y_mm, lineRoute[i].heading_cdeg, lineRoute[i].speed_cms,
            driveRoute[i].x_mm, driveRoute[i].y_mm, driveRoute[i].heading_cdeg,
            driveRoute[i].speed_cms, driveRouteArcMm[i]);
}
'''
    fixture_path, route_path = directory / 'route_main.c', directory / 'route.c'
    fixture_path.write_text(fixture, encoding='utf-8')
    route_path.write_text(route_source, encoding='utf-8')
    route_object = directory / 'route.o'
    run(['gcc', *host_flags(), '-w', '-c', route_path, '-o', route_object])
    executable = directory / 'route.exe'
    run(['gcc', *host_flags(), '-Wall', '-Wextra', '-Werror', fixture_path, route_object,
         '-Wl,--gc-sections', '-lm', '-o', executable])
    route_result = run([executable, primary], destination / f'{primary.stem}-route.csv')

    xy_source = directory / 'xy.c'
    run(['python', PROJECT / 'tests/extract_log_xy_source.py',
         PROJECT / 'Core/Src/courseAnalysis.c', xy_source])
    text = xy_source.read_text(encoding='utf-8')
    text += r'''
#include <stdio.h>
#include <stdlib.h>
int main(int argc, char **argv) {
    if (argc != 3) return 2;
    FILE *input = fopen(argv[1], "r"); if (!input) return 3;
    int goal = atoi(argv[2]), previousPulse = 0, pulse; float yaw, goalX = NAN;
    clearXYcie();
    while (fscanf(input, "%d,%f", &pulse, &yaw) == 2) {
        float previousX = xycie.x; calcXYcie(pulse, yaw);
        if (!isfinite(goalX) && goal >= previousPulse && goal <= pulse) {
            float ratio = pulse > previousPulse ? (float)(goal - previousPulse) / (pulse - previousPulse) : 1.0f;
            goalX = previousX + (xycie.x - previousX) * ratio;
        }
        previousPulse = pulse;
    }
    fclose(input);
    printf("{\"x_mm\":%.9g,\"y_mm\":%.9g,\"goal_x_mm\":%.9g,\"goal_x_valid\":%d}\n",
        (double)xycie.x, (double)xycie.y, (double)goalX, isfinite(goalX) && fabsf(goalX) <= 60.0f);
    return 0;
}
'''
    xy_source.write_text(text, encoding='utf-8')
    with primary.open(encoding='utf-8-sig', newline='') as stream:
        metadata = dict(item.split('=', 1) for item in stream.readline().strip().split(',') if '=' in item)
        records = list(csv.DictReader(stream))
    pulse_yaw = directory / 'pulse-yaw.csv'
    pulse_yaw.write_text(''.join(f'{r["encTotalOptimal"]},{r["imuAngle_Z"]}\n' for r in records), encoding='utf-8')
    executable = directory / 'xy.exe'
    run(['gcc', *host_flags(), '-Wall', '-Wextra', '-Werror', xy_source, '-lm', '-o', executable])
    xy_result = run([executable, pulse_yaw, metadata['goalMarkerOnset_p']],
                    destination / f'{primary.stem}-xy.json')
    return {'path': str(primary), 'sha256': hashlib.sha256(primary.read_bytes()).hexdigest(),
            'rows': len(records), 'route_results': [s for s in route_result.splitlines() if s.startswith('result,')],
            'xy': json.loads(xy_result)}


def build_and_audit(variant, primary):
    dirty = run(['git', 'status', '--porcelain', '--', 'robotrace_v2/Core',
                 'robotrace_v2/CMakeLists.txt', 'robotrace_v2/cmake',
                 'analysis/script/fpu_float_validation.py'], cwd=ROOT).strip()
    if dirty:
        raise RuntimeError('Commit firmware and validation script before archiving: ' + dirty)
    destination = OUT / variant
    destination.mkdir(parents=True, exist_ok=True)
    commit = run(['git', 'rev-parse', 'HEAD'], cwd=ROOT).strip()
    manifest = {'variant': variant, 'commit': commit, 'timing': True, 'log_profile': 1,
                'builds': {}, 'primary_logs': []}
    for config in ('Debug', 'Release'):
        build = PROJECT / 'build/fpu-float-validation' / variant / config
        archive = destination / config
        archive.mkdir(parents=True, exist_ok=True)
        configure = ['cmake', '--preset', config, '-B', build, '-DROBOTRACE_ISR_TIMING=ON',
                     '-DROBOTRACE_LOG_SCHEMA_PROFILE_LIGHT=1']
        run(configure, archive / 'configure.txt')
        run(['cmake', '--build', build, '--clean-first'], archive / 'build.txt')
        for suffix in ('elf', 'map'):
            shutil.copy2(build / f'robotrace_v2.{suffix}', archive / f'robotrace_v2.{suffix}')
        commands = json.loads((build / 'compile_commands.json').read_text(encoding='utf-8'))
        compiler = commands[0]['command'].split()[0].replace('\\\\', '\\')
        objdump = Path(compiler).with_name('arm-none-eabi-objdump.exe')
        assembly = run([objdump, '-d', archive / 'robotrace_v2.elf'], archive / 'disassembly.txt')
        calls, function = {}, ''
        for line in assembly.splitlines():
            entry = re.match(r'^[0-9a-f]+ <([^>]+)>:', line)
            if entry:
                function = entry[1]
            call = re.search(r'\bbl\S*\s+.*<(__aeabi_(?:d\w+|f2d))>', line)
            if call:
                calls.setdefault(function, []).append(call[1])
        (archive / 'double-calls.json').write_text(json.dumps(calls, indent=2), encoding='utf-8')
        diagnostics = []
        if config == 'Debug':
            for entry in commands:
                if '/Core/Src/' not in entry['file'].replace('\\', '/'):
                    continue
                tokens = entry['command'].split()
                args = [t for t in tokens if t.startswith('-I') or (t.startswith('-D') and not t.startswith('-DGIT_'))]
                diagnostics.append(run([compiler, *args, '-mcpu=cortex-m4', '-mfpu=fpv4-sp-d16',
                                        '-mfloat-abi=hard', '-std=gnu11', '-fsyntax-only',
                                        '-Wdouble-promotion', '-Wfloat-conversion', '-Wunsuffixed-float-constants', entry['file']]))
            (archive / 'compiler-diagnostics.txt').write_text('\n'.join(diagnostics), encoding='utf-8')
        manifest['builds'][config] = {'configure_command': [str(a) for a in configure],
            'elf_sha256': hashlib.sha256((archive / 'robotrace_v2.elf').read_bytes()).hexdigest(),
            'compiler': run([compiler, '--version']).splitlines()[0],
            'normal_warnings': [s for s in (archive / 'build.txt').read_text(encoding='utf-8').splitlines() if 'warning:' in s]}
        print(f'{variant}: {config} build/audit OK', flush=True)
    for script in ('run_pc_tests.ps1', 'run_log_xy_tests.ps1', 'run_log_schema_tests.ps1'):
        run(['powershell', '-NoProfile', '-File', PROJECT / 'tests' / script], destination / f'{script}.txt')
    with tempfile.TemporaryDirectory(prefix='robotrace-fpu-') as temp:
        directory = Path(temp)
        timing_test(directory)
        for log in primary:
            manifest['primary_logs'].append(snapshots(directory, destination, log))
    (destination / 'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding='utf-8')
    print(f'{variant}: host tests, timing aggregation and primary replay OK', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variant', required=True, choices=['baseline', 'float'])
    parser.add_argument('--primary', type=Path, action='append', required=True)
    args = parser.parse_args()
    build_and_audit(args.variant, args.primary)
