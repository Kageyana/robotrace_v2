"""制御API・PID画面・速度設定・メタデータの実関数をホストで検証する。"""
from pathlib import Path
import argparse
import re
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SRC = ROOT / 'robotrace_v2/Core/Src'
INC = ROOT / 'robotrace_v2/Core/Inc'


def function(text, name):
    match = re.search(r'(?m)^(?:static\s+)?[A-Za-z_][\w *]*\b' + re.escape(name) + r'\([^;{}]*\)\s*\{', text)
    if not match:
        raise ValueError(f'Function missing: {name}')
    tokens = re.compile(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]', re.S)
    depth = 1
    for token in tokens.finditer(text, match.end()):
        if token.group() == '{': depth += 1
        elif token.group() == '}': depth -= 1
        if depth == 0: return text[match.start():token.end()] + '\n'
    raise ValueError(name)


def run(command, log):
    result = subprocess.run([str(x) for x in command], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, encoding='utf-8', errors='replace')
    log.write_text(result.stdout, encoding='utf-8')
    if result.returncode:
        print(result.stdout)
        raise RuntimeError(f'Failed: {command}; log={log}')
    return result.stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, help='任意の作業前Coreを含むフォルダ')
    args = parser.parse_args()
    output = ROOT / 'build/main-refactor-20261025/validation'
    output.mkdir(parents=True, exist_ok=True)
    setup = (SRC/'setup.c').read_text(encoding='utf-8-sig')
    motor = (SRC/'motor.c').read_text(encoding='utf-8-sig')
    pid_h = (INC/'PIDcontrol.h').read_text(encoding='utf-8-sig')
    setup_h = (INC/'setup.h').read_text(encoding='utf-8-sig')
    pid_type = re.search(r'typedef struct \{[^}]*\} pidParam;', pid_h).group()
    pattern_type = re.search(r'typedef struct\s*\{[^}]*\} Pattern;', setup_h).group()
    test_type = re.search(r'typedef struct\s*\{[^}]*\} TestFlags;', setup_h).group()
    generated = pid_type + '\n' + pattern_type + '\n' + test_type + '\n'
    generated += function(motor, 'clampCommand')
    generated += function(motor, 'motorCommandOutSynth')
    generated += function(setup, 'setup_pid')
    reference = ''
    if args.baseline:
        old_motor = (args.baseline/'Core/Src/motor.c').read_text(encoding='utf-8-sig')
        old_setup = (args.baseline/'Core/Src/setup.c').read_text(encoding='utf-8-sig')
        reference = '#define HAVE_BASELINE 1\n' + function(old_motor, 'motorCommandOutSynth')
        for name in ['setup_pid_trace', 'setup_pid_traceOmegaFB', 'setup_pid_speed']:
            reference += function(old_setup, name)
        for name in ['motorCommandOutSynth', 'setup_pid_traceOmegaFB', 'setup_pid_trace', 'setup_pid_speed']:
            reference = re.sub(r'\b' + name + r'\b', 'before_' + name, reference)
    (output/'types.inc').write_text(generated[:generated.index('static int16_t clampCommand')], encoding='utf-8')
    (output/'production.inc').write_text(generated[generated.index('static int16_t clampCommand'):]+reference, encoding='utf-8')
    for optimization in ['-O0', '-O2']:
        exe = output/f'pid{optimization[2:]}.exe'
        run(['gcc','-std=c11',optimization,'-Wall','-Wextra','-Werror','-I',output,
             HERE/'pid_motor.c','-lm','-o',exe], output/f'pid{optimization}-compile.log')
        print(run([exe],output/f'pid{optimization}.log').strip())
    control = (SRC/'control.c').read_text(encoding='utf-8-sig')
    control_h = (INC/'control.h').read_text(encoding='utf-8-sig')
    course_h = (INC/'courseAnalysis.h').read_text(encoding='utf-8-sig')
    constants = '\n'.join(line for line in (control_h+'\n'+course_h).splitlines()
                          if re.match(r'#define (PARAM_[A-Z0-9_]+|MACHINEACCELE|MACHINEDECREACE|FILENAME_TARGET_SPEED)\b',line))
    settings = constants+'\n'+re.search(r'speedParam tgtParam = \{.*?\};',control,re.S).group()+'\n'
    names = ['applyTargetSpeedStoredValue', 'writeTgtspeeds', 'readTgtspeeds']
    settings += ''.join(function(control,name) for name in names)
    if args.baseline:
        old_control = (args.baseline/'Core/Src/control.c').read_text(encoding='utf-8-sig')
        reference = '#define HAVE_BASELINE 1\n'+re.search(r'speedParam tgtParam = \{.*?\};',old_control,re.S).group()+'\n'
        reference += ''.join(function(old_control,name) for name in names)
        for name in names+['tgtParam']:
            reference = re.sub(r'\b'+name+r'\b','before_'+name,reference)
        # 旧writerのsizeofとint16_t比較だけを参照実装内で許容し、現行はWerrorを維持する。
        settings += '#pragma GCC diagnostic push\n#pragma GCC diagnostic ignored "-Wsign-compare"\n'+reference+'#pragma GCC diagnostic pop\n'
    (output/'settings.inc').write_text(settings,encoding='utf-8')
    for optimization in ['-O0','-O2']:
        exe = output/f'settings{optimization[2:]}.exe'
        run(['gcc','-std=c11',optimization,'-Wall','-Wextra','-Werror','-I',output,'-I',INC,
             HERE/'speed_settings.c','-lm','-o',exe],output/f'settings{optimization}-compile.log')
        print(run([exe],output/f'settings{optimization}.log').strip())
    path = (SRC/'pathFollower.c').read_text(encoding='utf-8-sig')
    (output/'path_metadata.inc').write_text(function(path,'pathReadSourceMetadata'),encoding='utf-8')
    for optimization in ['-O0','-O2']:
        exe = output/f'metadata{optimization[2:]}.exe'
        run(['gcc','-std=c11',optimization,'-Wall','-Wextra','-Werror','-I',INC,'-I',output,
             HERE/'metadata.c',SRC/'logMetadata.c','-lm','-o',exe],output/f'metadata{optimization}-compile.log')
        print(run([exe],output/f'metadata{optimization}.log').strip())


if __name__=='__main__':
    main()
