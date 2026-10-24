"""実ファームのbin保存・型付き解析・現行V2を両ログ構成で検証する。

GCC/生成exeの既知の起動制約がある環境では、許可された制限外で実行する。
出力はリポジトリ内build/log-storage-validationだけへ保存する。
"""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import math
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PROJECT = ROOT / "robotrace_v2"


def run(command, log):
    result = subprocess.run([str(x) for x in command], stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace")
    log.write_text(result.stdout, encoding="utf-8")
    if result.returncode:
        print(result.stdout)
        raise RuntimeError(f"Failed ({result.returncode}): {command}; log={log}")
    return result.stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--converter", type=Path, help="任意のPC用RlogToCsv.exe")
    args = parser.parse_args()
    output = ROOT / "build/log-storage-validation"
    output.mkdir(parents=True, exist_ok=True)
    includes = ["Core/Inc", "FATFS/Target", "FATFS/App", "Drivers/STM32F4xx_HAL_Driver/Inc",
                "Drivers/STM32F4xx_HAL_Driver/Inc/Legacy", "Middlewares/Third_Party/FatFs/src",
                "Drivers/CMSIS/Device/ST/STM32F4xx/Include", "Drivers/CMSIS/Include"]
    flags = ["gcc", "-std=gnu11", "-w", "-ffunction-sections", "-fdata-sections",
             "-DSTM32F446xx", "-DUSE_HAL_DRIVER"]
    for directory in includes:
        flags += ["-I", str(PROJECT / directory)]
    results = []
    for profile in (1, 0):
        folder = output / f"profile{profile}"
        folder.mkdir(exist_ok=True)
        binary = folder / "check.exe"
        command = flags + [f"-DLOG_SCHEMA_PROFILE_LIGHT={profile}", HERE / "check.c"]
        command += [PROJECT / "Core/Src" / name for name in ("slipDetection.c", "runMemory.c", "autoRun.c")]
        command += ["-Wl,--gc-sections,--exclude-all-symbols", "-lm", "-o", binary]
        run(command, folder / "compile.log")
        text = run([binary, folder], folder / "check.log")
        curved = folder / "curved"
        curved.mkdir(exist_ok=True)
        run([binary, curved, "curved"], folder / "curved.log")
        item = {"profile": profile, "passed": True,
                "binary_sha256": hashlib.sha256((folder / "1.bin").read_bytes()).hexdigest()}
        if args.converter:
            converted = folder / "pc.csv"
            # 再実行で以前のCSVを成功成果物として扱わない。
            if converted.exists():
                converted.unlink()
            run([args.converter.resolve(), folder / "1.bin", converted], folder / "converter.log")
            with converted.open(encoding="utf-8", newline="") as stream:
                metadata = stream.readline()
                actual = list(csv.DictReader(stream))
            with (folder / "rows.csv").open(encoding="utf-8", newline="") as stream:
                expected = list(csv.DictReader(stream))
            assert len(actual) == len(expected) == 50
            for produced, source in zip(actual, expected):
                for key, value in source.items():
                    assert math.isclose(float(produced[key]), float(value), rel_tol=2e-6, abs_tol=1e-5), (key, produced[key], value)
            assert "binaryLogNumber=1," in metadata
            item["pc_conversion_rows"] = len(actual)
        results.append(item)
        print(f"PASS profile={profile}: {text.strip().splitlines()[-1]}")
    detector = output / "slip-detector.exe"
    run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", PROJECT / "Core/Inc",
         HERE / "slip_detector.c", PROJECT / "Core/Src/slipDetection.c", "-lm", "-o", detector], output / "slip-compile.log")
    print(run([detector], output / "slip.log").strip())
    policy = output / "auto-run.exe"
    run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", PROJECT / "Core/Inc",
         HERE / "auto_run.c", HERE / "path_policy.c", PROJECT / "Core/Src/autoRun.c",
         "-lm", "-o", policy], output / "auto-run-compile.log")
    print(run([policy], output / "auto-run.log").strip())
    (output / "validation.json").write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
