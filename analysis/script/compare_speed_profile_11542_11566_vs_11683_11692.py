"""Compare speed-profile behavior per autoStart number for the P12/FF150 baseline."""

import csv
import math
import sys
from collections import defaultdict
from pathlib import Path
from statistics import mean


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

import compare_logs_11177_11272 as base
import check_logs_11407_metrics as metrics


LOG_DIR = base.LOG_DIR
OUT_DIR = Path(r"D:\robotrace\robotrace_v2\analysis")
PULSE_METER = 53424.0
BATTERY_START_MIN_V = 7.3

CONDITIONS = {
    "baseline_p12_ff150": [
        range(11542, 11547),
        range(11547, 11552),
        range(11552, 11557),
        range(11557, 11562),
        range(11562, 11567),
    ],
    "latest_p12_ff150": [range(11683, 11688), range(11688, 11693)],
}


def parse_float(value, default=math.nan):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def load(number):
    with (LOG_DIR / f"{number}.csv").open("r", encoding="utf-8-sig", newline="") as fp:
        reader = csv.reader(fp)
        columns, params = base.parse_header(next(reader))
        rows = [dict(zip(columns, row[: len(columns)])) for row in reader if len(row) >= len(columns)]
    return params, rows


def valid_summary(number):
    summary = base.summarize_log(number)
    if summary["batteryHeader_V"] < BATTERY_START_MIN_V:
        summary["valid"] = False
        summary["reason"] = f"batteryHeader_V<{BATTERY_START_MIN_V:g}"
    return summary


def band(target_pulse):
    if target_pulse < 120:
        return "<120"
    if target_pulse < 160:
        return "120-159"
    if target_pulse < 180:
        return "160-179"
    return ">=180"


def per_log_row(condition, set_id, number):
    summary = valid_summary(number)
    metric = metrics.metric_row(number)
    params, rows = load(number)
    bands = defaultdict(list)
    for item in rows:
        target = parse_float(item.get("targetSpeed"))
        actual = parse_float(item.get("encCurrentN"))
        if math.isnan(target) or math.isnan(actual):
            continue
        bands[band(target)].append((target - actual) / PULSE_METER * 1000.0)

    return {
        "condition": condition,
        "set": set_id,
        "log": number,
        "autoStart": int(float(summary["autoStart"])),
        "valid": summary["valid"],
        "reason": summary["reason"],
        "start_voltage_V": summary["batteryHeader_V"],
        "lap_ms": summary["last_cnt_ms"],
        "max_target_pulse_ms": summary["max_target_mps"] * PULSE_METER / 1000.0,
        "max_actual_pulse_ms": summary["max_speed_mps"] * PULSE_METER / 1000.0,
        "speed_error_signed_mean_mps": metric["speedErr_mean_mps"],
        "speed_error_abs_p95_mps": metric["speedErr_abs_p95_mps"],
        "slip_count": summary["slip_count"],
        "slipLat_count": summary["slipLat_count"],
        "lineTraceCtrl_rms": metric["lineTraceCtrl_rms"],
        "gyroVal_Z_rms": metric["gyroVal_Z_rms"],
        "target_lt120_signed_err_mps": mean(bands["<120"]) if bands["<120"] else math.nan,
        "target_120_159_signed_err_mps": mean(bands["120-159"]) if bands["120-159"] else math.nan,
        "target_160_179_signed_err_mps": mean(bands["160-179"]) if bands["160-179"] else math.nan,
        "target_ge180_signed_err_mps": mean(bands[">=180"]) if bands[">=180"] else math.nan,
    }


def write_csv(path, rows):
    with path.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    output = []
    for condition, groups in CONDITIONS.items():
        for set_id, group in enumerate(groups, 1):
            for number in group:
                output.append(per_log_row(condition, set_id, number))

    output_path = OUT_DIR / "log_11542_11692_speed_profile_by_autostart.csv"
    write_csv(output_path, output)
    print(f"output={output_path}")

    for condition in CONDITIONS:
        print(condition)
        for auto_start in (2, 3, 4, 5):
            rows = [row for row in output if row["condition"] == condition and row["autoStart"] == auto_start and row["valid"]]
            for row in rows:
                print(
                    f"  auto={auto_start} set={row['set']} log={row['log']} "
                    f"lap={row['lap_ms']}ms startV={row['start_voltage_V']:.2f} "
                    f"signedErr={row['speed_error_signed_mean_mps']:.3f} "
                    f"p95={row['speed_error_abs_p95_mps']:.3f} "
                    f"slipLat={row['slipLat_count']} "
                    f"band120_159={row['target_120_159_signed_err_mps']:.3f} "
                    f"band160_179={row['target_160_179_signed_err_mps']:.3f} "
                    f"band180={row['target_ge180_signed_err_mps']:.3f}"
                )


if __name__ == "__main__":
    main()
