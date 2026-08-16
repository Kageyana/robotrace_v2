"""Check FF=175 logs by complete autoStart set and run number."""

import csv
import math
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

import compare_logs_11177_11272 as base
import check_logs_11407_metrics as metrics


LOG_START = 11597
LOG_END = 11640
BATTERY_START_MIN_V = 7.3
OUT_DIR = Path(r"D:\robotrace\robotrace_v2\analysis")


def summarize(number):
    row = base.summarize_log(number)
    if row["valid"] and row["batteryHeader_V"] < BATTERY_START_MIN_V:
        row["valid"] = False
        row["reason"] = f"batteryHeader_V<{BATTERY_START_MIN_V:g}"
    row["condition"] = "kp12_ff175"
    return row


def write_csv(path, rows):
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def split_auto_start_groups(rows):
    groups = []
    current = []
    for row in rows:
        if row["autoStart"] == "1.00" and current:
            groups.append(current)
            current = []
        current.append(row)
    if current:
        groups.append(current)
    return groups


def complete_sets(rows):
    complete = []
    excluded = []
    for group in split_auto_start_groups(rows):
        sequence = [int(float(row["autoStart"])) for row in group]
        if sequence == [1, 2, 3, 4, 5] and all(row["valid"] for row in group):
            complete.append(group)
        else:
            reasons = [row["reason"] for row in group if row["reason"]]
            excluded.append(
                {
                    "logs": f"{group[0]['log']}-{group[-1]['log']}",
                    "autoStart_sequence": ",".join(str(value) for value in sequence),
                    "reason": ";".join(reasons) or "incomplete_autoStart_set",
                }
            )
    return complete, excluded


def run_rows(complete):
    output = []
    for set_id, group in enumerate(complete, 1):
        for row in group:
            if int(float(row["autoStart"])) == 1:
                continue
            metric = metrics.metric_row(row["log"])
            output.append(
                {
                    "set": set_id,
                    "log": row["log"],
                    "autoStart": int(float(row["autoStart"])),
                    "start_voltage_V": row["batteryHeader_V"],
                    "lap_ms": row["last_cnt_ms"],
                    "speed_error_abs_p95_mps": metric["speedErr_abs_p95_mps"],
                    "speed_error_target_minus_actual_mean_mps": metric["speedErr_mean_mps"],
                    "lineTraceCtrl_rms": metric["lineTraceCtrl_rms"],
                    "gyroVal_Z_rms": metric["gyroVal_Z_rms"],
                    "motorDutySat_pct": metric["motorDutySat_pct"],
                    "slip_count": row["slip_count"],
                    "slipLat_count": row["slipLat_count"],
                }
            )
    return output


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    rows = [summarize(number) for number in range(LOG_START, LOG_END + 1)]
    complete, excluded = complete_sets(rows)
    per_run = run_rows(complete)

    summary_path = OUT_DIR / "log_11597_11640_speed_ff175_summary.csv"
    runs_path = OUT_DIR / "log_11597_11640_speed_ff175_by_autostart.csv"
    excluded_path = OUT_DIR / "log_11597_11640_speed_ff175_excluded_sets.csv"
    write_csv(summary_path, rows)
    write_csv(runs_path, per_run)
    write_csv(excluded_path, excluded)

    print(f"summary={summary_path}")
    print(f"by_autostart={runs_path}")
    print(f"excluded_sets={excluded_path}")
    print(f"complete_sets={len(complete)}")
    for item in excluded:
        print(f"excluded={item['logs']} sequence={item['autoStart_sequence']} reason={item['reason']}")
    for auto_start in (2, 3, 4, 5):
        print(f"autoStart={auto_start}")
        for row in per_run:
            if row["autoStart"] == auto_start:
                print(
                    f"  set={row['set']} log={row['log']} lap={row['lap_ms']}ms "
                    f"startV={float(row['start_voltage_V']):.2f} "
                    f"p95={float(row['speed_error_abs_p95_mps']):.3f} "
                    f"slip={row['slip_count']}/{row['slipLat_count']}"
                )


if __name__ == "__main__":
    main()
