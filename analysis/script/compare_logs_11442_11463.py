"""Analyze voltage-command logs 11442-11463 with start-voltage filtering."""

import csv
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

import check_logs_11407_metrics as metrics
import compare_logs_11177_11272 as base


LOG_START = 11442
LOG_END = 11463
BATTERY_MIN_V = 7.3
OUT_DIR = Path(r"D:\robotrace\robotrace_v2\analysis")


def summarize(number):
    row = base.summarize_log(number)
    if row["valid"] and row["batteryHeader_V"] < BATTERY_MIN_V:
        row["valid"] = False
        row["reason"] = f"batteryHeader_V<{BATTERY_MIN_V:g}"
    return row


def write_csv(path, rows):
    with path.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main():
    rows = [summarize(number) for number in range(LOG_START, LOG_END + 1)]
    set_rows = base.set_summary(rows)
    metrics_rows = [metrics.metric_row(number) for number in range(LOG_START, LOG_END + 1)]

    outputs = {
        "summary": (OUT_DIR / "log_11442_11463_voltage_command_check_summary.csv", rows),
        "sets": (OUT_DIR / "log_11442_11463_voltage_command_check_sets.csv", set_rows),
        "metrics": (OUT_DIR / "log_11442_11463_voltage_command_metrics.csv", metrics_rows),
    }
    for path, data in outputs.values():
        write_csv(path, data)

    print(f"valid={sum(row['valid'] for row in rows)}/{len(rows)}")
    for row in rows:
        if not row["valid"]:
            print(f"excluded={row['log']} reason={row['reason']}")
    for row in set_rows:
        print(
            f"set={row['set']} logs={row['logs']} "
            f"boostMean={row['boost_time_mean_ms']:.1f}ms "
            f"runningMeanV={row['boost_battery_mean_V']:.3f}V "
            f"p95err={row['boost_p95_speed_err_mean_mps']:.3f} "
            f"pwmSat={row['boost_pwm_sat_rows_sum']} "
            f"slip={row['boost_slip_sum']} slipLat={row['boost_slipLat_sum']}"
        )
    for name, (path, _) in outputs.items():
        print(f"{name}={path}")


if __name__ == "__main__":
    main()
