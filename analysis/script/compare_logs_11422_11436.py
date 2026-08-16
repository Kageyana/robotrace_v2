"""Analyze the latest voltage-command logs 11422-11436."""

import csv
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

import check_logs_11407_metrics as metrics
import compare_logs_11177_11272 as base


LOG_START = 11422
LOG_END = 11436
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
    rows = [summarize(n) for n in range(LOG_START, LOG_END + 1)]
    sets = base.set_summary(rows)
    summary_path = OUT_DIR / "log_11422_11436_voltage_command_check_summary.csv"
    sets_path = OUT_DIR / "log_11422_11436_voltage_command_check_sets.csv"
    metrics_path = OUT_DIR / "log_11422_11436_voltage_command_metrics.csv"
    write_csv(summary_path, rows)
    write_csv(sets_path, sets)

    metric_rows = [metrics.metric_row(n) for n in range(LOG_START, LOG_END + 1)]
    write_csv(metrics_path, metric_rows)

    print(f"valid_header_threshold={sum(row['valid'] for row in rows)}/{len(rows)}")
    for row in sets:
        print(
            f"set={row['set']} logs={row['logs']} "
            f"boostMean={row['boost_time_mean_ms']:.1f}ms "
            f"batteryMean={row['boost_battery_mean_V']:.3f}V "
            f"p95err={row['boost_p95_speed_err_mean_mps']:.3f} "
            f"pwmSat={row['boost_pwm_sat_rows_sum']} "
            f"slip={row['boost_slip_sum']} slipLat={row['boost_slipLat_sum']}"
        )
    print(summary_path)
    print(sets_path)
    print(metrics_path)


if __name__ == "__main__":
    main()
