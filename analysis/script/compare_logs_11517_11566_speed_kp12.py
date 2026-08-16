"""Compare speed-control P=10 and P=12 logs after the 7.2 V command change."""

import csv
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

import check_logs_11407_metrics as metrics
import compare_logs_11177_11272 as base


LOG_RANGES = {
    "speed_kp10": range(11517, 11537),
    "speed_kp12": range(11537, 11567),
}
BATTERY_START_MIN_V = 7.3
OUT_DIR = Path(r"D:\robotrace\robotrace_v2\analysis")


def summarize(number, label):
    row = base.summarize_log(number)
    row["condition"] = label
    if row["valid"] and row["batteryHeader_V"] < BATTERY_START_MIN_V:
        row["valid"] = False
        row["reason"] = f"batteryHeader_V<{BATTERY_START_MIN_V:g}"
    return row


def write_csv(path, rows):
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    summary_rows = []
    set_rows = []
    metric_rows = []
    for label, log_numbers in LOG_RANGES.items():
        rows = [summarize(number, label) for number in log_numbers]
        summary_rows.extend(rows)
        grouped_sets = base.set_summary(rows)
        for row in grouped_sets:
            row["condition"] = label
        set_rows.extend(grouped_sets)
        for number in log_numbers:
            row = metrics.metric_row(number)
            row["condition"] = label
            metric_rows.append(row)

    summary_path = OUT_DIR / "log_11517_11566_speed_kp12_comparison.csv"
    set_path = OUT_DIR / "log_11517_11566_speed_kp12_sets.csv"
    metric_path = OUT_DIR / "log_11517_11566_speed_kp12_metrics.csv"
    write_csv(summary_path, summary_rows)
    write_csv(set_path, set_rows)
    write_csv(metric_path, metric_rows)

    print(f"summary={summary_path}")
    print(f"sets={set_path}")
    print(f"metrics={metric_path}")
    for label, log_numbers in LOG_RANGES.items():
        rows = [row for row in summary_rows if row["condition"] == label]
        valid = [row for row in rows if row["valid"]]
        print(f"{label}: valid={len(valid)}/{len(rows)}")
        for row in rows:
            if not row["valid"]:
                print(f"  excluded={row['log']} reason={row['reason']}")
    for row in set_rows:
        print(
            f"{row['condition']} set={row['set']} logs={row['logs']} "
            f"lapMean={row['boost_time_mean_ms']:.1f}ms "
            f"startRunVoltageMean={row['boost_battery_mean_V']:.3f}V "
            f"p95SpeedErr={row['boost_p95_speed_err_mean_mps']:.3f}m/s "
            f"pwmSatRows={row['boost_pwm_sat_rows_sum']} "
            f"slip={row['boost_slip_sum']} slipLat={row['boost_slipLat_sum']}"
        )


if __name__ == "__main__":
    main()
