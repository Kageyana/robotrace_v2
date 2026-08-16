"""Compare FF=150 and FF=175 by autoStart run number, not set averages."""

import csv
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

import compare_logs_11542_11596_speed_ff175 as comparison
import compare_logs_11177_11272 as base


OUT_PATH = Path(
    r"D:\robotrace\robotrace_v2\analysis\log_11542_11596_speed_ff175_by_autostart.csv"
)


def main():
    rows = []
    for condition, log_numbers in comparison.LOG_RANGES.items():
        for number in log_numbers:
            summary = comparison.summarize(number, condition)
            if not summary["valid"] or float(summary["optimalTrace"]) == 0.0:
                continue
            metric = comparison.metrics.metric_row(number)
            rows.append(
                {
                    "condition": condition,
                    "set_log": number - (int(float(summary["autoStart"])) - 1),
                    "log": number,
                    "autoStart": int(float(summary["autoStart"])),
                    "start_voltage_V": summary["batteryHeader_V"],
                    "lap_ms": summary["last_cnt_ms"],
                    "speed_error_abs_p95_mps": metric["speedErr_abs_p95_mps"],
                    "speed_error_target_minus_actual_mean_mps": metric["speedErr_mean_mps"],
                    "lineTraceCtrl_rms": metric["lineTraceCtrl_rms"],
                    "gyroVal_Z_rms": metric["gyroVal_Z_rms"],
                    "motorDutySat_pct": metric["motorDutySat_pct"],
                    "slip_count": summary["slip_count"],
                    "slipLat_count": summary["slipLat_count"],
                }
            )

    rows.sort(key=lambda row: (row["autoStart"], row["condition"], row["log"]))
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    with OUT_PATH.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    print(OUT_PATH)
    for auto_start in (2, 3, 4, 5):
        print(f"autoStart={auto_start}")
        for condition in comparison.LOG_RANGES:
            selected = [
                row
                for row in rows
                if row["condition"] == condition and row["autoStart"] == auto_start
            ]
            print(
                f"  {condition}: "
                + ", ".join(
                    f"log{row['log']} lap={row['lap_ms']}ms "
                    f"p95={float(row['speed_error_abs_p95_mps']):.3f} "
                    f"slip={row['slip_count']}/{row['slipLat_count']}"
                    for row in selected
                )
            )


if __name__ == "__main__":
    main()
