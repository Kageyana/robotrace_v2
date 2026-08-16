"""Summarize voltage-command logs 11402-11421 using the 7.3 V threshold."""

from pathlib import Path
import sys


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

import compare_logs_11177_11272 as base


LOG_START = 11402
LOG_END = 11421
BATTERY_MIN_V = 7.3
OUT_DIR = Path(r"D:\robotrace\robotrace_v2\analysis")


def summarize_log(number):
    row = base.summarize_log(number)
    if row["valid"] and row["batteryHeader_V"] < BATTERY_MIN_V:
        row["valid"] = False
        row["reason"] = f"batteryHeader_V<{BATTERY_MIN_V:g}"
    return row


def main():
    rows = [summarize_log(number) for number in range(LOG_START, LOG_END + 1)]
    set_rows = base.set_summary(rows)
    output_rows = {
        "summary": rows,
        "by_autostart": base.grouped_summary(rows),
        "sets": set_rows,
        "by_build": base.build_summary(rows, set_rows),
    }
    output_paths = {
        "summary": OUT_DIR / "log_11402_11421_voltage_command_check_summary.csv",
        "by_autostart": OUT_DIR / "log_11402_11421_voltage_command_check_by_autostart.csv",
        "sets": OUT_DIR / "log_11402_11421_voltage_command_check_sets.csv",
        "by_build": OUT_DIR / "log_11402_11421_voltage_command_check_by_build.csv",
    }
    for key, path in output_paths.items():
        base.write_csv(path, output_rows[key])

    print(f"battery_min_V={BATTERY_MIN_V}")
    print(f"valid={sum(row['valid'] for row in rows)}/{len(rows)}")
    for row in set_rows:
        print(
            f"set={row['set']} logs={row['logs']} "
            f"boostMean={row['boost_time_mean_ms']:.1f}ms "
            f"battery={row['boost_battery_mean_V']:.3f}V "
            f"p95err={row['boost_p95_speed_err_mean_mps']:.3f} "
            f"pwmSat={row['boost_pwm_sat_rows_sum']} "
            f"slip={row['boost_slip_sum']} slipLat={row['boost_slipLat_sum']}"
        )
    for key, path in output_paths.items():
        print(f"{key}={path}")


if __name__ == "__main__":
    main()
