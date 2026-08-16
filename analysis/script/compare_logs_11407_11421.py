"""Summarize the latest voltage-command logs using the 7.3 V tuning threshold."""

from pathlib import Path
import sys


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

import compare_logs_11177_11272 as base


LOG_START = 11407
LOG_END = 11421
BATTERY_MIN_V = 7.3
OUT_DIR = Path(r"D:\robotrace\robotrace_v2\analysis")


def summarize_log(number):
    row = base.summarize_log(number)
    if row["valid"] and row["batteryHeader_V"] < BATTERY_MIN_V:
        row["valid"] = False
        row["reason"] = f"batteryHeader_V<{BATTERY_MIN_V:g}"
    return row


def write_csv(path, rows):
    base.write_csv(path, rows)


def main():
    rows = [summarize_log(number) for number in range(LOG_START, LOG_END + 1)]
    group_rows = base.grouped_summary(rows)
    set_rows = base.set_summary(rows)
    build_rows = base.build_summary(rows, set_rows)

    summary_path = OUT_DIR / "log_11407_11421_voltage_command_check_summary.csv"
    auto_path = OUT_DIR / "log_11407_11421_voltage_command_check_by_autostart.csv"
    set_path = OUT_DIR / "log_11407_11421_voltage_command_check_sets.csv"
    build_path = OUT_DIR / "log_11407_11421_voltage_command_check_by_build.csv"

    write_csv(summary_path, rows)
    write_csv(auto_path, group_rows)
    write_csv(set_path, set_rows)
    write_csv(build_path, build_rows)

    print(f"summary_csv={summary_path}")
    print(f"autostart_csv={auto_path}")
    print(f"sets_csv={set_path}")
    print(f"build_csv={build_path}")
    print(f"battery_min_V={BATTERY_MIN_V}")
    print(f"valid={sum(row['valid'] for row in rows)}/{len(rows)}")
    for row in rows:
        if not row["valid"]:
            print(f"invalid {row['log']} {row['reason']}")

    print("sets")
    for row in set_rows:
        print(
            f"set={row['set']} logs={row['logs']} auto1={row['time_auto1_ms']}ms "
            f"boostMean={row['boost_time_mean_ms']:.1f}ms "
            f"battery={row['boost_battery_mean_V']:.3f}V "
            f"p95err={row['boost_p95_speed_err_mean_mps']:.3f} "
            f"pwmSat={row['boost_pwm_sat_rows_sum']} "
            f"slip={row['boost_slip_sum']} slipLat={row['boost_slipLat_sum']} "
            f"maxCmd={row['boost_max_abs_cmd_mV']:.0f}"
        )


if __name__ == "__main__":
    main()
