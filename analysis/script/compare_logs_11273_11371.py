"""Compare robotrace logs 11273-11371 using the shared log summarizer."""

from pathlib import Path
import sys


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

import compare_logs_11177_11272 as base


base.LOG_START = 11273
base.LOG_END = 11371


def main():
    base.OUT_DIR.mkdir(parents=True, exist_ok=True)
    rows = [base.summarize_log(number) for number in range(base.LOG_START, base.LOG_END + 1)]
    group_rows = base.grouped_summary(rows)
    set_rows = base.set_summary(rows)
    build_rows = base.build_summary(rows, set_rows)

    prefix = "log_11273_11371_compare"
    paths = {
        "summary": base.OUT_DIR / f"{prefix}_summary.csv",
        "auto": base.OUT_DIR / f"{prefix}_by_build_autostart.csv",
        "sets": base.OUT_DIR / f"{prefix}_sets.csv",
        "build": base.OUT_DIR / f"{prefix}_by_build.csv",
        "svg": base.OUT_DIR / f"{prefix}.svg",
    }
    base.write_csv(paths["summary"], rows)
    base.write_csv(paths["auto"], group_rows)
    base.write_csv(paths["sets"], set_rows)
    base.write_csv(paths["build"], build_rows)
    base.write_svg(paths["svg"], rows, set_rows)

    print("outputs")
    for key, path in paths.items():
        print(f"{key}={path}")
    print(f"valid={sum(row['valid'] for row in rows)}/{len(rows)}")
    for row in rows:
        if not row["valid"]:
            print(f"invalid {row['log']} rows={row['rows']} reason={row['reason']}")

    print("build_summary")
    for row in build_rows:
        print(
            f"build={row['buildTime']} logs={row['log_count']} sets={row['set_count']} "
            f"auto1={row['auto1_mean_ms']:.1f}ms boostSet={row['boost_set_mean_ms']:.1f}ms "
            f"batt={row['boost_battery_mean_V']:.3f}V "
            f"p95err={row['boost_p95_speed_err_mean_mps']:.3f} "
            f"sat={row['boost_pwm_sat_rows_sum']} "
            f"slip={row['boost_slip_sum']} slipLat={row['boost_slipLat_sum']} "
            f"maxCmd={row['boost_max_abs_cmd_mV']:.0f}"
        )

    print("auto_summary")
    for row in group_rows:
        print(
            f"build={row['buildTime']} auto={row['autoStart']} n={row['count']} "
            f"mean={row['time_mean_ms']:.1f}ms min={row['time_min_ms']} max={row['time_max_ms']} "
            f"batt={row['battery_mean_V']:.3f}V "
            f"p95err={row['p95_speed_err_mean_mps']:.3f} "
            f"sat={row['pwm_sat_rows_sum']} slip={row['slip_sum']} "
            f"slipLat={row['slipLat_sum']} maxCmd={row['max_abs_cmd_mV_max']:.0f}"
        )

    print("set_summary")
    for row in set_rows:
        print(
            f"set={row['set']:02d} logs={row['logs']} build={row['buildTime']} "
            f"auto1={row['time_auto1_ms']}ms boostMean={row['boost_time_mean_ms']:.1f}ms "
            f"batt={row['boost_battery_mean_V']:.3f}V "
            f"sat={row['boost_pwm_sat_rows_sum']} "
            f"p95err={row['boost_p95_speed_err_mean_mps']:.3f} "
            f"slip={row['boost_slip_sum']} slipLat={row['boost_slipLat_sum']}"
        )


if __name__ == "__main__":
    main()
