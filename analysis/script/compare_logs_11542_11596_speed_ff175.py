"""Compare speed PID P=12 with speed feedforward 150 and 175."""

import csv
import math
import statistics
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

import check_logs_11407_metrics as metrics
import compare_logs_11177_11272 as base


LOG_RANGES = {
    "kp12_ff150": range(11542, 11567),
    "kp12_ff175": range(11567, 11597),
}
EXPECTED_FF = {"kp12_ff150": 150.0, "kp12_ff175": 175.0}
BATTERY_START_MIN_V = 7.3
OUT_DIR = Path(r"D:\robotrace\robotrace_v2\analysis")
PULSE_METER = 53424.0


def read_params(number):
    path = Path(r"F:\Dropbox\Document\robotrace\Log\v2") / f"{number}.csv"
    with path.open("r", encoding="utf-8-sig", newline="") as fp:
        header = next(csv.reader(fp))
    return {
        key: value
        for cell in header
        if "=" in cell
        for key, value in [cell.split("=", 1)]
    }


def summarize(number, condition):
    row = base.summarize_log(number)
    params = read_params(number)
    reasons = []
    if row["batteryHeader_V"] < BATTERY_START_MIN_V:
        reasons.append(f"batteryHeader_V<{BATTERY_START_MIN_V:g}")
    if float(params.get("veloCtrl.kp", "nan")) != 12.0:
        reasons.append(f"veloCtrl.kp={params.get('veloCtrl.kp', '')}")
    if float(params.get("speedFeedForwardGain", "nan")) != EXPECTED_FF[condition]:
        reasons.append(
            f"speedFeedForwardGain={params.get('speedFeedForwardGain', '')}"
        )
    if reasons:
        row["valid"] = False
        row["reason"] = ";".join(reasons)
    row["condition"] = condition
    row["speedFeedForwardGain"] = params.get("speedFeedForwardGain", "")
    row["veloCtrl.kp"] = params.get("veloCtrl.kp", "")
    return row


def write_csv(path, rows):
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def band_rows(condition, log_numbers, summary_by_log):
    bands = [("<120", -math.inf, 120), ("120-159", 120, 160),
             ("160-179", 160, 180), ("180-189", 180, 190),
             (">=190", 190, math.inf)]
    output = []
    for band, lower, upper in bands:
        errors = []
        saturated = 0
        rows_count = 0
        for number in log_numbers:
            summary = summary_by_log[number]
            if not summary["valid"] or float(summary["optimalTrace"]) == 0.0:
                continue
            params, rows = metrics.load(number)
            del params
            for row in rows:
                target = float(row["targetSpeed"])
                if not lower <= target < upper:
                    continue
                errors.append(
                    (target - float(row["encCurrentN"])) / PULSE_METER * 1000.0
                )
                rows_count += 1
                saturated += int(
                    max(abs(float(row["motorpwmL"])), abs(float(row["motorpwmR"])))
                    >= 1000.0
                )
        ordered = sorted(abs(value) for value in errors)
        p95 = ordered[int(0.95 * (len(ordered) - 1))] if ordered else math.nan
        output.append(
            {
                "condition": condition,
                "target_band": band,
                "rows": rows_count,
                "target_minus_actual_mean_mps": statistics.mean(errors) if errors else math.nan,
                "rmse_mps": math.sqrt(statistics.mean(value * value for value in errors))
                if errors
                else math.nan,
                "abs_p95_mps": p95,
                "motor_pwm_sat_pct": 100.0 * saturated / rows_count
                if rows_count
                else math.nan,
            }
        )
    return output


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    summary_rows = []
    set_rows = []
    metric_rows = []
    bands = []
    for condition, log_numbers in LOG_RANGES.items():
        rows = [summarize(number, condition) for number in log_numbers]
        summary_rows.extend(rows)
        summary_by_log = {row["log"]: row for row in rows}
        sets = base.set_summary(rows)
        for row in sets:
            row["condition"] = condition
        set_rows.extend(sets)
        for number in log_numbers:
            if summary_by_log[number]["valid"]:
                row = metrics.metric_row(number)
                row["condition"] = condition
                metric_rows.append(row)
        bands.extend(band_rows(condition, log_numbers, summary_by_log))

    outputs = {
        "summary": OUT_DIR / "log_11542_11596_speed_ff175_summary.csv",
        "sets": OUT_DIR / "log_11542_11596_speed_ff175_sets.csv",
        "metrics": OUT_DIR / "log_11542_11596_speed_ff175_metrics.csv",
        "bands": OUT_DIR / "log_11542_11596_speed_ff175_by_target_band.csv",
    }
    write_csv(outputs["summary"], summary_rows)
    write_csv(outputs["sets"], set_rows)
    write_csv(outputs["metrics"], metric_rows)
    write_csv(outputs["bands"], bands)

    for name, path in outputs.items():
        print(f"{name}={path}")
    for condition, log_numbers in LOG_RANGES.items():
        valid = [row for row in summary_rows if row["condition"] == condition and row["valid"]]
        print(f"{condition}: valid={len(valid)}/{len(log_numbers)}")
        for row in summary_rows:
            if row["condition"] == condition and not row["valid"]:
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
