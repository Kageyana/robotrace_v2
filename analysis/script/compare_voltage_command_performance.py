import csv
import math
from collections import defaultdict
from pathlib import Path
from statistics import mean, median


LOG_DIR = Path(r"F:\Dropbox\Document\robotrace\Log\v2")
OUT_DIR = Path(r"D:\robotrace\robotrace_v2\analysis")
PULSE_METER = 53424.0

COHORTS = {
    "duty_standard": {
        "label": "DUTY command / standard speed plan",
        "min_log": 10800,
        "max_log": 10991,
        "has_voltage_cols": False,
        "bstStraight": "3.30",
        "bst1500": "3.20",
        "bst1300": "3.00",
    },
    "voltage_standard": {
        "label": "Voltage command / standard speed plan",
        "min_log": 11030,
        "max_log": 11177,
        "has_voltage_cols": True,
        "bstStraight": "3.30",
        "bst1500": "3.00",
        "bst1300": "2.90",
    },
    "duty_high_transition": {
        "label": "DUTY command / high speed transition",
        "min_log": 10992,
        "max_log": 11016,
        "has_voltage_cols": False,
        "bstStraight": "4.00",
        "bst1500": "3.50",
        "bst1300": "3.00",
    },
    "voltage_high_transition": {
        "label": "Voltage command / high speed transition",
        "min_log": 11017,
        "max_log": 11027,
        "has_voltage_cols": True,
        "bstStraight": "4.00",
        "bst1500": "3.50",
        "bst1300": "3.00",
    },
}

REQUIRED_BASE = [
    "cntlog",
    "encCurrentN",
    "gyroVal_Z",
    "courseMarker",
    "encTotalOptimal",
    "ROC",
    "targetSpeed",
    "optimalIndex",
    "slipFlag",
    "slipFlagLat",
    "lineTraceCtrl",
    "targetAngularvelo",
    "motorpwmL",
    "motorpwmR",
]

REQUIRED_VOLTAGE = ["batteryVoltage_mV", "motorVoltageCmdL_mV", "motorVoltageCmdR_mV"]


def parse_float(value, default=math.nan):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def parse_header(row):
    columns = []
    params = {}
    for cell in row:
        if not cell:
            continue
        if "=" in cell:
            key, value = cell.split("=", 1)
            params[key] = value
        else:
            columns.append(cell)
    return columns, params


def percentile(values, ratio):
    if not values:
        return math.nan
    ordered = sorted(values)
    index = int(ratio * (len(ordered) - 1))
    return ordered[index]


def linear_slope(xs, ys):
    points = [(x, y) for x, y in zip(xs, ys) if not math.isnan(x) and not math.isnan(y)]
    if len(points) < 2:
        return math.nan
    x_mean = mean(x for x, _ in points)
    y_mean = mean(y for _, y in points)
    denom = sum((x - x_mean) ** 2 for x, _ in points)
    if denom == 0.0:
        return math.nan
    return sum((x - x_mean) * (y - y_mean) for x, y in points) / denom


def linear_r2(xs, ys):
    points = [(x, y) for x, y in zip(xs, ys) if not math.isnan(x) and not math.isnan(y)]
    if len(points) < 2:
        return math.nan
    slope = linear_slope([x for x, _ in points], [y for _, y in points])
    if math.isnan(slope):
        return math.nan
    x_mean = mean(x for x, _ in points)
    y_mean = mean(y for _, y in points)
    intercept = y_mean - slope * x_mean
    ss_tot = sum((y - y_mean) ** 2 for _, y in points)
    if ss_tot == 0.0:
        return math.nan
    ss_res = sum((y - (slope * x + intercept)) ** 2 for x, y in points)
    return 1.0 - ss_res / ss_tot


def read_log(number):
    path = LOG_DIR / f"{number}.csv"
    if not path.exists():
        return None

    with path.open("r", encoding="utf-8-sig", newline="") as fp:
        reader = csv.reader(fp)
        header = next(reader)
        columns, params = parse_header(header)
        rows = []
        for row in reader:
            if not row or all(cell == "" for cell in row):
                continue
            values = row[: len(columns)]
            if len(values) == len(columns):
                rows.append(dict(zip(columns, values)))

    return path, columns, params, rows


def matches_cohort(columns, params, cohort):
    has_voltage = all(column in columns for column in REQUIRED_VOLTAGE)
    if has_voltage != cohort["has_voltage_cols"]:
        return False
    return (
        params.get("tgtParam.bstStraight") == cohort["bstStraight"]
        and params.get("tgtParam.bst1500") == cohort["bst1500"]
        and params.get("tgtParam.bst1300") == cohort["bst1300"]
        and params.get("speedFeedForwardGain") == "150.00"
    )


def summarize_log(number, cohort_name, cohort):
    loaded = read_log(number)
    if loaded is None:
        return None

    path, columns, params, rows = loaded
    if not matches_cohort(columns, params, cohort):
        return None

    reasons = []
    missing = [column for column in REQUIRED_BASE if column not in columns]
    if missing:
        reasons.append("missing_cols=" + "|".join(missing))

    cntlog = [int(parse_float(row.get("cntlog"), -1)) for row in rows]
    steps = [b - a for a, b in zip(cntlog, cntlog[1:])]
    nonmonotonic = sum(1 for step in steps if step <= 0)
    if nonmonotonic:
        reasons.append(f"cntlog_nonmonotonic={nonmonotonic}")

    emc_stop = parse_float(params.get("emcStop"))
    if not math.isnan(emc_stop) and emc_stop != 0.0:
        reasons.append(f"emcStop={emc_stop:g}")

    target_mps = [parse_float(row.get("targetSpeed")) / PULSE_METER * 1000.0 for row in rows]
    speed_mps = [parse_float(row.get("encCurrentN")) / PULSE_METER * 1000.0 for row in rows]
    speed_err = [
        abs(target - speed)
        for target, speed in zip(target_mps, speed_mps)
        if not math.isnan(target) and not math.isnan(speed)
    ]

    battery_log = []
    if all(column in columns for column in REQUIRED_VOLTAGE):
        battery_log = [
            parse_float(row.get("batteryVoltage_mV")) / 1000.0
            for row in rows
            if not math.isnan(parse_float(row.get("batteryVoltage_mV")))
        ]
    battery_header = parse_float(params.get("batteryVoltage_V"))
    battery_for_compare = mean(battery_log) if battery_log else battery_header

    pwm_values = []
    duty_errors = []
    saturation_rows = 0
    if all(column in columns for column in REQUIRED_VOLTAGE):
        for row in rows:
            battery_mv = parse_float(row.get("batteryVoltage_mV"))
            saturated = False
            for pwm_key, cmd_key in (
                ("motorpwmL", "motorVoltageCmdL_mV"),
                ("motorpwmR", "motorVoltageCmdR_mV"),
            ):
                pwm = parse_float(row.get(pwm_key))
                cmd = parse_float(row.get(cmd_key))
                if not math.isnan(pwm):
                    pwm_values.append(pwm)
                    saturated |= abs(pwm) >= 1000.0
                if battery_mv > 1000.0 and not math.isnan(pwm) and not math.isnan(cmd) and abs(cmd) > 1.0:
                    predicted = max(-1000.0, min(1000.0, cmd / battery_mv * 1000.0))
                    duty_errors.append(abs(pwm - predicted))
            if saturated:
                saturation_rows += 1
    else:
        pwm_values = [
            parse_float(row.get(key))
            for row in rows
            for key in ("motorpwmL", "motorpwmR")
            if not math.isnan(parse_float(row.get(key)))
        ]

    slip_count = sum(1 for row in rows if int(parse_float(row.get("slipFlag"), 0)) != 0)
    slip_lat_count = sum(1 for row in rows if int(parse_float(row.get("slipFlagLat"), 0)) != 0)

    return {
        "cohort": cohort_name,
        "label": cohort["label"],
        "log": number,
        "valid": len(reasons) == 0,
        "reason": ";".join(reasons),
        "cols": len(columns),
        "rows": len(rows),
        "file_size_B": path.stat().st_size,
        "last_cnt_ms": cntlog[-1] if cntlog else math.nan,
        "last_encTotalOptimal": int(parse_float(rows[-1].get("encTotalOptimal"), 0)) if rows else 0,
        "optimalTrace": params.get("optimalTrace", ""),
        "autoStart": params.get("autoStart", ""),
        "emcStop": params.get("emcStop", ""),
        "batteryHeader_V": battery_header,
        "batteryLog_min_V": min(battery_log) if battery_log else math.nan,
        "batteryLog_mean_V": mean(battery_log) if battery_log else math.nan,
        "batteryLog_max_V": max(battery_log) if battery_log else math.nan,
        "batteryCompare_V": battery_for_compare,
        "max_target_mps": max(target_mps) if target_mps else math.nan,
        "max_speed_mps": max(speed_mps) if speed_mps else math.nan,
        "mean_abs_speed_err_mps": mean(speed_err) if speed_err else math.nan,
        "p95_abs_speed_err_mps": percentile(speed_err, 0.95),
        "slip_count": slip_count,
        "slipLat_count": slip_lat_count,
        "pwm_sat_rows": saturation_rows,
        "max_abs_pwm": max([abs(value) for value in pwm_values] or [math.nan]),
        "median_duty_err": median(duty_errors) if duty_errors else math.nan,
        "max_duty_err": max(duty_errors) if duty_errors else math.nan,
        "cntlog_median_step_ms": median(steps) if steps else math.nan,
        "cntlog_max_step_ms": max(steps) if steps else math.nan,
        "lineTraceOmegaFBCtrl_kp": params.get("lineTraceOmegaFBCtrl.kp", ""),
        "lineTraceOmegaFBCtrl_kd": params.get("lineTraceOmegaFBCtrl.kd", ""),
        "branch": params.get("branch", ""),
        "gitCommit": params.get("gitCommit", ""),
        "buildTime": params.get("buildTime", ""),
    }


def mark_incomplete(rows):
    grouped = defaultdict(list)
    for row in rows:
        grouped[(row["cohort"], row["autoStart"])].append(row)

    for group in grouped.values():
        valid_distances = [
            row["last_encTotalOptimal"]
            for row in group
            if row["valid"] and row["last_encTotalOptimal"] > 0
        ]
        if not valid_distances:
            continue
        reference = median(valid_distances)
        for row in group:
            if row["valid"] and row["last_encTotalOptimal"] < 0.9 * reference:
                row["valid"] = False
                reason = f"incomplete_distance={row['last_encTotalOptimal']}_lt_90pct_ref_{int(reference)}"
                row["reason"] = reason if not row["reason"] else row["reason"] + ";" + reason


def write_csv(path, rows):
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def cohort_auto_summary(rows):
    result = []
    grouped = defaultdict(list)
    for row in rows:
        if row["valid"]:
            grouped[(row["cohort"], row["autoStart"])].append(row)

    for (cohort_name, auto_start), group in sorted(grouped.items(), key=lambda item: (item[0][0], float(item[0][1]))):
        times = [row["last_cnt_ms"] for row in group]
        batteries = [row["batteryCompare_V"] for row in group]
        result.append(
            {
                "cohort": cohort_name,
                "label": group[0]["label"],
                "autoStart": auto_start,
                "count": len(group),
                "time_mean_ms": mean(times),
                "time_min_ms": min(times),
                "time_max_ms": max(times),
                "time_std_ms": math.sqrt(mean([(value - mean(times)) ** 2 for value in times])),
                "first_log": group[0]["log"],
                "first_time_ms": group[0]["last_cnt_ms"],
                "last_log": group[-1]["log"],
                "last_time_ms": group[-1]["last_cnt_ms"],
                "first_to_last_delta_ms": group[-1]["last_cnt_ms"] - group[0]["last_cnt_ms"],
                "battery_mean_V": mean(batteries),
                "battery_min_V": min(batteries),
                "battery_max_V": max(batteries),
                "time_slope_ms_per_V": linear_slope(batteries, times),
                "time_battery_r2": linear_r2(batteries, times),
                "p95_speed_err_mean_mps": mean(row["p95_abs_speed_err_mps"] for row in group),
                "mean_speed_err_mean_mps": mean(row["mean_abs_speed_err_mps"] for row in group),
                "pwm_sat_rows_sum": sum(row["pwm_sat_rows"] for row in group),
                "slip_sum": sum(row["slip_count"] for row in group),
                "slipLat_sum": sum(row["slipLat_count"] for row in group),
            }
        )
    return result


def make_sets(rows):
    sets = []
    for cohort_name in COHORTS:
        cohort_rows = [row for row in rows if row["cohort"] == cohort_name and row["valid"]]
        current = []
        for row in cohort_rows:
            auto = int(parse_float(row["autoStart"], 0))
            if auto == 1:
                if current:
                    sets.append(current)
                current = [row]
            elif current:
                current.append(row)
                if auto == 5:
                    sets.append(current)
                    current = []
        if current:
            sets.append(current)

    result = []
    for group in sets:
        autos = [int(parse_float(row["autoStart"], 0)) for row in group]
        if autos != [1, 2, 3, 4, 5]:
            continue
        boosted = [row for row in group if parse_float(row["optimalTrace"]) != 0.0]
        if len(boosted) != 4:
            continue
        result.append(
            {
                "cohort": group[0]["cohort"],
                "label": group[0]["label"],
                "logs": f"{group[0]['log']}-{group[-1]['log']}",
                "first_log": group[0]["log"],
                "last_log": group[-1]["log"],
                "auto1_time_ms": group[0]["last_cnt_ms"],
                "boost_mean_time_ms": mean(row["last_cnt_ms"] for row in boosted),
                "boost_min_time_ms": min(row["last_cnt_ms"] for row in boosted),
                "boost_max_time_ms": max(row["last_cnt_ms"] for row in boosted),
                "boost_battery_mean_V": mean(row["batteryCompare_V"] for row in boosted),
                "boost_p95_speed_err_mean_mps": mean(row["p95_abs_speed_err_mps"] for row in boosted),
                "boost_pwm_sat_rows_sum": sum(row["pwm_sat_rows"] for row in boosted),
                "boost_slip_sum": sum(row["slip_count"] for row in boosted),
                "boost_slipLat_sum": sum(row["slipLat_count"] for row in boosted),
            }
        )
    return result


def cohort_set_summary(sets):
    result = []
    grouped = defaultdict(list)
    for row in sets:
        grouped[row["cohort"]].append(row)
    for cohort_name, group in sorted(grouped.items()):
        times = [row["boost_mean_time_ms"] for row in group]
        batteries = [row["boost_battery_mean_V"] for row in group]
        result.append(
            {
                "cohort": cohort_name,
                "label": group[0]["label"],
                "set_count": len(group),
                "first_set": group[0]["logs"],
                "last_set": group[-1]["logs"],
                "boost_mean_time_ms": mean(times),
                "boost_min_set_mean_time_ms": min(times),
                "boost_max_set_mean_time_ms": max(times),
                "boost_time_std_ms": math.sqrt(mean([(value - mean(times)) ** 2 for value in times])),
                "first_to_last_boost_delta_ms": group[-1]["boost_mean_time_ms"] - group[0]["boost_mean_time_ms"],
                "battery_mean_V": mean(batteries),
                "battery_min_V": min(batteries),
                "battery_max_V": max(batteries),
                "set_time_slope_ms_per_V": linear_slope(batteries, times),
                "set_time_battery_r2": linear_r2(batteries, times),
                "boost_p95_speed_err_mean_mps": mean(row["boost_p95_speed_err_mean_mps"] for row in group),
                "boost_pwm_sat_rows_sum": sum(row["boost_pwm_sat_rows_sum"] for row in group),
                "boost_slip_sum": sum(row["boost_slip_sum"] for row in group),
                "boost_slipLat_sum": sum(row["boost_slipLat_sum"] for row in group),
            }
        )
    return result


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    rows = []
    for cohort_name, cohort in COHORTS.items():
        for number in range(cohort["min_log"], cohort["max_log"] + 1):
            row = summarize_log(number, cohort_name, cohort)
            if row is not None:
                rows.append(row)

    mark_incomplete(rows)
    auto_rows = cohort_auto_summary(rows)
    set_rows = make_sets(rows)
    set_summary_rows = cohort_set_summary(set_rows)

    summary_path = OUT_DIR / "voltage_command_performance_logs.csv"
    auto_path = OUT_DIR / "voltage_command_performance_by_autostart.csv"
    sets_path = OUT_DIR / "voltage_command_performance_sets.csv"
    set_summary_path = OUT_DIR / "voltage_command_performance_set_summary.csv"

    write_csv(summary_path, rows)
    write_csv(auto_path, auto_rows)
    write_csv(sets_path, set_rows)
    write_csv(set_summary_path, set_summary_rows)

    print(f"logs_csv={summary_path}")
    print(f"auto_csv={auto_path}")
    print(f"sets_csv={sets_path}")
    print(f"set_summary_csv={set_summary_path}")
    print("invalid")
    for row in rows:
        if not row["valid"]:
            print(f"{row['cohort']} {row['log']} {row['reason']}")
    print("set_summary")
    for row in set_summary_rows:
        print(
            f"{row['cohort']} sets={row['set_count']} mean={row['boost_mean_time_ms']:.1f}ms "
            f"std={row['boost_time_std_ms']:.1f}ms delta={row['first_to_last_boost_delta_ms']:.1f}ms "
            f"batt={row['battery_mean_V']:.3f}V range={row['battery_min_V']:.3f}-{row['battery_max_V']:.3f}V "
            f"slope={row['set_time_slope_ms_per_V']:.1f}ms/V r2={row['set_time_battery_r2']:.3f} "
            f"p95err={row['boost_p95_speed_err_mean_mps']:.3f} sat={row['boost_pwm_sat_rows_sum']}"
        )
    print("auto_summary")
    for row in auto_rows:
        if row["autoStart"] == "1.00" or row["cohort"].endswith("standard"):
            print(
                f"{row['cohort']} auto={row['autoStart']} n={row['count']} mean={row['time_mean_ms']:.1f}ms "
                f"delta={row['first_to_last_delta_ms']:.1f}ms batt={row['battery_mean_V']:.3f}V "
                f"slope={row['time_slope_ms_per_V']:.1f}ms/V p95err={row['p95_speed_err_mean_mps']:.3f} "
                f"sat={row['pwm_sat_rows_sum']}"
            )


if __name__ == "__main__":
    main()
