import csv
import math
from collections import defaultdict
from pathlib import Path
from statistics import mean, median


LOG_DIR = Path(r"F:\Dropbox\Document\robotrace\Log\v2")
OUT_DIR = Path(r"D:\robotrace\robotrace_v2\analysis")
LOG_START = 11177
LOG_END = 11272
PULSE_METER = 53424.0

REQUIRED_COLUMNS = [
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
    "batteryVoltage_mV",
    "motorVoltageCmdL_mV",
    "motorVoltageCmdR_mV",
    "encCurrentCorr_p",
    "x",
    "y",
]


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


def slope(xs, ys):
    points = [(x, y) for x, y in zip(xs, ys) if not math.isnan(x) and not math.isnan(y)]
    if len(points) < 2:
        return math.nan
    xm = mean(x for x, _ in points)
    ym = mean(y for _, y in points)
    denom = sum((x - xm) ** 2 for x, _ in points)
    if denom == 0.0:
        return math.nan
    return sum((x - xm) * (y - ym) for x, y in points) / denom


def summarize_log(number):
    path = LOG_DIR / f"{number}.csv"
    if not path.exists():
        return {"log": number, "valid": False, "reason": "missing"}

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

    reasons = []
    missing = [column for column in REQUIRED_COLUMNS if column not in columns]
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

    battery_v = [
        parse_float(row.get("batteryVoltage_mV")) / 1000.0
        for row in rows
        if not math.isnan(parse_float(row.get("batteryVoltage_mV")))
    ]
    target_mps = [parse_float(row.get("targetSpeed")) / PULSE_METER * 1000.0 for row in rows]
    speed_mps = [parse_float(row.get("encCurrentN")) / PULSE_METER * 1000.0 for row in rows]
    speed_err = [
        abs(target - speed)
        for target, speed in zip(target_mps, speed_mps)
        if not math.isnan(target) and not math.isnan(speed)
    ]

    saturation_rows = 0
    duty_errors = []
    pwm_values = []
    cmd_values = []
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
            if not math.isnan(cmd):
                cmd_values.append(cmd)
            if battery_mv > 1000.0 and not math.isnan(pwm) and not math.isnan(cmd) and abs(cmd) > 1.0:
                predicted = max(-1000.0, min(1000.0, cmd / battery_mv * 1000.0))
                duty_errors.append(abs(pwm - predicted))
        if saturated:
            saturation_rows += 1

    slip_count = sum(1 for row in rows if int(parse_float(row.get("slipFlag"), 0)) != 0)
    slip_lat_count = sum(1 for row in rows if int(parse_float(row.get("slipFlagLat"), 0)) != 0)

    return {
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
        "batteryHeader_V": parse_float(params.get("batteryVoltage_V")),
        "battery_min_V": min(battery_v) if battery_v else math.nan,
        "battery_mean_V": mean(battery_v) if battery_v else math.nan,
        "battery_max_V": max(battery_v) if battery_v else math.nan,
        "max_target_mps": max(target_mps) if target_mps else math.nan,
        "max_speed_mps": max(speed_mps) if speed_mps else math.nan,
        "mean_abs_speed_err_mps": mean(speed_err) if speed_err else math.nan,
        "p95_abs_speed_err_mps": percentile(speed_err, 0.95),
        "slip_count": slip_count,
        "slipLat_count": slip_lat_count,
        "pwm_sat_rows": saturation_rows,
        "max_abs_pwm": max([abs(value) for value in pwm_values] or [math.nan]),
        "max_abs_cmd_mV": max([abs(value) for value in cmd_values] or [math.nan]),
        "median_duty_err": median(duty_errors) if duty_errors else math.nan,
        "max_duty_err": max(duty_errors) if duty_errors else math.nan,
        "cntlog_median_step_ms": median(steps) if steps else math.nan,
        "cntlog_max_step_ms": max(steps) if steps else math.nan,
        "x_last_mm": parse_float(rows[-1].get("x")) if rows else math.nan,
        "y_last_mm": parse_float(rows[-1].get("y")) if rows else math.nan,
        "lineTraceOmegaFBCtrl_kp": params.get("lineTraceOmegaFBCtrl.kp", ""),
        "lineTraceOmegaFBCtrl_kd": params.get("lineTraceOmegaFBCtrl.kd", ""),
        "branch": params.get("branch", ""),
        "gitCommit": params.get("gitCommit", ""),
        "buildTime": params.get("buildTime", ""),
    }


def write_csv(path, rows):
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def grouped_summary(rows):
    groups = defaultdict(list)
    for row in rows:
        if row["valid"]:
            groups[(row["buildTime"], row["autoStart"])].append(row)

    result = []
    for (build_time, auto_start), group in sorted(groups.items(), key=lambda item: (item[0][0], float(item[0][1]))):
        times = [row["last_cnt_ms"] for row in group]
        batteries = [row["battery_mean_V"] for row in group]
        result.append(
            {
                "buildTime": build_time,
                "autoStart": auto_start,
                "count": len(group),
                "time_mean_ms": mean(times),
                "time_min_ms": min(times),
                "time_max_ms": max(times),
                "time_range_ms": max(times) - min(times),
                "first_log": group[0]["log"],
                "first_time_ms": group[0]["last_cnt_ms"],
                "last_log": group[-1]["log"],
                "last_time_ms": group[-1]["last_cnt_ms"],
                "first_to_last_delta_ms": group[-1]["last_cnt_ms"] - group[0]["last_cnt_ms"],
                "battery_mean_V": mean(batteries),
                "battery_min_mean_V": min(batteries),
                "battery_max_mean_V": max(batteries),
                "time_slope_ms_per_V": slope(batteries, times),
                "p95_speed_err_mean_mps": mean(row["p95_abs_speed_err_mps"] for row in group),
                "mean_speed_err_mean_mps": mean(row["mean_abs_speed_err_mps"] for row in group),
                "pwm_sat_rows_sum": sum(row["pwm_sat_rows"] for row in group),
                "slip_sum": sum(row["slip_count"] for row in group),
                "slipLat_sum": sum(row["slipLat_count"] for row in group),
                "max_abs_cmd_mV_max": max(row["max_abs_cmd_mV"] for row in group),
                "max_abs_pwm_max": max(row["max_abs_pwm"] for row in group),
            }
        )
    return result


def set_summary(rows):
    result = []
    current = []
    for row in rows:
        if not row["valid"]:
            continue
        auto = int(parse_float(row["autoStart"], 0))
        if auto == 1:
            current = [row]
        elif current:
            current.append(row)
            if auto == 5:
                if [int(parse_float(item["autoStart"], 0)) for item in current] == [1, 2, 3, 4, 5]:
                    boosted = [item for item in current if parse_float(item["optimalTrace"]) != 0.0]
                    result.append(
                        {
                            "set": len(result) + 1,
                            "logs": f"{current[0]['log']}-{current[-1]['log']}",
                            "buildTime": current[0]["buildTime"],
                            "time_auto1_ms": current[0]["last_cnt_ms"],
                            "boost_time_mean_ms": mean(item["last_cnt_ms"] for item in boosted),
                            "boost_time_min_ms": min(item["last_cnt_ms"] for item in boosted),
                            "boost_time_max_ms": max(item["last_cnt_ms"] for item in boosted),
                            "boost_battery_mean_V": mean(item["battery_mean_V"] for item in boosted),
                            "boost_pwm_sat_rows_sum": sum(item["pwm_sat_rows"] for item in boosted),
                            "boost_p95_speed_err_mean_mps": mean(item["p95_abs_speed_err_mps"] for item in boosted),
                            "boost_mean_speed_err_mean_mps": mean(item["mean_abs_speed_err_mps"] for item in boosted),
                            "boost_slip_sum": sum(item["slip_count"] for item in boosted),
                            "boost_slipLat_sum": sum(item["slipLat_count"] for item in boosted),
                            "boost_max_abs_cmd_mV": max(item["max_abs_cmd_mV"] for item in boosted),
                        }
                    )
                current = []
    return result


def escape_xml(text):
    return str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def write_svg(path, rows, sets):
    valid = [row for row in rows if row["valid"]]
    if not valid or not sets:
        path.write_text("", encoding="utf-8")
        return

    width = 1100
    height = 680
    margin_left = 70
    margin_right = 30
    margin_top = 40
    chart_h = 170
    gap = 45
    plot_w = width - margin_left - margin_right

    min_log = min(row["log"] for row in valid)
    max_log = max(row["log"] for row in valid)
    colors = {"1.00": "#444", "2.00": "#0067b1", "3.00": "#2b8a3e", "4.00": "#c15a00", "5.00": "#8a3ffc"}

    def x_for(log_number):
        if max_log == min_log:
            return margin_left
        return margin_left + (log_number - min_log) / (max_log - min_log) * plot_w

    def chart_y(index):
        return margin_top + index * (chart_h + gap)

    def scale(value, vmin, vmax, top):
        if vmax == vmin:
            return top + chart_h / 2
        return top + chart_h - (value - vmin) / (vmax - vmin) * chart_h

    time_values = [row["last_cnt_ms"] / 1000.0 for row in valid]
    battery_values = [row["battery_mean_V"] for row in valid]
    sat_values = [row["pwm_sat_rows"] for row in valid]
    boost_set_values = [row["boost_time_mean_ms"] / 1000.0 for row in sets]

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<style>text{font-family:Arial, sans-serif;font-size:13px;fill:#222}.small{font-size:11px}.axis{stroke:#bbb;stroke-width:1}.grid{stroke:#eee;stroke-width:1}.line{fill:none;stroke-width:2}.dot{stroke:#fff;stroke-width:1}</style>',
        f'<text x="{margin_left}" y="24" font-size="18">Logs {LOG_START}-{LOG_END} comparison</text>',
    ]

    charts = [
        ("Lap time [s]", time_values, 0),
        ("Battery mean [V]", battery_values, 1),
        ("PWM saturated rows", sat_values, 2),
    ]

    for title, values, idx in charts:
        top = chart_y(idx)
        vmin = min(values)
        vmax = max(values)
        pad = (vmax - vmin) * 0.08 if vmax > vmin else 1.0
        vmin -= pad
        vmax += pad
        svg.append(f'<text x="10" y="{top + 18}">{escape_xml(title)}</text>')
        svg.append(f'<line class="axis" x1="{margin_left}" y1="{top}" x2="{margin_left}" y2="{top + chart_h}"/>')
        svg.append(f'<line class="axis" x1="{margin_left}" y1="{top + chart_h}" x2="{width - margin_right}" y2="{top + chart_h}"/>')
        for frac in (0.0, 0.5, 1.0):
            y = top + frac * chart_h
            value = vmax - frac * (vmax - vmin)
            svg.append(f'<line class="grid" x1="{margin_left}" y1="{y:.1f}" x2="{width - margin_right}" y2="{y:.1f}"/>')
            svg.append(f'<text class="small" x="8" y="{y + 4:.1f}">{value:.2f}</text>')

        by_auto = defaultdict(list)
        for row in valid:
            if idx == 0:
                value = row["last_cnt_ms"] / 1000.0
            elif idx == 1:
                value = row["battery_mean_V"]
            else:
                value = row["pwm_sat_rows"]
            by_auto[row["autoStart"]].append((row["log"], value))

        for auto_start, points in sorted(by_auto.items(), key=lambda item: float(item[0])):
            path_points = " ".join(f"{x_for(log):.1f},{scale(value, vmin, vmax, top):.1f}" for log, value in points)
            color = colors.get(auto_start, "#555")
            svg.append(f'<polyline class="line" points="{path_points}" stroke="{color}"/>')
            for log, value in points:
                svg.append(f'<circle class="dot" cx="{x_for(log):.1f}" cy="{scale(value, vmin, vmax, top):.1f}" r="3" fill="{color}"/>')

    top = chart_y(3)
    vmin = min(boost_set_values)
    vmax = max(boost_set_values)
    pad = (vmax - vmin) * 0.08 if vmax > vmin else 1.0
    vmin -= pad
    vmax += pad
    svg.append(f'<text x="10" y="{top + 18}">Boost set mean [s]</text>')
    svg.append(f'<line class="axis" x1="{margin_left}" y1="{top}" x2="{margin_left}" y2="{top + chart_h}"/>')
    svg.append(f'<line class="axis" x1="{margin_left}" y1="{top + chart_h}" x2="{width - margin_right}" y2="{top + chart_h}"/>')
    points = []
    for index, row in enumerate(sets):
        x = margin_left + index / max(len(sets) - 1, 1) * plot_w
        y = scale(row["boost_time_mean_ms"] / 1000.0, vmin, vmax, top)
        points.append(f"{x:.1f},{y:.1f}")
        svg.append(f'<circle class="dot" cx="{x:.1f}" cy="{y:.1f}" r="3" fill="#005f73"/>')
    svg.append(f'<polyline class="line" points="{" ".join(points)}" stroke="#005f73"/>')

    legend_x = width - 360
    legend_y = 22
    for i, auto_start in enumerate(sorted(colors, key=float)):
        x = legend_x + i * 67
        svg.append(f'<rect x="{x}" y="{legend_y - 10}" width="12" height="12" fill="{colors[auto_start]}"/>')
        svg.append(f'<text class="small" x="{x + 16}" y="{legend_y}">auto {int(float(auto_start))}</text>')

    svg.append("</svg>")
    path.write_text("\n".join(svg), encoding="utf-8")


def build_summary(rows, sets):
    result = []
    by_build = defaultdict(list)
    by_build_sets = defaultdict(list)
    for row in rows:
        if row["valid"]:
            by_build[row["buildTime"]].append(row)
    for row in sets:
        by_build_sets[row["buildTime"]].append(row)

    for build_time, group in sorted(by_build.items()):
        boosted = [row for row in group if parse_float(row["optimalTrace"]) != 0.0]
        set_group = by_build_sets.get(build_time, [])
        result.append(
            {
                "buildTime": build_time,
                "log_count": len(group),
                "set_count": len(set_group),
                "boost_log_count": len(boosted),
                "auto1_mean_ms": mean(row["last_cnt_ms"] for row in group if row["autoStart"] == "1.00")
                if any(row["autoStart"] == "1.00" for row in group)
                else math.nan,
                "boost_log_mean_ms": mean(row["last_cnt_ms"] for row in boosted) if boosted else math.nan,
                "boost_set_mean_ms": mean(row["boost_time_mean_ms"] for row in set_group) if set_group else math.nan,
                "boost_set_min_ms": min([row["boost_time_mean_ms"] for row in set_group], default=math.nan),
                "boost_set_max_ms": max([row["boost_time_mean_ms"] for row in set_group], default=math.nan),
                "boost_battery_mean_V": mean(row["battery_mean_V"] for row in boosted) if boosted else math.nan,
                "boost_p95_speed_err_mean_mps": mean(row["p95_abs_speed_err_mps"] for row in boosted) if boosted else math.nan,
                "boost_pwm_sat_rows_sum": sum(row["pwm_sat_rows"] for row in boosted),
                "boost_slip_sum": sum(row["slip_count"] for row in boosted),
                "boost_slipLat_sum": sum(row["slipLat_count"] for row in boosted),
                "boost_max_abs_cmd_mV": max([row["max_abs_cmd_mV"] for row in boosted], default=math.nan),
            }
        )
    return result


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    rows = [summarize_log(number) for number in range(LOG_START, LOG_END + 1)]
    group_rows = grouped_summary(rows)
    set_rows = set_summary(rows)
    build_rows = build_summary(rows, set_rows)

    summary_path = OUT_DIR / "log_11177_11272_compare_summary.csv"
    auto_path = OUT_DIR / "log_11177_11272_compare_by_build_autostart.csv"
    set_path = OUT_DIR / "log_11177_11272_compare_sets.csv"
    build_path = OUT_DIR / "log_11177_11272_compare_by_build.csv"
    svg_path = OUT_DIR / "log_11177_11272_compare.svg"

    write_csv(summary_path, rows)
    write_csv(auto_path, group_rows)
    write_csv(set_path, set_rows)
    write_csv(build_path, build_rows)
    write_svg(svg_path, rows, set_rows)

    valid_rows = [row for row in rows if row["valid"]]
    print(f"summary_csv={summary_path}")
    print(f"auto_csv={auto_path}")
    print(f"sets_csv={set_path}")
    print(f"build_csv={build_path}")
    print(f"svg={svg_path}")
    print(f"valid={len(valid_rows)}/{len(rows)}")
    for row in rows:
        if not row["valid"]:
            print(f"invalid {row['log']} {row['reason']}")

    print("build_summary")
    for row in build_rows:
        print(
            f"build={row['buildTime']} logs={row['log_count']} sets={row['set_count']} "
            f"auto1={row['auto1_mean_ms']:.1f}ms boostSet={row['boost_set_mean_ms']:.1f}ms "
            f"batt={row['boost_battery_mean_V']:.3f}V p95err={row['boost_p95_speed_err_mean_mps']:.3f} "
            f"sat={row['boost_pwm_sat_rows_sum']} maxCmd={row['boost_max_abs_cmd_mV']:.0f}"
        )

    print("auto_summary")
    for row in group_rows:
        print(
            f"build={row['buildTime']} auto={row['autoStart']} n={row['count']} "
            f"mean={row['time_mean_ms']:.1f}ms min={row['time_min_ms']} max={row['time_max_ms']} "
            f"delta={row['first_to_last_delta_ms']}ms batt={row['battery_mean_V']:.3f}V "
            f"p95err={row['p95_speed_err_mean_mps']:.3f} sat={row['pwm_sat_rows_sum']} "
            f"slip={row['slip_sum']} slipLat={row['slipLat_sum']} maxCmd={row['max_abs_cmd_mV_max']:.0f}"
        )

    print("set_trend")
    for row in set_rows:
        print(
            f"set={row['set']:02d} logs={row['logs']} build={row['buildTime']} "
            f"auto1={row['time_auto1_ms']} boostMean={row['boost_time_mean_ms']:.1f}ms "
            f"batt={row['boost_battery_mean_V']:.3f}V sat={row['boost_pwm_sat_rows_sum']} "
            f"p95err={row['boost_p95_speed_err_mean_mps']:.3f} maxCmd={row['boost_max_abs_cmd_mV']:.0f}"
        )


if __name__ == "__main__":
    main()
