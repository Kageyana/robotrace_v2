from __future__ import annotations

import csv
import math
from pathlib import Path
from statistics import mean, median


LOG_DIR = Path(r"F:\Dropbox\Document\robotrace\Log\v2")
OUT_DIR = Path(__file__).resolve().parents[1]
START_LOG = 11030
END_LOG = 11082
PULSE_MILLIMETER = 54.324


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
]


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    pos = (len(ordered) - 1) * pct / 100.0
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return ordered[lo]
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (pos - lo)


def safe_float(value: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def safe_int(value: str) -> int:
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return 0


def parse_log(path: Path) -> dict[str, object]:
    raw = path.read_bytes().replace(b"\x00", b"")
    text = raw.decode("utf-8", errors="replace")
    rows = list(csv.reader(text.splitlines()))
    if not rows:
        return {"log": int(path.stem), "valid": False, "reason": "empty"}

    header = rows[0]
    param_start = next((i for i, name in enumerate(header) if "=" in name), len(header))
    columns = header[:param_start]
    params: dict[str, str] = {}
    for item in header[param_start:]:
        if "=" in item:
            key, value = item.split("=", 1)
            params[key] = value

    missing = [name for name in REQUIRED_COLUMNS if name not in columns]
    idx = {name: columns.index(name) for name in columns}
    data = [row for row in rows[1:] if row and len(row) >= len(columns)]

    def col(name: str) -> list[float]:
        if name not in idx:
            return []
        return [safe_float(row[idx[name]]) for row in data]

    cnt = col("cntlog")
    enc = col("encCurrentN")
    tgt = col("targetSpeed")
    gyro = col("gyroVal_Z")
    slip = col("slipFlag")
    slip_lat = col("slipFlagLat")
    pwm_l = col("motorpwmL")
    pwm_r = col("motorpwmR")
    batt_mv = col("batteryVoltage_mV")
    cmd_l_mv = col("motorVoltageCmdL_mV")
    cmd_r_mv = col("motorVoltageCmdR_mV")
    x = col("x")
    y = col("y")
    enc_total = col("encTotalOptimal")

    diffs = [cnt[i] - cnt[i - 1] for i in range(1, len(cnt)) if not math.isnan(cnt[i]) and not math.isnan(cnt[i - 1])]
    nonmono = sum(1 for d in diffs if d <= 0)
    large_gap = sum(1 for d in diffs if d > 50)
    speed_err = [
        abs((e - t) / PULSE_MILLIMETER)
        for e, t in zip(enc, tgt)
        if not math.isnan(e) and not math.isnan(t)
    ]

    duty_err: list[float] = []
    for pwm, cmd, batt in list(zip(pwm_l, cmd_l_mv, batt_mv)) + list(zip(pwm_r, cmd_r_mv, batt_mv)):
        if math.isnan(pwm) or math.isnan(cmd) or math.isnan(batt) or batt <= 0:
            continue
        expected = max(-1000.0, min(1000.0, cmd / batt * 1000.0))
        duty_err.append(abs(pwm - expected))

    emc_stop = safe_float(params.get("emcStop", "nan"))
    valid = (not missing) and len(data) > 0 and emc_stop == 0.0 and nonmono == 0 and large_gap == 0
    reasons = []
    if missing:
        reasons.append("missing:" + "|".join(missing))
    if len(data) == 0:
        reasons.append("no_data")
    if emc_stop != 0.0:
        reasons.append(f"emcStop={emc_stop:g}")
    if nonmono:
        reasons.append(f"cntlog_nonmonotonic={nonmono}")
    if large_gap:
        reasons.append(f"cntlog_gap_gt50ms={large_gap}")

    return {
        "log": int(path.stem),
        "valid": valid,
        "reason": ";".join(reasons),
        "rows": len(data),
        "last_cnt_ms": int(cnt[-1]) if cnt else 0,
        "last_encTotalOptimal": int(enc_total[-1]) if enc_total else 0,
        "optimalTrace": safe_float(params.get("optimalTrace", "nan")),
        "autoStart": safe_float(params.get("autoStart", "nan")),
        "emcStop": emc_stop,
        "batteryHeader_V": safe_float(params.get("batteryVoltage_V", "nan")),
        "batteryLog_min_V": min(batt_mv) / 1000.0 if batt_mv else math.nan,
        "batteryLog_mean_V": mean(batt_mv) / 1000.0 if batt_mv else math.nan,
        "batteryLog_max_V": max(batt_mv) / 1000.0 if batt_mv else math.nan,
        "max_target_mps": max(tgt) / PULSE_MILLIMETER if tgt else math.nan,
        "max_speed_mps": max(enc) / PULSE_MILLIMETER if enc else math.nan,
        "mean_abs_speed_err_mps": mean(speed_err) if speed_err else math.nan,
        "p95_abs_speed_err_mps": percentile(speed_err, 95),
        "max_abs_gyro_dps": max((abs(v) for v in gyro), default=math.nan),
        "slip_count": int(sum(1 for v in slip if v >= 0.5)),
        "slipLat_count": int(sum(1 for v in slip_lat if v >= 0.5)),
        "pwm_sat_count": int(sum(1 for v in pwm_l + pwm_r if abs(v) >= 1000)),
        "max_abs_pwm": max((abs(v) for v in pwm_l + pwm_r), default=math.nan),
        "max_abs_cmd_mV": max((abs(v) for v in cmd_l_mv + cmd_r_mv), default=math.nan),
        "median_duty_err": median(duty_err) if duty_err else math.nan,
        "max_duty_err": max(duty_err) if duty_err else math.nan,
        "cntlog_median_step_ms": median(diffs) if diffs else math.nan,
        "cntlog_max_step_ms": max(diffs) if diffs else math.nan,
        "x_last_mm": x[-1] if x else math.nan,
        "y_last_mm": y[-1] if y else math.nan,
        "lineTraceOmegaFBCtrl_kp": safe_float(params.get("lineTraceOmegaFBCtrl.kp", "nan")),
        "lineTraceOmegaFBCtrl_kd": safe_float(params.get("lineTraceOmegaFBCtrl.kd", "nan")),
        "branch": params.get("branch", ""),
        "gitCommit": params.get("gitCommit", ""),
    }


def group_rows(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    groups: dict[tuple[float, float], list[dict[str, object]]] = {}
    for row in rows:
        key = (float(row["optimalTrace"]), float(row["autoStart"]))
        groups.setdefault(key, []).append(row)

    out = []
    for (optimal, auto), items in sorted(groups.items()):
        valid_items = [r for r in items if r["valid"]]
        out.append(
            {
                "optimalTrace": optimal,
                "autoStart": auto,
                "logs": " ".join(str(r["log"]) for r in items),
                "count": len(items),
                "valid_count": len(valid_items),
                "invalid_logs": " ".join(str(r["log"]) for r in items if not r["valid"]),
                "battery_mean_range_V": (
                    f"{min(float(r['batteryLog_mean_V']) for r in items):.3f}-"
                    f"{max(float(r['batteryLog_mean_V']) for r in items):.3f}"
                ),
                "last_cnt_mean_ms": (
                    f"{mean(float(r['last_cnt_ms']) for r in valid_items):.1f}"
                    if valid_items
                    else ""
                ),
                "p95_speed_err_mean_mps": (
                    f"{mean(float(r['p95_abs_speed_err_mps']) for r in valid_items):.3f}"
                    if valid_items
                    else ""
                ),
                "slip_total": sum(int(r["slip_count"]) for r in items),
                "slipLat_total": sum(int(r["slipLat_count"]) for r in items),
                "pwm_sat_total": sum(int(r["pwm_sat_count"]) for r in items),
            }
        )
    return out


def mark_distance_outliers(rows: list[dict[str, object]]) -> None:
    by_mode: dict[float, list[dict[str, object]]] = {}
    for row in rows:
        if row["valid"]:
            by_mode.setdefault(float(row["optimalTrace"]), []).append(row)

    for items in by_mode.values():
        if len(items) < 3:
            continue
        distances = sorted(float(r["last_encTotalOptimal"]) for r in items)
        ref = median(distances)
        for row in items:
            distance = float(row["last_encTotalOptimal"])
            if ref > 0.0 and distance < ref * 0.90:
                row["valid"] = False
                reason = str(row["reason"])
                suffix = f"incomplete_distance={distance:.0f}_lt_90pct_ref_{ref:.0f}"
                row["reason"] = suffix if not reason else reason + ";" + suffix


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def read_columns(path: Path, names: list[str]) -> dict[str, list[float]]:
    raw = path.read_bytes().replace(b"\x00", b"")
    rows = list(csv.reader(raw.decode("utf-8", errors="replace").splitlines()))
    header = rows[0]
    param_start = next((i for i, name in enumerate(header) if "=" in name), len(header))
    columns = header[:param_start]
    idx = {name: columns.index(name) for name in names if name in columns}
    out = {name: [] for name in names}
    for row in rows[1:]:
        if not row or len(row) < len(columns):
            continue
        for name, i in idx.items():
            out[name].append(safe_float(row[i]))
    return out


def finite(values: list[float]) -> list[float]:
    return [v for v in values if not math.isnan(v) and not math.isinf(v)]


def scale(value: float, src_min: float, src_max: float, dst_min: float, dst_max: float) -> float:
    if src_max <= src_min:
        return (dst_min + dst_max) / 2.0
    return dst_min + (value - src_min) * (dst_max - dst_min) / (src_max - src_min)


def padded_range(values: list[float], include: list[float] | None = None) -> tuple[float, float]:
    vals = finite(values + (include or []))
    lo = min(vals)
    hi = max(vals)
    pad = (hi - lo) * 0.08 if hi > lo else max(abs(hi) * 0.08, 1.0)
    return lo - pad, hi + pad


def save_summary_plot(rows: list[dict[str, object]]) -> None:
    width = 1120
    height = 900
    left = 95
    right = 30
    top = 70
    panel_h = 175
    gap = 28
    plot_w = width - left - right
    logs = [int(r["log"]) for r in rows]
    x_min, x_max = min(logs), max(logs)
    series = [
        ("battery mean [V]", [float(r["batteryLog_mean_V"]) for r in rows], [7.6]),
        ("last cnt [s]", [float(r["last_cnt_ms"]) / 1000.0 for r in rows], []),
        ("p95 speed err [m/s]", [float(r["p95_abs_speed_err_mps"]) for r in rows], []),
        ("PWM saturation count", [float(r["pwm_sat_count"]) for r in rows], []),
    ]
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<text x="40" y="34" font-family="Arial" font-size="22" fill="#222">Logs 11030-11082 voltage-command check</text>',
        '<text x="875" y="34" font-family="Arial" font-size="13" fill="#0067a5">blue: valid</text>',
        '<text x="975" y="34" font-family="Arial" font-size="13" fill="#c43c39">red: invalid</text>',
    ]
    for pi, (label, values, refs) in enumerate(series):
        y0 = top + pi * (panel_h + gap)
        y_min, y_max = padded_range(values, refs)
        svg.append(f'<rect x="{left}" y="{y0}" width="{plot_w}" height="{panel_h}" fill="#fafafa" stroke="#ddd"/>')
        svg.append(f'<text x="18" y="{y0 + 28}" font-family="Arial" font-size="14" fill="#222">{label}</text>')
        for t in range(5):
            yy = y0 + panel_h * t / 4.0
            val = y_max - (y_max - y_min) * t / 4.0
            svg.append(f'<line x1="{left}" y1="{yy:.1f}" x2="{left + plot_w}" y2="{yy:.1f}" stroke="#e6e6e6"/>')
            svg.append(f'<text x="{left - 8}" y="{yy + 4:.1f}" text-anchor="end" font-family="Arial" font-size="11" fill="#666">{val:.2f}</text>')
        for ref in refs:
            yy = scale(ref, y_min, y_max, y0 + panel_h, y0)
            svg.append(f'<line x1="{left}" y1="{yy:.1f}" x2="{left + plot_w}" y2="{yy:.1f}" stroke="#555" stroke-dasharray="5,5"/>')
        for row, value in zip(rows, values):
            x = scale(int(row["log"]), x_min, x_max, left, left + plot_w)
            y = scale(value, y_min, y_max, y0 + panel_h, y0)
            color = "#0067a5" if row["valid"] else "#c43c39"
            svg.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="3.5" fill="{color}"/>')
    for tick in range(x_min, x_max + 1, 10):
        x = scale(tick, x_min, x_max, left, left + plot_w)
        svg.append(f'<text x="{x:.1f}" y="{height - 20}" text-anchor="middle" font-family="Arial" font-size="11" fill="#666">{tick}</text>')
    svg.append(f'<text x="{left + plot_w / 2:.1f}" y="{height - 4}" text-anchor="middle" font-family="Arial" font-size="13" fill="#222">log number</text>')
    svg.append("</svg>")
    (OUT_DIR / "log_11030_11082_voltage_command_check_summary.svg").write_text("\n".join(svg), encoding="utf-8")


def save_xy_plot(rows: list[dict[str, object]]) -> None:
    latest_by_auto: dict[int, int] = {}
    for row in rows:
        if row["valid"] and float(row["optimalTrace"]) in (0.0, 2.0):
            latest_by_auto[int(float(row["autoStart"]))] = int(row["log"])

    traces: list[tuple[int, dict[str, list[float]]]] = []
    for auto in sorted(latest_by_auto):
        log_num = latest_by_auto[auto]
        cols = read_columns(LOG_DIR / f"{log_num}.csv", ["x", "y"])
        if not cols["x"] or not cols["y"]:
            continue
        traces.append((log_num, cols))
    all_x = finite([x for _, cols in traces for x in cols["x"]])
    all_y = finite([y for _, cols in traces for y in cols["y"]])
    x_min, x_max = padded_range(all_x)
    y_min, y_max = padded_range(all_y)
    width = 840
    height = 840
    margin = 70
    plot = width - margin * 2
    colors = ["#0067a5", "#d55e00", "#009e73", "#cc79a7", "#0072b2"]
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<text x="40" y="34" font-family="Arial" font-size="22" fill="#222">Latest valid trajectory per autoStart</text>',
        f'<rect x="{margin}" y="{margin}" width="{plot}" height="{plot}" fill="#fafafa" stroke="#ddd"/>',
    ]
    for t in range(5):
        x = margin + plot * t / 4.0
        y = margin + plot * t / 4.0
        xv = x_min + (x_max - x_min) * t / 4.0
        yv = y_max - (y_max - y_min) * t / 4.0
        svg.append(f'<line x1="{x:.1f}" y1="{margin}" x2="{x:.1f}" y2="{margin + plot}" stroke="#e6e6e6"/>')
        svg.append(f'<line x1="{margin}" y1="{y:.1f}" x2="{margin + plot}" y2="{y:.1f}" stroke="#e6e6e6"/>')
        svg.append(f'<text x="{x:.1f}" y="{margin + plot + 18}" text-anchor="middle" font-family="Arial" font-size="11" fill="#666">{xv:.0f}</text>')
        svg.append(f'<text x="{margin - 8}" y="{y + 4:.1f}" text-anchor="end" font-family="Arial" font-size="11" fill="#666">{yv:.0f}</text>')
    for i, (log_num, cols) in enumerate(traces):
        pts = []
        for x_raw, y_raw in zip(cols["x"], cols["y"]):
            if math.isnan(x_raw) or math.isnan(y_raw):
                continue
            x = scale(x_raw, x_min, x_max, margin, margin + plot)
            y = scale(y_raw, y_min, y_max, margin + plot, margin)
            pts.append(f"{x:.1f},{y:.1f}")
        color = colors[i % len(colors)]
        svg.append(f'<polyline points="{" ".join(pts)}" fill="none" stroke="{color}" stroke-width="1.4"/>')
        svg.append(f'<text x="{margin + 12}" y="{margin + 22 + i * 18}" font-family="Arial" font-size="13" fill="{color}">{log_num}</text>')
    svg.append(f'<text x="{width / 2:.1f}" y="{height - 16}" text-anchor="middle" font-family="Arial" font-size="13" fill="#222">x [mm]</text>')
    svg.append(f'<text x="18" y="{height / 2:.1f}" transform="rotate(-90 18 {height / 2:.1f})" text-anchor="middle" font-family="Arial" font-size="13" fill="#222">y [mm]</text>')
    svg.append("</svg>")
    (OUT_DIR / "log_11030_11082_latest_valid_xy.svg").write_text("\n".join(svg), encoding="utf-8")


def main() -> None:
    files = [
        p
        for p in LOG_DIR.glob("*.csv")
        if p.stem.isdigit() and START_LOG <= int(p.stem) <= END_LOG
    ]
    rows = [parse_log(path) for path in sorted(files, key=lambda p: int(p.stem))]
    mark_distance_outliers(rows)
    summary_path = OUT_DIR / "log_11030_11082_voltage_command_check_summary.csv"
    sets_path = OUT_DIR / "log_11030_11082_voltage_command_check_sets.csv"
    write_csv(summary_path, rows)
    write_csv(sets_path, group_rows(rows))
    save_summary_plot(rows)
    save_xy_plot(rows)
    print(f"logs={len(rows)}")
    print(f"summary={summary_path}")
    print(f"sets={sets_path}")
    print(f"valid={sum(1 for r in rows if r['valid'])}")
    print(f"invalid={sum(1 for r in rows if not r['valid'])}")


if __name__ == "__main__":
    main()
