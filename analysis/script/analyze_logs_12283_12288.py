#!/usr/bin/env python3
"""12283 と広いコースの最新5走 (12284～12288) を比較する。"""

from __future__ import annotations

import csv
import math
import statistics
from pathlib import Path


LOG_DIR = Path(r"C:\Users\ucawa\Dropbox\Document\robotrace\Log\v2")
OUT_DIR = Path(__file__).resolve().parents[1]
LOG_NUMBERS = range(12283, 12289)
PULSES_PER_METER = 53424.0

REQUIRED_COLUMNS = (
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
)

TARGET_PARAMETER_NAMES = (
    "tgtParam.bstStraight",
    "tgtParam.bst1500",
    "tgtParam.bst1300",
    "tgtParam.bst1000",
    "tgtParam.bst800",
    "tgtParam.bst700",
    "tgtParam.bst600",
    "tgtParam.bst500",
    "tgtParam.bst400",
    "tgtParam.bst300",
    "tgtParam.bst200",
    "tgtParam.bst100",
    "tgtParam.acceleF",
    "tgtParam.acceleD",
    "tgtParam.decelLeadMm",
)


def percentile(values: list[float], fraction: float) -> float:
    values = sorted(values)
    if not values:
        return math.nan
    position = (len(values) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return values[lower]
    return values[lower] * (upper - position) + values[upper] * (position - lower)


def load_log(number: int) -> dict:
    path = LOG_DIR / f"{number}.csv"
    with path.open("r", encoding="utf-8-sig", newline="") as source:
        reader = csv.reader(source)
        header = next(reader)
        data_names = [name for name in header if name and "=" not in name]
        params = {}
        for item in header[len(data_names) :]:
            if "=" in item:
                key, value = item.split("=", 1)
                params[key] = value

        rows = []
        row_width_mismatch = 0
        for raw in reader:
            values = [value for value in raw if value != ""]
            if not values:
                continue
            if len(values) != len(data_names):
                row_width_mismatch += 1
                continue
            rows.append({name: float(value) for name, value in zip(data_names, values)})

    missing = [name for name in REQUIRED_COLUMNS if name not in data_names]
    cnt = [row["cntlog"] for row in rows]
    gaps = [b - a for a, b in zip(cnt, cnt[1:])]
    errors = [row["encCurrentN"] - row["targetSpeed"] for row in rows]
    abs_errors = [abs(value) for value in errors]
    pwms = [abs(row[name]) for row in rows for name in ("motorpwmL", "motorpwmR")]
    gyro = [abs(row["gyroVal_Z"]) for row in rows]

    summary = {
        "log": number,
        "course_context": "home_parameter_reference" if number == 12283 else "wide_course_latest5",
        "optimalTrace": int(float(params.get("optimalTrace", "-1"))),
        "autoStart": int(float(params.get("autoStart", "-1"))),
        "emcStop": int(float(params.get("emcStop", "-1"))),
        "batteryVoltage_V": float(params.get("batteryVoltage_V", "nan")),
        "rows": len(rows),
        "row_width_mismatch": row_width_mismatch,
        "required_columns_missing": ";".join(missing),
        "cntlog_nonpositive_gaps": sum(gap <= 0 for gap in gaps),
        "cntlog_median_gap_ms": statistics.median(gaps) if gaps else math.nan,
        "cntlog_max_gap_ms": max(gaps, default=math.nan),
        "lap_end_ms": int(cnt[-1]),
        "end_distance_p": int(rows[-1]["encTotalOptimal"]),
        "max_target_p_per_ms": int(max(row["targetSpeed"] for row in rows)),
        "max_target_m_per_s": max(row["targetSpeed"] for row in rows) * 1000.0 / PULSES_PER_METER,
        "speed_error_mae_p_per_ms": statistics.fmean(abs_errors),
        "speed_error_p95_p_per_ms": percentile(abs_errors, 0.95),
        "speed_error_mae_m_per_s": statistics.fmean(abs_errors) * 1000.0 / PULSES_PER_METER,
        "speed_error_p95_m_per_s": percentile(abs_errors, 0.95) * 1000.0 / PULSES_PER_METER,
        "gyro_abs_p95_deg_per_s": percentile(gyro, 0.95),
        "gyro_abs_max_deg_per_s": max(gyro),
        "slip_longitudinal_percent": 100.0 * sum(row["slipFlag"] != 0 for row in rows) / len(rows),
        "slip_lateral_percent": 100.0 * sum(row["slipFlagLat"] != 0 for row in rows) / len(rows),
        "pwm_abs_ge_950_percent": 100.0 * sum(value >= 950 for value in pwms) / len(pwms),
        "gitCommit": params.get("gitCommit", ""),
    }
    return {"number": number, "params": params, "rows": rows, "summary": summary}


def write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", encoding="utf-8-sig", newline="") as target:
        writer = csv.DictWriter(target, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _polyline(points: list[tuple[float, float]], bounds: tuple[float, float, float, float], panel: tuple[int, int, int, int]) -> str:
    xmin, xmax, ymin, ymax = bounds
    px, py, width, height = panel
    xspan = xmax - xmin or 1.0
    yspan = ymax - ymin or 1.0
    step = max(1, len(points) // 1000)
    result = []
    for x, y in points[::step]:
        sx = px + (x - xmin) / xspan * width
        sy = py + height - (y - ymin) / yspan * height
        result.append(f"{sx:.1f},{sy:.1f}")
    return " ".join(result)


def _bounds(series: list[list[tuple[float, float]]]) -> tuple[float, float, float, float]:
    points = [point for line in series for point in line]
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    xpad = (xmax - xmin) * 0.03 or 1.0
    ypad = (ymax - ymin) * 0.05 or 1.0
    return xmin - xpad, xmax + xpad, ymin - ypad, ymax + ypad


def plot_logs(logs: list[dict]) -> None:
    """外部描画ライブラリなしで検証用4面SVGを生成する。"""
    colors = ("#6b7280", "#2563eb", "#059669", "#d97706", "#dc2626", "#7c3aed")
    panels = ((70, 55, 500, 330), (650, 55, 500, 330), (70, 465, 500, 330), (650, 465, 500, 330))
    titles = ("XY trajectory [mm]", "Target(solid) / actual(dashed) speed [m/s]", "Gyro Z [deg/s]", "Slip flags (longitudinal + 1.2*lateral)")
    xy_series, speed_series, gyro_series, slip_series = [], [], [], []
    for log in logs:
        rows = log["rows"]
        time_s = [row["cntlog"] / 1000.0 for row in rows]
        xy_series.append([(row["x"], row["y"]) for row in rows])
        speed_series.extend(
            [
                list(zip(time_s, [row["targetSpeed"] * 1000.0 / PULSES_PER_METER for row in rows])),
                list(zip(time_s, [row["encCurrentN"] * 1000.0 / PULSES_PER_METER for row in rows])),
            ]
        )
        gyro_series.append(list(zip(time_s, [row["gyroVal_Z"] for row in rows])))
        slip_series.append(list(zip(time_s, [row["slipFlag"] + 1.2 * row["slipFlagLat"] for row in rows])))

    all_series = (xy_series, speed_series, gyro_series, slip_series)
    svg = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="1220" height="850" viewBox="0 0 1220 850">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<style>text{font-family:Segoe UI,Arial,sans-serif;fill:#111827}.title{font-size:16px;font-weight:600}.tick{font-size:11px}.legend{font-size:12px}</style>',
    ]
    for index, (panel, title, series) in enumerate(zip(panels, titles, all_series)):
        px, py, width, height = panel
        bounds = _bounds(series)
        xmin, xmax, ymin, ymax = bounds
        svg.extend(
            [
                f'<text class="title" x="{px}" y="{py - 18}">{title}</text>',
                f'<rect x="{px}" y="{py}" width="{width}" height="{height}" fill="none" stroke="#9ca3af"/>',
                f'<line x1="{px}" y1="{py + height / 2}" x2="{px + width}" y2="{py + height / 2}" stroke="#e5e7eb"/>',
                f'<line x1="{px + width / 2}" y1="{py}" x2="{px + width / 2}" y2="{py + height}" stroke="#e5e7eb"/>',
                f'<text class="tick" x="{px}" y="{py + height + 17}">{xmin:.1f}</text>',
                f'<text class="tick" x="{px + width - 35}" y="{py + height + 17}">{xmax:.1f}</text>',
                f'<text class="tick" x="{px - 55}" y="{py + 10}">{ymax:.1f}</text>',
                f'<text class="tick" x="{px - 55}" y="{py + height}">{ymin:.1f}</text>',
            ]
        )
        for series_index, points in enumerate(series):
            if index == 1:
                log_index = series_index // 2
                dashed = '' if series_index % 2 == 0 else ' stroke-dasharray="4 3" opacity="0.55"'
            else:
                log_index = series_index
                dashed = ""
            svg.append(
                f'<polyline points="{_polyline(points, bounds, panel)}" fill="none" stroke="{colors[log_index]}" stroke-width="1.1"{dashed}/>'
            )

    for index, log in enumerate(logs):
        x = 70 + index * 115
        svg.append(f'<line x1="{x}" y1="830" x2="{x + 24}" y2="830" stroke="{colors[index]}" stroke-width="3"/>')
        svg.append(f'<text class="legend" x="{x + 30}" y="834">{log["number"]}</text>')
    svg.append("</svg>")
    (OUT_DIR / "log_12283_12288_validation.svg").write_text("\n".join(svg), encoding="utf-8")


def main() -> None:
    logs = [load_log(number) for number in LOG_NUMBERS]
    write_csv(OUT_DIR / "log_12283_12288_validation_summary.csv", [log["summary"] for log in logs])

    target_rows = []
    for log in logs:
        row = {
            "log": log["number"],
            "course_context": log["summary"]["course_context"],
            "optimalTrace": log["summary"]["optimalTrace"],
            "autoStart": log["summary"]["autoStart"],
        }
        row.update({name: log["params"].get(name, "") for name in TARGET_PARAMETER_NAMES})
        target_rows.append(row)
    write_csv(OUT_DIR / "log_12283_12288_target_speed_parameters.csv", target_rows)
    plot_logs(logs)


if __name__ == "__main__":
    main()
