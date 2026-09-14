#!/usr/bin/env python3
"""Compare failed primary runs 12434-12436 with nearby successful runs."""

from __future__ import annotations

import csv
import math
from pathlib import Path


PULSE_PER_MM = 53.424
LOG_DIR = Path(r"C:\Users\ucawa\Dropbox\Document\robotrace\Log\v2")
OUTPUT_DIR = Path(__file__).resolve().parents[1]
RUN_IDS = (12431, 12434, 12435, 12436, 12437)
FAILED_IDS = {12434, 12435, 12436}


def parse_metadata(line: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for field in line.rstrip("\r\n").split(","):
        if "=" in field:
            key, value = field.split("=", 1)
            result[key] = value
    return result


def load_log(run_id: int) -> tuple[dict[str, str], list[dict[str, float]]]:
    path = LOG_DIR / f"{run_id}.csv"
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        metadata = parse_metadata(handle.readline())
        reader = csv.DictReader(handle)
        rows: list[dict[str, float]] = []
        for source in reader:
            if not source or not source.get("cntlog"):
                continue
            rows.append({key: float(value) for key, value in source.items() if key and value})
    return metadata, rows


def distance_mm(row: dict[str, float]) -> float:
    return row["encTotalOptimal"] / PULSE_PER_MM


def window(rows: list[dict[str, float]], start_mm: float, end_mm: float) -> list[dict[str, float]]:
    return [row for row in rows if start_mm <= distance_mm(row) < end_mm]


def integrate_deg(rows: list[dict[str, float]], column: str) -> float:
    total = 0.0
    for previous, current in zip(rows, rows[1:]):
        dt_s = (current["cntlog"] - previous["cntlog"]) * 0.001
        total += 0.5 * (previous[column] + current[column]) * dt_s
    return total


def nearest(rows: list[dict[str, float]], target_mm: float) -> dict[str, float]:
    return min(rows, key=lambda row: abs(distance_mm(row) - target_mm))


def svg_polyline(points: list[tuple[float, float]], x_map, y_map) -> str:
    encoded = " ".join(f"{x_map(x):.1f},{y_map(y):.1f}" for x, y in points)
    return f'<polyline points="{encoded}" fill="none" stroke-width="2"/>'


def write_yaw_svg(logs: dict[int, list[dict[str, float]]]) -> None:
    width, height = 1100, 700
    left, right, top, bottom = 85, 30, 45, 65
    plot_w, plot_h = width - left - right, height - top - bottom
    x_min, x_max = 500.0, 1050.0
    y_min, y_max = -900.0, 900.0
    colors = {12431: "#2ca02c", 12434: "#d62728", 12435: "#ff7f0e", 12436: "#9467bd", 12437: "#1f77b4"}
    x_map = lambda value: left + (value - x_min) / (x_max - x_min) * plot_w
    y_map = lambda value: top + (y_max - value) / (y_max - y_min) * plot_h
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<style>text{font-family:Segoe UI,Arial,sans-serif;font-size:14px}.axis{stroke:#333;stroke-width:1}.grid{stroke:#ddd;stroke-width:1}.target{stroke-dasharray:7 4}.actual{opacity:.85}</style>',
        '<text x="550" y="25" text-anchor="middle" font-size="19">Start curve yaw command and response</text>',
    ]
    for x in range(500, 1051, 100):
        px = x_map(float(x))
        parts.append(f'<line class="grid" x1="{px:.1f}" y1="{top}" x2="{px:.1f}" y2="{top + plot_h}"/>')
        parts.append(f'<text x="{px:.1f}" y="{height - 35}" text-anchor="middle">{x}</text>')
    for y in range(-900, 901, 300):
        py = y_map(float(y))
        parts.append(f'<line class="grid" x1="{left}" y1="{py:.1f}" x2="{left + plot_w}" y2="{py:.1f}"/>')
        parts.append(f'<text x="{left - 10}" y="{py + 5:.1f}" text-anchor="end">{y}</text>')
    parts.extend([
        f'<line class="axis" x1="{left}" y1="{top + plot_h}" x2="{left + plot_w}" y2="{top + plot_h}"/>',
        f'<line class="axis" x1="{left}" y1="{top}" x2="{left}" y2="{top + plot_h}"/>',
        f'<text x="{left + plot_w / 2:.1f}" y="{height - 8}" text-anchor="middle">Travel distance [mm]</text>',
        f'<text x="20" y="{top + plot_h / 2:.1f}" transform="rotate(-90 20 {top + plot_h / 2:.1f})" text-anchor="middle">Yaw rate [deg/s]</text>',
    ])
    for run_id, rows in logs.items():
        selected = [row for row in rows if x_min <= distance_mm(row) <= x_max]
        target = [(distance_mm(row), row["targetAngularvelo"]) for row in selected]
        actual = [(distance_mm(row), row["gyroVal_Z"]) for row in selected]
        color = colors[run_id]
        parts.append(f'<g stroke="{color}" class="target">{svg_polyline(target, x_map, y_map)}</g>')
        parts.append(f'<g stroke="{color}" class="actual">{svg_polyline(actual, x_map, y_map)}</g>')
    legend_x, legend_y = 865, 58
    for index, run_id in enumerate(RUN_IDS):
        y = legend_y + index * 23
        label = "failed" if run_id in FAILED_IDS else "success"
        parts.append(f'<line x1="{legend_x}" y1="{y}" x2="{legend_x + 28}" y2="{y}" stroke="{colors[run_id]}" stroke-width="3"/>')
        parts.append(f'<text x="{legend_x + 36}" y="{y + 5}">{run_id} ({label})</text>')
    parts.append('<text x="865" y="190">dashed: target / solid: gyro</text>')
    parts.append("</svg>")
    (OUTPUT_DIR / "log_12434_12436_start_curve_yaw.svg").write_text("\n".join(parts), encoding="utf-8")


def write_xy_svg(logs: dict[int, list[dict[str, float]]]) -> None:
    width, height = 700, 760
    left, right, top, bottom = 80, 35, 45, 60
    x_min, x_max = -25.0, 180.0
    y_min, y_max = 0.0, 1050.0
    plot_w, plot_h = width - left - right, height - top - bottom
    colors = {12431: "#2ca02c", 12434: "#d62728", 12435: "#ff7f0e", 12436: "#9467bd", 12437: "#1f77b4"}
    x_map = lambda value: left + (value - x_min) / (x_max - x_min) * plot_w
    y_map = lambda value: top + (y_max - value) / (y_max - y_min) * plot_h
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<style>text{font-family:Segoe UI,Arial,sans-serif;font-size:14px}.axis{stroke:#333}.grid{stroke:#ddd}</style>',
        '<text x="350" y="25" text-anchor="middle" font-size="19">Dead-reckoned trajectory to first line loss</text>',
    ]
    for x in range(0, 181, 50):
        px = x_map(float(x))
        parts.append(f'<line class="grid" x1="{px:.1f}" y1="{top}" x2="{px:.1f}" y2="{top + plot_h}"/>')
        parts.append(f'<text x="{px:.1f}" y="{height - 28}" text-anchor="middle">{x}</text>')
    for y in range(0, 1051, 200):
        py = y_map(float(y))
        parts.append(f'<line class="grid" x1="{left}" y1="{py:.1f}" x2="{left + plot_w}" y2="{py:.1f}"/>')
        parts.append(f'<text x="{left - 10}" y="{py + 5:.1f}" text-anchor="end">{y}</text>')
    parts.extend([
        f'<line class="axis" x1="{left}" y1="{top + plot_h}" x2="{left + plot_w}" y2="{top + plot_h}"/>',
        f'<line class="axis" x1="{left}" y1="{top}" x2="{left}" y2="{top + plot_h}"/>',
        f'<text x="{left + plot_w / 2:.1f}" y="{height - 5}" text-anchor="middle">x [mm]</text>',
        f'<text x="20" y="{top + plot_h / 2:.1f}" transform="rotate(-90 20 {top + plot_h / 2:.1f})" text-anchor="middle">y [mm]</text>',
    ])
    for run_id, rows in logs.items():
        selected = [row for row in rows if distance_mm(row) <= 1050.0]
        points = [(row["x"], row["y"]) for row in selected]
        parts.append(f'<g stroke="{colors[run_id]}">{svg_polyline(points, x_map, y_map)}</g>')
        end = selected[-1]
        parts.append(f'<text x="{x_map(end["x"]) + 5:.1f}" y="{y_map(end["y"]) + 4:.1f}" fill="{colors[run_id]}">{run_id}</text>')
    parts.append("</svg>")
    (OUTPUT_DIR / "log_12434_12436_start_curve_xy.svg").write_text("\n".join(parts), encoding="utf-8")


def main() -> None:
    metadata_by_id: dict[int, dict[str, str]] = {}
    logs: dict[int, list[dict[str, float]]] = {}
    for run_id in RUN_IDS:
        metadata_by_id[run_id], logs[run_id] = load_log(run_id)

    summary_path = OUTPUT_DIR / "log_12434_12436_start_curve_summary.csv"
    fields = [
        "log", "result", "emcStop", "battery_V", "samples", "end_ms", "end_distance_mm",
        "cntlog_max_gap_ms", "kalman_innovation_reject", "kalman_invalid_update", "kalman_output_guard",
        "positive_target_peak_dps", "positive_gyro_peak_dps", "positive_target_integral_deg",
        "positive_gyro_integral_deg", "reverse_target_below_minus100_mm",
        "negative_target_peak_dps", "negative_gyro_peak_dps",
        "negative_target_integral_deg", "negative_gyro_integral_deg", "max_abs_wheel_delta_p",
        "active_yaw_mae_dps", "x_at_700_mm", "x_at_900_mm", "x_end_mm", "y_end_mm",
    ]
    with summary_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for run_id in RUN_IDS:
            metadata, rows = metadata_by_id[run_id], logs[run_id]
            positive = window(rows, 550.0, 760.0)
            negative = window(rows, 760.0, 930.0)
            active = [row for row in rows if 550.0 <= distance_mm(row) < 930.0 and abs(row["targetAngularvelo"]) >= 100.0]
            reverse = next(row for row in rows if distance_mm(row) >= 650.0 and row["targetAngularvelo"] < -100.0)
            gaps = [current["cntlog"] - previous["cntlog"] for previous, current in zip(rows, rows[1:])]
            at_700, at_900 = nearest(rows, 700.0), nearest(rows, 900.0)
            writer.writerow({
                "log": run_id,
                "result": "failed" if run_id in FAILED_IDS else "success",
                "emcStop": metadata.get("emcStop", ""),
                "battery_V": metadata.get("batteryVoltage_V", ""),
                "samples": len(rows),
                "end_ms": f'{rows[-1]["cntlog"]:.0f}',
                "end_distance_mm": f"{distance_mm(rows[-1]):.1f}",
                "cntlog_max_gap_ms": f"{max(gaps):.0f}",
                "kalman_innovation_reject": metadata.get("distanceKalman.innovationRejectCount", ""),
                "kalman_invalid_update": metadata.get("distanceKalman.invalidUpdateCount", ""),
                "kalman_output_guard": metadata.get("distanceKalman.outputGuardCount", ""),
                "positive_target_peak_dps": f'{max(row["targetAngularvelo"] for row in positive):.1f}',
                "positive_gyro_peak_dps": f'{max(row["gyroVal_Z"] for row in positive):.1f}',
                "positive_target_integral_deg": f'{integrate_deg(positive, "targetAngularvelo"):.1f}',
                "positive_gyro_integral_deg": f'{integrate_deg(positive, "gyroVal_Z"):.1f}',
                "reverse_target_below_minus100_mm": f"{distance_mm(reverse):.1f}",
                "negative_target_peak_dps": f'{min(row["targetAngularvelo"] for row in negative):.1f}',
                "negative_gyro_peak_dps": f'{min(row["gyroVal_Z"] for row in negative):.1f}',
                "negative_target_integral_deg": f'{integrate_deg(negative, "targetAngularvelo"):.1f}',
                "negative_gyro_integral_deg": f'{integrate_deg(negative, "gyroVal_Z"):.1f}',
                "max_abs_wheel_delta_p": f'{max(abs(row["encCurrentL"] - row["encCurrentR"]) for row in window(rows, 550.0, 930.0)):.0f}',
                "active_yaw_mae_dps": f'{sum(abs(row["targetAngularvelo"] - row["gyroVal_Z"]) for row in active) / len(active):.1f}',
                "x_at_700_mm": f'{at_700["x"]:.1f}',
                "x_at_900_mm": f'{at_900["x"]:.1f}',
                "x_end_mm": f'{rows[-1]["x"]:.1f}',
                "y_end_mm": f'{rows[-1]["y"]:.1f}',
            })

    aligned_path = OUTPUT_DIR / "log_12434_12436_start_curve_aligned.csv"
    aligned_fields = ["log", "result", "distance_mm", "cntlog_ms", "target_yaw_dps", "gyro_dps", "inferred_sensor_diff", "enc_left_p", "enc_right_p", "x_mm", "y_mm"]
    with aligned_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=aligned_fields)
        writer.writeheader()
        for run_id in RUN_IDS:
            for target_mm in range(500, 1001, 25):
                row = nearest(logs[run_id], float(target_mm))
                sensor_diff = row["targetAngularvelo"] * 512.0 / row["encCurrentN"] if row["encCurrentN"] else math.nan
                writer.writerow({
                    "log": run_id,
                    "result": "failed" if run_id in FAILED_IDS else "success",
                    "distance_mm": target_mm,
                    "cntlog_ms": f'{row["cntlog"]:.0f}',
                    "target_yaw_dps": f'{row["targetAngularvelo"]:.1f}',
                    "gyro_dps": f'{row["gyroVal_Z"]:.1f}',
                    "inferred_sensor_diff": f"{sensor_diff:.1f}",
                    "enc_left_p": f'{row["encCurrentL"]:.0f}',
                    "enc_right_p": f'{row["encCurrentR"]:.0f}',
                    "x_mm": f'{row["x"]:.1f}',
                    "y_mm": f'{row["y"]:.1f}',
                })

    write_yaw_svg(logs)
    write_xy_svg(logs)
    print(summary_path)
    print(aligned_path)


if __name__ == "__main__":
    main()
