#!/usr/bin/env python3
"""一次走行ログ12384～12394のジャイロ積分とXY閉路誤差を比較する。"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt

from robotrace_units import PULSE_MILLIMETER


LOG_NUMBERS = (12384, 12386, 12387, 12388, 12389, 12394)
TREAD_MM = 109.0
FIXED_STRAIGHTS_MM = (
    ("S1_start_up", 300.0, 1700.0),
    ("S2_outer_down", 5000.0, 8000.0),
    ("S3_inner_up", 10000.0, 12500.0),
    ("S4_late_down", 28500.0, 31500.0),
)


@dataclass
class LogData:
    number: int
    metadata: dict[str, str]
    rows: list[dict[str, float]]
    time_ms: list[int]
    distance_mm: list[float]
    heading_deg: list[float]
    encoder_heading_deg: list[float]


def read_log(path: Path) -> LogData:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        first = next(csv.reader([stream.readline()]))
        metadata = {}
        for field in first:
            if "=" in field:
                key, value = field.split("=", 1)
                metadata[key] = value
        reader = csv.DictReader(stream)
        rows = []
        for raw in reader:
            if not raw or raw.get("cntlog", "") == "":
                continue
            rows.append({key: float(value) for key, value in raw.items()
                         if key and value not in (None, "")})

    time_ms: list[int] = []
    wrap_base = 0
    previous_raw: int | None = None
    for row in rows:
        current = int(row["cntlog"])
        if previous_raw is not None and current < previous_raw:
            wrap_base += 65536
        time_ms.append(current + wrap_base)
        previous_raw = current

    distance_mm = [0.0]
    heading_deg = [0.0]
    encoder_heading_deg = [0.0]
    for index in range(1, len(rows)):
        dt_ms = time_ms[index] - time_ms[index - 1]
        distance_mm.append(distance_mm[-1] + rows[index]["encCurrentCorr_p"]
                           / PULSE_MILLIMETER * dt_ms)
        heading_deg.append(heading_deg[-1] + rows[index]["gyroVal_Z"] * dt_ms / 1000.0)
        encoder_rate = ((rows[index]["encCurrentL"] - rows[index]["encCurrentR"])
                        / PULSE_MILLIMETER / TREAD_MM * 180.0 / math.pi * 1000.0)
        encoder_heading_deg.append(encoder_heading_deg[-1] + encoder_rate * dt_ms / 1000.0)

    return LogData(int(path.stem), metadata, rows, time_ms, distance_mm,
                   heading_deg, encoder_heading_deg)


def decode_temperature(raw: float) -> float | None:
    code = int(raw) & 0x7FF
    if code == 0x400:
        return None
    signed = code - 2048 if code & 0x400 else code
    return 23.0 + signed * 0.125


def wrap_parallel_y(angle_deg: float) -> float:
    """線分方向を無視し、Y軸平行からの偏差を[-90, 90) degで返す。"""
    return (angle_deg + 90.0) % 180.0 - 90.0


def integrate_pose(log: LogData, gyro_scale: float = 1.0,
                   gyro_bias_dps: float = 0.0,
                   temp_slope_dps_per_c: float = 0.0) -> tuple[float, float, float]:
    x_mm = 0.0
    y_mm = 0.0
    heading_deg = 0.0
    previous_ms = 0
    temperatures = [decode_temperature(row.get("imuTempRaw", 0.0)) for row in log.rows]
    calibration_temp = next((value for value in temperatures if value is not None), 23.0)
    for row, current_ms, temperature in zip(log.rows, log.time_ms, temperatures):
        dt_s = (current_ms - previous_ms) / 1000.0
        thermal_term = 0.0 if temperature is None else temp_slope_dps_per_c * (temperature - calibration_temp)
        heading_deg += (row["gyroVal_Z"] * gyro_scale + gyro_bias_dps + thermal_term) * dt_s
        distance_mm = row["encCurrentCorr_p"] / PULSE_MILLIMETER * dt_s * 1000.0
        heading_rad = math.radians(heading_deg)
        x_mm += distance_mm * math.sin(heading_rad)
        y_mm += distance_mm * math.cos(heading_rad)
        previous_ms = current_ms
    return x_mm, y_mm, heading_deg


def integrate_pose_trapezoid(log: LogData) -> tuple[float, float, float]:
    """保存点間を台形積分し、endLogの右端矩形積分との差を確認する。"""
    x_mm = 0.0
    y_mm = 0.0
    heading_deg = 0.0
    previous_ms = 0
    previous_gyro = log.rows[0]["gyroVal_Z"]
    previous_speed = log.rows[0]["encCurrentCorr_p"]
    for index, (row, current_ms) in enumerate(zip(log.rows, log.time_ms)):
        dt_s = (current_ms - previous_ms) / 1000.0
        if index == 0:
            gyro = row["gyroVal_Z"]
            speed = row["encCurrentCorr_p"]
        else:
            gyro = 0.5 * (previous_gyro + row["gyroVal_Z"])
            speed = 0.5 * (previous_speed + row["encCurrentCorr_p"])
        previous_heading = heading_deg
        heading_deg += gyro * dt_s
        midpoint_heading = math.radians(0.5 * (previous_heading + heading_deg))
        distance_mm = speed / PULSE_MILLIMETER * dt_s * 1000.0
        x_mm += distance_mm * math.sin(midpoint_heading)
        y_mm += distance_mm * math.cos(midpoint_heading)
        previous_ms = current_ms
        previous_gyro = row["gyroVal_Z"]
        previous_speed = row["encCurrentCorr_p"]
    return x_mm, y_mm, heading_deg


def segment_metrics(log: LogData, begin: int, end: int) -> dict[str, float]:
    rows = log.rows
    elapsed_s = (log.time_ms[end] - log.time_ms[begin]) / 1000.0
    dx = rows[end]["x"] - rows[begin]["x"]
    dy = rows[end]["y"] - rows[begin]["y"]
    chord_mm = math.hypot(dx, dy)
    slope_deg = math.degrees(math.atan2(dx, dy))
    gyro_delta = log.heading_deg[end] - log.heading_deg[begin]
    encoder_delta = log.encoder_heading_deg[end] - log.encoder_heading_deg[begin]
    temps = [decode_temperature(rows[i].get("imuTempRaw", 0.0)) for i in range(begin, end + 1)]
    valid_temps = [value for value in temps if value is not None]
    return {
        "begin_ms": log.time_ms[begin],
        "end_ms": log.time_ms[end],
        "begin_dist_mm": log.distance_mm[begin],
        "end_dist_mm": log.distance_mm[end],
        "length_mm": log.distance_mm[end] - log.distance_mm[begin],
        "chord_mm": chord_mm,
        "slope_from_y_deg": wrap_parallel_y(slope_deg),
        "gyro_delta_deg": gyro_delta,
        "gyro_mean_dps": gyro_delta / elapsed_s,
        "encoder_delta_deg": encoder_delta,
        "temp_begin_c": valid_temps[0] if valid_temps else math.nan,
        "temp_end_c": valid_temps[-1] if valid_temps else math.nan,
    }


def find_long_straights(log: LogData) -> list[tuple[int, int]]:
    """400 mm窓の積分角変化が小さい区間を連結し、長い直線候補を返す。"""
    n = len(log.rows)
    straight = [False] * n
    left = 0
    for right in range(n):
        while left < right and log.distance_mm[right] - log.distance_mm[left] > 450.0:
            left += 1
        if log.distance_mm[right] - log.distance_mm[left] >= 350.0:
            delta = abs(log.heading_deg[right] - log.heading_deg[left])
            if delta <= 8.0:
                for index in range(left, right + 1):
                    straight[index] = True

    segments: list[tuple[int, int]] = []
    begin: int | None = None
    gap = 0
    for index, is_straight in enumerate(straight):
        if is_straight:
            if begin is None:
                begin = index
            gap = 0
        elif begin is not None:
            gap += 1
            if gap > 12:
                end = index - gap
                if log.distance_mm[end] - log.distance_mm[begin] >= 800.0:
                    segments.append((begin, end))
                begin = None
                gap = 0
    if begin is not None:
        end = n - 1 - gap
        if log.distance_mm[end] - log.distance_mm[begin] >= 800.0:
            segments.append((begin, end))
    return segments


def write_outputs(logs: list[LogData], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = output_dir / "logs_12384_12394_primary_pose_drift_summary.csv"
    segment_path = output_dir / "logs_12384_12394_primary_straight_segments.csv"
    fixed_path = output_dir / "logs_12384_12394_primary_fixed_straights.csv"

    with summary_path.open("w", encoding="utf-8-sig", newline="") as stream:
        fieldnames = ["log", "commit", "controller_version", "rows", "duration_s",
                      "distance_mm", "final_x_mm", "final_y_mm", "gyro_net_deg",
                      "gyro_closure_error_deg", "closure_bias_dps",
                      "bias_corrected_final_x_mm", "scale_corrected_final_x_mm",
                      "best_temp_slope_dps_per_c", "temp_corrected_final_x_mm",
                      "trapezoid_final_x_mm", "trapezoid_gyro_net_deg",
                      "temp_begin_c", "temp_end_c", "cntlog_wraps", "max_dt_ms",
                      "nonpositive_dt_count"]
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for log in logs:
            final = log.rows[-1]
            temps = [decode_temperature(row.get("imuTempRaw", 0.0)) for row in log.rows]
            temps = [value for value in temps if value is not None]
            net = log.heading_deg[-1]
            nearest_turn = round(net / 360.0) * 360.0
            closure_bias_dps = (nearest_turn - net) / (log.time_ms[-1] / 1000.0)
            bias_x, _, _ = integrate_pose(log, gyro_bias_dps=closure_bias_dps)
            scale_x, _, _ = integrate_pose(log, gyro_scale=nearest_turn / net)
            candidates = [value / 10000.0 for value in range(-1000, 1001)]
            best_temp_slope = min(candidates,
                                  key=lambda value: abs(integrate_pose(
                                      log, temp_slope_dps_per_c=value)[0]))
            temp_x, _, _ = integrate_pose(log, temp_slope_dps_per_c=best_temp_slope)
            trapezoid_x, _, trapezoid_heading = integrate_pose_trapezoid(log)
            intervals = [log.time_ms[index] - log.time_ms[index - 1]
                         for index in range(1, len(log.time_ms))]
            writer.writerow({
                "log": log.number,
                "commit": log.metadata.get("gitCommit", ""),
                "controller_version": log.metadata.get("routeControllerVersion", ""),
                "rows": len(log.rows),
                "duration_s": f"{log.time_ms[-1] / 1000.0:.3f}",
                "distance_mm": f"{log.distance_mm[-1]:.3f}",
                "final_x_mm": f"{final['x']:.3f}",
                "final_y_mm": f"{final['y']:.3f}",
                "gyro_net_deg": f"{net:.3f}",
                "gyro_closure_error_deg": f"{net - nearest_turn:.3f}",
                "closure_bias_dps": f"{closure_bias_dps:.6f}",
                "bias_corrected_final_x_mm": f"{bias_x:.3f}",
                "scale_corrected_final_x_mm": f"{scale_x:.3f}",
                "best_temp_slope_dps_per_c": f"{best_temp_slope:.6f}",
                "temp_corrected_final_x_mm": f"{temp_x:.3f}",
                "trapezoid_final_x_mm": f"{trapezoid_x:.3f}",
                "trapezoid_gyro_net_deg": f"{trapezoid_heading:.3f}",
                "temp_begin_c": f"{temps[0]:.3f}" if temps else "",
                "temp_end_c": f"{temps[-1]:.3f}" if temps else "",
                "cntlog_wraps": log.time_ms[-1] // 65536,
                "max_dt_ms": max(intervals),
                "nonpositive_dt_count": sum(value <= 0 for value in intervals),
            })

    all_segments: list[dict[str, float | int]] = []
    for log in logs:
        for order, (begin, end) in enumerate(find_long_straights(log), start=1):
            metrics: dict[str, float | int] = {"log": log.number, "segment": order}
            metrics.update(segment_metrics(log, begin, end))
            all_segments.append(metrics)
    with segment_path.open("w", encoding="utf-8-sig", newline="") as stream:
        fieldnames = list(all_segments[0]) if all_segments else ["log", "segment"]
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(all_segments)

    fixed_rows: list[dict[str, float | int | str]] = []
    for log in logs:
        for name, begin_mm, end_mm in FIXED_STRAIGHTS_MM:
            begin = min(range(len(log.distance_mm)), key=lambda i: abs(log.distance_mm[i] - begin_mm))
            end = min(range(len(log.distance_mm)), key=lambda i: abs(log.distance_mm[i] - end_mm))
            metrics: dict[str, float | int | str] = {"log": log.number, "straight": name}
            metrics.update(segment_metrics(log, begin, end))
            fixed_rows.append(metrics)
    with fixed_path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(fixed_rows[0]))
        writer.writeheader()
        writer.writerows(fixed_rows)

    plt.rcParams.update({"font.family": "Yu Gothic", "axes.unicode_minus": False})
    figure, axes = plt.subplots(1, 2, figsize=(15, 7), constrained_layout=True)
    for log in logs:
        x = [row["x"] for row in log.rows]
        y = [row["y"] for row in log.rows]
        axes[0].plot(x, y, linewidth=1.4, label=str(log.number))
        axes[1].plot(log.distance_mm, log.heading_deg, linewidth=1.3, label=str(log.number))
    axes[0].set_title("一次走行のXY軌跡")
    axes[0].set_xlabel("X [mm]")
    axes[0].set_ylabel("Y [mm]")
    axes[0].set_aspect("equal", adjustable="box")
    axes[0].grid(True, alpha=0.3)
    axes[1].set_title("ジャイロ積分角")
    axes[1].set_xlabel("累積距離 [mm]")
    axes[1].set_ylabel("積分角 [deg]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(ncol=2)
    figure.savefig(output_dir / "logs_12384_12394_primary_pose_drift.png", dpi=180)
    plt.close(figure)

    reference = next(log for log in logs if log.number == 12389)
    figure, axis = plt.subplots(figsize=(9, 9), constrained_layout=True)
    points = axis.scatter([row["x"] for row in reference.rows],
                          [row["y"] for row in reference.rows],
                          c=reference.distance_mm, s=5, cmap="turbo")
    next_label_mm = 0.0
    for index, distance_mm in enumerate(reference.distance_mm):
        if distance_mm >= next_label_mm:
            axis.annotate(f"{distance_mm / 1000.0:.0f} m",
                          (reference.rows[index]["x"], reference.rows[index]["y"]),
                          fontsize=8)
            next_label_mm += 2000.0
    axis.set_title("12389 XY軌跡と累積距離")
    axis.set_xlabel("X [mm]")
    axis.set_ylabel("Y [mm]")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, alpha=0.3)
    figure.colorbar(points, ax=axis, label="累積距離 [mm]")
    figure.savefig(output_dir / "log_12389_xy_distance_map.png", dpi=180)
    plt.close(figure)

    print(summary_path)
    print(segment_path)
    print(fixed_path)


def main() -> int:
    log_dir = Path(r"C:\Users\ucawa\Dropbox\Document\robotrace\Log\v2")
    logs = [read_log(log_dir / f"{number}.csv") for number in LOG_NUMBERS]
    write_outputs(logs, Path("analysis"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
