#!/usr/bin/env python3
"""ログ12439～12445で距離カルマン融合とXY閉路誤差を検証する。"""

from __future__ import annotations

import csv
import math
import os
from pathlib import Path

# Windowsのユーザーフォントキャッシュへ書かず、ワークスペース内へ限定する。
os.environ.setdefault("MPLCONFIGDIR", str(Path("analysis/.mplconfig").resolve()))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from robotrace_units import PULSE_MILLIMETER


LOG_NUMBERS = range(12439, 12446)
PRIMARY_LOG_NUMBERS = (12439, 12441, 12444)
LOG_DIR = Path(r"F:\Dropbox\Document\robotrace\Log\v2")
OUTPUT_DIR = Path("analysis/kalman_loop_closure_12439_12445")


def read_log(path: Path) -> tuple[dict[str, str], list[dict[str, float]], list[int]]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        metadata: dict[str, str] = {}
        for field in next(csv.reader([stream.readline()])):
            if "=" in field:
                key, value = field.split("=", 1)
                metadata[key] = value
        rows = [
            {
                key: float(value)
                for key, value in row.items()
                if key and value not in (None, "")
            }
            for row in csv.DictReader(stream)
            if row.get("cntlog", "") != ""
        ]

    times: list[int] = []
    wrap_offset = 0
    previous_raw: int | None = None
    for row in rows:
        raw = int(row["cntlog"])
        if previous_raw is not None and raw < previous_raw:
            wrap_offset += 65536
        times.append(raw + wrap_offset)
        previous_raw = raw
    return metadata, rows, times


def integrate_trajectory(
    rows: list[dict[str, float]],
    times_ms: list[int],
    pulse_column: str,
    gyro_bias_dps: float = 0.0,
    gyro_scale: float = 1.0,
) -> tuple[list[float], list[float], list[float], float]:
    x_values: list[float] = []
    y_values: list[float] = []
    headings_deg: list[float] = []
    x_mm = 0.0
    y_mm = 0.0
    heading_deg = 0.0
    distance_mm = 0.0
    previous_time_ms = 0
    for row, time_ms in zip(rows, times_ms):
        dt_s = (time_ms - previous_time_ms) / 1000.0
        previous_time_ms = time_ms
        heading_deg += (row["gyroVal_Z"] * gyro_scale - gyro_bias_dps) * dt_s
        velocity_mmps = row[pulse_column] / PULSE_MILLIMETER * 1000.0
        distance_step_mm = velocity_mmps * dt_s
        heading_rad = math.radians(heading_deg)
        x_mm += distance_step_mm * math.sin(heading_rad)
        y_mm += distance_step_mm * math.cos(heading_rad)
        distance_mm += distance_step_mm
        x_values.append(x_mm)
        y_values.append(y_mm)
        headings_deg.append(heading_deg)
    return x_values, y_values, headings_deg, distance_mm


def integrate_midpoint_trajectory(
    rows: list[dict[str, float]], times_ms: list[int], pulse_column: str
) -> tuple[list[float], list[float], list[float], float]:
    """速度・角速度を台形補間し、区間中点方位でXYを積分する。"""
    x_values: list[float] = []
    y_values: list[float] = []
    headings_deg: list[float] = []
    x_mm = 0.0
    y_mm = 0.0
    heading_deg = 0.0
    distance_mm = 0.0
    previous_time_ms = 0
    previous_speed_mmps = rows[0][pulse_column] / PULSE_MILLIMETER * 1000.0
    previous_gyro_dps = rows[0]["gyroVal_Z"]
    for row, time_ms in zip(rows, times_ms):
        dt_s = (time_ms - previous_time_ms) / 1000.0
        current_speed_mmps = row[pulse_column] / PULSE_MILLIMETER * 1000.0
        current_gyro_dps = row["gyroVal_Z"]
        distance_step_mm = 0.5 * (previous_speed_mmps + current_speed_mmps) * dt_s
        heading_step_deg = 0.5 * (previous_gyro_dps + current_gyro_dps) * dt_s
        midpoint_heading_rad = math.radians(heading_deg + 0.5 * heading_step_deg)
        x_mm += distance_step_mm * math.sin(midpoint_heading_rad)
        y_mm += distance_step_mm * math.cos(midpoint_heading_rad)
        heading_deg += heading_step_deg
        distance_mm += distance_step_mm
        x_values.append(x_mm)
        y_values.append(y_mm)
        headings_deg.append(heading_deg)
        previous_time_ms = time_ms
        previous_speed_mmps = current_speed_mmps
        previous_gyro_dps = current_gyro_dps
    return x_values, y_values, headings_deg, distance_mm


def find_bias_for_zero_x(
    rows: list[dict[str, float]], times_ms: list[int], pulse_column: str
) -> tuple[float, float]:
    """0deg/s近傍で終点X=0となる一定ジャイロバイアスを探索する。"""
    candidates = [(-5.0 + index * 0.005) for index in range(2001)]
    values = []
    for bias in candidates:
        x_values, _, _, _ = integrate_trajectory(rows, times_ms, pulse_column, bias)
        values.append(x_values[-1])

    roots: list[tuple[float, float]] = []
    for index in range(1, len(candidates)):
        if values[index - 1] == 0.0:
            roots.append((abs(candidates[index - 1]), candidates[index - 1]))
            continue
        if values[index - 1] * values[index] > 0.0:
            continue
        low = candidates[index - 1]
        high = candidates[index]
        low_value = values[index - 1]
        for _ in range(40):
            middle = 0.5 * (low + high)
            middle_x = integrate_trajectory(rows, times_ms, pulse_column, middle)[0][-1]
            if low_value * middle_x <= 0.0:
                high = middle
            else:
                low = middle
                low_value = middle_x
        root = 0.5 * (low + high)
        roots.append((abs(root), root))

    if roots:
        bias = min(roots)[1]
    else:
        best_index = min(range(len(candidates)), key=lambda index: abs(values[index]))
        bias = candidates[best_index]
    final_x = integrate_trajectory(rows, times_ms, pulse_column, bias)[0][-1]
    return bias, final_x


def find_scale_for_zero_x(
    rows: list[dict[str, float]], times_ms: list[int], pulse_column: str
) -> tuple[float, float, bool]:
    """1.0近傍で終点X=0となるジャイロ倍率を探索する。"""
    candidates = [(0.95 + index * 0.0001) for index in range(1001)]
    values = []
    for scale in candidates:
        x_values, _, _, _ = integrate_trajectory(
            rows, times_ms, pulse_column, gyro_scale=scale
        )
        values.append(x_values[-1])

    roots: list[tuple[float, float]] = []
    for index in range(1, len(candidates)):
        if values[index - 1] * values[index] > 0.0:
            continue
        low = candidates[index - 1]
        high = candidates[index]
        low_value = values[index - 1]
        for _ in range(40):
            middle = 0.5 * (low + high)
            middle_x = integrate_trajectory(
                rows, times_ms, pulse_column, gyro_scale=middle
            )[0][-1]
            if low_value * middle_x <= 0.0:
                high = middle
            else:
                low = middle
                low_value = middle_x
        root = 0.5 * (low + high)
        roots.append((abs(root - 1.0), root))

    if roots:
        scale = min(roots)[1]
        root_found = True
    else:
        best_index = min(range(len(candidates)), key=lambda index: abs(values[index]))
        scale = candidates[best_index]
        root_found = False
    final_x = integrate_trajectory(
        rows, times_ms, pulse_column, gyro_scale=scale
    )[0][-1]
    return scale, final_x, root_found


def nearest_full_turn_heading(heading_deg: float) -> float:
    return round(heading_deg / 360.0) * 360.0


def correlation(values_a: list[float], values_b: list[float]) -> float:
    if len(values_a) < 2 or len(values_a) != len(values_b):
        return math.nan
    mean_a = sum(values_a) / len(values_a)
    mean_b = sum(values_b) / len(values_b)
    covariance = sum((a - mean_a) * (b - mean_b) for a, b in zip(values_a, values_b))
    variance_a = sum((a - mean_a) ** 2 for a in values_a)
    variance_b = sum((b - mean_b) ** 2 for b in values_b)
    denominator = math.sqrt(variance_a * variance_b)
    return covariance / denominator if denominator > 0.0 else math.nan


def smoothed_accel_correlation(
    rows: list[dict[str, float]], times_ms: list[int], imu_column: str,
    straight_only: bool,
) -> float:
    """約100msの中央差分速度と移動平均IMU加速度の相関を求める。"""
    window = 5
    encoder_accel: list[float] = []
    imu_accel: list[float] = []
    for index in range(window, len(rows) - window):
        if straight_only and abs(rows[index]["gyroVal_Z"]) >= 20.0:
            continue
        dt_s = (times_ms[index + window] - times_ms[index - window]) / 1000.0
        if dt_s <= 0.0:
            continue
        speed_before = rows[index - window]["encCurrentN"] / PULSE_MILLIMETER
        speed_after = rows[index + window]["encCurrentN"] / PULSE_MILLIMETER
        encoder_accel.append((speed_after - speed_before) / dt_s)
        imu_accel.append(
            sum(row[imu_column] for row in rows[index - window:index + window + 1]) /
            (2 * window + 1)
        )
    return correlation(imu_accel, encoder_accel)


def summarize_log(number: int) -> tuple[dict[str, object], dict[str, list[float]]]:
    metadata, rows, times_ms = read_log(LOG_DIR / f"{number}.csv")
    intervals_ms = [times_ms[0]] + [
        times_ms[index] - times_ms[index - 1] for index in range(1, len(times_ms))
    ]
    fused_x, fused_y, fused_heading, fused_distance = integrate_trajectory(
        rows, times_ms, "encCurrentCorr_p"
    )
    raw_x, raw_y, raw_heading, raw_distance = integrate_trajectory(
        rows, times_ms, "encCurrentN"
    )
    midpoint_fused_x, midpoint_fused_y, midpoint_fused_heading, midpoint_fused_distance = (
        integrate_midpoint_trajectory(rows, times_ms, "encCurrentCorr_p")
    )
    stored_x = [row["x"] for row in rows]
    stored_y = [row["y"] for row in rows]
    reconstruction_error = [
        math.hypot(x - sx, y - sy)
        for x, y, sx, sy in zip(fused_x, fused_y, stored_x, stored_y)
    ]

    gyro_net_deg = fused_heading[-1]
    full_turn_target_deg = nearest_full_turn_heading(gyro_net_deg)
    duration_s = times_ms[-1] / 1000.0
    heading_closure_bias_dps = (
        (gyro_net_deg - full_turn_target_deg) / duration_s if duration_s > 0.0 else math.nan
    )
    heading_x, heading_y, heading_values, _ = integrate_trajectory(
        rows, times_ms, "encCurrentCorr_p", heading_closure_bias_dps
    )
    if number in PRIMARY_LOG_NUMBERS:
        zero_x_bias_dps, zero_x_residual_mm = find_bias_for_zero_x(
            rows, times_ms, "encCurrentCorr_p"
        )
        zero_x_x, zero_x_y, zero_x_heading, _ = integrate_trajectory(
            rows, times_ms, "encCurrentCorr_p", zero_x_bias_dps
        )
        zero_x_gyro_scale, zero_x_scale_residual_mm, zero_x_scale_root_found = find_scale_for_zero_x(
            rows, times_ms, "encCurrentCorr_p"
        )
        _, zero_x_scale_y, zero_x_scale_heading, _ = integrate_trajectory(
            rows, times_ms, "encCurrentCorr_p", gyro_scale=zero_x_gyro_scale
        )
    else:
        zero_x_bias_dps = math.nan
        zero_x_residual_mm = math.nan
        zero_x_x, zero_x_y, zero_x_heading = fused_x, fused_y, fused_heading
        zero_x_gyro_scale = math.nan
        zero_x_scale_residual_mm = math.nan
        zero_x_scale_root_found = False
        zero_x_scale_y, zero_x_scale_heading = fused_y, fused_heading

    raw_speeds_mps = [row["encCurrentN"] / PULSE_MILLIMETER for row in rows]
    fused_speeds_mps = [row["encCurrentCorr_p"] / PULSE_MILLIMETER for row in rows]
    imu_accel = [row["imuLinearAccelY_mps2"] for row in rows]
    encoder_accel: list[float] = []
    aligned_imu_accel: list[float] = []
    for index in range(1, len(rows)):
        dt_s = intervals_ms[index] / 1000.0
        if dt_s <= 0.0:
            continue
        encoder_accel.append((raw_speeds_mps[index] - raw_speeds_mps[index - 1]) / dt_s)
        aligned_imu_accel.append(imu_accel[index])

    valid_intervals = all(value > 0 for value in intervals_ms)
    maximum_interval_ms = max(intervals_ms)
    fused_minus_raw_mm = fused_distance - raw_distance
    speed_difference = [fused - raw for fused, raw in zip(fused_speeds_mps, raw_speeds_mps)]
    rms_speed_difference_mps = math.sqrt(
        sum(value * value for value in speed_difference) / len(speed_difference)
    )
    summary: dict[str, object] = {
        "log": number,
        "mode": int(float(metadata.get("optimalTrace", "-1"))),
        "auto_start": int(float(metadata.get("autoStart", "-1"))),
        "emc_stop": int(float(metadata.get("emcStop", "-1"))),
        "samples": len(rows),
        "duration_ms": times_ms[-1],
        "cntlog_valid": valid_intervals,
        "max_interval_ms": maximum_interval_ms,
        "battery_v": float(metadata.get("batteryVoltage_V", "nan")),
        "log_schema_version": int(metadata.get("logSchemaVersion", "-1")),
        "log_record_size_bytes": int(metadata.get("logRecordSizeBytes", "-1")),
        "debug_overflow": int(metadata.get("dbgOverflowFinal", "-1")),
        "log_overflow": int(metadata.get("logOverflowFinal", "-1")),
        "kalman_reject": int(metadata.get("distanceKalman.innovationRejectCount", "-1")),
        "kalman_fallback": int(metadata.get("distanceKalman.fallbackCount", "-1")),
        "kalman_invalid": int(metadata.get("distanceKalman.invalidUpdateCount", "-1")),
        "kalman_guard": int(metadata.get("distanceKalman.outputGuardCount", "-1")),
        "fused_distance_mm": fused_distance,
        "raw_distance_mm": raw_distance,
        "fused_minus_raw_mm": fused_minus_raw_mm,
        "fused_raw_speed_rms_mps": rms_speed_difference_mps,
        "stored_final_x_mm": stored_x[-1],
        "stored_final_y_mm": stored_y[-1],
        "reintegrated_fused_final_x_mm": fused_x[-1],
        "reintegrated_fused_final_y_mm": fused_y[-1],
        "reintegrated_raw_final_x_mm": raw_x[-1],
        "reintegrated_raw_final_y_mm": raw_y[-1],
        "midpoint_fused_final_x_mm": midpoint_fused_x[-1],
        "midpoint_fused_final_y_mm": midpoint_fused_y[-1],
        "midpoint_fused_distance_mm": midpoint_fused_distance,
        "midpoint_gyro_net_deg": midpoint_fused_heading[-1],
        "reconstruction_max_error_mm": max(reconstruction_error),
        "gyro_net_deg": gyro_net_deg,
        "full_turn_target_deg": full_turn_target_deg,
        "heading_closure_bias_dps": heading_closure_bias_dps,
        "heading_closed_final_x_mm": heading_x[-1],
        "heading_closed_final_y_mm": heading_y[-1],
        "zero_x_bias_dps": zero_x_bias_dps,
        "zero_x_residual_mm": zero_x_residual_mm,
        "zero_x_final_y_mm": zero_x_y[-1],
        "zero_x_final_heading_deg": zero_x_heading[-1],
        "zero_x_gyro_scale": zero_x_gyro_scale,
        "zero_x_scale_residual_mm": zero_x_scale_residual_mm,
        "zero_x_scale_root_found": zero_x_scale_root_found,
        "zero_x_scale_final_y_mm": zero_x_scale_y[-1],
        "zero_x_scale_final_heading_deg": zero_x_scale_heading[-1],
        "imu_encoder_accel_correlation": correlation(aligned_imu_accel, encoder_accel),
        "imu_y_encoder_accel_smoothed_correlation": smoothed_accel_correlation(
            rows, times_ms, "imuLinearAccelY_mps2", False
        ),
        "imu_y_encoder_accel_straight_correlation": smoothed_accel_correlation(
            rows, times_ms, "imuLinearAccelY_mps2", True
        ),
        "imu_x_encoder_accel_smoothed_correlation": smoothed_accel_correlation(
            rows, times_ms, "imuLinearAccelX_mps2", False
        ),
        "imu_x_encoder_accel_straight_correlation": smoothed_accel_correlation(
            rows, times_ms, "imuLinearAccelX_mps2", True
        ),
    }
    series = {
        "stored_x": stored_x,
        "stored_y": stored_y,
        "fused_x": fused_x,
        "fused_y": fused_y,
        "raw_x": raw_x,
        "raw_y": raw_y,
        "midpoint_fused_x": midpoint_fused_x,
        "midpoint_fused_y": midpoint_fused_y,
        "heading_x": heading_x,
        "heading_y": heading_y,
        "zero_x_x": zero_x_x,
        "zero_x_y": zero_x_y,
    }
    return summary, series


def main() -> int:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    results = [summarize_log(number) for number in LOG_NUMBERS]
    summaries = [summary for summary, _ in results]
    with (OUTPUT_DIR / "logs_12439_12445_summary.csv").open(
        "w", encoding="utf-8-sig", newline=""
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)

    primary = {
        int(summary["log"]): (summary, series)
        for summary, series in results
        if int(summary["log"]) in PRIMARY_LOG_NUMBERS
    }
    primary_scales = {
        number: float(primary[number][0]["zero_x_gyro_scale"])
        for number in PRIMARY_LOG_NUMBERS
    }
    primary_biases = {
        number: float(primary[number][0]["zero_x_bias_dps"])
        for number in PRIMARY_LOG_NUMBERS
    }
    common_scale = sum(primary_scales.values()) / len(primary_scales)
    common_bias = sum(primary_biases.values()) / len(primary_biases)
    method_rows: list[dict[str, object]] = []
    for number in PRIMARY_LOG_NUMBERS:
        _, rows, times_ms = read_log(LOG_DIR / f"{number}.csv")
        leave_one_out_scale = sum(
            scale for log, scale in primary_scales.items() if log != number
        ) / (len(primary_scales) - 1)
        leave_one_out_bias = sum(
            bias for log, bias in primary_biases.items() if log != number
        ) / (len(primary_biases) - 1)
        common_bias_x = integrate_trajectory(
            rows, times_ms, "encCurrentCorr_p", gyro_bias_dps=common_bias
        )[0][-1]
        leave_one_out_bias_x = integrate_trajectory(
            rows, times_ms, "encCurrentCorr_p", gyro_bias_dps=leave_one_out_bias
        )[0][-1]
        common_scale_x = integrate_trajectory(
            rows, times_ms, "encCurrentCorr_p", gyro_scale=common_scale
        )[0][-1]
        leave_one_out_scale_x = integrate_trajectory(
            rows, times_ms, "encCurrentCorr_p", gyro_scale=leave_one_out_scale
        )[0][-1]
        method_rows.append({
            "log": number,
            "baseline_x_mm": float(primary[number][0]["stored_final_x_mm"]),
            "midpoint_final_x_mm": float(primary[number][0]["midpoint_fused_final_x_mm"]),
            "per_run_bias_dps": primary_biases[number],
            "common_bias_dps": common_bias,
            "common_bias_final_x_mm": common_bias_x,
            "leave_one_out_bias_dps": leave_one_out_bias,
            "leave_one_out_bias_final_x_mm": leave_one_out_bias_x,
            "per_run_gyro_scale": primary_scales[number],
            "common_gyro_scale": common_scale,
            "common_scale_final_x_mm": common_scale_x,
            "leave_one_out_gyro_scale": leave_one_out_scale,
            "leave_one_out_scale_final_x_mm": leave_one_out_scale_x,
            "linear_endpoint_warp_final_x_mm": 0.0,
        })
    with (OUTPUT_DIR / "primary_closure_methods.csv").open(
        "w", encoding="utf-8-sig", newline=""
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=list(method_rows[0]))
        writer.writeheader()
        writer.writerows(method_rows)

    plt.rcParams.update({"font.family": "Yu Gothic", "axes.unicode_minus": False})
    figure, axes = plt.subplots(1, 3, figsize=(18, 6), constrained_layout=True)
    for number in PRIMARY_LOG_NUMBERS:
        summary, series = primary[number]
        axes[0].plot(series["stored_x"], series["stored_y"], label=str(number))
        axes[1].plot(series["fused_x"], series["fused_y"], label=f"{number} 融合")
        axes[1].plot(
            series["raw_x"], series["raw_y"], linestyle="--", alpha=0.75,
            label=f"{number} 生enc",
        )
        axes[2].plot(series["fused_x"], series["fused_y"], alpha=0.35)
        axes[2].plot(
            series["zero_x_x"], series["zero_x_y"],
            label=f"{number}: bias={float(summary['zero_x_bias_dps']):+.3f} deg/s",
        )

    axes[0].set_title("保存済み一次走行XY")
    axes[1].set_title("同一ジャイロで融合距離/生encを比較")
    axes[2].set_title("一定ジャイロバイアスで終点X=0")
    for axis in axes:
        axis.set_xlabel("X [mm]")
        axis.set_ylabel("Y [mm]")
        axis.set_aspect("equal", adjustable="box")
        axis.grid(True, alpha=0.3)
        axis.legend(fontsize=8)
        axis.scatter([0.0], [0.0], color="black", marker="x", zorder=10)
    figure.savefig(OUTPUT_DIR / "primary_loop_closure_comparison.png", dpi=180)
    plt.close(figure)

    for summary in summaries:
        print(
            f"{summary['log']}: mode={summary['mode']} emc={summary['emc_stop']} "
            f"X={float(summary['stored_final_x_mm']):.1f}mm "
            f"rawX={float(summary['reintegrated_raw_final_x_mm']):.1f}mm "
            f"fused-raw={float(summary['fused_minus_raw_mm']):+.1f}mm "
            f"gyro={float(summary['gyro_net_deg']):.2f}deg "
            f"biasX0={float(summary['zero_x_bias_dps']):+.4f}deg/s"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
