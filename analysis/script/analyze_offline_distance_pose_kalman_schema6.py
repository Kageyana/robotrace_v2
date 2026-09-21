#!/usr/bin/env python3
"""Schema 6の加速度ログで、10ms距離KFとジャイロXY積分を比較する。

ログのエンコーダ速度・IMU加速度は約10msごとの瞬時値であり、実機の
1ms距離KFを厳密に再生するものではない。左右エンコーダも1ms瞬時値なので、
区間の独立した方位観測としては使用しない。
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

from path_log_recovery import read_csv_log
from robotrace_units import PULSE_METER


DEFAULT_LOG_DIR = Path(r"F:\Dropbox\Document\robotrace\Log\v2")
DEFAULT_OUTPUT = Path("analysis/offline_distance_pose_kalman_schema6.csv")
DEFAULT_LOGS = (12439, 12441, 12444)
SIGMA_ACCEL_MPS2 = 1.5
SIGMA_ENCODER_MPS = 0.08
BIAS_RW_MPS2_SQRT_S = 0.05
INITIAL_BIAS_SIGMA_MPS2 = 0.5


def distance_kf_step(
    state: list[float], covariance: list[list[float]],
    dt: float, acceleration: float, encoder_speed: float,
) -> tuple[list[float], list[list[float]], bool]:
    """実機と同じ[s,v,b]モデルをログ間隔dtに拡張して1回更新する。"""
    dt2 = dt * dt
    acceleration = max(-20.0, min(20.0, acceleration))
    effective = acceleration - state[2]
    predicted = [
        state[0] + state[1] * dt + 0.5 * effective * dt2,
        max(-10.0, min(10.0, state[1] + effective * dt)),
        state[2],
    ]
    transition = (
        (1.0, dt, -0.5 * dt2),
        (0.0, 1.0, -dt),
        (0.0, 0.0, 1.0),
    )
    noise = (0.5 * dt2, dt, 0.0)
    predicted_covariance = [
        [
            sum(
                transition[i][k] * covariance[k][l] * transition[j][l]
                for k in range(3) for l in range(3)
            ) + SIGMA_ACCEL_MPS2**2 * noise[i] * noise[j]
            + (BIAS_RW_MPS2_SQRT_S**2 * dt if i == j == 2 else 0.0)
            for j in range(3)
        ]
        for i in range(3)
    ]
    encoder_variance = SIGMA_ENCODER_MPS**2
    innovation_variance = predicted_covariance[1][1] + encoder_variance
    innovation = encoder_speed - predicted[1]
    if abs(innovation) > 4.0 * math.sqrt(innovation_variance):
        # 予測のみ採用して共分散を再初期化する。1ms実機の棄却数とは比較しない。
        reset_covariance = [
            [0.0, 0.0, 0.0],
            [0.0, encoder_variance, 0.0],
            [0.0, 0.0, INITIAL_BIAS_SIGMA_MPS2**2],
        ]
        return predicted, reset_covariance, True
    gain = [predicted_covariance[i][1] / innovation_variance for i in range(3)]
    updated = [predicted[i] + gain[i] * innovation for i in range(3)]
    left = [
        [(1.0 if i == j else 0.0) - (gain[i] if j == 1 else 0.0)
         for j in range(3)]
        for i in range(3)
    ]
    joseph = [
        [
            sum(left[i][k] * predicted_covariance[k][l] * left[j][l]
                for k in range(3) for l in range(3))
            + gain[i] * encoder_variance * gain[j]
            for j in range(3)
        ]
        for i in range(3)
    ]
    return updated, joseph, False


def analyze_log(path: Path) -> dict[str, object]:
    source = read_csv_log(path)
    required = {
        "cntlog", "encCurrentN", "encTotalOptimal", "gyroVal_Z",
        "imuLinearAccelY_mps2", "x", "y",
    }
    if not required.issubset(source.fields):
        raise ValueError(f"{path.name}: missing {sorted(required - set(source.fields))}")
    metadata = source.parameters
    if int(metadata.get("logSchemaVersion", "-1")) != 6:
        raise ValueError(f"{path.name}: requires schema 6")
    for name, expected in (
        ("optimalTrace", 0), ("emcStop", 0),
        ("logOverflowFinal", 0), ("dbgOverflowFinal", 0),
        ("distanceKalman.invalidUpdateCount", 0),
        ("distanceKalman.outputGuardCount", 0),
    ):
        if int(float(metadata.get(name, "nan"))) != expected:
            raise ValueError(f"{path.name}: invalid {name}={metadata.get(name)}")

    # スキーマ6の履歴換算値。新形式のpulse/mを適用しない。
    ppm = PULSE_METER
    first_speed = float(source.rows[0]["encCurrentN"]) * 1000.0 / ppm
    state = [0.0, first_speed, 0.0]
    covariance = [
        [0.0, 0.0, 0.0],
        [0.0, SIGMA_ENCODER_MPS**2, 0.0],
        [0.0, 0.0, INITIAL_BIAS_SIGMA_MPS2**2],
    ]
    previous_time = 0
    previous_pulse = 0
    previous_kf_distance_m = 0.0
    gyro_heading_deg = 0.0
    xy = {name: [0.0, 0.0] for name in ("logged_fused", "sampled_encoder", "offline_kf")}
    raw_distance_m = 0.0
    max_kf_recorded_delta_mm = 0.0
    offline_rejects = 0
    for row in source.rows:
        time_ms = int(row["cntlog"])
        dt = (time_ms - previous_time) * 0.001
        if not 0.0 < dt <= 0.020:
            raise ValueError(f"{path.name}: invalid cntlog interval {dt * 1000:.1f} ms")
        acceleration = float(row["imuLinearAccelY_mps2"])
        gyro = float(row["gyroVal_Z"])
        pulse = int(row["encTotalOptimal"])
        encoder_speed = float(row["encCurrentN"]) * 1000.0 / ppm
        if not all(math.isfinite(value) for value in (acceleration, gyro, encoder_speed)):
            raise ValueError(f"{path.name}: nonfinite sensor value")
        state, covariance, rejected = distance_kf_step(
            state, covariance, dt, acceleration, encoder_speed
        )
        offline_rejects += int(rejected)
        raw_delta_m = encoder_speed * dt
        raw_distance_m += raw_delta_m
        deltas_m = {
            "logged_fused": (pulse - previous_pulse) / ppm,
            "sampled_encoder": raw_delta_m,
            "offline_kf": state[0] - previous_kf_distance_m,
        }
        gyro_heading_deg += gyro * dt
        heading_rad = math.radians(gyro_heading_deg)
        for name, delta_m in deltas_m.items():
            xy[name][0] += 1000.0 * delta_m * math.sin(heading_rad)
            xy[name][1] += 1000.0 * delta_m * math.cos(heading_rad)
        max_kf_recorded_delta_mm = max(
            max_kf_recorded_delta_mm, abs(state[0] * 1000.0 - pulse * 1000.0 / ppm)
        )
        previous_time = time_ms
        previous_pulse = pulse
        previous_kf_distance_m = state[0]

    recorded_distance_mm = previous_pulse * 1000.0 / ppm
    return {
        "log": path.stem,
        "schema": 6,
        "rows": len(source.rows),
        "duration_ms": previous_time,
        "pulse_per_m": ppm,
        "battery_v": metadata.get("batteryVoltage_V", ""),
        "recorded_fused_distance_mm": round(recorded_distance_mm, 3),
        "sampled_encoder_distance_mm": round(raw_distance_m * 1000.0, 3),
        "offline_kf_distance_mm": round(state[0] * 1000.0, 3),
        "offline_kf_minus_recorded_mm": round(state[0] * 1000.0 - recorded_distance_mm, 3),
        "max_offline_kf_recorded_gap_mm": round(max_kf_recorded_delta_mm, 3),
        "offline_kf_rejects": offline_rejects,
        "stored_x_mm": round(float(source.rows[-1]["x"]), 3),
        "recorded_fused_x_mm": round(xy["logged_fused"][0], 3),
        "sampled_encoder_x_mm": round(xy["sampled_encoder"][0], 3),
        "offline_kf_x_mm": round(xy["offline_kf"][0], 3),
        "offline_kf_minus_recorded_x_mm": round(
            xy["offline_kf"][0] - xy["logged_fused"][0], 3
        ),
        "stored_y_mm": round(float(source.rows[-1]["y"]), 3),
        "recorded_fused_y_mm": round(xy["logged_fused"][1], 3),
        "sampled_encoder_y_mm": round(xy["sampled_encoder"][1], 3),
        "offline_kf_y_mm": round(xy["offline_kf"][1], 3),
        "gyro_heading_deg": round(gyro_heading_deg, 3),
        "independent_heading_observations": 0,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("numbers", nargs="*", type=int, default=list(DEFAULT_LOGS))
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    results = [analyze_log(args.log_dir / f"{number}.csv") for number in args.numbers]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8-sig", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=list(results[0]))
        writer.writeheader()
        writer.writerows(results)
    for row in results:
        print(row)


if __name__ == "__main__":
    main()
