#!/usr/bin/env python3
"""四輪スキッドステアの距離校正・旋回診断・横滑り候補をオフライン評価する。

横加速度が約10 ms間隔の瞬時値として保存されたログは、1 ms制御を再現しない。
横速度はエンコーダ2個とジャイロだけでは観測できないため、本スクリプトの
横滑り軌跡は外部基準で比較するための候補であり、走行制御へは反映しない。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from dataclasses import dataclass
from pathlib import Path

try:
    from .path_log_recovery import read_csv_log
    from .robotrace_units import pulse_meter_for_log
except ImportError:
    from path_log_recovery import read_csv_log
    from robotrace_units import pulse_meter_for_log


DEFAULT_OUTPUT_DIR = Path("analysis/skid_odometry")
PHYSICAL_TREAD_MM = 109.0
MIN_TURN_DEG = 1.0


def _number(value: str, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name}: finite number required")
    return result


def _percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def _read_run(path: Path, required: set[str]):
    log = read_csv_log(path)
    missing = sorted(required - set(log.fields))
    if missing:
        raise ValueError(f"{path.name}: missing columns: {', '.join(missing)}")
    for name in ("emcStop", "logOverflowFinal", "dbgOverflowFinal"):
        if name not in log.parameters:
            raise ValueError(f"{path.name}: missing {name} metadata")
        if _number(log.parameters[name], name) != 0:
            raise ValueError(f"{path.name}: invalid {name}={log.parameters[name]}")
    if "logExpectedRows" in log.parameters:
        expected = int(_number(log.parameters["logExpectedRows"], "logExpectedRows"))
        if expected != len(log.rows):
            raise ValueError(f"{path.name}: expected {expected} rows, got {len(log.rows)}")
    previous_ms = 0
    for row in log.rows:
        current_ms = int(_number(row["cntlog"], "cntlog"))
        if not 0 < current_ms - previous_ms <= 20:
            raise ValueError(f"{path.name}: invalid cntlog interval at {current_ms} ms")
        previous_ms = current_ms
    return log


def check_distance(path: Path, nominal_ppm: float) -> dict:
    """動力1 m測定のパルス値またはUI距離表示を検証する。"""
    if not math.isfinite(nominal_ppm) or nominal_ppm <= 0:
        raise ValueError("nominal ppm must be finite and positive")
    trials = []
    with path.open("r", encoding="utf-8-sig", newline="") as source:
        reader = csv.DictReader(source)
        fields = set(reader.fieldnames or [])
        required = {"run_id", "actual_mm", "powered"}
        pulse_fields = {"left_p", "right_p"}
        display_fields = {"left_start_mm", "left_end_mm", "right_start_mm", "right_end_mm"}
        delta_fields = {"left_delta_mm", "right_delta_mm"}
        if not required.issubset(fields):
            raise ValueError(f"measurement CSV requires {sorted(required)}")
        use_pulses = pulse_fields.issubset(fields)
        use_display = display_fields.issubset(fields)
        use_delta = delta_fields.issubset(fields)
        if sum((use_pulses, use_display, use_delta)) != 1:
            raise ValueError("choose pulse, start/end display, or display-delta columns")
        for row in reader:
            if row["powered"].strip().lower() not in {"1", "true", "yes"}:
                raise ValueError(f"run {row['run_id']}: powered measurement required")
            actual = _number(row["actual_mm"], "actual_mm")
            if use_pulses:
                left = _number(row["left_p"], "left_p")
                right = _number(row["right_p"], "right_p")
                display_uncertainty_mm = 0.0
                source_name = "encoder_pulses"
            elif use_display:
                left_display_mm = (_number(row["left_end_mm"], "left_end_mm")
                                   - _number(row["left_start_mm"], "left_start_mm"))
                right_display_mm = (_number(row["right_end_mm"], "right_end_mm")
                                    - _number(row["right_start_mm"], "right_start_mm"))
                left = left_display_mm * nominal_ppm / 1000.0
                right = right_display_mm * nominal_ppm / 1000.0
                # 前後2回の1 mm丸め表示差には各側で最大±1 mmの不確かさがある。
                display_uncertainty_mm = 1.0
                source_name = "rounded_ui_mm"
            else:
                left_display_mm = _number(row["left_delta_mm"], "left_delta_mm")
                right_display_mm = _number(row["right_delta_mm"], "right_delta_mm")
                left = left_display_mm * nominal_ppm / 1000.0
                right = right_display_mm * nominal_ppm / 1000.0
                # 画面の前後表示を引いた値として、両端の丸めを最大±1 mmで扱う。
                display_uncertainty_mm = 1.0
                source_name = "rounded_ui_delta_mm"
            if not 900 <= actual <= 1100 or min(left, right) <= 0:
                raise ValueError(f"run {row['run_id']}: expected forward travel near 1 m")
            estimated = (left + right) * 500.0 / nominal_ppm
            left_estimated = left * 1000.0 / nominal_ppm
            right_estimated = right * 1000.0 / nominal_ppm
            trials.append({
                "run_id": row["run_id"],
                "actual_mm": actual,
                "measurement_source": source_name,
                "display_quantization_bound_mm": display_uncertainty_mm,
                "estimated_mm": estimated,
                "error_percent": 100.0 * (estimated - actual) / actual,
                "left_error_percent": 100.0 * (left_estimated - actual) / actual,
                "right_error_percent": 100.0 * (right_estimated - actual) / actual,
                "forward_worst_case_abs_error_percent": (
                    abs(estimated - actual) + display_uncertainty_mm
                ) * 100.0 / actual,
                "side_worst_case_abs_error_percent": (
                    max(abs(left_estimated - actual), abs(right_estimated - actual))
                    + display_uncertainty_mm
                ) * 100.0 / actual,
                "left_ppm": left * 1000.0 / actual,
                "right_ppm": right * 1000.0 / actual,
            })
    if len(trials) < 3 or len({trial["run_id"] for trial in trials}) != len(trials):
        raise ValueError("three or more distinct powered runs are required")
    return {
        "nominal_ppm": nominal_ppm,
        "left_ppm_median": statistics.median(t["left_ppm"] for t in trials),
        "right_ppm_median": statistics.median(t["right_ppm"] for t in trials),
        "observed_scale_within_one_percent": all(
            abs(t["error_percent"]) <= 1.0 for t in trials),
        "side_observed_within_one_percent": all(
            max(abs(t["left_error_percent"]), abs(t["right_error_percent"])) <= 1.0
            for t in trials),
        "distance_scale_verified": all(t["forward_worst_case_abs_error_percent"] <= 1.0
                                       for t in trials),
        "side_scale_verified": all(t["side_worst_case_abs_error_percent"] <= 1.0
                                   for t in trials),
        "trials": trials,
    }


def _side_scales(metadata: dict[str, str], common_ppm: float) -> tuple[float, float, str]:
    if metadata.get("headingCalibration.enabled") == "1":
        try:
            left = _number(metadata["headingCalibration.pulsePerMeterL"], "left ppm")
            right = _number(metadata["headingCalibration.pulsePerMeterR"], "right ppm")
            if all(abs(side / common_ppm - 1.0) <= 0.02 for side in (left, right)):
                return left, right, "enabled per-side calibration"
        except KeyError:
            pass
    return common_ppm, common_ppm, "common log scale; per-side calibration not validated"


def turn_sample(left_p: float, right_p: float, gyro_dps: float,
                dt_s: float, left_ppm: float, right_ppm: float) -> tuple[float, float]:
    """CW正の角速度に対する診断用有効トレッド[mm]と前進速度[m/s]。"""
    left_mm = left_p * 1000.0 / left_ppm
    right_mm = right_p * 1000.0 / right_ppm
    delta_heading_rad = math.radians(gyro_dps * dt_s)
    return (left_mm - right_mm) / delta_heading_rad, (left_mm + right_mm) * 0.0005 / dt_s


def diagnose_turns(path: Path, side_ppm: tuple[float, float] | None = None) -> dict:
    log = _read_run(path, {"cntlog", "gyroVal_Z", "encIntervalL_p", "encIntervalR_p"})
    if int(_number(log.parameters.get("logSchemaVersion", "0"), "schema")) < 9:
        raise ValueError(f"{path.name}: interval encoders require schema 9+")
    if _number(log.parameters.get("optimalTrace", "nan"), "optimalTrace") != 0:
        raise ValueError(f"{path.name}: use primary-run logs for turn calibration")
    common_ppm = pulse_meter_for_log(log.parameters)
    if side_ppm is None:
        left_ppm, right_ppm, scale_source = _side_scales(log.parameters, common_ppm)
    else:
        left_ppm, right_ppm = side_ppm
        if any(not math.isfinite(value) or value <= 0 or
               abs(value / common_ppm - 1.0) > 0.02 for value in side_ppm):
            raise ValueError(f"{path.name}: side ppm must be positive and within 2% of log scale")
        scale_source = "explicit provisional side scale; not logged calibration"
    groups: dict[tuple[str, str, str], list[float]] = {}
    accepted = 0
    previous_ms = 0
    for row in log.rows:
        time_ms = int(row["cntlog"])
        dt = (time_ms - previous_ms) * 0.001
        gyro = _number(row["gyroVal_Z"], "gyroVal_Z")
        delta_deg = gyro * dt
        if abs(delta_deg) >= MIN_TURN_DEG:
            tread, speed = turn_sample(
                _number(row["encIntervalL_p"], "encIntervalL_p"),
                _number(row["encIntervalR_p"], "encIntervalR_p"),
                gyro, dt, left_ppm, right_ppm,
            )
            if math.isfinite(tread) and 0.3 <= speed <= 10.0 and 20 <= tread <= 400:
                speed_band = "0.3-1" if speed < 1 else "1-2" if speed < 2 else "2-4" if speed < 4 else "4-10"
                radius_mm = speed * 1000.0 / abs(math.radians(gyro))
                radius_band = "<200" if radius_mm < 200 else "200-500" if radius_mm < 500 else "500-1000" if radius_mm < 1000 else ">=1000"
                direction = "CW" if gyro > 0 else "CCW"
                groups.setdefault((speed_band, radius_band, direction), []).append(tread)
                accepted += 1
        previous_ms = time_ms
    bins = [
        {"speed_mps": speed, "radius_mm": radius, "turn": direction, "samples": len(values),
         "tread_median_mm": statistics.median(values),
         "tread_p10_mm": _percentile(values, 0.1),
         "tread_p90_mm": _percentile(values, 0.9)}
        for (speed, radius, direction), values in sorted(groups.items())
    ]
    return {
        "log": str(path), "common_ppm": common_ppm,
        "side_scale_source": scale_source, "left_ppm": left_ppm,
        "right_ppm": right_ppm, "physical_tread_mm": PHYSICAL_TREAD_MM,
        "samples_used": accepted, "samples_total": len(log.rows),
        "closure_valid": log.parameters.get("closureValid", "unknown"),
        "bins": bins,
        "note": "diagnostic only; do not replace gyro heading with encoder yaw",
    }


@dataclass
class LateralState:
    x_mm: float = 0.0
    y_mm: float = 0.0
    heading_deg: float = 0.0
    lateral_mps: float = 0.0


def advance_pose(state: LateralState, delta_forward_mm: float, gyro_dps: float,
                 lateral_accel_mps2: float, dt_s: float, bias_mps2: float,
                 time_constant_s: float, max_lateral_mps: float) -> tuple[float, float, float]:
    """瞬時横加速度を区間代表値とする限定的な候補。右方向とCWを正とする。"""
    forward_mps = delta_forward_mm * 0.001 / dt_s
    omega = math.radians(gyro_dps)
    residual = lateral_accel_mps2 - bias_mps2 - omega * forward_mps
    state.lateral_mps += residual * dt_s
    state.lateral_mps *= math.exp(-dt_s / time_constant_s)
    state.lateral_mps = max(-max_lateral_mps, min(max_lateral_mps, state.lateral_mps))
    delta_lateral_mm = state.lateral_mps * dt_s * 1000.0
    middle = math.radians(state.heading_deg + 0.5 * gyro_dps * dt_s)
    state.x_mm += delta_forward_mm * math.sin(middle) + delta_lateral_mm * math.cos(middle)
    state.y_mm += delta_forward_mm * math.cos(middle) - delta_lateral_mm * math.sin(middle)
    state.heading_deg += gyro_dps * dt_s
    return residual, state.x_mm, state.y_mm


def _reference(path: Path) -> list[tuple[int, float, float]]:
    with path.open("r", encoding="utf-8-sig", newline="") as source:
        reader = csv.DictReader(source)
        if not {"cntlog", "x_mm", "y_mm"}.issubset(reader.fieldnames or []):
            raise ValueError("reference requires cntlog,x_mm,y_mm")
        points = [(int(_number(r["cntlog"], "reference cntlog")),
                   _number(r["x_mm"], "reference x"),
                   _number(r["y_mm"], "reference y")) for r in reader]
    if len(points) < 2 or any(b[0] <= a[0] for a, b in zip(points, points[1:])):
        raise ValueError("reference needs two or more strictly increasing timestamps")
    return points


def replay_lateral(path: Path, output_dir: Path, reference_path: Path | None,
                   time_constant_s: float, max_lateral_mps: float) -> dict:
    log = _read_run(path, {"cntlog", "encTotalOptimal", "gyroVal_Z", "imuLinearAccelX_mps2"})
    if _number(log.parameters.get("optimalTrace", "nan"), "optimalTrace") != 0:
        raise ValueError(f"{path.name}: first-run log required; secondary x/y may contain line correction")
    if (not math.isfinite(time_constant_s) or not math.isfinite(max_lateral_mps)
            or time_constant_s <= 0 or max_lateral_mps <= 0):
        raise ValueError("time constant and lateral speed cap must be positive")
    ppm = pulse_meter_for_log(log.parameters)
    parsed = []
    previous_time = 0
    previous_pulse = 0
    straight_accel = []
    for row in log.rows:
        time_ms = int(row["cntlog"])
        pulse = int(_number(row["encTotalOptimal"], "encTotalOptimal"))
        dt_s = (time_ms - previous_time) * 0.001
        if pulse < previous_pulse:
            raise ValueError(f"{path.name}: negative distance increment at {time_ms} ms")
        delta_mm = (pulse - previous_pulse) * 1000.0 / ppm
        gyro = _number(row["gyroVal_Z"], "gyroVal_Z")
        lateral_accel = _number(row["imuLinearAccelX_mps2"], "imuLinearAccelX_mps2")
        if abs(gyro) < 20 and delta_mm / (1000 * dt_s) >= 0.3:
            straight_accel.append(lateral_accel)
        parsed.append((time_ms, dt_s, delta_mm, gyro, lateral_accel))
        previous_time, previous_pulse = time_ms, pulse
    if len(straight_accel) < 30:
        raise ValueError(f"{path.name}: insufficient straight samples for lateral bias estimate")
    bias = statistics.median(straight_accel)
    reference = _reference(reference_path) if reference_path else None
    if reference and (reference[0][0] > parsed[0][0] or reference[-1][0] < parsed[-1][0]):
        raise ValueError("reference must cover the whole log; timestamps share cntlog origin")
    baseline = LateralState()
    candidate = LateralState()
    output_dir.mkdir(parents=True, exist_ok=True)
    trajectory = output_dir / f"{path.stem}_lateral_candidate.csv"
    baseline_errors = []
    candidate_errors = []
    max_lateral_seen = 0.0
    ref_index = 0
    with trajectory.open("w", encoding="utf-8", newline="") as target:
        writer = csv.writer(target)
        writer.writerow(["cntlog", "baseline_x_mm", "baseline_y_mm", "candidate_x_mm",
                         "candidate_y_mm", "heading_deg", "lateral_mps", "residual_mps2",
                         "reference_x_mm", "reference_y_mm"])
        for time_ms, dt_s, delta_mm, gyro, accel in parsed:
            # 基準軌跡は横速度ゼロ。向心加速度に一致する入力で残差をゼロにする。
            baseline_expected_accel = math.radians(gyro) * delta_mm * 0.001 / dt_s
            advance_pose(baseline, delta_mm, gyro, baseline_expected_accel,
                         dt_s, 0.0, time_constant_s, max_lateral_mps)
            residual, _, _ = advance_pose(candidate, delta_mm, gyro, accel,
                                          dt_s, bias, time_constant_s, max_lateral_mps)
            max_lateral_seen = max(max_lateral_seen, abs(candidate.lateral_mps))
            rx = ry = ""
            if reference:
                while ref_index + 1 < len(reference) - 1 and reference[ref_index + 1][0] < time_ms:
                    ref_index += 1
                first, second = reference[ref_index:ref_index + 2]
                fraction = (time_ms - first[0]) / (second[0] - first[0])
                rx = first[1] + fraction * (second[1] - first[1])
                ry = first[2] + fraction * (second[2] - first[2])
                baseline_errors.append(math.hypot(baseline.x_mm - rx, baseline.y_mm - ry))
                candidate_errors.append(math.hypot(candidate.x_mm - rx, candidate.y_mm - ry))
            writer.writerow([time_ms, baseline.x_mm, baseline.y_mm,
                             candidate.x_mm, candidate.y_mm, candidate.heading_deg,
                             candidate.lateral_mps, residual, rx, ry])
    summary = {
        "log": str(path), "trajectory": str(trajectory), "samples": len(parsed),
        "pulse_per_m": ppm, "sampled_acceleration_is_interval_mean": False,
        "lateral_bias_mps2": bias, "straight_bias_samples": len(straight_accel),
        "time_constant_s": time_constant_s, "max_lateral_mps": max_lateral_mps,
        "max_abs_candidate_lateral_mps": max_lateral_seen,
        "baseline_end_mm": [baseline.x_mm, baseline.y_mm],
        "candidate_end_mm": [candidate.x_mm, candidate.y_mm],
        "reference_validated": reference is not None,
        "note": "10 ms sampled acceleration is an offline approximation, not 1 ms firmware replay",
    }
    if {"x", "y"}.issubset(log.fields):
        stored_x = _number(log.rows[-1]["x"], "stored x")
        stored_y = _number(log.rows[-1]["y"], "stored y")
        summary["logged_end_mm"] = [stored_x, stored_y]
        summary["baseline_logged_end_gap_mm"] = math.hypot(
            baseline.x_mm - stored_x, baseline.y_mm - stored_y)
    if reference:
        summary["baseline_position_error_p95_mm"] = _percentile(baseline_errors, 0.95)
        summary["candidate_position_error_p95_mm"] = _percentile(candidate_errors, 0.95)
        summary["baseline_final_error_mm"] = baseline_errors[-1]
        summary["candidate_final_error_mm"] = candidate_errors[-1]
    summary_path = output_dir / f"{path.stem}_lateral_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    distance = sub.add_parser("distance", help="動力1 m計測CSVを検証")
    distance.add_argument("measurements", type=Path)
    distance.add_argument("--nominal-ppm", type=float, default=58019.0)
    distance.add_argument("--output", type=Path)
    turn = sub.add_parser("turn", help="スキーマ9+一次ログの有効トレッド診断")
    turn.add_argument("logs", nargs="+", type=Path)
    turn.add_argument("--left-ppm", type=float)
    turn.add_argument("--right-ppm", type=float)
    turn.add_argument("--output", type=Path)
    lateral = sub.add_parser("lateral", help="横加速度を持つ一次ログの候補軌跡")
    lateral.add_argument("log", type=Path)
    lateral.add_argument("--reference", type=Path)
    lateral.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    lateral.add_argument("--time-constant-s", type=float, default=0.15)
    lateral.add_argument("--max-lateral-mps", type=float, default=0.5)
    args = parser.parse_args()
    if args.command == "distance":
        result = check_distance(args.measurements, args.nominal_ppm)
    elif args.command == "turn":
        if (args.left_ppm is None) != (args.right_ppm is None):
            parser.error("--left-ppm and --right-ppm must be provided together")
        side_ppm = None if args.left_ppm is None else (args.left_ppm, args.right_ppm)
        result = [diagnose_turns(p, side_ppm) for p in args.logs]
    else:
        result = replay_lateral(args.log, args.output_dir, args.reference,
                                args.time_constant_s, args.max_lateral_mps)
    rendered = json.dumps(result, indent=2, ensure_ascii=False) + "\n"
    if getattr(args, "output", None):
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")


if __name__ == "__main__":
    main()
