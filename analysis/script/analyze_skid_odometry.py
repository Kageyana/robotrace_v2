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
    from .robotrace_units import CURRENT_PULSE_METER, pulse_meter_for_log
except ImportError:
    from path_log_recovery import read_csv_log
    from robotrace_units import CURRENT_PULSE_METER, pulse_meter_for_log


DEFAULT_OUTPUT_DIR = Path("analysis/skid_odometry")
PHYSICAL_TREAD_MM = 109.0
MIN_TURN_DEG = 1.0
MIN_TURN_RATE_DPS = 5.0
MIN_SPEED_MPS = 0.3
MAX_SPEED_MPS = 10.0
MAX_CNTLOG_DELTA_MS = 0x7FFF
MIN_TIME_OFFSET_SEGMENTS = 5


def _number(value: str, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name}: finite number required")
    return result


def _integer(value: str, name: str) -> int:
    result = _number(value, name)
    if not result.is_integer():
        raise ValueError(f"{name}: integer required")
    return int(result)


def _percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def _read_run(path: Path, required: set[str], *, strict_rows: bool = False):
    log = read_csv_log(path, strict_rows=strict_rows)
    missing = sorted(required - set(log.fields))
    if missing:
        raise ValueError(f"{path.name}: missing columns: {', '.join(missing)}")
    for name in ("emcStop", "logOverflowFinal", "dbgOverflowFinal"):
        if name not in log.parameters:
            raise ValueError(f"{path.name}: missing {name} metadata")
        if _number(log.parameters[name], name) != 0:
            raise ValueError(f"{path.name}: invalid {name}={log.parameters[name]}")
    if "logExpectedRows" in log.parameters:
        expected = _integer(log.parameters["logExpectedRows"], "logExpectedRows")
        if expected != len(log.rows):
            raise ValueError(f"{path.name}: expected {expected} rows, got {len(log.rows)}")
    if "cntlog" in required:
        _cntlog_deltas_ms(log, path.name)
    return log


def _cntlog_deltas_ms(log, label: str) -> list[int]:
    """cntlogのU16折り返しを展開し、連続する正の区間時間を返す。"""
    deltas = []
    previous = 0
    for row in log.rows:
        current = _integer(row["cntlog"], "cntlog")
        if not 0 <= current <= 0xFFFF:
            raise ValueError(f"{label}: cntlog outside U16 range: {current}")
        delta = (current - previous) & 0xFFFF
        if not 0 < delta <= MAX_CNTLOG_DELTA_MS:
            raise ValueError(f"{label}: invalid or ambiguous cntlog interval at {current} ms")
        deltas.append(delta)
        previous = current
    return deltas


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
                ppm_rounding_bound = 0.0
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
                ppm_rounding_bound = nominal_ppm / actual
                source_name = "rounded_ui_mm"
            else:
                left_display_mm = _number(row["left_delta_mm"], "left_delta_mm")
                right_display_mm = _number(row["right_delta_mm"], "right_delta_mm")
                left = left_display_mm * nominal_ppm / 1000.0
                right = right_display_mm * nominal_ppm / 1000.0
                # 画面の前後表示を引いた値として、両端の丸めを最大±1 mmで扱う。
                display_uncertainty_mm = 1.0
                ppm_rounding_bound = nominal_ppm / actual
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
                "side_ppm_display_rounding_bound": ppm_rounding_bound,
                "measurement_conditions": {
                    name: (row.get(name) or "").strip() for name in reader.fieldnames
                    if name is not None
                    if name not in {"run_id", "actual_mm", "powered", "left_p", "right_p",
                                    "left_start_mm", "left_end_mm", "right_start_mm",
                                    "right_end_mm", "left_delta_mm", "right_delta_mm"}
                    and (row.get(name) or "").strip()
                },
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
        "measurement_source": trials[0]["measurement_source"],
        "measurement_conditions": [trial["measurement_conditions"] for trial in trials],
        "display_rounding_model": {
            "display_resolution_mm": 1.0,
            "per_side_delta_bound_mm": 1.0 if trials[0]["measurement_source"].startswith("rounded_ui") else 0.0,
            "actual_distance_uncertainty_included": False,
        },
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
        left = _number(metadata["headingCalibration.pulsePerMeterL"], "left ppm")
        right = _number(metadata["headingCalibration.pulsePerMeterR"], "right ppm")
        if not all(abs(side / common_ppm - 1.0) <= 0.02 for side in (left, right)):
            raise ValueError("enabled heading calibration side ppm must be within 2% of common scale")
        return left, right, "existing enabled per-side calibration (diagnostic reference)"
    return common_ppm, common_ppm, "common log scale; per-side calibration not validated"


def turn_sample(left_p: float, right_p: float, gyro_dps: float,
                dt_s: float, left_ppm: float, right_ppm: float) -> tuple[float, float]:
    """CW正の角速度に対する診断用有効トレッド[mm]と前進速度[m/s]。"""
    if dt_s <= 0 or left_ppm <= 0 or right_ppm <= 0:
        raise ValueError("dt and pulse-per-meter values must be positive")
    left_mm = left_p * 1000.0 / left_ppm
    right_mm = right_p * 1000.0 / right_ppm
    delta_heading_rad = math.radians(gyro_dps * dt_s)
    return (left_mm - right_mm) / delta_heading_rad, (left_mm + right_mm) * 0.0005 / dt_s


def _speed_band(speed_mps: float) -> str:
    if speed_mps < 1.0:
        return "0.3-1"
    if speed_mps < 2.0:
        return "1-2"
    if speed_mps < 4.0:
        return "2-4"
    return "4-10"


def _fit_time_proportional_offset(segments: list[dict]) -> dict | None:
    """Δs=T*Δθ+q*Δt の補助回帰を残差診断用に計算する。"""
    if len(segments) < MIN_TIME_OFFSET_SEGMENTS or \
            {segment["turn"] for segment in segments} != {"CW", "CCW"}:
        return None
    xx = xz = zz = xy = zy = 0.0
    for segment in segments:
        theta = segment["gyro_angle_rad"]
        duration = segment["duration_s"]
        delta_side = segment["left_mm"] - segment["right_mm"]
        xx += theta * theta
        xz += theta * duration
        zz += duration * duration
        xy += theta * delta_side
        zy += duration * delta_side
    determinant = xx * zz - xz * xz
    if determinant <= max(1e-12, xx * zz * 1e-10):
        return None
    tread = (xy * zz - zy * xz) / determinant
    q_mmps = (zy * xx - xy * xz) / determinant
    if not math.isfinite(tread) or not math.isfinite(q_mmps):
        return None
    deltas_side = [s["left_mm"] - s["right_mm"] for s in segments]
    predicted = [tread * s["gyro_angle_rad"] + q_mmps * s["duration_s"]
                 for s in segments]
    mean_delta = statistics.mean(deltas_side)
    residual_sum_squares = sum((actual - estimate) ** 2
                               for actual, estimate in zip(deltas_side, predicted))
    total_sum_squares = sum((actual - mean_delta) ** 2 for actual in deltas_side)
    residual_rms_mm = math.sqrt(residual_sum_squares / len(segments))
    r_squared = (1.0 - residual_sum_squares / total_sum_squares
                 if total_sum_squares > 0 else None)
    return {
        "q_mmps": q_mmps,
        "residual_rms_mm": residual_rms_mm,
        "r_squared": r_squared,
        "segments": len(segments),
    }


def diagnose_turns(path: Path, side_ppm: tuple[float, float] | None = None) -> dict:
    log = _read_run(path, {"cntlog", "gyroVal_Z", "encIntervalL_p", "encIntervalR_p"},
                    strict_rows=True)
    required_metadata = (
        "logSchemaVersion", "optimalTrace", "closureValid", "closureReason",
        "distanceScaleVerified", "encoderPulsePerMeter", "imuCalibrationValid",
        "imuCalibrationSamples", "imuCalibrationReadErrors", "gyroSampleFault",
        "encoderIntervalFault", "logExpectedRows", "emcStop", "logOverflowFinal",
        "dbgOverflowFinal", "headingCalibration.enabled",
        "headingCalibration.pulsePerMeterL", "headingCalibration.pulsePerMeterR",
        "headingCalibration.effectiveTread_mm",
    )
    missing_metadata = [name for name in required_metadata if name not in log.parameters]
    if missing_metadata:
        raise ValueError(f"{path.name}: missing metadata: {', '.join(missing_metadata)}")
    expected_checks = {
        "logSchemaVersion": 10,
        "optimalTrace": 0,
        "closureValid": 1,
        "closureReason": 0,
        "distanceScaleVerified": 1,
        "encoderPulsePerMeter": int(CURRENT_PULSE_METER),
        "imuCalibrationValid": 1,
        "imuCalibrationSamples": 100,
        "imuCalibrationReadErrors": 0,
        "gyroSampleFault": 0,
        "encoderIntervalFault": 0,
        "emcStop": 0,
        "logOverflowFinal": 0,
        "dbgOverflowFinal": 0,
    }
    for name, expected in expected_checks.items():
        actual = _number(log.parameters[name], name)
        if actual != expected:
            raise ValueError(f"{path.name}: invalid {name}={log.parameters[name]} (expected {expected})")
    if _number(log.parameters["headingCalibration.enabled"], "headingCalibration.enabled") not in (0, 1):
        raise ValueError(f"{path.name}: invalid headingCalibration.enabled")
    calibration_reference = {
        "headingCalibration.pulsePerMeterL": _number(
            log.parameters["headingCalibration.pulsePerMeterL"], "headingCalibration.pulsePerMeterL"),
        "headingCalibration.pulsePerMeterR": _number(
            log.parameters["headingCalibration.pulsePerMeterR"], "headingCalibration.pulsePerMeterR"),
        "headingCalibration.effectiveTread_mm": _number(
            log.parameters["headingCalibration.effectiveTread_mm"], "headingCalibration.effectiveTread_mm"),
    }
    if any(value <= 0 for value in calibration_reference.values()):
        raise ValueError(f"{path.name}: invalid heading calibration reference values")
    expected_rows = _integer(log.parameters["logExpectedRows"], "logExpectedRows")
    if expected_rows <= 0 or expected_rows != len(log.rows):
        raise ValueError(f"{path.name}: expected {expected_rows} rows, got {len(log.rows)}")
    common_ppm = pulse_meter_for_log(log.parameters)
    if common_ppm != CURRENT_PULSE_METER:
        raise ValueError(f"{path.name}: unsupported pulse scale {common_ppm}")
    if side_ppm is None:
        left_ppm, right_ppm, scale_source = _side_scales(log.parameters, common_ppm)
    else:
        left_ppm, right_ppm = side_ppm
        if any(not math.isfinite(value) or value <= 0 or
               abs(value / common_ppm - 1.0) > 0.02 for value in side_ppm):
            raise ValueError(f"{path.name}: side ppm must be positive and within 2% of log scale")
        scale_source = "explicit provisional side scale; not logged calibration"
    deltas_ms = _cntlog_deltas_ms(log, path.name)
    segments: list[dict] = []
    rejected_intervals: dict[str, int] = {}
    rejected_segments: dict[str, int] = {}
    active: dict | None = None

    def count_rejection(target: dict[str, int], reason: str, count: int = 1) -> None:
        target[reason] = target.get(reason, 0) + count

    def finish_segment() -> None:
        nonlocal active
        if active is None:
            return
        if abs(active["gyro_angle_rad"]) < math.radians(MIN_TURN_DEG):
            count_rejection(rejected_segments, "net_rotation_below_1deg")
        else:
            delta_side = active["left_mm"] - active["right_mm"]
            raw_tread = delta_side / active["gyro_angle_rad"]
            if not math.isfinite(raw_tread) or not 20.0 <= raw_tread <= 400.0:
                count_rejection(rejected_segments, "raw_tread_outside_20_to_400_mm")
            else:
                active["raw_tread_mm"] = raw_tread
                active["speed_mps"] = (active["left_mm"] + active["right_mm"]) * 0.0005 / active["duration_s"]
                active["mean_sample_speed_mps"] = active["speed_sum_mps"] / active["samples"]
                segments.append(active)
        active = None

    for row, delta_ms in zip(log.rows, deltas_ms):
        dt = delta_ms * 0.001
        gyro = _number(row["gyroVal_Z"], "gyroVal_Z")
        left_mm = _number(row["encIntervalL_p"], "encIntervalL_p") * 1000.0 / left_ppm
        right_mm = _number(row["encIntervalR_p"], "encIntervalR_p") * 1000.0 / right_ppm
        speed = (left_mm + right_mm) * 0.0005 / dt
        if not MIN_SPEED_MPS <= speed <= MAX_SPEED_MPS:
            finish_segment()
            count_rejection(rejected_intervals, "speed_outside_0.3_to_10_mps")
            continue
        if abs(gyro) < MIN_TURN_RATE_DPS:
            finish_segment()
            count_rejection(rejected_intervals, "gyro_rate_below_5_dps")
            continue
        direction = "CW" if gyro > 0 else "CCW"
        band = _speed_band(speed)
        if active is not None and (active["turn"] != direction or active["speed_band"] != band):
            finish_segment()
        if active is None:
            active = {"turn": direction, "speed_band": band, "left_mm": 0.0,
                      "right_mm": 0.0, "gyro_angle_rad": 0.0, "duration_s": 0.0,
                      "samples": 0, "speed_sum_mps": 0.0}
        active["left_mm"] += left_mm
        active["right_mm"] += right_mm
        active["gyro_angle_rad"] += math.radians(gyro * dt)
        active["duration_s"] += dt
        active["samples"] += 1
        active["speed_sum_mps"] += speed
    finish_segment()

    by_speed: dict[str, list[dict]] = {}
    for segment in segments:
        by_speed.setdefault(segment["speed_band"], []).append(segment)
    time_proportional_offsets = {}
    for speed_band, speed_segments in by_speed.items():
        offset_diagnostic = _fit_time_proportional_offset(speed_segments)
        if offset_diagnostic is not None:
            time_proportional_offsets[speed_band] = offset_diagnostic

    groups: dict[tuple[str, str], list[dict]] = {}
    for segment in segments:
        groups.setdefault((segment["speed_band"], segment["turn"]), []).append(segment)
    bins = []
    for (speed_band, direction), values in sorted(groups.items()):
        raw_fit = sum(s["gyro_angle_rad"] * (s["left_mm"] - s["right_mm"]) for s in values) / \
            sum(s["gyro_angle_rad"] ** 2 for s in values)
        yaw_errors = [math.degrees(s["gyro_angle_rad"] -
                                   (s["left_mm"] - s["right_mm"]) / raw_fit)
                      for s in values]
        radii = [s["speed_mps"] * 1000.0 / abs(s["gyro_angle_rad"] / s["duration_s"])
                 for s in values]
        bins.append({
            "speed_mps": speed_band,
            "mean_speed_mps": statistics.mean(s["mean_sample_speed_mps"] for s in values),
            "turn": direction,
            "segments": len(values),
            "samples": sum(s["samples"] for s in values),
            "tread_fit_mm": raw_fit,
            "tread_median_mm": statistics.median(s["raw_tread_mm"] for s in values),
            "tread_p10_mm": _percentile([s["raw_tread_mm"] for s in values], 0.1),
            "tread_p90_mm": _percentile([s["raw_tread_mm"] for s in values], 0.9),
            "gyro_angle_minus_encoder_angle_median_deg": statistics.median(yaw_errors),
            "gyro_angle_minus_encoder_angle_p95_abs_deg": _percentile([abs(x) for x in yaw_errors], 0.95),
            "gyro_minus_encoder_rate_mean_dps": sum(yaw_errors) /
            sum(s["duration_s"] for s in values),
            "mean_turn_radius_mm": statistics.mean(radii),
            "icr_candidate_left_mm": -raw_fit * 0.5,
            "icr_candidate_right_mm": raw_fit * 0.5,
        })

    heading_reference = {}
    for key in ("headingCalibration.enabled", "headingCalibration.pulsePerMeterL",
                "headingCalibration.pulsePerMeterR", "headingCalibration.effectiveTread_mm"):
        value = log.parameters.get(key)
        if value is not None:
            try:
                heading_reference[key] = int(value) if key.endswith("enabled") else float(value)
            except ValueError:
                heading_reference[key] = value
    existing_tread = heading_reference.get("headingCalibration.effectiveTread_mm")
    for item in bins:
        item["difference_from_heading_calibration_effective_tread_mm"] = (
            item["tread_fit_mm"] - existing_tread if isinstance(existing_tread, (int, float)) else None
        )
    return {
        "log": str(path), "common_ppm": common_ppm,
        "run_conditions": {name: log.parameters.get(name) for name in (
            "fwVersion", "gitCommit", "branch", "batteryVoltage_V", "logRecordSizeBytes",
        )},
        "side_scale_source": scale_source, "left_ppm": left_ppm,
        "right_ppm": right_ppm, "physical_tread_mm": PHYSICAL_TREAD_MM,
        "samples_used": sum(s["samples"] for s in segments), "samples_total": len(log.rows),
        "segments_used": len(segments),
        "input_checks": {name: log.parameters[name] for name in required_metadata},
        "cntlog_interval_ms": {"min": min(deltas_ms), "max": max(deltas_ms),
                               "median": statistics.median(deltas_ms),
                               "u16_wraps": sum(1 for a, b in zip(log.rows, log.rows[1:])
                                                if int(b["cntlog"]) < int(a["cntlog"]))},
        "excluded_intervals": rejected_intervals,
        "rejected_segments": rejected_segments,
        "time_proportional_offset_by_speed_band": time_proportional_offsets,
        "heading_calibration_reference_only": heading_reference,
        "side_scale_difference_from_heading_calibration_ppm": {
            "left": (left_ppm - heading_reference["headingCalibration.pulsePerMeterL"]
                     if isinstance(heading_reference.get("headingCalibration.pulsePerMeterL"), (int, float)) else None),
            "right": (right_ppm - heading_reference["headingCalibration.pulsePerMeterR"]
                      if isinstance(heading_reference.get("headingCalibration.pulsePerMeterR"), (int, float)) else None),
        },
        "closure_valid": log.parameters.get("closureValid", "unknown"),
        "bins": bins,
        "note": "offline diagnosis only; uncorrected tread and symmetric ICR candidates are not adopted or written to heading_cal.txt; the time-proportional offset is a residual diagnostic, not a physical gyro-bias estimate",
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
    elapsed_ms = 0
    previous_pulse = 0
    straight_accel = []
    for row, delta_ms in zip(log.rows, _cntlog_deltas_ms(log, path.name)):
        elapsed_ms += delta_ms
        time_ms = elapsed_ms
        pulse = int(_number(row["encTotalOptimal"], "encTotalOptimal"))
        dt_s = delta_ms * 0.001
        if pulse < previous_pulse:
            raise ValueError(f"{path.name}: negative distance increment at {time_ms} ms")
        delta_mm = (pulse - previous_pulse) * 1000.0 / ppm
        gyro = _number(row["gyroVal_Z"], "gyroVal_Z")
        lateral_accel = _number(row["imuLinearAccelX_mps2"], "imuLinearAccelX_mps2")
        if abs(gyro) < 20 and delta_mm / (1000 * dt_s) >= 0.3:
            straight_accel.append(lateral_accel)
        parsed.append((time_ms, dt_s, delta_mm, gyro, lateral_accel))
        previous_pulse = pulse
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
    turn = sub.add_parser("turn", help="正常なschema 10一次ログの有効トレッド・ICR診断")
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
