#!/usr/bin/env python3
"""独立カメラXY基準で現行オドメトリと横ICR候補をオフライン比較する。"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from pathlib import Path

import numpy as np

try:
    from .camera_xy_reference import LogTimeline, _read_log, _wrap_degrees
except ImportError:
    from camera_xy_reference import LogTimeline, _read_log, _wrap_degrees


WINDOW_DURATION_S = 0.1
MIN_WINDOW_ANGLE_DEG = 5.0
MIN_SIGNAL_TO_NOISE = 3.0
MIN_FIT_WINDOWS = 10
SPEED_BANDS = ((0.3, 1.0, "0.3-1"), (1.0, 2.0, "1-2"),
               (2.0, 4.0, "2-4"), (4.0, 10.0, "4-10"))


def speed_band(speed_mps: float) -> str | None:
    for low, high, name in SPEED_BANDS:
        if low <= speed_mps < high or (name == "4-10" and speed_mps == high):
            return name
    return None


def _read_reference(path: Path, diagnostics_path: Path) -> tuple[list[dict], float, dict]:
    try:
        diagnostics = json.loads(diagnostics_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"{diagnostics_path}: invalid camera diagnostics") from exc
    if diagnostics.get("reference_valid") is not True:
        raise ValueError(f"{diagnostics_path}: camera reference failed its quality gates")
    tracking = diagnostics["tracking"]
    if tracking.get("camera_motion_mode") == "register":
        motion_error = tracking.get("registration_error_max_mm")
        if motion_error is None:
            raise ValueError(f"{diagnostics_path}: registered reference lacks verification error")
    else:
        motion_error = tracking.get("camera_motion_p95_mm") or 0.0
    accuracy = max(
        float(diagnostics["calibration"].get("check_max_mm",
                                            diagnostics["calibration"]["check_rms_mm"])),
        float(motion_error),
        1e-6,
    )
    with path.open("r", encoding="utf-8-sig", newline="") as source:
        reader = csv.DictReader(source)
        required = {"time_ms", "x_mm", "y_mm", "heading_cw_deg", "track_valid"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"{path}: requires columns {sorted(required)}")
        rows = []
        previous_time = -math.inf
        for line_number, row in enumerate(reader, start=2):
            time_ms = float(row["time_ms"])
            if not math.isfinite(time_ms) or time_ms <= previous_time:
                raise ValueError(f"{path}: line {line_number}: time_ms must increase")
            previous_time = time_ms
            valid = row["track_valid"] == "1"
            if valid:
                values = [float(row[name]) for name in ("x_mm", "y_mm", "heading_cw_deg")]
                if not all(math.isfinite(value) for value in values):
                    raise ValueError(f"{path}: line {line_number}: invalid tracked pose")
                x_mm, y_mm, heading = values
            else:
                if any(row[name].strip() for name in ("x_mm", "y_mm", "heading_cw_deg")):
                    raise ValueError(f"{path}: line {line_number}: invalid rows must remain blank")
                x_mm = y_mm = heading = None
            rows.append({"time_ms": time_ms, "x_mm": x_mm, "y_mm": y_mm,
                         "heading_cw_deg": heading, "valid": valid})
    if len(rows) < 2:
        raise ValueError(f"{path}: at least two camera rows required")
    return rows, accuracy, diagnostics


def _timeline_arrays(timeline: LogTimeline) -> dict[str, np.ndarray]:
    times = np.asarray([0.0, *timeline.time_ms], dtype=np.float64)
    pulse = np.asarray([0.0, *timeline.pulse], dtype=np.float64)
    x = np.asarray([0.0, *[float(row["x"]) for row in timeline.rows]], dtype=np.float64)
    y = np.asarray([0.0, *[float(row["y"]) for row in timeline.rows]], dtype=np.float64)
    gyro = np.asarray([0.0, *[float(row["gyroVal_Z"]) for row in timeline.rows]], dtype=np.float64)
    heading = np.zeros(len(times), dtype=np.float64)
    for index in range(1, len(times)):
        heading[index] = heading[index - 1] + math.radians(gyro[index]) * \
            ((times[index] - times[index - 1]) * 0.001)
    return {"time_ms": times, "pulse": pulse, "x_mm": x, "y_mm": y,
            "gyro_dps": gyro, "heading_rad": heading}


def _interpolate(array_times: np.ndarray, values: np.ndarray, time_ms: float) -> float:
    if time_ms < array_times[0] - 1e-9 or time_ms > array_times[-1] + 1e-9:
        raise ValueError("requested timestamp is outside log coverage")
    return float(np.interp(time_ms, array_times, values))


def _turn_windows(reference: list[dict], timeline: LogTimeline, accuracy_mm: float,
                  nominal_frame_period_ms: float) -> tuple[list[dict], dict[str, int]]:
    arrays = _timeline_arrays(timeline)
    valid = [index for index, row in enumerate(reference)
             if row["valid"] and 0.0 <= row["time_ms"] <= arrays["time_ms"][-1]]
    rejected = {"track_gap_or_long_gap": 0, "short_turn": 0, "low_signal": 0,
                "speed_outside_or_undefined": 0}
    windows = []
    max_gap = nominal_frame_period_ms * 1.5
    cursor = 0
    while cursor < len(valid) - 1:
        first_index = valid[cursor]
        first = reference[first_index]
        target_time = first["time_ms"] + WINDOW_DURATION_S * 1000.0
        end_cursor = cursor + 1
        gap_failed = False
        while end_cursor < len(valid):
            previous = reference[valid[end_cursor - 1]]
            current = reference[valid[end_cursor]]
            if current["time_ms"] - previous["time_ms"] > max_gap:
                gap_failed = True
                break
            if current["time_ms"] >= target_time:
                break
            end_cursor += 1
        if gap_failed or end_cursor >= len(valid):
            rejected["track_gap_or_long_gap"] += 1
            cursor += 1
            continue
        end_index = valid[end_cursor]
        last = reference[end_index]
        dt_s = (last["time_ms"] - first["time_ms"]) * 0.001
        heading_values = np.unwrap(np.radians([
            reference[valid[item]]["heading_cw_deg"] for item in range(cursor, end_cursor + 1)
        ]))
        delta_theta = float(heading_values[-1] - heading_values[0])
        if abs(math.degrees(delta_theta)) < MIN_WINDOW_ANGLE_DEG:
            rejected["short_turn"] += 1
            cursor = end_cursor
            continue
        mid_heading = (heading_values[0] + heading_values[-1]) * 0.5
        dx = last["x_mm"] - first["x_mm"]
        dy = last["y_mm"] - first["y_mm"]
        lateral_mm = dx * math.cos(mid_heading) - dy * math.sin(mid_heading)
        lateral_uncertainty_mm = math.sqrt(2.0) * accuracy_mm
        if abs(lateral_mm) < MIN_SIGNAL_TO_NOISE * lateral_uncertainty_mm:
            rejected["low_signal"] += 1
            cursor = end_cursor
            continue
        pulse_first = _interpolate(arrays["time_ms"], arrays["pulse"], first["time_ms"])
        pulse_last = _interpolate(arrays["time_ms"], arrays["pulse"], last["time_ms"])
        pulse_per_meter = float(timeline.metadata["encoderPulsePerMeter"])
        forward_mm = (pulse_last - pulse_first) * 1000.0 / pulse_per_meter
        speed = forward_mm / dt_s / 1000.0
        band = speed_band(speed)
        if band is None:
            rejected["speed_outside_or_undefined"] += 1
            cursor = end_cursor
            continue
        windows.append({
            "speed_band": band,
            "turn": "CW" if delta_theta > 0 else "CCW",
            "delta_theta_rad": delta_theta,
            "lateral_delta_mm": lateral_mm,
            "duration_s": dt_s,
            "speed_mps": speed,
            "signal_to_noise": abs(lateral_mm) / lateral_uncertainty_mm,
            "first_time_ms": first["time_ms"],
            "last_time_ms": last["time_ms"],
        })
        cursor = end_cursor
    return windows, rejected


def fit_k_lat_for_runs(runs: list[dict]) -> dict:
    """5本の学習走行から速度帯・旋回方向別K_latを推定する。"""
    if len(runs) != 5:
        raise ValueError(f"exactly five training runs required; got {len(runs)}")
    grouped: dict[tuple[str, str], list[dict]] = {}
    run_summaries = []
    all_run_ids = set()
    for run in runs:
        run_id = str(run["run_id"])
        if run_id in all_run_ids:
            raise ValueError(f"duplicate run_id: {run_id}")
        all_run_ids.add(run_id)
        timeline = _read_log(Path(run["log"]))
        reference, accuracy, diagnostics = _read_reference(
            Path(run["reference_csv"]), Path(run["diagnostics_json"]))
        frame_period = float(diagnostics["video"]["nominal_frame_period_ms"])
        windows, rejected = _turn_windows(reference, timeline, accuracy, frame_period)
        for window in windows:
            grouped.setdefault((window["speed_band"], window["turn"]), []).append(window)
        run_summaries.append({"run_id": run_id, "log": str(run["log"]),
                              "accepted_windows": len(windows),
                              "rejected_windows": rejected})
    coefficients = {}
    for (band, turn), windows in sorted(grouped.items()):
        if len(windows) < MIN_FIT_WINDOWS:
            continue
        thetas = np.asarray([window["delta_theta_rad"] for window in windows])
        lateral = np.asarray([window["lateral_delta_mm"] for window in windows])
        denominator = float(np.dot(thetas, thetas))
        if denominator <= 1e-12:
            continue
        k_lat = float(np.dot(thetas, lateral) / denominator)
        residual = lateral - k_lat * thetas
        total = lateral - float(np.mean(lateral))
        residual_rms = float(np.sqrt(np.mean(residual * residual)))
        r_squared = (1.0 - float(np.dot(residual, residual) / np.dot(total, total))
                     if float(np.dot(total, total)) > 0 else None)
        coefficients.setdefault(band, {})[turn] = {
            "k_lat_mm_per_rad": k_lat,
            "segments": len(windows),
            "residual_rms_mm": residual_rms,
            "r_squared": r_squared,
            "signal_to_noise_min": min(window["signal_to_noise"] for window in windows),
            "signal_to_noise_median": statistics.median(
                window["signal_to_noise"] for window in windows),
        }
    if not coefficients:
        raise ValueError("no K_lat bin has enough above-noise training windows")
    return {"model_type": "offline lateral ICR coefficient",
            "window_duration_s": WINDOW_DURATION_S,
            "minimum_window_angle_deg": MIN_WINDOW_ANGLE_DEG,
            "minimum_signal_to_noise": MIN_SIGNAL_TO_NOISE,
            "minimum_fit_windows_per_bin": MIN_FIT_WINDOWS,
            "coefficients": coefficients,
            "training_run_ids": sorted(all_run_ids),
            "training_runs": run_summaries,
            "note": "diagnostic candidate only; do not update firmware or SD settings"}


def _candidate_trajectory(timeline: LogTimeline, model: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    arrays = _timeline_arrays(timeline)
    times = [0.0]
    xs = [0.0]
    ys = [0.0]
    heading = 0.0
    previous_time = 0.0
    previous_pulse = 0.0
    x, y = 0.0, 0.0
    for row, time, pulse in zip(timeline.rows, timeline.time_ms, timeline.pulse):
        dt_s = (time - previous_time) * 0.001
        ds_mm = (pulse - previous_pulse) * 1000.0 / float(timeline.metadata["encoderPulsePerMeter"])
        speed = ds_mm / dt_s / 1000.0
        omega = float(row["gyroVal_Z"])
        dtheta = math.radians(omega) * dt_s
        band = speed_band(speed)
        direction = "CW" if dtheta > 0 else "CCW"
        coeff = model.get("coefficients", {}).get(band or "", {}).get(direction)
        k_lat = 0.0 if coeff is None or abs(omega) < 5.0 else float(coeff["k_lat_mm_per_rad"])
        lateral_mm = k_lat * dtheta
        mid_heading = heading + 0.5 * dtheta
        x += ds_mm * math.sin(mid_heading) + lateral_mm * math.cos(mid_heading)
        y += ds_mm * math.cos(mid_heading) - lateral_mm * math.sin(mid_heading)
        heading += dtheta
        times.append(time)
        xs.append(x)
        ys.append(y)
        previous_time, previous_pulse = time, pulse
    return np.asarray(times), np.asarray(xs), np.asarray(ys)


def _position_metrics(reference: list[dict], timeline: LogTimeline,
                      candidate: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None) -> dict:
    arrays = _timeline_arrays(timeline)
    valid = [row for row in reference if row["valid"] and
             0.0 <= row["time_ms"] <= timeline.time_ms[-1]]
    if not valid:
        raise ValueError(f"{timeline.path.name}: no valid camera samples overlap the log")
    candidate_values = None if candidate is None else candidate
    baseline_errors = []
    candidate_errors = []
    heading_errors = []
    for row in valid:
        time = row["time_ms"]
        baseline_x = _interpolate(arrays["time_ms"], arrays["x_mm"], time)
        baseline_y = _interpolate(arrays["time_ms"], arrays["y_mm"], time)
        baseline_errors.append(math.hypot(row["x_mm"] - baseline_x,
                                          row["y_mm"] - baseline_y))
        integrated_heading = _interpolate(arrays["time_ms"], arrays["heading_rad"], time)
        heading_errors.append(abs(_wrap_degrees(
            row["heading_cw_deg"] - math.degrees(integrated_heading))))
        if candidate_values is not None:
            ct, cx, cy = candidate_values
            candidate_x = _interpolate(ct, cx, time)
            candidate_y = _interpolate(ct, cy, time)
            candidate_errors.append(math.hypot(row["x_mm"] - candidate_x,
                                               row["y_mm"] - candidate_y))
    result = {
        "camera_samples_compared": len(valid),
        "time_coverage_ms": [valid[0]["time_ms"], valid[-1]["time_ms"]],
        "current_pose_position_error_p95_mm": float(np.percentile(baseline_errors, 95)),
        "current_pose_end_error_mm": baseline_errors[-1],
        "gyro_heading_error_p95_deg": float(np.percentile(heading_errors, 95)),
        "gyro_heading_end_error_deg": heading_errors[-1],
    }
    if candidate_values is not None:
        result["candidate_pose_position_error_p95_mm"] = float(np.percentile(candidate_errors, 95))
        result["candidate_pose_end_error_mm"] = candidate_errors[-1]
        result["candidate_vs_current_p95_improvement_mm"] = (
            result["current_pose_position_error_p95_mm"] -
            result["candidate_pose_position_error_p95_mm"])
    return result


def evaluate_baseline(log_path: Path, reference_path: Path,
                      diagnostics_path: Path) -> dict:
    """初回撮影向けに現行XYとジャイロ方位の誤差だけを出す。"""
    timeline = _read_log(log_path)
    reference, accuracy, _ = _read_reference(reference_path, diagnostics_path)
    return {
        "run_id": log_path.stem,
        "log": str(log_path),
        "camera_reference_accuracy_mm": accuracy,
        **_position_metrics(reference, timeline),
        "note": "current odometry baseline only; no K_lat candidate was applied",
    }


def evaluate_k_lat_for_runs(runs: list[dict], model: dict) -> dict:
    """5本の固定検証走で現行XYと候補K_lat積分を比較する。"""
    if len(runs) != 5:
        raise ValueError(f"exactly five held-out validation runs required; got {len(runs)}")
    validation_ids = {str(run["run_id"]) for run in runs}
    if len(validation_ids) != len(runs):
        raise ValueError("duplicate validation run_id")
    overlap = validation_ids & set(model.get("training_run_ids", []))
    if overlap:
        raise ValueError(f"training and validation runs overlap: {sorted(overlap)}")
    metrics = []
    for run in runs:
        timeline = _read_log(Path(run["log"]))
        reference, _, _ = _read_reference(
            Path(run["reference_csv"]), Path(run["diagnostics_json"]))
        candidate = _candidate_trajectory(timeline, model)
        metrics.append({"run_id": str(run["run_id"]), **_position_metrics(reference, timeline, candidate)})
    return {
        "model_training_run_ids": model["training_run_ids"],
        "validation_run_ids": sorted(validation_ids),
        "runs": metrics,
        "run_to_run_variation": {
            "current_pose_position_error_p95_mean_mm": statistics.mean(
                row["current_pose_position_error_p95_mm"] for row in metrics),
            "current_pose_position_error_p95_stdev_mm": statistics.pstdev(
                row["current_pose_position_error_p95_mm"] for row in metrics),
            "candidate_pose_position_error_p95_mean_mm": statistics.mean(
                row["candidate_pose_position_error_p95_mm"] for row in metrics),
            "candidate_pose_position_error_p95_stdev_mm": statistics.pstdev(
                row["candidate_pose_position_error_p95_mm"] for row in metrics),
            "current_pose_end_error_mean_mm": statistics.mean(
                row["current_pose_end_error_mm"] for row in metrics),
            "current_pose_end_error_stdev_mm": statistics.pstdev(
                row["current_pose_end_error_mm"] for row in metrics),
            "candidate_pose_end_error_mean_mm": statistics.mean(
                row["candidate_pose_end_error_mm"] for row in metrics),
            "candidate_pose_end_error_stdev_mm": statistics.pstdev(
                row["candidate_pose_end_error_mm"] for row in metrics),
        },
        "heading_note": "camera heading and gyro-integrated heading are reported separately from XY error",
    }


def _manifest_runs(path: Path, split: str) -> list[dict]:
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"{path}: invalid run manifest") from exc
    if not isinstance(manifest, dict) or not isinstance(manifest.get("runs"), list):
        raise ValueError(f"{path}: JSON must contain a runs array")
    runs = []
    for item in manifest["runs"]:
        if not isinstance(item, dict) or item.get("split") != split:
            continue
        required = ("run_id", "log", "reference_csv", "diagnostics_json")
        if any(not item.get(key) for key in required):
            raise ValueError(f"{path}: each {split} run needs {required}")
        runs.append({"run_id": str(item["run_id"]), **{
            key: str((path.parent / item[key]).resolve()) for key in required[1:]
        }})
    if len(runs) != 5:
        raise ValueError(f"{path}: exactly five split={split} runs required; got {len(runs)}")
    return runs


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    fit_parser = sub.add_parser("fit", help="学習5走から横ICR係数候補を推定")
    fit_parser.add_argument("manifest", type=Path)
    fit_parser.add_argument("--output", type=Path, required=True)
    baseline_parser = sub.add_parser("baseline", help="単走の現行XY・ジャイロ誤差を測る")
    baseline_parser.add_argument("log", type=Path)
    baseline_parser.add_argument("reference_csv", type=Path)
    baseline_parser.add_argument("diagnostics_json", type=Path)
    baseline_parser.add_argument("--output", type=Path, required=True)
    evaluate_parser = sub.add_parser("evaluate", help="固定検証5走を比較")
    evaluate_parser.add_argument("manifest", type=Path)
    evaluate_parser.add_argument("model", type=Path)
    evaluate_parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        if args.command == "fit":
            runs = _manifest_runs(args.manifest, "train")
            result = fit_k_lat_for_runs(runs)
        elif args.command == "baseline":
            result = evaluate_baseline(args.log, args.reference_csv, args.diagnostics_json)
        else:
            runs = _manifest_runs(args.manifest, "validation")
            model = json.loads(args.model.read_text(encoding="utf-8"))
            result = evaluate_k_lat_for_runs(runs, model)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n",
                               encoding="utf-8")
        print(json.dumps(result, indent=2, ensure_ascii=False))
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(2) from exc


if __name__ == "__main__":
    main()
