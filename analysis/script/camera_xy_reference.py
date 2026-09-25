#!/usr/bin/env python3
"""固定カメラ映像からschema 10一次走行の独立XY基準を作る。"""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import subprocess
import sys
from dataclasses import dataclass
from fractions import Fraction
from pathlib import Path
from typing import Iterator

import numpy as np
from PIL import Image

try:
    from .path_log_recovery import read_csv_log
    from .robotrace_units import CURRENT_PULSE_METER
except ImportError:
    from path_log_recovery import read_csv_log
    from robotrace_units import CURRENT_PULSE_METER


MIN_CALIBRATION_POINTS = 8
MIN_VALIDATION_POINTS = 4
POSITION_VALIDATION_LIMIT_MM = 5.0
MIN_STATIC_REFERENCES = 3
MAX_CNTLOG_DELTA_MS = 0x7FFF


@dataclass(frozen=True)
class CalibrationPoint:
    point_id: str
    pixel_x: float
    pixel_y: float
    floor_x_mm: float
    floor_y_mm: float
    split: str


@dataclass(frozen=True)
class LogTimeline:
    path: Path
    metadata: dict[str, str]
    rows: list[dict[str, str]]
    time_ms: list[float]
    pulse: list[float]


def _finite(value: object, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label}: finite number required") from exc
    if not math.isfinite(number):
        raise ValueError(f"{label}: finite number required")
    return number


def _load_json(path: Path) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"{path}: invalid JSON") from exc
    if not isinstance(value, dict):
        raise ValueError(f"{path}: JSON object required")
    return value


def read_calibration_points(path: Path) -> list[CalibrationPoint]:
    points: list[CalibrationPoint] = []
    with path.open("r", encoding="utf-8-sig", newline="") as source:
        reader = csv.DictReader(source)
        required = {"id", "pixel_x", "pixel_y", "floor_x_mm", "floor_y_mm", "split"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"{path}: requires columns {sorted(required)}")
        for line_number, row in enumerate(reader, start=2):
            point_id = (row.get("id") or "").strip()
            split = (row.get("split") or "").strip().lower()
            if not point_id or split not in {"fit", "check"}:
                raise ValueError(f"{path}: line {line_number}: invalid id or split")
            points.append(CalibrationPoint(
                point_id=point_id,
                pixel_x=_finite(row.get("pixel_x"), f"line {line_number} pixel_x"),
                pixel_y=_finite(row.get("pixel_y"), f"line {line_number} pixel_y"),
                floor_x_mm=_finite(row.get("floor_x_mm"), f"line {line_number} floor_x_mm"),
                floor_y_mm=_finite(row.get("floor_y_mm"), f"line {line_number} floor_y_mm"),
                split=split,
            ))
    if len({point.point_id for point in points}) != len(points):
        raise ValueError(f"{path}: duplicate calibration point id")
    fit_count = sum(point.split == "fit" for point in points)
    check_count = sum(point.split == "check" for point in points)
    if fit_count < MIN_CALIBRATION_POINTS or check_count < MIN_VALIDATION_POINTS:
        raise ValueError(
            f"{path}: need at least {MIN_CALIBRATION_POINTS} fit and "
            f"{MIN_VALIDATION_POINTS} independent check points; got {fit_count}/{check_count}"
        )
    return points


def _normalization(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    center = np.mean(points, axis=0)
    distances = np.linalg.norm(points - center, axis=1)
    mean_distance = float(np.mean(distances))
    if mean_distance <= 1e-12:
        raise ValueError("homography points are degenerate")
    scale = math.sqrt(2.0) / mean_distance
    matrix = np.array([
        [scale, 0.0, -scale * center[0]],
        [0.0, scale, -scale * center[1]],
        [0.0, 0.0, 1.0],
    ])
    homogeneous = np.column_stack((points, np.ones(len(points))))
    normalized = (matrix @ homogeneous.T).T[:, :2]
    return matrix, normalized


def fit_homography(pixel_xy: np.ndarray, floor_xy: np.ndarray) -> np.ndarray:
    """正規化DLTで画素座標を床面座標へ写す射影変換を求める。"""
    source = np.asarray(pixel_xy, dtype=np.float64)
    target = np.asarray(floor_xy, dtype=np.float64)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 2:
        raise ValueError("pixel and floor points must have matching N×2 shapes")
    if len(source) < 4 or not np.isfinite(source).all() or not np.isfinite(target).all():
        raise ValueError("homography requires at least four finite point pairs")
    source_transform, source_normalized = _normalization(source)
    target_transform, target_normalized = _normalization(target)
    rows = []
    for (x, y), (u, v) in zip(source_normalized, target_normalized):
        rows.append([-x, -y, -1.0, 0.0, 0.0, 0.0, u * x, u * y, u])
        rows.append([0.0, 0.0, 0.0, -x, -y, -1.0, v * x, v * y, v])
    _, singular_values, right_vectors = np.linalg.svd(np.asarray(rows), full_matrices=True)
    if singular_values[-2] <= singular_values[0] * 1e-12:
        raise ValueError("homography calibration points are degenerate")
    normalized_h = right_vectors[-1].reshape(3, 3)
    homography = np.linalg.inv(target_transform) @ normalized_h @ source_transform
    if abs(homography[2, 2]) <= 1e-12:
        raise ValueError("homography cannot be normalized")
    homography /= homography[2, 2]
    if not np.isfinite(homography).all() or abs(np.linalg.det(homography)) <= 1e-15:
        raise ValueError("homography is singular")
    return homography


def transform_point(homography: np.ndarray, pixel_x: float, pixel_y: float) -> tuple[float, float]:
    point = homography @ np.array([pixel_x, pixel_y, 1.0], dtype=np.float64)
    if not np.isfinite(point).all() or abs(point[2]) <= 1e-12:
        raise ValueError("point maps to infinity")
    return float(point[0] / point[2]), float(point[1] / point[2])


def calibration_report(points: list[CalibrationPoint]) -> tuple[np.ndarray, dict]:
    fitting = [point for point in points if point.split == "fit"]
    checks = [point for point in points if point.split == "check"]
    matrix = fit_homography(
        np.array([[point.pixel_x, point.pixel_y] for point in fitting]),
        np.array([[point.floor_x_mm, point.floor_y_mm] for point in fitting]),
    )
    rows = []
    for point in checks:
        predicted_x, predicted_y = transform_point(matrix, point.pixel_x, point.pixel_y)
        error = math.hypot(predicted_x - point.floor_x_mm,
                           predicted_y - point.floor_y_mm)
        rows.append({"id": point.point_id, "error_mm": error,
                     "predicted_x_mm": predicted_x, "predicted_y_mm": predicted_y})
    errors = [row["error_mm"] for row in rows]
    rms = math.sqrt(sum(error * error for error in errors) / len(errors))
    maximum = max(errors)
    return matrix, {
        "fit_points": len(fitting),
        "independent_check_points": len(checks),
        "check_rms_mm": rms,
        "check_max_mm": maximum,
        "check_points": rows,
        "position_gate_mm": POSITION_VALIDATION_LIMIT_MM,
        "passed": maximum <= POSITION_VALIDATION_LIMIT_MM,
        "homography_pixel_to_floor": matrix.tolist(),
    }


def _rate(value: str, label: str) -> float:
    try:
        rate = float(Fraction(value))
    except (ValueError, ZeroDivisionError) as exc:
        raise ValueError(f"{label}: invalid frame rate {value!r}") from exc
    if not math.isfinite(rate) or rate <= 0:
        raise ValueError(f"{label}: invalid frame rate {value!r}")
    return rate


def probe_video(path: Path, ffprobe: str = "ffprobe") -> dict:
    executable = shutil.which(ffprobe) if not Path(ffprobe).is_file() else ffprobe
    if executable is None:
        raise ValueError(f"ffprobe executable not found: {ffprobe}")
    command = [
        str(executable), "-v", "error", "-select_streams", "v:0",
        "-show_entries", "stream=width,height,avg_frame_rate,r_frame_rate,codec_name:frame=best_effort_timestamp_time",
        "-show_frames", "-of", "json", str(path),
    ]
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    if result.returncode != 0:
        raise ValueError(f"ffprobe failed for {path}: {result.stderr.strip()}")
    try:
        payload = json.loads(result.stdout)
        streams = payload["streams"]
        frames = payload["frames"]
        stream = streams[0]
        timestamps = [float(frame["best_effort_timestamp_time"]) * 1000.0 for frame in frames]
    except (KeyError, IndexError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError(f"{path}: ffprobe did not return timed video frames") from exc
    if len(timestamps) < 2 or any(not math.isfinite(time) for time in timestamps):
        raise ValueError(f"{path}: video must contain at least two timed frames")
    intervals = [right - left for left, right in zip(timestamps, timestamps[1:])]
    if any(interval <= 0 for interval in intervals):
        raise ValueError(f"{path}: video frame timestamps must be strictly increasing")
    median_interval_ms = float(np.median(intervals))
    measured_average_fps = (len(timestamps) - 1) * 1000.0 / (timestamps[-1] - timestamps[0])
    stream_fps = _rate(stream.get("avg_frame_rate", "0/0"), "avg_frame_rate")
    frame_period_ms = 1000.0 / stream_fps
    return {
        "path": str(path),
        "width": int(stream["width"]),
        "height": int(stream["height"]),
        "codec": stream.get("codec_name", "unknown"),
        "nominal_stream_fps": _rate(stream.get("r_frame_rate", "0/0"), "r_frame_rate"),
        "stream_average_fps": stream_fps,
        "measured_average_fps": measured_average_fps,
        "median_frame_interval_fps": 1000.0 / median_interval_ms,
        "frame_count": len(timestamps),
        "first_video_time_ms": timestamps[0],
        "last_video_time_ms": timestamps[-1],
        "median_frame_interval_ms": median_interval_ms,
        "nominal_frame_period_ms": frame_period_ms,
        "half_frame_ms": frame_period_ms * 0.5,
        "long_frame_gaps": sum(intervals[index] > median_interval_ms * 1.5
                                for index in range(len(intervals))),
        "max_frame_gap_ms": max(intervals),
        "long_frame_gap_intervals_ms": [
            [timestamps[index], timestamps[index + 1]]
            for index, interval in enumerate(intervals)
            if interval > median_interval_ms * 1.5
        ],
        "timestamps_ms": timestamps,
    }


def _video_fps_gate(probe: dict, allow_isolated_drops: bool) -> tuple[bool, str]:
    if not allow_isolated_drops:
        passed = min(probe["stream_average_fps"], probe["measured_average_fps"]) >= 60.0
        return passed, "" if passed else "video frame rate is below 60 fps"
    # PTS keeps an isolated dropped frame visible. Keep its time gap instead of
    # relabeling later frames; downstream turn windows reject long intervals.
    allowed_gaps = max(1, math.ceil((probe["frame_count"] - 1) * 0.001))
    passed = (probe["nominal_stream_fps"] >= 59.9 and
              probe["median_frame_interval_fps"] >= 59.9 and
              probe["measured_average_fps"] >= 59.0 and
              probe["long_frame_gaps"] <= allowed_gaps and
              probe["max_frame_gap_ms"] <= probe["nominal_frame_period_ms"] * 3.05)
    return passed, "" if passed else "video has excessive frame gaps or runs below 60 fps class"


def write_frame_timestamps(probe: dict, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8", newline="") as target:
        writer = csv.writer(target)
        writer.writerow(["frame_index", "video_time_ms"])
        writer.writerows((index, f"{timestamp:.6f}")
                         for index, timestamp in enumerate(probe["timestamps_ms"]))


def _read_exact(stream, count: int) -> bytes | None:
    chunks = []
    remaining = count
    while remaining:
        chunk = stream.read(remaining)
        if not chunk:
            if not chunks:
                return None
            raise ValueError("ffmpeg ended in the middle of a decoded frame")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def iter_video_frames(path: Path, probe: dict, ffmpeg: str = "ffmpeg",
                      noautorotate: bool = False) -> Iterator[Image.Image]:
    executable = shutil.which(ffmpeg) if not Path(ffmpeg).is_file() else ffmpeg
    if executable is None:
        raise ValueError(f"ffmpeg executable not found: {ffmpeg}")
    command = [
        str(executable), "-v", "error",
        *(["-noautorotate"] if noautorotate else []),
        "-i", str(path), "-map", "0:v:0",
        "-fps_mode", "passthrough", "-f", "rawvideo", "-pix_fmt", "rgb24", "pipe:1",
    ]
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    assert process.stdout is not None
    frame_size = probe["width"] * probe["height"] * 3
    try:
        for _ in range(probe["frame_count"]):
            frame = _read_exact(process.stdout, frame_size)
            if frame is None:
                raise ValueError("ffmpeg decoded fewer frames than ffprobe")
            yield Image.frombytes("RGB", (probe["width"], probe["height"]), frame)
        if _read_exact(process.stdout, frame_size) is not None:
            raise ValueError("ffmpeg decoded more frames than ffprobe")
    finally:
        process.stdout.close()
        return_code = process.wait()
        stderr = process.stderr.read().decode("utf-8", errors="replace") if process.stderr else ""
        if process.stderr:
            process.stderr.close()
        if return_code != 0:
            raise ValueError(f"ffmpeg failed for {path}: {stderr.strip()}")


def _color_config(value: object, label: str) -> dict[str, float | int]:
    if not isinstance(value, dict):
        raise ValueError(f"camera setup requires {label} color object")
    required = ("hue_min", "hue_max", "saturation_min", "value_min", "search_radius_px")
    if not all(name in value for name in required):
        raise ValueError(f"{label}: missing color fields {required}")
    hue_min = int(value["hue_min"])
    hue_max = int(value["hue_max"])
    if not 0 <= hue_min <= 255 or not 0 <= hue_max <= 255:
        raise ValueError(f"{label}: Pillow HSV hue must be in [0,255]")
    saturation_min = int(value["saturation_min"])
    value_min = int(value["value_min"])
    radius = int(value["search_radius_px"])
    area_min = int(value.get("area_min_px", 4))
    area_max = int(value.get("area_max_px", 10000))
    if not 0 <= saturation_min <= 255 or not 0 <= value_min <= 255:
        raise ValueError(f"{label}: saturation and value thresholds must be in [0,255]")
    if radius < 1 or area_min < 1 or area_max < area_min:
        raise ValueError(f"{label}: invalid search radius or pixel-area limits")
    return {"hue_min": hue_min, "hue_max": hue_max,
            "saturation_min": saturation_min, "value_min": value_min,
            "search_radius_px": radius, "area_min_px": area_min,
            "area_max_px": area_max}


def _find_color_centroid(frame: Image.Image, center: tuple[float, float],
                         color: dict[str, float | int], gap_frames: int = 0
                         ) -> tuple[tuple[float, float], int] | None:
    height, width = frame.height, frame.width
    radius = int(color["search_radius_px"] * min(4.0, 1.0 + gap_frames * 0.5))
    x, y = center
    left = max(0, int(math.floor(x - radius)))
    right = min(width, int(math.ceil(x + radius + 1)))
    top = max(0, int(math.floor(y - radius)))
    bottom = min(height, int(math.ceil(y + radius + 1)))
    if left >= right or top >= bottom:
        return None
    hsv = np.asarray(frame.crop((left, top, right, bottom)).convert("HSV"), dtype=np.uint8)
    hue = hsv[:, :, 0]
    hue_min, hue_max = int(color["hue_min"]), int(color["hue_max"])
    if hue_min <= hue_max:
        hue_mask = (hue >= hue_min) & (hue <= hue_max)
    else:
        hue_mask = (hue >= hue_min) | (hue <= hue_max)
    mask = (hue_mask & (hsv[:, :, 1] >= int(color["saturation_min"])) &
            (hsv[:, :, 2] >= int(color["value_min"])))
    count = int(np.count_nonzero(mask))
    if count < int(color["area_min_px"]) or count > int(color["area_max_px"]):
        return None
    ys, xs = np.nonzero(mask)
    return (left + float(np.mean(xs)), top + float(np.mean(ys))), count


def register_ground_frame(
    homography: np.ndarray,
    current_reference_pixels: list[tuple[float, float]],
    initial_reference_pixels: list[tuple[float, float]],
    current_verification_pixels: list[tuple[float, float]],
    initial_verification_pixels: list[tuple[float, float]],
) -> tuple[np.ndarray, float]:
    """床の固定点で画像を初期フレームへ合わせ、別の点で誤差を測る。"""
    if len(current_reference_pixels) < 4 or len(current_reference_pixels) != len(initial_reference_pixels):
        raise ValueError("registration requires four matched fixed points")
    if not current_verification_pixels or len(current_verification_pixels) != len(initial_verification_pixels):
        raise ValueError("registration requires matched verification points")
    warp = fit_homography(np.asarray(current_reference_pixels),
                          np.asarray(initial_reference_pixels))
    errors = []
    for current, initial in zip(current_verification_pixels, initial_verification_pixels):
        corrected_pixel = transform_point(warp, *current)
        corrected_xy = transform_point(homography, *corrected_pixel)
        initial_xy = transform_point(homography, *initial)
        errors.append(math.dist(corrected_xy, initial_xy))
    return warp, max(errors)


def _wrap_degrees(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def _circular_lerp_degrees(first: float, second: float, fraction: float) -> float:
    return _wrap_degrees(first + _wrap_degrees(second - first) * fraction)


def robot_origin_pose(front_xy: tuple[float, float], rear_xy: tuple[float, float],
                      geometry: dict) -> tuple[float, float, float, float]:
    """マーカー2点から原点位置、時計回り方位、実測基線長を求める。"""
    dx = front_xy[0] - rear_xy[0]
    dy = front_xy[1] - rear_xy[1]
    baseline = math.hypot(dx, dy)
    marker_heading = math.degrees(math.atan2(dx, dy))
    lateral_offset = float(geometry.get("front_marker_lateral_offset_mm", 0.0))
    reference_baseline = float(geometry.get("marker_baseline_mm", baseline))
    if abs(lateral_offset) >= reference_baseline:
        raise ValueError("front marker lateral offset must be smaller than marker baseline")
    marker_axis_offset = math.degrees(math.asin(lateral_offset / reference_baseline))
    heading = _wrap_degrees(marker_heading - marker_axis_offset)
    midpoint_x = (front_xy[0] + rear_xy[0]) * 0.5
    midpoint_y = (front_xy[1] + rear_xy[1]) * 0.5
    angle = math.radians(heading)
    right_x, right_y = math.cos(angle), -math.sin(angle)
    forward_x, forward_y = math.sin(angle), math.cos(angle)
    origin_x = midpoint_x + float(geometry["marker_midpoint_to_origin_right_mm"]) * right_x + \
        float(geometry["marker_midpoint_to_origin_forward_mm"]) * forward_x
    origin_y = midpoint_y + float(geometry["marker_midpoint_to_origin_right_mm"]) * right_y + \
        float(geometry["marker_midpoint_to_origin_forward_mm"]) * forward_y
    return origin_x, origin_y, heading, baseline


def _read_log(path: Path) -> LogTimeline:
    log = read_csv_log(path, strict_rows=True)
    required_columns = {"cntlog", "encTotalOptimal", "gyroVal_Z", "x", "y"}
    missing = sorted(required_columns - set(log.fields))
    if missing:
        raise ValueError(f"{path.name}: missing columns: {', '.join(missing)}")
    required_metadata = {
        "logSchemaVersion": "10", "optimalTrace": "0", "closureValid": "1",
        "closureReason": "0", "emcStop": "0", "logOverflowFinal": "0",
        "dbgOverflowFinal": "0", "distanceScaleVerified": "1",
        "encoderPulsePerMeter": str(int(CURRENT_PULSE_METER)),
        "logRecordSizeBytes": "36", "imuCalibrationValid": "1",
        "imuCalibrationSamples": "100", "imuCalibrationReadErrors": "0",
        "gyroSampleFault": "0", "encoderIntervalFault": "0",
        "timing.startResetMeasured": "1", "goalMarkerOnsetValid": "1",
    }
    for name, expected in required_metadata.items():
        actual = log.parameters.get(name)
        try:
            matches = actual is not None and float(actual) == float(expected)
        except ValueError:
            matches = False
        if not matches:
            raise ValueError(f"{path.name}: invalid or missing {name}={actual!r}")
    expected_rows = int(log.parameters.get("logExpectedRows", "-1"))
    if expected_rows <= 0 or expected_rows != len(log.rows):
        raise ValueError(f"{path.name}: logExpectedRows does not match data rows")
    delay = _finite(log.parameters.get("timing.startResetDelay_ms"), "startResetDelay_ms")
    if delay < 0:
        raise ValueError(f"{path.name}: startResetDelay_ms must be nonnegative")
    goal_pulse = int(log.parameters.get("goalMarkerOnset_p", "0"))
    if goal_pulse <= 0:
        raise ValueError(f"{path.name}: goalMarkerOnset_p must be positive")
    times = []
    pulses = []
    elapsed = 0
    previous_cnt = 0
    previous_pulse = 0.0
    for index, row in enumerate(log.rows):
        current = int(row["cntlog"])
        if not 0 <= current <= 0xFFFF:
            raise ValueError(f"{path.name}: cntlog outside U16 at row {index + 1}")
        delta = (current - previous_cnt) & 0xFFFF
        if not 0 < delta <= MAX_CNTLOG_DELTA_MS:
            raise ValueError(f"{path.name}: invalid cntlog interval at row {index + 1}")
        elapsed += delta
        current_pulse = _finite(row["encTotalOptimal"], "encTotalOptimal")
        if current_pulse < previous_pulse:
            raise ValueError(f"{path.name}: encTotalOptimal moved backward at row {index + 1}")
        _finite(row["gyroVal_Z"], "gyroVal_Z")
        _finite(row["x"], "x")
        _finite(row["y"], "y")
        times.append(float(elapsed))
        pulses.append(current_pulse)
        previous_cnt = current
        previous_pulse = current_pulse
    if goal_pulse > pulses[-1]:
        raise ValueError(f"{path.name}: goalMarkerOnset_p is beyond logged cumulative pulse")
    return LogTimeline(path, log.parameters, log.rows, times, pulses)


def _time_at_pulse(timeline: LogTimeline, pulse: float) -> float:
    previous_time, previous_pulse = 0.0, 0.0
    for time, current_pulse in zip(timeline.time_ms, timeline.pulse):
        if previous_pulse <= pulse <= current_pulse and current_pulse > previous_pulse:
            fraction = (pulse - previous_pulse) / (current_pulse - previous_pulse)
            return previous_time + fraction * (time - previous_time)
        previous_time, previous_pulse = time, current_pulse
    raise ValueError(f"goal pulse {pulse:g} is not bracketed by encTotalOptimal")


def _pose_at_video_time(rows: list[dict], time_ms: float, max_gap_ms: float) -> dict | None:
    valid = [row for row in rows if row["markers_valid"] and row["camera_stable"]]
    for row in valid:
        if abs(row["video_time_ms"] - time_ms) <= 1e-8:
            return {**row, "alignment_method": "exact_frame", "bracket_gap_ms": 0.0}
    for first, second in zip(valid, valid[1:]):
        if first["video_time_ms"] < time_ms < second["video_time_ms"]:
            gap = second["video_time_ms"] - first["video_time_ms"]
            if gap > max_gap_ms:
                return None
            fraction = (time_ms - first["video_time_ms"]) / gap
            return {
                "origin_world_x_mm": first["origin_world_x_mm"] + fraction *
                    (second["origin_world_x_mm"] - first["origin_world_x_mm"]),
                "origin_world_y_mm": first["origin_world_y_mm"] + fraction *
                    (second["origin_world_y_mm"] - first["origin_world_y_mm"]),
                "heading_world_cw_deg": _circular_lerp_degrees(
                    first["heading_world_cw_deg"], second["heading_world_cw_deg"], fraction),
                "alignment_method": "between_adjacent_valid_frames",
                "bracket_gap_ms": gap,
            }
    return None


def _missing_intervals(rows: list[dict]) -> list[dict]:
    intervals = []
    start = None
    for index, row in enumerate(rows):
        if not row["track_valid"] and start is None:
            start = index
        if row["track_valid"] and start is not None:
            intervals.append({"first_video_time_ms": rows[start]["video_time_ms"],
                              "last_video_time_ms": rows[index - 1]["video_time_ms"],
                              "frame_count": index - start})
            start = None
    if start is not None:
        intervals.append({"first_video_time_ms": rows[start]["video_time_ms"],
                          "last_video_time_ms": rows[-1]["video_time_ms"],
                          "frame_count": len(rows) - start})
    return intervals


def _stationary_pose_jitter(rows: list[dict], interval: object) -> dict:
    if interval is None:
        return {"configured": False}
    if not isinstance(interval, list) or len(interval) != 2:
        raise ValueError("stationary_check_video_interval_ms must be [start_ms,end_ms]")
    start_ms = _finite(interval[0], "stationary check start")
    end_ms = _finite(interval[1], "stationary check end")
    if end_ms <= start_ms:
        raise ValueError("stationary check interval must have positive duration")
    selected = [row for row in rows if row["markers_valid"] and row["camera_stable"] and
                start_ms <= row["video_time_ms"] <= end_ms]
    if len(selected) < 3:
        return {"configured": True, "samples": len(selected), "sufficient_samples": False}
    xs = np.asarray([row["origin_world_x_mm"] for row in selected])
    ys = np.asarray([row["origin_world_y_mm"] for row in selected])
    headings = np.radians([row["heading_world_cw_deg"] for row in selected])
    mean_heading = math.atan2(float(np.mean(np.sin(headings))),
                              float(np.mean(np.cos(headings))))
    heading_errors = np.asarray([_wrap_degrees(math.degrees(value - mean_heading))
                                 for value in headings])
    std_x, std_y = float(np.std(xs)), float(np.std(ys))
    return {
        "configured": True,
        "video_interval_ms": [start_ms, end_ms],
        "samples": len(selected),
        "sufficient_samples": True,
        "position_std_x_mm": std_x,
        "position_std_y_mm": std_y,
        "position_rms_mm": math.hypot(std_x, std_y),
        "heading_rms_deg": float(np.sqrt(np.mean(heading_errors * heading_errors))),
    }


def _write_csv(path: Path, fields: list[str], rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as target:
        writer = csv.DictWriter(target, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _tracking_rows(video: Path, probe: dict, points: list[CalibrationPoint],
                   homography: np.ndarray, config: dict, ffmpeg: str) -> list[dict]:
    marker_config = config.get("markers")
    if not isinstance(marker_config, dict) or not {"front", "rear"}.issubset(marker_config):
        raise ValueError("camera setup requires front and rear marker configuration")
    markers = {}
    for name in ("front", "rear"):
        entry = marker_config[name]
        if not isinstance(entry, dict) or "seed_pixel" not in entry:
            raise ValueError(f"markers.{name} requires seed_pixel")
        seed = entry["seed_pixel"]
        if not isinstance(seed, list) or len(seed) != 2:
            raise ValueError(f"markers.{name}.seed_pixel must contain pixel x/y")
        color = _color_config(entry, f"markers.{name}")
        markers[name] = {"center": (_finite(seed[0], f"markers.{name}.seed_pixel[0]"),
                                     _finite(seed[1], f"markers.{name}.seed_pixel[1]")),
                         "gap": 0, "color": color}
    geometry = config.get("geometry")
    geometry_fields = (
        "marker_baseline_mm", "marker_midpoint_to_origin_right_mm",
        "marker_midpoint_to_origin_forward_mm", "right_marker_sensor_right_mm",
        "right_marker_sensor_forward_mm", "marker_height_mm",
    )
    if not isinstance(geometry, dict) or any(name not in geometry for name in geometry_fields):
        raise ValueError(f"camera setup geometry requires {geometry_fields}")
    geometry = {name: _finite(geometry[name], f"geometry.{name}") for name in geometry_fields}
    if geometry["marker_baseline_mm"] <= 0 or geometry["marker_height_mm"] < 0:
        raise ValueError("marker baseline must be positive and height nonnegative")
    geometry["front_marker_lateral_offset_mm"] = _finite(
        config["geometry"].get("front_marker_lateral_offset_mm", 0.0),
        "geometry.front_marker_lateral_offset_mm")
    if abs(geometry["front_marker_lateral_offset_mm"]) >= geometry["marker_baseline_mm"]:
        raise ValueError("front marker lateral offset must be smaller than marker baseline")
    baseline_tolerance = _finite(config.get("max_marker_baseline_error_mm", 15.0),
                                 "max_marker_baseline_error_mm")
    max_camera_motion = _finite(config.get("max_camera_motion_mm", 5.0),
                                "max_camera_motion_mm")
    motion_mode = config.get("camera_motion_mode", "reject")
    if motion_mode not in ("reject", "register"):
        raise ValueError("camera_motion_mode must be reject or register")
    max_registration_error = _finite(config.get("max_registration_error_mm", 5.0),
                                     "max_registration_error_mm")
    marker_plane_scale = 1.0
    if motion_mode == "register":
        capture = config.get("capture_conditions", {})
        if not isinstance(capture, dict):
            raise ValueError("capture_conditions must be an object")
        camera_height = _finite(capture.get("mount_height_mm"), "capture_conditions.mount_height_mm")
        if camera_height <= geometry["marker_height_mm"]:
            raise ValueError("camera height must exceed marker height")
        marker_plane_scale = (camera_height - geometry["marker_height_mm"]) / camera_height
    fixed_ids = config.get("fixed_reference_ids")
    if not isinstance(fixed_ids, list) or len(set(fixed_ids)) < MIN_STATIC_REFERENCES:
        raise ValueError(f"camera setup requires at least {MIN_STATIC_REFERENCES} fixed_reference_ids")
    points_by_id = {point.point_id: point for point in points}
    if any(point_id not in points_by_id or points_by_id[point_id].split != "check"
           for point_id in fixed_ids):
        raise ValueError("fixed_reference_ids must name independent check points")
    fixed_color = _color_config(config.get("fixed_reference_color"), "fixed_reference_color")
    fixed_refs = [{"point": points_by_id[point_id], "center":
                   (points_by_id[point_id].pixel_x, points_by_id[point_id].pixel_y),
                   "gap": 0} for point_id in fixed_ids]
    fit_refs = []
    fit_color = None
    if motion_mode == "register":
        fit_color = _color_config(config.get("fit_reference_color"), "fit_reference_color")
        fit_refs = [{"point": point, "center": (point.pixel_x, point.pixel_y), "gap": 0}
                    for point in points if point.split == "fit"]
    output = []
    timestamps = probe["timestamps_ms"]
    for frame_index, frame in enumerate(iter_video_frames(
            video, probe, ffmpeg, bool(config.get("decode_without_rotation", False)))):
        marker_pixel = {}
        marker_count = {}
        for name, state in markers.items():
            detected = _find_color_centroid(frame, state["center"], state["color"], state["gap"])
            if detected is None:
                state["gap"] += 1
                marker_pixel[name] = None
                marker_count[name] = 0
            else:
                state["center"], marker_count[name] = detected
                state["gap"] = 0
                marker_pixel[name] = state["center"]
        static_vectors = []
        fixed_current = []
        fixed_initial = []
        static_count = 0
        for state in fixed_refs:
            point = state["point"]
            detected = _find_color_centroid(frame, state["center"], fixed_color, state["gap"])
            if detected is None:
                state["gap"] += 1
                continue
            state["center"], _ = detected
            state["gap"] = 0
            fixed_current.append(state["center"])
            fixed_initial.append((point.pixel_x, point.pixel_y))
            current_xy = transform_point(homography, *state["center"])
            initial_xy = transform_point(homography, point.pixel_x, point.pixel_y)
            static_vectors.append((current_xy[0] - initial_xy[0],
                                   current_xy[1] - initial_xy[1]))
            static_count += 1
        if static_count >= MIN_STATIC_REFERENCES:
            magnitudes = [math.hypot(dx, dy) for dx, dy in static_vectors]
            camera_motion_mm = float(np.median(magnitudes))
            camera_stable = camera_motion_mm <= max_camera_motion
        else:
            camera_motion_mm = None
            camera_stable = False

        fit_current = []
        fit_initial = []
        if motion_mode == "register":
            for state in fit_refs:
                detected = _find_color_centroid(frame, state["center"], fit_color, state["gap"])
                if detected is None:
                    state["gap"] += 1
                    continue
                state["center"], _ = detected
                state["gap"] = 0
                fit_current.append(state["center"])
                fit_initial.append((state["point"].pixel_x, state["point"].pixel_y))
        registration_warp = None
        registration_error = None
        if motion_mode == "register":
            camera_stable = False
            if static_count >= MIN_STATIC_REFERENCES and len(fit_current) == len(fit_refs):
                try:
                    registration_warp, registration_error = register_ground_frame(
                        homography, fixed_current, fixed_initial,
                        fit_current, fit_initial)
                    camera_stable = registration_error <= max_registration_error
                except (ValueError, np.linalg.LinAlgError):
                    pass

        origin_x = origin_y = heading = baseline = None
        markers_valid = False
        if marker_pixel["front"] is not None and marker_pixel["rear"] is not None:
            if motion_mode == "reject" or registration_warp is not None:
                front_pixel = marker_pixel["front"]
                rear_pixel = marker_pixel["rear"]
                if registration_warp is not None:
                    front_pixel = transform_point(registration_warp, *front_pixel)
                    rear_pixel = transform_point(registration_warp, *rear_pixel)
                front_xy = tuple(value * marker_plane_scale
                                 for value in transform_point(homography, *front_pixel))
                rear_xy = tuple(value * marker_plane_scale
                                for value in transform_point(homography, *rear_pixel))
                origin_x, origin_y, heading, baseline = robot_origin_pose(front_xy, rear_xy, geometry)
                markers_valid = abs(baseline - geometry["marker_baseline_mm"]) <= baseline_tolerance
        reference_valid = markers_valid and camera_stable
        output.append({
            "frame_index": frame_index,
            "video_time_ms": timestamps[frame_index],
            "front_pixel_x": "" if marker_pixel["front"] is None else marker_pixel["front"][0],
            "front_pixel_y": "" if marker_pixel["front"] is None else marker_pixel["front"][1],
            "rear_pixel_x": "" if marker_pixel["rear"] is None else marker_pixel["rear"][0],
            "rear_pixel_y": "" if marker_pixel["rear"] is None else marker_pixel["rear"][1],
            "front_pixels": marker_count["front"], "rear_pixels": marker_count["rear"],
            "origin_world_x_mm": origin_x, "origin_world_y_mm": origin_y,
            "heading_world_cw_deg": heading, "marker_baseline_mm": baseline,
            "camera_motion_mm": camera_motion_mm,
            "camera_reference_count": static_count,
            "registration_error_max_mm": registration_error,
            "verification_reference_count": len(fit_current),
            "markers_valid": markers_valid,
            "camera_stable": camera_stable,
            "reference_valid": reference_valid,
        })
    if len(output) != probe["frame_count"]:
        raise ValueError("decoded frame count does not match ffprobe timestamps")
    return output


def _log_time_at_event(timeline: LogTimeline, event: dict) -> tuple[float, dict]:
    start_video = _finite(event.get("start_sensor_onset_video_ms"),
                          "start_sensor_onset_video_ms")
    goal_video = _finite(event.get("goal_sensor_onset_video_ms"),
                         "goal_sensor_onset_video_ms")
    if goal_video <= start_video:
        raise ValueError("goal sensor event must follow start sensor event")
    goal_pulse = int(timeline.metadata["goalMarkerOnset_p"])
    goal_log_time = _time_at_pulse(timeline, float(goal_pulse))
    start_delay = float(timeline.metadata["timing.startResetDelay_ms"])
    offset = -start_video - start_delay
    predicted_goal_log_time = goal_video + offset
    residual = goal_log_time - predicted_goal_log_time
    return offset, {
        "start_sensor_onset_video_ms": start_video,
        "startResetDelay_ms": start_delay,
        "start_event_log_time_ms": -start_delay,
        "goal_sensor_onset_video_ms": goal_video,
        "goalMarkerOnset_p": goal_pulse,
        "goal_event_log_time_ms": goal_log_time,
        "goal_video_predicted_log_time_ms": predicted_goal_log_time,
        "goal_sync_residual_ms": residual,
    }


def _write_tracking_csv(path: Path, rows: list[dict]) -> None:
    fields = ["frame_index", "video_time_ms", "front_pixel_x", "front_pixel_y",
              "rear_pixel_x", "rear_pixel_y", "front_pixels", "rear_pixels",
              "origin_world_x_mm", "origin_world_y_mm", "heading_world_cw_deg",
              "marker_baseline_mm", "camera_motion_mm", "camera_reference_count",
              "registration_error_max_mm", "verification_reference_count",
              "markers_valid", "camera_stable", "reference_valid"]
    encoded = []
    for row in rows:
        item = dict(row)
        for name in ("markers_valid", "camera_stable", "reference_valid"):
            item[name] = int(bool(item[name]))
        encoded.append(item)
    _write_csv(path, fields, encoded)


def build_reference(video: Path, log_path: Path, calibration_csv: Path,
                    setup_json: Path, events_json: Path, output_dir: Path,
                    *, ffmpeg: str = "ffmpeg", ffprobe: str = "ffprobe") -> dict:
    points = read_calibration_points(calibration_csv)
    homography, calibration = calibration_report(points)
    config = _load_json(setup_json)
    capture_conditions = config.get("capture_conditions", {})
    if not isinstance(capture_conditions, dict):
        raise ValueError("capture_conditions must be a JSON object")
    events = _load_json(events_json)
    timeline = _read_log(log_path)
    probe = probe_video(video, ffprobe)
    timestamps = probe["timestamps_ms"]
    median_interval_ms = probe["median_frame_interval_ms"]
    fps_passed, fps_reason = _video_fps_gate(
        probe, config.get("allow_isolated_frame_drops", False) is True)
    motion_mode = config.get("camera_motion_mode", "reject")

    output_dir.mkdir(parents=True, exist_ok=True)
    for artifact_name in ("reference.csv", "tracking.csv", "diagnostics.json"):
        (output_dir / artifact_name).unlink(missing_ok=True)
    tracking = _tracking_rows(video, probe, points, homography, config, ffmpeg)
    _write_tracking_csv(output_dir / "tracking.csv", tracking)

    failures = []
    if not calibration["passed"]:
        failures.append("independent calibration-point error exceeds 5 mm")
    if not fps_passed:
        failures.append(fps_reason)
    if motion_mode == "register":
        if any(row["registration_error_max_mm"] is not None and
               row["registration_error_max_mm"] >
               float(config.get("max_registration_error_mm", 5.0))
               for row in tracking):
            failures.append("registered ground reference residual exceeds threshold")
    elif any(row["camera_motion_mm"] is not None and
             row["camera_motion_mm"] > float(config.get("max_camera_motion_mm", 5.0))
             for row in tracking):
        failures.append("fixed reference points detected camera motion beyond threshold")
    offset_ms, sync = _log_time_at_event(timeline, events)
    half_frame_ms = probe["half_frame_ms"]
    sync_passed = abs(sync["goal_sync_residual_ms"]) <= half_frame_ms
    if not sync_passed:
        failures.append("start/goal synchronization residual exceeds half a frame")

    start_video_ms = sync["start_sensor_onset_video_ms"]
    zero_video_ms = start_video_ms + sync["startResetDelay_ms"]
    start_pose = _pose_at_video_time(tracking, zero_video_ms, median_interval_ms * 1.5)
    origin_pose_valid = start_pose is not None
    if not origin_pose_valid:
        failures.append("log time zero is not bracketed by adjacent valid camera tracks")
    else:
        heading_error = _wrap_degrees(
            start_pose["heading_world_cw_deg"] -
            float(config.get("expected_start_heading_cw_deg", 0.0))
        )
        if abs(heading_error) > float(config.get("start_heading_tolerance_deg", 20.0)):
            failures.append("front/rear marker order or start-axis alignment failed")

    reference_rows = []
    if origin_pose_valid:
        x0 = start_pose["origin_world_x_mm"]
        y0 = start_pose["origin_world_y_mm"]
        heading0 = start_pose["heading_world_cw_deg"]
        angle0 = math.radians(heading0)
        c0, s0 = math.cos(angle0), math.sin(angle0)
        for row in tracking:
            valid = row["reference_valid"]
            if valid:
                dx = row["origin_world_x_mm"] - x0
                dy = row["origin_world_y_mm"] - y0
                # 初期機体座標: X右・Y前、方位は時計回り正。
                x_mm = dx * c0 - dy * s0
                y_mm = dx * s0 + dy * c0
                heading_rel = _wrap_degrees(row["heading_world_cw_deg"] - heading0)
            else:
                x_mm = y_mm = heading_rel = ""
            reference_rows.append({
                "time_ms": row["video_time_ms"] + offset_ms,
                "video_time_ms": row["video_time_ms"],
                "x_mm": x_mm,
                "y_mm": y_mm,
                "heading_cw_deg": heading_rel,
                "track_valid": int(valid),
                "camera_motion_mm": "" if row["camera_motion_mm"] is None else row["camera_motion_mm"],
                "registration_error_max_mm": ("" if row["registration_error_max_mm"] is None
                                              else row["registration_error_max_mm"]),
                "camera_reference_count": row["camera_reference_count"],
                "frame_index": row["frame_index"],
            })
    if not origin_pose_valid:
        reference_rows = [{
            "time_ms": row["video_time_ms"] + offset_ms,
            "video_time_ms": row["video_time_ms"], "x_mm": "", "y_mm": "",
            "heading_cw_deg": "", "track_valid": 0,
            "camera_motion_mm": "" if row["camera_motion_mm"] is None else row["camera_motion_mm"],
            "registration_error_max_mm": ("" if row["registration_error_max_mm"] is None
                                          else row["registration_error_max_mm"]),
            "camera_reference_count": row["camera_reference_count"],
            "frame_index": row["frame_index"],
        } for row in tracking]

    valid_count = sum(row["track_valid"] for row in reference_rows)
    if valid_count == 0:
        failures.append("no valid camera reference frames")
    if not failures:
        fields = ["time_ms", "video_time_ms", "x_mm", "y_mm", "heading_cw_deg",
                  "track_valid", "camera_motion_mm", "registration_error_max_mm",
                  "camera_reference_count", "frame_index"]
        _write_csv(output_dir / "reference.csv", fields, reference_rows)

    motion_values = [row["camera_motion_mm"] for row in tracking
                     if row["camera_motion_mm"] is not None]
    registration_values = [row["registration_error_max_mm"] for row in tracking
                           if row["registration_error_max_mm"] is not None]
    diagnostics = {
        "log": str(log_path),
        "log_number": log_path.stem,
        "run_conditions": {key: timeline.metadata.get(key) for key in
                           ("batteryVoltage_V", "gitCommit", "branch", "fwVersion",
                            "logSchemaVersion", "optimalTrace", "logRecordSizeBytes")},
        "capture_conditions_missing": [
            key for key, value in capture_conditions.items()
            if value is None or value == ""
        ],
        "video": {key: value for key, value in probe.items() if key != "timestamps_ms"},
        "camera_setup": config,
        "calibration": calibration,
        "synchronization": {
            **sync,
            "camera_to_log_offset_ms": offset_ms,
            "half_frame_limit_ms": half_frame_ms,
            "passed": sync_passed,
            "log_zero_pose_alignment_method": (start_pose["alignment_method"]
                                               if start_pose is not None else None),
            "log_zero_pose_bracket_gap_ms": (start_pose["bracket_gap_ms"]
                                             if start_pose is not None else None),
        },
        "tracking": {
            "camera_motion_mode": motion_mode,
            "frame_count": len(tracking),
            "valid_reference_frames": valid_count,
            "missing_or_invalid_intervals": _missing_intervals(reference_rows),
            "camera_motion_p95_mm": (float(np.percentile(motion_values, 95))
                                      if motion_values else None),
            "camera_motion_max_mm": max(motion_values) if motion_values else None,
            "registration_error_p95_mm": (float(np.percentile(registration_values, 95))
                                          if registration_values else None),
            "registration_error_max_mm": (max(registration_values)
                                          if registration_values else None),
            "stationary_pose_jitter": _stationary_pose_jitter(
                tracking, config.get("stationary_check_video_interval_ms")),
            "position_rows_are_interpolated": False,
        },
        "reference_valid": not failures,
        "failures": failures,
        "artifacts": {
            "tracking_csv": "tracking.csv",
            "reference_csv": "reference.csv" if not failures else None,
        },
    }
    (output_dir / "diagnostics.json").write_text(
        json.dumps(diagnostics, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    if failures:
        raise ValueError(f"camera reference rejected: {'; '.join(failures)}")
    return diagnostics


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    probe_parser = sub.add_parser("probe", help="映像フレーム時刻表を出力")
    probe_parser.add_argument("video", type=Path)
    probe_parser.add_argument("--output", type=Path, required=True)
    probe_parser.add_argument("--ffprobe", default="ffprobe")
    build_parser = sub.add_parser("build", help="動画・schema 10ログから独立XY参照を生成")
    build_parser.add_argument("video", type=Path)
    build_parser.add_argument("log", type=Path)
    build_parser.add_argument("calibration_csv", type=Path)
    build_parser.add_argument("camera_setup_json", type=Path)
    build_parser.add_argument("sync_events_json", type=Path)
    build_parser.add_argument("--output-dir", type=Path, required=True)
    build_parser.add_argument("--ffmpeg", default="ffmpeg")
    build_parser.add_argument("--ffprobe", default="ffprobe")
    args = parser.parse_args()
    try:
        if args.command == "probe":
            info = probe_video(args.video, args.ffprobe)
            write_frame_timestamps(info, args.output)
            del info["timestamps_ms"]
            print(json.dumps(info, indent=2, ensure_ascii=False))
        else:
            result = build_reference(
                args.video, args.log, args.calibration_csv, args.camera_setup_json,
                args.sync_events_json, args.output_dir,
                ffmpeg=args.ffmpeg, ffprobe=args.ffprobe,
            )
            print(json.dumps(result, indent=2, ensure_ascii=False))
    except (OSError, ValueError, subprocess.SubprocessError) as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(2) from exc


if __name__ == "__main__":
    main()
