"""Reusable theoretical-lap calculation for BOOST_DISTANCE logs."""

from __future__ import annotations

import csv
import math
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from statistics import median
from typing import Iterable


PULSE_MILLIMETER = 54.324
PPAD_SAMPLES_PER_SEGMENT = 5
TARGET_INDEX_MIN_COVERAGE = 0.99
SPEED_EPSILON_MPS = 1e-9


class TheoreticalLapError(ValueError):
    """Raised when a log cannot produce a reliable theoretical lap."""


@dataclass(frozen=True)
class LogData:
    number: int
    path: Path
    columns: tuple[str, ...]
    params: dict[str, float]
    rows: tuple[dict[str, str], ...]


@dataclass(frozen=True)
class CourseSegment:
    index: int
    distance_start_mm: float
    distance_end_mm: float
    distance_mm: float
    x_start_mm: float
    y_start_mm: float
    x_end_mm: float
    y_end_mm: float
    roc_mm: float
    turn_sign: int
    signed_curvature_per_mm: float


@dataclass(frozen=True)
class CourseGeometry:
    base_log: int
    distance_mm: float
    segments: tuple[CourseSegment, ...]


@dataclass(frozen=True)
class TargetProfile:
    speeds_pulse_ms: tuple[float, ...]
    coverage_pct: float
    filled_count: int


@dataclass(frozen=True)
class AccelDecelTiming:
    lap_ms: float
    initial_speed_mps: float
    acceleration_transition_count: int
    deceleration_transition_count: int
    acceleration_unreached_count: int
    deceleration_unreached_count: int


@dataclass(frozen=True)
class TheoreticalLapResult:
    base_log: int
    secondary_log: int
    course_distance_mm: float
    theoretical_lap_ms: float
    instantaneous_theoretical_lap_ms: float
    accel_decel_adjustment_ms: float
    actual_lap_ms: float
    lap_gap_ms: float
    lap_gap_pct: float
    acceleration_mps2: float
    deceleration_mps2: float
    initial_speed_mps: float
    acceleration_transition_count: int
    deceleration_transition_count: int
    acceleration_unreached_count: int
    deceleration_unreached_count: int
    target_index_coverage_pct: float
    target_index_filled_count: int


def number(value: str | float | int | None) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def read_log(path: Path, log_number: int | None = None) -> LogData:
    with path.open("r", encoding="utf-8-sig", newline="") as file:
        reader = csv.reader(file)
        try:
            header = next(reader)
        except StopIteration as exc:
            raise TheoreticalLapError(f"empty log: {path}") from exc

        columns: list[str] = []
        params: dict[str, float] = {}
        parameter_started = False
        for raw_cell in header:
            cell = raw_cell.strip()
            if not cell:
                continue
            if "=" in cell:
                parameter_started = True
                key, value = cell.split("=", 1)
                params[key] = number(value)
            elif not parameter_started:
                columns.append(cell)

        rows_list: list[dict[str, str]] = []
        for row_number, raw in enumerate(reader, start=2):
            if not raw or all(not cell.strip() for cell in raw):
                continue
            if len(raw) < len(columns):
                raise TheoreticalLapError(
                    f"log row {row_number} has {len(raw)} fields; "
                    f"expected at least {len(columns)}"
                )
            rows_list.append(dict(zip(columns, raw[: len(columns)])))
        rows = tuple(rows_list)

    if log_number is None:
        try:
            log_number = int(path.stem)
        except ValueError as exc:
            raise TheoreticalLapError(f"log number is not numeric: {path.name}") from exc
    return LogData(log_number, path, tuple(columns), params, rows)


def _require_columns(log: LogData, required: Iterable[str]) -> None:
    missing = sorted(set(required).difference(log.columns))
    if missing:
        raise TheoreticalLapError(
            f"log {log.number} missing columns: {'|'.join(missing)}"
        )


def _finite_row_values(rows: tuple[dict[str, str], ...], column: str) -> list[float]:
    values = [number(row.get(column)) for row in rows]
    if not values or any(not math.isfinite(value) for value in values):
        raise TheoreticalLapError(f"invalid {column} data")
    return values


def build_course_geometry(
    base_log: LogData,
    *,
    pulse_per_mm: float = PULSE_MILLIMETER,
    samples_per_segment: int = PPAD_SAMPLES_PER_SEGMENT,
) -> CourseGeometry:
    """Build PPAD-aligned geometry solely from a first-run log."""

    if samples_per_segment <= 0:
        raise ValueError("samples_per_segment must be positive")
    if pulse_per_mm <= 0:
        raise ValueError("pulse_per_mm must be positive")
    _require_columns(
        base_log,
        ("encTotalOptimal", "ROC", "gyroVal_Z", "x", "y"),
    )
    if not base_log.rows:
        raise TheoreticalLapError(f"base log {base_log.number} has no rows")

    distances = [
        value / pulse_per_mm
        for value in _finite_row_values(base_log.rows, "encTotalOptimal")
    ]
    if distances[-1] <= 0:
        raise TheoreticalLapError(f"base log {base_log.number} has non-positive distance")
    if any(current <= previous for previous, current in zip(distances, distances[1:])):
        raise TheoreticalLapError(
            f"base log {base_log.number} encTotalOptimal is not strictly increasing"
        )

    segments: list[CourseSegment] = []
    previous_distance = 0.0
    previous_x = 0.0
    previous_y = 0.0
    for start in range(0, len(base_log.rows), samples_per_segment):
        group = base_log.rows[start : start + samples_per_segment]
        end_row = group[-1]
        end_distance = number(end_row.get("encTotalOptimal")) / pulse_per_mm
        end_x = number(end_row.get("x"))
        end_y = number(end_row.get("y"))
        roc_values = [number(row.get("ROC")) for row in group]
        gyro_values = [number(row.get("gyroVal_Z")) for row in group]
        if any(
            not math.isfinite(value)
            for value in (end_x, end_y, *roc_values, *gyro_values)
        ):
            raise TheoreticalLapError(
                f"base log {base_log.number} has invalid course geometry values"
            )

        roc_mm = float(median(roc_values))
        gyro_median = float(median(gyro_values))
        turn_sign = 1 if gyro_median > 0 else (-1 if gyro_median < 0 else 0)
        signed_curvature = turn_sign / roc_mm if turn_sign and roc_mm > 0 else 0.0
        segment_distance = end_distance - previous_distance
        if segment_distance <= 0:
            raise TheoreticalLapError(
                f"base log {base_log.number} has a non-positive PPAD segment"
            )

        segments.append(
            CourseSegment(
                index=len(segments),
                distance_start_mm=previous_distance,
                distance_end_mm=end_distance,
                distance_mm=segment_distance,
                x_start_mm=previous_x,
                y_start_mm=previous_y,
                x_end_mm=end_x,
                y_end_mm=end_y,
                roc_mm=roc_mm,
                turn_sign=turn_sign,
                signed_curvature_per_mm=signed_curvature,
            )
        )
        previous_distance = end_distance
        previous_x = end_x
        previous_y = end_y

    return CourseGeometry(base_log.number, distances[-1], tuple(segments))


def _select_index_speed(rows: list[dict[str, str]]) -> float:
    values = [number(row.get("targetSpeed")) for row in rows]
    if not values or any(not math.isfinite(value) for value in values):
        raise TheoreticalLapError("invalid targetSpeed data")
    counts = Counter(values)
    highest_count = max(counts.values())
    tied = {value for value, count in counts.items() if count == highest_count}
    return next(value for value in reversed(values) if value in tied)


def build_target_profile(
    secondary_log: LogData,
    segment_count: int,
    *,
    minimum_coverage: float = TARGET_INDEX_MIN_COVERAGE,
) -> TargetProfile:
    if segment_count <= 0:
        raise TheoreticalLapError("course has no segments")
    _require_columns(secondary_log, ("optimalIndex", "targetSpeed"))

    grouped: dict[int, list[dict[str, str]]] = {}
    for row in secondary_log.rows:
        index_value = number(row.get("optimalIndex"))
        if not math.isfinite(index_value) or not index_value.is_integer():
            raise TheoreticalLapError(
                f"log {secondary_log.number} has invalid optimalIndex"
            )
        index = int(index_value)
        if index < 0:
            raise TheoreticalLapError(
                f"log {secondary_log.number} has negative optimalIndex"
            )
        if index >= segment_count:
            raise TheoreticalLapError(
                f"log {secondary_log.number} optimalIndex exceeds base course"
            )
        grouped.setdefault(index, []).append(row)

    observed = {index: _select_index_speed(rows) for index, rows in grouped.items()}
    coverage = len(observed) / segment_count
    if 0 not in observed or (segment_count - 1) not in observed:
        raise TheoreticalLapError(
            f"log {secondary_log.number} target profile has an edge gap"
        )
    if coverage + 1e-12 < minimum_coverage:
        raise TheoreticalLapError(
            f"log {secondary_log.number} target index coverage is {coverage * 100:.2f}%"
        )

    speeds: list[float | None] = [observed.get(index) for index in range(segment_count)]
    missing = [index for index, speed in enumerate(speeds) if speed is None]
    for index in missing:
        left = index - 1
        while left >= 0 and speeds[left] is None:
            left -= 1
        right = index + 1
        while right < segment_count and speeds[right] is None:
            right += 1
        if left < 0 or right >= segment_count:
            raise TheoreticalLapError(
                f"log {secondary_log.number} target profile cannot be interpolated"
            )
        ratio = (index - left) / (right - left)
        speeds[index] = float(speeds[left]) + (
            float(speeds[right]) - float(speeds[left])
        ) * ratio

    completed = tuple(float(speed) for speed in speeds if speed is not None)
    if len(completed) != segment_count or any(speed <= 0 for speed in completed):
        raise TheoreticalLapError(
            f"log {secondary_log.number} target profile contains non-positive speed"
        )
    return TargetProfile(completed, coverage * 100.0, len(missing))


def calculate_accel_decel_timing(
    geometry: CourseGeometry,
    profile: TargetProfile,
    *,
    acceleration_mps2: float,
    deceleration_mps2: float,
    pulse_per_mm: float = PULSE_MILLIMETER,
    initial_speed_mps: float,
) -> AccelDecelTiming:
    """Integrate causal target-speed changes with acceleration limits."""

    if acceleration_mps2 <= 0 or not math.isfinite(acceleration_mps2):
        raise TheoreticalLapError("invalid acceleration setting")
    if deceleration_mps2 <= 0 or not math.isfinite(deceleration_mps2):
        raise TheoreticalLapError("invalid deceleration setting")
    if pulse_per_mm <= 0:
        raise ValueError("pulse_per_mm must be positive")
    if len(geometry.segments) != len(profile.speeds_pulse_ms):
        raise TheoreticalLapError("course and target profile length mismatch")

    target_speeds_mps = tuple(
        speed / pulse_per_mm for speed in profile.speeds_pulse_ms
    )
    if not target_speeds_mps or any(speed <= 0 for speed in target_speeds_mps):
        raise TheoreticalLapError("target profile contains non-positive speed")

    if initial_speed_mps <= 0 or not math.isfinite(initial_speed_mps):
        raise TheoreticalLapError("invalid initial speed")

    speed_mps = initial_speed_mps
    total_time_s = 0.0
    acceleration_transition_count = 0
    deceleration_transition_count = 0
    acceleration_unreached_count = 0
    deceleration_unreached_count = 0

    for segment, target_mps in zip(geometry.segments, target_speeds_mps):
        distance_m = segment.distance_mm / 1000.0
        if distance_m <= 0:
            raise TheoreticalLapError("course contains non-positive segment distance")

        speed_delta = target_mps - speed_mps
        if abs(speed_delta) <= SPEED_EPSILON_MPS:
            total_time_s += distance_m / target_mps
            speed_mps = target_mps
            continue

        if speed_delta > 0:
            rate_mps2 = acceleration_mps2
            transition_distance_m = (
                target_mps * target_mps - speed_mps * speed_mps
            ) / (2.0 * rate_mps2)
            acceleration_transition_count += 1
            if transition_distance_m > distance_m:
                exit_speed_mps = math.sqrt(
                    speed_mps * speed_mps + 2.0 * rate_mps2 * distance_m
                )
                total_time_s += 2.0 * distance_m / (speed_mps + exit_speed_mps)
                speed_mps = exit_speed_mps
                acceleration_unreached_count += 1
            else:
                total_time_s += (target_mps - speed_mps) / rate_mps2
                total_time_s += (distance_m - transition_distance_m) / target_mps
                speed_mps = target_mps
            continue

        rate_mps2 = deceleration_mps2
        transition_distance_m = (
            speed_mps * speed_mps - target_mps * target_mps
        ) / (2.0 * rate_mps2)
        deceleration_transition_count += 1
        if transition_distance_m > distance_m:
            exit_speed_sq = speed_mps * speed_mps - 2.0 * rate_mps2 * distance_m
            if exit_speed_sq <= 0:
                raise TheoreticalLapError("deceleration reaches zero before target")
            exit_speed_mps = math.sqrt(exit_speed_sq)
            total_time_s += 2.0 * distance_m / (speed_mps + exit_speed_mps)
            speed_mps = exit_speed_mps
            deceleration_unreached_count += 1
        else:
            total_time_s += (speed_mps - target_mps) / rate_mps2
            total_time_s += (distance_m - transition_distance_m) / target_mps
            speed_mps = target_mps

    return AccelDecelTiming(
        lap_ms=total_time_s * 1000.0,
        initial_speed_mps=initial_speed_mps,
        acceleration_transition_count=acceleration_transition_count,
        deceleration_transition_count=deceleration_transition_count,
        acceleration_unreached_count=acceleration_unreached_count,
        deceleration_unreached_count=deceleration_unreached_count,
    )


def calculate_theoretical_lap(
    geometry: CourseGeometry,
    secondary_log: LogData,
    *,
    pulse_per_mm: float = PULSE_MILLIMETER,
    acceleration_mps2: float | None = None,
    deceleration_mps2: float | None = None,
    initial_speed_mps: float | None = None,
) -> TheoreticalLapResult:
    if pulse_per_mm <= 0:
        raise ValueError("pulse_per_mm must be positive")
    required_columns = ["cntlog", "optimalIndex", "targetSpeed"]
    if initial_speed_mps is None:
        required_columns.append("encCurrentN")
    _require_columns(secondary_log, required_columns)
    if not secondary_log.rows:
        raise TheoreticalLapError(f"secondary log {secondary_log.number} has no rows")

    profile = build_target_profile(secondary_log, len(geometry.segments))
    instantaneous_ms = sum(
        segment.distance_mm * pulse_per_mm / target_speed
        for segment, target_speed in zip(geometry.segments, profile.speeds_pulse_ms)
    )
    if acceleration_mps2 is None:
        acceleration_mps2 = secondary_log.params.get("tgtParam.acceleF", math.nan)
    if deceleration_mps2 is None:
        deceleration_mps2 = secondary_log.params.get("tgtParam.acceleD", math.nan)
    if initial_speed_mps is None:
        initial_speed_pulse_ms = number(secondary_log.rows[0].get("encCurrentN"))
        if not math.isfinite(initial_speed_pulse_ms) or initial_speed_pulse_ms <= 0:
            raise TheoreticalLapError(
                f"secondary log {secondary_log.number} has invalid initial encCurrentN"
            )
        initial_speed_mps = initial_speed_pulse_ms / pulse_per_mm
    timing = calculate_accel_decel_timing(
        geometry,
        profile,
        acceleration_mps2=acceleration_mps2,
        deceleration_mps2=deceleration_mps2,
        pulse_per_mm=pulse_per_mm,
        initial_speed_mps=initial_speed_mps,
    )
    theoretical_ms = timing.lap_ms
    actual_ms = number(secondary_log.rows[-1].get("cntlog"))
    if not math.isfinite(actual_ms) or actual_ms <= 0:
        raise TheoreticalLapError(
            f"secondary log {secondary_log.number} has invalid final cntlog"
        )
    gap_ms = actual_ms - theoretical_ms
    return TheoreticalLapResult(
        base_log=geometry.base_log,
        secondary_log=secondary_log.number,
        course_distance_mm=geometry.distance_mm,
        theoretical_lap_ms=theoretical_ms,
        instantaneous_theoretical_lap_ms=instantaneous_ms,
        accel_decel_adjustment_ms=theoretical_ms - instantaneous_ms,
        actual_lap_ms=actual_ms,
        lap_gap_ms=gap_ms,
        lap_gap_pct=100.0 * gap_ms / theoretical_ms,
        acceleration_mps2=acceleration_mps2,
        deceleration_mps2=deceleration_mps2,
        initial_speed_mps=timing.initial_speed_mps,
        acceleration_transition_count=timing.acceleration_transition_count,
        deceleration_transition_count=timing.deceleration_transition_count,
        acceleration_unreached_count=timing.acceleration_unreached_count,
        deceleration_unreached_count=timing.deceleration_unreached_count,
        target_index_coverage_pct=profile.coverage_pct,
        target_index_filled_count=profile.filled_count,
    )


def geometry_rows(geometry: CourseGeometry) -> list[dict[str, int | float]]:
    return [
        {
            "base_log": geometry.base_log,
            "segment_index": segment.index,
            "distance_start_mm": segment.distance_start_mm,
            "distance_end_mm": segment.distance_end_mm,
            "segment_distance_mm": segment.distance_mm,
            "x_start_mm": segment.x_start_mm,
            "y_start_mm": segment.y_start_mm,
            "x_end_mm": segment.x_end_mm,
            "y_end_mm": segment.y_end_mm,
            "roc_mm": segment.roc_mm,
            "turn_sign": segment.turn_sign,
            "signed_curvature_per_mm": segment.signed_curvature_per_mm,
        }
        for segment in geometry.segments
    ]
