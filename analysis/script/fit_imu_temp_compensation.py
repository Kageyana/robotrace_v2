#!/usr/bin/env python3
"""Fit and validate BMI088 gyro-Z temperature compensation.

The input files are the ``imu_temp_XXXXX.csv`` files written by the firmware.
The fit file is used only to estimate the slope; the validation file is never
used to refit it.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path


REQUIRED_COLUMNS = (
    "elapsed_ms",
    "temp_C",
    "gyroZMean_dps",
    "gyroZStd_dps",
)
LEGACY_ENCODER_COLUMN = "encoderMovement"
SETTING_SCALE = 1_000_000
SETTING_MIN = -100_000
SETTING_MAX = 100_000
INVALID_TEMPERATURE = -998.0


@dataclass(frozen=True)
class Sample:
    temperature_c: float
    gyro_z_dps: float
    gyro_z_std_dps: float
    encoder_movement: float | None = None


@dataclass(frozen=True)
class Dataset:
    path: Path
    samples: tuple[Sample, ...]
    source_rows: int


@dataclass(frozen=True)
class Regression:
    slope: float
    intercept: float


def _finite(value: float) -> bool:
    return math.isfinite(value)


def load_dataset(path: Path, max_std_dps: float) -> Dataset:
    """Load only stationary, low-noise, valid-temperature points."""
    samples: list[Sample] = []
    source_rows = 0
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        missing = [column for column in REQUIRED_COLUMNS if column not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f"{path}: missing columns: {', '.join(missing)}")
        has_encoder_movement = LEGACY_ENCODER_COLUMN in (reader.fieldnames or [])
        for row in reader:
            source_rows += 1
            try:
                temperature_c = float(row["temp_C"])
                gyro_z_dps = float(row["gyroZMean_dps"])
                gyro_z_std_dps = float(row["gyroZStd_dps"])
                encoder_movement = float(row[LEGACY_ENCODER_COLUMN]) if has_encoder_movement else None
            except (TypeError, ValueError):
                continue
            if not all(_finite(value) for value in (temperature_c, gyro_z_dps, gyro_z_std_dps)):
                continue
            if encoder_movement is not None and not _finite(encoder_movement):
                continue
            if temperature_c <= INVALID_TEMPERATURE:
                continue
            if gyro_z_std_dps > max_std_dps:
                continue
            if encoder_movement is not None and encoder_movement != 0.0:
                continue
            samples.append(Sample(temperature_c, gyro_z_dps, gyro_z_std_dps, encoder_movement))
    return Dataset(path, tuple(samples), source_rows)


def validate_requirements(dataset: Dataset, min_points: int, min_span_c: float, label: str) -> None:
    if len(dataset.samples) < min_points:
        raise ValueError(f"{label}: usable points {len(dataset.samples)} < {min_points}")
    span = temperature_span(dataset.samples)
    if span < min_span_c:
        raise ValueError(f"{label}: temperature span {span:.3f} °C < {min_span_c:.3f} °C")


def temperature_span(samples: tuple[Sample, ...] | list[Sample]) -> float:
    temperatures = [sample.temperature_c for sample in samples]
    return max(temperatures) - min(temperatures)


def linear_regression(samples: tuple[Sample, ...] | list[Sample]) -> Regression:
    if len(samples) < 2:
        raise ValueError("at least two points are required for regression")
    mean_x = statistics.fmean(sample.temperature_c for sample in samples)
    mean_y = statistics.fmean(sample.gyro_z_dps for sample in samples)
    denominator = sum((sample.temperature_c - mean_x) ** 2 for sample in samples)
    if denominator <= 0.0:
        raise ValueError("temperature values have no span")
    numerator = sum(
        (sample.temperature_c - mean_x) * (sample.gyro_z_dps - mean_y)
        for sample in samples
    )
    slope = numerator / denominator
    return Regression(slope, mean_y - slope * mean_x)


def robust_fit(samples: tuple[Sample, ...], iterations: int = 6) -> tuple[Regression, tuple[Sample, ...]]:
    """Iteratively reject residual outliers using a MAD-based threshold."""
    selected = list(samples)
    for _ in range(iterations):
        regression = linear_regression(selected)
        residuals = [
            sample.gyro_z_dps - (regression.intercept + regression.slope * sample.temperature_c)
            for sample in selected
        ]
        median_residual = statistics.median(residuals)
        deviations = [abs(residual - median_residual) for residual in residuals]
        mad = statistics.median(deviations)
        threshold = max(3.0 * 1.4826 * mad, 0.01)
        filtered = [sample for sample, residual in zip(selected, residuals) if abs(residual - median_residual) <= threshold]
        if len(filtered) == len(selected) or len(filtered) < 2:
            break
        selected = filtered
    return linear_regression(selected), tuple(selected)


def slope_and_rms(samples: tuple[Sample, ...] | list[Sample]) -> tuple[Regression, float]:
    regression = linear_regression(samples)
    mean_y = statistics.fmean(sample.gyro_z_dps for sample in samples)
    rms = math.sqrt(statistics.fmean((sample.gyro_z_dps - mean_y) ** 2 for sample in samples))
    return regression, rms


def corrected_samples(samples: tuple[Sample, ...], slope: float, reference_temperature_c: float) -> tuple[Sample, ...]:
    return tuple(
        Sample(
            sample.temperature_c,
            sample.gyro_z_dps - slope * (sample.temperature_c - reference_temperature_c),
            sample.gyro_z_std_dps,
            sample.encoder_movement,
        )
        for sample in samples
    )


def print_dataset(label: str, dataset: Dataset) -> None:
    print(
        f"{label}: source_rows={dataset.source_rows} usable_points={len(dataset.samples)} "
        f"temperature_span_C={temperature_span(dataset.samples):.6f}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("fit_csv", type=Path, help="coefficient-generation CSV")
    parser.add_argument("validation_csv", type=Path, help="independent validation CSV")
    parser.add_argument("--max-std-dps", type=float, default=0.5, help="maximum gyroZStd_dps (default: 0.5)")
    parser.add_argument("--min-points", type=int, default=10, help="minimum usable points (default: 10)")
    parser.add_argument("--min-span-c", type=float, default=3.0, help="minimum temperature span in °C (default: 3)")
    parser.add_argument("--setting-output", type=Path, help="optionally write the adopted zSlope_x1000000 without a newline")
    args = parser.parse_args()

    try:
        fit_data = load_dataset(args.fit_csv, args.max_std_dps)
        validation_data = load_dataset(args.validation_csv, args.max_std_dps)
        validate_requirements(fit_data, args.min_points, args.min_span_c, "fit")
        validate_requirements(validation_data, args.min_points, args.min_span_c, "validation")
        fit_regression, fit_used = robust_fit(fit_data.samples)
        if len(fit_used) < args.min_points or temperature_span(fit_used) < args.min_span_c:
            raise ValueError("fit: robust outlier removal left too few points or too little temperature span")

        fit_raw_regression, fit_raw_rms = slope_and_rms(fit_used)
        reference_temperature_c = statistics.fmean(sample.temperature_c for sample in fit_used)
        validation_raw_regression, validation_raw_rms = slope_and_rms(validation_data.samples)
        validation_corrected = corrected_samples(validation_data.samples, fit_regression.slope, reference_temperature_c)
        validation_corrected_regression, validation_corrected_rms = slope_and_rms(validation_corrected)
        setting_value = round(fit_regression.slope * SETTING_SCALE)

        print_dataset("fit", fit_data)
        print(f"fit_robust_points={len(fit_used)}")
        print(f"fit_raw_slope_dps_per_C={fit_raw_regression.slope:.9f}")
        print(f"fit_raw_rms_dps={fit_raw_rms:.9f}")
        print(f"validation_points={len(validation_data.samples)}")
        print(f"validation_temperature_span_C={temperature_span(validation_data.samples):.6f}")
        print(f"validation_raw_slope_dps_per_C={validation_raw_regression.slope:.9f}")
        print(f"validation_raw_rms_dps={validation_raw_rms:.9f}")
        print(f"validation_corrected_slope_dps_per_C={validation_corrected_regression.slope:.9f}")
        print(f"validation_corrected_rms_dps={validation_corrected_rms:.9f}")
        print(f"reference_temperature_C={reference_temperature_c:.6f}")
        print(f"zSlope_x1000000={setting_value}")

        slope_ok = abs(validation_corrected_regression.slope) <= abs(validation_raw_regression.slope) * 0.30
        rms_ok = validation_corrected_rms <= validation_raw_rms * 1.10
        setting_ok = SETTING_MIN <= setting_value <= SETTING_MAX
        adopted = slope_ok and rms_ok and setting_ok
        print(f"validation_slope_ratio={abs(validation_corrected_regression.slope) / abs(validation_raw_regression.slope) if validation_raw_regression.slope else 0.0:.6f}")
        print(f"validation_rms_ratio={validation_corrected_rms / validation_raw_rms if validation_raw_rms else 0.0:.6f}")
        print(f"adopt={'yes' if adopted else 'no'}")
        if args.setting_output is not None and adopted:
            args.setting_output.write_text(str(setting_value), encoding="ascii", newline="")
        return 0 if adopted else 1
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
