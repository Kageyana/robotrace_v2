#!/usr/bin/env python3
"""Validate and summarize robotrace Version 3/4 IMU-temperature logs."""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


PULSE_METER = 53424.0
TEMP_INVALID_CDEG = -32768
EXPECTED_CORE_FIELDS = {
    "cntlog",
    "encCurrentN",
    "gyroVal_Z",
    "courseMarker",
    "encTotalOptimal",
    "ROC",
    "targetSpeed",
    "optimalIndex",
    "slipFlag",
    "slipFlagLat",
    "lineTraceCtrl",
    "targetAngularvelo",
    "batteryVoltage_mV",
    "imuTemp_cdegC",
    "encCurrentCorr_p",
    "lineValid",
    "pathErrorY_mm",
    "pathErrorHeading_cdeg",
    "pathState",
    "x",
    "y",
}


@dataclass
class LogData:
    path: Path
    header_layout: str
    fields: list[str]
    parameters: dict[str, str]
    rows: list[dict[str, str]]
    row_length_errors: int


def number(value: str | None, default: float = math.nan) -> float:
    try:
        return float(value) if value is not None else default
    except (TypeError, ValueError):
        return default


def integer(value: str | None, default: int = 0) -> int:
    value_number = number(value)
    return int(value_number) if math.isfinite(value_number) else default


def read_log(path: Path) -> LogData:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        parameter_line = stream.readline()
        column_line = stream.readline()
        if not parameter_line:
            raise ValueError(f"missing parameter line: {path}")
        if not parameter_line.endswith(("\n", "\r")):
            raise ValueError(f"parameter line has no newline: {path}")
        if not column_line:
            raise ValueError(f"missing column header: {path}")
        if not column_line.endswith(("\n", "\r")):
            raise ValueError(f"column header has no newline: {path}")

        parameter_items = next(csv.reader([parameter_line]))
        fields = next(csv.reader([column_line]))
        if parameter_items and parameter_items[-1] == "":
            parameter_items = parameter_items[:-1]
        if fields and fields[-1] == "":
            fields = fields[:-1]
        if not parameter_items or any("=" not in item or not item.split("=", 1)[0] for item in parameter_items):
            raise ValueError(f"invalid parameter line: {path}")
        if not fields or any(not field for field in fields):
            raise ValueError(f"invalid column header: {path}")

        header_layout = "split_parameters_then_columns"
        data_records = list(csv.reader(stream))

    parameters: dict[str, str] = {}
    for item in parameter_items:
        if "=" in item:
            key, value = item.split("=", 1)
            parameters[key] = value

    rows: list[dict[str, str]] = []
    row_length_errors = 0
    for record in data_records:
        if record and record[-1] == "":
            record = record[:-1]
        if len(record) != len(fields):
            row_length_errors += 1
            continue
        rows.append(dict(zip(fields, record)))
    return LogData(path, header_layout, fields, parameters, rows, row_length_errors)


def unwrap_times(rows: list[dict[str, str]]) -> tuple[list[int], bool, int]:
    raw = [integer(row.get("cntlog")) for row in rows]
    if not raw:
        return [], False, 0
    unwrapped = [raw[0]]
    wrap_offset = 0
    monotonic = True
    for previous, current in zip(raw, raw[1:]):
        if current < previous:
            if previous - current > 32768:
                wrap_offset += 65536
            else:
                monotonic = False
        unwrapped.append(current + wrap_offset)
    gaps = [b - a for a, b in zip(unwrapped, unwrapped[1:])]
    if any(gap <= 0 for gap in gaps):
        monotonic = False
    return unwrapped, monotonic, max(gaps, default=0)


def cumulative_gyro_angle(rows: list[dict[str, str]], times_ms: list[int]) -> list[float]:
    gyro = [number(row.get("gyroVal_Z"), 0.0) for row in rows]
    angles = [0.0]
    for index in range(1, len(rows)):
        dt_s = (times_ms[index] - times_ms[index - 1]) / 1000.0
        angles.append(angles[-1] + 0.5 * (gyro[index - 1] + gyro[index]) * dt_s)
    return angles


def yaw_angle_source(log: LogData) -> str:
    """Return the source used for the logged yaw angle comparison."""
    version = integer(log.parameters.get("logSchemaVersion"), -1)
    if version == 4:
        return "stored_1ms_angle"
    return "integrated_log_gyro"


def yaw_angle_series(log: LogData, times_ms: list[int]) -> tuple[list[float], str]:
    """Use Version 4's stored 1 ms angle, or integrate the legacy gyro log."""
    source = yaw_angle_source(log)
    if source == "stored_1ms_angle":
        return [number(row.get("imuYawAngle_deg")) for row in log.rows], source
    return cumulative_gyro_angle(log.rows, times_ms), source


def lap_end_indices(log: LogData, expected_laps: int) -> list[int]:
    events: list[int] = []
    previous_marker = 0
    for index, row in enumerate(log.rows):
        marker = integer(row.get("courseMarker"))
        if marker == 1 and previous_marker != 1:
            events.append(index)
        previous_marker = marker
    if not events or expected_laps <= 0:
        return []

    # このコースでは右マーカーが1周内に2回現れる。近接した2イベントを
    # 同じ周回のグループとして扱い、十分離れた先頭イベントだけを採用する。
    selected = [events[0]]
    last_distance = integer(log.rows[events[0]].get("encTotalOptimal"))
    for index in events[1:]:
        distance = integer(log.rows[index].get("encTotalOptimal"))
        if distance - last_distance > 2.0 * PULSE_METER:
            selected.append(index)
            last_distance = distance

    # ログ開始直後のイベントはスタート地点なので周回終了から除外する。
    if selected and (
        selected[0] <= 10
        or integer(log.rows[selected[0]].get("encTotalOptimal")) < PULSE_METER
    ):
        selected = selected[1:]
    ends = selected[:expected_laps]
    if len(ends) < expected_laps and log.rows:
        ends.append(len(log.rows) - 1)
    return ends[:expected_laps]


def summarize(log: LogData) -> tuple[dict[str, object], list[float], list[int], list[float]]:
    times_ms, monotonic, max_gap_ms = unwrap_times(log.rows)
    angles, angle_source = yaw_angle_series(log, times_ms) if times_ms else ([], yaw_angle_source(log))
    temperatures = [integer(row.get("imuTemp_cdegC"), TEMP_INVALID_CDEG) for row in log.rows]
    valid_temperatures = [value / 100.0 for value in temperatures if value != TEMP_INVALID_CDEG]
    expected_laps = max(integer(log.parameters.get("sgMarkerAtLogEnd")) - 1, 0)
    mode = integer(log.parameters.get("optimalTrace"), -1)
    emergency = integer(log.parameters.get("emcStop"), -1)
    lap_indices = lap_end_indices(log, expected_laps) if mode == 0 and emergency == 0 else []
    lap_angles = [angles[index] for index in lap_indices] if angles else []
    lap_errors = [angle - 360.0 * (index + 1) for index, angle in enumerate(lap_angles)]
    start_time = times_ms[0] if times_ms else 0
    lap_times_s: list[float] = []
    previous_time = start_time
    for index in lap_indices:
        lap_times_s.append((times_ms[index] - previous_time) / 1000.0)
        previous_time = times_ms[index]

    tcal = number(log.parameters.get("imuTempCalibration_C"))
    temp_integral = 0.0
    if times_ms and math.isfinite(tcal):
        for index in range(1, len(log.rows)):
            previous_temp = temperatures[index - 1]
            current_temp = temperatures[index]
            if previous_temp == TEMP_INVALID_CDEG or current_temp == TEMP_INVALID_CDEG:
                continue
            dt_s = (times_ms[index] - times_ms[index - 1]) / 1000.0
            temp_integral += 0.5 * (previous_temp / 100.0 + current_temp / 100.0 - 2.0 * tcal) * dt_s

    fields = set(log.fields)
    schema_version = integer(log.parameters.get("logSchemaVersion"), -1)
    required_fields = set(EXPECTED_CORE_FIELDS)
    if schema_version == 4:
        required_fields.add("imuYawAngle_deg")
    missing_fields = sorted(required_fields - fields)
    forbidden_pwm_fields = sorted(fields & {"motorpwmL", "motorpwmR"})
    forbidden_fields = sorted(fields & {"motorVoltageCmdL_mV", "motorVoltageCmdR_mV"})
    final = log.rows[-1] if log.rows else {}
    speed_errors = [
        (number(row.get("encCurrentN"), 0.0) - number(row.get("targetSpeed"), 0.0))
        * 1000.0
        / PULSE_METER
        for row in log.rows
    ]
    summary: dict[str, object] = {
        "log": log.path.stem,
        "header_layout": log.header_layout,
        "firmware_header_compatible": 1,
        "gitCommit": log.parameters.get("gitCommit", ""),
        "branch": log.parameters.get("branch", ""),
        "logSchemaVersion": schema_version,
        "optimalTrace": mode,
        "emcStop": emergency,
        "rows": len(log.rows),
        "row_length_errors": log.row_length_errors,
        "missing_required_columns": ";".join(missing_fields),
        "forbidden_motor_pwm_columns": ";".join(forbidden_pwm_fields),
        "forbidden_voltage_columns": ";".join(forbidden_fields),
        "cntlog_monotonic": int(monotonic),
        "max_cntlog_gap_ms": max_gap_ms,
        "duration_s": round((times_ms[-1] - times_ms[0]) / 1000.0, 3) if len(times_ms) > 1 else 0.0,
        "batteryVoltage_V": number(log.parameters.get("batteryVoltage_V")),
        "imuTempCalibrationValid": integer(log.parameters.get("imuTempCalibrationValid"), -1),
        "imuTempCalibrationStart_C": number(log.parameters.get("imuTempCalibrationStart_C")),
        "imuTempCalibration_C": tcal,
        "imuTempCalibrationEnd_C": number(log.parameters.get("imuTempCalibrationEnd_C")),
        "imuTempCalibrationSamples": integer(log.parameters.get("imuTempCalibrationSamples"), -1),
        "imuTempCalibrationReadErrors": integer(log.parameters.get("imuTempCalibrationReadErrors"), -1),
        "imuGyroOffsetZ_dps": number(log.parameters.get("imuGyroOffsetZ_dps")),
        "imuTempCompEnabled": integer(log.parameters.get("imuTempCompEnabled"), -1),
        "imuTempCoeff_dpsPerC": number(log.parameters.get("imuTempCoeff_dpsPerC")),
        "imuTempEnd_C": number(log.parameters.get("imuTempEnd_C")),
        "imuTempInvalidRows": temperatures.count(TEMP_INVALID_CDEG),
        "imuTempMin_C": min(valid_temperatures, default=math.nan),
        "imuTempMax_C": max(valid_temperatures, default=math.nan),
        "imuTempFinal_C": valid_temperatures[-1] if valid_temperatures else math.nan,
        "tempIntegral_Cs": temp_integral,
        "sgMarkerAtLogEnd": integer(log.parameters.get("sgMarkerAtLogEnd"), -1),
        "lap_count_recovered": len(lap_indices),
        "lap_times_s": ";".join(f"{value:.3f}" for value in lap_times_s),
        "yaw_angle_source": angle_source,
        "final_imuYawAngle_deg": angles[-1] if angles else math.nan,
        "gyro_angle_deg": angles[-1] if angles else math.nan,
        "six_lap_yaw_error_deg": lap_errors[-1] if len(lap_errors) == 6 else math.nan,
        "lap_yaw_errors_deg": ";".join(f"{value:.3f}" for value in lap_errors),
        "final_x_mm": number(final.get("x")),
        "final_y_mm": number(final.get("y")),
        "speed_error_rms_mps": math.sqrt(sum(value * value for value in speed_errors) / len(speed_errors)) if speed_errors else math.nan,
        "slip_ratio_pct": 100.0 * sum(integer(row.get("slipFlag")) != 0 for row in log.rows) / len(log.rows) if log.rows else math.nan,
        "slip_lat_ratio_pct": 100.0 * sum(integer(row.get("slipFlagLat")) != 0 for row in log.rows) / len(log.rows) if log.rows else math.nan,
    }
    return summary, angles, lap_indices, lap_errors


def write_summary(path: Path, summaries: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)


def temperature_integral_series(log: LogData, times_ms: list[int]) -> list[float]:
    tcal = number(log.parameters.get("imuTempCalibration_C"))
    values = [integer(row.get("imuTemp_cdegC"), TEMP_INVALID_CDEG) for row in log.rows]
    integrals = [0.0]
    for index in range(1, len(log.rows)):
        previous = values[index - 1]
        current = values[index]
        increment = 0.0
        if previous != TEMP_INVALID_CDEG and current != TEMP_INVALID_CDEG and math.isfinite(tcal):
            dt_s = (times_ms[index] - times_ms[index - 1]) / 1000.0
            increment = 0.5 * (previous / 100.0 + current / 100.0 - 2.0 * tcal) * dt_s
        integrals.append(integrals[-1] + increment)
    return integrals


def fit_temperature_coefficient(
    logs: list[LogData], compensation_enabled: int = 0
) -> list[dict[str, object]]:
    eligible: list[tuple[LogData, list[int], list[int], list[float], list[float], str]] = []
    for log in logs:
        if (
            integer(log.parameters.get("optimalTrace"), -1) != 0
            or integer(log.parameters.get("emcStop"), -1) != 0
            or integer(log.parameters.get("sgMarkerAtLogEnd"), -1) != 7
            or integer(log.parameters.get("imuTempCompEnabled"), -1) != compensation_enabled
            or number(log.parameters.get("batteryVoltage_V"), 0.0) < 7.5
        ):
            continue
        times, monotonic, _ = unwrap_times(log.rows)
        lap_indices = lap_end_indices(log, 6)
        if not monotonic or len(lap_indices) != 6:
            continue
        angles, angle_source = yaw_angle_series(log, times)
        integrals = temperature_integral_series(log, times)
        eligible.append((log, times, lap_indices, angles, integrals, angle_source))

    def fit(subset: list[tuple[LogData, list[int], list[int], list[float], list[float], str]]) -> float:
        run_count = len(subset)
        rows: list[list[float]] = []
        targets: list[float] = []
        for run_index, (_, times, lap_indices, angles, integrals, _) in enumerate(subset):
            start = times[0]
            for lap_number, index in enumerate(lap_indices, 1):
                design = [0.0] * (run_count + 1)
                design[run_index] = (times[index] - start) / 1000.0
                design[-1] = integrals[index]
                rows.append(design)
                targets.append(angles[index] - 360.0 * lap_number)
        if not rows:
            return math.nan
        coefficients, _, _, _ = np.linalg.lstsq(np.asarray(rows), np.asarray(targets), rcond=None)
        return float(coefficients[-1])

    results: list[dict[str, object]] = []
    groups: dict[str, list[tuple[LogData, list[int], list[int], list[float], list[float], str]]] = {}
    for item in eligible:
        groups.setdefault(item[5], []).append(item)
    if not groups:
        groups[""] = []
    for source, group in groups.items():
        full = fit(group)
        results.append({
            "fit": "all",
            "excluded_log": "",
            "run_count": len(group),
            "yaw_angle_source": source,
            "temperature_coefficient_dps_per_C": full,
            "stored_value_x1000000": round(full * 1_000_000) if math.isfinite(full) else "",
        })
        for index, item in enumerate(group):
            coefficient = fit(group[:index] + group[index + 1 :])
            results.append({
                "fit": "leave_one_run_out",
                "excluded_log": item[0].path.stem,
                "run_count": len(group) - 1,
                "yaw_angle_source": source,
                "temperature_coefficient_dps_per_C": coefficient,
                "stored_value_x1000000": round(coefficient * 1_000_000) if math.isfinite(coefficient) else "",
            })
    return results


def match_compensation_runs(
    logs: list[LogData], summaries: list[dict[str, object]]
) -> list[dict[str, object]]:
    summary_by_log = {str(summary["log"]): summary for summary in summaries}
    off_logs = [
        log
        for log in logs
        if integer(log.parameters.get("optimalTrace"), -1) == 0
        and integer(log.parameters.get("emcStop"), -1) == 0
        and integer(log.parameters.get("sgMarkerAtLogEnd"), -1) == 7
        and integer(log.parameters.get("imuTempCompEnabled"), -1) == 0
    ]
    on_logs = [
        log
        for log in logs
        if integer(log.parameters.get("optimalTrace"), -1) == 0
        and integer(log.parameters.get("emcStop"), -1) == 0
        and integer(log.parameters.get("sgMarkerAtLogEnd"), -1) == 7
        and integer(log.parameters.get("imuTempCompEnabled"), -1) == 1
    ]
    unused = {log.path.stem for log in off_logs}
    pairs: list[dict[str, object]] = []
    for on_log in sorted(
        on_logs, key=lambda item: number(item.parameters.get("imuTempCalibration_C"))
    ):
        on_summary = summary_by_log[on_log.path.stem]
        source = yaw_angle_source(on_log)
        candidates = [
            log
            for log in off_logs
            if log.path.stem in unused and yaw_angle_source(log) == source
        ]
        if not candidates:
            break
        off_log = min(
            candidates,
            key=lambda item: abs(
                number(item.parameters.get("imuTempCalibration_C"))
                - number(on_log.parameters.get("imuTempCalibration_C"))
            )
            + 2.0
            * abs(
                number(item.parameters.get("batteryVoltage_V"))
                - number(on_log.parameters.get("batteryVoltage_V"))
            ),
        )
        unused.remove(off_log.path.stem)
        off_summary = summary_by_log[off_log.path.stem]
        temperature_difference = number(on_summary["imuTempCalibration_C"]) - number(
            off_summary["imuTempCalibration_C"]
        )
        voltage_difference = number(on_summary["batteryVoltage_V"]) - number(
            off_summary["batteryVoltage_V"]
        )
        on_error = abs(number(on_summary["six_lap_yaw_error_deg"]))
        off_error = abs(number(off_summary["six_lap_yaw_error_deg"]))
        pairs.append(
            {
                "off_log": off_log.path.stem,
                "on_log": on_log.path.stem,
                "yaw_angle_source": source,
                "off_tcal_C": number(off_summary["imuTempCalibration_C"]),
                "on_tcal_C": number(on_summary["imuTempCalibration_C"]),
                "tcal_difference_C": temperature_difference,
                "off_battery_V": number(off_summary["batteryVoltage_V"]),
                "on_battery_V": number(on_summary["batteryVoltage_V"]),
                "battery_difference_V": voltage_difference,
                "pair_within_limits": int(
                    abs(temperature_difference) <= 1.0
                    and abs(voltage_difference) <= 0.25
                ),
                "off_yaw_error_deg": off_error,
                "on_yaw_error_deg": on_error,
                "yaw_error_change_deg": on_error - off_error,
            }
        )
    return pairs


def plot_compensation_comparison(path: Path, pairs: list[dict[str, object]]) -> None:
    if not pairs:
        return
    positions = np.arange(len(pairs))
    width = 0.38
    fig, axis = plt.subplots(figsize=(9, 5))
    axis.bar(
        positions - width / 2,
        [number(item["off_yaw_error_deg"]) for item in pairs],
        width,
        label="compensation OFF",
    )
    axis.bar(
        positions + width / 2,
        [number(item["on_yaw_error_deg"]) for item in pairs],
        width,
        label="compensation ON",
    )
    axis.set_xticks(
        positions,
        [f"{item['off_log']} / {item['on_log']}" for item in pairs],
        rotation=20,
    )
    axis.set_ylabel("Absolute six-lap yaw error [deg]")
    axis.grid(True, axis="y", alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_temperature(path: Path, logs: list[LogData]) -> None:
    fig, axis = plt.subplots(figsize=(10, 5))
    for log in logs:
        times, _, _ = unwrap_times(log.rows)
        if not times:
            continue
        values = [integer(row.get("imuTemp_cdegC"), TEMP_INVALID_CDEG) for row in log.rows]
        x = [(value - times[0]) / 1000.0 for value in times]
        y = [value / 100.0 if value != TEMP_INVALID_CDEG else math.nan for value in values]
        axis.plot(x, y, linewidth=1.0, label=log.path.stem)
    axis.set_xlabel("Time [s]")
    axis.set_ylabel("BMI088 temperature [°C]")
    axis.grid(True, alpha=0.3)
    axis.legend(ncol=3, fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_yaw_errors(path: Path, summaries: list[dict[str, object]]) -> None:
    fig, axis = plt.subplots(figsize=(9, 5))
    for summary in summaries:
        text = str(summary["lap_yaw_errors_deg"])
        if not text:
            continue
        values = [float(value) for value in text.split(";")]
        axis.plot(
            range(1, len(values) + 1),
            values,
            marker="o",
            label=f"{summary['log']} [{summary['yaw_angle_source']}]",
        )
    axis.axhline(0.0, color="black", linewidth=0.8)
    axis.set_xlabel("Lap")
    axis.set_ylabel("Integrated yaw error [deg]")
    axis.grid(True, alpha=0.3)
    axis.legend(ncol=3, fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_xy(path: Path, logs: list[LogData]) -> None:
    fig, axis = plt.subplots(figsize=(7, 7))
    for log in logs:
        if integer(log.parameters.get("optimalTrace"), -1) != 0 or integer(log.parameters.get("emcStop"), -1) != 0:
            continue
        axis.plot(
            [number(row.get("x")) for row in log.rows],
            [number(row.get("y")) for row in log.rows],
            linewidth=0.8,
            label=f"{log.path.stem} [v{integer(log.parameters.get('logSchemaVersion'), -1)}/{yaw_angle_source(log)}]",
        )
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x [mm]")
    axis.set_ylabel("y [mm]")
    axis.grid(True, alpha=0.3)
    axis.legend(ncol=3, fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_latest_overview(path: Path, log: LogData) -> None:
    times, _, _ = unwrap_times(log.rows)
    elapsed = [(value - times[0]) / 1000.0 for value in times]
    speed = [number(row.get("encCurrentN")) * 1000.0 / PULSE_METER for row in log.rows]
    target_speed = [number(row.get("targetSpeed")) * 1000.0 / PULSE_METER for row in log.rows]
    gyro = [number(row.get("gyroVal_Z")) for row in log.rows]
    target_gyro = [number(row.get("targetAngularvelo")) for row in log.rows]
    temperature = [
        integer(row.get("imuTemp_cdegC"), TEMP_INVALID_CDEG) / 100.0
        if integer(row.get("imuTemp_cdegC"), TEMP_INVALID_CDEG) != TEMP_INVALID_CDEG
        else math.nan
        for row in log.rows
    ]
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    axes[0, 0].plot([number(row.get("x")) for row in log.rows], [number(row.get("y")) for row in log.rows])
    axes[0, 0].set_aspect("equal", adjustable="box")
    axes[0, 0].set(xlabel="x [mm]", ylabel="y [mm]", title="XY trajectory")
    axes[0, 1].plot(elapsed, speed, label="actual")
    axes[0, 1].plot(elapsed, target_speed, label="target")
    axes[0, 1].set(xlabel="Time [s]", ylabel="Speed [m/s]", title="Speed tracking")
    axes[0, 1].legend()
    axes[1, 0].plot(elapsed, gyro, label="actual")
    axes[1, 0].plot(elapsed, target_gyro, label="target", alpha=0.7)
    axes[1, 0].set(xlabel="Time [s]", ylabel="Yaw rate [deg/s]", title="Angular velocity")
    axes[1, 0].legend()
    axes[1, 1].plot(elapsed, temperature)
    axes[1, 1].set(xlabel="Time [s]", ylabel="Temperature [°C]", title="BMI088 temperature")
    for axis in axes.flat:
        axis.grid(True, alpha=0.3)
    fig.suptitle(f"Log {log.path.stem} [{yaw_angle_source(log)}]")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    logs = [read_log(path) for path in args.logs]
    summarized = [summarize(log) for log in logs]
    summaries = [item[0] for item in summarized]
    args.output_dir.mkdir(parents=True, exist_ok=True)
    stem = f"log_{logs[0].path.stem}_{logs[-1].path.stem}"
    write_summary(args.output_dir / f"{stem}_imu_temp_summary.csv", summaries)
    coefficient_rows = fit_temperature_coefficient(logs, compensation_enabled=0)
    write_summary(args.output_dir / f"{stem}_temperature_coefficient_off.csv", coefficient_rows)
    compensation_rows = fit_temperature_coefficient(logs, compensation_enabled=1)
    write_summary(args.output_dir / f"{stem}_temperature_coefficient_on.csv", compensation_rows)
    pairs = match_compensation_runs(logs, summaries)
    if pairs:
        write_summary(args.output_dir / f"{stem}_compensation_pairs.csv", pairs)
        plot_compensation_comparison(
            args.output_dir / f"{stem}_compensation_yaw.png", pairs
        )
    valid_primary = [
        log
        for log in logs
        if integer(log.parameters.get("optimalTrace"), -1) == 0
        and integer(log.parameters.get("emcStop"), -1) == 0
    ]
    plot_temperature(args.output_dir / f"{stem}_temperature.png", valid_primary)
    plot_yaw_errors(args.output_dir / f"{stem}_yaw_error.png", summaries)
    plot_xy(args.output_dir / f"{stem}_xy.png", valid_primary)
    plot_latest_overview(args.output_dir / f"log_{logs[-1].path.stem}_overview.png", logs[-1])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
