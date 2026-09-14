#!/usr/bin/env python3
"""最新ログ12439～12443の健全性、速度、ジャイロ、一次走行XYを集計する。"""

from __future__ import annotations

import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt

from robotrace_units import PULSE_MILLIMETER


LOG_NUMBERS = range(12439, 12444)


def percentile(values: list[float], ratio: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return math.nan
    return ordered[round((len(ordered) - 1) * ratio)]


def decode_temperature(raw: float) -> float | None:
    code = int(raw) & 0x7FF
    if code == 0x400:
        return None
    signed = code - 2048 if code & 0x400 else code
    return 23.0 + signed * 0.125


def read_log(path: Path) -> tuple[dict[str, str], list[dict[str, float]], list[int]]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        metadata = {}
        for field in next(csv.reader([stream.readline()])):
            if "=" in field:
                key, value = field.split("=", 1)
                metadata[key] = value
        rows = [{key: float(value) for key, value in row.items()
                 if key and value not in (None, "")}
                for row in csv.DictReader(stream) if row.get("cntlog", "") != ""]
    times: list[int] = []
    wrap = 0
    previous: int | None = None
    for row in rows:
        raw = int(row["cntlog"])
        if previous is not None and raw < previous:
            wrap += 65536
        times.append(raw + wrap)
        previous = raw
    return metadata, rows, times


def slope_from_y(rows: list[dict[str, float]], distances: list[float],
                 begin_mm: float, end_mm: float) -> float:
    begin = min(range(len(distances)), key=lambda index: abs(distances[index] - begin_mm))
    end = min(range(len(distances)), key=lambda index: abs(distances[index] - end_mm))
    dx = rows[end]["x"] - rows[begin]["x"]
    dy = rows[end]["y"] - rows[begin]["y"]
    return (math.degrees(math.atan2(dx, dy)) + 90.0) % 180.0 - 90.0


def summarize(path: Path) -> tuple[dict[str, object], list[dict[str, float]], list[int], list[float]]:
    metadata, rows, times = read_log(path)
    intervals = [times[index] - times[index - 1] for index in range(1, len(times))]
    distances = [0.0]
    heading = 0.0
    for index in range(1, len(rows)):
        dt_ms = intervals[index - 1]
        distances.append(distances[-1] + rows[index]["encCurrentCorr_p"] * dt_ms / PULSE_MILLIMETER)
        heading += rows[index]["gyroVal_Z"] * dt_ms / 1000.0
    speed_errors = [abs((row["encCurrentCorr_p"] - row["targetSpeed"]) / PULSE_MILLIMETER)
                    for row in rows]
    straight_gyro = [row["gyroVal_Z"] for row in rows if abs(row["ROC"]) >= 700.0]
    temperatures = [decode_temperature(row["imuTempRaw"]) for row in rows]
    temperatures = [value for value in temperatures if value is not None]
    primary = int(float(metadata.get("optimalTrace", "-1"))) == 0
    summary: dict[str, object] = {
        "log": path.stem,
        "mode": int(float(metadata.get("optimalTrace", "-1"))),
        "auto_start": int(float(metadata.get("autoStart", "-1"))),
        "emc_stop": int(float(metadata.get("emcStop", "-1"))),
        "battery_v": float(metadata.get("batteryVoltage_V", "nan")),
        "lap_ms": times[-1],
        "samples": len(rows),
        "cntlog_valid": all(value > 0 for value in intervals),
        "max_dt_ms": max(intervals),
        "distance_mm": distances[-1],
        "final_x_mm": rows[-1]["x"],
        "final_y_mm": rows[-1]["y"],
        "gyro_net_deg": heading,
        "straight_gyro_rms_dps": math.sqrt(sum(value * value for value in straight_gyro) / len(straight_gyro)),
        "speed_error_p95_mps": percentile(speed_errors, 0.95),
        "line_valid_ratio": sum(row["lineValid"] != 0 for row in rows) / len(rows),
        "imu_temp_begin_c": temperatures[0],
        "imu_temp_end_c": temperatures[-1],
        "kalman_reject": int(metadata.get("distanceKalman.innovationRejectCount", "-1")),
        "kalman_fallback": int(metadata.get("distanceKalman.fallbackCount", "-1")),
        "kalman_invalid": int(metadata.get("distanceKalman.invalidUpdateCount", "-1")),
        "kalman_guard": int(metadata.get("distanceKalman.outputGuardCount", "-1")),
        "outer_straight_slope_deg": slope_from_y(rows, distances, 5000.0, 8000.0) if primary else math.nan,
        "inner_straight_slope_deg": slope_from_y(rows, distances, 10000.0, 12500.0) if primary else math.nan,
    }
    return summary, rows, times, distances


def main() -> int:
    log_dir = Path(r"C:\Users\ucawa\Dropbox\Document\robotrace\Log\v2")
    output_dir = Path("analysis/logs_12439_12443")
    output_dir.mkdir(parents=True, exist_ok=True)
    results = [summarize(log_dir / f"{number}.csv") for number in LOG_NUMBERS]
    summaries = [result[0] for result in results]
    summary_path = output_dir / "logs_12439_12443_summary.csv"
    with summary_path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)

    plt.rcParams.update({"font.family": "Yu Gothic", "axes.unicode_minus": False})
    figure, axes = plt.subplots(1, 2, figsize=(15, 7), constrained_layout=True)
    for summary, rows, times, _ in results:
        label = f"{summary['log']} mode={summary['mode']} emc={summary['emc_stop']}"
        axes[0].plot([row["x"] for row in rows], [row["y"] for row in rows], label=label)
        axes[1].plot([value / 1000.0 for value in times],
                     [row["encCurrentCorr_p"] / PULSE_MILLIMETER for row in rows], label=label)
    axes[0].set(xlabel="X [mm]", ylabel="Y [mm]", title="XY軌跡")
    axes[0].set_aspect("equal", adjustable="box")
    axes[1].set(xlabel="走行時間 [s]", ylabel="融合速度 [m/s]", title="速度")
    for axis in axes:
        axis.grid(True, alpha=0.3)
    axes[1].legend(fontsize=8)
    figure.savefig(output_dir / "logs_12439_12443_xy_speed.png", dpi=180)
    plt.close(figure)
    print(summary_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
