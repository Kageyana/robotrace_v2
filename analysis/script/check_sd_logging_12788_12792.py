#!/usr/bin/env python3
"""実走行12788～12792のログ整合性と走行状態を確認する。"""

from __future__ import annotations

import csv
import math
import os
import tempfile
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "robotrace-matplotlib"))
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


LOG_DIR = Path(r"F:\Dropbox\Document\robotrace\Log\v2")
OUT_DIR = Path(__file__).resolve().parents[1]
LOG_NUMBERS = range(12788, 12793)
REQUIRED_COLUMNS = (
    "cntlog", "encCurrentN", "gyroVal_Z", "courseMarker", "encTotalOptimal",
    "targetSpeed", "slipFlag", "slipFlagLat", "motorpwmL", "motorpwmR",
    "batteryVoltage_mV", "x", "y",
)


def load_log(number: int) -> tuple[dict[str, str], list[dict[str, float]], int, int]:
    with (LOG_DIR / f"{number}.csv").open(encoding="utf-8-sig", newline="") as source:
        reader = csv.reader(source)
        metadata = dict(item.split("=", 1) for item in next(reader) if "=" in item)
        header = next(reader)
        names = [name for name in header if name]
        missing = sorted(set(REQUIRED_COLUMNS) - set(names))
        if missing:
            raise ValueError(f"{number}: missing columns: {', '.join(missing)}")
        rows: list[dict[str, float]] = []
        bad_width = 0
        nonfinite = 0
        for raw in reader:
            if not raw or all(not item for item in raw):
                continue
            if len(raw) != len(header):
                bad_width += 1
                continue
            row = {name: float(raw[index]) for index, name in enumerate(header) if name}
            nonfinite += any(not math.isfinite(value) for value in row.values())
            rows.append(row)
    return metadata, rows, bad_width, nonfinite


def main() -> None:
    summaries: list[dict[str, object]] = []
    logs: list[tuple[int, str, list[dict[str, float]]]] = []
    for number in LOG_NUMBERS:
        metadata, rows, bad_width, nonfinite = load_log(number)
        if not rows:
            raise ValueError(f"{number}: no data rows")
        mode = metadata.get("requestedMode", "UNKNOWN")
        pulse_per_meter = float(metadata["encoderPulsePerMeter"])
        cnt = [row["cntlog"] for row in rows]
        gaps = [right - left for left, right in zip(cnt, cnt[1:])]
        distance_steps = [
            right["encTotalOptimal"] - left["encTotalOptimal"]
            for left, right in zip(rows, rows[1:])
            if right["courseMarker"] == 0
        ]
        marker_corrections = sum(
            right["encTotalOptimal"] < left["encTotalOptimal"]
            and right["courseMarker"] != 0
            for left, right in zip(rows, rows[1:])
        )
        speeds = [row["encCurrentN"] * 1000 / pulse_per_meter for row in rows]
        targets = [row["targetSpeed"] * 1000 / pulse_per_meter for row in rows]
        summaries.append({
            "log": number,
            "mode": mode,
            "rows": len(rows),
            "expected_rows": metadata.get("logExpectedRows", ""),
            "bad_width_rows": bad_width,
            "nonfinite_rows": nonfinite,
            "cntlog_first_ms": int(cnt[0]),
            "cntlog_last_ms": int(cnt[-1]),
            "cntlog_nonpositive_gaps": sum(gap <= 0 for gap in gaps),
            "cntlog_max_gap_ms": int(max(gaps)),
            "distance_nonmarker_min_step_p": int(min(distance_steps)),
            "distance_nonmarker_max_step_p": int(max(distance_steps)),
            "distance_nonmarker_negative_steps": sum(step < 0 for step in distance_steps),
            "distance_marker_corrections": marker_corrections,
            "record_rate_per_sec": round(len(rows) * 1000 / (cnt[-1] - cnt[0]), 1),
            "speed_peak_mps": round(max(speeds), 2),
            "target_peak_mps": round(max(targets), 2),
            "speed_mae_mps": round(sum(abs(a - b) for a, b in zip(speeds, targets)) / len(rows), 3),
            "gyro_peak_abs_dps": round(max(abs(row["gyroVal_Z"]) for row in rows), 1),
            "slip_rows": sum(row["slipFlag"] != 0 for row in rows),
            "lateral_slip_rows": sum(row["slipFlagLat"] != 0 for row in rows),
            "battery_min_v": round(min(row["batteryVoltage_mV"] for row in rows) / 1000, 3),
            "pwm_peak_abs": int(max(abs(row[name]) for row in rows for name in ("motorpwmL", "motorpwmR"))),
            "emc_stop": metadata.get("emcStop", ""),
            "shortcut_level": metadata.get("shortcutLevel", ""),
            "path_error_peak_abs_mm": round(max(abs(row.get("pathErrorY_mm", 0)) for row in rows), 2),
        })
        logs.append((number, mode, rows))

    summary_path = OUT_DIR / "log_12788_12792_sd_integrity.csv"
    with summary_path.open("w", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)

    fig, axes = plt.subplots(len(logs), 2, figsize=(12, 15), constrained_layout=True)
    for index, (number, mode, rows) in enumerate(logs):
        xy, speed = axes[index]
        xy.plot([row["x"] for row in rows], [row["y"] for row in rows], lw=1)
        xy.set(title=f"{number} {mode} XY", xlabel="x [mm]", ylabel="y [mm]")
        xy.axis("equal")
        # 速度換算は各ログのメタデータ値を使用する。
        with (LOG_DIR / f"{number}.csv").open(encoding="utf-8-sig", newline="") as source:
            meta = dict(item.split("=", 1) for item in next(csv.reader(source)) if "=" in item)
        pulse_per_meter = float(meta["encoderPulsePerMeter"])
        time_s = [row["cntlog"] / 1000 for row in rows]
        speed.plot(time_s, [row["encCurrentN"] * 1000 / pulse_per_meter for row in rows], label="actual", lw=1)
        speed.plot(time_s, [row["targetSpeed"] * 1000 / pulse_per_meter for row in rows], label="target", lw=1)
        speed.set(title=f"{number} speed", xlabel="time [s]", ylabel="speed [m/s]")
        speed.legend(loc="upper right")
    fig.savefig(OUT_DIR / "log_12788_12792_xy_speed.png", dpi=140)
    plt.close(fig)

    fig, axes = plt.subplots(len(logs), 1, figsize=(11, 12), constrained_layout=True)
    for axis, (number, mode, rows) in zip(axes, logs):
        axis.plot([row["cntlog"] / 1000 for row in rows], [row["gyroVal_Z"] for row in rows], lw=1)
        axis.set(title=f"{number} {mode} gyro Z", xlabel="time [s]", ylabel="deg/s")
    fig.savefig(OUT_DIR / "log_12788_12792_gyro.png", dpi=140)
    plt.close(fig)
    print(summary_path)


if __name__ == "__main__":
    main()
