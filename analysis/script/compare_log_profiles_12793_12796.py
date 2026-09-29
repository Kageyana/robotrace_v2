#!/usr/bin/env python3
"""同一コースの通常56 Bと詳細109 Bログを比較する。"""

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
LOG_NUMBERS = (12793, 12794, 12795, 12796)


def load_log(number: int) -> tuple[dict[str, str], list[dict[str, float]], int, int]:
    with (LOG_DIR / f"{number}.csv").open(encoding="utf-8-sig", newline="") as source:
        reader = csv.reader(source)
        metadata = dict(item.split("=", 1) for item in next(reader) if "=" in item)
        header = next(reader)
        rows = []
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


def rms(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values))


def main() -> None:
    logs = {number: load_log(number) for number in LOG_NUMBERS}
    summary = []
    for number, (meta, rows, bad_width, nonfinite) in logs.items():
        if not rows:
            raise ValueError(f"{number}: no data rows")
        pulses_per_meter = float(meta["encoderPulsePerMeter"])
        gaps = [b["cntlog"] - a["cntlog"] for a, b in zip(rows, rows[1:])]
        distance_steps = [
            b["encTotalOptimal"] - a["encTotalOptimal"]
            for a, b in zip(rows, rows[1:]) if b["courseMarker"] == 0
        ]
        straights = [row for row in rows if row["ROC"] >= 1500]
        speed_errors = [row["encCurrentN"] - row["targetSpeed"] for row in rows]
        summary.append({
            "log": number,
            "profile_bytes": 109 if len(rows[0]) == 46 else 56,
            "mode": meta.get("requestedMode", ""),
            "optimal_trace": meta.get("optimalTrace", ""),
            "rows": len(rows),
            "expected_rows": meta.get("logExpectedRows", ""),
            "bad_width_rows": bad_width,
            "nonfinite_rows": nonfinite,
            "last_cntlog_ms": int(rows[-1]["cntlog"]),
            "cntlog_nonpositive_gaps": sum(gap <= 0 for gap in gaps),
            "cntlog_max_gap_ms": int(max(gaps)),
            "nonmarker_distance_step_min_p": int(min(distance_steps)),
            "nonmarker_distance_step_max_p": int(max(distance_steps)),
            "nonmarker_distance_negative_steps": sum(step < 0 for step in distance_steps),
            "record_rate_per_sec": round(len(rows) * 1000 / (rows[-1]["cntlog"] - rows[0]["cntlog"]), 1),
            "speed_peak_mps": round(max(row["encCurrentN"] for row in rows) * 1000 / pulses_per_meter, 2),
            "target_peak_mps": round(max(row["targetSpeed"] for row in rows) * 1000 / pulses_per_meter, 2),
            "speed_mae_mps": round(sum(abs(value) for value in speed_errors) / len(rows) * 1000 / pulses_per_meter, 3),
            "straight_gyro_rms_dps": round(rms([row["gyroVal_Z"] for row in straights]), 1),
            "straight_line_ctrl_rms": round(rms([row["lineTraceCtrl"] for row in straights]), 1),
            "pwm_difference_rms": round(rms([row["motorpwmL"] - row["motorpwmR"] for row in rows]), 1),
            "pwm_peak_abs": int(max(abs(row[name]) for row in rows for name in ("motorpwmL", "motorpwmR"))),
            "slip_rows": sum(row["slipFlag"] != 0 for row in rows),
            "lateral_slip_rows": sum(row["slipFlagLat"] != 0 for row in rows),
            "battery_start_v": meta.get("batteryVoltage_V", ""),
            "battery_min_v": round(min(row["batteryVoltage_mV"] for row in rows) / 1000, 3),
            "line_omega_kp": meta.get("lineTraceOmegaFBCtrl.kp", ""),
            "line_omega_kd": meta.get("lineTraceOmegaFBCtrl.kd", ""),
            "emc_stop": meta.get("emcStop", ""),
        })

    with (OUT_DIR / "log_12793_12796_profile_comparison.csv").open("w", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=list(summary[0]))
        writer.writeheader()
        writer.writerows(summary)

    fig, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    colors = {12793: "#4477AA", 12794: "#66CCEE", 12795: "#CC3311"}
    for number in (12793, 12794, 12795):
        meta, rows, _, _ = logs[number]
        label = f"{number} ({'109B' if number == 12795 else '56B'})"
        color = colors[number]
        dist = [row["encTotalOptimal"] / float(meta["encoderPulsePerMeter"]) for row in rows]
        axes[0, 0].plot([row["x"] for row in rows], [row["y"] for row in rows], label=label, color=color, lw=1)
        axes[0, 1].plot(dist, [row["encCurrentN"] * 1000 / float(meta["encoderPulsePerMeter"]) for row in rows], label=label, color=color, lw=0.9)
        axes[1, 0].plot(dist, [row["gyroVal_Z"] for row in rows], label=label, color=color, lw=0.9)
        axes[1, 1].plot(dist, [row["lineTraceCtrl"] for row in rows], label=label, color=color, lw=0.9)
    axes[0, 0].set(title="Primary XY", xlabel="x [mm]", ylabel="y [mm]")
    axes[0, 0].axis("equal")
    axes[0, 1].set(title="Encoder speed", xlabel="logged distance [m]", ylabel="m/s")
    axes[1, 0].set(title="Gyro Z", xlabel="logged distance [m]", ylabel="deg/s")
    axes[1, 1].set(title="Line control output", xlabel="logged distance [m]", ylabel="PWM command")
    for axis in axes.flat:
        axis.legend(loc="best")
    fig.savefig(OUT_DIR / "log_12793_12795_primary_profile_overlay.png", dpi=140)
    plt.close(fig)

    meta, rows, _, _ = logs[12796]
    time = [row["cntlog"] / 1000 for row in rows]
    scale = 1000 / float(meta["encoderPulsePerMeter"])
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True, constrained_layout=True)
    axes[0].plot(time, [row["encCurrentN"] * scale for row in rows], label="actual")
    axes[0].plot(time, [row["targetSpeed"] * scale for row in rows], label="target")
    axes[0].set(ylabel="speed [m/s]", title="12796 detailed DISTANCE")
    axes[0].legend()
    axes[1].plot(time, [row["gyroVal_Z"] for row in rows], lw=0.8)
    axes[1].set(ylabel="gyro Z [deg/s]")
    axes[2].step(time, [row["slipFlag"] for row in rows], label="longitudinal", where="post")
    axes[2].step(time, [row["slipFlagLat"] for row in rows], label="lateral", where="post")
    axes[2].set(xlabel="time [s]", ylabel="slip flag")
    axes[2].legend()
    fig.savefig(OUT_DIR / "log_12796_detailed_distance.png", dpi=140)
    plt.close(fig)
    print(OUT_DIR / "log_12793_12796_profile_comparison.csv")


if __name__ == "__main__":
    main()
