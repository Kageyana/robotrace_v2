#!/usr/bin/env python3
"""Compare primary-run failures with the recovered primary runs on the same build."""

from __future__ import annotations

import argparse
import csv
import os
import statistics
import tempfile
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "robotrace_mpl_cache"))
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from analyze_primary_runs_12458_12466 import at_distance, distance, load_log


REFERENCE = (12368,)
FAILED = tuple(range(12459, 12467))
RECOVERED = (12467, 12468, 12473, 12478)
DISTANCES = (500, 1000, 1100, 1200, 1300, 1350, 1400)


def time_at_pulse(rows: list[dict[str, float]], pulse: float) -> float | None:
    for before, after in zip(rows, rows[1:]):
        low, high = before["encTotalOptimal"], after["encTotalOptimal"]
        if low <= pulse <= high and high != low:
            return before["cntlog"] + ((pulse - low) / (high - low)) * (after["cntlog"] - before["cntlog"])
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    logs = {}
    summaries = []
    for run_id in REFERENCE + FAILED + RECOVERED:
        meta, rows = load_log(args.log_dir / f"{run_id}.csv")
        logs[run_id] = rows
        gaps = [right["cntlog"] - left["cntlog"] for left, right in zip(rows, rows[1:])]
        first_left = next((distance(row) for row in rows if row["courseMarker"] == 2 and distance(row) > 100), None)
        marker_pulse = float(meta.get("goalMarkerOnset_p", "0"))
        marker_time = time_at_pulse(rows, marker_pulse) if marker_pulse > 0 else None
        curve_rows = [row for row in rows if 400 <= distance(row) <= 1400]
        item = {
            "log": run_id,
            "group": "reference" if run_id in REFERENCE else "failed" if run_id in FAILED else "recovered",
            "buildTime": meta.get("buildTime", ""),
            "battery_V": meta.get("batteryVoltage_V", ""),
            "calibrationHash": meta.get("runStartLineCalibrationFNV1a32", ""),
            "startOmegaKp": meta.get("runStartOmega.kp", ""),
            "emcStop": meta.get("emcStop", ""),
            "closureValid": meta.get("closureValid", ""),
            "goalMarkerXRaw_mm": meta.get("goalMarkerXRaw_mm", ""),
            "goalMarkerS_mm": meta.get("goalMarkerS_mm", ""),
            "goalMarkerTime_ms": "" if marker_time is None else round(marker_time, 1),
            "rowCount": len(rows),
            "expectedRows": meta.get("logExpectedRows", ""),
            "cntlogMaxGap_ms": round(max(gaps, default=0)),
            "endTime_ms": round(rows[-1]["cntlog"]),
            "endDistance_mm": round(distance(rows[-1])),
            "firstLeftLog_mm": "" if first_left is None else round(first_left),
            "meanAbsSpeedError400to1400_p_per_ms": round(statistics.mean(
                abs(row["targetSpeed"] - row["encCurrentN"]) for row in curve_rows), 2),
        }
        for mm in DISTANCES:
            row = at_distance(rows, mm)
            for field, column in (("targetAngularvelo", "targetYaw_dps"),
                                  ("gyroVal_Z", "gyro_dps"),
                                  ("encCurrentN", "speed_p_per_ms"),
                                  ("targetSpeed", "targetSpeed_p_per_ms"),
                                  ("ROC", "ROC_mm"), ("x", "x_mm"), ("y", "y_mm")):
                item[f"{column}_at{mm}"] = "" if row is None else round(row[field], 1)
        summaries.append(item)
    output = args.output_dir / "log_12467_12482_primary_comparison.csv"
    with output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)
    print(output)

    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))
    groups = ((REFERENCE, "#4c68a1", "12368 reference"),
              (FAILED, "#d65a36", "12459-66 failed"),
              (RECOVERED, "#227a4d", "12467/68/73/78 recovered"))
    for ax, lower, upper, title in ((axes[0], 400, 700, "First curve"),
                                     (axes[1], 1050, 1400, "Second curve")):
        grid = list(range(lower, upper + 1, 25))
        for ids, color, label in groups:
            for run_id in ids:
                points = [(distance(row), row["targetAngularvelo"], row["gyroVal_Z"])
                          for row in logs[run_id] if lower <= distance(row) <= upper]
                ax.plot([point[0] for point in points], [point[1] for point in points],
                        color=color, alpha=0.15, linewidth=1)
            for field, style, suffix in (("targetAngularvelo", "-", "target"),
                                         ("gyroVal_Z", "--", "gyro")):
                medians = [statistics.median(at_distance(logs[run_id], mm)[field] for run_id in ids)
                           for mm in grid]
                ax.plot(grid, medians, color=color, linestyle=style,
                        linewidth=2.3 if style == "-" else 1.5, label=f"{label} {suffix}")
        ax.set(xlabel="Travel from start marker [mm]", ylabel="Yaw rate [deg/s]", title=title)
        ax.grid(alpha=0.25)
    axes[1].legend(fontsize=7)
    fig.tight_layout()
    plot = args.output_dir / "log_12467_12482_primary_yaw.png"
    fig.savefig(plot, dpi=160)
    plt.close(fig)
    print(plot)

    fig, (speed_axis, xy_axis) = plt.subplots(1, 2, figsize=(13, 5.5))
    for ids, color, label in groups:
        for run_id in ids:
            rows = logs[run_id]
            speed_points = [row for row in rows if 400 <= distance(row) <= 1400]
            speed_axis.plot([distance(row) for row in speed_points],
                            [row["encCurrentN"] for row in speed_points],
                            color=color, alpha=0.2, linewidth=1)
            xy_axis.plot([row["x"] for row in rows], [row["y"] for row in rows],
                         color=color, alpha=0.25, linewidth=1)
        grid = list(range(400, 1401, 25))
        median_speed = [statistics.median(at_distance(logs[run_id], mm)["encCurrentN"] for run_id in ids)
                        for mm in grid]
        speed_axis.plot(grid, median_speed, color=color, linewidth=2.3, label=label)
    speed_axis.axhline(53, color="black", alpha=0.5, linestyle=":", label="New target ~53 pulse/ms")
    speed_axis.set(xlabel="Travel from start marker [mm]", ylabel="Raw encoder speed [pulse/ms]",
                   title="Speed tracking at first two curves")
    speed_axis.grid(alpha=0.25)
    speed_axis.legend(fontsize=7)
    xy_axis.set(xlabel="Dead-reckoned X [mm]", ylabel="Dead-reckoned Y [mm]",
                title="Primary-run raw XY (different logging algorithms)")
    xy_axis.set_aspect("equal", adjustable="box")
    xy_axis.grid(alpha=0.25)
    fig.tight_layout()
    speed_xy_plot = args.output_dir / "log_12467_12482_primary_speed_xy.png"
    fig.savefig(speed_xy_plot, dpi=160)
    plt.close(fig)
    print(speed_xy_plot)


if __name__ == "__main__":
    main()
