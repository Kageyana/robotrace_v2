#!/usr/bin/env python3
"""Compare the start of primary logs 12458-12466 with completed log 12368."""

from __future__ import annotations

import argparse
import csv
import os
import statistics
import tempfile
from pathlib import Path

# Windowsのユーザープロファイルが読み取り専用の実行環境でも図を保存できるようにする。
os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "robotrace_mpl_cache"))
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


PULSES_PER_MM = 53.424
BASELINE_IDS = (12368,)
NEW_IDS = tuple(range(12458, 12467))
FIELDS = ("targetAngularvelo", "gyroVal_Z", "encCurrentN", "targetSpeed", "ROC", "x", "y")


def load_log(path: Path) -> tuple[dict[str, str], list[dict[str, float]]]:
    with path.open(encoding="utf-8-sig", newline="") as stream:
        metadata = dict(field.split("=", 1) for field in stream.readline().strip().split(",") if "=" in field)
        rows = [
            {key: float(value) for key, value in row.items() if key and value}
            for row in csv.DictReader(stream)
            if row.get("cntlog")
        ]
    if not rows:
        raise ValueError(f"No log records: {path}")
    return metadata, rows


def distance(row: dict[str, float]) -> float:
    return row["encTotalOptimal"] / PULSES_PER_MM


def at_distance(rows: list[dict[str, float]], mm: float) -> dict[str, float] | None:
    for previous, current in zip(rows, rows[1:]):
        a, b = distance(previous), distance(current)
        if a <= mm <= b and a != b:
            ratio = (mm - a) / (b - a)
            return {key: previous[key] + ratio * (current[key] - previous[key]) for key in FIELDS}
    return None


def fmt(value: float | None, digits: int = 1) -> str:
    return "" if value is None else f"{value:.{digits}f}"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    summaries = []
    aligned = []
    for run_id in BASELINE_IDS + NEW_IDS:
        meta, rows = load_log(args.log_dir / f"{run_id}.csv")
        gaps = [int(right["cntlog"] - left["cntlog"]) for left, right in zip(rows, rows[1:])]
        marker = next((row for row in rows if row["courseMarker"] == 2 and distance(row) > 100), None)
        sampled = {mm: at_distance(rows, mm) for mm in range(400, 1401, 25)}
        active = [row for row in rows if 400 <= distance(row) <= 800]
        fusion_window = [row for row in rows if 400 <= distance(row) <= 1400]
        fusion_deltas = [row["encCurrentCorr_p"] - row["encCurrentN"] for row in fusion_window]
        summaries.append({
            "log": run_id,
            "buildTime": meta.get("buildTime", ""),
            "schema": meta.get("logSchemaVersion", ""),
            "records": len(rows),
            "expectedRecords": meta.get("logExpectedRows", ""),
            "cntlogMaxGap_ms": max(gaps, default=0),
            "distanceEnd_mm": fmt(distance(rows[-1]), 0),
            "emcStop": meta.get("emcStop", ""),
            "closureValid": meta.get("closureValid", ""),
            "closureReason": meta.get("closureReason", ""),
            "goalMarkerS_mm": meta.get("goalMarkerS_mm", ""),
            "goalMarkerXRaw_mm": meta.get("goalMarkerXRaw_mm", ""),
            "firstLeftLog_mm": fmt(distance(marker) if marker else None, 0),
            "omegaKpStart": meta.get("runStartOmega.kp", meta.get("lineTraceOmegaFBCtrl.kp", "")),
            "battery_V": meta.get("batteryVoltage_V", ""),
            "calibrationHash": meta.get("runStartLineCalibrationFNV1a32", ""),
            "isrMax_us": meta.get("timing.isrMax_us", ""),
            "isrOverruns": meta.get("timing.isrOverrunCount", ""),
            "imuMax_us": meta.get("timing.imuReadMax_us", ""),
            "imuErrors": meta.get("timing.imuReadErrorCount", ""),
            "lineMaxInterval_ms": meta.get("timing.lineUpdateMaxInterval_ms", ""),
            "lineStaleCycles": meta.get("timing.lineStaleCycleCount", ""),
            "phaseMismatch": meta.get("timing.adcPhaseMismatchCount", ""),
            "startResetDelay_ms": meta.get("timing.startResetDelay_ms", ""),
            "startResetPulseDelta_p": meta.get("timing.startResetPulseDelta_p", ""),
            "logOverflow": meta.get("logOverflowFinal", ""),
            "kalmanReject": meta.get("distanceKalman.innovationRejectCount", ""),
            "sampledMeanFusionMinusRaw_p_per_ms": fmt(statistics.mean(fusion_deltas) if fusion_deltas else None, 2),
            "sampledMaxAbsFusionMinusRaw_p_per_ms": fmt(max(map(abs, fusion_deltas)) if fusion_deltas else None, 0),
            "peakAbsTarget400to800_dps": fmt(max((abs(row["targetAngularvelo"]) for row in active), default=0)),
            "targetAt500_dps": fmt(sampled[500]["targetAngularvelo"] if sampled[500] else None),
            "gyroAt500_dps": fmt(sampled[500]["gyroVal_Z"] if sampled[500] else None),
            "targetAt700_dps": fmt(sampled[700]["targetAngularvelo"] if sampled[700] else None),
            "gyroAt700_dps": fmt(sampled[700]["gyroVal_Z"] if sampled[700] else None),
            "xAt700_mm": fmt(sampled[700]["x"] if sampled[700] else None),
            "yAt1000_mm": fmt(sampled[1000]["y"] if sampled[1000] else None),
            "targetAt1300_dps": fmt(sampled[1300]["targetAngularvelo"] if sampled[1300] else None),
            "gyroAt1300_dps": fmt(sampled[1300]["gyroVal_Z"] if sampled[1300] else None),
            "rocAt1400_mm": fmt(sampled[1400]["ROC"] if sampled[1400] else None),
            "xAt1400_mm": fmt(sampled[1400]["x"] if sampled[1400] else None),
        })
        for mm, values in sampled.items():
            if values is not None:
                aligned.append({"log": run_id, "distance_mm": mm, **{key: fmt(values[key]) for key in FIELDS}})

    for name, data in (("summary", summaries), ("aligned", aligned)):
        path = args.output_dir / f"log_12458_12466_{name}.csv"
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(data[0]))
            writer.writeheader()
            writer.writerows(data)
        print(path)

    figure, (yaw_axis, xy_axis) = plt.subplots(1, 2, figsize=(13, 5.5))
    for ids, color, label in ((BASELINE_IDS, "#227a4d", "Completed 12368"),
                              (NEW_IDS, "#d65a36", "12458-12466")):
        for run_id in ids:
            points = [row for row in aligned if row["log"] == run_id and 400 <= row["distance_mm"] <= 900]
            yaw_axis.plot([row["distance_mm"] for row in points],
                          [float(row["targetAngularvelo"]) for row in points],
                          color=color, alpha=0.27, linewidth=1)
            xy_axis.plot([float(row["x"]) for row in points],
                         [float(row["y"]) for row in points],
                         color=color, alpha=0.3, linewidth=1)
        xs = list(range(400, 901, 25))
        target_median = [statistics.median(float(row["targetAngularvelo"]) for row in aligned
                                           if row["log"] in ids and row["distance_mm"] == mm) for mm in xs]
        actual_median = [statistics.median(float(row["gyroVal_Z"]) for row in aligned
                                           if row["log"] in ids and row["distance_mm"] == mm) for mm in xs]
        yaw_axis.plot(xs, target_median, color=color, linewidth=2.6, label=f"{label} target")
        yaw_axis.plot(xs, actual_median, color=color, linewidth=1.6, linestyle="--", label=f"{label} gyro")
        marker_distances = [float(row["firstLeftLog_mm"]) for row in summaries if row["log"] in ids]
        yaw_axis.axvspan(min(marker_distances), max(marker_distances), color=color, alpha=0.10)
    yaw_axis.set(xlabel="Travel from start marker [mm]", ylabel="Yaw rate [deg/s]",
                 title="First curve: target (solid), gyro (dashed)", xlim=(400, 900))
    yaw_axis.grid(alpha=0.25)
    yaw_axis.legend(fontsize=8)
    xy_axis.set(xlabel="Dead-reckoned X [mm]", ylabel="Dead-reckoned Y [mm]",
                title="Trajectory at 400-900 mm")
    xy_axis.set_aspect("equal", adjustable="box")
    xy_axis.grid(alpha=0.25)
    figure.tight_layout()
    plot_path = args.output_dir / "log_12458_12466_first_curve.png"
    figure.savefig(plot_path, dpi=160)
    plt.close(figure)
    print(plot_path)

    figure, (yaw_axis, xy_axis) = plt.subplots(1, 2, figsize=(13, 5.5))
    for ids, color, label in ((BASELINE_IDS, "#227a4d", "Completed 12368"),
                              (NEW_IDS, "#d65a36", "12458-12466")):
        for run_id in ids:
            points = [row for row in aligned if row["log"] == run_id and 1000 <= row["distance_mm"] <= 1400]
            yaw_axis.plot([row["distance_mm"] for row in points],
                          [float(row["targetAngularvelo"]) for row in points],
                          color=color, alpha=0.27, linewidth=1)
            xy_axis.plot([float(row["x"]) for row in points],
                         [float(row["y"]) for row in points],
                         color=color, alpha=0.3, linewidth=1)
        xs = list(range(1000, 1401, 25))
        for field, style, suffix in (("targetAngularvelo", "-", "target"), ("gyroVal_Z", "--", "gyro")):
            medians = [statistics.median(float(row[field]) for row in aligned
                                        if row["log"] in ids and row["distance_mm"] == mm) for mm in xs]
            yaw_axis.plot(xs, medians, color=color, linewidth=2.3 if style == "-" else 1.6,
                          linestyle=style, label=f"{label} {suffix}")
    yaw_axis.set(xlabel="Travel from start marker [mm]", ylabel="Yaw rate [deg/s]",
                 title="Second curve: target (solid), gyro (dashed)", xlim=(1000, 1400))
    yaw_axis.grid(alpha=0.25)
    yaw_axis.legend(fontsize=8)
    xy_axis.set(xlabel="Dead-reckoned X [mm]", ylabel="Dead-reckoned Y [mm]",
                title="Trajectory at 1000-1400 mm")
    xy_axis.set_aspect("equal", adjustable="box")
    xy_axis.grid(alpha=0.25)
    figure.tight_layout()
    plot_path = args.output_dir / "log_12458_12466_second_curve.png"
    figure.savefig(plot_path, dpi=160)
    plt.close(figure)
    print(plot_path)

    for mm in (400, 450, 500, 550, 600, 650, 700, 750, 800, 1100, 1200, 1300, 1350, 1400):
        group_values = []
        for group in (BASELINE_IDS, NEW_IDS):
            values = [float(row["targetAngularvelo"]) for row in aligned if row["distance_mm"] == mm and row["log"] in group]
            group_values.append(f"{statistics.median(values):.0f} [{min(values):.0f},{max(values):.0f}]")
        print(f"{mm}mm target yaw baseline/new median [min,max] deg/s: {' / '.join(group_values)}")


if __name__ == "__main__":
    main()
