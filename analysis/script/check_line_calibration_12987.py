"""12987と同方式の完走ログを比較し、提供されたlsvalの範囲を検証する。"""
import csv
import json
from pathlib import Path
import numpy as np
import os
OUT = Path("analysis/line_calibration_12987")
OUT.mkdir(parents=True, exist_ok=True)
os.environ["MPLCONFIGDIR"] = str((OUT / "mplconfig").resolve())
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path("F:/Dropbox/Document/robotrace/Log/v2")
OUT = Path("analysis/line_calibration_12987")
OUT.mkdir(parents=True, exist_ok=True)
NUMBERS = [12971, 12976, 12983, 12984, 12985, 12986, 12987]

def load(number):
    with (ROOT / f"{number}.csv").open(encoding="utf-8-sig", newline="") as f:
        metadata = dict(v.split("=", 1) for v in next(csv.reader(f)) if "=" in v)
        rows = list(csv.DictReader(f))
    data = {k: np.array([float(r[k]) for r in rows]) for k in rows[0] if k}
    return metadata, data

runs = {n: load(n) for n in NUMBERS}
raw = Path("D:/デスクトップ/lsval.txt").read_text(encoding="utf-8-sig")
(OUT / "lsval_snapshot.txt").write_text(raw, encoding="utf-8")
values = [int(v) for v in raw.strip().split(",") if v.strip()]
assert len(values) == 20
maximum, minimum = np.array(values[:10]), np.array(values[10:])
span = maximum - minimum
assert ((minimum >= 0) & (maximum <= 4095) & (span > 0)).all()
with (OUT / "calibration.csv").open("w", encoding="utf-8-sig", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["sensor", "minimum", "maximum", "span", "normalized_gain"])
    writer.writerows((i, int(minimum[i]), int(maximum[i]), int(span[i]), round(4095 / span[i], 4)) for i in range(10))
summary = []
for n, (m, c) in runs.items():
    sensors = np.column_stack([c[f"lSensorCari{i}"] for i in range(10)])
    distance = c["encTotalOptimal"] / 58.019
    time = c["cntlog"]
    lost = (sensors[:, 3:7] < 100).all(axis=1)
    tail = len(time) - 1
    while tail > 0 and lost[tail - 1]:
        tail -= 1
    straight = (distance >= 800) & (distance < 1000)
    curve = (distance >= 5700) & (distance < 5950)
    row = {"log": n, "rows": len(time), "expected_rows": int(m["logExpectedRows"]),
           "emcStop": int(float(m["emcStop"])), "duration_ms": int(time[-1]),
           "distance_mm": round(distance[-1], 1), "cntlog_monotonic": bool((np.diff(time) > 0).all()),
           "data_finite": bool(all(np.isfinite(a).all() for a in c.values())),
           "largest_row_spacing_mm": round(float(np.diff(distance).max()), 2),
           "median_speed_mps": round(float(np.median(c["encCurrentN"] / 58.019)), 3),
           "battery_header_V": float(m["batteryVoltage_V"]),
           "terminal_center_lost_start_ms": int(time[tail]) if lost[-1] else None,
           "terminal_center_lost_start_mm": round(distance[tail], 1) if lost[-1] else None,
           "straight_sensor4_median": float(np.median(sensors[straight, 4])),
           "straight_sensor5_median": float(np.median(sensors[straight, 5])),
           "curve_gyro_median_dps": round(float(np.median(c["gyroVal_Z"][curve])), 1) if curve.any() else None,
           "curve_wheel_difference_median": float(np.median((c["encCurrentL"] - c["encCurrentR"])[curve])) if curve.any() else None,
           "line_kp": float(m["lineTraceCtrl.kp"]), "line_kd": float(m["lineTraceCtrl.kd"]),
           "lineomega_kp": float(m["lineTraceOmegaFBCtrl.kp"]), "lineomega_kd": float(m["lineTraceOmegaFBCtrl.kd"])}
    summary.append(row)
with (OUT / "run_summary.csv").open("w", encoding="utf-8-sig", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=list(summary[0]))
    writer.writeheader(); writer.writerows(summary)
(OUT / "validation.json").write_text(json.dumps({"runs": summary, "calibration_valid_range": True,
    "raw_sensor_samples_available": False, "calibration_snapshot_association": "User-supplied current SD settings; not embedded in log",
    "success_comparison_logs": [12971, 12976], "failed_runs_excluded_from_performance_adoption": [12983,12984,12985,12986,12987]}, indent=2), encoding="utf-8")

m, c = runs[12987]
s = np.column_stack([c[f"lSensorCari{i}"] for i in range(10)])
t = c["cntlog"] / 1000
fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True, constrained_layout=True)
axes[0].pcolormesh(t, np.arange(10), s.T, shading="nearest", vmin=0, vmax=4095, cmap="viridis")
axes[0].set_ylabel("Sensor (0=left)")
axes[0].set_title("12987: calibrated sensors / speed / yaw / slip (emcStop=5)")
axes[1].plot(t, c["targetSpeed"] / 58.019, label="Target")
axes[1].plot(t, c["encCurrentN"] / 58.019, label="Actual", alpha=.8)
axes[1].set_ylabel("Speed [m/s]"); axes[1].legend()
axes[2].plot(t, c["targetAngularvelo"], label="Line target")
axes[2].plot(t, c["gyroVal_Z"], label="Measured gyro", alpha=.8)
axes[2].set_ylabel("Yaw [deg/s]"); axes[2].legend()
axes[3].plot(t, c["slipFlag"], label="Longitudinal")
axes[3].plot(t, c["slipFlagLat"], label="Lateral")
axes[3].set_ylabel("Slip flag"); axes[3].set_xlabel("Time [s]"); axes[3].legend()
fig.savefig(OUT / "12987_sensors_control.png", dpi=140); plt.close(fig)
fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
for n in [12971, 12976, 12987]:
    _, v = runs[n]
    d = v["encTotalOptimal"] / 58.019
    q = d <= 6300
    axes[0].plot(v["x"][q], v["y"][q], label=str(n))
    q = (d >= 5500) & (d <= 6150)
    axes[1].plot(d[q], v["gyroVal_Z"][q], label=str(n))
axes[0].set_aspect("equal", adjustable="datalim"); axes[0].set_xlabel("X [mm]"); axes[0].set_ylabel("Y [mm]")
axes[0].set_title("Estimated path: first 6.3 m")
axes[1].set_xlabel("Distance [mm]"); axes[1].set_ylabel("Measured yaw [deg/s]"); axes[1].set_title("Same-distance curve comparison")
for ax in axes: ax.legend(); ax.grid(alpha=.3)
fig.savefig(OUT / "12987_vs_success.png", dpi=140); plt.close(fig)
print(json.dumps({"span": span.tolist(), "run_summary": summary}, ensure_ascii=False))
