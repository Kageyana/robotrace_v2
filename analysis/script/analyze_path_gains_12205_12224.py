"""Summarize recent PATH REPLAY/SHORTCUT logs and record gain metadata."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from statistics import mean


LOG_DIR = Path(r"F:\Dropbox\Document\robotrace\Log\v2")
OUT_DIR = Path(__file__).resolve().parents[1] / "path_gains_12205_12224"
LOG_NUMBERS = range(12205, 12225)
DATA_FIELDS = [
    "cntlog", "encCurrentN", "gyroVal_Z", "courseMarker", "encTotalOptimal",
    "ROC", "targetSpeed", "optimalIndex", "slipFlag", "slipFlagLat",
    "lineTraceCtrl", "targetAngularvelo", "motorpwmL", "motorpwmR",
    "batteryVoltage_mV", "motorVoltageCmdL_mV", "motorVoltageCmdR_mV",
    "encCurrentCorr_p", "linePointX_mm", "linePointY_mm", "lineValid",
    "pathErrorY_mm", "pathErrorHeading_cdeg", "pathState", "pathLegalMargin_mm",
    "x", "y",
]


def f(row: dict[str, str], key: str) -> float:
    return float(row[key])


def p95(values: list[float]) -> float:
    if not values:
        return float("nan")
    values = sorted(values)
    pos = 0.95 * (len(values) - 1)
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return values[lo]
    return values[lo] + (values[hi] - values[lo]) * (pos - lo)


def load_log(number: int):
    path = LOG_DIR / f"{number}.csv"
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.reader(stream)
        header = next(reader)
        params = {}
        for field in header:
            if "=" in field:
                key, value = field.split("=", 1)
                params[key] = value
        rows = []
        for raw in reader:
            if len(raw) < len(DATA_FIELDS):
                continue
            rows.append(dict(zip(DATA_FIELDS, raw[: len(DATA_FIELDS)])))
    return path, params, rows


def summarize(number: int):
    path, params, rows = load_log(number)
    cnt = [f(row, "cntlog") for row in rows]
    x = [f(row, "x") for row in rows]
    y = [f(row, "y") for row in rows]
    ey = [abs(f(row, "pathErrorY_mm")) for row in rows]
    eh = [abs(f(row, "pathErrorHeading_cdeg")) / 100.0 for row in rows]
    target_yaw = [abs(f(row, "targetAngularvelo")) for row in rows]
    gyro = [abs(f(row, "gyroVal_Z")) for row in rows]
    yaw_err = [abs(f(row, "targetAngularvelo") - f(row, "gyroVal_Z")) for row in rows]
    speed = [f(row, "targetSpeed") for row in rows]
    states = [int(float(row["pathState"])) for row in rows]
    line_valid = [int(float(row["lineValid"])) for row in rows]
    slips = [int(float(row["slipFlag"])) != 0 or int(float(row["slipFlagLat"])) != 0 for row in rows]
    diffs = [b - a for a, b in zip(cnt, cnt[1:])]
    state_counts = {str(state): states.count(state) for state in sorted(set(states))}
    return {
        "log": number,
        "rows": len(rows),
        "mode": params.get("optimalTrace", ""),
        "autoStart": params.get("autoStart", ""),
        "emcStop": params.get("emcStop", ""),
        "batteryVoltage_V": params.get("batteryVoltage_V", ""),
        "routeSourceLog": params.get("routeSourceLog", ""),
        "shortcutLevel": params.get("shortcutLevel", ""),
        "cnt_first": cnt[0] if cnt else "",
        "cnt_last": cnt[-1] if cnt else "",
        "duration_s": cnt[-1] / 1000.0 if cnt else "",
        "max_cnt_gap_ms": max(diffs) if diffs else "",
        "non_monotonic": sum(1 for d in diffs if d <= 0),
        "end_x_mm": x[-1] if x else "",
        "end_y_mm": y[-1] if y else "",
        "path_length_mm": sum(math.hypot(bx - ax, by - ay) for ax, ay, bx, by in zip(x, y, x[1:], y[1:])),
        "max_abs_pathErrorY_mm": max(ey) if ey else "",
        "p95_abs_pathErrorY_mm": p95(ey),
        "max_abs_heading_deg": max(eh) if eh else "",
        "p95_abs_heading_deg": p95(eh),
        "max_abs_targetAngularvelo_deg_s": max(target_yaw) if target_yaw else "",
        "max_abs_gyro_deg_s": max(gyro) if gyro else "",
        "p95_abs_yaw_error_deg_s": p95(yaw_err),
        "mean_targetSpeed_pulse_ms": mean(speed) if speed else "",
        "max_targetSpeed_pulse_ms": max(speed) if speed else "",
        "line_invalid_rows": sum(1 for value in line_valid if value == 0),
        "slip_rows": sum(slips),
        "path_states": ";".join(f"{key}:{value}" for key, value in state_counts.items()),
        "final_courseMarker": rows[-1]["courseMarker"] if rows else "",
    }


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    summaries = []
    for number in LOG_NUMBERS:
        path = LOG_DIR / f"{number}.csv"
        if path.exists():
            summaries.append(summarize(number))
    output = OUT_DIR / "path_gains_12205_12224_summary.csv"
    fields = list(summaries[0]) if summaries else []
    with output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(summaries)
    print(output)
    for row in summaries:
        print(",".join(str(row[field]) for field in fields))


if __name__ == "__main__":
    main()
