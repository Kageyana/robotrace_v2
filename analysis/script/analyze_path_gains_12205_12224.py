"""Summarize recent PATH REPLAY/SHORTCUT logs and record gain metadata."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from statistics import mean

from path_log_recovery import read_csv_log, recover_path_columns


LOG_DIR = Path(r"F:\Dropbox\Document\robotrace\Log\v2")
OUT_DIR = Path(__file__).resolve().parents[1] / "path_gains_12205_12224"
LOG_NUMBERS = range(12205, 12225)
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
    log = read_csv_log(path)
    rows = log.rows
    recovery = recover_path_columns(log)
    for row, restored in zip(rows, recovery.row_values):
        row.update({key: str(value) for key, value in restored.items() if key.endswith("_mm")})
    return path, log.parameters, rows, recovery


def summarize(number: int):
    path, params, rows, recovery = load_log(number)
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
    margins = [float(row["pathLegalMargin_mm"]) for row in rows
               if math.isfinite(float(row["pathLegalMargin_mm"]))]
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
        "analysisSourceLog": params.get("analysisSourceLog", ""),
        "slipSourceLog": params.get("slipSourceLog", ""),
        "shortcutLevel": params.get("shortcutLevel", ""),
        "recovery_status": recovery.status,
        "recovery_reason": recovery.reason,
        "recovery_source_log": recovery.source_log,
        "recovery_missing_samples": recovery.missing_samples,
        "legal_margin_min_mm": (math.nan if recovery.missing_samples > 0 else (min(margins) if margins else math.nan)),
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
