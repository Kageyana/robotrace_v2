"""Validate the newest speed_ff=160 autoStart sets and compare valid runs by run number."""

from __future__ import annotations

import csv
import math
from pathlib import Path

import compare_logs_11921_11965_speed_ff160_by_autostart as base


NEW_LOGS = list(range(11966, 11991))
BASELINE_CSV = base.OUT_DIR / "log_11921_11965_speed_ff160_by_autostart.csv"


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    logs = {log_number: base.load(log_number) for log_number in NEW_LOGS}
    summaries: dict[int, dict] = {}
    for log_number, log in logs.items():
        is_secondary = log["params"].get("optimalTrace", math.nan) == 2
        valid, reason = base.valid_log(log, 2 if is_secondary else 0, 160.0)
        summary = {
            "log": log_number,
            "autoStart": log["params"].get("autoStart", math.nan),
            "optimalTrace": log["params"].get("optimalTrace", math.nan),
            "emcStop": log["params"].get("emcStop", math.nan),
            "speedFeedForwardGain": log["params"].get("speedFeedForwardGain", math.nan),
            "valid": valid,
            "invalid_reason": reason,
        }
        summary.update(base.metrics(log))
        summaries[log_number] = summary

    groups = base.grouped_by_set(NEW_LOGS, logs)
    complete, excluded = base.complete_sets(groups, summaries)
    run_rows: list[dict] = []
    for set_id, group in enumerate(complete, 1):
        for log_number in group:
            if int(summaries[log_number]["autoStart"]) == 1:
                continue
            row = {"condition": "speed_ff160_new", "set": set_id}
            row.update(summaries[log_number])
            run_rows.append(row)

    baseline_rows = []
    if BASELINE_CSV.exists():
        with BASELINE_CSV.open("r", encoding="utf-8", newline="") as file:
            baseline_rows = list(csv.DictReader(file))

    comparison: list[dict] = []
    for auto_start in (2, 3, 4, 5):
        old = [row for row in baseline_rows if row["condition"] == "speed_ff150_baseline" and int(float(row["autoStart"])) == auto_start]
        new = [row for row in run_rows if int(float(row["autoStart"])) == auto_start]
        result = {"autoStart": auto_start, "baseline_n": len(old), "new_n": len(new)}
        for field in ("last_cnt_ms", "speed_error_target_minus_actual_mean_mps", "speed_error_abs_p95_mps", "lineTraceCtrl_rms", "gyroVal_Z_rms", "motorDutySat_pct", "slipLat_pct"):
            old_value = base.mean([base.number(row[field]) for row in old])
            new_value = base.mean([base.number(row[field]) for row in new])
            result[f"baseline_{field}"] = old_value
            result[f"new_{field}"] = new_value
            result[f"new_minus_baseline_{field}"] = new_value - old_value
        comparison.append(result)

    set_rows = []
    for set_id, group in enumerate(complete, 1):
        secondary = [summaries[log_number] for log_number in group if int(summaries[log_number]["autoStart"]) != 1]
        set_rows.append({
            "condition": "speed_ff160_new",
            "set": set_id,
            "logs": f"{group[0]}-{group[-1]}",
            "complete": True,
            "battery_min_V": min(row["start_voltage_V"] for row in secondary),
            "battery_max_V": max(row["start_voltage_V"] for row in secondary),
            "lap_mean_secondary_ms": base.mean([row["last_cnt_ms"] for row in secondary]),
        })
    excluded_rows = [{"condition": "speed_ff160_new", **row} for row in excluded]

    base.write_csv(base.OUT_DIR / "log_11966_11990_speed_ff160_summary.csv", list(summaries.values()))
    write_csv(base.OUT_DIR / "log_11966_11990_speed_ff160_by_autostart.csv", run_rows)
    write_csv(base.OUT_DIR / "log_11966_11990_speed_ff160_comparison.csv", comparison)
    write_csv(base.OUT_DIR / "log_11966_11990_speed_ff160_sets.csv", set_rows)
    write_csv(base.OUT_DIR / "log_11966_11990_speed_ff160_excluded_sets.csv", excluded_rows)

    print(f"complete_sets={len(complete)}")
    for row in excluded_rows:
        print(f"excluded={row['logs']} sequence={row['autoStart_sequence']} reason={row['reason']}")
    for row in comparison:
        print(
            f"autoStart={row['autoStart']} n={row['baseline_n']}/{row['new_n']} "
            f"lap={row['baseline_last_cnt_ms']:.1f}->{row['new_last_cnt_ms']:.1f} "
            f"err_p95={row['baseline_speed_error_abs_p95_mps']:.3f}->{row['new_speed_error_abs_p95_mps']:.3f} "
            f"line={row['baseline_lineTraceCtrl_rms']:.2f}->{row['new_lineTraceCtrl_rms']:.2f} "
            f"gyro={row['baseline_gyroVal_Z_rms']:.2f}->{row['new_gyroVal_Z_rms']:.2f} "
            f"slipLat={row['baseline_slipLat_pct']:.2f}->{row['new_slipLat_pct']:.2f}"
        )


if __name__ == "__main__":
    main()
