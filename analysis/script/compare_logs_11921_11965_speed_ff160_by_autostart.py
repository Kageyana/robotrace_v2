"""Compare speed_ff=160 against the 7.1 V speed_ff=150 baseline by autoStart run."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from statistics import fmean


LOG_DIR = Path(r"F:\Dropbox\Document\robotrace\Log\v2")
OUT_DIR = Path(r"D:\robotrace\robotrace_v2\analysis")
BASELINE = list(range(11921, 11936))
CANDIDATE = list(range(11936, 11966))
PULSE_METER = 53424.0
START_VOLTAGE_MIN = 7.3
MIN_ROWS_SECONDARY = 850
MIN_LAST_CNT_MS = 5000
MAX_CNT_GAP_MS = 30
MIN_OPTIMAL_INDEX = 180


def number(value: str | None) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def mean(values: list[float]) -> float:
    values = [value for value in values if not math.isnan(value)]
    return fmean(values) if values else math.nan


def rms(values: list[float]) -> float:
    values = [value for value in values if not math.isnan(value)]
    return math.sqrt(fmean(value * value for value in values)) if values else math.nan


def p95_abs(values: list[float]) -> float:
    values = sorted(abs(value) for value in values if not math.isnan(value))
    if not values:
        return math.nan
    index = int(0.95 * (len(values) - 1))
    return values[index]


def percent_true(values: list[bool]) -> float:
    return 100.0 * sum(values) / len(values) if values else math.nan


def parse_csv(path: Path) -> tuple[list[str], dict[str, float], list[dict[str, str]]]:
    with path.open("r", encoding="utf-8-sig", newline="") as file:
        reader = csv.reader(file)
        header = next(reader)
        columns: list[str] = []
        params: dict[str, float] = {}
        parameter_started = False
        for cell in header:
            cell = cell.strip()
            if not cell:
                continue
            if "=" in cell:
                parameter_started = True
                key, value = cell.split("=", 1)
                params[key] = number(value)
            elif not parameter_started:
                columns.append(cell)
        rows = [dict(zip(columns, raw[: len(columns)])) for raw in reader if len(raw) >= len(columns)]
    return columns, params, rows


def load(log_number: int) -> dict:
    path = LOG_DIR / f"{log_number}.csv"
    columns, params, rows = parse_csv(path)
    return {"log": log_number, "columns": columns, "params": params, "rows": rows}


def valid_log(log: dict, expected_trace: int, expected_ff: float) -> tuple[bool, str]:
    params = log["params"]
    rows = log["rows"]
    reasons: list[str] = []
    if params.get("optimalTrace", math.nan) != expected_trace:
        reasons.append(f"optimalTrace={params.get('optimalTrace', math.nan)}")
    if params.get("speedFeedForwardGain", math.nan) != expected_ff:
        reasons.append(f"speedFeedForwardGain={params.get('speedFeedForwardGain', math.nan)}")
    if params.get("emcStop", math.nan) != 0:
        reasons.append(f"emcStop={params.get('emcStop', math.nan)}")
    if params.get("batteryVoltage_V", math.nan) < START_VOLTAGE_MIN:
        reasons.append(f"batteryVoltage_V={params.get('batteryVoltage_V', math.nan)}")
    required = {"cntlog", "encCurrentN", "targetSpeed", "gyroVal_Z", "lineTraceCtrl", "motorpwmL", "motorpwmR", "slipFlagLat"}
    missing = sorted(required.difference(log["columns"]))
    if missing:
        reasons.append("missing=" + "|".join(missing))
    cnt = [number(row.get("cntlog")) for row in rows]
    if len(rows) < MIN_ROWS_SECONDARY:
        reasons.append(f"rows={len(rows)}")
    if not cnt:
        reasons.append("rows=0")
    else:
        if cnt[-1] < MIN_LAST_CNT_MS:
            reasons.append(f"last_cnt_ms={cnt[-1]}")
        gaps = [b - a for a, b in zip(cnt, cnt[1:])]
        if any(gap <= 0 for gap in gaps):
            reasons.append("cntlog_nonmonotonic")
        if gaps and max(gaps) > MAX_CNT_GAP_MS:
            reasons.append(f"max_cnt_gap_ms={max(gaps)}")
        if params.get("optimalTrace", math.nan) == 2:
            max_index = max((number(row.get("optimalIndex")) for row in rows), default=math.nan)
            if math.isnan(max_index) or max_index < MIN_OPTIMAL_INDEX:
                reasons.append(f"optimalIndex_max={max_index}")
    return not reasons, ";".join(reasons)


def metrics(log: dict) -> dict:
    rows = log["rows"]
    errors = [(number(row.get("targetSpeed")) - number(row.get("encCurrentN"))) / PULSE_METER * 1000.0 for row in rows]
    return {
        "rows": len(rows),
        "last_cnt_ms": number(rows[-1].get("cntlog")) if rows else math.nan,
        "max_cnt_gap_ms": max((b - a for a, b in zip((number(row.get("cntlog")) for row in rows), (number(row.get("cntlog")) for row in rows[1:]))), default=math.nan),
        "optimalIndex_max": max((number(row.get("optimalIndex")) for row in rows), default=math.nan),
        "start_voltage_V": log["params"].get("batteryVoltage_V", math.nan),
        "speed_error_target_minus_actual_mean_mps": mean(errors),
        "speed_error_abs_p95_mps": p95_abs(errors),
        "lineTraceCtrl_rms": rms([number(row.get("lineTraceCtrl")) for row in rows]),
        "gyroVal_Z_rms": rms([number(row.get("gyroVal_Z")) for row in rows]),
        "motorDutySat_pct": percent_true([max(abs(number(row.get("motorpwmL"))), abs(number(row.get("motorpwmR")))) >= 1000 for row in rows]),
        "slipLat_pct": percent_true([number(row.get("slipFlagLat")) != 0 for row in rows]),
        "slipLat_count": sum(number(row.get("slipFlagLat")) != 0 for row in rows),
    }


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def target_band(value: float) -> str:
    if value < 120:
        return "<120"
    if value < 160:
        return "120-159"
    if value < 180:
        return "160-179"
    return ">=180"


def grouped_by_set(log_numbers: list[int], logs: dict[int, dict]) -> list[list[int]]:
    groups: list[list[int]] = []
    current: list[int] = []
    for log_number in log_numbers:
        auto_start = int(logs[log_number]["params"].get("autoStart", math.nan))
        if auto_start == 1 and current:
            groups.append(current)
            current = []
        current.append(log_number)
    if current:
        groups.append(current)
    return groups


def complete_sets(groups: list[list[int]], summaries: dict[int, dict]) -> tuple[list[list[int]], list[dict]]:
    complete: list[list[int]] = []
    excluded: list[dict] = []
    for group in groups:
        sequence = [int(summaries[number]["autoStart"]) for number in group]
        failed = [summaries[number]["invalid_reason"] for number in group if summaries[number]["valid"] is False]
        if sequence == [1, 2, 3, 4, 5] and not failed:
            complete.append(group)
        else:
            excluded.append({"logs": f"{group[0]}-{group[-1]}", "autoStart_sequence": ",".join(map(str, sequence)), "reason": ";".join(failed) or "incomplete_autoStart_set"})
    return complete, excluded


def main() -> None:
    numbers = BASELINE + CANDIDATE
    logs = {number_: load(number_) for number_ in numbers}
    summaries: dict[int, dict] = {}
    for log_number, log in logs.items():
        is_secondary = log["params"].get("optimalTrace", math.nan) == 2
        valid, reason = valid_log(log, 2 if is_secondary else 0, 150.0 if log_number in BASELINE else 160.0)
        summary = {"log": log_number, "autoStart": log["params"].get("autoStart", math.nan), "optimalTrace": log["params"].get("optimalTrace", math.nan), "emcStop": log["params"].get("emcStop", math.nan), "speedFeedForwardGain": log["params"].get("speedFeedForwardGain", math.nan), "valid": valid, "invalid_reason": reason}
        summary.update(metrics(log))
        summaries[log_number] = summary

    base_groups, base_excluded = complete_sets(grouped_by_set(BASELINE, logs), summaries)
    candidate_groups, candidate_excluded = complete_sets(grouped_by_set(CANDIDATE, logs), summaries)
    run_rows: list[dict] = []
    for label, groups in (("speed_ff150_baseline", base_groups), ("speed_ff160_candidate", candidate_groups)):
        for set_id, group in enumerate(groups, 1):
            for log_number in group:
                if int(summaries[log_number]["autoStart"]) == 1:
                    continue
                row = {"condition": label, "set": set_id}
                row.update(summaries[log_number])
                run_rows.append(row)

    comparison: list[dict] = []
    for auto_start in (2, 3, 4, 5):
        base_rows = [row for row in run_rows if row["condition"] == "speed_ff150_baseline" and int(row["autoStart"]) == auto_start]
        candidate_rows = [row for row in run_rows if row["condition"] == "speed_ff160_candidate" and int(row["autoStart"]) == auto_start]
        output = {"autoStart": auto_start, "baseline_n": len(base_rows), "candidate_n": len(candidate_rows)}
        for field in ("last_cnt_ms", "speed_error_target_minus_actual_mean_mps", "speed_error_abs_p95_mps", "lineTraceCtrl_rms", "gyroVal_Z_rms", "motorDutySat_pct", "slipLat_pct"):
            base_value = mean([number(row[field]) for row in base_rows])
            candidate_value = mean([number(row[field]) for row in candidate_rows])
            output[f"baseline_{field}"] = base_value
            output[f"candidate_{field}"] = candidate_value
            output[f"candidate_minus_baseline_{field}"] = candidate_value - base_value
        comparison.append(output)

    set_rows = []
    for label, groups in (("speed_ff150_baseline", base_groups), ("speed_ff160_candidate", candidate_groups)):
        for set_id, group in enumerate(groups, 1):
            secondary = [summaries[number] for number in group if int(summaries[number]["autoStart"]) != 1]
            set_rows.append({"condition": label, "set": set_id, "logs": f"{group[0]}-{group[-1]}", "complete": True, "battery_min_V": min(row["start_voltage_V"] for row in secondary), "battery_max_V": max(row["start_voltage_V"] for row in secondary), "lap_mean_secondary_ms": mean([row["last_cnt_ms"] for row in secondary])})
    excluded_rows = [{"condition": "speed_ff150_baseline", **row} for row in base_excluded] + [{"condition": "speed_ff160_candidate", **row} for row in candidate_excluded]

    band_rows: list[dict] = []
    for label, groups in (("speed_ff150_baseline", base_groups), ("speed_ff160_candidate", candidate_groups)):
        secondary_numbers = [log_number for group in groups for log_number in group if int(summaries[log_number]["autoStart"]) != 1]
        rows_by_band: dict[str, list[dict[str, str]]] = {band: [] for band in ("<120", "120-159", "160-179", ">=180")}
        for log_number in secondary_numbers:
            for row in logs[log_number]["rows"]:
                rows_by_band[target_band(number(row.get("targetSpeed")))].append(row)
        for band, rows in rows_by_band.items():
            errors = [(number(row.get("targetSpeed")) - number(row.get("encCurrentN"))) / PULSE_METER * 1000.0 for row in rows]
            band_rows.append({
                "condition": label,
                "band": band,
                "n": len(rows),
                "targetSpeed_mean_pulse_ms": mean([number(row.get("targetSpeed")) for row in rows]),
                "speed_error_target_minus_actual_mean_mps": mean(errors),
                "speed_error_abs_p95_mps": p95_abs(errors),
                "motorDutySat_pct": percent_true([max(abs(number(row.get("motorpwmL"))), abs(number(row.get("motorpwmR")))) >= 1000 for row in rows]),
                "slipLat_pct": percent_true([number(row.get("slipFlagLat")) != 0 for row in rows]),
            })

    write_csv(OUT_DIR / "log_11921_11965_speed_ff160_summary.csv", list(summaries.values()))
    write_csv(OUT_DIR / "log_11921_11965_speed_ff160_by_autostart.csv", run_rows)
    write_csv(OUT_DIR / "log_11921_11965_speed_ff160_comparison.csv", comparison)
    write_csv(OUT_DIR / "log_11921_11965_speed_ff160_sets.csv", set_rows)
    write_csv(OUT_DIR / "log_11921_11965_speed_ff160_excluded_sets.csv", excluded_rows)
    write_csv(OUT_DIR / "log_11921_11965_speed_ff160_by_target_band.csv", band_rows)

    print(f"baseline_complete_sets={len(base_groups)} candidate_complete_sets={len(candidate_groups)}")
    for row in excluded_rows:
        print(f"excluded {row['condition']} {row['logs']}: {row['reason']}")
    for row in comparison:
        print(f"autoStart={row['autoStart']} n={row['baseline_n']}/{row['candidate_n']} "
              f"lap={row['baseline_last_cnt_ms']:.1f}->{row['candidate_last_cnt_ms']:.1f} "
              f"err_p95={row['baseline_speed_error_abs_p95_mps']:.3f}->{row['candidate_speed_error_abs_p95_mps']:.3f} "
              f"line={row['baseline_lineTraceCtrl_rms']:.2f}->{row['candidate_lineTraceCtrl_rms']:.2f} "
              f"gyro={row['baseline_gyroVal_Z_rms']:.2f}->{row['candidate_gyroVal_Z_rms']:.2f} "
              f"slipLat={row['baseline_slipLat_pct']:.2f}->{row['candidate_slipLat_pct']:.2f}")


if __name__ == "__main__":
    main()
