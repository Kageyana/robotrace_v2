from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable


DEFAULT_LOG_DIR = Path(r"F:\Dropbox\Document\robotrace\Log\v2")


def parse_number(text: str):
    text = text.strip()
    if text == "":
        return math.nan
    try:
        value = float(text)
    except ValueError:
        return text
    return value


def mean(values: Iterable[float]) -> float:
    values = [v for v in values if not math.isnan(v)]
    if not values:
        return math.nan
    return sum(values) / len(values)


def rmse(values: Iterable[float]) -> float:
    values = [v for v in values if not math.isnan(v)]
    if not values:
        return math.nan
    return math.sqrt(sum(v * v for v in values) / len(values))


def percentile(values: Iterable[float], pct: float) -> float:
    values = sorted(v for v in values if not math.isnan(v))
    if not values:
        return math.nan
    if len(values) == 1:
        return values[0]
    pos = (len(values) - 1) * pct / 100.0
    lo = math.floor(pos)
    hi = math.ceil(pos)
    if lo == hi:
        return values[int(pos)]
    return values[lo] * (hi - pos) + values[hi] * (pos - lo)


def pct_true(values: Iterable[bool]) -> float:
    values = list(values)
    if not values:
        return math.nan
    return sum(1 for v in values if v) * 100.0 / len(values)


def fmt_logs(logs: Iterable[int]) -> str:
    return " ".join(str(log) for log in logs)


def read_log(log_dir: Path, log_num: int) -> dict:
    path = log_dir / f"{log_num}.csv"
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        columns: list[str] = []
        params: dict[str, object] = {}
        param_started = False
        for item in header:
            item = item.strip()
            if item == "":
                continue
            if "=" in item:
                param_started = True
                key, value = item.split("=", 1)
                params[key] = parse_number(value)
            elif not param_started:
                columns.append(item)

        rows: list[dict[str, float]] = []
        for raw in reader:
            if not raw or all(cell.strip() == "" for cell in raw):
                continue
            row: dict[str, float] = {}
            for col, value in zip(columns, raw):
                parsed = parse_number(value)
                row[col] = parsed if isinstance(parsed, float) else math.nan
            rows.append(row)

    return {"log": log_num, "path": path, "columns": columns, "params": params, "rows": rows}


def param_float(log: dict, key: str) -> float:
    value = log["params"].get(key, math.nan)
    return value if isinstance(value, float) else math.nan


def col_values(logs: list[dict], col: str) -> list[float]:
    return [row.get(col, math.nan) for log in logs for row in log["rows"]]


def speed_errors(logs: list[dict], predicate=lambda row: True) -> list[float]:
    errs: list[float] = []
    for log in logs:
        for row in log["rows"]:
            if predicate(row):
                errs.append(row.get("encCurrentN", math.nan) - row.get("targetSpeed", math.nan))
    return errs


def selected_rows(logs: list[dict], predicate) -> list[dict[str, float]]:
    return [row for log in logs for row in log["rows"] if predicate(row)]


def target_band_predicate(band: str):
    if band == "all":
        return lambda row: True
    if band == "<120":
        return lambda row: row.get("targetSpeed", math.nan) < 120
    if band == "120-159":
        return lambda row: 120 <= row.get("targetSpeed", math.nan) <= 159
    if band == "160-179":
        return lambda row: 160 <= row.get("targetSpeed", math.nan) <= 179
    if band == "180-189":
        return lambda row: 180 <= row.get("targetSpeed", math.nan) <= 189
    if band == ">=190":
        return lambda row: row.get("targetSpeed", math.nan) >= 190
    raise ValueError(f"unknown band: {band}")


def validate_log(
    log: dict,
    expected_speed_ff: float | None,
    require_speed_target_clip: bool,
) -> tuple[bool, str]:
    reasons: list[str] = []
    rows = log["rows"]
    if param_float(log, "optimalTrace") != 2:
        reasons.append(f"optimalTrace={param_float(log, 'optimalTrace')}")
    if param_float(log, "emcStop") != 0:
        reasons.append(f"emcStop={param_float(log, 'emcStop')}")
    if expected_speed_ff is not None and param_float(log, "speedFeedForwardGain") != expected_speed_ff:
        reasons.append(f"speedFeedForwardGain={param_float(log, 'speedFeedForwardGain')}")

    for base in ("veloCtrl", "veloCtrlL", "veloCtrlR"):
        if param_float(log, f"{base}.kp") != 10:
            reasons.append(f"{base}.kp={param_float(log, f'{base}.kp')}")
        if param_float(log, f"{base}.ki") != 0:
            reasons.append(f"{base}.ki={param_float(log, f'{base}.ki')}")
        if param_float(log, f"{base}.kd") != 0:
            reasons.append(f"{base}.kd={param_float(log, f'{base}.kd')}")

    required = {"cntlog", "encCurrentN", "targetSpeed", "ROC", "gyroVal_Z", "lineTraceCtrl"}
    if require_speed_target_clip:
        required.update({"targetSpeedL", "targetSpeedR", "speedTargetClip"})
    missing = sorted(required.difference(log["columns"]))
    if missing:
        reasons.append("missing=" + "|".join(missing))

    if len(rows) < 850:
        reasons.append(f"rows={len(rows)}")
    cnt = [row.get("cntlog", math.nan) for row in rows]
    if cnt:
        if cnt[-1] < 5000:
            reasons.append(f"last_cnt={cnt[-1]}")
        if any(b <= a for a, b in zip(cnt, cnt[1:])):
            reasons.append("cntlog_nonmonotonic")
    else:
        reasons.append("rows=0")
    opt_index = col_values([log], "optimalIndex")
    opt_index_max = max(opt_index) if opt_index else math.nan
    if math.isnan(opt_index_max) or opt_index_max < 180:
        reasons.append(f"optimalIndex_max={opt_index_max}")

    return not reasons, ";".join(reasons)


def summarize_log(log: dict, expected_speed_ff: float | None, require_clip: bool) -> dict:
    valid, invalid_reasons = validate_log(log, expected_speed_ff, require_clip)
    rows = log["rows"]
    cnt = col_values([log], "cntlog")
    opt_index = col_values([log], "optimalIndex")
    straight = selected_rows(
        [log],
        lambda row: abs(row.get("ROC", math.nan)) >= 2500
        and row.get("targetSpeed", math.nan) > 0
        and row.get("cntlog", math.nan) >= 100,
    )
    all_err = speed_errors([log])
    high180_err = speed_errors([log], lambda row: row.get("targetSpeed", math.nan) >= 180)
    high190_err = speed_errors([log], lambda row: row.get("targetSpeed", math.nan) >= 190)
    pwm_sat = pct_true(
        abs(row.get("motorpwmL", math.nan)) >= 999 or abs(row.get("motorpwmR", math.nan)) >= 999
        for row in rows
    )
    high190_rows = [row for row in rows if row.get("targetSpeed", math.nan) >= 190]
    high_pwm_sat = pct_true(
        abs(row.get("motorpwmL", math.nan)) >= 999 or abs(row.get("motorpwmR", math.nan)) >= 999
        for row in high190_rows
    )
    speed_clip = pct_true(row.get("speedTargetClip", 0) != 0 for row in rows)
    high190_clip = pct_true(row.get("speedTargetClip", 0) != 0 for row in high190_rows)
    curve_gain = pct_true(
        row.get("lineTraceOmegaFBCtrlkp", math.nan) == 5
        or row.get("lineTraceOmegaFBCtrlkd", math.nan) == 15
        for row in straight
    )
    return {
        "log": log["log"],
        "valid": valid,
        "invalid_reasons": invalid_reasons,
        "rows": len(rows),
        "last_cnt": cnt[-1] if cnt else math.nan,
        "optimalIndex_max": max(opt_index) if opt_index else math.nan,
        "batteryVoltage_V": param_float(log, "batteryVoltage_V"),
        "optimalTrace": param_float(log, "optimalTrace"),
        "autoStart": param_float(log, "autoStart"),
        "emcStop": param_float(log, "emcStop"),
        "veloCtrl.kp": param_float(log, "veloCtrl.kp"),
        "veloCtrl.ki": param_float(log, "veloCtrl.ki"),
        "veloCtrl.kd": param_float(log, "veloCtrl.kd"),
        "veloCtrlL.kp": param_float(log, "veloCtrlL.kp"),
        "veloCtrlR.kp": param_float(log, "veloCtrlR.kp"),
        "speedFeedForwardGain": param_float(log, "speedFeedForwardGain"),
        "lineTraceOmegaFBCtrl.kp": param_float(log, "lineTraceOmegaFBCtrl.kp"),
        "lineTraceOmegaFBCtrl.kd": param_float(log, "lineTraceOmegaFBCtrl.kd"),
        "speed_mean": mean(all_err),
        "speed_rmse": rmse(all_err),
        "speed_abs_p95": percentile([abs(e) for e in all_err], 95),
        "high180_mean": mean(high180_err),
        "high180_rmse": rmse(high180_err),
        "high180_abs_p95": percentile([abs(e) for e in high180_err], 95),
        "high190_mean": mean(high190_err),
        "high190_rmse": rmse(high190_err),
        "high190_abs_p95": percentile([abs(e) for e in high190_err], 95),
        "pwm_sat_pct": pwm_sat,
        "high190_pwm_sat_pct": high_pwm_sat,
        "speedTargetClip_pct": speed_clip,
        "high190_clip_pct": high190_clip,
        "slip_pct": pct_true(row.get("slipFlag", 0) != 0 for row in rows),
        "slipLat_pct": pct_true(row.get("slipFlagLat", 0) != 0 for row in rows),
        "gyro_rms": rmse(row.get("gyroVal_Z", math.nan) for row in straight),
        "gyro_abs_p95": percentile([abs(row.get("gyroVal_Z", math.nan)) for row in straight], 95),
        "line_rms": rmse(row.get("lineTraceCtrl", math.nan) for row in straight),
        "line_abs_p95": percentile([abs(row.get("lineTraceCtrl", math.nan)) for row in straight], 95),
        "curve_gain_straight_pct": curve_gain,
    }


def summarize_group(group: str, requested: list[int], logs: list[dict], summaries: dict[int, dict]) -> dict:
    valid_logs = [log for log in logs if summaries[log["log"]]["valid"]]
    invalid_logs = [log for log in logs if not summaries[log["log"]]["valid"]]
    rows = [row for log in valid_logs for row in log["rows"]]
    straight = [
        row
        for row in rows
        if abs(row.get("ROC", math.nan)) >= 2500
        and row.get("targetSpeed", math.nan) > 0
        and row.get("cntlog", math.nan) >= 100
    ]
    all_err = [row.get("encCurrentN", math.nan) - row.get("targetSpeed", math.nan) for row in rows]
    high180 = [row for row in rows if row.get("targetSpeed", math.nan) >= 180]
    high190 = [row for row in rows if row.get("targetSpeed", math.nan) >= 190]
    high180_err = [row.get("encCurrentN", math.nan) - row.get("targetSpeed", math.nan) for row in high180]
    high190_err = [row.get("encCurrentN", math.nan) - row.get("targetSpeed", math.nan) for row in high190]
    return {
        "group": group,
        "requested_logs": fmt_logs(requested),
        "valid_logs": fmt_logs(log["log"] for log in valid_logs),
        "invalid_logs": fmt_logs(log["log"] for log in invalid_logs),
        "valid_count": len(valid_logs),
        "battery_min": min((param_float(log, "batteryVoltage_V") for log in valid_logs), default=math.nan),
        "battery_mean": mean(param_float(log, "batteryVoltage_V") for log in valid_logs),
        "battery_max": max((param_float(log, "batteryVoltage_V") for log in valid_logs), default=math.nan),
        "lap_mean": mean(summaries[log["log"]]["last_cnt"] for log in valid_logs),
        "speed_mean": mean(all_err),
        "speed_rmse": rmse(all_err),
        "speed_abs_p95": percentile([abs(e) for e in all_err], 95),
        "high180_mean": mean(high180_err),
        "high180_rmse": rmse(high180_err),
        "high180_abs_p95": percentile([abs(e) for e in high180_err], 95),
        "high190_mean": mean(high190_err),
        "high190_rmse": rmse(high190_err),
        "high190_abs_p95": percentile([abs(e) for e in high190_err], 95),
        "pwm_sat_pct": pct_true(
            abs(row.get("motorpwmL", math.nan)) >= 999 or abs(row.get("motorpwmR", math.nan)) >= 999
            for row in rows
        ),
        "high190_pwm_sat_pct": pct_true(
            abs(row.get("motorpwmL", math.nan)) >= 999 or abs(row.get("motorpwmR", math.nan)) >= 999
            for row in high190
        ),
        "speedTargetClip_pct": pct_true(row.get("speedTargetClip", 0) != 0 for row in rows),
        "high190_clip_pct": pct_true(row.get("speedTargetClip", 0) != 0 for row in high190),
        "slip_pct": pct_true(row.get("slipFlag", 0) != 0 for row in rows),
        "slipLat_pct": pct_true(row.get("slipFlagLat", 0) != 0 for row in rows),
        "gyro_rms": rmse(row.get("gyroVal_Z", math.nan) for row in straight),
        "gyro_abs_p95": percentile([abs(row.get("gyroVal_Z", math.nan)) for row in straight], 95),
        "line_rms": rmse(row.get("lineTraceCtrl", math.nan) for row in straight),
        "line_abs_p95": percentile([abs(row.get("lineTraceCtrl", math.nan)) for row in straight], 95),
        "curve_gain_straight_pct": pct_true(
            row.get("lineTraceOmegaFBCtrlkp", math.nan) == 5
            or row.get("lineTraceOmegaFBCtrlkd", math.nan) == 15
            for row in straight
        ),
    }


def band_rows(group: str, logs: list[dict], summaries: dict[int, dict]) -> list[dict]:
    valid_logs = [log for log in logs if summaries[log["log"]]["valid"]]
    out: list[dict] = []
    for band in ("all", "<120", "120-159", "160-179", "180-189", ">=190"):
        predicate = target_band_predicate(band)
        rows = [row for log in valid_logs for row in log["rows"] if predicate(row)]
        errs = [row.get("encCurrentN", math.nan) - row.get("targetSpeed", math.nan) for row in rows]
        out.append(
            {
                "group": group,
                "band": band,
                "n": len(rows),
                "mean": mean(errs),
                "rmse": rmse(errs),
                "abs_p95": percentile([abs(e) for e in errs], 95),
                "pwm_sat_pct": pct_true(
                    abs(row.get("motorpwmL", math.nan)) >= 999
                    or abs(row.get("motorpwmR", math.nan)) >= 999
                    for row in rows
                ),
                "speedTargetClip_pct": pct_true(row.get("speedTargetClip", 0) != 0 for row in rows),
            }
        )
    return out


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--out-dir", type=Path, default=Path("analysis"))
    args = parser.parse_args()

    baseline = [9826, 9827, 9828, 9829, 9831, 9832, 9833, 9834]
    candidate_all = list(range(9835, 9850))
    candidate_set1 = [9836, 9837, 9838, 9839]
    candidate_set2 = [9841, 9842, 9843, 9844]
    candidate_set3 = [9846, 9847, 9848, 9849]

    all_numbers = sorted(set(baseline + candidate_all))
    logs = {num: read_log(args.log_dir, num) for num in all_numbers}
    summaries: dict[int, dict] = {}
    for num, log in logs.items():
        expected_ff = 150.0 if num in baseline else 200.0
        summaries[num] = summarize_log(log, expected_ff, require_clip=True)

    summary_path = args.out_dir / "log_9835_9849_speed_ff200_summary.csv"
    write_csv(summary_path, [summaries[num] for num in candidate_all])

    groups = [
        ("ff150_baseline_9826_9834", baseline),
        ("ff200_set1_9836_9839", candidate_set1),
        ("ff200_set2_9841_9844", candidate_set2),
        ("ff200_set3_9846_9849", candidate_set3),
        ("ff200_complete_9836_9849", candidate_set1 + candidate_set2 + candidate_set3),
    ]
    comparison = [
        summarize_group(name, nums, [logs[num] for num in nums], summaries) for name, nums in groups
    ]
    comparison_path = args.out_dir / "log_9826_9834_vs_9835_9849_speed_ff200_comparison.csv"
    write_csv(comparison_path, comparison)

    band_out: list[dict] = []
    for name, nums in groups:
        band_out.extend(band_rows(name, [logs[num] for num in nums], summaries))
    band_path = args.out_dir / "log_9826_9834_vs_9835_9849_speed_ff200_by_target_band.csv"
    write_csv(band_path, band_out)

    print(summary_path)
    print(comparison_path)
    print(band_path)


if __name__ == "__main__":
    main()
