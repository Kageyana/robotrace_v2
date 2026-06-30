#!/usr/bin/env python3
"""Create slip detection review tables from robotrace CSV logs."""

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


LOG_ROOT = Path(r"F:\Dropbox\Document\robotrace\Log\v2")
DEFAULT_OUT_DIR = Path("analysis")
PULSE_METER = 53424.0

LONG_HIGH_COUNT_REQ = 8
LONG_LOW_COUNT_REQ = 6
LAT_HIGH_COUNT_REQ = 3
LAT_LOW_COUNT_REQ = 5


ALIASES = {
    "speed_mps": ("speed", "currentSpeed", "encSpeed", "speed_mps"),
    "target_mps": ("targetSpeed_mps", "targetSpeed", "targetSpeedLog"),
    "enc_current_n": ("encCurrentN",),
    "slip_long": ("slipLongResidual_mps2", "slipRatio"),
    "slip_lat": ("slipLatResidual_mps2", "slipRatioLat"),
    "motor_l": ("motorpwmL", "motorPwmL", "veloCtrlL_pwm"),
    "motor_r": ("motorpwmR", "motorPwmR", "veloCtrlR_pwm"),
    "current_l": ("motorCurrentL", "currentL"),
    "current_r": ("motorCurrentR", "currentR"),
    "slip_threshold_high": ("slipThresholdHigh",),
    "slip_threshold_low": ("slipThresholdLow",),
    "slip_lat_high": ("slipLatHigh",),
    "slip_lat_low": ("slipLatLow",),
    "slip_cur_scale": ("slipCurScale", "curScale"),
    "slip_turning": ("slipTurningState", "turningState"),
    "slip_ax_bias": ("slipAxBias", "axBias"),
    "slip_ay_bias": ("slipAyBias", "ayBias"),
    "slip_long_on_count_enabled": ("slipLongOnCountEnabled",),
    "slip_long_lowload_clear_count": ("slipLongLowloadClearCount",),
    "slip_lat_on_count_enabled": ("slipLatOnCountEnabled",),
    "slip_lat_enabled": ("slipLatEnabled",),
    "slip_i_sum_f": ("slipISumF",),
    "slip_enc_ay_f": ("slipEncAyF",),
}


@dataclass
class LogData:
    log_num: int
    path: Path
    params: dict[str, str]
    columns: list[str]
    rows: list[dict[str, str]]


def parse_log_spec(value: str) -> list[int]:
    logs: list[int] = []
    for part in value.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            start_s, end_s = part.split("-", 1)
            start, end = int(start_s), int(end_s)
            step = 1 if end >= start else -1
            logs.extend(range(start, end + step, step))
        else:
            logs.append(int(part))
    return logs


def log_path(log_num: int, log_root: Path) -> Path:
    return log_root / f"{log_num}.csv"


def read_log(path: Path, log_num: int) -> LogData:
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        raw_rows = list(csv.reader(f))
    if not raw_rows:
        raise ValueError(f"{path} is empty")

    first_data_idx = 1
    for i, row in enumerate(raw_rows[1:], start=1):
        if row and row[0].strip().isdigit():
            first_data_idx = i
            break

    header = raw_rows[0]
    first_data = raw_rows[first_data_idx] if first_data_idx < len(raw_rows) else []
    while first_data and first_data[-1] == "":
        first_data = first_data[:-1]
    column_count = min(len(header), len(first_data)) if first_data else len(header)
    columns = [c.strip() for c in header[:column_count]]

    params: dict[str, str] = {}
    for token in header[column_count:]:
        token = token.strip()
        if not token or "=" not in token:
            continue
        key, val = token.split("=", 1)
        params[key] = val

    rows: list[dict[str, str]] = []
    for row in raw_rows[first_data_idx:]:
        if not row or not row[0].strip():
            continue
        values = row[:column_count]
        if len(values) < column_count:
            values.extend([""] * (column_count - len(values)))
        rows.append(dict(zip(columns, values)))

    return LogData(log_num=log_num, path=path, params=params, columns=columns, rows=rows)


def pick(row: dict[str, str], logical_name: str, default: str = "") -> str:
    for name in ALIASES.get(logical_name, (logical_name,)):
        if name in row:
            return row[name]
    return default


def has_column(columns: Iterable[str], logical_name: str) -> bool:
    cols = set(columns)
    return any(name in cols for name in ALIASES.get(logical_name, (logical_name,)))


def to_float(value: str, default: float = 0.0) -> float:
    try:
        if value == "":
            return default
        result = float(value)
        return result if math.isfinite(result) else default
    except ValueError:
        return default


def to_int(value: str, default: int = 0) -> int:
    try:
        if value == "":
            return default
        return int(float(value))
    except ValueError:
        return default


def param_float(params: dict[str, str], key: str, default: float = 0.0) -> float:
    return to_float(params.get(key, ""), default)


def param_int(params: dict[str, str], key: str, default: int = 0) -> int:
    return to_int(params.get(key, ""), default)


def mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def pct(count: int, total: int) -> float:
    return (100.0 * count / total) if total else 0.0


def cnt_status(rows: list[dict[str, str]]) -> tuple[int, int, int]:
    prev: int | None = None
    nonmonotonic = 0
    min_step = 0
    max_step = 0
    for row in rows:
        cnt = to_int(row.get("cntlog", ""), 0)
        if prev is not None:
            step = cnt - prev
            if step <= 0:
                nonmonotonic += 1
            if min_step == 0 or step < min_step:
                min_step = step
            if step > max_step:
                max_step = step
        prev = cnt
    return nonmonotonic, min_step, max_step


def speed_values(row: dict[str, str]) -> tuple[float, float]:
    speed_text = pick(row, "speed_mps")
    if speed_text:
        speed = to_float(speed_text, 0.0)
        if abs(speed) > 20.0:
            speed = speed * 1000.0 / PULSE_METER
    else:
        speed = to_float(pick(row, "enc_current_n"), 0.0) * 1000.0 / PULSE_METER

    target = to_float(pick(row, "target_mps"), 0.0)
    if abs(target) > 20.0:
        target = target * 1000.0 / PULSE_METER
    return speed, target


def pwm_abs(row: dict[str, str]) -> float:
    return max(abs(to_float(pick(row, "motor_l"), 0.0)), abs(to_float(pick(row, "motor_r"), 0.0)))


def current_sum(row: dict[str, str]) -> float:
    return abs(to_float(pick(row, "current_l"), 0.0)) + abs(to_float(pick(row, "current_r"), 0.0))


def slip_isum_for_gate(row: dict[str, str]) -> float:
    value = pick(row, "slip_i_sum_f")
    return to_float(value, 0.0) if value != "" else current_sum(row)


def simulate_hysteresis(values: list[float], highs: list[float], lows: list[float],
                        high_req: int, low_req: int, enabled: list[bool] | None = None,
                        lowload_clear_req: int = 0) -> list[int]:
    flag = False
    high_count = 0
    low_count = 0
    lowload_clear_count = 0
    result: list[int] = []
    for i, value in enumerate(values):
        allowed = True if enabled is None else enabled[i]
        if not flag:
            low_count = 0
            lowload_clear_count = 0
            if allowed and value > highs[i]:
                high_count += 1
                if high_count >= high_req:
                    flag = True
                    high_count = 0
                    low_count = 0
            else:
                high_count = 0
        else:
            high_count = 0
            if value < lows[i]:
                low_count += 1
                if low_count >= low_req:
                    flag = False
                    low_count = 0
                    high_count = 0
            else:
                low_count = 0
            if flag and lowload_clear_req > 0:
                if not allowed:
                    lowload_clear_count += 1
                    if lowload_clear_count >= lowload_clear_req:
                        flag = False
                        low_count = 0
                        high_count = 0
                        lowload_clear_count = 0
                else:
                    lowload_clear_count = 0
            else:
                lowload_clear_count = 0
        result.append(1 if flag else 0)
    return result


def thresholds(log: LogData, args: argparse.Namespace) -> tuple[list[float], list[float], list[float], list[float]]:
    long_high: list[float] = []
    long_low: list[float] = []
    lat_high: list[float] = []
    lat_low: list[float] = []
    for row in log.rows:
        cur_scale = to_float(pick(row, "slip_cur_scale"), 1.0)
        long_high.append(args.long_high if args.long_high is not None
                         else to_float(pick(row, "slip_threshold_high"), 10.0 * cur_scale))
        long_low.append(args.long_low if args.long_low is not None
                        else to_float(pick(row, "slip_threshold_low"), 8.0 * cur_scale))
        lat_high.append(args.lat_high if args.lat_high is not None
                        else to_float(pick(row, "slip_lat_high"), 4.6 * cur_scale))
        lat_low.append(args.lat_low if args.lat_low is not None
                       else to_float(pick(row, "slip_lat_low"), 3.8 * cur_scale))
    return long_high, long_low, lat_high, lat_low


def long_on_enabled_flags(log: LogData, args: argparse.Namespace) -> list[bool]:
    result: list[bool] = []
    for row in log.rows:
        speed, target = speed_values(row)
        speed_err = abs(target - speed)
        i_sum = slip_isum_for_gate(row)
        low_load = (speed_err < args.long_lowload_speederr_max) and (i_sum < args.long_lowload_isum_max)
        result.append(not low_load)
    return result


def simulated_flags(log: LogData, args: argparse.Namespace) -> tuple[list[int], list[int]]:
    long_values = [to_float(pick(row, "slip_long"), 0.0) for row in log.rows]
    lat_values = [to_float(pick(row, "slip_lat"), 0.0) for row in log.rows]
    long_high, long_low, lat_high, lat_low = thresholds(log, args)
    long_enabled = long_on_enabled_flags(log, args)

    lat_enabled: list[bool] | None = None
    if has_column(log.columns, "slip_lat_on_count_enabled"):
        lat_enabled = [to_int(pick(row, "slip_lat_on_count_enabled"), 0) != 0 for row in log.rows]
    elif has_column(log.columns, "slip_lat_enabled"):
        lat_enabled = [to_int(pick(row, "slip_lat_enabled"), 0) != 0 for row in log.rows]

    if args.lat_encay_min is not None and has_column(log.columns, "slip_enc_ay_f"):
        enc_gate = [abs(to_float(pick(row, "slip_enc_ay_f"), 0.0)) > args.lat_encay_min for row in log.rows]
        lat_enabled = enc_gate if lat_enabled is None else [a and b for a, b in zip(lat_enabled, enc_gate)]

    sim_long = simulate_hysteresis(
        long_values,
        long_high,
        long_low,
        LONG_HIGH_COUNT_REQ,
        LONG_LOW_COUNT_REQ,
        long_enabled,
        args.long_lowload_clear_count,
    )
    sim_lat = simulate_hysteresis(lat_values, lat_high, lat_low, LAT_HIGH_COUNT_REQ, LAT_LOW_COUNT_REQ, lat_enabled)
    return sim_long, sim_lat


def summarize_log(log: LogData, args: argparse.Namespace) -> dict[str, object]:
    speeds: list[float] = []
    targets: list[float] = []
    speed_errs: list[float] = []
    long_values: list[float] = []
    lat_values: list[float] = []
    pwm_values: list[float] = []
    current_values: list[float] = []
    slip_long_count = 0
    slip_lat_count = 0
    long_on_count_enabled_count = 0
    long_lowload_clear_counts: list[int] = []
    lat_enabled_count = 0
    lat_on_count_enabled_count = 0
    long_enabled = long_on_enabled_flags(log, args)

    for i, row in enumerate(log.rows):
        speed, target = speed_values(row)
        speeds.append(speed)
        targets.append(target)
        speed_errs.append(abs(target - speed))
        long_values.append(to_float(pick(row, "slip_long"), 0.0))
        lat_values.append(to_float(pick(row, "slip_lat"), 0.0))
        pwm_values.append(pwm_abs(row))
        current_values.append(current_sum(row))
        slip_long_count += to_int(row.get("slipFlag", ""), 0) != 0
        slip_lat_count += to_int(row.get("slipFlagLat", ""), 0) != 0
        long_on_count_enabled_count += long_enabled[i]
        long_lowload_clear_counts.append(to_int(pick(row, "slip_long_lowload_clear_count"), 0))
        lat_enabled_count += to_int(pick(row, "slip_lat_enabled"), 0) != 0
        lat_on_count_enabled_count += to_int(pick(row, "slip_lat_on_count_enabled"), 0) != 0

    sim_long, sim_lat = simulated_flags(log, args)
    nonmono, min_step, max_step = cnt_status(log.rows)
    return {
        "log": log.log_num,
        "rows": len(log.rows),
        "fwVersion": log.params.get("fwVersion", ""),
        "gitCommit": log.params.get("gitCommit", ""),
        "buildTime": log.params.get("buildTime", ""),
        "batteryVoltage_V": param_float(log.params, "batteryVoltage_V", 0.0),
        "optimalTrace": param_int(log.params, "optimalTrace", 0),
        "autoStart": param_int(log.params, "autoStart", 0),
        "emcStop": param_int(log.params, "emcStop", 0),
        "lap_ms": param_int(log.params, "lap", 0),
        "distance_mm": param_int(log.params, "distance", 0),
        "speed_mean_mps": mean(speeds),
        "speed_max_mps": max(speeds) if speeds else 0.0,
        "target_max_mps": max(targets) if targets else 0.0,
        "speed_err_mean_mps": mean(speed_errs),
        "speed_err_max_mps": max(speed_errs) if speed_errs else 0.0,
        "slipLong_count": int(slip_long_count),
        "slipLong_pct": pct(int(slip_long_count), len(log.rows)),
        "slipLat_count": int(slip_lat_count),
        "slipLat_pct": pct(int(slip_lat_count), len(log.rows)),
        "simSlipLong_count": sum(sim_long),
        "simSlipLat_count": sum(sim_lat),
        "slipLongResidual_max_mps2": max(long_values) if long_values else 0.0,
        "slipLatResidual_max_mps2": max(lat_values) if lat_values else 0.0,
        "longOnCountEnabled_count": int(long_on_count_enabled_count),
        "longLowloadClearCount_max": max(long_lowload_clear_counts) if long_lowload_clear_counts else 0,
        "latEnabled_count": int(lat_enabled_count),
        "latOnCountEnabled_count": int(lat_on_count_enabled_count),
        "pwm_abs_max": max(pwm_values) if pwm_values else 0.0,
        "current_sum_max_A": max(current_values) if current_values else 0.0,
        "cnt_nonmonotonic": nonmono,
        "cnt_min_step_ms": min_step,
        "cnt_max_step_ms": max_step,
        "columns_new": int(has_column(log.columns, "slip_threshold_high")),
    }


def aggregate_by_index(logs: list[LogData], args: argparse.Namespace) -> list[dict[str, object]]:
    buckets: dict[int, list[tuple[LogData, int, dict[str, str], int, int, bool]]] = defaultdict(list)
    for log in logs:
        sim_long, sim_lat = simulated_flags(log, args)
        long_enabled = long_on_enabled_flags(log, args)
        for i, row in enumerate(log.rows):
            idx = to_int(row.get("optimalIndex", ""), -1)
            if idx >= 0:
                buckets[idx].append((log, i, row, sim_long[i], sim_lat[i], long_enabled[i]))

    result: list[dict[str, object]] = []
    for idx, items in sorted(buckets.items()):
        speeds: list[float] = []
        targets: list[float] = []
        errs: list[float] = []
        long_values: list[float] = []
        lat_values: list[float] = []
        pwms: list[float] = []
        currents: list[float] = []
        threshold_highs: list[float] = []
        threshold_lows: list[float] = []
        lat_highs: list[float] = []
        lat_lows: list[float] = []
        cur_scales: list[float] = []
        ax_biases: list[float] = []
        ay_biases: list[float] = []
        long_lowload_clear_counts: list[int] = []
        turning_count = 0
        long_on_count_enabled_count = 0
        lat_on_count_enabled_count = 0
        slip_long = 0
        slip_lat = 0
        sim_long_count = 0
        sim_lat_count = 0
        log_nums = set()

        for log, _row_num, row, sim_l, sim_y, long_enabled in items:
            log_nums.add(log.log_num)
            speed, target = speed_values(row)
            speeds.append(speed)
            targets.append(target)
            errs.append(abs(target - speed))
            long_values.append(to_float(pick(row, "slip_long"), 0.0))
            lat_values.append(to_float(pick(row, "slip_lat"), 0.0))
            pwms.append(pwm_abs(row))
            currents.append(current_sum(row))
            threshold_highs.append(to_float(pick(row, "slip_threshold_high"), 0.0))
            threshold_lows.append(to_float(pick(row, "slip_threshold_low"), 0.0))
            lat_highs.append(to_float(pick(row, "slip_lat_high"), 0.0))
            lat_lows.append(to_float(pick(row, "slip_lat_low"), 0.0))
            cur_scales.append(to_float(pick(row, "slip_cur_scale"), 0.0))
            ax_biases.append(to_float(pick(row, "slip_ax_bias"), 0.0))
            ay_biases.append(to_float(pick(row, "slip_ay_bias"), 0.0))
            long_lowload_clear_counts.append(to_int(pick(row, "slip_long_lowload_clear_count"), 0))
            turning_count += to_int(pick(row, "slip_turning"), 0) != 0
            long_on_count_enabled_count += long_enabled
            lat_on_count_enabled_count += to_int(pick(row, "slip_lat_on_count_enabled"), 0) != 0
            slip_long += to_int(row.get("slipFlag", ""), 0) != 0
            slip_lat += to_int(row.get("slipFlagLat", ""), 0) != 0
            sim_long_count += sim_l
            sim_lat_count += sim_y

        samples = len(items)
        result.append({
            "optimalIndex": idx,
            "logs": len(log_nums),
            "samples": samples,
            "slipLong_count": int(slip_long),
            "slipLat_count": int(slip_lat),
            "slip_total": int(slip_long + slip_lat),
            "slip_pct": pct(int(slip_long + slip_lat), samples),
            "simSlipLong_count": sim_long_count,
            "simSlipLat_count": sim_lat_count,
            "speed_mean_mps": mean(speeds),
            "target_mean_mps": mean(targets),
            "speed_err_mean_mps": mean(errs),
            "pwm_abs_max": max(pwms) if pwms else 0.0,
            "current_sum_max_A": max(currents) if currents else 0.0,
            "slipLongResidual_max_mps2": max(long_values) if long_values else 0.0,
            "slipLatResidual_max_mps2": max(lat_values) if lat_values else 0.0,
            "slipThresholdHigh_mean": mean(threshold_highs),
            "slipThresholdLow_mean": mean(threshold_lows),
            "slipLatHigh_mean": mean(lat_highs),
            "slipLatLow_mean": mean(lat_lows),
            "slipCurScale_mean": mean(cur_scales),
            "turning_pct": pct(int(turning_count), samples),
            "simLongOnCountEnabled_pct": pct(int(long_on_count_enabled_count), samples),
            "slipLongLowloadClearCount_max": max(long_lowload_clear_counts) if long_lowload_clear_counts else 0,
            "latOnCountEnabled_pct": pct(int(lat_on_count_enabled_count), samples),
            "slipAxBias_mean": mean(ax_biases),
            "slipAyBias_mean": mean(ay_biases),
        })
    return result


def label_candidates(by_index: list[dict[str, object]]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for row in by_index:
        if int(row["slip_total"]) == 0 and int(row["simSlipLong_count"]) == 0 and int(row["simSlipLat_count"]) == 0:
            continue

        long_count = int(row["slipLong_count"])
        lat_count = int(row["slipLat_count"])
        slip_pct_value = float(row["slip_pct"])
        pwm_max = float(row["pwm_abs_max"])
        current_max = float(row["current_sum_max_A"])
        err_mean = float(row["speed_err_mean_mps"])
        long_res = float(row["slipLongResidual_max_mps2"])
        lat_res = float(row["slipLatResidual_max_mps2"])

        label = "needs_review"
        if slip_pct_value >= 30.0 or pwm_max >= 900.0 or current_max >= 2.0:
            label = "true_slip_candidate"
        if slip_pct_value > 0.0 and err_mean < 0.10 and pwm_max < 700.0 and current_max < 1.0:
            label = "false_positive_candidate"
        if long_count == 0 and lat_count == 0:
            label = "threshold_sim_only"

        direction = "long+lat" if long_count and lat_count else "long" if long_count else "lat"
        rows.append({
            **row,
            "review_direction": direction,
            "review_label": label,
            "review_reason": f"err={err_mean:.3f}, pwm={pwm_max:.0f}, current={current_max:.2f}, longRes={long_res:.2f}, latRes={lat_res:.2f}",
            "needs_video_review": 1,
        })
    return rows


def compare_summaries(old_rows: list[dict[str, object]], new_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    def avg(key: str, rows: list[dict[str, object]]) -> float:
        return mean([float(row[key]) for row in rows]) if rows else 0.0

    keys = [
        "batteryVoltage_V",
        "lap_ms",
        "speed_err_mean_mps",
        "slipLong_count",
        "slipLat_count",
        "simSlipLong_count",
        "simSlipLat_count",
        "slipLongResidual_max_mps2",
        "slipLatResidual_max_mps2",
        "pwm_abs_max",
        "current_sum_max_A",
    ]
    result: list[dict[str, object]] = []
    for key in keys:
        old_val = avg(key, old_rows)
        new_val = avg(key, new_rows)
        result.append({
            "metric": key,
            "baseline_mean": old_val,
            "target_mean": new_val,
            "delta": new_val - old_val,
        })
    return result


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def load_logs(log_nums: list[int], log_root: Path) -> list[LogData]:
    logs: list[LogData] = []
    for log_num in log_nums:
        path = log_path(log_num, log_root)
        if not path.exists():
            raise FileNotFoundError(path)
        logs.append(read_log(path, log_num))
    return logs


def range_label(log_nums: list[int]) -> str:
    if not log_nums:
        return "none"
    if len(log_nums) > 1 and log_nums == list(range(log_nums[0], log_nums[-1] + 1)):
        return f"{log_nums[0]}_{log_nums[-1]}"
    return "_".join(str(num) for num in log_nums)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--logs", required=True, help="Log numbers, for example 9561-9570 or 9561,9563")
    parser.add_argument("--baseline", help="Optional baseline log numbers for comparison")
    parser.add_argument("--log-root", type=Path, default=LOG_ROOT)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument("--long-high", type=float, help="Override longitudinal ON threshold for offline simulation")
    parser.add_argument("--long-low", type=float, help="Override longitudinal OFF threshold for offline simulation")
    parser.add_argument("--long-lowload-speederr-max", type=float, default=0.12,
                        help="Suppress longitudinal ON count below this speed error [m/s]")
    parser.add_argument("--long-lowload-isum-max", type=float, default=0.35,
                        help="Suppress longitudinal ON count below this filtered current sum [A]")
    parser.add_argument("--long-lowload-clear-count", type=int, default=4,
                        help="Clear longitudinal slip after this many low-load samples while already ON")
    parser.add_argument("--lat-high", type=float, help="Override lateral ON threshold for offline simulation")
    parser.add_argument("--lat-low", type=float, help="Override lateral OFF threshold for offline simulation")
    parser.add_argument("--lat-encay-min", type=float, help="Override lateral encAy gate when slipEncAyF is present")
    args = parser.parse_args()

    log_nums = parse_log_spec(args.logs)
    logs = load_logs(log_nums, args.log_root)
    label = range_label(log_nums)

    summaries = [summarize_log(log, args) for log in logs]
    by_index = aggregate_by_index(logs, args)
    candidates = label_candidates(by_index)
    write_csv(args.out_dir / f"log_{label}_summary.csv", summaries)
    write_csv(args.out_dir / f"log_{label}_slip_by_index.csv", by_index)
    write_csv(args.out_dir / f"log_{label}_label_candidates.csv", candidates)

    if args.baseline:
        baseline_nums = parse_log_spec(args.baseline)
        baseline_logs = load_logs(baseline_nums, args.log_root)
        baseline_label = range_label(baseline_nums)
        baseline_summaries = [summarize_log(log, args) for log in baseline_logs]
        write_csv(args.out_dir / f"log_{baseline_label}_summary.csv", baseline_summaries)
        write_csv(args.out_dir / f"log_{baseline_label}_slip_by_index.csv", aggregate_by_index(baseline_logs, args))
        comparison = compare_summaries(baseline_summaries, summaries)
        write_csv(args.out_dir / f"log_{baseline_label}_vs_{label}_comparison.csv", comparison)

    invalid = [row["log"] for row in summaries if row["emcStop"] != 0 or row["cnt_nonmonotonic"] != 0]
    print(f"logs={label} rows={sum(int(row['rows']) for row in summaries)} candidates={len(candidates)} invalid={invalid}")
    print(f"summary={args.out_dir / f'log_{label}_summary.csv'}")
    print(f"by_index={args.out_dir / f'log_{label}_slip_by_index.csv'}")
    print(f"candidates={args.out_dir / f'log_{label}_label_candidates.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
