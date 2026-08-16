#!/usr/bin/env python3
"""Diagnose motor PWM saturation timing with yaw/line classification logs."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Callable


DEFAULT_LOG_DIR = Path(r"F:\Dropbox\Document\robotrace\Log\v2")
PWM_SAT_TH = 999.0
SPEED_REQ_SAT_TH = 1000.0
TARGET_SPEED_DELTA_TH = 2.0
LINE_TARGET_DELTA_TH = 30.0
YAW_ERROR_HIGH_DEG_S = 150.0
MOTOR_OUT_MISMATCH_TH = 100.0
STRAIGHT_ROC_TH = 2500.0
FOCUS_INDEXES = (10, 11, 23)
WATCH_INDEXES = (10, 11, 23, 24, 120, 124)
PWM_SAT_CLASS_LOG_PROFILE = 6
LINE_OMEGA_CAP_DEFAULT_DEG_S = 1200.0
CLASS_NAMES = (
    "motor_out_mismatch",
    "speed_profile_sat",
    "line_target_delta_sat",
    "yaw_error_sat",
    "speed_out_hold_sat",
    "unknown_motor_sat",
    "unclassified_motor_sat",
)


def parse_ranges(spec: str) -> list[int]:
    nums: list[int] = []
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            start_s, end_s = part.split("-", 1)
            start = int(start_s)
            end = int(end_s)
            step = 1 if end >= start else -1
            nums.extend(range(start, end + step, step))
        else:
            nums.append(int(part))
    return nums


def parse_float(value: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def mean(values: list[float]) -> float:
    vals = [v for v in values if not math.isnan(v)]
    return sum(vals) / len(vals) if vals else math.nan


def rms(values: list[float]) -> float:
    vals = [v for v in values if not math.isnan(v)]
    return math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else math.nan


def p95_abs(values: list[float]) -> float:
    vals = sorted(abs(v) for v in values if not math.isnan(v))
    if not vals:
        return math.nan
    return vals[min(len(vals) - 1, math.ceil(0.95 * len(vals)) - 1)]


def pct_true(values: list[bool]) -> float:
    return 100.0 * sum(1 for v in values if v) / len(values) if values else math.nan


class LogData:
    def __init__(self, number: int, path: Path) -> None:
        self.number = number
        self.path = path
        self.columns: list[str] = []
        self.params: dict[str, float | str] = {}
        self.rows: list[list[str]] = []
        self.idx: dict[str, int] = {}
        self._read()

    def _read(self) -> None:
        with self.path.open("r", encoding="utf-8-sig", newline="") as f:
            reader = csv.reader(f)
            header = next(reader)
            param_started = False
            for token in header:
                token = token.strip()
                if not token:
                    continue
                if "=" in token:
                    param_started = True
                    key, value = token.split("=", 1)
                    parsed = parse_float(value)
                    self.params[key] = value if math.isnan(parsed) else parsed
                elif not param_started:
                    self.columns.append(token)
            self.idx = {name: i for i, name in enumerate(self.columns)}
            self.rows = [row for row in reader if row]

    def param(self, name: str, default: float = math.nan) -> float:
        value = self.params.get(name, default)
        return value if isinstance(value, float) else default

    def value(self, row: list[str], column: str) -> float:
        i = self.idx.get(column)
        if i is None or i >= len(row):
            return math.nan
        return parse_float(row[i])

    def values(self, column: str) -> list[float]:
        return [self.value(row, column) for row in self.rows]

    def cnt_gaps(self) -> list[float]:
        cnt = self.values("cntlog")
        return [cnt[i] - cnt[i - 1] for i in range(1, len(cnt))]

    def cnt_monotonic(self) -> bool:
        return all(g > 0 for g in self.cnt_gaps())

    def cnt_gap_gt30_count(self) -> int:
        return sum(1 for g in self.cnt_gaps() if g > 30)

    def max_cnt_gap(self) -> float:
        gaps = self.cnt_gaps()
        return max(gaps) if gaps else math.nan

    def last_cnt(self) -> float:
        vals = self.values("cntlog")
        return vals[-1] if vals else math.nan

    def optmax(self) -> float:
        vals = [v for v in self.values("optimalIndex") if not math.isnan(v)]
        return max(vals) if vals else math.nan

    def has_sync_columns(self) -> bool:
        return {"targetAngularvelo", "speedPwmOutL", "speedPwmOutR"}.issubset(self.idx)

    def valid_reason(self, battery_min: float, require_sync_columns: bool = False) -> str:
        reasons: list[str] = []
        profile_param = self.param("pwmSatClassLogProfile", -1)
        profile = int(round(profile_param)) if not math.isnan(profile_param) else -1
        required = {
            "cntlog",
            "encCurrentN",
            "gyroVal_Z",
            "ROC",
            "targetSpeed",
            "optimalIndex",
            "slipFlagLat",
            "lineTraceCtrl",
            "motorpwmL",
            "motorpwmR",
        }
        if require_sync_columns:
            required.update({"targetAngularvelo", "speedPwmOutL", "speedPwmOutR"})
            if profile == PWM_SAT_CLASS_LOG_PROFILE:
                required.add("targetAngularveloRaw")
        missing = sorted(required.difference(self.idx))
        if missing:
            reasons.append("missing=" + "|".join(missing))
        if int(round(self.param("optimalTrace"))) != 2:
            reasons.append("optimalTrace!=2")
        if int(round(self.param("emcStop"))) != 0:
            reasons.append(f"emcStop={self.param('emcStop'):.0f}")
        if not self.cnt_monotonic():
            reasons.append("cntlog_not_monotonic")
        if self.cnt_gap_gt30_count() > 0:
            reasons.append("cntlog_gap_gt30")
        if self.param("batteryVoltage_V") < battery_min:
            reasons.append(f"battery_lt_{battery_min:g}")
        if len(self.rows) < 850:
            reasons.append("rows_lt_850")
        if self.optmax() < 180:
            reasons.append("optimalIndex_max_lt_180")
        if int(round(self.param("logRecordSizeBytes", -1))) != 36:
            reasons.append(f"logRecordSizeBytes={self.param('logRecordSizeBytes'):.0f}")
        if int(round(self.param("dbgOverflowFinal", -1))) != 0:
            reasons.append("dbgOverflowFinal!=0")
        if int(round(self.param("logOverflowFinal", -1))) != 0:
            reasons.append("logOverflowFinal!=0")
        if require_sync_columns and profile != PWM_SAT_CLASS_LOG_PROFILE:
            reasons.append(f"pwmSatClassLogProfile={self.param('pwmSatClassLogProfile'):.0f}")
        return "valid" if not reasons else ";".join(reasons)

    def is_valid(self, battery_min: float, require_sync_columns: bool = False) -> bool:
        return self.valid_reason(battery_min, require_sync_columns) == "valid"


def load_logs(numbers: list[int], log_dir: Path) -> list[LogData]:
    return [LogData(number, log_dir / f"{number}.csv") for number in numbers if (log_dir / f"{number}.csv").exists()]


def target_speed_scale(log: LogData, row: list[str]) -> float:
    base = log.value(row, "targetSpeedBase")
    target = log.value(row, "targetSpeed")
    if not math.isnan(base) and base > 0.0 and not math.isnan(target):
        return target * 100.0 / base
    return log.value(row, "targetSpeedScale")


def profile_accel_decel(log: LogData, row_index: int) -> bool:
    row = log.rows[row_index]
    target = log.value(row, "targetSpeed")
    if math.isnan(target):
        return False
    prev_delta = 0.0
    next_delta = 0.0
    if row_index > 0:
        prev = log.value(log.rows[row_index - 1], "targetSpeed")
        if not math.isnan(prev):
            prev_delta = abs(target - prev)
    if row_index + 1 < len(log.rows):
        nxt = log.value(log.rows[row_index + 1], "targetSpeed")
        if not math.isnan(nxt):
            next_delta = abs(nxt - target)
    return max(prev_delta, next_delta) >= TARGET_SPEED_DELTA_TH


def side_suffix(side: str) -> str:
    return "L" if side == "L" else "R"


def side_motor_sat(log: LogData, row: list[str], side: str) -> bool:
    suffix = side_suffix(side)
    return abs(log.value(row, f"motorpwm{suffix}")) >= PWM_SAT_TH


def side_speed_out_sat(log: LogData, row: list[str], side: str) -> bool:
    suffix = side_suffix(side)
    return abs(log.value(row, f"speedPwmOut{suffix}")) >= PWM_SAT_TH


def side_motor_out_mismatch(log: LogData, row: list[str], side: str) -> bool:
    suffix = side_suffix(side)
    motor = log.value(row, f"motorpwm{suffix}")
    out = log.value(row, f"speedPwmOut{suffix}")
    return not math.isnan(motor) and not math.isnan(out) and abs(motor - out) > MOTOR_OUT_MISMATCH_TH


def classify_pwm_sat_side(log: LogData, row: list[str], row_index: int, side: str) -> str:
    if not side_motor_sat(log, row, side):
        return "no_motor_sat"
    if not log.has_sync_columns():
        return "unclassified_motor_sat"

    speed_out_sat = side_speed_out_sat(log, row, side)
    accel_decel = profile_accel_decel(log, row_index)
    line_delta = abs(log.value(row, "lineTraceCtrl")) >= LINE_TARGET_DELTA_TH
    yaw_error = abs(log.value(row, "targetAngularvelo") - log.value(row, "gyroVal_Z")) >= YAW_ERROR_HIGH_DEG_S

    if side_motor_out_mismatch(log, row, side):
        return "motor_out_mismatch"
    if speed_out_sat and accel_decel:
        return "speed_profile_sat"
    if speed_out_sat and line_delta:
        return "line_target_delta_sat"
    if speed_out_sat and yaw_error:
        return "yaw_error_sat"
    if speed_out_sat:
        return "speed_out_hold_sat"
    return "unknown_motor_sat"


def classify_pwm_sat_any(log: LogData, row: list[str], row_index: int) -> str:
    left = classify_pwm_sat_side(log, row, row_index, "L")
    right = classify_pwm_sat_side(log, row, row_index, "R")
    priority = (
        "motor_out_mismatch",
        "speed_profile_sat",
        "line_target_delta_sat",
        "yaw_error_sat",
        "speed_out_hold_sat",
        "unknown_motor_sat",
        "unclassified_motor_sat",
        "no_motor_sat",
    )
    for item in priority:
        if left == item or right == item:
            return item
    return "no_motor_sat"


def row_pool(logs: list[LogData], battery_min: float, require_sync_columns: bool = False) -> list[tuple[LogData, int, list[str]]]:
    out: list[tuple[LogData, int, list[str]]] = []
    for log in logs:
        if not log.is_valid(battery_min, require_sync_columns):
            continue
        out.extend((log, i, row) for i, row in enumerate(log.rows))
    return out


def filter_pool(
    pool: list[tuple[LogData, int, list[str]]],
    pred: Callable[[LogData, int, list[str]], bool],
) -> list[tuple[LogData, int, list[str]]]:
    return [(log, i, row) for log, i, row in pool if pred(log, i, row)]


def vals(pool: list[tuple[LogData, int, list[str]]], column: str) -> list[float]:
    return [log.value(row, column) for log, _, row in pool if column in log.idx]


def class_pct(pool: list[tuple[LogData, int, list[str]]], class_name: str) -> float:
    classified = [classify_pwm_sat_any(log, row, i) for log, i, row in pool]
    return pct_true([item == class_name for item in classified])


def side_class_pct(pool: list[tuple[LogData, int, list[str]]], side: str, class_name: str) -> float:
    classified = [classify_pwm_sat_side(log, row, i, side) for log, i, row in pool]
    return pct_true([item == class_name for item in classified])


def motor_sat_pct(pool: list[tuple[LogData, int, list[str]]]) -> float:
    return pct_true([
        side_motor_sat(log, row, "L") or side_motor_sat(log, row, "R")
        for log, _, row in pool
    ])


def side_motor_sat_pct(pool: list[tuple[LogData, int, list[str]]], side: str) -> float:
    return pct_true([side_motor_sat(log, row, side) for log, _, row in pool])


def side_mismatch_pct(pool: list[tuple[LogData, int, list[str]]], side: str) -> float:
    return pct_true([side_motor_out_mismatch(log, row, side) for log, _, row in pool])


def cap_rows(pool: list[tuple[LogData, int, list[str]]]) -> list[tuple[LogData, int, list[str]]]:
    return [
        (log, i, row)
        for log, i, row in pool
        if "targetAngularveloRaw" in log.idx and "targetAngularvelo" in log.idx
    ]


def line_omega_cap_deg_s(log: LogData) -> float:
    cap = log.param("lineOmegaTargetCapDegS", LINE_OMEGA_CAP_DEFAULT_DEG_S)
    return cap if not math.isnan(cap) else LINE_OMEGA_CAP_DEFAULT_DEG_S


def cap_applied(log: LogData, row: list[str]) -> bool:
    raw = log.value(row, "targetAngularveloRaw")
    capped = log.value(row, "targetAngularvelo")
    cap = line_omega_cap_deg_s(log)
    return not math.isnan(raw) and not math.isnan(capped) and abs(raw) > cap and abs(capped) <= cap


def cap_missed_due_speed(log: LogData, row: list[str]) -> bool:
    raw = log.value(row, "targetAngularveloRaw")
    capped = log.value(row, "targetAngularvelo")
    cap = line_omega_cap_deg_s(log)
    return not math.isnan(raw) and not math.isnan(capped) and abs(raw) > cap and abs(capped) > cap


def cap_delta_abs(log: LogData, row: list[str]) -> float:
    raw = log.value(row, "targetAngularveloRaw")
    capped = log.value(row, "targetAngularvelo")
    if math.isnan(raw) or math.isnan(capped):
        return math.nan
    return abs(raw - capped)


def sync_available(logs: list[LogData], battery_min: float) -> bool:
    valid = [log for log in logs if log.is_valid(battery_min)]
    return bool(valid) and all(log.has_sync_columns() for log in valid)


def group_metrics(group: str, logs: list[LogData], battery_min: float, require_sync_columns: bool = False) -> dict[str, object]:
    valid_logs = [log for log in logs if log.is_valid(battery_min, require_sync_columns)]
    pool = row_pool(logs, battery_min, require_sync_columns)
    moving = filter_pool(pool, lambda log, _, row: log.value(row, "targetSpeed") > 0 and log.value(row, "cntlog") >= 100)
    straight = filter_pool(moving, lambda log, _, row: abs(log.value(row, "ROC")) >= STRAIGHT_ROC_TH)
    curve = filter_pool(moving, lambda log, _, row: abs(log.value(row, "ROC")) < STRAIGHT_ROC_TH)
    focus = filter_pool(pool, lambda log, _, row: int(log.value(row, "optimalIndex")) in FOCUS_INDEXES)
    cap_moving = cap_rows(moving)
    cap_focus = cap_rows(focus)
    has_sync = sync_available(valid_logs, battery_min)
    yaw_errors = [
        log.value(row, "targetAngularvelo") - log.value(row, "gyroVal_Z")
        for log, _, row in moving
        if "targetAngularvelo" in log.idx
    ]

    def cls(name: str) -> float:
        return class_pct(moving, name) if has_sync else math.nan

    data: dict[str, object] = {
        "group": group,
        "battery_min": battery_min,
        "valid_logs": " ".join(str(log.number) for log in valid_logs),
        "n_valid": len(valid_logs),
        "sync_columns_available": int(has_sync),
        "batteryVoltage_V_mean": mean([log.param("batteryVoltage_V") for log in valid_logs]),
        "lap_ms_mean": mean([log.last_cnt() for log in valid_logs]),
        "rows_mean": mean([float(len(log.rows)) for log in valid_logs]),
        "motor_sat_pct": motor_sat_pct(moving),
        "motor_sat_L_pct": side_motor_sat_pct(moving, "L"),
        "motor_sat_R_pct": side_motor_sat_pct(moving, "R"),
        "motor_out_mismatch_L_pct": side_mismatch_pct(moving, "L") if has_sync else math.nan,
        "motor_out_mismatch_R_pct": side_mismatch_pct(moving, "R") if has_sync else math.nan,
        "straight_gyro_rms": rms(vals(straight, "gyroVal_Z")),
        "straight_lineTrace_rms": rms(vals(straight, "lineTraceCtrl")),
        "target_curve_lineTrace_rms": rms(vals(curve, "lineTraceCtrl")),
        "target_curve_motor_sat_pct": motor_sat_pct(curve),
        "focus_10_11_23_scale_mean": mean([target_speed_scale(log, row) for log, _, row in focus]),
        "focus_10_11_23_slipLat_pct": mean(vals(focus, "slipFlagLat")) * 100.0,
        "focus_10_11_23_lineTrace_rms": rms(vals(focus, "lineTraceCtrl")),
        "focus_10_11_23_motor_sat_pct": motor_sat_pct(focus),
        "speedPwmReqL_abs_rms": rms([abs(v) for v in vals(moving, "speedPwmReqL")]),
        "speedPwmReqR_abs_rms": rms([abs(v) for v in vals(moving, "speedPwmReqR")]),
        "speedPwmOutL_abs_rms": rms([abs(v) for v in vals(moving, "speedPwmOutL")]),
        "speedPwmOutR_abs_rms": rms([abs(v) for v in vals(moving, "speedPwmOutR")]),
        "yaw_error_abs_rms": rms([abs(v) for v in yaw_errors]),
        "yaw_error_abs_p95": p95_abs(yaw_errors),
        "cap_sample_count": len(cap_moving),
        "cap_applied_pct": pct_true([cap_applied(log, row) for log, _, row in cap_moving]),
        "cap_missed_due_speed_pct": pct_true([cap_missed_due_speed(log, row) for log, _, row in cap_moving]),
        "cap_delta_abs_mean": mean([cap_delta_abs(log, row) for log, _, row in cap_moving]),
        "cap_delta_abs_p95": p95_abs([cap_delta_abs(log, row) for log, _, row in cap_moving]),
        "focus_10_11_23_cap_applied_pct": pct_true([cap_applied(log, row) for log, _, row in cap_focus]),
        "focus_10_11_23_cap_missed_due_speed_pct": pct_true([cap_missed_due_speed(log, row) for log, _, row in cap_focus]),
        "focus_10_11_23_cap_delta_abs_mean": mean([cap_delta_abs(log, row) for log, _, row in cap_focus]),
        "focus_10_11_23_cap_delta_abs_p95": p95_abs([cap_delta_abs(log, row) for log, _, row in cap_focus]),
    }
    for name in CLASS_NAMES:
        data[f"{name}_pct"] = cls(name)
        data[f"{name}_L_pct"] = side_class_pct(moving, "L", name) if has_sync else math.nan
        data[f"{name}_R_pct"] = side_class_pct(moving, "R", name) if has_sync else math.nan
    return data


def summary_rows(group: str, logs: list[LogData], battery_min: float) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for log in logs:
        out.append({
            "group": group,
            "log": log.number,
            "valid_reason": log.valid_reason(battery_min, True),
            "optimalTrace": log.param("optimalTrace"),
            "autoStart": log.param("autoStart"),
            "emcStop": log.param("emcStop"),
            "batteryVoltage_V": log.param("batteryVoltage_V"),
            "logRecordSizeBytes": log.param("logRecordSizeBytes"),
            "dbgOverflowFinal": log.param("dbgOverflowFinal"),
            "logOverflowFinal": log.param("logOverflowFinal"),
            "rows": len(log.rows),
            "last_cnt_ms": log.last_cnt(),
            "max_cnt_gap_ms": log.max_cnt_gap(),
            "cnt_gap_gt30_count": log.cnt_gap_gt30_count(),
            "optimalIndex_max": log.optmax(),
            "slipLocalLatEnable": log.param("slipLocalLatEnable"),
            "pwmSatClassLogProfile": log.param("pwmSatClassLogProfile"),
            "pwmSatYawErrorHighDegS": log.param("pwmSatYawErrorHighDegS"),
            "lineOmegaTargetCapEnable": log.param("lineOmegaTargetCapEnable"),
            "lineOmegaTargetCapDegS": log.param("lineOmegaTargetCapDegS"),
            "lineOmegaTargetCapMaxSpeedPulse": log.param("lineOmegaTargetCapMaxSpeedPulse"),
            "has_speedPwmReqL": int("speedPwmReqL" in log.idx),
            "has_speedPwmReqR": int("speedPwmReqR" in log.idx),
            "has_speedPwmOutL": int("speedPwmOutL" in log.idx),
            "has_speedPwmOutR": int("speedPwmOutR" in log.idx),
            "has_targetAngularvelo": int("targetAngularvelo" in log.idx),
            "has_targetAngularveloRaw": int("targetAngularveloRaw" in log.idx),
            "has_speedPwmSatL": int("speedPwmSatL" in log.idx),
            "has_speedPwmSatR": int("speedPwmSatR" in log.idx),
            "has_targetSpeedBase": int("targetSpeedBase" in log.idx),
        })
    return out


def by_index_rows(group: str, logs: list[LogData], battery_min: float, require_sync_columns: bool = False) -> list[dict[str, object]]:
    pool = row_pool(logs, battery_min, require_sync_columns)
    has_sync = sync_available([log for log in logs if log.is_valid(battery_min, require_sync_columns)], battery_min)
    out: list[dict[str, object]] = []
    for idx in WATCH_INDEXES:
        rows = filter_pool(pool, lambda log, _, row, idx=idx: int(log.value(row, "optimalIndex")) == idx)
        cap_index_rows = cap_rows(rows)
        yaw_errors = [
            log.value(row, "targetAngularvelo") - log.value(row, "gyroVal_Z")
            for log, _, row in rows
            if "targetAngularvelo" in log.idx
        ]
        data: dict[str, object] = {
            "group": group,
            "optimalIndex": idx,
            "samples": len(rows),
            "targetSpeed_mean": mean(vals(rows, "targetSpeed")),
            "targetSpeedBase_mean": mean(vals(rows, "targetSpeedBase")),
            "targetSpeedScale_calc_mean": mean([target_speed_scale(log, row) for log, _, row in rows]),
            "encCurrentN_mean": mean(vals(rows, "encCurrentN")),
            "motor_sat_pct": motor_sat_pct(rows),
            "motor_sat_L_pct": side_motor_sat_pct(rows, "L"),
            "motor_sat_R_pct": side_motor_sat_pct(rows, "R"),
            "motor_out_mismatch_L_pct": side_mismatch_pct(rows, "L") if has_sync else math.nan,
            "motor_out_mismatch_R_pct": side_mismatch_pct(rows, "R") if has_sync else math.nan,
            "lineTrace_rms": rms(vals(rows, "lineTraceCtrl")),
            "gyro_rms": rms(vals(rows, "gyroVal_Z")),
            "slipFlagLat_pct": mean(vals(rows, "slipFlagLat")) * 100.0,
            "speedPwmReqL_abs_rms": rms([abs(v) for v in vals(rows, "speedPwmReqL")]),
            "speedPwmReqR_abs_rms": rms([abs(v) for v in vals(rows, "speedPwmReqR")]),
            "speedPwmOutL_abs_rms": rms([abs(v) for v in vals(rows, "speedPwmOutL")]),
            "speedPwmOutR_abs_rms": rms([abs(v) for v in vals(rows, "speedPwmOutR")]),
            "yaw_error_abs_rms": rms([abs(v) for v in yaw_errors]),
            "yaw_error_abs_p95": p95_abs(yaw_errors),
            "cap_sample_count": len(cap_index_rows),
            "cap_applied_pct": pct_true([cap_applied(log, row) for log, _, row in cap_index_rows]),
            "cap_missed_due_speed_pct": pct_true([cap_missed_due_speed(log, row) for log, _, row in cap_index_rows]),
            "cap_delta_abs_mean": mean([cap_delta_abs(log, row) for log, _, row in cap_index_rows]),
            "cap_delta_abs_p95": p95_abs([cap_delta_abs(log, row) for log, _, row in cap_index_rows]),
        }
        for name in CLASS_NAMES:
            data[f"{name}_pct"] = class_pct(rows, name) if has_sync else math.nan
            data[f"{name}_L_pct"] = side_class_pct(rows, "L", name) if has_sync else math.nan
            data[f"{name}_R_pct"] = side_class_pct(rows, "R", name) if has_sync else math.nan
        out.append(data)
    return out


def set_rows(group: str, logs: list[LogData], battery_min: float, require_sync_columns: bool = False) -> list[dict[str, object]]:
    sets: list[list[LogData]] = []
    current: list[LogData] = []
    for log in sorted(logs, key=lambda item: item.number):
        auto = int(round(log.param("autoStart")))
        if auto <= 1:
            if current:
                sets.append(current)
                current = []
            continue
        if auto == 2 and current:
            sets.append(current)
            current = []
        current.append(log)
    if current:
        sets.append(current)

    out: list[dict[str, object]] = []
    for idx, set_logs in enumerate(sets, start=1):
        valid = [log for log in set_logs if log.is_valid(battery_min, require_sync_columns)]
        excluded = [
            f"{log.number}:{log.valid_reason(battery_min, require_sync_columns)}"
            for log in set_logs
            if not log.is_valid(battery_min, require_sync_columns)
        ]
        metrics = group_metrics(f"{group}_set{idx}", valid, battery_min, require_sync_columns)
        row = {
            "group": f"{group}_set{idx}",
            "logs": " ".join(str(log.number) for log in set_logs),
            "valid_logs": " ".join(str(log.number) for log in valid),
            "excluded_logs": " | ".join(excluded),
            "n_valid": len(valid),
            "complete_4_second_runs": int(len(valid) == 4),
            "batteryVoltage_V_mean": metrics["batteryVoltage_V_mean"],
            "lap_ms_mean": metrics["lap_ms_mean"],
            "motor_sat_pct": metrics["motor_sat_pct"],
            "motor_out_mismatch_L_pct": metrics["motor_out_mismatch_L_pct"],
            "motor_out_mismatch_R_pct": metrics["motor_out_mismatch_R_pct"],
            "straight_gyro_rms": metrics["straight_gyro_rms"],
            "straight_lineTrace_rms": metrics["straight_lineTrace_rms"],
            "target_curve_lineTrace_rms": metrics["target_curve_lineTrace_rms"],
            "focus_10_11_23_scale_mean": metrics["focus_10_11_23_scale_mean"],
            "focus_10_11_23_slipLat_pct": metrics["focus_10_11_23_slipLat_pct"],
            "focus_10_11_23_lineTrace_rms": metrics["focus_10_11_23_lineTrace_rms"],
            "yaw_error_abs_rms": metrics["yaw_error_abs_rms"],
            "yaw_error_abs_p95": metrics["yaw_error_abs_p95"],
            "cap_applied_pct": metrics["cap_applied_pct"],
            "cap_missed_due_speed_pct": metrics["cap_missed_due_speed_pct"],
            "cap_delta_abs_mean": metrics["cap_delta_abs_mean"],
            "cap_delta_abs_p95": metrics["cap_delta_abs_p95"],
            "focus_10_11_23_cap_applied_pct": metrics["focus_10_11_23_cap_applied_pct"],
            "focus_10_11_23_cap_missed_due_speed_pct": metrics["focus_10_11_23_cap_missed_due_speed_pct"],
        }
        for name in CLASS_NAMES:
            row[f"{name}_pct"] = metrics[f"{name}_pct"]
        out.append(row)
    return out


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def range_label(numbers: list[int]) -> str:
    return f"{min(numbers)}_{max(numbers)}" if numbers else "none"


def combined_label(*number_lists: list[int]) -> str:
    nums = [num for nums in number_lists for num in nums]
    return range_label(nums)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--baseline", default="10747,10748,10749,10750,10755,10756,10757,10758")
    parser.add_argument("--max90-reference", default="10782,10783,10784,10785,10787,10788,10789,10790")
    parser.add_argument("--max95-reference", default="10792-10818")
    parser.add_argument("--new", required=True)
    parser.add_argument("--battery-min", type=float, default=7.3)
    parser.add_argument("--out-dir", type=Path, default=Path("analysis"))
    args = parser.parse_args()

    base_nums = parse_ranges(args.baseline)
    max90_nums = parse_ranges(args.max90_reference)
    max95_nums = parse_ranges(args.max95_reference)
    new_nums = parse_ranges(args.new)
    base_logs = load_logs(base_nums, args.log_dir)
    max90_logs = load_logs(max90_nums, args.log_dir)
    max95_logs = load_logs(max95_nums, args.log_dir)
    new_logs = load_logs(new_nums, args.log_dir)
    new_label = range_label(new_nums)
    compare_label = combined_label(base_nums, max90_nums, max95_nums)

    summary = summary_rows("capmax90_rawtarget", new_logs, args.battery_min)
    comparison = [
        group_metrics("capless_profile5_reference", base_logs, args.battery_min, False),
        group_metrics("max90_reference", max90_logs, args.battery_min, False),
        group_metrics("max95_reference", max95_logs, args.battery_min, False),
        group_metrics("capmax90_rawtarget", new_logs, args.battery_min, True),
        group_metrics("capmax90_rawtarget_bat76_reference", new_logs, 7.6, True),
    ]
    by_index = (
        by_index_rows("capless_profile5_reference", base_logs, args.battery_min, False)
        + by_index_rows("max90_reference", max90_logs, args.battery_min, False)
        + by_index_rows("max95_reference", max95_logs, args.battery_min, False)
        + by_index_rows("capmax90_rawtarget", new_logs, args.battery_min, True)
        + by_index_rows("capmax90_rawtarget_bat76_reference", new_logs, 7.6, True)
    )
    sets = (
        set_rows("capless_profile5_reference", base_logs, args.battery_min, False)
        + set_rows("max90_reference", max90_logs, args.battery_min, False)
        + set_rows("max95_reference", max95_logs, args.battery_min, False)
        + set_rows("capmax90_rawtarget", new_logs, args.battery_min, True)
        + set_rows("capmax90_rawtarget_bat76_reference", new_logs, 7.6, True)
    )

    write_csv(args.out_dir / f"log_{new_label}_capmax90_rawtarget_summary.csv", summary)
    write_csv(args.out_dir / f"log_{compare_label}_vs_{new_label}_capmax90_rawtarget_comparison.csv", comparison)
    write_csv(args.out_dir / f"log_{new_label}_capmax90_rawtarget_by_index.csv", by_index)
    write_csv(args.out_dir / f"log_{new_label}_capmax90_rawtarget_sets.csv", sets)
    print(f"wrote PWM saturation raw target report for {new_label}")


if __name__ == "__main__":
    main()
