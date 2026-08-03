#!/usr/bin/env python3
"""Summarize line-omega sensor components for BOOST_DISTANCE logs."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


DEFAULT_LOG_DIR = Path(r"F:\Dropbox\Document\robotrace\Log\v2")
STRAIGHT_ROC_TH = 2500.0
MOTOR_SAT_TH = 999.0
LINE_OMEGA_CAP_DEFAULT_DEG_S = 1200.0
PWM_SAT_CLASS_LOG_PROFILE = 7
LINE_OMEGA_TARGET_CAP_MAX_SPEED_DEFAULT = 90.0
PULSE_MILLIMETER = 54.324
RAD2DEG = 180.0 / math.pi
LOCAL_CAP_DEFAULT_ENABLE = 0
LOCAL_CAP_DEFAULT_DEG_S = 1200.0
LOCAL_CAP_DEFAULT_INDEXES = (10, 11, 23)
LOCAL_SCALE_DEFAULT_ENABLE = 0
LOCAL_SCALE_DEFAULT = 0.90
LOCAL_SCALE_DEFAULT_PCT = 90
LOCAL_SCALE_DEFAULT_MAX_SPEED = 95.0
LOCAL_SCALE_DEFAULT_INDEXES = (10, 11, 23, 24)
DIFF_COMPRESS_DEFAULT_ENABLE = 0
DIFF_COMPRESS_DEFAULT_START_PERMILLE = 850.0
DIFF_COMPRESS_DEFAULT_FULL_PERMILLE = 950.0
DIFF_COMPRESS_DEFAULT_MIN_SCALE_PCT = 90.0
DIFF_COMPRESS_DEFAULT_MAX_SPEED = 95.0
DIFF_COMPRESS_DEFAULT_SUM_MIN = 1000.0
CURVE_FF_DEFAULT_ENABLE = 1
CURVE_FF_DEFAULT_ROC_MAX_MM = 200.0
CURVE_FF_DEFAULT_ROC_MIN_MM = 80.0
CURVE_FF_DEFAULT_MAX_SPEED = 95.0
CURVE_FF_DEFAULT_GAIN_PCT = 100
CURVE_FF_DEFAULT_FB_SCALE_PCT = 35
FOCUS_INDEXES = (10, 11, 23, 24)
WATCH_INDEXES = (10, 11, 23, 24, 120, 124)


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


def parse_float(value: str | None) -> float:
    try:
        return float(value) if value is not None else math.nan
    except ValueError:
        return math.nan


def mean(values: list[float]) -> float:
    vals = [v for v in values if not math.isnan(v)]
    return sum(vals) / len(vals) if vals else math.nan


def rms(values: list[float]) -> float:
    vals = [v for v in values if not math.isnan(v)]
    return math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else math.nan


def p05(values: list[float]) -> float:
    vals = sorted(v for v in values if not math.isnan(v))
    if not vals:
        return math.nan
    return vals[min(len(vals) - 1, math.ceil(0.05 * len(vals)) - 1)]


def p95(values: list[float]) -> float:
    vals = sorted(v for v in values if not math.isnan(v))
    if not vals:
        return math.nan
    return vals[min(len(vals) - 1, math.ceil(0.95 * len(vals)) - 1)]


def p95_abs(values: list[float]) -> float:
    return p95([abs(v) for v in values if not math.isnan(v)])


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
        self.missing = not path.exists()
        if not self.missing:
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

    def param_int(self, name: str, default: int = -1) -> int:
        value = self.param(name)
        return int(round(value)) if not math.isnan(value) else default

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
        return sum(1 for g in self.cnt_gaps() if g > 30.0)

    def max_cnt_gap(self) -> float:
        gaps = self.cnt_gaps()
        return max(gaps) if gaps else math.nan

    def last_cnt(self) -> float:
        vals = self.values("cntlog")
        return vals[-1] if vals else math.nan

    def optmax(self) -> float:
        vals = [v for v in self.values("optimalIndex") if not math.isnan(v)]
        return max(vals) if vals else math.nan

    def has_sensor_diag_columns(self) -> bool:
        return {"lineOmegaSensorDiff", "lineOmegaSensorSum", "targetAngularveloRaw"}.issubset(self.idx)

    def valid_reason(
        self,
        battery_min: float,
        require_sensor_diag: bool = False,
        expected_local_cap_enable: int = LOCAL_CAP_DEFAULT_ENABLE,
        expected_local_scale_enable: int = -1,
        expected_diff_compress_enable: int = -1,
        expected_curve_ff_enable: int = -1,
    ) -> str:
        if self.missing:
            return "missing_file"
        reasons: list[str] = []
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
            "targetAngularvelo",
        }
        if require_sensor_diag:
            required.update({"targetAngularveloRaw", "lineOmegaSensorDiff", "lineOmegaSensorSum"})
        missing = sorted(required.difference(self.idx))
        if missing:
            reasons.append("missing=" + "|".join(missing))
        if self.param_int("optimalTrace") != 2:
            reasons.append("optimalTrace!=2")
        if self.param_int("emcStop") != 0:
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
        if self.param_int("logRecordSizeBytes") != 36:
            reasons.append(f"logRecordSizeBytes={self.param('logRecordSizeBytes'):.0f}")
        if self.param_int("dbgOverflowFinal") != 0:
            reasons.append("dbgOverflowFinal!=0")
        if self.param_int("logOverflowFinal") != 0:
            reasons.append("logOverflowFinal!=0")
        profile_param = self.param("pwmSatClassLogProfile", -1)
        if require_sensor_diag and self.param_int("pwmSatClassLogProfile") != PWM_SAT_CLASS_LOG_PROFILE:
            reasons.append(f"pwmSatClassLogProfile={profile_param:.0f}")
        if require_sensor_diag:
            if self.param_int("lineOmegaSensorDiagProfile") != 1:
                reasons.append("lineOmegaSensorDiagProfile!=1")
            if expected_local_cap_enable >= 0:
                local_indexes = (
                    self.param_int("lineOmegaLocalTargetCapIndex0"),
                    self.param_int("lineOmegaLocalTargetCapIndex1"),
                    self.param_int("lineOmegaLocalTargetCapIndex2"),
                )
                if self.param_int("lineOmegaLocalTargetCapEnable") != expected_local_cap_enable:
                    reasons.append(f"lineOmegaLocalTargetCapEnable!={expected_local_cap_enable}")
                if self.param_int("lineOmegaLocalTargetCapDegS") != int(LOCAL_CAP_DEFAULT_DEG_S):
                    reasons.append("lineOmegaLocalTargetCapDegS!=1200")
                if local_indexes != LOCAL_CAP_DEFAULT_INDEXES:
                    reasons.append("lineOmegaLocalTargetCapIndex!=10_11_23")
            if expected_local_scale_enable >= 0:
                scale_indexes = (
                    self.param_int("lineOmegaLocalTargetScaleIndex0"),
                    self.param_int("lineOmegaLocalTargetScaleIndex1"),
                    self.param_int("lineOmegaLocalTargetScaleIndex2"),
                    self.param_int("lineOmegaLocalTargetScaleIndex3"),
                )
                if self.param_int("lineOmegaLocalTargetScaleEnable") != expected_local_scale_enable:
                    reasons.append(f"lineOmegaLocalTargetScaleEnable!={expected_local_scale_enable}")
                if self.param_int("lineOmegaLocalTargetScalePct") != LOCAL_SCALE_DEFAULT_PCT:
                    reasons.append("lineOmegaLocalTargetScalePct!=90")
                if self.param_int("lineOmegaLocalTargetScaleMaxSpeedPulse") != int(LOCAL_SCALE_DEFAULT_MAX_SPEED):
                    reasons.append("lineOmegaLocalTargetScaleMaxSpeedPulse!=95")
                if scale_indexes != LOCAL_SCALE_DEFAULT_INDEXES:
                    reasons.append("lineOmegaLocalTargetScaleIndex!=10_11_23_24")
            if expected_diff_compress_enable >= 0:
                if self.param_int("lineOmegaDiffCompressEnable") != expected_diff_compress_enable:
                    reasons.append(f"lineOmegaDiffCompressEnable!={expected_diff_compress_enable}")
                if self.param("lineOmegaDiffCompressStartPermille") != DIFF_COMPRESS_DEFAULT_START_PERMILLE:
                    reasons.append("lineOmegaDiffCompressStartPermille!=850")
                if self.param("lineOmegaDiffCompressFullPermille") != DIFF_COMPRESS_DEFAULT_FULL_PERMILLE:
                    reasons.append("lineOmegaDiffCompressFullPermille!=950")
                if self.param("lineOmegaDiffCompressMinScalePct") != DIFF_COMPRESS_DEFAULT_MIN_SCALE_PCT:
                    reasons.append("lineOmegaDiffCompressMinScalePct!=90")
                if self.param("lineOmegaDiffCompressMaxSpeedPulse") != DIFF_COMPRESS_DEFAULT_MAX_SPEED:
                    reasons.append("lineOmegaDiffCompressMaxSpeedPulse!=95")
                if self.param("lineOmegaDiffCompressSumMin") != DIFF_COMPRESS_DEFAULT_SUM_MIN:
                    reasons.append("lineOmegaDiffCompressSumMin!=1000")
            if expected_curve_ff_enable >= 0:
                if self.param_int("lineOmegaCurveFfEnable") != expected_curve_ff_enable:
                    reasons.append(f"lineOmegaCurveFfEnable!={expected_curve_ff_enable}")
                if self.param("lineOmegaCurveFfRocMaxMm") != CURVE_FF_DEFAULT_ROC_MAX_MM:
                    reasons.append("lineOmegaCurveFfRocMaxMm!=200")
                if self.param("lineOmegaCurveFfRocMinMm") != CURVE_FF_DEFAULT_ROC_MIN_MM:
                    reasons.append("lineOmegaCurveFfRocMinMm!=80")
                if self.param("lineOmegaCurveFfMaxSpeedPulse") != CURVE_FF_DEFAULT_MAX_SPEED:
                    reasons.append("lineOmegaCurveFfMaxSpeedPulse!=95")
                if self.param_int("lineOmegaCurveFfGainPct") != CURVE_FF_DEFAULT_GAIN_PCT:
                    reasons.append("lineOmegaCurveFfGainPct!=100")
                if self.param_int("lineOmegaCurveFbScalePct") != CURVE_FF_DEFAULT_FB_SCALE_PCT:
                    reasons.append("lineOmegaCurveFbScalePct!=35")
        return "valid" if not reasons else ";".join(reasons)

    def is_valid(
        self,
        battery_min: float,
        require_sensor_diag: bool = False,
        expected_local_cap_enable: int = LOCAL_CAP_DEFAULT_ENABLE,
        expected_local_scale_enable: int = -1,
        expected_diff_compress_enable: int = -1,
        expected_curve_ff_enable: int = -1,
    ) -> bool:
        return self.valid_reason(
            battery_min,
            require_sensor_diag,
            expected_local_cap_enable,
            expected_local_scale_enable,
            expected_diff_compress_enable,
            expected_curve_ff_enable,
        ) == "valid"


def load_logs(numbers: list[int], log_dir: Path) -> list[LogData]:
    return [LogData(number, log_dir / f"{number}.csv") for number in numbers]


def row_pool(
    logs: list[LogData],
    battery_min: float,
    require_sensor_diag: bool = False,
    expected_local_cap_enable: int = LOCAL_CAP_DEFAULT_ENABLE,
    expected_local_scale_enable: int = -1,
    expected_diff_compress_enable: int = -1,
    expected_curve_ff_enable: int = -1,
) -> list[tuple[LogData, int, list[str]]]:
    pool: list[tuple[LogData, int, list[str]]] = []
    for log in logs:
        if not log.is_valid(
            battery_min,
            require_sensor_diag,
            expected_local_cap_enable,
            expected_local_scale_enable,
            expected_diff_compress_enable,
            expected_curve_ff_enable,
        ):
            continue
        pool.extend((log, i, row) for i, row in enumerate(log.rows))
    return pool


def filter_pool(
    pool: list[tuple[LogData, int, list[str]]],
    predicate,
) -> list[tuple[LogData, int, list[str]]]:
    return [(log, i, row) for log, i, row in pool if predicate(log, i, row)]


def vals(pool: list[tuple[LogData, int, list[str]]], column: str) -> list[float]:
    return [log.value(row, column) for log, _i, row in pool]


def motor_sat(log: LogData, row: list[str]) -> bool:
    return abs(log.value(row, "motorpwmL")) >= MOTOR_SAT_TH or abs(log.value(row, "motorpwmR")) >= MOTOR_SAT_TH


def line_omega_cap_deg_s(log: LogData) -> float:
    cap = log.param("lineOmegaTargetCapDegS", LINE_OMEGA_CAP_DEFAULT_DEG_S)
    return cap if not math.isnan(cap) else LINE_OMEGA_CAP_DEFAULT_DEG_S


def line_omega_cap_max_speed(log: LogData) -> float:
    speed = log.param("lineOmegaTargetCapMaxSpeedPulse", LINE_OMEGA_TARGET_CAP_MAX_SPEED_DEFAULT)
    return speed if not math.isnan(speed) else LINE_OMEGA_TARGET_CAP_MAX_SPEED_DEFAULT


def cap_applied(log: LogData, row: list[str]) -> bool:
    raw = log.value(row, "targetAngularveloRaw")
    target = log.value(row, "targetAngularvelo")
    cap = line_omega_cap_deg_s(log)
    return not math.isnan(raw) and not math.isnan(target) and abs(raw) > cap and abs(target) <= cap


def cap_missed_due_speed(log: LogData, row: list[str]) -> bool:
    raw = log.value(row, "targetAngularveloRaw")
    target = log.value(row, "targetAngularvelo")
    cap = line_omega_cap_deg_s(log)
    return not math.isnan(raw) and not math.isnan(target) and abs(raw) > cap and abs(target) > cap


def local_cap_applied_due_index(log: LogData, row: list[str]) -> bool:
    target_speed = log.value(row, "targetSpeed")
    if math.isnan(target_speed) or target_speed <= line_omega_cap_max_speed(log):
        return False
    return cap_applied(log, row)


def local_scale_indexes(log: LogData) -> tuple[int, ...]:
    indexes = (
        log.param_int("lineOmegaLocalTargetScaleIndex0"),
        log.param_int("lineOmegaLocalTargetScaleIndex1"),
        log.param_int("lineOmegaLocalTargetScaleIndex2"),
        log.param_int("lineOmegaLocalTargetScaleIndex3"),
    )
    if any(index < 0 for index in indexes):
        return LOCAL_SCALE_DEFAULT_INDEXES
    return indexes


def local_scale_max_speed(log: LogData) -> float:
    speed = log.param("lineOmegaLocalTargetScaleMaxSpeedPulse", LOCAL_SCALE_DEFAULT_MAX_SPEED)
    return speed if not math.isnan(speed) else LOCAL_SCALE_DEFAULT_MAX_SPEED


def local_target_scale_candidate(log: LogData, row: list[str]) -> bool:
    if log.param_int("lineOmegaLocalTargetScaleEnable", 0) != 1:
        return False
    target_speed = log.value(row, "targetSpeed")
    index = log.value(row, "optimalIndex")
    if math.isnan(target_speed) or math.isnan(index):
        return False
    return target_speed <= local_scale_max_speed(log) and int(round(index)) in set(local_scale_indexes(log))


def local_target_scale_applied(log: LogData, row: list[str]) -> bool:
    raw = log.value(row, "targetAngularveloRaw")
    target = log.value(row, "targetAngularvelo")
    return local_target_scale_candidate(log, row) and not math.isnan(raw) and not math.isnan(target) and abs(target) < abs(raw)


def target_angular_ratio(log: LogData, row: list[str]) -> float:
    raw = log.value(row, "targetAngularveloRaw")
    target = log.value(row, "targetAngularvelo")
    if math.isnan(raw) or math.isnan(target) or abs(raw) < 1.0:
        return math.nan
    return abs(target) / abs(raw)


def target_angular_delta_abs(log: LogData, row: list[str]) -> float:
    raw = log.value(row, "targetAngularveloRaw")
    target = log.value(row, "targetAngularvelo")
    if math.isnan(raw) or math.isnan(target):
        return math.nan
    return abs(raw - target)


def sensor_diff_norm(log: LogData, row: list[str]) -> float:
    diff = log.value(row, "lineOmegaSensorDiff")
    total = log.value(row, "lineOmegaSensorSum")
    if math.isnan(diff) or math.isnan(total) or total <= 0.0:
        return math.nan
    return abs(diff) / total


def diff_compress_param(log: LogData, name: str, default: float) -> float:
    value = log.param(name, default)
    return value if not math.isnan(value) else default


def diff_compress_permille(log: LogData, row: list[str]) -> float:
    norm = sensor_diff_norm(log, row)
    return norm * 1000.0 if not math.isnan(norm) else math.nan


def diff_compress_expected_scale_pct(log: LogData, row: list[str]) -> float:
    permille = diff_compress_permille(log, row)
    if math.isnan(permille):
        return math.nan
    start = diff_compress_param(log, "lineOmegaDiffCompressStartPermille", DIFF_COMPRESS_DEFAULT_START_PERMILLE)
    full = diff_compress_param(log, "lineOmegaDiffCompressFullPermille", DIFF_COMPRESS_DEFAULT_FULL_PERMILLE)
    min_scale = diff_compress_param(log, "lineOmegaDiffCompressMinScalePct", DIFF_COMPRESS_DEFAULT_MIN_SCALE_PCT)
    if permille <= start:
        return 100.0
    if permille >= full or full <= start:
        return min_scale
    return 100.0 - ((permille - start) * (100.0 - min_scale) / (full - start))


def diff_compress_candidate(log: LogData, row: list[str]) -> bool:
    target_speed = log.value(row, "targetSpeed")
    sensor_sum = log.value(row, "lineOmegaSensorSum")
    permille = diff_compress_permille(log, row)
    if math.isnan(target_speed) or math.isnan(sensor_sum) or math.isnan(permille):
        return False
    max_speed = diff_compress_param(log, "lineOmegaDiffCompressMaxSpeedPulse", DIFF_COMPRESS_DEFAULT_MAX_SPEED)
    sum_min = diff_compress_param(log, "lineOmegaDiffCompressSumMin", DIFF_COMPRESS_DEFAULT_SUM_MIN)
    start = diff_compress_param(log, "lineOmegaDiffCompressStartPermille", DIFF_COMPRESS_DEFAULT_START_PERMILLE)
    return target_speed <= max_speed and sensor_sum >= sum_min and permille > start


def diff_compress_applied(log: LogData, row: list[str]) -> bool:
    if log.param_int("lineOmegaDiffCompressEnable", 0) != 1 or not diff_compress_candidate(log, row):
        return False
    raw = log.value(row, "targetAngularveloRaw")
    target = log.value(row, "targetAngularvelo")
    return not math.isnan(raw) and not math.isnan(target) and abs(target) < abs(raw)


def curve_ff_param(log: LogData, name: str, default: float) -> float:
    value = log.param(name, default)
    return value if not math.isnan(value) else default


def curve_ff_candidate(log: LogData, row: list[str]) -> bool:
    if log.param_int("lineOmegaCurveFfEnable", 0) != 1:
        return False
    target_speed = log.value(row, "targetSpeed")
    roc = abs(log.value(row, "ROC"))
    sensor_diff = log.value(row, "lineOmegaSensorDiff")
    if math.isnan(target_speed) or math.isnan(roc) or math.isnan(sensor_diff):
        return False
    max_speed = curve_ff_param(log, "lineOmegaCurveFfMaxSpeedPulse", CURVE_FF_DEFAULT_MAX_SPEED)
    roc_max = curve_ff_param(log, "lineOmegaCurveFfRocMaxMm", CURVE_FF_DEFAULT_ROC_MAX_MM)
    return 0.0 < target_speed <= max_speed and 0.0 < roc <= roc_max and abs(sensor_diff) >= 1.0


def curve_ff_expected_pre_cap(log: LogData, row: list[str]) -> float:
    if not curve_ff_candidate(log, row):
        return math.nan
    raw = log.value(row, "targetAngularveloRaw")
    target_speed = log.value(row, "targetSpeed")
    roc = abs(log.value(row, "ROC"))
    sensor_diff = log.value(row, "lineOmegaSensorDiff")
    if math.isnan(raw) or math.isnan(target_speed) or math.isnan(roc) or math.isnan(sensor_diff):
        return math.nan
    roc_min = curve_ff_param(log, "lineOmegaCurveFfRocMinMm", CURVE_FF_DEFAULT_ROC_MIN_MM)
    if roc < roc_min:
        roc = roc_min
    gain_pct = curve_ff_param(log, "lineOmegaCurveFfGainPct", CURVE_FF_DEFAULT_GAIN_PCT)
    fb_scale_pct = curve_ff_param(log, "lineOmegaCurveFbScalePct", CURVE_FF_DEFAULT_FB_SCALE_PCT)
    speed_mm_s = (target_speed / PULSE_MILLIMETER) * 1000.0
    curve_ff = (speed_mm_s / roc) * RAD2DEG * (gain_pct / 100.0)
    if sensor_diff < 0:
        curve_ff = -curve_ff
    sensor_fb = raw * (fb_scale_pct / 100.0)
    return curve_ff + sensor_fb


def curve_ff_applied(log: LogData, row: list[str]) -> bool:
    raw = log.value(row, "targetAngularveloRaw")
    expected = curve_ff_expected_pre_cap(log, row)
    return not math.isnan(raw) and not math.isnan(expected) and abs(expected - raw) >= 1.0


def curve_ff_expected_ratio(log: LogData, row: list[str]) -> float:
    raw = log.value(row, "targetAngularveloRaw")
    expected = curve_ff_expected_pre_cap(log, row)
    if math.isnan(raw) or math.isnan(expected) or abs(raw) < 1.0:
        return math.nan
    return abs(expected) / abs(raw)


def curve_ff_expected_delta_abs(log: LogData, row: list[str]) -> float:
    raw = log.value(row, "targetAngularveloRaw")
    expected = curve_ff_expected_pre_cap(log, row)
    if math.isnan(raw) or math.isnan(expected):
        return math.nan
    return abs(expected - raw)


def curve_ff_final_delta_abs(log: LogData, row: list[str]) -> float:
    raw = log.value(row, "targetAngularveloRaw")
    target = log.value(row, "targetAngularvelo")
    if math.isnan(raw) or math.isnan(target):
        return math.nan
    return abs(target - raw)


def yaw_error(log: LogData, row: list[str]) -> float:
    target = log.value(row, "targetAngularvelo")
    gyro = log.value(row, "gyroVal_Z")
    if math.isnan(target) or math.isnan(gyro):
        return math.nan
    return target - gyro


def active_row(log: LogData, _i: int, row: list[str]) -> bool:
    return log.value(row, "targetSpeed") > 0.0 and log.value(row, "cntlog") >= 100.0


def straight_row(log: LogData, i: int, row: list[str]) -> bool:
    return active_row(log, i, row) and abs(log.value(row, "ROC")) >= STRAIGHT_ROC_TH


def curve_row(log: LogData, i: int, row: list[str]) -> bool:
    return active_row(log, i, row) and abs(log.value(row, "ROC")) < STRAIGHT_ROC_TH


def index_row(indexes: tuple[int, ...]):
    index_set = set(indexes)

    def _predicate(log: LogData, _i: int, row: list[str]) -> bool:
        idx = log.value(row, "optimalIndex")
        return not math.isnan(idx) and int(round(idx)) in index_set

    return _predicate


def group_metrics(
    group: str,
    logs: list[LogData],
    battery_min: float,
    require_sensor_diag: bool = False,
    expected_local_cap_enable: int = LOCAL_CAP_DEFAULT_ENABLE,
    expected_local_scale_enable: int = -1,
    expected_diff_compress_enable: int = -1,
    expected_curve_ff_enable: int = -1,
) -> dict[str, object]:
    valid_logs = [
        log for log in logs
        if log.is_valid(
            battery_min,
            require_sensor_diag,
            expected_local_cap_enable,
            expected_local_scale_enable,
            expected_diff_compress_enable,
            expected_curve_ff_enable,
        )
    ]
    pool = row_pool(
        logs,
        battery_min,
        require_sensor_diag,
        expected_local_cap_enable,
        expected_local_scale_enable,
        expected_diff_compress_enable,
        expected_curve_ff_enable,
    )
    active = filter_pool(pool, active_row)
    straight = filter_pool(pool, straight_row)
    curve = filter_pool(pool, curve_row)
    focus = filter_pool(pool, index_row(FOCUS_INDEXES))
    sensor_available = any(log.has_sensor_diag_columns() for log in valid_logs)
    yaw_errors = [yaw_error(log, row) for log, _i, row in focus]
    return {
        "group": group,
        "valid_log_count": len(valid_logs),
        "valid_logs": " ".join(str(log.number) for log in valid_logs),
        "battery_mean_V": mean([log.param("batteryVoltage_V") for log in valid_logs]),
        "lap_mean_ms": mean([log.last_cnt() for log in valid_logs]),
        "rows_total": len(pool),
        "motor_sat_pct": pct_true([motor_sat(log, row) for log, _i, row in active]),
        "straight_gyro_rms": rms(vals(straight, "gyroVal_Z")),
        "straight_gyro_p95_abs": p95_abs(vals(straight, "gyroVal_Z")),
        "straight_lineTrace_rms": rms(vals(straight, "lineTraceCtrl")),
        "straight_lineTrace_p95_abs": p95_abs(vals(straight, "lineTraceCtrl")),
        "target_curve_lineTrace_rms": rms(vals(curve, "lineTraceCtrl")),
        "target_curve_lineTrace_p95_abs": p95_abs(vals(curve, "lineTraceCtrl")),
        "idx10_11_23_24_samples": len(focus),
        "idx10_11_23_24_slipFlagLat_pct": pct_true([log.value(row, "slipFlagLat") >= 1.0 for log, _i, row in focus]),
        "idx10_11_23_24_lineTrace_rms": rms(vals(focus, "lineTraceCtrl")),
        "idx10_11_23_24_motor_sat_pct": pct_true([motor_sat(log, row) for log, _i, row in focus]),
        "idx10_11_23_24_yaw_error_rms": rms(yaw_errors),
        "idx10_11_23_24_yaw_error_p95_abs": p95_abs(yaw_errors),
        "idx10_11_23_24_targetAngular_raw_p95_abs": p95_abs(vals(focus, "targetAngularveloRaw")),
        "idx10_11_23_24_targetAngular_ratio_mean": mean([target_angular_ratio(log, row) for log, _i, row in focus]),
        "idx10_11_23_24_targetAngular_delta_abs_mean": mean([target_angular_delta_abs(log, row) for log, _i, row in focus]),
        "idx10_11_23_24_cap_applied_pct": pct_true([cap_applied(log, row) for log, _i, row in focus]),
        "idx10_11_23_24_cap_missed_due_speed_pct": pct_true([cap_missed_due_speed(log, row) for log, _i, row in focus]),
        "idx10_11_23_24_local_cap_applied_due_index_pct": pct_true([local_cap_applied_due_index(log, row) for log, _i, row in focus]),
        "idx10_11_23_24_local_target_scale_candidate_pct": pct_true([local_target_scale_candidate(log, row) for log, _i, row in focus]),
        "idx10_11_23_24_local_target_scale_applied_pct": pct_true([local_target_scale_applied(log, row) for log, _i, row in focus]),
        "idx10_11_23_24_diff_compress_candidate_pct": pct_true([diff_compress_candidate(log, row) for log, _i, row in focus]),
        "idx10_11_23_24_diff_compress_applied_pct": pct_true([diff_compress_applied(log, row) for log, _i, row in focus]),
        "idx10_11_23_24_diff_compress_expected_scale_pct_mean": mean([diff_compress_expected_scale_pct(log, row) for log, _i, row in focus]),
        "idx10_11_23_24_curve_ff_candidate_pct": pct_true([curve_ff_candidate(log, row) for log, _i, row in focus]),
        "idx10_11_23_24_curve_ff_applied_pct": pct_true([curve_ff_applied(log, row) for log, _i, row in focus]),
        "idx10_11_23_24_curve_ff_expected_ratio_mean": mean([curve_ff_expected_ratio(log, row) for log, _i, row in focus]),
        "idx10_11_23_24_curve_ff_expected_delta_abs_mean": mean([curve_ff_expected_delta_abs(log, row) for log, _i, row in focus]),
        "idx10_11_23_24_curve_ff_final_delta_abs_mean": mean([curve_ff_final_delta_abs(log, row) for log, _i, row in focus]),
        "sensor_diag_columns": sensor_available,
        "idx10_11_23_24_lineOmegaSensorDiff_mean": mean(vals(focus, "lineOmegaSensorDiff")),
        "idx10_11_23_24_lineOmegaSensorDiff_rms": rms(vals(focus, "lineOmegaSensorDiff")),
        "idx10_11_23_24_lineOmegaSensorDiff_p95_abs": p95_abs(vals(focus, "lineOmegaSensorDiff")),
        "idx10_11_23_24_lineOmegaSensorSum_mean": mean(vals(focus, "lineOmegaSensorSum")),
        "idx10_11_23_24_lineOmegaSensorSum_p05": p05(vals(focus, "lineOmegaSensorSum")),
        "idx10_11_23_24_lineOmegaSensorSum_p95": p95(vals(focus, "lineOmegaSensorSum")),
        "idx10_11_23_24_sensorDiffNorm_mean": mean([sensor_diff_norm(log, row) for log, _i, row in focus]),
        "idx10_11_23_24_sensorDiffNorm_p95": p95([sensor_diff_norm(log, row) for log, _i, row in focus]),
    }


def summary_rows(
    group: str,
    logs: list[LogData],
    battery_min: float,
    require_sensor_diag: bool = False,
    expected_local_cap_enable: int = LOCAL_CAP_DEFAULT_ENABLE,
    expected_local_scale_enable: int = -1,
    expected_diff_compress_enable: int = -1,
    expected_curve_ff_enable: int = -1,
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for log in logs:
        rows.append(
            {
                "group": group,
                "log": log.number,
                "valid_reason": log.valid_reason(
                    battery_min,
                    require_sensor_diag,
                    expected_local_cap_enable,
                    expected_local_scale_enable,
                    expected_diff_compress_enable,
                    expected_curve_ff_enable,
                ),
                "rows": len(log.rows),
                "last_cnt_ms": log.last_cnt(),
                "batteryVoltage_V": log.param("batteryVoltage_V"),
                "optimalTrace": log.param("optimalTrace"),
                "emcStop": log.param("emcStop"),
                "optimalIndex_max": log.optmax(),
                "cntlog_max_gap_ms": log.max_cnt_gap(),
                "cntlog_gap_gt30_count": log.cnt_gap_gt30_count(),
                "logRecordSizeBytes": log.param("logRecordSizeBytes"),
                "logBufferRecordCapacity": log.param("logBufferRecordCapacity"),
                "dbgOverflowFinal": log.param("dbgOverflowFinal"),
                "logOverflowFinal": log.param("logOverflowFinal"),
                "pwmSatClassLogProfile": log.param("pwmSatClassLogProfile"),
                "lineOmegaSensorDiagProfile": log.param("lineOmegaSensorDiagProfile"),
                "lineOmegaTargetCapMaxSpeedPulse": log.param("lineOmegaTargetCapMaxSpeedPulse"),
                "lineOmegaLocalTargetCapEnable": log.param("lineOmegaLocalTargetCapEnable"),
                "lineOmegaLocalTargetCapDegS": log.param("lineOmegaLocalTargetCapDegS"),
                "lineOmegaLocalTargetCapIndex0": log.param("lineOmegaLocalTargetCapIndex0"),
                "lineOmegaLocalTargetCapIndex1": log.param("lineOmegaLocalTargetCapIndex1"),
                "lineOmegaLocalTargetCapIndex2": log.param("lineOmegaLocalTargetCapIndex2"),
                "lineOmegaLocalTargetScaleEnable": log.param("lineOmegaLocalTargetScaleEnable"),
                "lineOmegaLocalTargetScale": log.param("lineOmegaLocalTargetScale"),
                "lineOmegaLocalTargetScalePct": log.param("lineOmegaLocalTargetScalePct"),
                "lineOmegaLocalTargetScaleMaxSpeedPulse": log.param("lineOmegaLocalTargetScaleMaxSpeedPulse"),
                "lineOmegaLocalTargetScaleIndex0": log.param("lineOmegaLocalTargetScaleIndex0"),
                "lineOmegaLocalTargetScaleIndex1": log.param("lineOmegaLocalTargetScaleIndex1"),
                "lineOmegaLocalTargetScaleIndex2": log.param("lineOmegaLocalTargetScaleIndex2"),
                "lineOmegaLocalTargetScaleIndex3": log.param("lineOmegaLocalTargetScaleIndex3"),
                "lineOmegaDiffCompressEnable": log.param("lineOmegaDiffCompressEnable"),
                "lineOmegaDiffCompressStartPermille": log.param("lineOmegaDiffCompressStartPermille"),
                "lineOmegaDiffCompressFullPermille": log.param("lineOmegaDiffCompressFullPermille"),
                "lineOmegaDiffCompressMinScalePct": log.param("lineOmegaDiffCompressMinScalePct"),
                "lineOmegaDiffCompressMaxSpeedPulse": log.param("lineOmegaDiffCompressMaxSpeedPulse"),
                "lineOmegaDiffCompressSumMin": log.param("lineOmegaDiffCompressSumMin"),
                "lineOmegaCurveFfEnable": log.param("lineOmegaCurveFfEnable"),
                "lineOmegaCurveFfRocMaxMm": log.param("lineOmegaCurveFfRocMaxMm"),
                "lineOmegaCurveFfRocMinMm": log.param("lineOmegaCurveFfRocMinMm"),
                "lineOmegaCurveFfMaxSpeedPulse": log.param("lineOmegaCurveFfMaxSpeedPulse"),
                "lineOmegaCurveFfGainPct": log.param("lineOmegaCurveFfGainPct"),
                "lineOmegaCurveFbScalePct": log.param("lineOmegaCurveFbScalePct"),
                "has_lineOmegaSensorDiff": "lineOmegaSensorDiff" in log.idx,
                "has_lineOmegaSensorSum": "lineOmegaSensorSum" in log.idx,
                "has_targetAngularveloRaw": "targetAngularveloRaw" in log.idx,
            }
        )
    return rows


def by_index_rows(
    group: str,
    logs: list[LogData],
    battery_min: float,
    require_sensor_diag: bool = False,
    expected_local_cap_enable: int = LOCAL_CAP_DEFAULT_ENABLE,
    expected_local_scale_enable: int = -1,
    expected_diff_compress_enable: int = -1,
    expected_curve_ff_enable: int = -1,
) -> list[dict[str, object]]:
    pool = row_pool(
        logs,
        battery_min,
        require_sensor_diag,
        expected_local_cap_enable,
        expected_local_scale_enable,
        expected_diff_compress_enable,
        expected_curve_ff_enable,
    )
    rows: list[dict[str, object]] = []
    for idx in WATCH_INDEXES:
        idx_pool = filter_pool(pool, index_row((idx,)))
        yaw_errors = [yaw_error(log, row) for log, _i, row in idx_pool]
        rows.append(
            {
                "group": group,
                "optimalIndex": idx,
                "samples": len(idx_pool),
                "targetSpeed_mean": mean(vals(idx_pool, "targetSpeed")),
                "slipFlagLat_pct": pct_true([log.value(row, "slipFlagLat") >= 1.0 for log, _i, row in idx_pool]),
                "lineTrace_rms": rms(vals(idx_pool, "lineTraceCtrl")),
                "lineTrace_p95_abs": p95_abs(vals(idx_pool, "lineTraceCtrl")),
                "gyro_rms": rms(vals(idx_pool, "gyroVal_Z")),
                "yaw_error_rms": rms(yaw_errors),
                "yaw_error_p95_abs": p95_abs(yaw_errors),
                "motor_sat_pct": pct_true([motor_sat(log, row) for log, _i, row in idx_pool]),
                "targetAngular_raw_p95_abs": p95_abs(vals(idx_pool, "targetAngularveloRaw")),
                "targetAngular_ratio_mean": mean([target_angular_ratio(log, row) for log, _i, row in idx_pool]),
                "targetAngular_delta_abs_mean": mean([target_angular_delta_abs(log, row) for log, _i, row in idx_pool]),
                "cap_applied_pct": pct_true([cap_applied(log, row) for log, _i, row in idx_pool]),
                "cap_missed_due_speed_pct": pct_true([cap_missed_due_speed(log, row) for log, _i, row in idx_pool]),
                "local_cap_applied_due_index_pct": pct_true([local_cap_applied_due_index(log, row) for log, _i, row in idx_pool]),
                "local_target_scale_candidate_pct": pct_true([local_target_scale_candidate(log, row) for log, _i, row in idx_pool]),
                "local_target_scale_applied_pct": pct_true([local_target_scale_applied(log, row) for log, _i, row in idx_pool]),
                "diff_compress_candidate_pct": pct_true([diff_compress_candidate(log, row) for log, _i, row in idx_pool]),
                "diff_compress_applied_pct": pct_true([diff_compress_applied(log, row) for log, _i, row in idx_pool]),
                "diff_compress_expected_scale_pct_mean": mean([diff_compress_expected_scale_pct(log, row) for log, _i, row in idx_pool]),
                "curve_ff_candidate_pct": pct_true([curve_ff_candidate(log, row) for log, _i, row in idx_pool]),
                "curve_ff_applied_pct": pct_true([curve_ff_applied(log, row) for log, _i, row in idx_pool]),
                "curve_ff_expected_ratio_mean": mean([curve_ff_expected_ratio(log, row) for log, _i, row in idx_pool]),
                "curve_ff_expected_delta_abs_mean": mean([curve_ff_expected_delta_abs(log, row) for log, _i, row in idx_pool]),
                "curve_ff_final_delta_abs_mean": mean([curve_ff_final_delta_abs(log, row) for log, _i, row in idx_pool]),
                "lineOmegaSensorDiff_mean": mean(vals(idx_pool, "lineOmegaSensorDiff")),
                "lineOmegaSensorDiff_rms": rms(vals(idx_pool, "lineOmegaSensorDiff")),
                "lineOmegaSensorDiff_p95_abs": p95_abs(vals(idx_pool, "lineOmegaSensorDiff")),
                "lineOmegaSensorSum_mean": mean(vals(idx_pool, "lineOmegaSensorSum")),
                "lineOmegaSensorSum_p05": p05(vals(idx_pool, "lineOmegaSensorSum")),
                "lineOmegaSensorSum_p95": p95(vals(idx_pool, "lineOmegaSensorSum")),
                "sensorDiffNorm_mean": mean([sensor_diff_norm(log, row) for log, _i, row in idx_pool]),
                "sensorDiffNorm_p95": p95([sensor_diff_norm(log, row) for log, _i, row in idx_pool]),
            }
        )
    return rows


def set_rows(
    group: str,
    logs: list[LogData],
    battery_min: float,
    require_sensor_diag: bool = False,
    expected_local_cap_enable: int = LOCAL_CAP_DEFAULT_ENABLE,
    expected_local_scale_enable: int = -1,
    expected_diff_compress_enable: int = -1,
    expected_curve_ff_enable: int = -1,
) -> list[dict[str, object]]:
    valid_logs = [
        log for log in logs
        if log.is_valid(
            battery_min,
            require_sensor_diag,
            expected_local_cap_enable,
            expected_local_scale_enable,
            expected_diff_compress_enable,
            expected_curve_ff_enable,
        )
    ]
    rows: list[dict[str, object]] = []
    for set_index, start in enumerate(range(0, len(valid_logs), 4), 1):
        logs_set = valid_logs[start:start + 4]
        if not logs_set:
            continue
        metrics = group_metrics(
            f"{group}_set{set_index}",
            logs_set,
            battery_min,
            require_sensor_diag,
            expected_local_cap_enable,
            expected_local_scale_enable,
            expected_diff_compress_enable,
            expected_curve_ff_enable,
        )
        metrics["complete_4_second_runs"] = len(logs_set) == 4
        rows.append(metrics)
    return rows


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames: list[str] = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def range_label(numbers: list[int]) -> str:
    if not numbers:
        return "none"
    nums = sorted(numbers)
    return f"{nums[0]}_{nums[-1]}"


def combined_label(*number_lists: list[int]) -> str:
    nums = sorted({num for numbers in number_lists for num in numbers})
    return range_label(nums)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--profile7-reference", default="10948-10956")
    parser.add_argument("--diff-compress-reference", default="10958-10966")
    parser.add_argument("--new", required=True)
    parser.add_argument("--new-label", default="lineomega_curve_ff_fb35")
    parser.add_argument("--expected-local-cap-enable", type=int, choices=(0, 1), default=LOCAL_CAP_DEFAULT_ENABLE)
    parser.add_argument("--expected-local-target-scale-enable", type=int, choices=(-1, 0, 1), default=LOCAL_SCALE_DEFAULT_ENABLE)
    parser.add_argument("--expected-diff-compress-enable", type=int, choices=(-1, 0, 1), default=DIFF_COMPRESS_DEFAULT_ENABLE)
    parser.add_argument("--expected-curve-ff-enable", type=int, choices=(-1, 0, 1), default=CURVE_FF_DEFAULT_ENABLE)
    parser.add_argument("--battery-min", type=float, default=7.6)
    parser.add_argument("--out-dir", type=Path, default=Path("analysis"))
    args = parser.parse_args()

    profile7_reference_nums = parse_ranges(args.profile7_reference)
    diff_compress_reference_nums = parse_ranges(args.diff_compress_reference)
    new_nums = parse_ranges(args.new)

    profile7_reference_logs = load_logs(profile7_reference_nums, args.log_dir)
    diff_compress_reference_logs = load_logs(diff_compress_reference_nums, args.log_dir)
    new_logs = load_logs(new_nums, args.log_dir)

    comparison_rows = [
        group_metrics(
            "profile7_reference",
            profile7_reference_logs,
            args.battery_min,
            require_sensor_diag=True,
            expected_local_cap_enable=-1,
            expected_local_scale_enable=-1,
        ),
        group_metrics(
            "diff_compress_rejected_reference",
            diff_compress_reference_logs,
            args.battery_min,
            require_sensor_diag=True,
            expected_local_cap_enable=-1,
            expected_local_scale_enable=-1,
            expected_diff_compress_enable=1,
            expected_curve_ff_enable=-1,
        ),
        group_metrics(
            args.new_label,
            new_logs,
            args.battery_min,
            require_sensor_diag=True,
            expected_local_cap_enable=args.expected_local_cap_enable,
            expected_local_scale_enable=args.expected_local_target_scale_enable,
            expected_diff_compress_enable=args.expected_diff_compress_enable,
            expected_curve_ff_enable=args.expected_curve_ff_enable,
        ),
    ]

    summary = []
    summary.extend(summary_rows(
        "profile7_reference",
        profile7_reference_logs,
        args.battery_min,
        require_sensor_diag=True,
        expected_local_cap_enable=-1,
        expected_local_scale_enable=-1,
    ))
    summary.extend(summary_rows(
        "diff_compress_rejected_reference",
        diff_compress_reference_logs,
        args.battery_min,
        require_sensor_diag=True,
        expected_local_cap_enable=-1,
        expected_local_scale_enable=-1,
        expected_diff_compress_enable=1,
        expected_curve_ff_enable=-1,
    ))
    summary.extend(summary_rows(
        args.new_label,
        new_logs,
        args.battery_min,
        require_sensor_diag=True,
        expected_local_cap_enable=args.expected_local_cap_enable,
        expected_local_scale_enable=args.expected_local_target_scale_enable,
        expected_diff_compress_enable=args.expected_diff_compress_enable,
        expected_curve_ff_enable=args.expected_curve_ff_enable,
    ))

    by_index = []
    by_index.extend(by_index_rows(
        "profile7_reference",
        profile7_reference_logs,
        args.battery_min,
        require_sensor_diag=True,
        expected_local_cap_enable=-1,
        expected_local_scale_enable=-1,
    ))
    by_index.extend(by_index_rows(
        "diff_compress_rejected_reference",
        diff_compress_reference_logs,
        args.battery_min,
        require_sensor_diag=True,
        expected_local_cap_enable=-1,
        expected_local_scale_enable=-1,
        expected_diff_compress_enable=1,
        expected_curve_ff_enable=-1,
    ))
    by_index.extend(by_index_rows(
        args.new_label,
        new_logs,
        args.battery_min,
        require_sensor_diag=True,
        expected_local_cap_enable=args.expected_local_cap_enable,
        expected_local_scale_enable=args.expected_local_target_scale_enable,
        expected_diff_compress_enable=args.expected_diff_compress_enable,
        expected_curve_ff_enable=args.expected_curve_ff_enable,
    ))

    sets = []
    sets.extend(set_rows(
        "profile7_reference",
        profile7_reference_logs,
        args.battery_min,
        require_sensor_diag=True,
        expected_local_cap_enable=-1,
        expected_local_scale_enable=-1,
    ))
    sets.extend(set_rows(
        "diff_compress_rejected_reference",
        diff_compress_reference_logs,
        args.battery_min,
        require_sensor_diag=True,
        expected_local_cap_enable=-1,
        expected_local_scale_enable=-1,
        expected_diff_compress_enable=1,
        expected_curve_ff_enable=-1,
    ))
    sets.extend(set_rows(
        args.new_label,
        new_logs,
        args.battery_min,
        require_sensor_diag=True,
        expected_local_cap_enable=args.expected_local_cap_enable,
        expected_local_scale_enable=args.expected_local_target_scale_enable,
        expected_diff_compress_enable=args.expected_diff_compress_enable,
        expected_curve_ff_enable=args.expected_curve_ff_enable,
    ))

    new_label = range_label(new_nums)
    reference_label = combined_label(profile7_reference_nums, diff_compress_reference_nums)
    output_label = args.new_label
    write_csv(args.out_dir / f"log_{new_label}_{output_label}_summary.csv", summary)
    write_csv(args.out_dir / f"log_{reference_label}_vs_{new_label}_{output_label}_comparison.csv", comparison_rows)
    write_csv(args.out_dir / f"log_{new_label}_{output_label}_by_index.csv", by_index)
    write_csv(args.out_dir / f"log_{new_label}_{output_label}_sets.csv", sets)


if __name__ == "__main__":
    main()
