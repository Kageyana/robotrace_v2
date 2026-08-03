#!/usr/bin/env python3
"""Summarize emcStop line-detection diagnostic logs."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


DEFAULT_LOG_DIR = Path(r"F:\Dropbox\Document\robotrace\Log\v2")
STRAIGHT_ROC_TH = 2500.0
MOTOR_SAT_TH = 999.0
EMC_STOP_LINE_DIAG_PROFILE = 1
PWM_SAT_CLASS_LOG_PROFILE = 8
WATCH_INDEXES = (10, 11, 23, 24, 120, 124, 137, 141)


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

    def has_line_diag_columns(self) -> bool:
        return {"emcStopLineBrightCount", "emcStopLineUnbrightCount"}.issubset(self.idx)

    def valid_reason(self, battery_min: float, require_line_diag: bool = False) -> str:
        if self.missing:
            return "missing_file"
        reasons: list[str] = []
        required = {
            "cntlog",
            "encCurrentN",
            "gyroVal_Z",
            "courseMarker",
            "ROC",
            "targetSpeed",
            "optimalIndex",
            "slipFlagLat",
            "lineTraceCtrl",
            "targetAngularvelo",
            "motorpwmL",
            "motorpwmR",
            "x",
            "y",
        }
        if require_line_diag:
            required.update({"targetAngularveloRaw", "emcStopLineBrightCount", "emcStopLineUnbrightCount"})
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
        if require_line_diag:
            if self.param_int("pwmSatClassLogProfile") != PWM_SAT_CLASS_LOG_PROFILE:
                reasons.append(f"pwmSatClassLogProfile={self.param('pwmSatClassLogProfile'):.0f}")
            if self.param_int("emcStopLineDiagProfile") != EMC_STOP_LINE_DIAG_PROFILE:
                reasons.append(f"emcStopLineDiagProfile={self.param('emcStopLineDiagProfile'):.0f}")
        return "valid" if not reasons else ";".join(reasons)

    def is_valid(self, battery_min: float, require_line_diag: bool = False) -> bool:
        return self.valid_reason(battery_min, require_line_diag) == "valid"


def load_logs(numbers: list[int], log_dir: Path) -> list[LogData]:
    return [LogData(number, log_dir / f"{number}.csv") for number in numbers]


def row_pool(logs: list[LogData], battery_min: float, require_line_diag: bool = False) -> list[tuple[LogData, int, list[str]]]:
    pool: list[tuple[LogData, int, list[str]]] = []
    for log in logs:
        if not log.is_valid(battery_min, require_line_diag):
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


def active_row(log: LogData, _i: int, row: list[str]) -> bool:
    return log.value(row, "targetSpeed") > 0.0 and log.value(row, "cntlog") >= 100.0


def straight_row(log: LogData, i: int, row: list[str]) -> bool:
    return active_row(log, i, row) and abs(log.value(row, "ROC")) >= STRAIGHT_ROC_TH


def curve_row(log: LogData, i: int, row: list[str]) -> bool:
    return active_row(log, i, row) and abs(log.value(row, "ROC")) < STRAIGHT_ROC_TH


def index_row(index: int):
    def _predicate(log: LogData, _i: int, row: list[str]) -> bool:
        idx = log.value(row, "optimalIndex")
        return not math.isnan(idx) and int(round(idx)) == index

    return _predicate


def counter_delta(log: LogData, rows: list[list[str]], column: str) -> float:
    values = [log.value(row, column) for row in rows]
    values = [v for v in values if not math.isnan(v)]
    if not values:
        return math.nan
    return values[-1] - values[0]


def group_metrics(group: str, logs: list[LogData], battery_min: float, require_line_diag: bool = False) -> dict[str, object]:
    valid_logs = [log for log in logs if log.is_valid(battery_min, require_line_diag)]
    pool = row_pool(logs, battery_min, require_line_diag)
    active = filter_pool(pool, active_row)
    straight = filter_pool(pool, straight_row)
    curve = filter_pool(pool, curve_row)
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
        "slipFlagLat_pct": pct_true([log.value(row, "slipFlagLat") >= 1.0 for log, _i, row in active]),
        "lineBrightCount_delta_mean": mean([
            counter_delta(log, log.rows, "emcStopLineBrightCount") for log in valid_logs
        ]),
        "lineUnbrightCount_delta_mean": mean([
            counter_delta(log, log.rows, "emcStopLineUnbrightCount") for log in valid_logs
        ]),
    }


def summary_rows(group: str, logs: list[LogData], battery_min: float, require_line_diag: bool = False) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for log in logs:
        rows.append(
            {
                "group": group,
                "log": log.number,
                "valid_reason": log.valid_reason(battery_min, require_line_diag),
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
                "emcStopLineDiagProfile": log.param("emcStopLineDiagProfile"),
                "emcStopLatchedReasonFinal": log.param("emcStopLatchedReasonFinal"),
                "emcStopLatchedEncNFinal": log.param("emcStopLatchedEncNFinal"),
                "emcStopLatchedEncLFinal": log.param("emcStopLatchedEncLFinal"),
                "emcStopLatchedEncRFinal": log.param("emcStopLatchedEncRFinal"),
                "has_emcStopLineBrightCount": "emcStopLineBrightCount" in log.idx,
                "has_emcStopLineUnbrightCount": "emcStopLineUnbrightCount" in log.idx,
                "has_lineOmegaSensorDiff": "lineOmegaSensorDiff" in log.idx,
                "has_lineOmegaSensorSum": "lineOmegaSensorSum" in log.idx,
            }
        )
    return rows


def by_index_rows(group: str, logs: list[LogData], battery_min: float, require_line_diag: bool = False) -> list[dict[str, object]]:
    pool = row_pool(logs, battery_min, require_line_diag)
    rows: list[dict[str, object]] = []
    for index in WATCH_INDEXES:
        idx_pool = filter_pool(pool, index_row(index))
        rows.append(
            {
                "group": group,
                "optimalIndex": index,
                "samples": len(idx_pool),
                "targetSpeed_mean": mean(vals(idx_pool, "targetSpeed")),
                "slipFlagLat_pct": pct_true([log.value(row, "slipFlagLat") >= 1.0 for log, _i, row in idx_pool]),
                "lineTrace_rms": rms(vals(idx_pool, "lineTraceCtrl")),
                "lineTrace_p95_abs": p95_abs(vals(idx_pool, "lineTraceCtrl")),
                "gyro_rms": rms(vals(idx_pool, "gyroVal_Z")),
                "targetAngular_rms": rms(vals(idx_pool, "targetAngularvelo")),
                "targetAngular_raw_p95_abs": p95_abs(vals(idx_pool, "targetAngularveloRaw")),
                "motor_sat_pct": pct_true([motor_sat(log, row) for log, _i, row in idx_pool]),
                "lineBrightCount_delta_mean": mean([
                    counter_delta(log, [row for check_log, _i, row in idx_pool if check_log is log], "emcStopLineBrightCount")
                    for log in logs
                ]),
                "lineUnbrightCount_delta_mean": mean([
                    counter_delta(log, [row for check_log, _i, row in idx_pool if check_log is log], "emcStopLineUnbrightCount")
                    for log in logs
                ]),
            }
        )
    return rows


def stop_focus_rows(group: str, logs: list[LogData]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for log in logs:
        if log.missing or log.param_int("emcStop", 0) == 0 or not log.rows:
            continue
        last_cnt = log.last_cnt()
        if math.isnan(last_cnt):
            window = log.rows[-40:]
        else:
            window = [
                row for row in log.rows
                if not math.isnan(log.value(row, "cntlog")) and (last_cnt - log.value(row, "cntlog")) <= 200.0
            ]
        if not window:
            window = log.rows[-40:]
        last = window[-1]
        rows.append(
            {
                "group": group,
                "log": log.number,
                "emcStop": log.param("emcStop"),
                "optimalTrace": log.param("optimalTrace"),
                "autoStart": log.param("autoStart"),
                "batteryVoltage_V": log.param("batteryVoltage_V"),
                "last_cnt_ms": last_cnt,
                "window_rows": len(window),
                "last_optimalIndex": log.value(last, "optimalIndex"),
                "last_targetSpeed": log.value(last, "targetSpeed"),
                "last_encCurrentN": log.value(last, "encCurrentN"),
                "last_courseMarker": log.value(last, "courseMarker"),
                "last_x": log.value(last, "x"),
                "last_y": log.value(last, "y"),
                "last_gyroVal_Z": log.value(last, "gyroVal_Z"),
                "gyro_p95_abs_last200ms": p95_abs([log.value(row, "gyroVal_Z") for row in window]),
                "lineTrace_rms_last200ms": rms([log.value(row, "lineTraceCtrl") for row in window]),
                "lineTrace_p95_abs_last200ms": p95_abs([log.value(row, "lineTraceCtrl") for row in window]),
                "motor_sat_pct_last200ms": pct_true([motor_sat(log, row) for row in window]),
                "lineBrightCount_delta_last200ms": counter_delta(log, window, "emcStopLineBrightCount"),
                "lineUnbrightCount_delta_last200ms": counter_delta(log, window, "emcStopLineUnbrightCount"),
                "lineBrightCount_last": log.value(last, "emcStopLineBrightCount"),
                "lineUnbrightCount_last": log.value(last, "emcStopLineUnbrightCount"),
                "targetAngular_rms_last200ms": rms([log.value(row, "targetAngularvelo") for row in window]),
                "targetAngularRaw_p95_abs_last200ms": p95_abs([log.value(row, "targetAngularveloRaw") for row in window]),
                "emcStopLatchedReasonFinal": log.param("emcStopLatchedReasonFinal"),
                "emcStopLatchedEncNFinal": log.param("emcStopLatchedEncNFinal"),
                "emcStopLatchedEncLFinal": log.param("emcStopLatchedEncLFinal"),
                "emcStopLatchedEncRFinal": log.param("emcStopLatchedEncRFinal"),
                "has_line_diag_columns": log.has_line_diag_columns(),
            }
        )
    return rows


def set_rows(group: str, logs: list[LogData], battery_min: float, require_line_diag: bool = False) -> list[dict[str, object]]:
    valid_logs = [log for log in logs if log.is_valid(battery_min, require_line_diag)]
    rows: list[dict[str, object]] = []
    for set_index, start in enumerate(range(0, len(valid_logs), 4), 1):
        logs_set = valid_logs[start:start + 4]
        if not logs_set:
            continue
        metrics = group_metrics(f"{group}_set{set_index}", logs_set, battery_min, require_line_diag)
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
    parser.add_argument("--baseline", default="10880-10888")
    parser.add_argument("--kp6-reference", default="10890-10898")
    parser.add_argument("--reconfirm", default="10899-10916")
    parser.add_argument("--new", required=True)
    parser.add_argument("--new-label", default="emc_line_diag")
    parser.add_argument("--battery-min", type=float, default=7.7)
    parser.add_argument("--out-dir", type=Path, default=Path("analysis"))
    args = parser.parse_args()

    baseline_nums = parse_ranges(args.baseline)
    kp6_nums = parse_ranges(args.kp6_reference)
    reconfirm_nums = parse_ranges(args.reconfirm)
    new_nums = parse_ranges(args.new)

    baseline_logs = load_logs(baseline_nums, args.log_dir)
    kp6_logs = load_logs(kp6_nums, args.log_dir)
    reconfirm_logs = load_logs(reconfirm_nums, args.log_dir)
    new_logs = load_logs(new_nums, args.log_dir)

    summary = []
    summary.extend(summary_rows("lineomega_kp5_revert_base", baseline_logs, args.battery_min))
    summary.extend(summary_rows("lineomega_kp6_rejected", kp6_logs, args.battery_min))
    summary.extend(summary_rows("lineomega_kp5_reconfirm", reconfirm_logs, args.battery_min))
    summary.extend(summary_rows(args.new_label, new_logs, args.battery_min, require_line_diag=True))

    comparison = [
        group_metrics("lineomega_kp5_revert_base", baseline_logs, args.battery_min),
        group_metrics("lineomega_kp6_rejected", kp6_logs, args.battery_min),
        group_metrics("lineomega_kp5_reconfirm", reconfirm_logs, args.battery_min),
        group_metrics(args.new_label, new_logs, args.battery_min, require_line_diag=True),
    ]

    by_index = []
    by_index.extend(by_index_rows("lineomega_kp5_revert_base", baseline_logs, args.battery_min))
    by_index.extend(by_index_rows("lineomega_kp6_rejected", kp6_logs, args.battery_min))
    by_index.extend(by_index_rows("lineomega_kp5_reconfirm", reconfirm_logs, args.battery_min))
    by_index.extend(by_index_rows(args.new_label, new_logs, args.battery_min, require_line_diag=True))

    stops = []
    stops.extend(stop_focus_rows("lineomega_kp5_revert_base", baseline_logs))
    stops.extend(stop_focus_rows("lineomega_kp6_rejected", kp6_logs))
    stops.extend(stop_focus_rows("lineomega_kp5_reconfirm", reconfirm_logs))
    stops.extend(stop_focus_rows(args.new_label, new_logs))

    sets = []
    sets.extend(set_rows("lineomega_kp5_revert_base", baseline_logs, args.battery_min))
    sets.extend(set_rows("lineomega_kp6_rejected", kp6_logs, args.battery_min))
    sets.extend(set_rows("lineomega_kp5_reconfirm", reconfirm_logs, args.battery_min))
    sets.extend(set_rows(args.new_label, new_logs, args.battery_min, require_line_diag=True))

    new_label = range_label(new_nums)
    reference_label = combined_label(baseline_nums, kp6_nums, reconfirm_nums)
    write_csv(args.out_dir / f"log_{new_label}_{args.new_label}_summary.csv", summary)
    write_csv(args.out_dir / f"log_{reference_label}_vs_{new_label}_{args.new_label}_comparison.csv", comparison)
    write_csv(args.out_dir / f"log_{new_label}_{args.new_label}_by_index.csv", by_index)
    write_csv(args.out_dir / f"log_{new_label}_{args.new_label}_stop_focus.csv", stops)
    write_csv(args.out_dir / f"log_{new_label}_{args.new_label}_sets.csv", sets)


if __name__ == "__main__":
    main()
