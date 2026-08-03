#!/usr/bin/env python3
"""Summarize local lateral-slip speed-cap comparison logs."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


DEFAULT_LOG_DIR = Path(r"F:\Dropbox\Document\robotrace\Log\v2")
PWM_SAT_TH = 999.0
STRAIGHT_ROC_TH = 2500.0
FOCUS_INDEXES = (10, 11, 23)
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


def pct_true(values: list[bool]) -> float:
    return 100.0 * sum(1 for v in values if v) / len(values) if values else math.nan


def p95_abs(values: list[float]) -> float:
    vals = sorted(abs(v) for v in values if not math.isnan(v))
    if not vals:
        return math.nan
    idx = math.ceil(0.95 * len(vals)) - 1
    return vals[max(0, min(idx, len(vals) - 1))]


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

    def valid_reason(self, battery_min: float) -> str:
        reasons: list[str] = []
        required = {
            "cntlog",
            "encCurrentN",
            "gyroVal_Z",
            "ROC",
            "targetSpeed",
            "targetSpeedBase",
            "targetSpeedScale",
            "optimalIndex",
            "slipFlagLat",
            "lineTraceCtrl",
            "motorpwmL",
            "motorpwmR",
            "lineTraceOmegaFBCtrlkp",
            "lineTraceOmegaFBCtrlkd",
        }
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
        return "valid" if not reasons else ";".join(reasons)

    def is_valid(self, battery_min: float) -> bool:
        return self.valid_reason(battery_min) == "valid"


def load_logs(numbers: list[int], log_dir: Path) -> list[LogData]:
    logs: list[LogData] = []
    for number in numbers:
        path = log_dir / f"{number}.csv"
        if path.exists():
            logs.append(LogData(number, path))
    return logs


def rows_for(logs: list[LogData], battery_min: float) -> list[tuple[LogData, list[str]]]:
    out: list[tuple[LogData, list[str]]] = []
    for log in logs:
        if not log.is_valid(battery_min):
            continue
        out.extend((log, row) for row in log.rows)
    return out


def filter_rows(pool: list[tuple[LogData, list[str]]], pred) -> list[tuple[LogData, list[str]]]:
    return [(log, row) for log, row in pool if pred(log, row)]


def row_vals(pool: list[tuple[LogData, list[str]]], column: str) -> list[float]:
    return [log.value(row, column) for log, row in pool if column in log.idx]


def pwm_sat_flags(pool: list[tuple[LogData, list[str]]]) -> list[bool]:
    return [
        abs(log.value(row, "motorpwmL")) >= PWM_SAT_TH or abs(log.value(row, "motorpwmR")) >= PWM_SAT_TH
        for log, row in pool
        if "motorpwmL" in log.idx and "motorpwmR" in log.idx
    ]


def group_metrics(group: str, logs: list[LogData], battery_min: float) -> dict[str, object]:
    valid_logs = [log for log in logs if log.is_valid(battery_min)]
    pool = rows_for(logs, battery_min)
    straight = filter_rows(
        pool,
        lambda log, row: abs(log.value(row, "ROC")) >= STRAIGHT_ROC_TH
        and log.value(row, "targetSpeed") > 0
        and log.value(row, "cntlog") >= 100,
    )
    target_curve = filter_rows(
        pool,
        lambda log, row: abs(log.value(row, "ROC")) < STRAIGHT_ROC_TH
        and log.value(row, "targetSpeed") > 0
        and log.value(row, "cntlog") >= 100,
    )
    focus = filter_rows(pool, lambda log, row: int(log.value(row, "optimalIndex")) in FOCUS_INDEXES)
    band_120_159 = filter_rows(pool, lambda log, row: 120 <= log.value(row, "targetSpeed") <= 159)
    band_160_179 = filter_rows(pool, lambda log, row: 160 <= log.value(row, "targetSpeed") <= 179)

    return {
        "group": group,
        "battery_min": battery_min,
        "valid_logs": " ".join(str(log.number) for log in valid_logs),
        "n_valid": len(valid_logs),
        "batteryVoltage_V_mean": mean([log.param("batteryVoltage_V") for log in valid_logs]),
        "lap_ms_mean": mean([log.last_cnt() for log in valid_logs]),
        "rows_mean": mean([float(len(log.rows)) for log in valid_logs]),
        "pwm_sat_pct": pct_true(pwm_sat_flags(pool)),
        "straight_gyro_rms": rms(row_vals(straight, "gyroVal_Z")),
        "straight_gyro_abs_p95": p95_abs(row_vals(straight, "gyroVal_Z")),
        "straight_lineTrace_rms": rms(row_vals(straight, "lineTraceCtrl")),
        "straight_lineTrace_abs_p95": p95_abs(row_vals(straight, "lineTraceCtrl")),
        "target_curve_pwm_sat_pct": pct_true(pwm_sat_flags(target_curve)),
        "target_curve_lineTrace_rms": rms(row_vals(target_curve, "lineTraceCtrl")),
        "band_120_159_pwm_sat_pct": pct_true(pwm_sat_flags(band_120_159)),
        "band_160_179_pwm_sat_pct": pct_true(pwm_sat_flags(band_160_179)),
        "focus_10_11_23_target_mean": mean(row_vals(focus, "targetSpeed")),
        "focus_10_11_23_base_mean": mean(row_vals(focus, "targetSpeedBase")),
        "focus_10_11_23_scale_mean": mean(row_vals(focus, "targetSpeedScale")),
        "focus_10_11_23_slipLat_pct": mean(row_vals(focus, "slipFlagLat")) * 100.0,
        "focus_10_11_23_pwm_sat_pct": pct_true(pwm_sat_flags(focus)),
        "focus_10_11_23_lineTrace_rms": rms(row_vals(focus, "lineTraceCtrl")),
        "focus_10_11_23_samples": len(focus),
    }


def summary_rows(group: str, logs: list[LogData], battery_min: float) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for log in logs:
        out.append(
            {
                "group": group,
                "log": log.number,
                "valid_reason": log.valid_reason(battery_min),
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
                "slipLocalLatSpeedScale": log.param("slipLocalLatSpeedScale"),
                "slipLocalLatIndex0": log.param("slipLocalLatIndex0"),
                "slipLocalLatIndex1": log.param("slipLocalLatIndex1"),
                "slipLocalLatIndex2": log.param("slipLocalLatIndex2"),
                "slipLocalLatRocMaxMm": log.param("slipLocalLatRocMaxMm"),
                "slipLocalLatHoldRuns": log.param("slipLocalLatHoldRuns"),
                "has_targetSpeedBase": int("targetSpeedBase" in log.idx),
                "has_targetSpeedScale": int("targetSpeedScale" in log.idx),
                "has_lineTraceOmegaFBCtrlkp": int("lineTraceOmegaFBCtrlkp" in log.idx),
                "has_lineTraceOmegaFBCtrlkd": int("lineTraceOmegaFBCtrlkd" in log.idx),
                "has_slipLatResidual_mps2": int("slipLatResidual_mps2" in log.idx),
                "has_slipLatHigh": int("slipLatHigh" in log.idx),
            }
        )
    return out


def by_index_rows(group: str, logs: list[LogData], battery_min: float) -> list[dict[str, object]]:
    pool = rows_for(logs, battery_min)
    out: list[dict[str, object]] = []
    for idx in WATCH_INDEXES:
        rows = filter_rows(pool, lambda log, row, idx=idx: int(log.value(row, "optimalIndex")) == idx)
        out.append(
            {
                "group": group,
                "optimalIndex": idx,
                "samples": len(rows),
                "targetSpeed_mean": mean(row_vals(rows, "targetSpeed")),
                "targetSpeedBase_mean": mean(row_vals(rows, "targetSpeedBase")),
                "targetSpeedScale_mean": mean(row_vals(rows, "targetSpeedScale")),
                "encCurrentN_mean": mean(row_vals(rows, "encCurrentN")),
                "speed_err_mean": mean(
                    [log.value(row, "encCurrentN") - log.value(row, "targetSpeed") for log, row in rows]
                ),
                "pwm_sat_pct": pct_true(pwm_sat_flags(rows)),
                "lineTrace_rms": rms(row_vals(rows, "lineTraceCtrl")),
                "gyro_rms": rms(row_vals(rows, "gyroVal_Z")),
                "slipFlagLat_pct": mean(row_vals(rows, "slipFlagLat")) * 100.0,
            }
        )
    return out


def set_rows(group: str, logs: list[LogData], battery_min: float) -> list[dict[str, object]]:
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
        valid = [log for log in set_logs if log.is_valid(battery_min)]
        excluded = [f"{log.number}:{log.valid_reason(battery_min)}" for log in set_logs if not log.is_valid(battery_min)]
        metrics = group_metrics(f"{group}_set{idx}", valid, battery_min)
        out.append(
            {
                "group": f"{group}_set{idx}",
                "logs": " ".join(str(log.number) for log in set_logs),
                "valid_logs": " ".join(str(log.number) for log in valid),
                "excluded_logs": " | ".join(excluded),
                "n_valid": len(valid),
                "complete_4_second_runs": int(len(valid) == 4),
                "batteryVoltage_V_mean": metrics["batteryVoltage_V_mean"],
                "lap_ms_mean": metrics["lap_ms_mean"],
                "pwm_sat_pct": metrics["pwm_sat_pct"],
                "straight_gyro_rms": metrics["straight_gyro_rms"],
                "straight_lineTrace_rms": metrics["straight_lineTrace_rms"],
                "target_curve_pwm_sat_pct": metrics["target_curve_pwm_sat_pct"],
                "target_curve_lineTrace_rms": metrics["target_curve_lineTrace_rms"],
                "focus_10_11_23_scale_mean": metrics["focus_10_11_23_scale_mean"],
                "focus_10_11_23_slipLat_pct": metrics["focus_10_11_23_slipLat_pct"],
                "focus_10_11_23_pwm_sat_pct": metrics["focus_10_11_23_pwm_sat_pct"],
                "focus_10_11_23_lineTrace_rms": metrics["focus_10_11_23_lineTrace_rms"],
            }
        )
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


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--baseline", default="10570-10588")
    parser.add_argument("--rejected", default="10590-10603")
    parser.add_argument("--diag", default="10605-10613")
    parser.add_argument("--new", default="10614-10623")
    parser.add_argument("--tag", default="local_lat_speedcap_hold2")
    parser.add_argument("--new-group", default="localcap_hold2")
    parser.add_argument("--battery-min", type=float, default=7.7)
    parser.add_argument("--out-dir", type=Path, default=Path("analysis"))
    args = parser.parse_args()

    base_nums = parse_ranges(args.baseline)
    rej_nums = parse_ranges(args.rejected)
    diag_nums = parse_ranges(args.diag)
    new_nums = parse_ranges(args.new)

    base_logs = load_logs(base_nums, args.log_dir)
    rej_logs = load_logs(rej_nums, args.log_dir)
    diag_logs = load_logs(diag_nums, args.log_dir)
    new_logs = load_logs(new_nums, args.log_dir)
    new_label = range_label(new_nums)

    summary = summary_rows("localcap085", new_logs, args.battery_min)
    comparison = [
        group_metrics("baseline_lat030", base_logs, args.battery_min),
        group_metrics("rejected_lat033", rej_logs, args.battery_min),
        group_metrics("diag_lat030", diag_logs, args.battery_min),
        group_metrics(args.new_group, new_logs, args.battery_min),
        group_metrics(f"{args.new_group}_bat76_reference", new_logs, 7.6),
    ]
    by_index = (
        by_index_rows("baseline_lat030", base_logs, args.battery_min)
        + by_index_rows("rejected_lat033", rej_logs, args.battery_min)
        + by_index_rows("diag_lat030", diag_logs, args.battery_min)
        + by_index_rows(args.new_group, new_logs, args.battery_min)
        + by_index_rows(f"{args.new_group}_bat76_reference", new_logs, 7.6)
    )
    sets = (
        set_rows("baseline_lat030", base_logs, args.battery_min)
        + set_rows("rejected_lat033", rej_logs, args.battery_min)
        + set_rows("diag_lat030", diag_logs, args.battery_min)
        + set_rows(args.new_group, new_logs, args.battery_min)
        + set_rows(f"{args.new_group}_bat76_reference", new_logs, 7.6)
    )

    write_csv(args.out_dir / f"log_{new_label}_{args.tag}_summary.csv", summary)
    write_csv(args.out_dir / f"log_{range_label(base_nums)}_{range_label(diag_nums)}_vs_{new_label}_{args.tag}_comparison.csv", comparison)
    write_csv(args.out_dir / f"log_{new_label}_{args.tag}_by_index.csv", by_index)
    write_csv(args.out_dir / f"log_{new_label}_{args.tag}_sets.csv", sets)
    print(f"wrote local lateral speed cap report for {new_label}")


if __name__ == "__main__":
    main()
