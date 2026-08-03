#!/usr/bin/env python3
"""Summarize speed-plan scale debug logs for BOOST_DISTANCE runs."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


LOG_DIR = Path(r"F:\Dropbox\Document\robotrace\Log\v2")
HIGH_INDEXES = set(range(88, 96)) | {181, 182}
HIGH_MAIN_INDEXES = set(range(88, 96))
STRAIGHT_ROC_TH = 2500.0
PWM_SAT_TH = 999.0


def parse_ranges(spec: str) -> list[int]:
    nums: list[int] = []
    if not spec:
        return nums
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
    idx = math.ceil(0.95 * len(vals)) - 1
    return vals[max(0, min(idx, len(vals) - 1))]


def pct_true(flags: list[bool]) -> float:
    return 100.0 * sum(1 for f in flags if f) / len(flags) if flags else math.nan


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
            for token in header:
                if "=" in token:
                    key, value = token.split("=", 1)
                    parsed = parse_float(value)
                    self.params[key] = value if math.isnan(parsed) else parsed
                else:
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

    def cnt_monotonic(self) -> bool:
        cnt = self.values("cntlog")
        return all(cnt[i] > cnt[i - 1] for i in range(1, len(cnt)))

    def last_cnt(self) -> float:
        cnt = self.values("cntlog")
        return cnt[-1] if cnt else math.nan

    def optmax(self) -> float:
        vals = self.values("optimalIndex")
        vals = [v for v in vals if not math.isnan(v)]
        return max(vals) if vals else math.nan

    def is_valid_second(self, battery_min: float) -> bool:
        return (
            int(round(self.param("optimalTrace"))) == 2
            and int(round(self.param("emcStop"))) == 0
            and self.cnt_monotonic()
            and len(self.rows) >= 850
            and self.optmax() >= 180
            and self.param("batteryVoltage_V") >= battery_min
        )


def load_logs(numbers: list[int], log_dir: Path) -> list[LogData]:
    logs: list[LogData] = []
    for number in numbers:
        path = log_dir / f"{number}.csv"
        if path.exists():
            logs.append(LogData(number, path))
    return logs


def selected_rows(logs: list[LogData], battery_min: float, auto_min: int | None = None) -> list[tuple[LogData, list[str]]]:
    selected: list[tuple[LogData, list[str]]] = []
    for log in logs:
        if not log.is_valid_second(battery_min):
            continue
        if auto_min is not None and int(round(log.param("autoStart"))) < auto_min:
            continue
        for row in log.rows:
            selected.append((log, row))
    return selected


def row_metric(log: LogData, row: list[str], column: str) -> float:
    return log.value(row, column)


def aggregate_group(label: str, logs: list[LogData], battery_min: float) -> dict[str, str | float | int]:
    valid_logs = [log for log in logs if log.is_valid_second(battery_min)]
    rows = selected_rows(logs, battery_min)
    rows_auto3 = selected_rows(logs, battery_min, auto_min=3)

    def rows_where(pool: list[tuple[LogData, list[str]]], pred) -> list[tuple[LogData, list[str]]]:
        return [(log, row) for log, row in pool if pred(log, row)]

    high_rows = rows_where(rows, lambda log, row: int(row_metric(log, row, "optimalIndex")) in HIGH_INDEXES)
    high_main_rows = rows_where(rows_auto3, lambda log, row: int(row_metric(log, row, "optimalIndex")) in HIGH_MAIN_INDEXES)
    straight_rows = rows_where(
        rows,
        lambda log, row: abs(row_metric(log, row, "ROC")) >= STRAIGHT_ROC_TH
        and row_metric(log, row, "targetSpeed") > 0
        and row_metric(log, row, "cntlog") >= 100,
    )

    def vals(pool: list[tuple[LogData, list[str]]], column: str) -> list[float]:
        return [row_metric(log, row, column) for log, row in pool if column in log.idx]

    def either_pwm_sat(pool: list[tuple[LogData, list[str]]]) -> list[bool]:
        return [
            abs(row_metric(log, row, "motorpwmL")) >= PWM_SAT_TH
            or abs(row_metric(log, row, "motorpwmR")) >= PWM_SAT_TH
            for log, row in pool
            if "motorpwmL" in log.idx and "motorpwmR" in log.idx
        ]

    def either_speed_sat(pool: list[tuple[LogData, list[str]]]) -> list[bool]:
        return [
            row_metric(log, row, "speedPwmSatL") >= 0.5
            or row_metric(log, row, "speedPwmSatR") >= 0.5
            for log, row in pool
            if "speedPwmSatL" in log.idx and "speedPwmSatR" in log.idx
        ]

    def req_abs(pool: list[tuple[LogData, list[str]]]) -> list[float]:
        values: list[float] = []
        for log, row in pool:
            if "speedPwmReqL" not in log.idx or "speedPwmReqR" not in log.idx:
                continue
            values.append(max(abs(row_metric(log, row, "speedPwmReqL")), abs(row_metric(log, row, "speedPwmReqR"))))
        return values

    lap_values = [log.last_cnt() for log in valid_logs]
    all_rows = rows
    return {
        "group": label,
        "battery_min": battery_min,
        "valid_logs": len(valid_logs),
        "logs": " ".join(str(log.number) for log in valid_logs),
        "battery_mean": mean([log.param("batteryVoltage_V") for log in valid_logs]),
        "lap_mean_ms": mean(lap_values),
        "overall_pwm_sat_pct": pct_true(either_pwm_sat(all_rows)),
        "overall_speedPwmSat_pct": pct_true(either_speed_sat(all_rows)),
        "high_idx_target_mean": mean(vals(high_rows, "targetSpeed")),
        "high_idx_enc_mean": mean(vals(high_rows, "encCurrentN")),
        "high_idx_speed_err_mean": mean(
            [
                row_metric(log, row, "encCurrentN") - row_metric(log, row, "targetSpeed")
                for log, row in high_rows
            ]
        ),
        "high_idx_pwm_sat_pct": pct_true(either_pwm_sat(high_rows)),
        "high_idx_speedPwmSat_pct": pct_true(either_speed_sat(high_rows)),
        "high_idx_req_abs_p95": p95_abs(req_abs(high_rows)),
        "auto3_main88_95_target_mean": mean(vals(high_main_rows, "targetSpeed")),
        "auto3_main88_95_base_mean": mean(vals(high_main_rows, "targetSpeedBase")),
        "auto3_main88_95_scale_mean": mean(vals(high_main_rows, "targetSpeedScale")),
        "straight_gyro_rms": rms(vals(straight_rows, "gyroVal_Z")),
        "straight_gyro_abs_p95": p95_abs(vals(straight_rows, "gyroVal_Z")),
        "straight_lineTrace_rms": rms(vals(straight_rows, "lineTraceCtrl")),
        "straight_lineTrace_abs_p95": p95_abs(vals(straight_rows, "lineTraceCtrl")),
        "slipLat_rate_pct": mean(vals(all_rows, "slipFlagLat")) * 100.0,
        "max_latched_reason": max(vals(all_rows, "emcStopLatchedReason") or [0.0]),
    }


def log_summary_rows(label: str, logs: list[LogData], battery_min: float) -> list[dict[str, str | float | int]]:
    out: list[dict[str, str | float | int]] = []
    for log in logs:
        high_rows = [
            row
            for row in log.rows
            if "optimalIndex" in log.idx and int(log.value(row, "optimalIndex")) in HIGH_INDEXES
        ]
        out.append(
            {
                "group": label,
                "log": log.number,
                "optimalTrace": log.param("optimalTrace"),
                "autoStart": log.param("autoStart"),
                "emcStop": log.param("emcStop"),
                "batteryVoltage_V": log.param("batteryVoltage_V"),
                "rows": len(log.rows),
                "last_cnt_ms": log.last_cnt(),
                "cnt_monotonic": int(log.cnt_monotonic()),
                "optimalIndex_max": log.optmax(),
                "valid_battery78": int(log.is_valid_second(7.8)),
                "valid_battery76": int(log.is_valid_second(7.6)),
                "has_targetSpeedBase": int("targetSpeedBase" in log.idx),
                "has_targetSpeedScale": int("targetSpeedScale" in log.idx),
                "has_emcStopLatched": int("emcStopLatchedReason" in log.idx),
                "high_idx_target_mean": mean([log.value(row, "targetSpeed") for row in high_rows]),
                "high_idx_target_base_mean": mean([log.value(row, "targetSpeedBase") for row in high_rows])
                if "targetSpeedBase" in log.idx
                else math.nan,
                "high_idx_target_scale_mean": mean([log.value(row, "targetSpeedScale") for row in high_rows])
                if "targetSpeedScale" in log.idx
                else math.nan,
            }
        )
    return out


def by_index_rows(logs: list[LogData], battery_min: float) -> list[dict[str, float | int]]:
    rows = selected_rows(logs, battery_min, auto_min=3)
    buckets: dict[int, list[tuple[LogData, list[str]]]] = {}
    for log, row in rows:
        idx = int(row_metric(log, row, "optimalIndex"))
        buckets.setdefault(idx, []).append((log, row))

    out: list[dict[str, float | int]] = []
    for idx in sorted(buckets):
        pool = buckets[idx]
        def vals(column: str) -> list[float]:
            return [row_metric(log, row, column) for log, row in pool if column in log.idx]

        out.append(
            {
                "optimalIndex": idx,
                "samples": len(pool),
                "targetSpeed_mean": mean(vals("targetSpeed")),
                "targetSpeedBase_mean": mean(vals("targetSpeedBase")),
                "targetSpeedScale_mean": mean(vals("targetSpeedScale")),
                "encCurrentN_mean": mean(vals("encCurrentN")),
                "speedErr_mean": mean(
                    [
                        row_metric(log, row, "encCurrentN") - row_metric(log, row, "targetSpeed")
                        for log, row in pool
                    ]
                ),
                "speedPwmReq_abs_p95": p95_abs(
                    [
                        max(abs(row_metric(log, row, "speedPwmReqL")), abs(row_metric(log, row, "speedPwmReqR")))
                        for log, row in pool
                        if "speedPwmReqL" in log.idx and "speedPwmReqR" in log.idx
                    ]
                ),
                "speedPwmSat_pct": pct_true(
                    [
                        row_metric(log, row, "speedPwmSatL") >= 0.5
                        or row_metric(log, row, "speedPwmSatR") >= 0.5
                        for log, row in pool
                        if "speedPwmSatL" in log.idx and "speedPwmSatR" in log.idx
                    ]
                ),
                "pwmSat_pct": pct_true(
                    [
                        abs(row_metric(log, row, "motorpwmL")) >= PWM_SAT_TH
                        or abs(row_metric(log, row, "motorpwmR")) >= PWM_SAT_TH
                        for log, row in pool
                        if "motorpwmL" in log.idx and "motorpwmR" in log.idx
                    ]
                ),
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
    if not numbers:
        return "none"
    return f"{min(numbers)}_{max(numbers)}"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--log-dir", type=Path, default=LOG_DIR)
    parser.add_argument("--old092", default="10110-10119,10145-10154")
    parser.add_argument("--ref093", default="10155-10184")
    parser.add_argument("--new", default="10185-10194")
    parser.add_argument("--out-dir", type=Path, default=Path("analysis"))
    args = parser.parse_args()

    old_nums = parse_ranges(args.old092)
    ref_nums = parse_ranges(args.ref093)
    new_nums = parse_ranges(args.new)
    old_logs = load_logs(old_nums, args.log_dir)
    ref_logs = load_logs(ref_nums, args.log_dir)
    new_logs = load_logs(new_nums, args.log_dir)
    old_label = range_label(old_nums)
    ref_label = range_label(ref_nums)
    new_label = range_label(new_nums)

    summary_rows = (
        log_summary_rows("old092", old_logs, 7.8)
        + log_summary_rows("ref093", ref_logs, 7.8)
        + log_summary_rows("debug092", new_logs, 7.8)
    )
    comparison_rows = [
        aggregate_group("old092_bat78", old_logs, 7.8),
        aggregate_group("ref093_bat78", ref_logs, 7.8),
        aggregate_group("debug092_bat78", new_logs, 7.8),
        aggregate_group("debug092_bat76_reference", new_logs, 7.6),
    ]

    summary_path = args.out_dir / f"log_{new_label}_speed_plan_scale_debug_summary.csv"
    comparison_path = args.out_dir / f"log_{old_label}_{ref_label}_vs_{new_label}_speed_plan_scale_debug_comparison.csv"
    by_index_path = args.out_dir / f"log_{new_label}_speed_plan_scale_by_index.csv"

    write_csv(summary_path, summary_rows)
    write_csv(
        comparison_path,
        comparison_rows,
    )
    write_csv(by_index_path, by_index_rows(new_logs, 7.6))

    print("wrote:")
    print(summary_path)
    print(comparison_path)
    print(by_index_path)


if __name__ == "__main__":
    main()
