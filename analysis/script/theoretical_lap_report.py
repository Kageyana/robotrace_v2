"""Generate theoretical-lap reports for BOOST_DISTANCE autoStart logs."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from statistics import fmean

try:
    from .theoretical_lap import (
        CourseGeometry,
        LogData,
        TheoreticalLapError,
        build_course_geometry,
        calculate_theoretical_lap,
        geometry_rows,
        number,
        read_log,
    )
except ImportError:
    from theoretical_lap import (
        CourseGeometry,
        LogData,
        TheoreticalLapError,
        build_course_geometry,
        calculate_theoretical_lap,
        geometry_rows,
        number,
        read_log,
    )


DEFAULT_LOG_DIR = Path(r"F:\Dropbox\Document\robotrace\Log\v2")
DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parents[1]
START_VOLTAGE_MIN = 7.3
MAX_CNT_GAP_MS = 30.0
MIN_ROWS = 850
MIN_SECONDARY_OPTIMAL_INDEX = 180


RUN_FIELDS = (
    "base_log",
    "log",
    "autoStart",
    "course_distance_mm",
    "theoretical_lap_ms",
    "instantaneous_theoretical_lap_ms",
    "accel_decel_adjustment_ms",
    "actual_lap_ms",
    "lap_gap_ms",
    "lap_gap_pct",
    "acceleration_mps2",
    "deceleration_mps2",
    "initial_speed_mps",
    "acceleration_transition_count",
    "deceleration_transition_count",
    "acceleration_unreached_count",
    "deceleration_unreached_count",
    "target_index_coverage_pct",
    "target_index_filled_count",
    "batteryVoltage_V",
)

AUTOSTART_FIELDS = (
    "autoStart",
    "n",
    "theoretical_lap_mean_ms",
    "instantaneous_theoretical_lap_mean_ms",
    "accel_decel_adjustment_mean_ms",
    "actual_lap_mean_ms",
    "lap_gap_mean_ms",
    "lap_gap_mean_pct",
    "lap_gap_min_ms",
    "lap_gap_max_ms",
)

EXCLUDED_FIELDS = ("scope", "base_log", "log", "autoStart", "reason")


def validate_log(log: LogData, *, first_run: bool) -> list[str]:
    reasons: list[str] = []
    expected_trace = 0 if first_run else 2
    expected_auto = 1 if first_run else None
    trace = log.params.get("optimalTrace", math.nan)
    auto_start = log.params.get("autoStart", math.nan)
    if trace != expected_trace:
        reasons.append(f"optimalTrace={trace:g}")
    if expected_auto is not None and auto_start != expected_auto:
        reasons.append(f"autoStart={auto_start:g}")
    if log.params.get("emcStop", math.nan) != 0:
        reasons.append(f"emcStop={log.params.get('emcStop', math.nan):g}")
    battery_voltage = log.params.get("batteryVoltage_V", math.nan)
    if not math.isfinite(battery_voltage) or battery_voltage < START_VOLTAGE_MIN:
        reasons.append(f"batteryVoltage_V={battery_voltage:g}")
    for overflow_name in ("dbgOverflowFinal", "logOverflowFinal"):
        if overflow_name in log.params and log.params[overflow_name] != 0:
            reasons.append(f"{overflow_name}={log.params[overflow_name]:g}")

    required = {"cntlog", "encTotalOptimal"}
    if first_run:
        required.update(("ROC", "gyroVal_Z", "x", "y"))
    else:
        required.update(("encCurrentN", "targetSpeed", "optimalIndex"))
        for parameter in ("tgtParam.acceleF", "tgtParam.acceleD"):
            value = log.params.get(parameter, math.nan)
            if not math.isfinite(value) or value <= 0:
                reasons.append(f"{parameter}={value:g}")
    missing = sorted(required.difference(log.columns))
    if missing:
        reasons.append("missing=" + "|".join(missing))
        return reasons

    if len(log.rows) < MIN_ROWS:
        reasons.append(f"rows={len(log.rows)}")
    cnt = [number(row.get("cntlog")) for row in log.rows]
    if not cnt or any(not math.isfinite(value) for value in cnt):
        reasons.append("invalid_cntlog")
    else:
        gaps = [current - previous for previous, current in zip(cnt, cnt[1:])]
        if any(gap <= 0 for gap in gaps):
            reasons.append("cntlog_nonmonotonic")
        if gaps and max(gaps) > MAX_CNT_GAP_MS:
            reasons.append(f"max_cnt_gap_ms={max(gaps):g}")

    if first_run:
        distance = [number(row.get("encTotalOptimal")) for row in log.rows]
        if (
            not distance
            or any(not math.isfinite(value) for value in distance)
            or distance[-1] <= 0
        ):
            reasons.append("invalid_encTotalOptimal")
        elif any(current <= previous for previous, current in zip(distance, distance[1:])):
            reasons.append("encTotalOptimal_nonmonotonic")
    else:
        indices = [number(row.get("optimalIndex")) for row in log.rows]
        max_index = max(indices, default=math.nan)
        if not math.isfinite(max_index) or max_index < MIN_SECONDARY_OPTIMAL_INDEX:
            reasons.append(f"optimalIndex_max={max_index:g}")
        if auto_start not in (2, 3, 4, 5):
            reasons.append(f"autoStart={auto_start:g}")
    return reasons


def group_auto_start_sets(logs: list[LogData]) -> list[list[LogData]]:
    groups: list[list[LogData]] = []
    current: list[LogData] = []
    for log in sorted(logs, key=lambda item: item.number):
        auto_start_value = log.params.get("autoStart", math.nan)
        auto_start = (
            int(auto_start_value)
            if math.isfinite(auto_start_value) and auto_start_value.is_integer()
            else -1
        )
        if auto_start == 1:
            if current:
                groups.append(current)
            current = [log]
        elif current:
            current.append(log)
        else:
            groups.append([log])
    if current:
        groups.append(current)
    return groups


def _rounded(value: float) -> float:
    return round(value, 6)


def result_row(
    geometry: CourseGeometry,
    secondary: LogData,
) -> dict[str, int | float]:
    result = calculate_theoretical_lap(geometry, secondary)
    return {
        "base_log": result.base_log,
        "log": result.secondary_log,
        "autoStart": int(secondary.params["autoStart"]),
        "course_distance_mm": _rounded(result.course_distance_mm),
        "theoretical_lap_ms": _rounded(result.theoretical_lap_ms),
        "instantaneous_theoretical_lap_ms": _rounded(
            result.instantaneous_theoretical_lap_ms
        ),
        "accel_decel_adjustment_ms": _rounded(result.accel_decel_adjustment_ms),
        "actual_lap_ms": _rounded(result.actual_lap_ms),
        "lap_gap_ms": _rounded(result.lap_gap_ms),
        "lap_gap_pct": _rounded(result.lap_gap_pct),
        "acceleration_mps2": _rounded(result.acceleration_mps2),
        "deceleration_mps2": _rounded(result.deceleration_mps2),
        "initial_speed_mps": _rounded(result.initial_speed_mps),
        "acceleration_transition_count": result.acceleration_transition_count,
        "deceleration_transition_count": result.deceleration_transition_count,
        "acceleration_unreached_count": result.acceleration_unreached_count,
        "deceleration_unreached_count": result.deceleration_unreached_count,
        "target_index_coverage_pct": _rounded(result.target_index_coverage_pct),
        "target_index_filled_count": result.target_index_filled_count,
        "batteryVoltage_V": secondary.params.get("batteryVoltage_V", math.nan),
    }


def summarize_by_autostart(rows: list[dict[str, int | float]]) -> list[dict[str, int | float]]:
    output: list[dict[str, int | float]] = []
    for auto_start in (2, 3, 4, 5):
        selected = [row for row in rows if row["autoStart"] == auto_start]
        if not selected:
            continue
        gaps = [float(row["lap_gap_ms"]) for row in selected]
        output.append(
            {
                "autoStart": auto_start,
                "n": len(selected),
                "theoretical_lap_mean_ms": _rounded(
                    fmean(float(row["theoretical_lap_ms"]) for row in selected)
                ),
                "instantaneous_theoretical_lap_mean_ms": _rounded(
                    fmean(
                        float(row["instantaneous_theoretical_lap_ms"])
                        for row in selected
                    )
                ),
                "accel_decel_adjustment_mean_ms": _rounded(
                    fmean(
                        float(row["accel_decel_adjustment_ms"])
                        for row in selected
                    )
                ),
                "actual_lap_mean_ms": _rounded(
                    fmean(float(row["actual_lap_ms"]) for row in selected)
                ),
                "lap_gap_mean_ms": _rounded(fmean(gaps)),
                "lap_gap_mean_pct": _rounded(
                    fmean(float(row["lap_gap_pct"]) for row in selected)
                ),
                "lap_gap_min_ms": _rounded(min(gaps)),
                "lap_gap_max_ms": _rounded(max(gaps)),
            }
        )
    return output


def write_csv(path: Path, rows: list[dict], fieldnames: tuple[str, ...] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    selected_fields = list(fieldnames or (tuple(rows[0].keys()) if rows else ()))
    with path.open("w", encoding="utf-8", newline="") as file:
        if not selected_fields:
            return
        writer = csv.DictWriter(file, fieldnames=selected_fields)
        writer.writeheader()
        writer.writerows(rows)


def load_range(log_dir: Path, start: int, end: int) -> tuple[list[LogData], list[dict]]:
    logs: list[LogData] = []
    excluded: list[dict] = []
    for log_number in range(start, end + 1):
        path = log_dir / f"{log_number}.csv"
        if not path.exists():
            excluded.append(
                {
                    "scope": "range",
                    "base_log": "",
                    "log": log_number,
                    "autoStart": "",
                    "reason": "missing_file",
                }
            )
            continue
        try:
            logs.append(read_log(path, log_number))
        except (OSError, TheoreticalLapError) as exc:
            excluded.append(
                {
                    "scope": "range",
                    "base_log": "",
                    "log": log_number,
                    "autoStart": "",
                    "reason": str(exc),
                }
            )
    return logs, excluded


def analyze_complete_sets(logs: list[LogData]) -> tuple[list[dict], list[dict], list[dict]]:
    run_rows: list[dict] = []
    course_rows: list[dict] = []
    excluded: list[dict] = []
    for group in group_auto_start_sets(logs):
        sequence = [int(log.params.get("autoStart", -1)) for log in group]
        base_number = group[0].number if sequence and sequence[0] == 1 else ""
        if sequence != [1, 2, 3, 4, 5]:
            excluded.append(
                {
                    "scope": "set",
                    "base_log": base_number,
                    "log": f"{group[0].number}-{group[-1].number}",
                    "autoStart": "|".join(map(str, sequence)),
                    "reason": "incomplete_autoStart_set",
                }
            )
            continue

        validation: list[str] = []
        for index, log in enumerate(group):
            reasons = validate_log(log, first_run=(index == 0))
            if reasons:
                validation.append(f"{log.number}:{'|'.join(reasons)}")
        if validation:
            excluded.append(
                {
                    "scope": "set",
                    "base_log": base_number,
                    "log": f"{group[0].number}-{group[-1].number}",
                    "autoStart": "1|2|3|4|5",
                    "reason": ";".join(validation),
                }
            )
            continue

        base = group[0]
        try:
            geometry = build_course_geometry(base)
            new_rows = [result_row(geometry, secondary) for secondary in group[1:]]
        except TheoreticalLapError as exc:
            excluded.append(
                {
                    "scope": "set",
                    "base_log": base.number,
                    "log": f"{group[0].number}-{group[-1].number}",
                    "autoStart": "1|2|3|4|5",
                    "reason": str(exc),
                }
            )
            continue
        run_rows.extend(new_rows)
        course_rows.extend(geometry_rows(geometry))
    return run_rows, course_rows, excluded


def analyze_explicit(base: LogData, secondary_logs: list[LogData]) -> tuple[list[dict], list[dict], list[dict]]:
    excluded: list[dict] = []
    base_reasons = validate_log(base, first_run=True)
    if base_reasons:
        return [], [], [
            {
                "scope": "explicit",
                "base_log": base.number,
                "log": base.number,
                "autoStart": 1,
                "reason": "|".join(base_reasons),
            }
        ]

    geometry = build_course_geometry(base)
    run_rows: list[dict] = []
    for secondary in secondary_logs:
        reasons = validate_log(secondary, first_run=False)
        if reasons:
            excluded.append(
                {
                    "scope": "explicit",
                    "base_log": base.number,
                    "log": secondary.number,
                    "autoStart": secondary.params.get("autoStart", ""),
                    "reason": "|".join(reasons),
                }
            )
            continue
        try:
            run_rows.append(result_row(geometry, secondary))
        except TheoreticalLapError as exc:
            excluded.append(
                {
                    "scope": "explicit",
                    "base_log": base.number,
                    "log": secondary.number,
                    "autoStart": secondary.params.get("autoStart", ""),
                    "reason": str(exc),
                }
            )
    return run_rows, geometry_rows(geometry), excluded


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--start", type=int)
    parser.add_argument("--end", type=int)
    parser.add_argument("--base-log", type=int)
    parser.add_argument("--secondary-logs", type=int, nargs="+")
    args = parser.parse_args()
    range_mode = args.start is not None or args.end is not None
    explicit_mode = args.base_log is not None or args.secondary_logs is not None
    if range_mode == explicit_mode:
        parser.error("select either --start/--end or --base-log/--secondary-logs")
    if range_mode and (args.start is None or args.end is None or args.start > args.end):
        parser.error("--start and --end must define an ascending range")
    if explicit_mode and (args.base_log is None or not args.secondary_logs):
        parser.error("explicit mode requires --base-log and --secondary-logs")
    return args


def main() -> int:
    args = parse_args()
    excluded: list[dict] = []
    if args.start is not None:
        logs, load_excluded = load_range(args.log_dir, args.start, args.end)
        run_rows, course_rows, analysis_excluded = analyze_complete_sets(logs)
        excluded.extend(load_excluded)
        excluded.extend(analysis_excluded)
        label = f"{args.start}_{args.end}"
    else:
        base = read_log(args.log_dir / f"{args.base_log}.csv", args.base_log)
        secondary_logs = [
            read_log(args.log_dir / f"{number_}.csv", number_)
            for number_ in args.secondary_logs
        ]
        run_rows, course_rows, excluded = analyze_explicit(base, secondary_logs)
        all_numbers = [args.base_log, *args.secondary_logs]
        label = f"{min(all_numbers)}_{max(all_numbers)}"

    by_autostart = summarize_by_autostart(run_rows)
    prefix = args.output_dir / f"log_{label}"
    write_csv(
        Path(f"{prefix}_theoretical_lap_by_run.csv"),
        run_rows,
        RUN_FIELDS,
    )
    write_csv(
        Path(f"{prefix}_theoretical_lap_by_autostart.csv"),
        by_autostart,
        AUTOSTART_FIELDS,
    )
    write_csv(Path(f"{prefix}_course_geometry.csv"), course_rows)
    write_csv(
        Path(f"{prefix}_theoretical_lap_excluded.csv"),
        excluded,
        EXCLUDED_FIELDS,
    )

    print(f"valid_secondary_logs={len(run_rows)} excluded={len(excluded)}")
    for row in run_rows:
        print(
            f"log={row['log']} autoStart={row['autoStart']} "
            f"theoretical={row['theoretical_lap_ms']:.1f}ms "
            f"accel_adjustment={row['accel_decel_adjustment_ms']:+.1f}ms "
            f"actual={row['actual_lap_ms']:.1f}ms "
            f"gap={row['lap_gap_ms']:.1f}ms ({row['lap_gap_pct']:.2f}%)"
        )
    for row in excluded:
        print(f"excluded {row['log']}: {row['reason']}")
    return 0 if run_rows else 1


if __name__ == "__main__":
    raise SystemExit(main())
