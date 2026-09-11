#!/usr/bin/env python3
"""PATH REPLAY/SHORTCUTログを検証し、旧保存列または新形式の復元列を解析する。"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

from path_log_recovery import (
    PATH_MODES,
    CsvLog,
    parameter_int,
    parameter_number,
    parse_optimal_index,
    read_csv_log,
    recover_path_columns,
)


REQUIRED_COLUMNS = {
    "cntlog", "optimalIndex", "x", "y", "lineValid", "pathErrorY_mm",
    "pathErrorHeading_cdeg", "pathState",
}
SHORTCUT_MODE = 3
SHORTCUT_BUILD_STATUS_NAMES = {
    0: "not_requested", 1: "success", 2: "disabled_by_setting",
    3: "legal_geometry_invalid", 4: "no_corridor", 5: "offset_violation",
    6: "new_intersection", 7: "insufficient_reduction",
}


@dataclass
class RunSummary:
    path: Path
    optimal_trace: int
    auto_start: int
    emc_stop: int
    battery_voltage_v: float
    route_controller_version: int
    shortcut_build_status: int
    shortcut_corridor_count: int
    shortcut_reduction_mm: float
    samples: int
    lap_time_ms: int
    cntlog_valid: bool
    max_index_jump: int
    index_jump_ge5_count: int
    lateral_p95_mm: float
    lateral_max_mm: float
    heading_p95_deg: float
    heading_max_deg: float
    legal_margin_min_mm: float
    fallback_samples: int
    localization_lost_samples: int
    line_valid_ratio: float
    recovery_status: str
    recovery_reason: str
    recovery_source_log: int
    recovery_missing_samples: int


def percentile(values: list[float], ratio: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil(len(ordered) * ratio) - 1))
    return ordered[index]


def unwrap_cntlog_u16(values: list[int]) -> tuple[list[int], bool]:
    """16 bitのcntlog折り返しを展開し、時間差の妥当性も返す。"""
    if not values:
        return [], False
    unwrapped: list[int] = []
    offset = 0
    previous_raw = values[0]
    previous_unwrapped: int | None = None
    valid = True
    for raw in values:
        if raw < previous_raw:
            if previous_raw - raw > 0x8000:
                offset += 0x10000
            else:
                valid = False
        current = raw + offset
        if previous_unwrapped is not None:
            delta = current - previous_unwrapped
            if delta <= 0 or delta > 1000:
                valid = False
        unwrapped.append(current)
        previous_raw = raw
        previous_unwrapped = current
    return unwrapped, valid


def _float_values(log: CsvLog) -> list[dict[str, Any]]:
    missing = sorted(REQUIRED_COLUMNS - set(log.fields))
    if missing:
        raise ValueError(f"{log.path}: missing columns: {', '.join(missing)}")
    values: list[dict[str, Any]] = []
    for line_number, row in enumerate(log.rows, start=2):
        try:
            current = {
                name: (math.nan if name == "optimalIndex" and row[name].strip() == ""
                       else float(row[name]))
                for name in REQUIRED_COLUMNS
                if name != "optimalIndex"
            }
            raw_index = row["optimalIndex"].strip()
            current["optimalIndex"] = math.nan if raw_index == "" else float(raw_index)
        except (KeyError, ValueError, OverflowError) as exc:
            raise ValueError(f"{log.path}:{line_number}: invalid numeric field") from exc
        values.append(current)
    return values


def read_log(
    path: Path,
    source_log_dir: Path | None = None,
    shortcut_level: int | None = None,
    max_level: int | None = None,
) -> tuple[RunSummary, list[dict[str, Any]]]:
    log = read_csv_log(path)
    rows = _float_values(log)
    recovery = recover_path_columns(log, source_log_dir, shortcut_level, max_level)
    for row, restored in zip(rows, recovery.row_values):
        row.update(restored)

    cntlog_raw = [int(row["cntlog"]) for row in rows]
    cntlog, cntlog_valid = unwrap_cntlog_u16(cntlog_raw)
    route_count = recovery.route.route_count if recovery.route is not None else None
    route_indices = [parse_optimal_index(row["optimalIndex"], route_count) for row in rows]
    index_jumps = [now - before for before, now in zip(route_indices, route_indices[1:])
                   if now is not None and before is not None]
    lateral = [abs(float(row["pathErrorY_mm"])) for row in rows]
    heading = [abs(float(row["pathErrorHeading_cdeg"])) * 0.01 for row in rows]
    margins = [float(row["pathLegalMargin_mm"]) for row in rows if math.isfinite(float(row["pathLegalMargin_mm"]))]
    states = [int(row["pathState"]) for row in rows]
    line_valid = [int(row["lineValid"]) != 0 for row in rows]

    optimal_trace = parameter_int(log.parameters, "optimalTrace", -1)
    auto_start = parameter_int(log.parameters, "autoStart", 0)
    emc_stop = parameter_int(log.parameters, "emcStop", -1)
    summary = RunSummary(
        path=path,
        optimal_trace=-1 if optimal_trace is None else optimal_trace,
        auto_start=0 if auto_start is None else auto_start,
        emc_stop=-1 if emc_stop is None else emc_stop,
        battery_voltage_v=parameter_number(log.parameters, "batteryVoltage_V"),
        route_controller_version=parameter_int(log.parameters, "routeControllerVersion", -1),
        shortcut_build_status=parameter_int(log.parameters, "shortcutBuildStatus", -1),
        shortcut_corridor_count=parameter_int(log.parameters, "shortcutCorridorCount", -1),
        shortcut_reduction_mm=parameter_number(log.parameters, "shortcutReduction_mm"),
        samples=len(rows),
        lap_time_ms=cntlog[-1],
        cntlog_valid=cntlog_valid,
        max_index_jump=max([0, *index_jumps]),
        index_jump_ge5_count=sum(jump >= 5 for jump in index_jumps),
        lateral_p95_mm=percentile(lateral, 0.95),
        lateral_max_mm=max(lateral),
        heading_p95_deg=percentile(heading, 0.95),
        heading_max_deg=max(heading),
        legal_margin_min_mm=(math.nan if recovery.missing_samples > 0 else (min(margins) if margins else math.nan)),
        fallback_samples=sum(state in (2, 3) for state in states),
        localization_lost_samples=sum(state == 4 for state in states),
        line_valid_ratio=sum(line_valid) / len(line_valid),
        recovery_status=recovery.status,
        recovery_reason=recovery.reason,
        recovery_source_log=recovery.source_log,
        recovery_missing_samples=recovery.missing_samples,
    )
    return summary, rows


def write_summary(path: Path, summaries: Iterable[RunSummary]) -> None:
    fields = list(RunSummary.__dataclass_fields__)
    with path.open("w", encoding="utf-8", newline="") as destination:
        writer = csv.DictWriter(destination, fieldnames=fields)
        writer.writeheader()
        for summary in summaries:
            row = summary.__dict__.copy()
            row["path"] = str(summary.path)
            writer.writerow(row)


def reference_segments(rows: list[dict[str, Any]]) -> list[list[tuple[float, float]]]:
    """欠測行を跨がずに参照経路を連続区間へ分割する。"""
    segments: list[list[tuple[float, float]]] = []
    current: list[tuple[float, float]] = []
    for row in rows:
        x = float(row["linePointX_mm"])
        y = float(row["linePointY_mm"])
        if math.isfinite(x) and math.isfinite(y):
            current.append((x, y))
        elif current:
            segments.append(current)
            current = []
    if current:
        segments.append(current)
    return segments


def write_xy_plot(path: Path, runs: list[tuple[RunSummary, list[dict[str, Any]]]]) -> bool:
    try:
        import os
        import tempfile
        os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "robotrace-matplotlib"))
        import matplotlib
        matplotlib.use("Agg")  # ファイル出力ではGUIやTcl/Tkを必要としない。
        import matplotlib.pyplot as plt
    except ImportError:
        return False

    figure, axis = plt.subplots(figsize=(7, 7))
    reference_available = False
    missing_notes: list[str] = []
    for summary, rows in runs:
        label = summary.path.stem
        axis.plot([row["x"] for row in rows], [row["y"] for row in rows], linewidth=1.0, label=f"{label} robot")
        segments = reference_segments(rows)
        for segment_index, segment in enumerate(segments):
            reference_available = True
            axis.plot([point[0] for point in segment], [point[1] for point in segment],
                      linestyle="--", linewidth=1.0,
                      label=f"{label} reference" if segment_index == 0 else None)
        if summary.recovery_missing_samples > 0:
            missing_notes.append(f"{label}: reference missing {summary.recovery_missing_samples} samples")
    if not reference_available:
        if runs and all(summary.recovery_status == "not_applicable" for summary, _ in runs):
            note = "reference route not applicable"
        else:
            note = "reference route unavailable"
        axis.text(0.02, 0.98, note, transform=axis.transAxes, va="top", color="tab:red")
    elif missing_notes:
        axis.text(0.02, 0.98, "\n".join(missing_notes), transform=axis.transAxes,
                  va="top", color="tab:red")
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x [mm]")
    axis.set_ylabel("y [mm]")
    axis.grid(True)
    axis.legend(fontsize=7)
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)
    return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("analysis"))
    parser.add_argument("--source-log-dir", type=Path, help="元ログを探すフォルダ。既定は二次ログと同じフォルダ")
    parser.add_argument("--shortcut-level", type=int, help="旧形式または設定欠落時の復元試験用要求Level")
    parser.add_argument("--max-level", type=int, help="経路生成時maxLevelが欠落した復元試験用設定")
    parser.add_argument("--require-autostart-five", action="store_true")
    parser.add_argument("--allow-invalid", action="store_true",
                        help="緊急停止、cntlog異常、5点以上のindexジャンプがあっても解析結果を出力する")
    args = parser.parse_args()

    runs = [read_log(path, args.source_log_dir, args.shortcut_level, args.max_level) for path in args.logs]
    summaries = [run[0] for run in runs]
    modes = {summary.optimal_trace for summary in summaries}
    if not modes <= PATH_MODES:
        raise SystemExit(f"path mode only (3 or 4), got {sorted(modes)}")
    if len(modes) != 1:
        raise SystemExit(f"do not compare different modes: {sorted(modes)}")
    controller_versions = {summary.route_controller_version for summary in summaries}
    if 4 in controller_versions and any(version != 4 for version in controller_versions):
        print(
            "warning: pathErrorHeading_cdeg is lookahead-heading error through controller version 3 "
            "and nearest-heading error from version 4; do not compare heading percentiles directly",
            file=sys.stderr,
        )
    invalid: list[str] = []
    for summary in summaries:
        reasons: list[str] = []
        if summary.emc_stop != 0:
            reasons.append(f"emcStop={summary.emc_stop}")
        if not summary.cntlog_valid:
            reasons.append("cntlog")
        if summary.index_jump_ge5_count > 0:
            reasons.append(f"indexJump>=5:{summary.index_jump_ge5_count}")
        if summary.route_controller_version >= 11 and summary.optimal_trace == SHORTCUT_MODE:
            if summary.shortcut_build_status != 1:
                reasons.append(f"shortcutBuildStatus={summary.shortcut_build_status}")
            if summary.shortcut_corridor_count < 1:
                reasons.append(f"shortcutCorridorCount={summary.shortcut_corridor_count}")
            if summary.shortcut_reduction_mm < 5.0:
                reasons.append(f"shortcutReduction={summary.shortcut_reduction_mm}")
        if reasons:
            invalid.append(f"{summary.path.name} ({', '.join(reasons)})")
    if invalid and not args.allow_invalid:
        raise SystemExit(f"invalid runs: {', '.join(invalid)}")
    if args.require_autostart_five:
        auto_starts = sorted(summary.auto_start for summary in summaries)
        if auto_starts != [1, 2, 3, 4, 5]:
            raise SystemExit(f"autoStart must be exactly 1..5, got {auto_starts}")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    stems = [summary.path.stem for summary in summaries]
    prefix = f"path_{stems[0]}" if len(stems) == 1 else f"path_{stems[0]}_{stems[-1]}"
    summary_path = args.output_dir / f"{prefix}_summary.csv"
    plot_path = args.output_dir / f"{prefix}_xy.png"
    write_summary(summary_path, summaries)
    plotted = write_xy_plot(plot_path, runs)

    for summary in summaries:
        shortcut_status_name = SHORTCUT_BUILD_STATUS_NAMES.get(
            summary.shortcut_build_status, str(summary.shortcut_build_status)
        )
        if summary.recovery_status == "not_applicable":
            margin = "対象外"
        elif math.isnan(summary.legal_margin_min_mm):
            margin = "判定不能"
        else:
            margin = f"{summary.legal_margin_min_mm:.2f}mm"
        print(
            f"{summary.path.name}: mode={summary.optimal_trace} controller={summary.route_controller_version} "
            f"shortcut_status={shortcut_status_name} corridors={summary.shortcut_corridor_count} "
            f"reduction={summary.shortcut_reduction_mm:.2f}mm emc={summary.emc_stop} "
            f"lat_p95={summary.lateral_p95_mm:.2f}mm heading_p95={summary.heading_p95_deg:.2f}deg "
            f"margin_min={margin} fallback={summary.fallback_samples} "
            f"jump_max={summary.max_index_jump} jump_ge5={summary.index_jump_ge5_count} "
            f"recovery={summary.recovery_status} missing={summary.recovery_missing_samples} "
            f"source={summary.recovery_source_log} reason={summary.recovery_reason}"
        )
    print(summary_path)
    if plotted:
        print(plot_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
