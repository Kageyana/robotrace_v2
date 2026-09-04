#!/usr/bin/env python3
"""経路再走行・ショートカット走行ログを同一モード内で検証する。"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


REQUIRED_COLUMNS = {
    "cntlog",
    "x",
    "y",
    "linePointX_mm",
    "linePointY_mm",
    "lineValid",
    "pathErrorY_mm",
    "pathErrorHeading_cdeg",
    "pathState",
    "pathLegalMargin_mm",
}
PATH_MODES = {3, 4}


@dataclass
class RunSummary:
    path: Path
    optimal_trace: int
    auto_start: int
    emc_stop: int
    battery_voltage_v: float
    samples: int
    lap_time_ms: int
    cntlog_valid: bool
    lateral_p95_mm: float
    lateral_max_mm: float
    heading_p95_deg: float
    heading_max_deg: float
    legal_margin_min_mm: float
    fallback_samples: int
    localization_lost_samples: int
    line_valid_ratio: float


def parse_parameter(cell: str) -> tuple[str, str] | None:
    if "=" not in cell:
        return None
    name, value = cell.split("=", 1)
    name = name.strip()
    if not name:
        return None
    return name, value.strip()


def percentile(values: list[float], ratio: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil(len(ordered) * ratio) - 1))
    return ordered[index]


def parameter_number(parameters: dict[str, str], name: str, default: float = math.nan) -> float:
    try:
        return float(parameters[name])
    except (KeyError, ValueError):
        return default


def read_log(path: Path) -> tuple[RunSummary, list[dict[str, float]]]:
    with path.open("r", encoding="utf-8-sig", newline="") as source:
        reader = csv.reader(source)
        try:
            header = next(reader)
        except StopIteration as exc:
            raise ValueError(f"{path}: empty CSV") from exc

        columns = {name.strip(): index for index, name in enumerate(header) if name.strip() in REQUIRED_COLUMNS}
        missing = sorted(REQUIRED_COLUMNS - columns.keys())
        if missing:
            raise ValueError(f"{path}: missing columns: {', '.join(missing)}")

        parameters: dict[str, str] = {}
        for cell in header:
            parsed = parse_parameter(cell)
            if parsed is not None:
                parameters[parsed[0]] = parsed[1]

        rows: list[dict[str, float]] = []
        max_index = max(columns.values())
        for line_number, row in enumerate(reader, start=2):
            if len(row) <= max_index:
                continue
            try:
                values = {name: float(row[index]) for name, index in columns.items()}
            except ValueError as exc:
                raise ValueError(f"{path}:{line_number}: invalid numeric field") from exc
            rows.append(values)

    if not rows:
        raise ValueError(f"{path}: no data rows")

    cntlog = [int(row["cntlog"]) for row in rows]
    cntlog_valid = all(now > before and now - before <= 1000 for before, now in zip(cntlog, cntlog[1:]))
    lateral = [abs(row["pathErrorY_mm"]) for row in rows]
    heading = [abs(row["pathErrorHeading_cdeg"]) * 0.01 for row in rows]
    margins = [row["pathLegalMargin_mm"] for row in rows]
    states = [int(row["pathState"]) for row in rows]
    line_valid = [int(row["lineValid"]) != 0 for row in rows]

    optimal_trace = int(round(parameter_number(parameters, "optimalTrace", -1)))
    auto_start = int(round(parameter_number(parameters, "autoStart", 0)))
    emc_stop = int(round(parameter_number(parameters, "emcStop", -1)))
    summary = RunSummary(
        path=path,
        optimal_trace=optimal_trace,
        auto_start=auto_start,
        emc_stop=emc_stop,
        battery_voltage_v=parameter_number(parameters, "batteryVoltage_V"),
        samples=len(rows),
        lap_time_ms=cntlog[-1],
        cntlog_valid=cntlog_valid,
        lateral_p95_mm=percentile(lateral, 0.95),
        lateral_max_mm=max(lateral),
        heading_p95_deg=percentile(heading, 0.95),
        heading_max_deg=max(heading),
        legal_margin_min_mm=min(margins),
        fallback_samples=sum(state in (2, 3) for state in states),
        localization_lost_samples=sum(state == 4 for state in states),
        line_valid_ratio=sum(line_valid) / len(line_valid),
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


def write_xy_plot(path: Path, runs: list[tuple[RunSummary, list[dict[str, float]]]]) -> bool:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return False

    figure, axis = plt.subplots(figsize=(7, 7))
    for summary, rows in runs:
        label = summary.path.stem
        axis.plot([row["linePointX_mm"] for row in rows], [row["linePointY_mm"] for row in rows],
                  linestyle="--", linewidth=1.0, label=f"{label} line")
        axis.plot([row["x"] for row in rows], [row["y"] for row in rows],
                  linewidth=1.0, label=f"{label} robot")
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
    parser.add_argument("--require-autostart-five", action="store_true")
    args = parser.parse_args()

    runs = [read_log(path) for path in args.logs]
    summaries = [run[0] for run in runs]
    modes = {summary.optimal_trace for summary in summaries}
    if not modes <= PATH_MODES:
        raise SystemExit(f"path mode only (3 or 4), got {sorted(modes)}")
    if len(modes) != 1:
        raise SystemExit(f"do not compare different modes: {sorted(modes)}")
    invalid = [summary.path.name for summary in summaries if summary.emc_stop != 0 or not summary.cntlog_valid]
    if invalid:
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
        print(
            f"{summary.path.name}: mode={summary.optimal_trace} emc={summary.emc_stop} "
            f"lat_p95={summary.lateral_p95_mm:.2f}mm heading_p95={summary.heading_p95_deg:.2f}deg "
            f"margin_min={summary.legal_margin_min_mm:.2f}mm fallback={summary.fallback_samples}"
        )
    print(summary_path)
    if plotted:
        print(plot_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
