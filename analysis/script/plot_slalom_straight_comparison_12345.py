#!/usr/bin/env python3
"""指定時間のスラロームを直線回廊化し、現行方式と重ねて表示する。"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib.pyplot as plt

from compare_shortcut_routes_12345 import (
    c_round,
    distance,
    expand_anchors,
    offsets,
    read_raw,
    resample,
    route_length,
    smooth_route,
)
from path_log_recovery import read_csv_log


LEGAL_OFFSET_MM = 49.5
PROPOSED_OFFSET_LIMIT_MM = 34.0
TRANSITION_POINTS = 3  # 40 mm間隔で前後120 mm


def smoothstep(value: float) -> float:
    value = min(1.0, max(0.0, value))
    return value * value * (3.0 - 2.0 * value)


def read_interval(path: Path, start_ms: int, end_ms: int) -> tuple[tuple[float, float], tuple[float, float]]:
    first: tuple[float, float] | None = None
    interval: list[tuple[float, float]] = []
    for row in read_csv_log(path).rows:
        try:
            point = (float(row["x"]), float(row["y"]))
            time_ms = int(float(row["cntlog"]))
        except (KeyError, TypeError, ValueError):
            continue
        if first is None:
            first = point
        if start_ms <= time_ms <= end_ms:
            interval.append(point)
    if first is None or len(interval) < 2:
        raise ValueError("指定時間範囲の経路点が不足しています")
    return ((interval[0][0] - first[0], interval[0][1] - first[1]),
            (interval[-1][0] - first[0], interval[-1][1] - first[1]))


def nearest_index(route: list[list[int]], point: tuple[float, float]) -> int:
    return min(range(len(route)),
               key=lambda index: (route[index][0] - point[0]) ** 2 + (route[index][1] - point[1]) ** 2)


def build_straight_corridor(route: list[list[int]], begin: int, end: int) -> list[list[int]]:
    result = [point[:] for point in route]
    span = end - begin
    for index in range(begin, end + 1):
        progress = (index - begin) / span
        chord_x = route[begin][0] + (route[end][0] - route[begin][0]) * progress
        chord_y = route[begin][1] + (route[end][1] - route[begin][1]) * progress
        entry_weight = smoothstep((index - begin) / TRANSITION_POINTS)
        exit_weight = smoothstep((end - index) / TRANSITION_POINTS)
        weight = min(entry_weight, exit_weight)
        move_x = (chord_x - route[index][0]) * weight
        move_y = (chord_y - route[index][1]) * weight
        move = math.hypot(move_x, move_y)
        if move > PROPOSED_OFFSET_LIMIT_MM:
            move_x *= PROPOSED_OFFSET_LIMIT_MM / move
            move_y *= PROPOSED_OFFSET_LIMIT_MM / move
        result[index][0] = c_round(route[index][0] + move_x)
        result[index][1] = c_round(route[index][1] + move_y)
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--start-ms", type=int, default=29400)
    parser.add_argument("--end-ms", type=int, default=30700)
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args()

    raw = read_raw(args.log)
    route, marker_anchors = resample(raw)
    interval_start, interval_end = read_interval(args.log, args.start_ms, args.end_ms)
    begin = nearest_index(route, interval_start)
    end = nearest_index(route, interval_end)
    if begin >= end:
        raise ValueError("指定時間範囲に対応する経路indexが逆転しています")

    anchors = expand_anchors(marker_anchors)
    current = smooth_route(route, [0.0 if anchor else 1.0 for anchor in anchors])
    proposed = build_straight_corridor(route, begin, end)

    source_segment = route[begin:end + 1]
    current_segment = current[begin:end + 1]
    proposed_segment = proposed[begin:end + 1]
    proposed_offset = max(offsets(source_segment, proposed_segment))
    source_length = route_length(source_segment)
    current_length = route_length(current_segment)
    proposed_length = route_length(proposed_segment)
    remaining = LEGAL_OFFSET_MM - proposed_offset

    plt.rcParams.update({
        "font.family": "Yu Gothic",
        "axes.unicode_minus": False,
        "font.size": 11,
    })
    figure, axis = plt.subplots(figsize=(10.5, 9.0), constrained_layout=True)
    if figure.canvas.manager is not None:
        figure.canvas.manager.set_window_title("12345 スラローム直線回廊比較")

    axis.plot([point[0] for point in route],
              [point[1] for point in route],
              color="#4b5563", linewidth=4.0, linestyle=(0, (6, 3)),
              label="一次走行経路（Level 0）", zorder=1)
    axis.plot([point[0] for point in current],
              [point[1] for point in current],
              color="#dc2626", linewidth=2.0,
              label="現行Elastic Band候補", zorder=2)
    axis.plot([point[0] for point in proposed],
              [point[1] for point in proposed],
              color="#2563eb", linewidth=2.4,
              label="提案：直線回廊＋前後120 mm接続（変位34 mm制限）", zorder=3)
    axis.plot([point[0] for point in proposed[begin:end + 1]],
              [point[1] for point in proposed[begin:end + 1]],
              color="#2563eb", linewidth=4.0, zorder=4)
    axis.scatter([route[begin][0], route[end][0]], [route[begin][1], route[end][1]],
                 color="#111827", marker="o", s=36, zorder=5, label="直線化区間の入口・出口")

    axis.set_title(f"一次走行12345：{args.start_ms / 1000:.1f}～{args.end_ms / 1000:.1f}秒の経路比較",
                   fontsize=15)
    axis.set_xlabel("X [mm]")
    axis.set_ylabel("Y [mm]")
    axis.set_aspect("equal", adjustable="box")
    axis.margins(0.03)
    axis.grid(True, linewidth=0.5, alpha=0.35)
    axis.legend(loc="upper center", ncol=2, framealpha=0.94)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=180, bbox_inches="tight")
    print(f"indices={begin}..{end}")
    print(f"source_mm={source_length:.3f} current_mm={current_length:.3f} proposed_mm={proposed_length:.3f}")
    print(f"max_offset_mm={proposed_offset:.3f} legal_remaining_mm={remaining:.3f}")
    print(args.output)
    if not args.no_show:
        plt.show()
    plt.close(figure)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
