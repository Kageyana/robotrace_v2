#!/usr/bin/env python3
"""一次走行経路からスラローム区間を抽出し、直線回廊候補を比較表示する。"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib.pyplot as plt

from compare_shortcut_routes_12345 import (
    c_round,
    distance,
    expand_anchors,
    headings,
    offsets,
    read_raw,
    resample,
    route_length,
    smooth_route,
    wrap_rad,
)


MAX_OFFSET_MM = 34.0
MIN_SPAN_POINTS = 15       # 600 mm
MAX_SPAN_POINTS = 40       # 1600 mm
TRANSITION_POINTS = 3      # 120 mm
HEADING_LIMIT_DEG = 12.0
MIN_SIGN_CHANGES = 3
MIN_SAVING_MM = 5.0
MAX_CORRIDOR_COUNT = 16


def smoothstep(value: float) -> float:
    value = min(1.0, max(0.0, value))
    return value * value * (3.0 - 2.0 * value)


def point_line_distance(point: list[int], begin: list[int], end: list[int]) -> tuple[float, float]:
    dx = end[0] - begin[0]
    dy = end[1] - begin[1]
    length_squared = dx * dx + dy * dy
    if length_squared <= 0.0:
        return math.inf, 0.0
    progress = ((point[0] - begin[0]) * dx + (point[1] - begin[1]) * dy) / length_squared
    projected_x = begin[0] + progress * dx
    projected_y = begin[1] + progress * dy
    return math.hypot(point[0] - projected_x, point[1] - projected_y), progress


def signed_turns(route: list[list[int]], direction: list[float], begin: int, end: int) -> list[int]:
    signs: list[int] = []
    threshold = math.radians(2.0)
    for index in range(max(begin + 2, 2), min(end - 1, len(route) - 2)):
        delta = wrap_rad(direction[index + 2] - direction[index - 2])
        if abs(delta) < threshold:
            continue
        sign = 1 if delta > 0.0 else -1
        if not signs or signs[-1] != sign:
            signs.append(sign)
    return signs


def find_candidates(route: list[list[int]]) -> list[tuple[int, int, float, float, int]]:
    # ファームウェアと同じく接線方位を0.01 degへ丸めてから候補判定する。
    direction = [math.radians(c_round(math.degrees(value) * 100.0) * 0.01)
                 for value in headings(route, 2)]
    candidates: list[tuple[int, int, float, float, int]] = []
    heading_limit = math.radians(HEADING_LIMIT_DEG)
    for begin in range(4, len(route) - MIN_SPAN_POINTS - 4):
        for span in range(MIN_SPAN_POINTS, MAX_SPAN_POINTS + 1):
            end = begin + span
            if end >= len(route) - 4:
                break
            chord_heading = math.atan2(route[end][0] - route[begin][0], route[end][1] - route[begin][1])
            if abs(wrap_rad(direction[begin] - chord_heading)) > heading_limit:
                continue
            if abs(wrap_rad(direction[end] - chord_heading)) > heading_limit:
                continue
            maximum_offset = 0.0
            projected = True
            for point in route[begin:end + 1]:
                offset, progress = point_line_distance(point, route[begin], route[end])
                maximum_offset = max(maximum_offset, offset)
                if progress < -0.02 or progress > 1.02 or maximum_offset > MAX_OFFSET_MM:
                    projected = False
                    break
            if not projected:
                continue
            signs = signed_turns(route, direction, begin, end)
            sign_changes = max(0, len(signs) - 1)
            if sign_changes < MIN_SIGN_CHANGES:
                continue
            source_length = route_length(route[begin:end + 1])
            chord_length = distance(route[begin], route[end])
            saving = source_length - chord_length
            if saving < MIN_SAVING_MM:
                continue
            candidates.append((begin, end, saving, maximum_offset, sign_changes))
    return candidates


def select_non_overlapping(candidates: list[tuple[int, int, float, float, int]]) -> list[tuple[int, int, float, float, int]]:
    selected: list[tuple[int, int, float, float, int]] = []
    for candidate in sorted(candidates, key=lambda item: (item[2], item[1] - item[0]), reverse=True):
        begin, end = candidate[0], candidate[1]
        if any(not (end + 5 < chosen[0] or begin > chosen[1] + 5) for chosen in selected):
            continue
        selected.append(candidate)
        if len(selected) >= MAX_CORRIDOR_COUNT:
            break
    return sorted(selected)


def apply_corridor(route: list[list[int]], result: list[list[int]], begin: int, end: int) -> None:
    span = end - begin
    for index in range(begin, end + 1):
        progress = (index - begin) / span
        chord_x = route[begin][0] + (route[end][0] - route[begin][0]) * progress
        chord_y = route[begin][1] + (route[end][1] - route[begin][1]) * progress
        weight = min(smoothstep((index - begin) / TRANSITION_POINTS),
                     smoothstep((end - index) / TRANSITION_POINTS))
        move_x = (chord_x - route[index][0]) * weight
        move_y = (chord_y - route[index][1]) * weight
        move = math.hypot(move_x, move_y)
        if move > MAX_OFFSET_MM:
            move_x *= MAX_OFFSET_MM / move
            move_y *= MAX_OFFSET_MM / move
        result[index][0] = c_round(route[index][0] + move_x)
        result[index][1] = c_round(route[index][1] + move_y)


def segment_intersection_pairs(route: list[list[int]]) -> set[tuple[int, int]]:
    """隣接しない経路線分同士の交差位置を線分indexの組で返す。"""
    def orientation(a: list[int], b: list[int], c: list[int]) -> int:
        cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        return (cross > 0) - (cross < 0)

    intersections: set[tuple[int, int]] = set()
    for first in range(len(route) - 1):
        for second in range(first + 2, len(route) - 1):
            if first == 0 and second == len(route) - 2:
                continue
            a, b = route[first], route[first + 1]
            c, d = route[second], route[second + 1]
            if orientation(a, b, c) * orientation(a, b, d) < 0 \
                    and orientation(c, d, a) * orientation(c, d, b) < 0:
                intersections.add((first, second))
    return intersections


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args()

    route, marker_anchors = resample(read_raw(args.log))
    anchors = expand_anchors(marker_anchors)
    current = smooth_route(route, [0.0 if anchor else 1.0 for anchor in anchors])
    selected = select_non_overlapping(find_candidates(route))
    proposed = [point[:] for point in route]
    for begin, end, _, _, _ in selected:
        apply_corridor(route, proposed, begin, end)
    source_intersections = segment_intersection_pairs(route)
    proposed_intersections = segment_intersection_pairs(proposed)
    new_intersections = proposed_intersections - source_intersections
    reduction = route_length(route) - route_length(proposed)
    build_status = 1 if selected and reduction >= MIN_SAVING_MM and not new_intersections else (
        4 if not selected else 6 if new_intersections else 7
    )

    plt.rcParams.update({"font.family": "Yu Gothic", "axes.unicode_minus": False, "font.size": 11})
    figure, axis = plt.subplots(figsize=(10.5, 9.0), constrained_layout=True)
    if figure.canvas.manager is not None:
        figure.canvas.manager.set_window_title(f"{args.log.stem} 直線回廊比較")
    axis.plot([point[0] for point in route], [point[1] for point in route],
              color="#4b5563", linewidth=4.0, linestyle=(0, (6, 3)),
              label="一次走行経路（Level 0）", zorder=1)
    axis.plot([point[0] for point in current], [point[1] for point in current],
              color="#dc2626", linewidth=2.0, label="旧Elastic Band候補", zorder=2)
    axis.plot([point[0] for point in proposed], [point[1] for point in proposed],
              color="#2563eb", linewidth=2.2, label="直線回廊候補", zorder=3)
    for number, (begin, end, _, _, _) in enumerate(selected, start=1):
        axis.plot([point[0] for point in proposed[begin:end + 1]],
                  [point[1] for point in proposed[begin:end + 1]],
                  color="#2563eb", linewidth=4.0, zorder=4)
        axis.scatter([route[begin][0], route[end][0]], [route[begin][1], route[end][1]],
                     color="#111827", s=28, zorder=5)
        middle = (begin + end) // 2
        axis.annotate(str(number), (proposed[middle][0], proposed[middle][1]),
                      xytext=(5, 5), textcoords="offset points", fontsize=10)
    axis.set_title(f"一次走行{args.log.stem}：旧方式と直線回廊方式の全コース比較", fontsize=15)
    axis.set_xlabel("X [mm]")
    axis.set_ylabel("Y [mm]")
    axis.set_aspect("equal", adjustable="box")
    axis.margins(0.03)
    axis.grid(True, linewidth=0.5, alpha=0.35)
    axis.legend(loc="upper center", ncol=3, framealpha=0.94)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=180, bbox_inches="tight")
    print(f"route_points={len(route)} candidates={len(selected)}")
    for number, (begin, end, saving, maximum_offset, sign_changes) in enumerate(selected, start=1):
        actual_offset = max(offsets(route[begin:end + 1], proposed[begin:end + 1]))
        print(f"corridor{number}: index={begin}..{end} saving_mm={saving:.3f} "
              f"source_offset_mm={maximum_offset:.3f} applied_offset_mm={actual_offset:.3f} "
              f"sign_changes={sign_changes}")
    print(f"source_length_mm={route_length(route):.3f} current_length_mm={route_length(current):.3f} "
          f"proposed_length_mm={route_length(proposed):.3f}")
    print(f"source_intersections={len(source_intersections)} "
          f"proposed_intersections={len(proposed_intersections)} "
          f"new_intersections={len(new_intersections)}")
    print(f"shortcutBuildStatus={build_status} shortcutCorridorCount={len(selected)} "
          f"shortcutReduction_mm={reduction:.3f}")
    print(args.output)
    if not args.no_show:
        plt.show()
    plt.close(figure)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
