#!/usr/bin/env python3
"""一次走行ログから現行方式とカーブ限定方式のLevel 1候補を比較描画する。"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from path_log_recovery import read_csv_log


SPACING_MM = 40.0
ANCHOR_HALF_WIDTH_MM = 100.0
MAX_OFFSET_MM = 12.375
SMOOTH_MOVE_MAX_MM = 2.0
SMOOTH_ITERATIONS = 100
CURVE_RADIUS_THRESHOLD_MM = 2000.0
CURVE_EXPAND_POINTS = 3
CURVE_TAPER_POINTS = 4


def c_round(value: float) -> int:
    return math.floor(value + 0.5) if value >= 0.0 else math.ceil(value - 0.5)


def distance(a: tuple[int, int] | list[int], b: tuple[int, int] | list[int]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def wrap_rad(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def read_raw(path: Path) -> list[tuple[float, float, int]]:
    points: list[tuple[float, float, int]] = []
    for row in read_csv_log(path).rows:
        try:
            points.append((float(row["x"]), float(row["y"]), int(float(row["courseMarker"]))))
        except (KeyError, TypeError, ValueError):
            continue
    if len(points) < 2:
        raise ValueError("有効なx, y, courseMarkerが不足しています")
    return points


def resample(raw: list[tuple[float, float, int]]) -> tuple[list[list[int]], list[bool]]:
    first_x, first_y, _ = raw[0]
    last_x, last_y, _ = raw[-1]
    route = [[0, 0]]
    anchors = [True]
    accumulated = 0.0
    previous: tuple[float, float] | None = None
    marker_pending = False
    for raw_x, raw_y, marker in raw:
        x = raw_x - first_x
        y = raw_y - first_y
        if marker != 0:
            marker_pending = True
        if previous is None:
            previous = (x, y)
            continue
        segment_x, segment_y = previous
        remaining = math.hypot(x - segment_x, y - segment_y)
        while accumulated + remaining >= SPACING_MM:
            needed = SPACING_MM - accumulated
            ratio = needed / remaining
            segment_x += (x - segment_x) * ratio
            segment_y += (y - segment_y) * ratio
            route.append([c_round(segment_x), c_round(segment_y)])
            anchors.append(marker_pending)
            marker_pending = False
            remaining -= needed
            accumulated = 0.0
        accumulated += remaining
        previous = (x, y)
    route[-1] = [c_round(last_x - first_x), c_round(last_y - first_y)]
    anchors[-1] = True
    return route, anchors


def expand_anchors(anchors: list[bool]) -> list[bool]:
    expanded = anchors[:]
    radius = math.ceil(ANCHOR_HALF_WIDTH_MM / SPACING_MM)
    for index, anchor in enumerate(anchors):
        if not anchor:
            continue
        for target in range(max(0, index - radius), min(len(anchors), index + radius + 1)):
            expanded[target] = True
    return expanded


def smooth_route(route: list[list[int]], movable_weight: list[float]) -> list[list[int]]:
    drive = [point[:] for point in route]
    for _ in range(SMOOTH_ITERATIONS):
        for index in range(1, len(drive) - 1):
            weight = movable_weight[index]
            if weight <= 0.0:
                continue
            x, y = drive[index]
            target_x = 0.5 * (drive[index - 1][0] + drive[index + 1][0])
            target_y = 0.5 * (drive[index - 1][1] + drive[index + 1][1])
            dx = target_x - x
            dy = target_y - y
            move = math.hypot(dx, dy)
            move_limit = SMOOTH_MOVE_MAX_MM * weight
            if move > move_limit:
                dx *= move_limit / move
                dy *= move_limit / move
            x += dx
            y += dy
            from_line_x = x - route[index][0]
            from_line_y = y - route[index][1]
            offset = math.hypot(from_line_x, from_line_y)
            offset_limit = MAX_OFFSET_MM * weight
            if offset > offset_limit:
                x = route[index][0] + from_line_x * offset_limit / offset
                y = route[index][1] + from_line_y * offset_limit / offset
            drive[index] = [c_round(x), c_round(y)]
    return drive


def headings(route: list[list[int]], window: int = 2) -> list[float]:
    result: list[float] = []
    for index in range(len(route)):
        before = max(0, index - window)
        after = min(len(route) - 1, index + window)
        dx = route[after][0] - route[before][0]
        dy = route[after][1] - route[before][1]
        result.append(math.atan2(dx, dy))
    return result


def curve_mask(route: list[list[int]]) -> list[bool]:
    direction = headings(route, 2)
    curvature = [0.0] * len(route)
    for index in range(2, len(route) - 2):
        ds = distance(route[index - 2], route[index + 2])
        if ds > 1.0:
            curvature[index] = abs(wrap_rad(direction[index + 2] - direction[index - 2])) / ds
    threshold = 1.0 / CURVE_RADIUS_THRESHOLD_MM
    raw = [value >= threshold for value in curvature]
    expanded = raw[:]
    for index, active in enumerate(raw):
        if active:
            for target in range(max(0, index - CURVE_EXPAND_POINTS),
                                min(len(route), index + CURVE_EXPAND_POINTS + 1)):
                expanded[target] = True
    # 3点未満の孤立区間は座標量子化ノイズとして除外する。
    index = 0
    while index < len(expanded):
        if not expanded[index]:
            index += 1
            continue
        end = index
        while end < len(expanded) and expanded[end]:
            end += 1
        if end - index < 3:
            for target in range(index, end):
                expanded[target] = False
        index = end
    return expanded


def tapered_curve_weights(mask: list[bool]) -> list[float]:
    weights = [0.0] * len(mask)
    index = 0
    while index < len(mask):
        if not mask[index]:
            index += 1
            continue
        end = index
        while end < len(mask) and mask[end]:
            end += 1
        for target in range(index, end):
            edge_distance = min(target - index + 1, end - target)
            weights[target] = min(1.0, edge_distance / CURVE_TAPER_POINTS)
        index = end
    weights[0] = weights[-1] = 0.0
    return weights


def smoothstep(value: float) -> float:
    value = min(1.0, max(0.0, value))
    return value * value * (3.0 - 2.0 * value)


def racing_line_profile(progress: float) -> float:
    """入口外側→頂点内側→出口外側を滑らかにつなぐ符号付き比率。"""
    knots = ((0.0, 0.0), (0.20, -1.0), (0.50, 1.0), (0.80, -1.0), (1.0, 0.0))
    for (x0, y0), (x1, y1) in zip(knots, knots[1:]):
        if progress <= x1:
            blend = smoothstep((progress - x0) / (x1 - x0))
            return y0 + (y1 - y0) * blend
    return 0.0


def curve_segments(mask: list[bool]) -> list[tuple[int, int]]:
    segments: list[tuple[int, int]] = []
    index = 0
    while index < len(mask):
        if not mask[index]:
            index += 1
            continue
        end = index
        while end + 1 < len(mask) and mask[end + 1]:
            end += 1
        segments.append((index, end))
        index = end + 1
    return segments


def build_racing_line(route: list[list[int]], mask: list[bool]) -> list[list[int]]:
    """直線を固定し、各カーブ内だけをアウト・イン・アウトへ変位させる。"""
    result = [point[:] for point in route]
    direction = headings(route, 2)
    for begin, end in curve_segments(mask):
        if end - begin < 5:
            continue
        signed_turn = 0.0
        for index in range(max(1, begin), min(len(route) - 1, end + 1)):
            ax = route[index][0] - route[index - 1][0]
            ay = route[index][1] - route[index - 1][1]
            bx = route[index + 1][0] - route[index][0]
            by = route[index + 1][1] - route[index][1]
            signed_turn += ax * by - ay * bx
        turn_sign = 1.0 if signed_turn >= 0.0 else -1.0
        span = end - begin
        for index in range(begin, end + 1):
            progress = (index - begin) / span
            offset = MAX_OFFSET_MM * racing_line_profile(progress)
            # 左法線(-ty, tx)へturn_signを掛けると旋回内側になる。
            tangent_x = math.sin(direction[index])
            tangent_y = math.cos(direction[index])
            normal_x = -tangent_y * turn_sign
            normal_y = tangent_x * turn_sign
            result[index][0] = c_round(route[index][0] + normal_x * offset)
            result[index][1] = c_round(route[index][1] + normal_y * offset)
    return result


def route_length(route: list[list[int]]) -> float:
    return sum(distance(route[index - 1], route[index]) for index in range(1, len(route)))


def offsets(source: list[list[int]], target: list[list[int]]) -> list[float]:
    return [distance(a, b) for a, b in zip(source, target)]


def robust_radius(route: list[list[int]], mask: list[bool]) -> float:
    direction = headings(route, 2)
    curvature: list[float] = []
    for index in range(2, len(route) - 2):
        if not mask[index]:
            continue
        ds = distance(route[index - 2], route[index + 2])
        if ds > 1.0:
            value = abs(wrap_rad(direction[index + 2] - direction[index - 2])) / ds
            if value > 1.0e-6:
                curvature.append(value)
    if not curvature:
        return math.inf
    curvature.sort()
    p95 = curvature[min(len(curvature) - 1, math.ceil(0.95 * len(curvature)) - 1)]
    return 1.0 / p95


def load_fonts() -> tuple[ImageFont.FreeTypeFont, ImageFont.FreeTypeFont, ImageFont.FreeTypeFont]:
    path = r"C:\Windows\Fonts\YuGothM.ttc"
    return (ImageFont.truetype(path, 34), ImageFont.truetype(path, 24), ImageFont.truetype(path, 18))


def draw_panel(draw: ImageDraw.ImageDraw, box: tuple[int, int, int, int], title: str,
               source: list[list[int]], candidate: list[list[int]], color: str,
               subtitle: str, zoom_index: int | None = None,
               mask: list[bool] | None = None, anchors: list[bool] | None = None) -> None:
    title_font, label_font, small_font = load_fonts()
    left, top, right, bottom = box
    draw.rectangle(box, outline="#9ca3af", width=2)
    draw.text((left + 16, top + 10), title, fill="#111827", font=label_font)
    draw.text((left + 16, top + 44), subtitle, fill="#374151", font=small_font)
    plot = (left + 72, top + 82, right - 24, bottom - 54)
    if zoom_index is None:
        indices = range(len(source))
        points_for_domain = source + candidate
    else:
        indices = range(max(0, zoom_index - 15), min(len(source), zoom_index + 16))
        points_for_domain = [source[i] for i in indices] + [candidate[i] for i in indices]
    min_x = min(point[0] for point in points_for_domain)
    max_x = max(point[0] for point in points_for_domain)
    min_y = min(point[1] for point in points_for_domain)
    max_y = max(point[1] for point in points_for_domain)
    span_x = max(1.0, max_x - min_x)
    span_y = max(1.0, max_y - min_y)
    scale = min((plot[2] - plot[0]) / span_x, (plot[3] - plot[1]) / span_y) * 0.92
    center_x = 0.5 * (min_x + max_x)
    center_y = 0.5 * (min_y + max_y)
    pixel_center_x = 0.5 * (plot[0] + plot[2])
    pixel_center_y = 0.5 * (plot[1] + plot[3])

    def project(point: list[int]) -> tuple[float, float]:
        return (pixel_center_x + (point[0] - center_x) * scale,
                pixel_center_y - (point[1] - center_y) * scale)

    draw.rectangle(plot, outline="#d1d5db", width=1)
    source_pixels = [project(source[i]) for i in indices]
    candidate_pixels = [project(candidate[i]) for i in indices]
    if len(source_pixels) > 1:
        draw.line(source_pixels, fill="#6b7280", width=5)
        draw.line(candidate_pixels, fill=color, width=3)
    if mask is not None and zoom_index is None:
        segment: list[tuple[float, float]] = []
        for index in indices:
            if mask[index]:
                segment.append(project(candidate[index]))
            else:
                if len(segment) > 1:
                    draw.line(segment, fill=color, width=4)
                segment = []
        if len(segment) > 1:
            draw.line(segment, fill=color, width=4)
    if anchors is not None and zoom_index is not None:
        for index in indices:
            if anchors[index]:
                px, py = project(source[index])
                draw.ellipse((px - 3, py - 3, px + 3, py + 3), fill="#f59e0b")
    draw.line((left + 16, bottom - 22, left + 60, bottom - 22), fill="#6b7280", width=5)
    draw.text((left + 68, bottom - 34), "一次経路", fill="#374151", font=small_font)
    draw.line((left + 190, bottom - 22, left + 234, bottom - 22), fill=color, width=3)
    draw.text((left + 242, bottom - 34), "生成候補", fill="#374151", font=small_font)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    raw = read_raw(args.log)
    source, marker_anchors = resample(raw)
    current_anchors = expand_anchors(marker_anchors)
    current_weights = [0.0 if anchor else 1.0 for anchor in current_anchors]
    current_candidate = smooth_route(source, current_weights)

    curves = curve_mask(source)
    new_candidate = build_racing_line(source, curves)

    source_length = route_length(source)
    current_length = route_length(current_candidate)
    new_length = route_length(new_candidate)
    current_offsets = offsets(source, current_candidate)
    new_offsets = offsets(source, new_candidate)
    current_zoom = max(range(len(source)), key=current_offsets.__getitem__)
    new_zoom = max(range(len(source)), key=new_offsets.__getitem__)
    straight_max = max((new_offsets[i] for i, curve in enumerate(curves) if not curve), default=0.0)
    radius_before = robust_radius(source, curves)
    radius_current = robust_radius(current_candidate, curves)
    radius_after = robust_radius(new_candidate, curves)

    image = Image.new("RGB", (1800, 1320), "#ffffff")
    draw = ImageDraw.Draw(image)
    title_font, label_font, small_font = load_fonts()
    draw.text((48, 25), "一次走行12345：Level 1経路生成方式の比較", fill="#111827", font=title_font)
    draw.text((48, 72), "灰色＝一次走行の40 mm再標本化経路。色線＝各方式が生成したLevel 1候補。",
              fill="#374151", font=small_font)

    current_reduction = (1.0 - current_length / source_length) * 100.0
    new_reduction = (1.0 - new_length / source_length) * 100.0
    draw_panel(draw, (40, 115, 880, 680), "現行方式：全非固定点をElastic Bandで平滑化",
               source, current_candidate, "#dc2626",
               f"短縮率 {current_reduction:.3f}%（0.5%条件未達のため実機では棄却）")
    draw_panel(draw, (920, 115, 1760, 680), "新方式案：直線固定・カーブだけアウト・イン・アウト変形",
               source, new_candidate, "#2563eb",
               f"短縮率 {new_reduction:.3f}%／直線最大変位 {straight_max:.1f} mm", mask=curves)
    draw_panel(draw, (40, 720, 880, 1275), "現行方式：最大変位箇所の拡大",
               source, current_candidate, "#dc2626",
               f"最大変位 {max(current_offsets):.1f} mm／R95 {radius_before:.0f}→{radius_current:.0f} mm／固定点 {sum(current_anchors)}/{len(source)}",
               zoom_index=current_zoom, anchors=current_anchors)
    draw_panel(draw, (920, 720, 1760, 1275), "新方式案：最大変位箇所の拡大",
               source, new_candidate, "#2563eb",
               f"最大変位 {max(new_offsets):.1f} mm／R95 {radius_before:.0f}→{radius_after:.0f} mm",
               zoom_index=new_zoom)
    draw.text((48, 1288), "新方式は解析用プロトタイプであり、ファームウェアには未実装です。",
              fill="#374151", font=small_font)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    image.save(args.output)
    print(f"points={len(source)} source_length_mm={source_length:.3f}")
    print(f"current_length_mm={current_length:.3f} reduction_pct={current_reduction:.4f} "
          f"max_offset_mm={max(current_offsets):.3f} anchors={sum(current_anchors)}")
    print(f"new_length_mm={new_length:.3f} reduction_pct={new_reduction:.4f} "
          f"max_offset_mm={max(new_offsets):.3f} curve_points={sum(curves)} "
          f"straight_max_mm={straight_max:.3f} radius95_before_mm={radius_before:.1f} "
          f"radius95_current_mm={radius_current:.1f} radius95_after_mm={radius_after:.1f}")
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
