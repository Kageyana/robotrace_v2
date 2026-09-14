#!/usr/bin/env python3
"""新ログからVersion 12/13のPATH参照列をメモリ上で復元する共通処理。"""

from __future__ import annotations

import csv
import math
import struct
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Any


PATH_MODES = {3, 4}
PATH_ROUTE_CONTROLLER_VERSIONS = {12, 13}
PATH_ROUTE_SPACING_MM = 40.0
PATH_ROUTE_MAX_POINTS = 1514
PATH_GOAL_EXTENSION_MM = 500.0
PATH_GOAL_EXTENSION_POINTS = 13
PATH_CORRIDOR_MIN_SPAN_POINTS = 15
PATH_CORRIDOR_MAX_SPAN_POINTS = 40
PATH_CORRIDOR_TRANSITION_POINTS = 3
PATH_CORRIDOR_END_GUARD_POINTS = 4
PATH_CORRIDOR_OVERLAP_GUARD_POINTS = 5
PATH_CORRIDOR_MAX_COUNT = 16
PATH_CORRIDOR_HEADING_LIMIT_DEG = 12.0
PATH_CORRIDOR_TURN_THRESHOLD_DEG = 2.0
PATH_CORRIDOR_MIN_SIGN_CHANGES = 3
PATH_CORRIDOR_MAX_OFFSET_MM = 34.0
PATH_CORRIDOR_OFFSET_TOLERANCE_MM = 0.5
PATH_CORRIDOR_MIN_PROJECTION = -0.02
PATH_CORRIDOR_MAX_PROJECTION = 1.02
PATH_CORRIDOR_MIN_SAVING_MM = 5.0
PATH_TRACKING_ERROR_BUDGET_MM = 15.0
PATH_LINE_HALF_WIDTH_MM = 9.5
PATH_OCCUPIED_HALF_WIDTH_MM = 65.0
PATH_LEGAL_RESERVE_MM = 10.0

BUILD_STATUS_NAMES = {
    0: "not_requested",
    1: "success",
    2: "disabled_by_setting",
    3: "legal_geometry_invalid",
    4: "no_corridor",
    5: "offset_violation",
    6: "new_intersection",
    7: "insufficient_reduction",
}


def f32(value: float) -> float:
    """C float演算の中間値を単精度へ丸める。"""
    return struct.unpack("<f", struct.pack("<f", float(value)))[0]


def c_round(value: float) -> int:
    """lroundf相当（0.5は絶対値を増やす方向）で整数化する。"""
    value = f32(value)
    return math.floor(value + 0.5) if value >= 0.0 else math.ceil(value - 0.5)


def int16_round(value: float) -> int:
    if value > 32767.0:
        return 32767
    if value < -32768.0:
        return -32768
    return c_round(value)


def wrap_deg(value: float) -> float:
    value = f32(value)
    while value > 180.0:
        value = f32(value - 360.0)
    while value < -180.0:
        value = f32(value + 360.0)
    return value


def point_distance(x0: float, y0: float, x1: float, y1: float) -> float:
    dx = f32(x1 - x0)
    dy = f32(y1 - y0)
    return f32(math.sqrt(f32(f32(dx * dx) + f32(dy * dy))))


@dataclass
class RoutePoint:
    x: int
    y: int
    heading_cdeg: int = 0


@dataclass
class RouteBuild:
    line: list[RoutePoint]
    drive: list[RoutePoint]
    flags: list[int]
    requested_level: int
    applied_level: int
    build_status: int
    corridor_count: int
    reduction_mm: float
    geometry_crc32: int

    @property
    def route_count(self) -> int:
        return len(self.line)


@dataclass
class CsvLog:
    path: Path
    fields: list[str]
    rows: list[dict[str, str]]
    parameters: dict[str, str]


@dataclass
class Recovery:
    status: str
    reason: str
    source_log: int
    source_path: Path | None
    route: RouteBuild | None
    row_values: list[dict[str, Any]]
    missing_samples: int = 0


def parameter_number(parameters: dict[str, str], name: str, default: float = math.nan) -> float:
    try:
        return float(parameters[name])
    except (KeyError, TypeError, ValueError):
        return default


def parameter_int(parameters: dict[str, str], name: str, default: int | None = None) -> int | None:
    value = parameter_number(parameters, name)
    if not math.isfinite(value):
        return default
    return int(round(value))


def parse_optimal_index(value: Any, route_count: int | None = None) -> int | None:
    """有限な整数値だけをoptimalIndexとして受け付ける。"""
    if value is None or str(value).strip() == "":
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number) or not number.is_integer():
        return None
    index = int(number)
    if index < 0 or (route_count is not None and index >= route_count):
        return None
    return index


def optimal_index_error(value: Any, route_count: int) -> str:
    """optimalIndexの欠測理由を返す。"""
    if value is None or str(value).strip() == "":
        return "optimalIndex空欄"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "optimalIndex数値不正"
    if not math.isfinite(number):
        return "optimalIndex非有限"
    if not number.is_integer():
        return f"optimalIndex整数でない: {value}"
    index = int(number)
    if index < 0 or index >= route_count:
        return f"optimalIndex範囲外: {index}"
    return "optimalIndex不正"


def read_csv_log(path: Path) -> CsvLog:
    with path.open("r", encoding="utf-8-sig", newline="") as source:
        reader = csv.reader(source)
        try:
            first_header = next(reader)
        except StopIteration as exc:
            raise ValueError(f"{path}: empty CSV") from exc

        fields: list[str] = []
        parameters: dict[str, str] = {}

        def parse_header(cells: list[str]) -> tuple[list[str], dict[str, str]]:
            parsed_fields: list[str] = []
            parsed_parameters: dict[str, str] = {}
            for cell in cells:
                cell = cell.strip()
                if not cell:
                    continue
                if "=" in cell:
                    name, value = cell.split("=", 1)
                    if name.strip():
                        parsed_parameters[name.strip()] = value.strip()
                else:
                    parsed_fields.append(cell)
            return parsed_fields, parsed_parameters

        fields, parameters = parse_header(first_header)
        data_start_line = 2
        if not fields and parameters:
            try:
                second_header = next(reader)
            except StopIteration as exc:
                raise ValueError(f"{path}: missing column header") from exc
            fields, unexpected_parameters = parse_header(second_header)
            if unexpected_parameters or not fields:
                raise ValueError(f"{path}: invalid column header")
            data_start_line = 3
        elif not fields:
            raise ValueError(f"{path}: invalid CSV header")

        rows: list[dict[str, str]] = []
        for line_number, raw in enumerate(reader, start=data_start_line):
            if len(raw) < len(fields):
                continue
            rows.append({name: raw[index].strip() for index, name in enumerate(fields)})
        if not rows:
            raise ValueError(f"{path}: no data rows")
    return CsvLog(path, fields, rows, parameters)


def _read_route_points(source: CsvLog) -> list[tuple[float, float, int]]:
    required = {"x", "y", "courseMarker"}
    missing = sorted(required - set(source.fields))
    if missing:
        raise ValueError(f"source {source.path}: missing columns: {', '.join(missing)}")
    points: list[tuple[float, float, int]] = []
    for row in source.rows:
        try:
            x = f32(float(row["x"]))
            y = f32(float(row["y"]))
            marker = int(float(row["courseMarker"]))
        except (KeyError, ValueError) as exc:
            raise ValueError(f"source {source.path}: invalid route row") from exc
        if math.isfinite(x) and math.isfinite(y):
            points.append((x, y, marker))
    return points


def compute_headings(route: list[RoutePoint]) -> None:
    if len(route) < 2:
        return
    for index in range(len(route)):
        before = max(0, index - 1)
        after = min(len(route) - 1, index + 1)
        dx = f32(route[after].x - route[before].x)
        dy = f32(route[after].y - route[before].y)
        angle = f32(math.atan2(dx, dy))
        degrees = f32(angle * (180.0 / math.pi))
        route[index].heading_cdeg = int16_round(f32(wrap_deg(degrees) * 100.0))


def smoothstep(value: float) -> float:
    value = f32(min(1.0, max(0.0, value)))
    return f32(f32(value * value) * f32(3.0 - f32(2.0 * value)))


def segment_intersects(route: list[RoutePoint], i: int, j: int) -> bool:
    ax, ay = float(route[i].x), float(route[i].y)
    bx, by = float(route[i + 1].x), float(route[i + 1].y)
    cx, cy = float(route[j].x), float(route[j].y)
    dx, dy = float(route[j + 1].x), float(route[j + 1].y)
    ab_c = f32(f32(bx - ax) * f32(cy - ay) - f32(by - ay) * f32(cx - ax))
    ab_d = f32(f32(bx - ax) * f32(dy - ay) - f32(by - ay) * f32(dx - ax))
    cd_a = f32(f32(dx - cx) * f32(ay - cy) - f32(dy - cy) * f32(ax - cx))
    cd_b = f32(f32(dx - cx) * f32(by - cy) - f32(dy - cy) * f32(bx - cx))
    return ((ab_c > 0.0 and ab_d < 0.0) or (ab_c < 0.0 and ab_d > 0.0)) and (
        (cd_a > 0.0 and cd_b < 0.0) or (cd_a < 0.0 and cd_b > 0.0)
    )


def path_length(route: list[RoutePoint]) -> float:
    length = 0.0
    for before, after in zip(route, route[1:]):
        length = f32(length + point_distance(before.x, before.y, after.x, after.y))
    return length


def evaluate_candidate(line: list[RoutePoint], drive: list[RoutePoint], begin: int, end: int) -> float | None:
    dx = f32(line[end].x - line[begin].x)
    dy = f32(line[end].y - line[begin].y)
    chord_squared = f32(f32(dx * dx) + f32(dy * dy))
    if chord_squared <= 0.0:
        return None
    chord_heading = f32(math.atan2(dx, dy) * (180.0 / math.pi))
    begin_heading = f32(drive[begin].heading_cdeg * 0.01)
    end_heading = f32(drive[end].heading_cdeg * 0.01)
    if abs(wrap_deg(f32(begin_heading - chord_heading))) > PATH_CORRIDOR_HEADING_LIMIT_DEG:
        return None
    if abs(wrap_deg(f32(end_heading - chord_heading))) > PATH_CORRIDOR_HEADING_LIMIT_DEG:
        return None

    source_length = 0.0
    for index in range(begin, end + 1):
        point_x = f32(line[index].x - line[begin].x)
        point_y = f32(line[index].y - line[begin].y)
        progress = f32(f32(point_x * dx) + f32(point_y * dy))
        progress = f32(progress / chord_squared)
        if progress < PATH_CORRIDOR_MIN_PROJECTION or progress > PATH_CORRIDOR_MAX_PROJECTION:
            return None
        projected_x = f32(line[begin].x + f32(progress * dx))
        projected_y = f32(line[begin].y + f32(progress * dy))
        if point_distance(line[index].x, line[index].y, projected_x, projected_y) > PATH_CORRIDOR_MAX_OFFSET_MM:
            return None
        if index > begin:
            source_length = f32(source_length + point_distance(
                line[index - 1].x, line[index - 1].y, line[index].x, line[index].y
            ))

    sign_changes = 0
    previous_sign = 0
    for index in range(begin + 2, end - 1):
        before_heading = f32(drive[index - 2].heading_cdeg * 0.01)
        after_heading = f32(drive[index + 2].heading_cdeg * 0.01)
        delta = wrap_deg(f32(after_heading - before_heading))
        if abs(delta) < PATH_CORRIDOR_TURN_THRESHOLD_DEG:
            continue
        sign = 1 if delta > 0.0 else -1
        if previous_sign != 0 and sign != previous_sign:
            sign_changes += 1
        previous_sign = sign
    if sign_changes < PATH_CORRIDOR_MIN_SIGN_CHANGES:
        return None
    return f32(source_length - f32(math.sqrt(chord_squared)))


def apply_candidate(line: list[RoutePoint], drive: list[RoutePoint], flags: list[int], begin: int, end: int) -> None:
    span = end - begin
    for index in range(begin, end + 1):
        progress = f32((index - begin) / span)
        chord_x = f32(line[begin].x + f32((line[end].x - line[begin].x) * progress))
        chord_y = f32(line[begin].y + f32((line[end].y - line[begin].y) * progress))
        entry_weight = smoothstep(f32((index - begin) / PATH_CORRIDOR_TRANSITION_POINTS))
        exit_weight = smoothstep(f32((end - index) / PATH_CORRIDOR_TRANSITION_POINTS))
        weight = min(entry_weight, exit_weight)
        move_x = f32(f32(chord_x - line[index].x) * weight)
        move_y = f32(f32(chord_y - line[index].y) * weight)
        move = point_distance(0.0, 0.0, move_x, move_y)
        if move > PATH_CORRIDOR_MAX_OFFSET_MM:
            scale = f32(PATH_CORRIDOR_MAX_OFFSET_MM / move)
            move_x = f32(move_x * scale)
            move_y = f32(move_y * scale)
        drive[index].x = int16_round(f32(line[index].x + move_x))
        drive[index].y = int16_round(f32(line[index].y + move_y))
        flags[index] |= 0x02

    reserve_begin = max(0, begin - PATH_CORRIDOR_OVERLAP_GUARD_POINTS)
    reserve_end = min(len(line) - 1, end + PATH_CORRIDOR_OVERLAP_GUARD_POINTS)
    for index in range(reserve_begin, reserve_end + 1):
        flags[index] |= 0x01


def generate_shortcut(line: list[RoutePoint], requested_level: int, max_level: int) -> tuple[list[RoutePoint], list[int], int, int, float, int]:
    drive = [RoutePoint(point.x, point.y, point.heading_cdeg) for point in line]
    flags = [0] * len(line)
    if requested_level == 0:
        return drive, flags, 0, 0, 0.0, 0
    if len(line) < PATH_CORRIDOR_MIN_SPAN_POINTS + (2 * PATH_CORRIDOR_END_GUARD_POINTS) + 1:
        return drive, flags, 0, 4, 0.0, 0
    if max_level == 0:
        return drive, flags, 0, 2, 0.0, 0
    level = min(requested_level, 1)

    legal_offset = f32(PATH_LINE_HALF_WIDTH_MM + PATH_OCCUPIED_HALF_WIDTH_MM - PATH_TRACKING_ERROR_BUDGET_MM - PATH_LEGAL_RESERVE_MM)
    if legal_offset <= 0.0 or PATH_CORRIDOR_MAX_OFFSET_MM > legal_offset or 100.0 + legal_offset > 200.0:
        return drive, flags, 0, 3, 0.0, 0

    for index in range(len(line)):
        before = max(0, index - 2)
        after = min(len(line) - 1, index + 2)
        dx = f32(line[after].x - line[before].x)
        dy = f32(line[after].y - line[before].y)
        heading = f32(math.atan2(dx, dy) * (180.0 / math.pi))
        drive[index].heading_cdeg = int16_round(f32(wrap_deg(heading) * 100.0))

    corridor_count = 0
    for _ in range(PATH_CORRIDOR_MAX_COUNT):
        best: tuple[int, int, float] | None = None
        for begin in range(PATH_CORRIDOR_END_GUARD_POINTS, len(line)):
            if begin + PATH_CORRIDOR_MIN_SPAN_POINTS + PATH_CORRIDOR_END_GUARD_POINTS >= len(line):
                break
            for span in range(PATH_CORRIDOR_MIN_SPAN_POINTS, PATH_CORRIDOR_MAX_SPAN_POINTS + 1):
                end = begin + span
                if end + PATH_CORRIDOR_END_GUARD_POINTS >= len(line):
                    break
                if any(flags[index] & 0x01 for index in range(begin, end + 1)):
                    continue
                saving = evaluate_candidate(line, drive, begin, end)
                if saving is None:
                    continue
                best_span = (best[1] - best[0]) if best is not None else 0
                if best is None or saving > best[2] or (saving == best[2] and span > best_span):
                    best = (begin, end, saving)
        if best is None:
            break
        apply_candidate(line, drive, flags, best[0], best[1])
        corridor_count += 1

    if corridor_count == 0:
        return [RoutePoint(point.x, point.y, point.heading_cdeg) for point in line], flags, 0, 4, 0.0, 0
    for index in range(len(line)):
        if point_distance(line[index].x, line[index].y, drive[index].x, drive[index].y) > PATH_CORRIDOR_MAX_OFFSET_MM + PATH_CORRIDOR_OFFSET_TOLERANCE_MM:
            return [RoutePoint(point.x, point.y, point.heading_cdeg) for point in line], flags, 0, 5, 0.0, 0
    for i in range(len(line) - 1):
        for j in range(i + 3, len(line) - 1):
            if segment_intersects(drive, i, j) and not segment_intersects(line, i, j):
                return [RoutePoint(point.x, point.y, point.heading_cdeg) for point in line], flags, 0, 6, 0.0, 0
    source_length = path_length(line)
    shortcut_length = path_length(drive)
    reduction = f32(source_length - shortcut_length)
    if source_length <= 0.0 or reduction < PATH_CORRIDOR_MIN_SAVING_MM:
        return [RoutePoint(point.x, point.y, point.heading_cdeg) for point in line], flags, 0, 7, 0.0, 0
    compute_headings(drive)
    return drive, flags, level, 1, reduction, corridor_count


def extend_to_origin(line: list[RoutePoint], drive: list[RoutePoint], flags: list[int]) -> None:
    if len(drive) < 2:
        raise ValueError("終端延長不可:経路点不足")
    end_x, end_y = f32(drive[-1].x), f32(drive[-1].y)
    distance = f32(math.sqrt(f32(f32(end_x * end_x) + f32(end_y * end_y))))
    if distance < 1.0:
        raise ValueError("終端延長不可:終点が原点")
    unit_x = f32(-end_x / distance)
    unit_y = f32(-end_y / distance)
    advanced = 0.0
    for _ in range(PATH_GOAL_EXTENSION_POINTS):
        next_advanced = min(PATH_GOAL_EXTENSION_MM, f32(advanced + PATH_ROUTE_SPACING_MM))
        x = int16_round(f32(end_x + f32(unit_x * next_advanced)))
        y = int16_round(f32(end_y + f32(unit_y * next_advanced)))
        drive.append(RoutePoint(x, y))
        line.append(RoutePoint(x, y))
        flags.append(0)
        advanced = next_advanced
    compute_headings(line)
    compute_headings(drive)


def geometry_crc32(route: RouteBuild | tuple[list[RoutePoint], list[RoutePoint]]) -> int:
    if isinstance(route, RouteBuild):
        line, drive = route.line, route.drive
    else:
        line, drive = route
    payload = bytearray(struct.pack("<H", len(line)))
    for source, target in zip(line, drive):
        payload.extend(struct.pack("<hhhh", source.x, source.y, target.x, target.y))
    return zlib.crc32(payload) & 0xFFFFFFFF


def build_route(source: CsvLog, requested_level: int, max_level: int) -> RouteBuild:
    raw_points = _read_route_points(source)
    if not raw_points:
        raise ValueError("元ログに有効なXY点がありません")
    first_x, first_y, _ = raw_points[0]
    previous_x, previous_y = first_x, first_y
    total_length = 0.0
    for x, y, _ in raw_points[1:]:
        total_length = f32(total_length + point_distance(previous_x, previous_y, x, y))
        previous_x, previous_y = x, y
    if total_length < PATH_ROUTE_SPACING_MM:
        raise ValueError("元ログの経路長が短すぎます")

    line = [RoutePoint(0, 0)]
    flags = [0]
    raw_traversed = 0.0
    accumulated = 0.0
    previous_raw_x = 0.0
    previous_raw_y = 0.0
    previous_corrected_x = 0.0
    previous_corrected_y = 0.0
    first_corrected = True
    last_x, last_y = raw_points[-1][0], raw_points[-1][1]
    for raw_x, raw_y, _ in raw_points:
        if not first_corrected:
            raw_traversed = f32(raw_traversed + point_distance(previous_raw_x, previous_raw_y, raw_x, raw_y))
        previous_raw_x, previous_raw_y = raw_x, raw_y
        progress = min(1.0, f32(raw_traversed / total_length))
        progress = f32(progress)
        blend = f32(f32(progress * progress) * f32(3.0 - f32(2.0 * progress)))
        corrected_x = f32(f32(raw_x - first_x) + f32(0.0 * blend))
        corrected_y = f32(f32(raw_y - first_y) + f32(0.0 * blend))
        if first_corrected:
            previous_corrected_x, previous_corrected_y = corrected_x, corrected_y
            first_corrected = False
            continue
        segment_start_x, segment_start_y = previous_corrected_x, previous_corrected_y
        remaining = point_distance(segment_start_x, segment_start_y, corrected_x, corrected_y)
        while f32(accumulated + remaining) >= PATH_ROUTE_SPACING_MM:
            if len(line) >= PATH_ROUTE_MAX_POINTS - PATH_GOAL_EXTENSION_POINTS:
                raise ValueError("元ログ経路が最大点数を超えました")
            needed = f32(PATH_ROUTE_SPACING_MM - accumulated)
            ratio = f32(needed / remaining)
            segment_start_x = f32(segment_start_x + f32(f32(corrected_x - segment_start_x) * ratio))
            segment_start_y = f32(segment_start_y + f32(f32(corrected_y - segment_start_y) * ratio))
            line.append(RoutePoint(int16_round(segment_start_x), int16_round(segment_start_y)))
            flags.append(0)
            remaining = f32(remaining - needed)
            accumulated = 0.0
        accumulated = f32(accumulated + remaining)
        previous_corrected_x, previous_corrected_y = corrected_x, corrected_y

    line[-1].x = int16_round(f32(last_x - first_x))
    line[-1].y = int16_round(f32(last_y - first_y))
    compute_headings(line)
    drive, flags, applied, status, reduction, corridor_count = generate_shortcut(line, requested_level, max_level)
    if status != 1:
        drive = [RoutePoint(point.x, point.y, point.heading_cdeg) for point in line]
        compute_headings(drive)
        applied = 0
    extend_to_origin(line, drive, flags)
    built = RouteBuild(line, drive, flags, requested_level, applied, status, corridor_count, reduction, 0)
    built.geometry_crc32 = geometry_crc32(built)
    return built


def _missing_values(count: int, reason: str, source_log: int) -> list[dict[str, Any]]:
    return [
        {
            "linePointX_mm": math.nan,
            "linePointY_mm": math.nan,
            "pathLegalMargin_mm": math.nan,
            "recovery_status": "missing",
            "recovery_reason": reason,
            "recovery_source_log": source_log,
        }
        for _ in range(count)
    ]


def _saved_values(log: CsvLog) -> list[dict[str, Any]]:
    values: list[dict[str, Any]] = []
    for row in log.rows:
        values.append({
            "linePointX_mm": float(row["linePointX_mm"]),
            "linePointY_mm": float(row["linePointY_mm"]),
            "pathLegalMargin_mm": float(row["pathLegalMargin_mm"]),
            "recovery_status": "saved",
            "recovery_reason": "CSV保存値を使用",
            "recovery_source_log": parameter_int(log.parameters, "routeSourceLog", 0) or 0,
        })
    return values


def _missing_recovery(
    log: CsvLog,
    reason: str,
    source_log: int = 0,
    source_path: Path | None = None,
    route: RouteBuild | None = None,
) -> Recovery:
    values = _missing_values(len(log.rows), reason, source_log)
    return Recovery("missing", reason, source_log, source_path, route, values, len(values))


def _summarize_recovery(
    log: CsvLog,
    base_reason: str,
    source_log: int,
    source_path: Path | None,
    route: RouteBuild | None,
    values: list[dict[str, Any]],
) -> Recovery:
    missing_samples = sum(value.get("recovery_status") == "missing" for value in values)
    if missing_samples == 0:
        status = "restored"
        reason = base_reason
    elif missing_samples == len(values):
        status = "missing"
        reason = base_reason
    else:
        status = "partial"
        reason = f"{base_reason}; {missing_samples}行が欠測"
    return Recovery(status, reason, source_log, source_path, route, values, missing_samples)


def _route_header_mismatch(log: CsvLog, route: RouteBuild) -> str | None:
    expected_count = parameter_int(log.parameters, "routePointCount")
    if expected_count is None:
        return "routePointCountがありません"
    if expected_count != route.route_count:
        return f"routePointCount不一致: header={expected_count}, regenerated={route.route_count}"
    expected_crc = parameter_int(log.parameters, "routeGeometryCrc32")
    if expected_crc is None:
        return "routeGeometryCrc32がありません"
    if expected_crc != route.geometry_crc32:
        return f"routeGeometryCrc32不一致: header={expected_crc}, regenerated={route.geometry_crc32}"
    expected_version = parameter_int(log.parameters, "routeControllerVersion")
    if expected_version not in PATH_ROUTE_CONTROLLER_VERSIONS:
        return f"未対応のrouteControllerVersion={expected_version}"
    expected_level = parameter_int(log.parameters, "shortcutLevel")
    if expected_level is None or expected_level != route.applied_level:
        return f"shortcutLevel不一致: header={expected_level}, regenerated={route.applied_level}"
    expected_status = parameter_int(log.parameters, "shortcutBuildStatus")
    if expected_status is None or expected_status != route.build_status:
        return f"shortcutBuildStatus不一致: header={expected_status}, regenerated={route.build_status}"
    expected_corridors = parameter_int(log.parameters, "shortcutCorridorCount")
    if expected_corridors is None or expected_corridors != route.corridor_count:
        return f"shortcutCorridorCount不一致: header={expected_corridors}, regenerated={route.corridor_count}"
    expected_reduction = parameter_number(log.parameters, "shortcutReduction_mm")
    if math.isnan(expected_reduction) or abs(expected_reduction - route.reduction_mm) > 0.02:
        return f"shortcutReduction_mm不一致: header={expected_reduction}, regenerated={route.reduction_mm}"
    return None


def recover_path_columns(
    log: CsvLog,
    source_log_dir: Path | None = None,
    shortcut_level: int | None = None,
    max_level: int | None = None,
) -> Recovery:
    saved_columns = {"linePointX_mm", "linePointY_mm", "pathLegalMargin_mm"}
    if saved_columns <= set(log.fields):
        return Recovery("saved", "CSV保存値を使用", parameter_int(log.parameters, "routeSourceLog", 0) or 0,
                        None, None, _saved_values(log), 0)

    mode = parameter_int(log.parameters, "optimalTrace", -1)
    if mode not in PATH_MODES:
        return Recovery("not_applicable", "非PATH走行のため経路追従評価対象外", 0, None, None,
                        _missing_values(len(log.rows), "非PATH走行", 0), 0)

    version = parameter_int(log.parameters, "logSchemaVersion")
    if version not in (2, 3, 4, 5, 6):
        return _missing_recovery(log, f"未対応または不明なlogSchemaVersion={version}")
    source_number = parameter_int(log.parameters, "analysisSourceLog", 0) or 0
    if source_number <= 0:
        return _missing_recovery(log, "analysisSourceLogが0または不明")
    source_dir = source_log_dir if source_log_dir is not None else log.path.parent
    source_path = source_dir / f"{source_number}.csv"
    if not source_path.exists():
        return _missing_recovery(log, f"元ログがありません: {source_path}", source_number, source_path)

    requested = parameter_int(log.parameters, "shortcutRequestedLevel")
    assumed = False
    if requested is None:
        if shortcut_level is None:
            return _missing_recovery(log, "shortcutRequestedLevelがありません", source_number, source_path)
        requested = shortcut_level
        assumed = True
    generation_max = parameter_int(log.parameters, "routeShortcutSettings.maxLevel")
    if generation_max is None:
        generation_max = max_level
        assumed = True
    if generation_max is None:
        return _missing_recovery(log, "経路生成時maxLevelがありません", source_number, source_path)

    try:
        source = read_csv_log(source_path)
        route = build_route(source, requested, generation_max)
    except (OSError, ValueError, struct.error) as exc:
        reason = f"経路再生成失敗: {exc}"
        return _missing_recovery(log, reason, source_number, source_path)
    mismatch = _route_header_mismatch(log, route)
    if mismatch is not None:
        return _missing_recovery(log, mismatch, source_number, source_path, route)
    route_version = parameter_int(log.parameters, "routeControllerVersion")

    values: list[dict[str, Any]] = []
    for row in log.rows:
        raw_index = row.get("optimalIndex")
        index = parse_optimal_index(raw_index, route.route_count)
        if index is None:
            values.append({**_missing_values(1, optimal_index_error(raw_index, route.route_count), source_number)[0]})
            continue
        line = route.line[index]
        offset = point_distance(line.x, line.y, route.drive[index].x, route.drive[index].y)
        margin = f32(f32(PATH_LINE_HALF_WIDTH_MM + PATH_OCCUPIED_HALF_WIDTH_MM) - offset - PATH_TRACKING_ERROR_BUDGET_MM)
        values.append({
            "linePointX_mm": float(line.x),
            "linePointY_mm": float(line.y),
            "pathLegalMargin_mm": float(margin),
            "recovery_status": "restored",
            "recovery_reason": f"Version {route_version}経路を元ログから再生成",
            "recovery_source_log": source_number,
        })
    reason = "指定設定による復元（過去実機設定の証明ではない）" if assumed else f"Version {route_version}経路を元ログから再生成"
    for value in values:
        if value["recovery_status"] == "restored":
            value["recovery_reason"] = reason
    return _summarize_recovery(log, reason, source_number, source_path, route, values)
