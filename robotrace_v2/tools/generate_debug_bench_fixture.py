#!/usr/bin/env python3
"""Generate the Debug-only route and pose fixture from two saved run CSVs."""

from __future__ import annotations

import argparse
import csv
import hashlib
import math
from pathlib import Path


def round_away(value: float) -> int:
    return math.floor(value + 0.5) if value >= 0.0 else math.ceil(value - 0.5)


def read_run(path: Path) -> tuple[dict[str, str], list[dict[str, str]]]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.reader(stream))
    header_rows = [index for index, row in enumerate(rows) if row and row[0] == "cntlog"]
    if len(header_rows) != 1:
        raise ValueError(f"{path}: expected one cntlog header")
    header_index = header_rows[0]
    metadata: dict[str, str] = {}
    for row in rows[:header_index]:
        if not row:
            continue
        for field in row:
            if "=" in field:
                key, value = field.split("=", 1)
                metadata[key] = value
    names = rows[header_index]
    records = []
    for row in rows[header_index + 1 :]:
        if not row or not row[0].isdigit():
            continue
        records.append({name: row[index] for index, name in enumerate(names) if index < len(row)})
    if len(records) < 128:
        raise ValueError(f"{path}: only {len(records)} data rows")
    return metadata, records


def route_points(records: list[dict[str, str]]) -> list[tuple[int, int]]:
    points = [(0, 0)]
    previous_x = previous_y = accumulated = 0.0
    previous_pulse = 0.0
    for row in records:
        x = float(row["x"])
        y = float(row["y"])
        pulse = float(row["encTotalOptimal"])
        if not all(math.isfinite(value) for value in (x, y, pulse)) or pulse < previous_pulse:
            raise ValueError("source log contains invalid or non-monotonic route data")
        start_x, start_y = previous_x, previous_y
        remaining = math.hypot(x - start_x, y - start_y)
        while remaining > 0.0 and accumulated + remaining >= 40.0:
            needed = 40.0 - accumulated
            ratio = needed / remaining
            start_x += (x - start_x) * ratio
            start_y += (y - start_y) * ratio
            points.append((round_away(start_x), round_away(start_y)))
            remaining -= needed
            accumulated = 0.0
        accumulated += remaining
        previous_x, previous_y, previous_pulse = x, y, pulse
    if math.dist(points[-1], (previous_x, previous_y)) >= 0.5:
        points.append((round_away(previous_x), round_away(previous_y)))
    if len(points) < 2:
        raise ValueError("source log produced fewer than two route points")
    if any(not -32768 <= value <= 32767 for point in points for value in point):
        raise ValueError("route coordinates exceed int16 range")
    headings = []
    for index in range(len(points)):
        before = points[max(0, index - 1)]
        after = points[min(len(points) - 1, index + 1)]
        headings.append(math.degrees(math.atan2(after[0] - before[0], after[1] - before[1])))
    heading_steps = [abs((after - before + 180.0) % 360.0 - 180.0) for before, after in zip(headings, headings[1:])]
    if sum(step < 0.8 for step in heading_steps) < 20 or sum(step > 2.0 for step in heading_steps) < 20:
        raise ValueError("source route must include both straight and curved sections")
    return points


def pose_samples(records: list[dict[str, str]]) -> list[tuple[int, int, int, int, int]]:
    raw = [(0, 0.0, 0.0)]
    previous_time = 0
    for row in records:
        time_ms = int(row["cntlog"])
        x = float(row["x"])
        y = float(row["y"])
        if time_ms <= previous_time or time_ms > 40000 or not math.isfinite(x + y):
            raise ValueError("replay log has invalid time or position")
        raw.append((time_ms, x, y))
        previous_time = time_ms

    result = []
    arc_mm = 0.0
    previous_x = previous_y = 0.0
    for index, (time_ms, x, y) in enumerate(raw):
        before = raw[max(0, index - 1)]
        after = raw[min(len(raw) - 1, index + 1)]
        dx = after[1] - before[1]
        dy = after[2] - before[2]
        heading = math.degrees(math.atan2(dx, dy)) if dx or dy else 0.0
        if index:
            arc_mm += math.hypot(x - previous_x, y - previous_y)
        sample = (time_ms, round_away(x), round_away(y), round_away(heading * 100.0), round_away(arc_mm))
        if not 0 <= sample[0] <= 65535 or not -32768 <= sample[1] <= 32767 or not -32768 <= sample[2] <= 32767:
            raise ValueError("replay sample exceeds firmware fixture range")
        if not -32768 <= sample[3] <= 32767 or not 0 <= sample[4] <= 65535:
            raise ValueError("replay heading or arc length exceeds firmware fixture range")
        result.append(sample)
        previous_x, previous_y = x, y
    return result


def emit_array(lines: list[str], c_type: str, name: str, values: list[tuple[int, ...]], columns: int) -> None:
    lines.append(f"static const {c_type} {name}[{len(values)}] = {{")
    for offset in range(0, len(values), columns):
        chunk = values[offset : offset + columns]
        lines.append("\t" + ", ".join("{" + ", ".join(map(str, item)) + "}" for item in chunk) + ",")
    lines.append("};")
    lines.append("")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True, type=Path, help="primary source CSV (12418.csv)")
    parser.add_argument("--replay", required=True, type=Path, help="PATH replay CSV (12420.csv)")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    source_meta, source_records = read_run(args.source)
    replay_meta, replay_records = read_run(args.replay)
    if source_meta.get("optimalTrace") != "0.00":
        raise ValueError("source log must be a primary run")
    if replay_meta.get("routeSourceLog") != "12418":
        raise ValueError("replay log must identify 12418 as its route source")
    if replay_meta.get("emcStop") != "0.00":
        raise ValueError("replay log did not finish normally")
    if int(source_records[-1]["cntlog"]) != 36483 or int(replay_records[-1]["cntlog"]) != 36734:
        raise ValueError("fixture durations changed; update DEBUG_BENCH_*_DURATION_MS with the reviewed logs")

    route = route_points(source_records)
    samples = pose_samples(replay_records)
    source_hash = hashlib.sha256(args.source.read_bytes()).hexdigest()
    replay_hash = hashlib.sha256(args.replay.read_bytes()).hexdigest()
    shortcut_settings = {
        "MAX_LEVEL": int(float(replay_meta["shortcutSettings.maxLevel"])),
        "LOOKAHEAD_BASE_MM": int(float(replay_meta["shortcutSettings.lookaheadBaseMm"])),
        "LOOKAHEAD_PER_MPS_MM": int(float(replay_meta["shortcutSettings.lookaheadPerMpsMm"])),
        "KLATERAL_X100": int(float(replay_meta["shortcutSettings.kLateral_x100"])),
        "KHEADING_X100": int(float(replay_meta["shortcutSettings.kHeading_x100"])),
        "LINE_ALPHA_X1000": int(float(replay_meta["shortcutSettings.lineAlpha_x1000"])),
        "LINE_THETA_GAIN_X1E9": int(float(replay_meta["shortcutSettings.lineThetaGain_x1e9"])),
    }
    output_lines = [
        "#ifndef DEBUG_BENCH_FIXTURE_DATA_H_",
        "#define DEBUG_BENCH_FIXTURE_DATA_H_",
        "",
        "#include \"pathFollower.h\"",
        "",
        "typedef struct { uint16_t time_ms; int16_t x_mm; int16_t y_mm; int16_t heading_cdeg; uint16_t arc_mm; } DebugBenchPoseSample;",
        "",
        f"/* Generated from 12418.csv SHA256 {source_hash}. */",
        f"/* Replay from 12420.csv SHA256 {replay_hash}; route source metadata={replay_meta['routeSourceLog']}. */",
        f"#define DEBUG_BENCH_ROUTE_COUNT {len(route)}U",
        f"#define DEBUG_BENCH_POSE_COUNT {len(samples)}U",
        *[f"#define DEBUG_BENCH_SETTING_{key} {value}U" for key, value in shortcut_settings.items()],
        "",
    ]
    emit_array(output_lines, "PathBenchRoutePoint", "debugBenchSourceRoute", route, 4)
    emit_array(output_lines, "DebugBenchPoseSample", "debugBenchReplayPose", samples, 3)
    output_lines.extend(["#endif", ""])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(output_lines), encoding="utf-8", newline="\n")
    print(f"route={len(route)} replay={len(samples)} duration={samples[-1][0]} ms")


if __name__ == "__main__":
    main()
