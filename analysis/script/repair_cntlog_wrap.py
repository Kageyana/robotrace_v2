#!/usr/bin/env python3
"""16 bit cntlogの折り返しを補正し、派生XY座標を再計算する。"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


CNTLOG_MODULUS = 1 << 16
PULSE_MILLIMETER = 54.324
REQUIRED_COLUMNS = {"cntlog", "encCurrentCorr_p", "gyroVal_Z", "x", "y"}


def signed_int16(value: str) -> int:
    raw = int(float(value)) & 0xFFFF
    return raw - 0x10000 if raw >= 0x8000 else raw


def repair_log(source_path: Path, destination_path: Path) -> dict[str, float | int]:
    with source_path.open("r", encoding="utf-8-sig", newline="") as source:
        rows = list(csv.reader(source))
    if len(rows) < 2:
        raise ValueError(f"{source_path}: データ行がありません")

    header = rows[0]
    columns = {name.strip(): index for index, name in enumerate(header)}
    missing = sorted(REQUIRED_COLUMNS - columns.keys())
    if missing:
        raise ValueError(f"{source_path}: 必須列がありません: {', '.join(missing)}")

    cnt_index = columns["cntlog"]
    enc_index = columns["encCurrentCorr_p"]
    gyro_index = columns["gyroVal_Z"]
    x_index = columns["x"]
    y_index = columns["y"]

    previous_raw: int | None = None
    previous_time = 0
    offset = 0
    wrap_count = 0
    heading_deg = 0.0
    x_mm = 0.0
    y_mm = 0.0
    max_step_mm = 0.0
    wrap_row = -1

    for row_number, row in enumerate(rows[1:], start=2):
        raw_time = int(float(row[cnt_index]))
        if previous_raw is not None and raw_time < previous_raw:
            if previous_raw - raw_time <= CNTLOG_MODULUS // 2:
                raise ValueError(
                    f"{source_path}:{row_number}: cntlogが周回ではない後退をしています "
                    f"({previous_raw} -> {raw_time})"
                )
            offset += CNTLOG_MODULUS
            wrap_count += 1
            if wrap_row < 0:
                wrap_row = row_number

        current_time = raw_time + offset
        delta_ms = current_time - previous_time
        if delta_ms <= 0 or delta_ms > 1000:
            raise ValueError(f"{source_path}:{row_number}: 不正な時間差です ({delta_ms} ms)")

        dt = delta_ms / 1000.0
        heading_deg += float(row[gyro_index]) * dt
        distance_mm = signed_int16(row[enc_index]) / PULSE_MILLIMETER * delta_ms
        heading_rad = math.radians(heading_deg)
        dx_mm = distance_mm * math.sin(heading_rad)
        dy_mm = distance_mm * math.cos(heading_rad)
        x_mm += dx_mm
        y_mm += dy_mm
        max_step_mm = max(max_step_mm, math.hypot(dx_mm, dy_mm))

        row[cnt_index] = str(current_time)
        row[x_index] = f"{x_mm:.6f}"
        row[y_index] = f"{y_mm:.6f}"
        previous_raw = raw_time
        previous_time = current_time

    destination_path.parent.mkdir(parents=True, exist_ok=True)
    with destination_path.open("w", encoding="utf-8-sig", newline="") as destination:
        csv.writer(destination, lineterminator="\n").writerows(rows)

    return {
        "samples": len(rows) - 1,
        "wrap_count": wrap_count,
        "wrap_csv_row": wrap_row,
        "last_cntlog_ms": previous_time,
        "duration_ms": previous_time - int(float(rows[1][cnt_index])),
        "final_x_mm": x_mm,
        "final_y_mm": y_mm,
        "max_step_mm": max_step_mm,
    }


def plot_route(csv_path: Path, plot_path: Path) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError("Matplotlibが必要です") from exc

    with csv_path.open("r", encoding="utf-8-sig", newline="") as source:
        reader = csv.DictReader(source)
        rows = list(reader)

    robot_x = [float(row["x"]) for row in rows]
    robot_y = [float(row["y"]) for row in rows]
    line_x = [float(row["linePointX_mm"]) for row in rows]
    line_y = [float(row["linePointY_mm"]) for row in rows]

    figure, axis = plt.subplots(figsize=(9, 8))
    axis.plot(line_x, line_y, "--", linewidth=1.2, color="tab:orange", label="First-run source route")
    axis.plot(robot_x, robot_y, linewidth=1.1, color="tab:blue", label="Robot route (repaired)")
    axis.scatter([robot_x[0]], [robot_y[0]], marker="o", s=45, color="green", label="Start")
    axis.scatter([robot_x[-1]], [robot_y[-1]], marker="x", s=55, color="red", label="Stop")
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x [mm]")
    axis.set_ylabel("y [mm]")
    axis.set_title("Log 12353: route after cntlog wrap repair")
    axis.grid(True, alpha=0.35)
    axis.legend()
    figure.tight_layout()
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(plot_path, dpi=180)
    plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--plot", type=Path)
    args = parser.parse_args()

    result = repair_log(args.source, args.destination)
    if args.plot is not None:
        plot_route(args.destination, args.plot)

    for name, value in result.items():
        print(f"{name}={value}")
    print(args.destination)
    if args.plot is not None:
        print(args.plot)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
