#!/usr/bin/env python3
"""16 bit cntlogの折り返しを補正し、派生XY座標を再計算する。"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

from path_log_recovery import read_csv_log, recover_path_columns
from robotrace_units import PULSE_MILLIMETER


CNTLOG_MODULUS = 1 << 16
REQUIRED_COLUMNS = {"cntlog", "encCurrentCorr_p", "gyroVal_Z", "x", "y"}


def signed_int16(value: str) -> int:
    raw = int(float(value)) & 0xFFFF
    return raw - 0x10000 if raw >= 0x8000 else raw


def decode_distance_pulse(value: str, schema_version: int | None) -> int:
    """Version 5/6はS16、Version 2～4は旧U32として復号する。"""
    if schema_version is not None and schema_version >= 5:
        return signed_int16(value)
    # 旧ファームウェアはU32へキャストして保存していたが、1msのパルス値は
    # int16_tの値域内である。従来解析と同じく下位16ビットを符号付きで読む。
    return signed_int16(value)


def repair_log(source_path: Path, destination_path: Path) -> dict[str, float | int]:
    with source_path.open("r", encoding="utf-8-sig", newline="") as source:
        rows = list(csv.reader(source))
    if len(rows) < 2:
        raise ValueError(f"{source_path}: データ行がありません")

    first_nonempty = [cell for cell in rows[0] if cell]
    header_index = 1 if first_nonempty and all("=" in cell for cell in first_nonempty) else 0
    if header_index >= len(rows):
        raise ValueError(f"{source_path}: 列名行がありません")
    header = rows[header_index]
    metadata = {}
    if header_index == 1:
        for field in first_nonempty:
            name, value = field.split("=", 1)
            metadata[name.strip()] = value.strip()
    try:
        schema_version = int(float(metadata["logSchemaVersion"])) if "logSchemaVersion" in metadata else None
    except ValueError as exc:
        raise ValueError(f"{source_path}: logSchemaVersionが不正です") from exc
    data_start = header_index + 1
    if len(rows) <= data_start:
        raise ValueError(f"{source_path}: データ行がありません")
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

    for row_number, row in enumerate(rows[data_start:], start=data_start + 1):
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
        distance_mm = decode_distance_pulse(row[enc_index], schema_version) / PULSE_MILLIMETER * delta_ms
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
        "samples": len(rows) - data_start,
        "wrap_count": wrap_count,
        "wrap_csv_row": wrap_row,
        "last_cntlog_ms": previous_time,
        "duration_ms": previous_time - int(float(rows[data_start][cnt_index])),
        "final_x_mm": x_mm,
        "final_y_mm": y_mm,
        "max_step_mm": max_step_mm,
    }


def plot_route(csv_path: Path, plot_path: Path, source_log_dir: Path | None = None) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError("Matplotlibが必要です") from exc

    log = read_csv_log(csv_path)
    rows = log.rows

    robot_x = [float(row["x"]) for row in rows]
    robot_y = [float(row["y"]) for row in rows]
    recovery = recover_path_columns(log, source_log_dir)
    reference_segments: list[list[tuple[float, float]]] = []
    current_segment: list[tuple[float, float]] = []
    for value in recovery.row_values:
        x = float(value["linePointX_mm"])
        y = float(value["linePointY_mm"])
        if math.isfinite(x) and math.isfinite(y):
            current_segment.append((x, y))
        elif current_segment:
            reference_segments.append(current_segment)
            current_segment = []
    if current_segment:
        reference_segments.append(current_segment)

    figure, axis = plt.subplots(figsize=(9, 8))
    if reference_segments:
        for segment_index, segment in enumerate(reference_segments):
            axis.plot([point[0] for point in segment], [point[1] for point in segment], "--", linewidth=1.2,
                      color="tab:orange", label=f"Reference route ({recovery.status})" if segment_index == 0 else None)
    else:
        axis.text(0.02, 0.98, f"reference route unavailable: {recovery.reason}",
                  transform=axis.transAxes, va="top", color="tab:red", wrap=True)
    if recovery.missing_samples > 0:
        axis.text(0.02, 0.90, f"reference missing {recovery.missing_samples} samples",
                  transform=axis.transAxes, va="top", color="tab:red")
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
    parser.add_argument("--source-log-dir", type=Path,
                        help="復元元ログを探すフォルダ。既定は補修ログと同じフォルダ")
    args = parser.parse_args()

    result = repair_log(args.source, args.destination)
    if args.plot is not None:
        plot_route(args.destination, args.plot, args.source_log_dir)

    for name, value in result.items():
        print(f"{name}={value}")
    print(args.destination)
    if args.plot is not None:
        print(args.plot)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
