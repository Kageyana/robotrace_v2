"""Find speed-profile indices associated with slip and speed error per autoStart."""

import csv
import math
import sys
from collections import defaultdict
from pathlib import Path
from statistics import mean


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

import compare_logs_11177_11272 as base


LOG_DIR = base.LOG_DIR
OUT_DIR = Path(r"D:\robotrace\robotrace_v2\analysis")
PULSE_METER = 53424.0
BATTERY_START_MIN_V = 7.3

CONDITIONS = {
    "baseline_p12_ff150": [
        range(11542, 11547),
        range(11547, 11552),
        range(11552, 11557),
        range(11557, 11562),
        range(11562, 11567),
    ],
    "latest_p12_ff150": [range(11683, 11688), range(11688, 11693)],
}


def parse_float(value, default=math.nan):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def load(number):
    with (LOG_DIR / f"{number}.csv").open("r", encoding="utf-8-sig", newline="") as fp:
        reader = csv.reader(fp)
        columns, params = base.parse_header(next(reader))
        rows = [dict(zip(columns, row[: len(columns)])) for row in reader if len(row) >= len(columns)]
    return params, rows


def main():
    grouped = defaultdict(list)
    for condition, groups in CONDITIONS.items():
        for set_id, group in enumerate(groups, 1):
            for number in group:
                params, rows = load(number)
                if params.get("optimalTrace") == "0.00" or parse_float(params.get("batteryVoltage_V")) < BATTERY_START_MIN_V:
                    continue
                auto_start = int(parse_float(params.get("autoStart"), 0))
                for row in rows:
                    index = int(parse_float(row.get("optimalIndex"), -1))
                    if index < 0:
                        continue
                    target = parse_float(row.get("targetSpeed"))
                    actual = parse_float(row.get("encCurrentN"))
                    grouped[(condition, auto_start, index)].append(
                        {
                            "set": set_id,
                            "log": number,
                            "target": target,
                            "actual": actual,
                            "slip": int(parse_float(row.get("slipFlag"), 0)) != 0,
                            "slipLat": int(parse_float(row.get("slipFlagLat"), 0)) != 0,
                            "line": parse_float(row.get("lineTraceCtrl")),
                            "gyro": parse_float(row.get("gyroVal_Z")),
                        }
                    )

    output = []
    for (condition, auto_start, index), values in sorted(grouped.items()):
        speed_error = [
            (item["target"] - item["actual"]) / PULSE_METER * 1000.0
            for item in values
            if not math.isnan(item["target"]) and not math.isnan(item["actual"])
        ]
        output.append(
            {
                "condition": condition,
                "autoStart": auto_start,
                "optimalIndex": index,
                "samples": len(values),
                "sets": len({item["set"] for item in values}),
                "targetSpeed_mean_pulse_ms": mean(item["target"] for item in values),
                "speedError_signed_mean_mps": mean(speed_error) if speed_error else math.nan,
                "slip_pct": 100.0 * sum(item["slip"] for item in values) / len(values),
                "slipLat_pct": 100.0 * sum(item["slipLat"] for item in values) / len(values),
                "lineTraceCtrl_rms": math.sqrt(mean(item["line"] * item["line"] for item in values if not math.isnan(item["line"]))),
                "gyroVal_Z_rms": math.sqrt(mean(item["gyro"] * item["gyro"] for item in values if not math.isnan(item["gyro"]))),
                "logs": ",".join(str(item["log"]) for item in values),
            }
        )

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    path = OUT_DIR / "log_11542_11692_speed_profile_by_index.csv"
    with path.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=list(output[0].keys()))
        writer.writeheader()
        writer.writerows(output)
    print(f"output={path}")

    print("latest_high_slip_indices")
    latest = [row for row in output if row["condition"] == "latest_p12_ff150" and row["samples"] >= 20]
    for row in sorted(latest, key=lambda item: (-item["slipLat_pct"], item["autoStart"], item["optimalIndex"]))[:20]:
        print(
            f"auto={row['autoStart']} idx={row['optimalIndex']} samples={row['samples']} "
            f"target={row['targetSpeed_mean_pulse_ms']:.1f} "
            f"slipLat={row['slipLat_pct']:.1f}% slip={row['slip_pct']:.1f}% "
            f"err={row['speedError_signed_mean_mps']:.3f}m/s"
        )


if __name__ == "__main__":
    main()
