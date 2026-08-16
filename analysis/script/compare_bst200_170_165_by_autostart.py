"""Compare bst200=1.70 and 1.65 using autoStart run numbers separately."""

import csv
import statistics
from pathlib import Path


ROOT = Path(r"D:\robotrace\robotrace_v2\analysis")
BASE = ROOT / "log_11709_11728_slip_frac070_by_autostart.csv"
CANDIDATE = ROOT / "log_11756_11770_bst200165_by_autostart.csv"
OUT = ROOT / "log_11709_11770_bst200_165_comparison_by_autostart.csv"

METRICS = (
    "lap_ms",
    "speed_error_signed_mean_mps",
    "speed_error_abs_p95_mps",
    "lineTraceCtrl_rms",
    "gyroVal_Z_rms",
    "motorDutySat_pct",
    "slip_count",
    "slipLat_count",
)


def load(path):
    with path.open("r", encoding="utf-8", newline="") as fp:
        return list(csv.DictReader(fp))


def mean(rows, key):
    return statistics.fmean(float(row[key]) for row in rows)


def grouped(rows):
    result = {}
    for row in rows:
        result.setdefault(int(row["autoStart"]), []).append(row)
    return result


def main():
    base = grouped(load(BASE))
    candidate = grouped(load(CANDIDATE))
    output = []
    for run_number in (2, 3, 4, 5):
        base_rows = base.get(run_number, [])
        candidate_rows = candidate.get(run_number, [])
        row = {
            "autoStart": run_number,
            "base_count": len(base_rows),
            "candidate_count": len(candidate_rows),
            "base_start_voltage_mean_V": mean(base_rows, "start_voltage_V"),
            "candidate_start_voltage_mean_V": mean(candidate_rows, "start_voltage_V"),
        }
        for metric in METRICS:
            row[f"base_{metric}_mean"] = mean(base_rows, metric)
            row[f"candidate_{metric}_mean"] = mean(candidate_rows, metric)
            row[f"candidate_minus_base_{metric}"] = (
                row[f"candidate_{metric}_mean"] - row[f"base_{metric}_mean"]
            )
        output.append(row)

    with OUT.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=list(output[0]))
        writer.writeheader()
        writer.writerows(output)

    print(f"output={OUT}")
    for row in output:
        print(
            f"autoStart={row['autoStart']} "
            f"lap={row['base_lap_ms_mean']:.1f}->{row['candidate_lap_ms_mean']:.1f} "
            f"lineRms={row['base_lineTraceCtrl_rms_mean']:.2f}->{row['candidate_lineTraceCtrl_rms_mean']:.2f} "
            f"gyroRms={row['base_gyroVal_Z_rms_mean']:.2f}->{row['candidate_gyroVal_Z_rms_mean']:.2f} "
            f"slipLat={row['base_slipLat_count_mean']:.1f}->{row['candidate_slipLat_count_mean']:.1f}"
        )


if __name__ == "__main__":
    main()
