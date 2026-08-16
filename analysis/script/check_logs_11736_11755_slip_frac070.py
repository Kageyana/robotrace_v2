"""Check the second CA_SLIP_FRAC_FULL=0.70 log batch."""

import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

import check_logs_11683_11692_speed_ff150 as base


LOG_START = 11736
LOG_END = 11755
OUT_DIR = Path(r"D:\robotrace\robotrace_v2\analysis")


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    rows = [base.summarize(number) for number in range(LOG_START, LOG_END + 1)]
    complete, excluded = base.classify_groups(rows)
    runs = base.per_run_rows(complete)

    summary_path = OUT_DIR / "log_11736_11755_slip_frac070_summary.csv"
    runs_path = OUT_DIR / "log_11736_11755_slip_frac070_by_autostart.csv"
    excluded_path = OUT_DIR / "log_11736_11755_slip_frac070_excluded_sets.csv"
    base.write_csv(summary_path, rows)
    base.write_csv(runs_path, runs)
    base.write_csv(excluded_path, excluded)

    print(f"summary={summary_path}")
    print(f"by_autostart={runs_path}")
    print(f"excluded_sets={excluded_path}")
    print(f"complete_sets={len(complete)}")
    for item in excluded:
        print(f"excluded={item['logs']} sequence={item['autoStart_sequence']} reason={item['reason']}")
    for auto_start in (2, 3, 4, 5):
        print(f"autoStart={auto_start}")
        for row in runs:
            if row["autoStart"] == auto_start:
                print(
                    f"  set={row['set']} log={row['log']} lap={row['lap_ms']}ms "
                    f"startV={float(row['start_voltage_V']):.2f} "
                    f"signedErr={float(row['speed_error_signed_mean_mps']):.3f} "
                    f"p95={float(row['speed_error_abs_p95_mps']):.3f} "
                    f"lineRms={float(row['lineTraceCtrl_rms']):.2f} "
                    f"gyroRms={float(row['gyroVal_Z_rms']):.2f} "
                    f"slip={row['slip_count']}/{row['slipLat_count']}"
                )


if __name__ == "__main__":
    main()
