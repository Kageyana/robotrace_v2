---
name: robotrace-log-analysis
description: ロボトレース走行ログを解析、比較、可視化するときに使う。CSVログ、XY軌跡、速度追従、角速度、スリップ、ラップタイム比較、autoStart 5走比較、emcStopやcntlog欠落の判定を扱う。
---

# Robotrace Log Analysis

## Overview

Use this skill when analyzing logs for the robotrace_v2 robot. Treat `AGENTS.md` as the source for project-wide units, log schema, paths, and safety policy.

## Inputs

- Logs live under `F:\Dropbox\Document\robotrace\Log\v2` when accessible.
- Logs are CSV, UTF-8, comma-separated, with a header.
- The schema source is `robotrace_v2/Core/Inc/log_schema.h`.
- The log header contains data names and `parameter=value` entries.
- Distinguish run mode by `optimalTrace`.
- Exclude failed runs when `emcStop != 0`.
- Invalidate logs with `cntlog` gaps or processing drops; identify and report the cause.
- When selecting recent logs, inspect the newest 10 logs first. If none of those are valid, continue checking older logs until a valid log is found, and report that the search range was expanded.
- If battery voltage differs significantly, recommend charging and retrying instead of comparing as equal conditions.

## Workflow

1. Inspect available logs and identify target log numbers. Start with the newest 10 logs; if all are invalid, continue to older logs instead of stopping.
2. Confirm the run mode from `optimalTrace` and compare only compatible run types.
3. Check validity:
   - `emcStop == 0`
   - `cntlog` is monotonic and plausible for distance-based logging
   - required columns from `log_schema.h` exist
   - `batteryVoltage_V` is suitable for comparison
4. Analyze required plots and tables:
   - XY trajectory
   - speed tracking
   - angular velocity
   - slip
   - lap time comparison table
   - theoretical lap and actual-minus-theoretical gap for each `autoStart=2/3/4/5`
5. Save generated graphs and tables under `analysis/`.
6. Report comparison target logs, changed condition, adoption decision, and remaining issues.

## Column Meanings

- `cntlog`: time after run start, based on `cntRun`, `[ms]`.
- `encCurrentN`: average left/right encoder pulse count per 1 ms.
- `gyroVal_Z`: IMU Z angular velocity, `[deg/s]`.
- `courseMarker`: confirmed marker state while running.
- `encTotalOptimal`: corrected distance count for secondary runs.
- `ROC`: curvature radius, `[mm]`.
- `targetSpeed`: target speed in encoder converted units, `[pulse/ms]`.
- `optimalIndex`: index into `PPAD[]` or `shortCutxycie[]`.
- `slipFlag`: longitudinal slip flag.
- `slipFlagLat`: lateral slip flag.
- `lineTraceCtrl`: current log column name; value is `lineTraceOmegaFBCtrl.pwm`.
- `targetAngularvelo`: log target angular velocity, `[deg/s]`.
- `motorpwmL`, `motorpwmR`: left/right motor PWM.
- `slipLongResidual_mps2`: longitudinal slip residual acceleration, `[m/s^2]`.
- `slipLatResidual_mps2`: lateral slip residual acceleration, `[m/s^2]`.
- `slipThresholdHigh`, `slipThresholdLow`: longitudinal slip ON/OFF thresholds, `[m/s^2]`.
- `slipLongOnCountEnabled`: longitudinal slip ON-count gate state.
- `slipLongLowloadClearCount`: consecutive low-load samples used to clear held longitudinal slip.
- `slipLatHigh`, `slipLatLow`: lateral slip ON/OFF thresholds, `[m/s^2]`.
- `slipCurScale`: current-based slip threshold scale.
- `slipTurningState`: turning hysteresis state used by slip detection.
- `slipAxBias`, `slipAyBias`: acceleration bias values used by slip detection, `[m/s^2]`.
- `x`, `y`: estimated position from the start marker origin, `[mm]`.

## Run Mode Checks

- `BOOST_NONE`: verify distance, angular velocity, markers, curvature radius, and XY plot. Pay special attention to angle drift.
- `BOOST_MARKER`: verify all markers detected in the first run are detected.
- `BOOST_DISTANCE`: verify current course position matches the estimated position and first-run distance.
- `BOOST_SHORTCUT`: verify the robot follows the planned optimal path and does not fully leave the line.
- Do not directly compare `BOOST_DISTANCE` and `BOOST_SHORTCUT` as equivalent modes.

## Theoretical Lap Evaluation

- For each complete `autoStart=1,2,3,4,5` set, use that set's first run as the course source for runs 2 through 5.
- Build PPAD-aligned course segments from every five first-run distance-log samples. Use `encTotalOptimal / 54.324` for physical segment distance and retain the final partial segment. Use first-run `x/y`, `ROC`, and `gyroVal_Z` for course shape and signed curvature.
- Aggregate each secondary run's `targetSpeed` by `optimalIndex`. Use the most frequent value and use the final value when frequencies tie. Interpolate only internal gaps when target-index coverage is at least 99%; reject edge gaps, lower coverage, and non-positive speeds.
- Treat each PPAD boundary as a target-speed command change. Use the earliest logged `encCurrentN / 54.324` after the start-marker reset as that run's initial measured speed, then integrate acceleration and deceleration within each segment using that log's `tgtParam.acceleF` and `tgtParam.acceleD`. Cruise at the new target after it is reached; carry the attained speed into the next segment when it is not reached.
- Use the acceleration-aware value as `theoretical_lap_ms`. Also retain the old instantaneous-switch value as `instantaneous_theoretical_lap_ms` and report `accel_decel_adjustment_ms = theoretical - instantaneous` for traceability. Use the final `cntlog` as `actual_lap_ms`; report `lap_gap_ms = actual - theoretical` and `lap_gap_pct`.
- Compare the theoretical gap separately for `autoStart=2`, `3`, `4`, and `5`. Do not average different run numbers into one adoption metric.
- Use `analysis/script/theoretical_lap_report.py --start <first> --end <last>` for complete-set reports, or `--base-log <first-run> --secondary-logs <logs...>` for explicit analysis.

## Output Rules

- Put analysis scripts in `analysis/script/`.
- Use Python when useful.
- Save results in `analysis/`.
- Name outputs so the target log number and analysis type are clear.
- Examples: `log_00012_summary.csv`, `log_00012_xy.png`.
- Standard secondary-run evaluation includes `base_log`, `course_distance_mm`, `theoretical_lap_ms`, `actual_lap_ms`, `lap_gap_ms`, `lap_gap_pct`, target-index coverage, and interpolated-index count. Save the per-run, per-autoStart, course-geometry, and excluded-log CSV files generated by `theoretical_lap_report.py`.
- Use `analysis/script/slip_detection_report.py` for slip detection review. It writes `log_<range>_summary.csv`, `log_<range>_slip_by_index.csv`, and `log_<range>_label_candidates.csv`. Use `--long-lowload-speederr-max`, `--long-lowload-isum-max`, and `--long-lowload-clear-count` to simulate longitudinal low-load gating and held-flag clearing.
- If the input contract is not yet implemented, decide it before adding a script: single log, multi-log comparison, and autoStart 5-run comparison are separate modes.
