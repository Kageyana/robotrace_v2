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
- `logSchemaVersion=2` omits `linePointX_mm`, `linePointY_mm`, and `pathLegalMargin_mm` from CSV and binary records. Old CSVs containing all three columns use the saved values first.
- For new PATH logs, recover the three omitted values in memory from `analysisSourceLog` (or `routeSourceLog` only for old logs), the source CSV, the Version 12 generator settings, and `optimalIndex`. Search beside the secondary log by default; use `--source-log-dir` for another folder.
- Do not substitute a cntlog-repaired source CSV automatically. Require route point-count, `routeGeometryCrc32`, controller version, and generated-result header checks before recovery. Missing source, unsupported version, CRC mismatch, or out-of-range index produces missing values, not zero or a normal value.
- Firmware-side secondary-log parsing resolves required fields by header name, not fixed column number. Required fields are `courseMarker`, `encTotalOptimal`, `ROC`, `targetSpeed`, `optimalIndex`, `slipFlag`, and `slipFlagLat`.
- Distinguish run mode by `optimalTrace`.
- Exclude failed runs when `emcStop != 0`.
- Invalidate logs with `cntlog` gaps or processing drops; identify and report the cause.
- If battery voltage differs significantly, recommend charging and retrying instead of comparing as equal conditions.

## Workflow

1. Inspect available logs and identify target log numbers.
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
- `x`, `y`: estimated position from the start marker origin, `[mm]`.
- `linePointX_mm`, `linePointY_mm`: corresponding first-run line point, `[mm]`; stored in old CSVs and reconstructed in memory for schema version 2.
- `pathErrorY_mm`: signed lateral path error, `[mm]`.
- `pathErrorHeading_cdeg`: heading error, `[0.01 deg]`.
- `pathState`: 1 tracking, 2 line fallback, 3 rejoin blend, 4 localization lost.
- `pathLegalMargin_mm`: remaining line-overlap margin after the tracking-error budget, `[mm]`; stored in old CSVs and reconstructed in memory for schema version 2.

## Run Mode Checks

- `BOOST_NONE`: verify distance, angular velocity, markers, curvature radius, and XY plot. Pay special attention to angle drift.
- `BOOST_MARKER`: verify all markers detected in the first run are detected.
- `BOOST_DISTANCE`: verify current course position matches the estimated position and first-run distance.
- `BOOST_PATH_REPLAY`: verify path lateral/heading error, fallback count, and line correction validity against the first-run route.
- `BOOST_SHORTCUT`: verify the robot follows the validated shortcut, `pathLegalMargin_mm` remains non-negative, and no localization fallback occurs.
- For schema version 2, report each reconstructed field as `saved`, `restored`, `partial`, or `missing`, including the reason, source log number, and `recovery_missing_samples`. `restored` requires every row to succeed; mixed row results are `partial`, and no successful row is `missing`.
- Accept `optimalIndex` only when it is finite, integer-valued, non-negative, and within the regenerated route. Blank, fractional, NaN, infinity, negative, and out-of-range values are missing. Compute index differences only between adjacent valid rows; never bridge across a missing row.
- If any reconstructed row is missing, report the whole-run legal margin as indeterminate rather than taking the minimum of only valid rows. Continue speed, slip, state, and recorded path-error analysis. Plot reference-route segments separately so missing intervals are not connected, and annotate the missing sample count.
- Compare only logs with the same run mode; distance, path replay, and shortcut modes are not equivalent.

## Output Rules

- Put analysis scripts in `analysis/script/`.
- Use Python when useful.
- Save results in `analysis/`.
- Name outputs so the target log number and analysis type are clear.
- Examples: `log_00012_summary.csv`, `log_00012_xy.png`.
- If the input contract is not yet implemented, decide it before adding a script: single log, multi-log comparison, and autoStart 5-run comparison are separate modes.
