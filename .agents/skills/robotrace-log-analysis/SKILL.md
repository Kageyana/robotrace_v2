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
- Version 4 CSV logs require non-running parameters on line 1, running-record column names on line 2, and data records from line 3. Both header lines must exist and terminate with a newline; missing or malformed headers are invalid logs.
- Temperature-compensated runs record `imuTempCalibrationValid`, `imuTempCalibrationStart_C`, `imuTempCalibration_C`, `imuTempCalibrationEnd_C`, `imuTempCalibrationSamples`, `imuTempCalibrationReadErrors`, `imuGyroOffsetZ_dps`, `imuTempCompEnabled`, `imuTempCoeff_dpsPerC`, and `imuTempEnd_C` in the header. Compare these values before attributing gyro drift changes to compensation.
- IMU calibration runs for 2 seconds: 2000 gyro samples at 1 ms and 20 temperature reads at 100 ms intervals. `imuTempCalibration_C` is the mean of valid temperature reads, `imuTempCalibrationValid` requires at least 16 valid reads, and `imuTempCalibrationReadErrors` counts invalid reads. A zero coefficient disables compensation but does not discard the statistics.
- `logSchemaVersion=2` / `3` / `4` omit `linePointX_mm`, `linePointY_mm`, and `pathLegalMargin_mm` from CSV and binary records. Version 3 is based on Version 2 and adds `imuTemp_cdegC` at the former motor-voltage-command position. Version 4 is based on Version 3, replaces the two motor PWM columns at the same 4-byte position with `imuYawAngle_deg` (`float`, cumulative 1 ms yaw angle), and keeps the 42-byte normal-profile or 95-byte detailed-profile record size. Old CSVs containing all three path columns use the saved values first.
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
   - `logSchemaVersion=3` or `4`
   - the two former voltage-command columns and `motorpwmL/R` are absent
   - `imuTemp_cdegC` column exists and uses `-32768` for invalid temperature
   - header temperature sample count plus read errors equals 20
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
- `gyroVal_Z`: IMU Z angular velocity, `[deg/s]`; used for `ROC` calculation and as the legacy yaw source.
- `imuYawAngle_deg`: Version 4 cumulative IMU Z angle after offset and temperature correction, updated at the 1 ms control period, `[deg]`.
- `courseMarker`: confirmed marker state while running.
- `encTotalOptimal`: corrected distance count for secondary runs.
- `ROC`: curvature radius, `[mm]`.
- `targetSpeed`: target speed in encoder converted units, `[pulse/ms]`.
- `optimalIndex`: index into `PPAD[]` or `shortCutxycie[]`.
- `slipFlag`: longitudinal slip flag.
- `slipFlagLat`: lateral slip flag.
- `lineTraceCtrl`: current log column name; value is `lineTraceOmegaFBCtrl.pwm`.
- `targetAngularvelo`: log target angular velocity, `[deg/s]`.
- `imuTemp_cdegC`: BMI088 temperature updated every 5 ms, multiplied by 100, `[0.01°C]`; `-32768` means invalid.
- `x`, `y`: estimated position from the start marker origin, `[mm]`; Version 4 uses the stored 1 ms yaw angle directly, while Version 2/3 logs use `gyroVal_Z` re-integration.
- `linePointX_mm`, `linePointY_mm`: corresponding first-run line point, `[mm]`; stored in old CSVs and reconstructed in memory for schema version 2/3.
- `pathErrorY_mm`: signed lateral path error, `[mm]`.
- `pathErrorHeading_cdeg`: heading error, `[0.01 deg]`.
- `pathState`: 1 tracking, 2 line fallback, 3 rejoin blend, 4 localization lost.
- `pathLegalMargin_mm`: remaining line-overlap margin after the tracking-error budget, `[mm]`; stored in old CSVs and reconstructed in memory for schema version 2/3.

## Run Mode Checks

- `BOOST_NONE`: verify distance, angular velocity, markers, curvature radius, and XY plot. Pay special attention to angle drift.
- `BOOST_MARKER`: verify all markers detected in the first run are detected.
- `BOOST_DISTANCE`: verify current course position matches the estimated position and first-run distance.
- `BOOST_PATH_REPLAY`: verify path lateral/heading error, fallback count, and line correction validity against the first-run route.
- `BOOST_SHORTCUT`: verify the robot follows the validated shortcut, `pathLegalMargin_mm` remains non-negative, and no localization fallback occurs.
- For the BMI088 temperature-compensation experiment, collect five new `BOOST_NONE` baseline runs with `imu_temp.txt=0` using Version 4. Existing Version 3 runs remain reference data only. Require `gitCommit`, `logSchemaVersion=4`, `imuTempCompEnabled=0`, `emcStop=0`, plausible `cntlog`, and start voltage at least 7.5 V before regression.
- Regress each run's lap-end gyro angle error against the temperature integral `I_n = integral((T(t)-Tcal) dt)` while allowing a run-specific steady bias `b_r t`. Accept a coefficient candidate only when at least four runs agree in sign and leave-one-run-out fits do not change sign. Store `round(K_T * 1000000)` in `setting/imu_temp.txt` only within `-100000..100000`.
- Repeat five matched runs with compensation enabled and compare six-lap absolute yaw error, median/max error, residual temperature-drift coefficient, goal X/Y error, completion rate, lap repeatability, and log continuity. Return the setting to `0` if the adoption criteria are not met.
- For schema version 2, 3, or 4, report each reconstructed field as `saved`, `restored`, `partial`, or `missing`, including the reason, source log number, and `recovery_missing_samples`. `restored` requires every row to succeed; mixed row results are `partial`, and no successful row is `missing`.
- Record the yaw source in summaries as exactly `stored_1ms_angle` for Version 4 or `integrated_log_gyro` for older logs. Do not combine different yaw sources in an unconditional trajectory or yaw-error comparison; include the schema version and source in tables and plots.
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
