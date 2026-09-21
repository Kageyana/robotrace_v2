---
name: robotrace-log-analysis
description: ロボトレース走行ログを解析、比較、可視化するときに使う。CSVログ、XY軌跡、速度追従、角速度、スリップ、ラップタイム比較、autoStart 5走比較、emcStopやcntlog欠落の判定を扱う。
---

# Robotrace Log Analysis

## Overview

Use this skill when analyzing logs for the robotrace_v2 robot. Treat `AGENTS.md` as the source for project-wide units, log schema, paths, and safety policy.

## Inputs

- Logs live under `F:\Dropbox\Document\robotrace\Log\v2` when accessible.
- Logs are CSV, UTF-8, comma-separated. The current format stores `name=value` metadata on line 1, column names on line 2, and data from line 3 onward. Readers must also accept legacy mixed headers containing column names and metadata on line 1, and old metadata-free column headers.
- The schema source is `robotrace_v2/Core/Inc/log_schema.h`.
- Schema 7 uses 48-byte light records; schema 8 uses 36-byte light records by moving the three linear-acceleration fields to the 121-byte detailed profile. Both keep signed instantaneous left/right encoder samples. `gyroVal_Z` is the mean of 1 ms gyro samples over the logged interval (older versions store the instantaneous value). Schema-7/8 first-run raw `x/y` use successive `encTotalOptimal` counts and midpoint heading; their `x_closed_mm/y_closed_mm` apply the historical marker-constrained X correction. Their marker anchor is at `goalMarkerOnset_p`; include `(0,0)` and the corrected goal marker point `(0,goalMarkerYRaw_mm)` in historical plots. Schema 2–8 logs remain analysis-readable but cannot source a new route on schema-9 firmware.
- Route controller version 14 regenerates schema-7/8 corrected routes, including the exact marker anchor and the actual final sample; `path_log_recovery.py` must verify the regenerated point count and CRC before restoring omitted path columns.
- Schema 9 keeps the 36-byte light record but replaces the two signed 1 ms encoder samples with signed left/right pulse sums over exactly the same interval as `gyroVal_Z`. Its `encoderPulsePerMeter` header is `56687` for old nominal-scale logs or `55116` for later measured-scale logs. Its `x/y` are gyro-only diagnostics and `x_fused_mm/y_fused_mm` are unwarped Kalman-bias-corrected coordinates. Both scales remain analysis-readable for historical PATH recovery with route controller version 16, but cannot source a new route.
- Schema 10 preserves the 36-byte record and interval encoder columns. `encoderPulsePerMeter=58019` passed five powered straight runs at Duty 100, with each mean forward-distance error below 1% even after the UI rounding bound. Builds made before this validation have `distanceScaleVerified=0`; newer builds set it to 1. `closureReason=10` applies to unverified builds when the other closure checks pass. `x/y` are the independent pre-run-IMU-calibrated gyro coordinates used by route controller version 17; `x_fused_mm/y_fused_mm` and `headingKalman.*` are diagnostic only. Markers only validate finish and `|goalMarkerXRaw_mm| <= 20 mm`. Schema 10 requires 100 successful IMU calibration samples, zero calibration read errors, a verified distance scale, and valid closure. `goalMarkerXFusedDiagnostic_mm` may differ from the gyro goal X. Route points and stop extension do not use the marker anchor.
- The log header contains data names and `parameter=value` entries.
- `logSchemaVersion=2` omits `linePointX_mm`, `linePointY_mm`, and `pathLegalMargin_mm` from CSV and binary records. `logSchemaVersion=3` adds the unsigned 16-bit BMI088 `imuTempRaw` code, replaces normal-light `motorpwmL`/`motorpwmR` with signed `encCurrentL`/`encCurrentR` after `ROC`, and also omits `slipFlag`, `slipFlagLat`, `lineTraceCtrl`, `motorVoltageCmdL_mV`, and `motorVoltageCmdR_mV` from the normal light profile, making one binary record 38 bytes. `logSchemaVersion=4` keeps that 38-byte light record and appends `lineMatchResidual_mm`, `poseCorrection_um`, and `poseCorrectionHeading_cdeg` only to the detailed profile. `logSchemaVersion=5` changes only `encCurrentCorr_p` from unsigned 32-bit to signed 16-bit and makes the normal light record 36 bytes; its column name remains unchanged. `logSchemaVersion=6` adds `imuLinearAccelX_mps2`, `imuLinearAccelY_mps2`, and `imuLinearAccelZ_mps2` as float32 `[m/s²]` columns immediately after `gyroVal_Z`, making the normal light record 48 bytes. Existing schema 2～6 CSVs remain readable by header name. Old CSVs containing the three reconstructable columns use the saved values first. Version 5/6 end-of-run headers also record `logRecordSizeBytes`, `dbgOverflowFinal`, `logOverflowFinal`, `distanceKalman.invalidUpdateCount`, `distanceKalman.maxAbsFusedDelta_p`, and `distanceKalman.outputGuardCount`.
- For new PATH logs, recover the three omitted values in memory from `analysisSourceLog` (or `routeSourceLog` only for old logs), the source CSV, the Version 12/13 generator settings, and `optimalIndex`. Search beside the secondary log by default; use `--source-log-dir` for another folder.
- Do not substitute a cntlog-repaired source CSV automatically. Require route point-count, `routeGeometryCrc32`, controller version, and generated-result header checks before recovery. Missing source, unsupported version, CRC mismatch, or out-of-range index produces missing values, not zero or a normal value.
- Firmware-side route parsing resolves required fields by header name, not fixed column number. Normal route generation requires `courseMarker`, `encTotalOptimal`, and `ROC`. The optional additional slip analysis requires a legacy or debug log containing `targetSpeed`, `optimalIndex`, `slipFlag`, and `slipFlagLat`.
- Do not treat the metadata line as a data header. Resolve the column-name row first, then map data by header name.
- Firmware CSV rows end with a comma; `csv.DictReader` exposes the trailing empty field as `""`. Ignore only that unnamed terminal field in completeness checks, and still reject missing named fields, extra values, or row-count mismatches.
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
   - slip when the columns are present; otherwise report it as not recorded
   - lap time comparison table
5. Save generated graphs and tables under `analysis/`.
6. Report comparison target logs, changed condition, adoption decision, and remaining issues.

## Column Meanings

- `cntlog`: time after run start, based on `cntRun`, `[ms]`.
- `encCurrentN`: average left/right encoder pulse count per 1 ms. Builds after the powered distance validation carry the half-pulse remainder between 1 ms samples to prevent cumulative truncation bias.
- `encCurrentL`, `encCurrentR`: signed left/right encoder pulse counts per 1 ms; schema versions 3–8 normal-light logs store them immediately after `ROC`.
- `encIntervalL_p`, `encIntervalR_p`: schema-9 signed left/right interval pulse sums, aligned with the interval-mean `gyroVal_Z`; do not treat these as instantaneous 1 ms samples.
- `encCurrentCorr_p`: signed Kalman-fused distance pulse difference per 1 ms; schema versions 5–8 store it as signed 16-bit.
- `encLog`, `encRightMarker`: raw encoder counters used for log timing and goal-marker distance; they are intentionally independent of fusion validation.
- `enc1`, `encCurve`, `encChangeGain`, `encTotalOptimal`, `encPID`: internal counters updated with the validated fused distance difference.
- `distanceKalman.innovationRejectCount`: count of 1 ms updates where the encoder observation was completely skipped because the innovation exceeded 4σ or the encoder speed exceeded ±10 m/s.
- `distanceKalman.fallbackCount`: count of transitions to raw-encoder fallback while IMU calibration, IMU validity, or finite-value checks were invalid.
- `distanceKalman.invalidUpdateCount`: count of finite-state, covariance-diagonal, or 1 ms fused-distance validation failures that caused a raw-encoder update and covariance reset.
- `distanceKalman.maxAbsFusedDelta_p`: maximum absolute validated fused 1 ms distance difference in pulses.
- `distanceKalman.outputGuardCount`: count of fused-output float values that were non-finite or outside ±535 pulses before integer conversion and therefore used the bounded raw encoder pulse.
- `logRecordSizeBytes`: binary record size recorded at log finalization; Versions 5/8 normal-light logs are 36 bytes, Versions 6/7 are 48 bytes, and Version 8 detailed logs are 121 bytes.
- `dbgOverflowFinal`: final debug/log-buffer overflow counter; `logOverflowFinal=1` means the log buffer reached its limit.
- `gyroVal_Z`: IMU Z angular velocity, `[deg/s]`.
- `imuLinearAccelX_mps2`, `imuLinearAccelY_mps2`, `imuLinearAccelZ_mps2`: X/Y/Z linear acceleration after static gravity-reference removal and rotation-center correction, `[m/s²]`; the Y value is the same value passed to the distance estimator. Schema 8 saves these columns only in detailed logs, so light logs cannot support sample-level acceleration diagnostics.
- `imuTempRaw`: BMI088 temperature register raw 11-bit code stored in a `uint16_t`; `0x400` is the invalid code. Convert with `temperature_C = signed_code * 0.125 + 23` after 11-bit two's-complement decoding.
- `courseMarker`: confirmed marker state while running.
- `encTotalOptimal`: corrected distance count for secondary runs.
- `ROC`: curvature radius, `[mm]`.
- `targetSpeed`: target speed in encoder converted units, `[pulse/ms]`.
- `optimalIndex`: index into `PPAD[]` or `shortCutxycie[]`.
- `slipFlag`: longitudinal slip flag; schema version 3/4 light logs omit it.
- `slipFlagLat`: lateral slip flag; schema version 3/4 light logs omit it.
- `lineTraceCtrl`: value is `lineTraceOmegaFBCtrl.pwm`; schema version 3/4 light logs omit it.
- `targetAngularvelo`: log target angular velocity, `[deg/s]`.
- `motorpwmL`, `motorpwmR`: left/right motor PWM; schema version 3/4 normal-light logs omit them, while detailed debug logs retain them. Schema version 2 normal-light CSVs contain them in their original positions.
- `motorVoltageCmdL_mV`, `motorVoltageCmdR_mV`: motor voltage commands before duty conversion; schema version 3/4 light logs omit them.
- `lineMatchResidual_mm`: observed line point to matched continuous first-run route segment distance `[mm]`; schema version 4 detailed logs only.
- `poseCorrection_um`: translation correction magnitude applied in one 5 ms update `[um]`; schema version 4 detailed logs only.
- `poseCorrectionHeading_cdeg`: heading correction applied in one 5 ms update `[0.01 deg]`; schema version 4 detailed logs only.
- `x`, `y`: estimated position from the start marker origin, `[mm]`.
- `linePointX_mm`, `linePointY_mm`: corresponding first-run line point, `[mm]`; stored in old CSVs and reconstructed in memory for schema version 2/3/4/5/6.
- `pathErrorY_mm`: signed lateral path error, `[mm]`.
- `pathErrorHeading_cdeg`: heading error, `[0.01 deg]`.
- `pathState`: 1 tracking, 2 line fallback, 3 rejoin blend, 4 localization lost.
- `pathLegalMargin_mm`: remaining line-overlap margin after the tracking-error budget, `[mm]`; stored in old CSVs and reconstructed in memory for schema version 2/3/4/5/6.

## Run Mode Checks

- `BOOST_NONE`: verify distance, angular velocity, markers, curvature radius, and XY plot. Pay special attention to angle drift.
- `BOOST_MARKER`: verify all markers detected in the first run are detected.
- `BOOST_DISTANCE`: verify current course position matches the estimated position and first-run distance.
- `BOOST_PATH_REPLAY`: verify path lateral/heading error, fallback count, and line correction validity against the first-run route.
- `BOOST_SHORTCUT`: verify the robot follows the validated shortcut, `pathLegalMargin_mm` remains non-negative, and no localization fallback occurs.
- For schema version 2/3/4/5/6/7, report each reconstructed field as `saved`, `restored`, `partial`, or `missing`, including the reason, source log number, and `recovery_missing_samples`. `restored` requires every row to succeed; mixed row results are `partial`, and no successful row is `missing`.
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
