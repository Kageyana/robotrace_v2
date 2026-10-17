---
name: robotrace-log-analysis
description: ロボトレース走行ログを解析、比較、可視化するときに使う。CSVログ、XY軌跡、速度追従、角速度、スリップ、ラップタイム比較、autoStart 5走比較、emcStopやcntlog欠落の判定を扱う。
---

# Robotrace Log Analysis

## 実行前の再発防止

最初のコマンドの前に [共通事前確認](../../../doc/skill-execution-preflight.md) を読み、パス・workdir・権限・実行環境を確定する。CSVの探索・読込・集計・比較を始めるときは [分野別の再発防止](references/failure-prevention.md) も読む。関連する失敗記録は最新追記まで確認し、既知の失敗条件を初回操作から回避する。

## Overview

Use this skill when analyzing logs for the robotrace_v2 robot. Treat `AGENTS.md` as the source for project-wide units, log schema, paths, and safety policy.

## Inputs

- Logs live under `F:\Dropbox\Document\robotrace\Log\v2` when accessible.
- Logs are CSV, UTF-8, comma-separated. New logs have `key=value` metadata on line 1, column names on line 2, and data from line 3. Older logs may combine column names and metadata on line 1; resolve columns by name.
- The robot retains numbered `.bin` logs after every run, series completion/abort and emergency stop. It does not automatically convert CSV at shutdown or startup. `C:\tool\AutoCopyCsvFromSd` copies bin into its temp folder, validates all CRCs, converts on the PC, and publishes CSV to the existing log folder. Preserve temp bin originals; see `doc/pc-binary-log-import.md`.
- CSV metadata adds `binaryLogNumber`, `binaryDataCrc`, and `binarySchema` as conversion identity. CRC text is signed 32-bit; compare bit patterns when needed. Data columns and units are unchanged.
- Current binary logs use self-describing RLOG v2: column names, storage types, order, numeric formats and derived calculation version are embedded once per file. The PC converter accepts stored-column additions/removals/reordering without rebuilding. Legacy RLOG v1 six-format support is retired. Robot analysis requires its current build's layout. Header/I/O failures preserve originals and block starts; PC conversion validates all CRCs and preserves originals on failure. See `doc/self-describing-log.md`. Do not parse bin/tmp/part as CSV.
- The schema source is `robotrace_v2/Core/Inc/log_schema.h`.
- The log header contains data names and `parameter=value` entries.
- `logDistanceTargetMm` records the target row spacing; use `encTotalOptimal` deltas to evaluate actual spacing because sensor values update at 1 ms and may exceed the target at high speed.
- New primary logs record `pathSourceFormatVersion=1`, `closureValid`, `closureReason`, `logExpectedRows`, IMU calibration validity/sample/error counts, and distance-scale verification fields in the metadata row. These fields do not change CSV columns or binary records.
- A PATH route source must be a normally completed primary log with source format version 1, valid closure, exact expected row count, successful IMU calibration, and verified distance conversion. Saved legacy-format and invalid primary logs must be rejected as route sources; if the primary validation fails, autorun must stop before the next run.
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

## Light and debug profiles

- The default light profile has 43 CSV columns and a 108-byte stored record, including individual encoders, line sensors, acceleration, six IMU diagnostic columns and both motor currents. The trailing empty CSV cell is excluded from this count.
- Logs 12872–12878 used the temporary minimal 13-column/22-byte profile. Raw-wheel, independent line-CROSS, temperature-series and XYZ-angle analyses cannot use those logs; check required column names before analysis.
- The debug profile retains 58 columns and 145 bytes, including all diagnostic fields. Metadata validation and firmware secondary-run required fields remain available in both profiles.

## Column Meanings

- `cntlog`: time after run start, based on `cntRun`, `[ms]`.
- `encCurrentN`: average left/right encoder pulse count per 1 ms.
- `encCurrentL`, `encCurrentR`: signed left/right pulse counts per 1 ms `[pulse/ms]`; divide by `PULSE_MILLIMETER` for `[m/s]`.
- `encTotalL`, `encTotalR`: signed left/right accumulated pulses `[pulse]`, reset only at the start-marker transition to the timed run. New logs use the start marker as the origin; logs through 12878 accumulated since power-on. The first logged row is after the marker, so it may already be nonzero. Subtract the initial logged value and divide by `PULSE_MILLIMETER` for relative distance from the first row `[mm]`. Individual wheel conversion is diagnostic; the measured scale was verified for the left/right average.
- `lSensorCari0` through `lSensorCari9`: calibrated, normalized line sensor values, dimensionless `0..4095`; index 0 is the leftmost sensor. These and the four individual encoder columns are included in both light and debug profiles.
- `gyroVal_Z`: IMU Z angular velocity, `[deg/s]`.
- `gyroVal_X`, `gyroVal_Y`: offset/direction-corrected X/Y angular velocity, `[deg/s]`. `imuTemp_C`: latest IMU temperature, `[°C]`, acquired every 5 ms; `-999` means invalid.
- `imuAngle_X`, `imuAngle_Y`, `imuAngle_Z`: existing `imuVal.angle.x/y/z`, `[deg]`, reset to zero at the start marker. X/Y are accelerometer-fused attitude; Z is the unwrapped integral of corrected gyro values at 1 ms, including the existing temperature correction. Use Z for yaw drift analysis instead of reintegrating sparsely logged gyro values. These six diagnostic columns are appended in both light and debug profiles; temporary minimal logs lack them.
- `acceleVal_X`, `acceleVal_Y`, `acceleVal_Z`: IMU acceleration after offset calibration, `[g]`, including gravity; X/Y also include the existing rotation-center correction. Multiply by `GRAVITY_MPS2` for `[m/s^2]`. All three axes are included in both light and debug profiles.
- `courseMarker`: confirmed marker state while running.
- `encTotalOptimal`: corrected distance count for secondary runs.
- `ROC`: curvature radius, `[mm]`.
- `targetSpeed`: target speed in encoder converted units, `[pulse/ms]`.
- `optimalIndex`: index into `PPAD[]` or `shortCutxycie[]`.
- `slipFlag`: longitudinal slip flag.
- `slipFlagLat`: lateral slip flag.
- `lineTraceCtrl`: current log column name; value is `lineTraceOmegaFBCtrl.pwm`.
- `motorCurrentL`, `motorCurrentR`: latest left/right motor current `[A]`, updated every 10 ms after 32-sample ADC averaging and zero-offset calibration. Included in both profiles; appended after IMU fields in the light profile since 2026-10-07. Earlier light logs omit them.
- `targetAngularvelo`: log target angular velocity, `[deg/s]`.
- 2026-09-30以降の通常・詳細ログは `motorpwmL`, `motorpwmR` と経路追従専用7列（`linePointX_mm`, `linePointY_mm`, `lineValid`, `pathErrorY_mm`, `pathErrorHeading_cdeg`, `pathState`, `pathLegalMargin_mm`）を出力しない。以下の経路追従専用列の意味は過去ログ向け。専用列必須の解析スクリプトは新ログに使用しない。
- `x`, `y`: スタートマーカー原点の推定位置[mm]。計算版2（logDerivedVersion=2、logXYDistance=raw_wheel_average）は、マーカー補正前の左右生エンコーダ累積値encTotalL/Rの平均距離差と、保存済み1ms積算角度imuAngle_Zの両端平均で計算する。int64の左右和を差分化して0.5倍し、半パルス・後退を保つ。encTotalOptimalのカーブ/CROSS位置補正をXYへ使わない。閉路Xも同じ計算とし、制御中自己位置・速度計画は変更しない。旧計算版1ではencTotalOptimalを使用していたが、ユーザー指定により新変換器の互換読込対象外。さらに古い疎なgyro再積分のログは別の過去形式であり、その形状を実滑りと即断しない。
- `linePointX_mm`, `linePointY_mm`: corresponding first-run line point, `[mm]`.
- `pathErrorY_mm`: signed lateral path error, `[mm]`.
- `pathErrorHeading_cdeg`: heading error, `[0.01 deg]`.
- `pathState`: 1 tracking, 2 line fallback, 3 rejoin blend, 4 localization lost.
- `pathLegalMargin_mm`: remaining line-overlap margin after the tracking-error budget, `[mm]`.

## Run Mode Checks

- `BOOST_NONE`: verify distance, angular velocity, markers, curvature radius, and XY plot. Pay special attention to angle drift.
- `BOOST_MARKER`: verify all markers detected in the first run are detected.
- `BOOST_DISTANCE`: verify current course position matches the estimated position and first-run distance.
- `BOOST_PATH_REPLAY`: verify XY trajectory and completion against the validated primary route. For legacy logs with path diagnostics, also verify lateral/heading error, fallback count, and line-position/heading correction. Goal is the halfway extension from the primary endpoint toward the origin; marker count does not terminate PATH mode.
- `BOOST_SHORTCUT`: verify the Level 1 straight corridor trajectory and completion. For legacy logs with path diagnostics, also verify non-negative `pathLegalMargin_mm` and absence of localization fallback. New logs cannot establish these diagnostic conditions directly. Goal uses the same extended-route arc length as PATH REPLAY.
- Route generation failure, extension failure, or point-capacity overflow must block route start. For automatic runs, invalid primary-source metadata blocks progression to the next run.
- PATH/SHORTCUT tuning remains unadopted until same-condition real-world runs provide at least 10 logs per level; assess completion rate and stop-position repeatability before lap time.
- Compare only logs with the same run mode; distance, path replay, and shortcut modes are not equivalent.

## Output Rules

- Put analysis scripts in `analysis/script/`.
- Use Python when useful.
- Save results in `analysis/`.
- Name outputs so the target log number and analysis type are clear.
- Examples: `log_00012_summary.csv`, `log_00012_xy.png`.
- If the input contract is not yet implemented, decide it before adding a script: single log, multi-log comparison, and autoStart 5-run comparison are separate modes.
