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
- During auto-start, completed intermediate runs remain numbered `.bin` logs on the robot. All sealed runs are converted after series completion/abort or emergency stop; manual runs convert at completion. CSVs become available only after this conversion finishes.
- CSV metadata adds `binaryLogNumber`, `binaryDataCrc`, and `binarySchema` as conversion identity. CRC text is signed 32-bit; compare bit patterns when needed. Data columns and units are unchanged.
- Binary recovery/format is documented in `doc/deferred-log-storage.md`. Do not parse `.tmp`, `.part`, or `.bin` as CSV. Recovery failures preserve the binary and block new starts. The six frozen pre-SHADOW/initial-SHADOW/lateral-candidate formats support either current profile; current light72/debug87 match the frozen lateral-candidate schemas, and light43 logs remain readable as the frozen pre-SHADOW schema.
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

- On 2026-10-14 the user requested restoration of the light profile to 72 CSV columns / 183 bytes, including the 19 SHADOW and 10 lateral-candidate diagnostics. Detailed remains 87 columns / 220 bytes; detector calculations and legacy control/plans are unchanged. The temporary light43 / 108-byte period from 2026-10-13, including 13212–13227, does not contain these diagnostics. Other historical generations have 62 columns / 157 bytes (initial SHADOW) and 43 columns / 108 bytes (pre-SHADOW). Individual encoders, line sensors, acceleration, six IMU diagnostic columns and both motor currents remain in their existing positions. The trailing empty CSV cell is excluded from this count.
- Logs 12872–12878 used the temporary minimal 13-column/22-byte profile. Raw-wheel, independent line-CROSS, temperature-series and XYZ-angle analyses cannot use those logs; check required column names before analysis.
- The debug profile has 87 columns and 220 bytes. The unchanged 87-column/220-byte lateral-candidate schema is frozen in `log_schema_candidate_v1.h`; other frozen predecessors have 77 columns / 194 bytes (initial SHADOW) and 58 columns / 145 bytes (pre-SHADOW). All six supported binary formats can be recovered by either current build profile using strict profile/schema CRC/record-length identification; predecessor CSV keeps its original columns. Metadata validation and firmware secondary-run required fields remain available in both profiles.

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
- `courseMarker`: confirmed marker state while running: 1=RIGHTMARKER, 2=LEFTMARKER, 3=CROSSLINE. A right-marker row is not equivalent to an SG increment; `checkStartGoalMarker()` also requires distance/qualification conditions. Do not infer missing SG counts from right-marker row counts. Match verified cross events for closure alignment and use saved `goalMarkerOnset_p` for the final onset.
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
- `x`, `y`: estimated position from the start marker origin, `[mm]`. After the 12879–12884 recovery, firmware CSV generation uses `encTotalOptimal` differences and midpoint heading from stored 1-ms `imuAngle_Z`; the closure X check uses the same integration. Earlier firmware reintegrated sparse `gyroVal_Z` and held the latest corrected speed across each logging interval. Sparse integration can alias and must be compared with stored yaw before interpreting large course distortions.
- `linePointX_mm`, `linePointY_mm`: corresponding first-run line point, `[mm]`.
- `pathErrorY_mm`: signed lateral path error, `[mm]`.
- `pathErrorHeading_cdeg`: heading error, `[0.01 deg]`.
- `pathState`: 1 tracking, 2 line fallback, 3 rejoin blend, 4 localization lost.
- `pathLegalMargin_mm`: remaining line-overlap margin after the tracking-error budget, `[mm]`.

## SHADOW slip diagnostics (2026-10-12)

These 29 diagnostics are restored to both the 72-column light and 87-column detailed profiles on 2026-10-14. Logs from the temporary 43-column period contain no `slipV2*`, `imuYawAlphaFiltered` or `imuLatTangentialApplied` samples; report them as unrecorded, never healthy zero. Detector metadata alone does not establish recorded diagnostics. Check actual columns, schema and build information before running candidate-analysis scripts.

- `ROBOTRACE_SLIP_DETECTOR=SHADOW` is the default. `LEGACY` disables the new detector. Existing `slipFlag` / `slipFlagLat`, next-run plans and distance correction still use the legacy detector.
- New residuals: `slipV2LongFast`, `slipV2LongSlow`, `slipV2Lat` [m/s²], `slipV2YawDps` [deg/s]. Flags are a bit mask: longitudinal=1, lateral=2, yaw mismatch=4. These are model inconsistencies, not measured physical slip amounts.
- From 2026-10-14 detector version 3 (`slipLongPolicy=fast_only`), only LongFast drives the longitudinal flag: abs >10 m/s² for 3ms ON, abs <8 m/s² for 5ms OFF. Version 2 requires both Fast and Slow for ON and OFF. Windows, thresholds, validity and residuals remain fixed; LongSlow stays diagnostic. `slipV2OnLongMs` follows the selected rule, while `slipV2PeakLong` still takes the maximum absolute value of both windows. Identical schemas do not identify the decision rule: check metadata and leave unknown policy unresolved. See `doc/slip-long-fast-only.md`.
- Only interpret new residuals/flags when `slipV2Valid=1`. `slipV2Reason` records current invalidity; `slipV2IntervalReason` ORs invalid reasons across the whole recording interval. See AGENTS.md and doc/slip-detector-v2.md for bit values.
- `slipV2PeakLong/PeakLat/PeakYawDps` preserve interval absolute maxima. `slipV2OnLongMs/OnLatMs/OnYawMs/ValidMs/TotalMs` accumulate observed 1ms calls, saturating at 65535. Use valid observed time as the denominator; exclude saturated intervals and report missing/invalid time. The unrecorded tail after the final row is not included.
- `slipV2CurrentValid` requires calibration and a fresh finite read; `slipV2CurrentHighSamples` counts distinct 10ms acquisitions, not held 1ms samples. `slipV2PwmSum` is the saturated absolute PWM sum (0..2000). Current/PWM do not gate new slip flags.
- Missing new columns in old CSV mean unrecorded, never healthy zero. Do not compare old/new flags as ground truth. Existing 13074..13088 interpolation replay lacks the stored static-gravity reference and acquisition-success history and is only a diagnostic approximation.

## Lateral tangential candidate diagnostics

- See `doc/slip-lateral-candidate.md`. The candidate is diagnostic only; common IMU values and initial SHADOW residuals are unchanged. It replaces only the already-applied X tangential term with its opposite sign: `currentLinearX - 2*appliedTangentialX`. The right-X assumption remains physically unverified. Do not fit the observed regression coefficient into the correction.
- `slipV2LatCandidate` and `slipV2LatCandidatePeak` are the signed residual and interval absolute peak [m/s²]; `slipV2LatCandidateFlag` is ON=1, not a bitmask. `slipV2LatCandidateValid/Reason` are independent of the initial SHADOW validity. Reason 256 means tangential sample unavailable and applies only to the candidate; other reason bits match v2.
- `slipV2LatCandidateOnMs/ValidMs` and `slipV2LatCandidateIntervalReason` preserve interval ON/valid time and invalidity OR. Counters saturate at 65535. Use candidate ValidMs for candidate rates and initial SHADOW ValidMs for baseline rates. Exclude saturated intervals and report unmatched validity. Never compare intervals with candidate-only missing data as equal conditions.
- `imuYawAlphaFiltered` [rad/s²] and `imuLatTangentialApplied` [m/s²] are instantaneous snapshots from the existing correction. They are not 30ms-averaged residuals. Align smoothing and intervals before studying angular-acceleration dependence; sparse CSV cannot reproduce the actual 1ms filter history.
- Check candidate version/formula/axes/offset/LPF metadata and 72/87 columns in the first real log after restoration. The temporary 43-column light logs cannot support candidate diagnostics. Initial SHADOW logs 13094..13124 lack these 10 columns: report candidate diagnostics as unrecorded, not zero. Their sparse reconstruction cannot prove candidate performance.

## Straight-road wobble and acceleration residuals

- For straight-road braking wobble, define road geometry from the validated primary route/plan and match secondary `optimalIndex` to that plan. Do not select straight road by small measured gyro/alpha: that excludes the wobble being investigated. Keep geometry, commanded acceleration/braking, and measured yaw activity as separate labels. Check source plan parameters and source identity before reusing a host planner. Inspect a straight-interior boundary sensitivity and label the primary geometry as estimated, not physical ground truth.
- Evaluate the user's two hypotheses separately: braking-time Yaw inconsistency with actual gyro/heading/steering reversals, and acceleration-time positive LongFast with wheel-speed growth exceeding IMU acceleration. Negative LongFast is not the same acceleration-spin evidence. Fast/Slow have different windows; check whether the version-2 both-window condition suppresses a short candidate and distinguish version-3 Fast-only flags by metadata. Do not adopt new thresholds from these descriptive statistics. See `doc/slip-turn-residuals.md` and `analysis/script/analyze_straight_accel_brake_13228_13247.py`.
- Windows Illegal System DLL Relocation / WinError 623 may affect existing GCC-built host executables as well as compilers. Use the same authorized read-only host analysis in the permitted execution environment; do not change firmware to work around that launch failure. See `doc/失敗事例/2026-10-11-bmi088-forward-y-sign.md`.

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

## Known malformed candidate CSV headers (13130–13144)

These 15 originals have 72 numeric values but only 71 header names: the 30-byte temporary column buffer truncated `slipV2LatCandidateIntervalReason` and lost its comma, joining the next name. Never pass the original directly to a name-based numeric analysis. Use the derived copies under `analysis/slip_candidate_13125_13144/header_repaired/`, created by `analysis/script/check_slip_candidate_13125_13144.py`. Restoration requires the known schema CRC -1299980178, exact expected malformed header reproduced from the source, all 72 data values per row and expected row count. Keep and hash originals; reject unknown corruption. The source fix removes the short temporary buffer and has host/build validation. All 11 post-fix physical CSVs 13145–13155 (22:28:09 build) have the complete 72-column header and matching data rows without repair. Logs 13145–13150 are user-confirmed hand-held/free-spin checks; exclude them from normal-course completion and lap comparisons. The detailed 87-column physical CSV was not captured in this batch. See `doc/失敗事例/2026-10-12-csv-long-column-name-truncation.md`.
