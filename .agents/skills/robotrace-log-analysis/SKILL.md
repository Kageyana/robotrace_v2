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

## Run Mode Checks

- `BOOST_NONE`: verify distance, angular velocity, markers, curvature radius, and XY plot. Pay special attention to angle drift.
- `BOOST_MARKER`: verify all markers detected in the first run are detected.
- `BOOST_DISTANCE`: verify current course position matches the estimated position and first-run distance.
- `BOOST_SHORTCUT`: verify the robot follows the planned optimal path and does not fully leave the line.
- Do not directly compare `BOOST_DISTANCE` and `BOOST_SHORTCUT` as equivalent modes.

## Output Rules

- Put analysis scripts in `analysis/script/`.
- Use Python when useful.
- Save results in `analysis/`.
- Name outputs so the target log number and analysis type are clear.
- Examples: `log_00012_summary.csv`, `log_00012_xy.png`.
- If the input contract is not yet implemented, decide it before adding a script: single log, multi-log comparison, and autoStart 5-run comparison are separate modes.
