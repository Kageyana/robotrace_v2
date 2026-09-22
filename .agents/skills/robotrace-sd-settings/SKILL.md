---
name: robotrace-sd-settings
description: robotrace_v2のSDカード内設定ファイルを読む、書く、修復する、保存形式を変更するときに使う。targetSpeeds.txt、PID設定、lsval.txt、lognum.txt、analysis.txt、boostログの形式を扱う。
---

# Robotrace SD Settings

## Overview

Use this skill when editing or reviewing SD-card setting file behavior. Treat `AGENTS.md` as the source for high-level precedence and safety policy.

## General Policy

- `./setting/*.txt` on the robot-side SD card is the source for real-machine settings.
- SD values take precedence over code defaults.
- Running without an SD card is allowed, but display a warning.
- If an SD card is inserted and a setting file is missing, create the target file with code defaults.
- Files are usually headerless ASCII numeric text, comma-separated or a single numeric value.
- Fixed-width files have read code that assumes the field shape; change readers and writers together.

## Setting File Formats

### PID Gains

Files: `line.txt`, `lineomega.txt`, `speed.txt`, `yawRate.txt`, `yaw.txt`, `dist.txt`

- Implementation: `PIDcontrol.c`, `writePIDparameters()`, `readPIDparameters()`
- Format: `kp,ki,kd`
- Write format: `%03d,%03d,%03d`
- Read format: `%d,%d,%d`
- No newline.
- Values are `int16_t` control gains.

### Speed Feedforward

File: `speed_ff.txt`

- Implementation: `PIDcontrol.c`, `writeSpeedFeedForwardGain()`, `readSpeedFeedForwardGain()`
- Format: `gain`
- Write format: `%03d`
- Read format: `%hd`
- No newline.
- Value is `speedFeedForwardGain`, with Crr multiplied by 1000.

### BMI088 Temperature Compensation

File: `imu_temp.txt`

- Implementation: `IMU.c`, `readImuTempCompensation()`
- Format: one signed integer `zSlope_x1000000`.
- No newline.
- Valid range: `-100000..100000`.
- Missing, corrupt, or out-of-range values are repaired to `0`; the robot remains runnable with temperature compensation effectively disabled.
- The runtime coefficient in `[deg/s/°C]` is the stored integer divided by `1000000`.

### Speed and Acceleration Parameters

File: `targetSpeeds.txt`

- Implementation: `control.c`, `writeTgtspeeds()`, `readTgtspeeds()`
- Format: 19 fixed-width comma-separated fields.
- Write format: each item `%04d,`
- Read format: each item `%04hd,`
- No newline.
- Stored values are rounded real values multiplied by 100, then divided by 100 on read.
- Order: `search`, `stop`, `bstStraight`, `bst1500`, `bst1300`, `bst1000`, `bst800`, `bst700`, `bst600`, `bst500`, `bst400`, `bst300`, `bst200`, `bst100`, `acceleF`, `acceleD`, `shortCut`, `decelLeadMm`, `pathReplay`.
- `search`, `stop`, `bst*`, `shortCut`, `pathReplay` are speeds `[m/s]`.
- `acceleF`, `acceleD` are accelerations `[m/s^2]`.
- `decelLeadMm` is distance `[mm]`.
- `pathReplay` is the Level 0 PATH REPLAY speed cap. Existing 18-field files retain all existing values and are repaired by appending its code default.

### Path Replay and Shortcut

File: `shortcut.txt`

- Implementation: `pathFollower.c`, `writeShortcutSettings()`, `readShortcutSettings()`
- Format: `maxLevel,lookaheadBaseMm,lookaheadPerMpsMm,Klateral_x100,Kheading_x100,lineAlpha_x1000`
- Write format: `%u,%03u,%03u,%04u,%04u,%03u`
- No newline.
- `maxLevel` is `0..3`; SD absent keeps Level 0.
- Until machine footprint and sensor coordinates are verified, keep `PATH_SHORTCUT_GEOMETRY_ENABLE=0` and `lineAlpha_x1000=000`.
- Partial reads apply valid fields; invalid or missing fields use defaults and the file is repaired.

### Line Sensor Calibration

File: `lsval.txt`

- Implementation: `lineSensor.c`, `writeLinesenval()`, `readLinesenval()`
- Format: `NUM_SENSORS` max values followed by `NUM_SENSORS` min values.
- Current `NUM_SENSORS` is 10, so there are 20 fields.
- Write format: each item `%04d,`
- Read format: each item `%hu,`
- No newline.
- Order: `lSensorMax[0]` to `lSensorMax[9]`, then `lSensorMin[0]` to `lSensorMin[9]`.
- If corrupt, running is prohibited by policy.

### Analysis and Log Numbers

File: `analysis.txt`

- Implementation: `courseAnalysis.c`, `saveLogNumber()`, `getLogNumber()`
- Format: 5-digit zero-padded log number.
- Write format: `%05d`
- Read format: `%5hd`
- Stores the analyzed log number.

File: `lognum.txt`

- Implementation: `SDcard.c`, `writeSavedLogNumber()`, `readSavedLogNumber()`
- Format: 5-digit zero-padded log number.
- Write format: `%05d`
- Read format: `%d`
- Used for next saved log number.
- If missing, unreadable, or `<= 0`, start from SD max log number + 1, or 1.

### Boost Speed Log

File: `boost_%05d.csv`

- Implementation: `courseAnalysis.c`
- Written when `WRITE_BOOSTSPEED_LOG` is enabled.
- Header: `index,boost_speed`
- Row format: `index,boost_speed`, with `boost_speed` as `%.3f`.
- Slip-analysis SD read errors can append diagnostic rows to the same file.

### Log provenance and schema versions 2, 3, and 4

- Firmware logs set `logSchemaVersion=2` and omit `linePointX_mm`, `linePointY_mm`, and `pathLegalMargin_mm` from both CSV and binary records. The internal path values remain available to the controller.
- Firmware logs set `logSchemaVersion=3` for the temperature-compensation measurement format. Version 3 keeps the Version 2 path-column omission, replaces the two motor voltage command fields with `imuTemp_cdegC`, and uses a 42-byte binary record in the normal profile or a 95-byte binary record in the detailed debug profile.
- Firmware logs set `logSchemaVersion=4` for the 1 ms cumulative IMU yaw-angle measurement format. Version 4 keeps the Version 3 temperature column and replaces `motorpwmL`/`motorpwmR` with `imuYawAngle_deg` at the same 4-byte record position; the normal and detailed record sizes remain 42 and 95 bytes.
- Version 4 CSV files require non-running parameters on line 1, running-record column names on line 2, and data from line 3. Both header lines must exist and terminate with a newline; missing or malformed headers are invalid.
- The Version 3 header records the 2-second IMU temperature calibration statistics: `imuTempCalibrationValid`, `imuTempCalibrationStart_C`, `imuTempCalibration_C`, `imuTempCalibrationEnd_C`, `imuTempCalibrationSamples`, `imuTempCalibrationReadErrors`, and `imuGyroOffsetZ_dps`, together with the compensation coefficient and enable state.
- Every run records `analysisSourceLog` and `slipSourceLog`. Primary runs, unknown sources, failed analysis, and unused slip analysis use `0`; a successful PATH analysis also keeps `routeSourceLog` for compatibility and the two values must match.
- PATH headers record `routePointCount`, `routeGeometryCrc32`, `shortcutRequestedLevel`, applied `shortcutLevel`, `shortcutBuildStatus`, `shortcutCorridorCount`, `shortcutReduction_mm`, and both the run-start `shortcutSettings.*` and generation-time `routeShortcutSettings.*` values. The six setting fields are `maxLevel`, `lookaheadBaseMm`, `lookaheadPerMpsMm`, `kLateral_x100`, `kHeading_x100`, and `lineAlpha_x1000`.
- PC analysis searches the secondary log directory for `analysisSourceLog` and regenerates the Version 12 route in memory. It must verify point count, CRC, generator result headers, and `optimalIndex` before restoring the three fields. It never overwrites the original CSV or silently replaces the source with a cntlog-repaired copy. For XY and yaw analysis, Version 4 uses `imuYawAngle_deg` directly; Version 2/3 uses integrated `gyroVal_Z`, and the method must be recorded and not mixed unconditionally.
- Missing source logs, unsupported schema/controller versions, CRC mismatches, and invalid indices are reported as missing with a reason and source number. Non-PATH runs are outside route-following evaluation. Store primary and secondary logs together for PC recovery; do not change the existing SD log deletion policy.

## Corruption Handling

- If a setting file is partially readable, apply only readable values.
- Use code defaults for out-of-range values.
- Repair corrupted files by overwriting with defaults or valid values.
- Prohibit running if `lsval.txt` is corrupt.
- Parser changes must preserve partial reflection, out-of-range defaulting, repair behavior, and `lsval.txt` run prohibition.
