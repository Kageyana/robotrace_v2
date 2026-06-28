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

### Speed and Acceleration Parameters

File: `targetSpeeds.txt`

- Implementation: `control.c`, `writeTgtspeeds()`, `readTgtspeeds()`
- Format: 18 fixed-width comma-separated fields.
- Write format: each item `%04d,`
- Read format: each item `%04hd,`
- No newline.
- Stored values are rounded real values multiplied by 100, then divided by 100 on read.
- Order: `search`, `stop`, `bstStraight`, `bst1500`, `bst1300`, `bst1000`, `bst800`, `bst700`, `bst600`, `bst500`, `bst400`, `bst300`, `bst200`, `bst100`, `acceleF`, `acceleD`, `shortCut`, `decelLeadMm`.
- `search`, `stop`, `bst*`, `shortCut` are speeds `[m/s]`.
- `acceleF`, `acceleD` are accelerations `[m/s^2]`.
- `decelLeadMm` is distance `[mm]`.

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

## Corruption Handling

- If a setting file is partially readable, apply only readable values.
- Use code defaults for out-of-range values.
- Repair corrupted files by overwriting with defaults or valid values.
- Prohibit running if `lsval.txt` is corrupt.
- Parser changes must preserve partial reflection, out-of-range defaulting, repair behavior, and `lsval.txt` run prohibition.
