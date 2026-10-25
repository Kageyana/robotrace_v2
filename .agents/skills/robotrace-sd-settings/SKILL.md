---
name: robotrace-sd-settings
description: robotrace_v2のSDカード内設定ファイルを読む、書く、修復する、保存形式を変更するときに使う。targetSpeeds.txt、PID設定、lsval.txt、lognum.txt、analysis.txt、boostログの形式を扱う。
---

# Robotrace SD Settings

## 2026-10-25：現行ログとSLIPフラグ集計

有効性・無効理由（電流有効性を含む）と記録間隔内統計16列はbin/CSVから削除済み。経路診断6列追加後の現行は通常profile=1・56列/154 B、詳細profile=0・62列/163 B。50列/135 B・56列/144 BはV2整理だけの時点のサイズ。`slipV2Flags`はbit0=縦、bit1=採用横、bit2=Yaw不整合。SLIP計画は版3・`slipPlannerPolicy=flag_sample_fraction`で、optimalIndex別の縦/横ON記録数/全記録数を使用する。時間率ではなく、最小ON時間3msの追加選別は廃止した。検出器版4・3/5ms継続判定と内部計算は維持する。

未観測区間は増速しない。有効性が未記録のため、無効時のフラグ0と正常OFF、記録間に終了した短いONは区別できない。更新後は新しい一次とDISTANCEを取得する。機体は旧スキーマを拒否し、PC自己記述変換・全CRC検証は維持する。現行の確認はリポジトリルートから必要権限で`python validation/log_storage/run.py --converter C:/tool/AutoCopyCsvFromSd/RlogToCsv.exe`。以下の旧列数・区間時間率の記述は履歴とし、現行は[正式採用とSLIP走行](../../../doc/slip-v2-active.md)と`log_schema.h`を優先する。

経路診断6列は末尾の`pathErrorY_mm`（F32・mm）、`pathErrorHeading_cdeg`（S16・0.01deg）、`pathState`（U8）、`pathYawFF_dps/pathYawLateralFB_dps/pathYawHeadingFB_dps`（各F32・deg/s）。5ms更新の最新値を距離間隔で保存し、PATH/SHORTCUT以外は0。方位偏差は最近傍接線−機体方位、操舵3成分は合計の±1800deg/s制限前。制御式は維持する。詳細は[自己記述ログ](../../../doc/self-describing-log.md)。

## 実行前の再発防止

最初のコマンドの前に [共通事前確認](../../../doc/skill-execution-preflight.md) を読み、パス・workdir・権限・実行環境を確定する。SD設定・保存・復旧・CSV形式を扱うときは [分野別の再発防止](references/failure-prevention.md) も読む。関連する失敗記録は最新追記まで確認し、既知の失敗条件を初回操作から回避する。


ログ形式変更のPC変換器検証では、`C:/tool/AutoCopyCsvFromSd/RlogToCsv.exe`もGCC生成ホストexeとして最初の起動からrequire_escalatedを使用する。今回の解析フォルダ内の合成bin/出力CSVに限定し、Python subprocessのstdout=PIPE/stderr=STDOUT等で出力を取得、終了コード0・出力実在を確認してから全CRC/列/数値を照合する。PowerShellのStandardOutputEncodingエラーだけで原因を決めず、直接取得した起動エラーを確認する。詳細は[PC変換器起動の追記](../../../doc/失敗事例/2026-10-11-bmi088-forward-y-sign.md)。

## 2026-10-21：V2正式採用後の現行仕様

ユーザー指示によりV1互換・SHADOW切替を削除した。縦はFast単独10/8m/s²・3/5ms、横はLatCandidate版5の6.7/4.5m/s²・3/5ms・LPF0.8を使用する（初回ON6.5、2026-10-24の既存変更でON6.7）。保存検出器版4、`slipDetectorMode=ACTIVE_V2`、`slipLatPolicy=matched_candidate_v5`。通常66列/170B、詳細72列/179B（重複整理前の正式採用ログ13571～13577は70列/181B）。旧slipFlag/slipFlagLatと旧詳細診断9列は存在しない。slipV2Flagsの横bitとslipV2Lat/PeakLat/OnLatMsも採用横と同じ意味へ変更した。以下のSHADOW・旧サイズ・旧制御維持の記述は導入時の履歴として扱い、現行は[正式採用とSLIP走行](../../../doc/slip-v2-active.md)を優先する。

横ログは`slipV2Lat/Flags横bit/PeakLat/OnLatMs`へ統一し、`slipV2LatCandidate*`列を廃止した。横独自の有効性は`slipV2LatValid/Reason/ValidMs/IntervalReason`へ改名し、横ON率は`slipV2OnLatMs / slipV2LatValidMs`を使う。横の理由256・再充填で共通有効性と異なるため共通ValidMsへ代用しない。SLIP読込も新列名だけを使い、更新後は新しい一次とDISTANCEを取得する。判定式・閾値・検出器版4・計画版2は維持し、列定義CRCで保存形式を識別する。

SLIPは同じ一次参照・正常ゴールのDISTANCEログから軸別の有効ms/ON msを集計する。不正・飽和・未観測を滑りなしへ置換しない。毎回一次計画から再生成し、DISTANCE→SLIPのオートスタート接続を確認する。未検証の滑り距離推定は制御へ入れない。最初の新形式実機ログと10本以上の実走行再現性は別に確認する。

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

### IMU temperature compensation

File: `imu_temp.txt`

- Implementation: `SDcard.c` and `IMU.c`.
- Format: one signed integer without a newline, representing the BMI088 gyro Z temperature coefficient `[deg/s/°C]` multiplied by 1,000,000.
- Accepted range: `-100000..100000`. Missing or invalid content is repaired to `0`; zero disables compensation.

### PID Gains

Files: `line.txt`, `lineomega.txt`, `speed.txt`, `yawRate.txt`

- Implementation: `PIDcontrol.c`, `writePIDparameters()`, `readPIDparameters()`
- Format: `kp,ki,kd`
- Write format: `%03d,%03d,%03d`
- Read format: `%d,%d,%d`
- No newline.
- Values are `int16_t` control gains.
- `yaw.txt` and `dist.txt` belonged to removed unused PIDs; do not read or write them.

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

- Implementation: `control.c`, `writeTgtspeeds()`, `readTgtspeeds()`. Structure, default order, UI ranges and storage order share `Core/Inc/speedParameters.h` / `SPEED_PARAM_FIELDS`; preserve the 19-field format when editing this list.
- `SPEED_PARAM_FIELDS` has a final metadata-output flag (0/1). The existing 15 speed metadata keys are emitted from this list in order; keep their names, formatting and selection unchanged. SD storage still uses all 19 fields. The `bst500` default uses `PARAM_BOOST_500` (1.5 m/s), restored by user instruction. Valid existing SD values remain authoritative.
- Format: 19 fixed-width comma-separated fields.
- Write format: each item `%04d,`
- Read format: each item `%04hd,`
- No newline.
- Stored values are rounded real values multiplied by 100, then divided by 100 on read.
- Order: `search`, `stop`, `bstStraight`, `bst1500`, `bst1300`, `bst1000`, `bst800`, `bst700`, `bst600`, `bst500`, `bst400`, `bst300`, `bst200`, `bst100`, `acceleF`, `acceleD`, `shortCut`, `decelLeadMm`, `pathReplay`.
- `search`, `stop`, `bst*`, `shortCut` are speeds `[m/s]`.
- `acceleF`, `acceleD` are accelerations `[m/s^2]`.
- `decelLeadMm` is distance `[mm]`.
- `pathReplay` is the Level 0 PATH REPLAY speed `[m/s]`.
- An 18-field legacy file retains its valid values and is repaired by appending the default `pathReplay` value.

### Path Replay and Shortcut

File: `shortcut.txt`

- Implementation: `pathFollower.c`, `writeShortcutSettings()`, `readShortcutSettings()`
- Format: `maxLevel,lookaheadBaseMm,lookaheadPerMpsMm,Klateral_x100,Kheading_x100,lineAlpha_x1000,lineThetaGain_x1e9`
- Write format: `%u,%03u,%03u,%04u,%04u,%03u,%04u`
- No newline.
- `maxLevel` is `0..1`; default settings are `1,080,040,3000,0600,010,000`.
- A valid six-field legacy file retains its values and is repaired by appending default `lineThetaGain_x1e9`.
- The sensor ordering and relative positions are matched to the KiCad board footprints. The existing central sensor forward offset is 95 mm; it was not independently remeasured against the STEP axle origin in this change.
- Partial reads apply valid fields; invalid or missing fields use defaults and the file is repaired.

### Auto-start run modes

File: `auto_run.txt`

- Implementation: `autoRun.c`, `SDcard.c`, and `control.c`.
- Format: one row for each run number 2 through 5, `runNumber,mode`; modes are `DISTANCE`, `SLIP`, `PATH`, and `SHORTCUT`.
- Default: `2,DISTANCE`, `3,SLIP`, `4,PATH`, `5,SHORTCUT`, one row per line.
- Duplicate modes are allowed. `SLIP` is valid only when the immediately preceding run is configured as `DISTANCE`.
- Missing, malformed, incomplete, duplicate-run, or dependency-invalid configuration is replaced as a whole with the default order. If repair cannot be saved, auto-start is blocked.
- Only a missing-file result or invalid content triggers repair. Other SD open/read/close errors block auto-start without writing the file; the display distinguishes an SD read error from a repair failure.
- Read during startup; reboot after editing the SD card file.
- Run 1 is always the primary run. `DISTANCE`, `PATH`, and `SHORTCUT` use its log. `SLIP` uses the primary log for its distance plan and the immediately preceding successful `DISTANCE` log for slip data.

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

### Current binary logs and analysis

- Runs write numbered `.tmp` files; `endLog()` seals records, schema and metadata with CRC and renames to `.bin`. The on-device CSV conversion API and its legacy tests were removed on 2026-10-24.
- `logSource.h` exposes current-bin-only `logSourceOpen(file, number, purpose)`, `logSourceInfo`, `logSourceMetadata`, `logSourceNext` and close/error/EOF. Metadata is immutable while open; data rows are typed `LogSourceRow`. No generated CSV headers, column maps or CSV fallback are used.
- The robot's selection list contains only bin files. Number allocation still reserves numeric `.csv`, `.bin`, `.tmp` and `.part` names. Keep tmp and all unimported originals. Boot validates headers/schema/length; opening an analysis source verifies every data and metadata CRC.
- The PC converter continues to decode self-describing RLOG v2 and import old CSV files. Current normal/debug profiles are 66 columns/170 B and 72 columns/179 B. Stored fields, encoding, CRC and derivedVersion are unchanged by API cleanup.
- Run `python validation/log_storage/run.py` from the repository root for both profiles, failure injection, current V2, auto-run configuration, source eligibility and all four plans. Add `--converter C:/tool/AutoCopyCsvFromSd/RlogToCsv.exe` to compare PC output values. Use permitted unrestricted execution for GCC and generated exe.
- Do not run removed `run_slip_log_compat_tests.ps1`, CSV-conversion performance runners or tests requiring pre-change snapshots. Check actual stop/save/reboot and the first real log separately.

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
- Slip-analysis SD read errors are reported through ITM and do not write diagnostic rows to the SD card.

## Corruption Handling

- If a setting file is partially readable, apply only readable values.
- Use code defaults for out-of-range values.
- Repair corrupted files by overwriting with defaults or valid values.
- Prohibit running if `lsval.txt` is corrupt.
- Parser changes must preserve partial reflection, out-of-range defaulting, repair behavior, and `lsval.txt` run prohibition.

## PC CSV output validation

The robot no longer implements `logBuildColumns()`/`setLogStr()` or CSV formatting. Validate current column names, separators, row counts and values through the deployed PC converter using the real saved bins exported by `validation/log_storage/run.py`. Keep unknown-schema and CRC rejection. Historical truncated headers remain historical originals; do not reintroduce the old firmware parser to read them.
