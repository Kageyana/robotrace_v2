---
name: robotrace-sd-settings
description: 実機SDカードのsetting/*.txtを読み書き・修復するときや設定形式/旧形式互換を変更するときに使う。
---

# SD設定ファイル

- 実機の `./setting/*.txt` をコード既定値より優先する。SD未挿入は警告して走行可能、挿入済みで欠落した設定は既定値で作成する。読めた有効項目は保持し、範囲外は既定値へ戻し、破損ファイルは修復する。`lsval.txt` 破損時は走行禁止。パーサー変更時は欠落・途中切れ・範囲外・既存形式の修復を確認する。
- 以下の形式は `robotrace_v2/Core/Src/` の実装を正とする。固定幅・末尾カンマ・改行なし等を独断で統一せず、読み手と書き手を同時に変更する。実機上の値を変更する前にファイルの現物を確認する。

## 形式と実装の起点

| ファイル（`setting/` 下） | 実装 | 保存形式・注意点 |
| --- | --- | --- |
| `line.txt`、`lineomega.txt`、`speed.txt`、`yawRate.txt`、`yaw.txt`、`dist.txt` | `PIDcontrol.c` `readPIDparameters` / `writePIDparameters` | `kp,ki,kd`、書式 `%03d,%03d,%03d`、改行なし。 |
| `speed_ff.txt` | `PIDcontrol.c` `readSpeedFeedForwardGain` / `writeSpeedFeedForwardGain` | 係数整数1項目、`%03d`、改行なし。 |
| `targetSpeeds.txt` | `control.c` `readTgtspeeds` / `writeTgtspeeds` | 順序は `speedParam` の19項目、各値×100を `%04d,` で保存（末尾カンマあり）。第19項目はLevel 0の `pathReplay`。旧18項目は値を保持して末尾の既定値を補い修復する。 |
| `shortcut.txt` | `pathFollower.c` `readShortcutSettings` / `writeShortcutSettings` | `maxLevel,lookaheadBaseMm,lookaheadPerMpsMm,kLateral_x100,kHeading_x100,lineAlpha_x1000,lineThetaGain_x1e9` の7項目、改行なし。既定 `1,080,040,3000,0600,010,0000`。旧6項目は有効値を保持し末尾 `0000` を追加。ヨー角補正は既定OFF。 |
| `heading_cal.txt` | `SDcard.c` `readHeadingCalibrationSettings` | `enabled,leftPulsePerM,rightPulsePerM,effectiveTreadCentiMm`、改行なし。欠落・破損時 `0,58092,57945,10602` で無効化して修復。正常な既存値は上書きせず、左右いずれかが `PULSE_METER` と2%超異なる場合はメモリ上で無効化。現行の経路方位には使わず診断用。 |
| `imu_temp.txt` | `SDcard.c` `readImuTempCompensation` / `IMU.c` | 符号付き整数1項目、改行なし。BMI088ジャイロZ温度係数 `[dps/°C] × 1000000`、範囲 `-100000..100000`。欠落・不正時は0で修復し補正OFF。 |
| `lsval.txt` | `lineSensor.c` `readLinesenval` / `writeLinesenval` | 10個の `lSensorMax`、続いて10個の `lSensorMin`、各 `%04u,`（末尾カンマあり）、改行なし。全センサー異常や破損時は走行禁止。 |
| `analysis.txt` / `lognum.txt` | `courseAnalysis.c` `getLogNumber` / `saveLogNumber`、`SDcard.c` `readSavedLogNumber` / `writeSavedLogNumber` | 5桁の番号。`analysis.txt` は解析元番号、`lognum.txt` は保存済み番号。後者の欠落・不正時はSD上の最大ログ番号の次、なければ1。 |

- `courseAnalysis.h` の `WRITE_BOOSTSPEED_LOG` 有効時は `courseAnalysis.c` が速度計画を `setting/boost_%05d.csv` に書く。無効時にもスリップ解析のSD読出しエラー診断で同名ファイルが作成され得る。常時生成される設定ファイルとして扱わない。
- 走行ログCSVのスキーマ・`analysisSourceLog` 等の出自・PC側PATH復元手順は `robotrace-log-analysis` を参照する。設定保存タイミングは `control.c` の走行開始処理を確認し、連続走行中の `autoStart>=2` ではパラメータ保存を繰り返さない。
