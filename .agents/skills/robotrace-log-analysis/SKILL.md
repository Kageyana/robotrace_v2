---
name: robotrace-log-analysis
description: 実機のCSVログを検証・可視化・比較し、一次経路元の妥当性やPATH経路の復元結果を判定するときに使う。
---

# 走行ログ解析

## 入力と有効性

- ログフォルダは環境依存。実在するパスを確認する。解析スクリプトは `analysis/script/`、出力は `analysis/` に置く。対象ログ番号と走行モード `optimalTrace` を記録し、一次・`BOOST_DISTANCE`・`BOOST_PATH_REPLAY`・`BOOST_SHORTCUT` を混ぜて比較しない。
- 現行列定義は `robotrace_v2/Core/Inc/log_schema.h`（schema 10: 軽量36 B、詳細121 B）。CSVはUTF-8で1行目が `key=value` メタデータ、2行目が列名、3行目以降がデータ。旧式の混在1行ヘッダ・メタデータなし形式も列名で解決する。ファームウェア行末のカンマは無名の空欄としてのみ無視し、名前のある列の不足や行数不一致は無視しない。
- まず `emcStop==0`、`cntlog` の連続性、必要列、バッテリー電圧を確認する。新形式は `logExpectedRows` とデータ行数/改行終端、`logOverflowFinal` も照合し、ヘッダのない旧ログに新形式の要件を遡及しない。PATH系は1サンプルの `optimalIndex` 前進が5点以上なら無効。Level 1評価は `shortcutBuildStatus=1`、回廊数・短縮量・合法余裕を確認する。失敗走行は原因分析に使ってよいが採用比較から除く。
- 軽量ログにはスリップ・モーターPWM・線形加速度・位置補正診断列がない。必要なら `robotrace_v2/CMakePresets.json` の `DebugMarker` または `ROBOTRACE_LOG_SCHEMA_PROFILE_LIGHT=0` で詳細ログを取得する。レコード長変更は制御負荷にも影響し得るので、通常ログと詳細ログを同条件とみなさない。

## スキーマ・経路の解釈

- schema 7/8 の `encCurrentL/R` は瞬間の1 msパルス、schema 9/10 の `encIntervalL_p/R_p` はジャイロ平均と同じログ区間の積算パルス。`encCurrentCorr_p` は融合後の符号付き1 ms差分。旧版と新版の値を同じ物理量として比較しない。
- schema 9 の `x/y` は診断用ジャイロXY、`x_fused_mm/y_fused_mm` が当時の経路XY。schema 10 は逆に `x/y` が経路用ジャイロXY、`x_fused_mm/y_fused_mm` は診断専用。現行ファームウェアの新規PATH元ログは schema 10 の正常一次走行のみで、`closureValid=1`、`distanceScaleVerified=1`、エンコーダ換算とIMU校正・読出し状態が合致する必要がある。旧ログは閲覧と履歴復元用に扱う。
- PATH系で軽量ログから省略された `linePointX_mm`、`linePointY_mm`、`pathLegalMargin_mm` は `analysis/script/path_log_recovery.py` で一次ログと設定からメモリ上に復元する。一次ログは二次ログと同じフォルダを探し、必要なら `--source-log-dir` を指定する。元CSVや `cntlog` 補修済みファイルを自動で上書き・代用しない。
- 復元時は `routeControllerVersion` に対応する経路を再生成し、`routePointCount`、`routeGeometryCrc32`、Level/生成結果のヘッダ値、有限な整数 `optimalIndex` とその範囲を照合する。元ログ欠落・CRC不一致・不正indexは0補完せず欠測とする。保存値優先、復元状態は `saved` / `restored` / `partial` / `missing` / `not_applicable` と理由・元ログ番号・欠測行数を出す。欠測行があれば走行全体の最小合法余裕は判定不能。
- `pathErrorHeading_cdeg` は経路制御版3以前は先読み方位との差、版4以降は最近傍接線との差。異なる定義のp95は直接比較しない。`imuTempRaw=0x400` は無効で、他の生値は11ビット符号付きとして `signed_code * 0.125 + 23` [°C] に変換する。

## 比較・出力

1. 対象モードで揃え、バッテリー・速度設定・ファームウェア/経路版・校正状態を照合する。autoStart比較は5走すべて揃った系列を用いる。電圧差が大きければ充電後に取り直す。
2. XY、ラップタイム、目標/実速度、角速度、PATH時の横・方位誤差とフォールバック、存在する場合だけスリップを図表化する。欠測の経路区間は線でつながず理由を注記する。
3. `analysis/script/analyze_path_following.py` はPATHモード専用。`--source-log-dir`、`--require-autostart-five`、診断用 `--allow-invalid` を必要に応じて使う。通常の採用比較で `--allow-invalid` を使わない。結果には比較対象・除外理由・変更条件・採否と残課題を残す。
