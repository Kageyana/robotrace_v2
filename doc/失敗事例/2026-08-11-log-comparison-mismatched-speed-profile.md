---
type: codex-failure
date: 2026-08-11
task: "robotrace_v2 走行ログ比較"
status: resolved
severity: medium
tags:
  - codex/failure
  - robotrace/log-analysis
---

# ログ比較で速度表条件差を見落としかけた

## 概要
走行ログ `10992`〜`11006` の確認時、最初に `10968`〜`10991` と同じ 4.0m/s 系として比較できると説明しかけたが、後続確認で `10968` は `tgtParam.bstStraight=3.30`、最新ログは `4.00` だと判明した。

## 発生した状況
- タスク: robotrace_v2 の最新ログ確認
- 実行環境: Codex / PowerShell / ログ `F:\Dropbox\Document\robotrace\Log\v2`
- 前提条件: `CURVE_FF_GAIN=100% / FB_SCALE=35%` 採用後のログ比較

## 何を試したか
1. 最新ログ `10992`〜`11006` を抽出した。
2. 既存の profile 7 用スクリプトで比較した。
3. ラップ改善を見た後、ヘッダの速度表を再確認した。

## 結果・エラー

```
curveFF_ref_10968_10991: tgtParam.bstStraight=3.30
latest_10992_11006:      tgtParam.bstStraight=4.00
```

このため、ラップや高速度帯 target の差は制御式だけではなく速度表変更の影響を含む。最初の説明はすぐ訂正したが、同条件比較として扱うリスクがあった。

## 原因
ログの有効性、`emcStop`、電圧、ラップ、制御設定を先に見て、速度表パラメータ `tgtParam.*` の差分確認が後回しになった。`lineOmegaCurveFfEnable` など目的の制御ヘッダだけを見て、速度プロファイル条件の固定確認が不足していた。

## 解決・回避策
比較表へ `tgtParam.bstStraight`, `bst200`, `bst100`, `lineomega`, `speed.txt`, `speed_ff` 相当の主要条件を必ず出す。条件が違う場合は、ラップ改善を採用判定に使わず「別速度表ログ」として扱う。

## 今後の予防策
ログ比較の最初に以下を必ず確認する。

- `optimalTrace`, `emcStop`, `batteryVoltage_V`
- `tgtParam.bstStraight`, `bst200`, `bst100`, `acceleF/D`, `decelLeadMm`
- `lineTraceOmegaFBCtrl`, `speedFeedForwardGain`, `veloCtrl/L/R`
- ログプロファイルと必須列

速度表が違う場合は、同条件比較ではなく「速度表変更込みの結果」と明記する。

## 関連

- 解析出力: `D:\robotrace\robotrace_v2\analysis\log_10992_11006_latest_check_metrics.csv`
- 解析出力: `D:\robotrace\robotrace_v2\analysis\log_10992_11006_latest_check_summary.csv`
