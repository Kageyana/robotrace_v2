---
type: codex-failure
date: 2026-10-14
task: "旋回進入・減速段階別の残差集計"
status: resolved
severity: low
tags: [codex/failure, analysis]
---

# 空の走行方式を平均して補助表へNaNを出力した

## 要約・結果

`analyze_slip_transition_timing_13228_13247.py`の初回実行で`Mean of empty slice`と`invalid value encountered in scalar divide`が発生した。両方式を持つ主集計には有効値があったが、ログ別補助表は片方式だけの入力に対して両方式の旋回平均を計算し、存在しない方式のNaN行を生成していた。ログ内容や検出器の異常ではない。

## 原因

集計ヘルパーを全ログ・ログ別で共用した際、旋回側の空層を除外する条件が欠けていた。また検証が主集計中心で、補助表の非有限値まで検査していなかった。

## 対処・再発防止

データのない方式の集計行は作らず、主集計・ログ別・境界感度・車輪速度基準の全統計を有限値検査へ含めた。検証JSONも`allow_nan=False`で保存する。空層を0平均や正常値へ置換しない。実データがない場合は行を省略し、対象区間数を保存する。

## 確認

修正後の同一入力再実行でRuntimeWarningなし。各方式4本の走行別集計、指令境界40/60/80msと車輪平滑化31/41/61msの感度集計が有限であることを確認した。原本12本と再利用した一次基準4計画のSHA256は不変。コード上の空層ガードと全統計検査を検証できたためresolvedとする。

## 関連

- `analysis/slip_transition_timing_13228_13247/report.md`
- `analysis/script/analyze_slip_transition_timing_13228_13247.py`
