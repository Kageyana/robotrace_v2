---
type: codex-failure
date: 2026-10-03
task: IMUによる車輪成分分解のログ比較
status: resolved
severity: low
tags: [codex/failure]
---

# 連番ログをすべて正常終了として扱う前提

## 発生した状況と確認結果

成分分解の比較スクリプトで12866～12871・12885～12890を選択した際、全ログが正常終了している前提のassertが12889で停止した。12889は行数一致で読めるがemcStop=3、closureValid=0、distanceScaleVerified=0。停止原因3の実機要因はこの作業では調べていない。

## 原因と対処

連番で存在することを正常完走の根拠にしていた。emcStopを数値で評価し、異常終了は除外理由付きでvalidation.jsonへ保存してから、正常ログのみ校正・距離検証・周回数を確認するよう修正した。

## 再発防止と確認方法

比較スクリプトでは各CSVのemcStop・期待行数・モードを対象ごとに確認する。失敗走行は正常走行の集計へ入れず、番号・理由を出力する。`python analysis/script/check_wheel_projection.py`が成功し、正常11ログ、除外12889/emcStop=3をvalidation.jsonへ出力することを確認した。
