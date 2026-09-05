---
type: codex-failure
date: 2026-08-11
task: "2次走行ログの再解析"
status: resolved
severity: high
tags:
  - codex/failure
  - robotrace/log-analysis
  - parser
---

# 2次ログを固定列番号で再解析して速度計画を誤読した

## 要約

ログ列追加後も readLogDistanceSlip() の2次ログ解析が固定列番号を使っていたため、targetSpeed、optimalIndex、slipFlag、slipFlagLat を誤読した。列ずれが解消された後も3走目以降が低速化したように見え、速度計画の原因調査を誤るリスクが生じた。

## 発生状況

- ログスキーマに診断列を追加した。
- 2次ログ解析側が固定位置を参照していた。
- 9990、9991、9995〜9997 などを速度計画の比較対象から除外した。

## 原因

ログの列順を制御コードと解析コードで共有する仕組みがなく、ヘッダを正として扱っていなかった。

## 解決

起動時にCSVヘッダから courseMarker、encTotalOptimal、ROC、targetSpeed、optimalIndex、slipFlag、slipFlagLat の列番号を取得し、必須列がない場合は既存の距離解析へフォールバックする方式へ変更した。

## 再発防止

2次ログ解析は固定列番号を追加しない。ログ列変更時の受入条件に、必須列のヘッダ検索、列欠落時のフォールバック、通常プロファイルでの targetSpeed 可変確認を含める。

## 関連

- AGENTS.md: D:/robotrace/robotrace_v2/AGENTS.md

