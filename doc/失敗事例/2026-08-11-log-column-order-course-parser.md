---
type: codex-failure
date: 2026-08-11
task: "ログ列追加後の1次・2次ログ解析"
status: resolved
severity: high
tags:
  - codex/failure
  - robotrace/log-analysis
  - schema
---

# ログを先頭側へ追加して固定列パーサを壊した

## 要約

ログに encCurrentL/R などの診断列を追加した際、既存の固定列番号を前提とする解析処理と整合しない位置へ列を挿入した。その結果、9960〜9974 では targetSpeed が全行 86 固定になるなど、実際の速度プロファイルと異なる解析結果になった。

## 発生状況

- 追加ログ列を encCurrentN 付近へ挿入した。
- courseAnalysis.c には先頭列の固定順を前提とする処理が残っていた。
- 列追加後のログを通常の速度評価対象として扱いかけた。

## 結果

targetSpeed、optimalIndex などが列ずれの影響を受け、該当ログは速度評価から除外した。

## 原因

ログスキーマの変更時に、固定列を使う読み手を洗い出さず、列の挿入位置だけを変更した。

## 解決

- 先頭6列を固定し、新規列は ROC より後ろへ配置した。
- 2次ログ解析はヘッダ名ベースで必須列を取得する方針へ変更した。

## 再発防止

ログ列追加前に、全解析側の読み取り方式を確認する。特に cntlog、encCurrentN、gyroVal_Z、courseMarker、encTotalOptimal、ROC の前には新規列を挿入しない。変更後の最初のログで targetSpeed が区間に応じて変化することを確認する。

## 関連

- AGENTS.md: D:/robotrace/robotrace_v2/AGENTS.md

