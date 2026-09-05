---
type: codex-failure
date: 2026-08-11
task: "emcStop=3/5の切り分け"
status: resolved
severity: medium
tags:
  - codex/failure
  - robotrace/emergency-stop
  - scope-control
---

# 不要になったSTOP_ENCODER_STOP診断APIを追加して後から削除した

## 要約

emcStop=3 の原因切り分け用として、左右エンコーダ値のラッチ、停止カウンタgetter/setter、停止診断ログ列を追加した。しかしユーザーが STOP_ENCODER_STOP の切り分け自体を不要と判断し、追加APIとログ列を削除した。

## 発生状況

- encCurrentN/L/R のラッチとgetter/setterを追加した。
- getEmcStopEncStopCount、getEmcStopLineBrightCount、getEmcStopLineUnbrightCount、getEmcStopOverSpeedCount を追加した。
- 診断目的が不要になった後も、一時的にコード・ログスキーマ・解析側へ広がった。

## 原因

単発のemcStop事象に対して、ユーザーが必要とする切り分け範囲を確認する前に、永続的なAPIとログ列まで実装した。

## 解決

STOP_ENCODER_STOP のラッチgetter/setterと停止カウンタgetter、関連ログ列を削除し、通常の停止条件を維持した。

## 再発防止

emcStop診断では、まず既存ログだけで判断可能か確認する。新しいAPIやログ列を追加する前に、診断を継続するか、単発事象として記録のみするかを確認する。追加する場合も、削除対象と終了条件を実装計画に明記する。

## 関連

- AGENTS.md: D:/robotrace/robotrace_v2/AGENTS.md

