---
type: codex-failure
date: 2026-08-11
task: "lineTraceCtrlと直線指標の採用判定"
status: resolved
severity: low
tags:
  - codex/failure
  - robotrace/log-analysis
  - tuning
---

# lineTraceCtrl RMSの悪化を採用不可と早く扱い過ぎた

## 要約

調整比較で lineTraceCtrl RMS や直線 gyroVal_Z RMS の小さな悪化を、他の指標とのトレードオフやユーザーの許容範囲を確認せずに不採用理由として強く扱った。その後、ユーザーから現在のRMSは許容するとの基準が示された。

## 発生状況

- ライン制御候補の判定で、RMSの微小な増加を単独で悪化と解釈した。
- PWM飽和、lap、完走率、横スリップと同じ重みで扱う場面があった。

## 原因

採用条件を固定値として運用し、ユーザーが許容する指標と、絶対に悪化させたくない指標を事前に分離していなかった。

## 解決

現在は、ユーザーが許容したRMS範囲内では、完走率、lap、速度追従、分類済みのライン由来飽和などを優先して判断する。

## 再発防止

調整開始時に、指標ごとの優先順位と許容範囲を確認する。RMSを報告するときは数値、基準との差、他の安全・性能指標を併記し、単独で不採用としない。

## 関連

- AGENTS.md: D:/robotrace/robotrace_v2/AGENTS.md

