---
type: codex-failure
date: 2026-08-11
task: "ログレコードサイズの走行影響調査"
status: resolved
severity: high
tags:
  - codex/failure
  - robotrace/logging
  - timing
---

# overflowが無くてもログレコードサイズが走行へ影響した

## 要約

64Bログでは dbgOverflowFinal=0、logOverflowFinal=0、cntlog の大きなギャップなしでも、33B基準に対してlap、直線ジャイロ、lineTraceCtrl が悪化した。48B、40Bでも悪化要素が残り、36Bを検証ログ上限候補とした。

## 発生状況

ログ負荷試験で33B、64B、48B、40B、36Bを比較した。overflowだけを見て安定と判断できる状態ではなかった。

## 原因または仮説

バッファoverflowではなく、ログ生成、メモリコピー、バッファ処理、SD書き込み要求などが1ms制御周期の余裕を消費した可能性がある。制御周期の直接計測までは未実施であり、因果は仮説として扱う。

## 解決

- 36B以内に必要列を収める運用へ戻した。
- 負荷試験用の40B以上のダミー列を削除した。
- 詳細診断列は同サイズの列入れ替えで一時使用する方針にした。

## 再発防止

ログ負荷の評価ではoverflow、cntlog gapだけでなくlap、直線指標、PWM分類を同時に比較する。通常検証ログのサイズ上限を明示し、必要な診断列は既存列と入れ替えてレコードサイズを増やさない。

## 関連

- AGENTS.md: D:/robotrace/robotrace_v2/AGENTS.md

