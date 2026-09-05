---
type: codex-failure
date: 2026-08-16
task: "CA_SLIP_EXPAND_1=0.40 の速度計画比較"
status: resolved
severity: medium
tags:
  - codex/failure
  - robotrace/tuning
  - slip-planning
---

# CA_SLIP_EXPAND_1=0.40で周回別改善が揃わなかった

## 要約
スリップリスクの隣接indexへの伝播を弱めるためCA_SLIP_EXPAND_1を0.50から0.40へ下げたが、周回番号別の速度誤差p95、lap、横スリップの改善が一貫しなかった。

## 発生した状況

- タスク: スリップ反映の隣接区間伝播の切り分け
- 実行環境: robotrace_v2、速度PID=12/0/0、speed_ff=150、CA_SLIP_FRAC_FULL=0.70
- 前提条件: 11991〜12005の3完走セット、開始電圧7.83〜7.97V

## 何を試したか

1. CA_SLIP_EXPAND_1=0.40で3セット取得した。
2. 基準CA_SLIP_EXPAND_1=0.50のログとautoStart=2〜5で比較した。

## 結果・エラー

- 2周目は速度誤差p95、lap、横スリップが悪化した。
- 3周目と5周目は一部改善したが、4周目は速度誤差p95と横スリップが悪化した。
- 改善方向が周回番号間で揃わず、採用条件を満たさなかった。

## 原因

隣接indexへのリスク伝播を弱めても、autoStartの前走スリップを反映した速度計画全体の変動を安定して改善できなかった。単一の近傍伝播係数だけでは効果を切り分けられなかった。

## 解決・回避策

CA_SLIP_EXPAND_1を基準値0.50へ戻した。復帰はコード確認のみで完了とした。

## 今後の予防策

autoStartの周回番号別評価を維持し、周回を混ぜた平均だけで採用しない。次の変更はCA_SLIP_EXPAND_2など別の伝播範囲を1項目だけ比較する。

## 関連

- 解析: D:/robotrace/robotrace_v2/analysis/log_11991_12005_slip_expand040_comparison.csv
