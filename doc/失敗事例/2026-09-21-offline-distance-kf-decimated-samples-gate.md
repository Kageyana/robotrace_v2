---
type: codex-failure
date: 2026-09-21
task: "加速度ログによる距離・方位のオフラインカルマン検証"
status: resolved
severity: low
tags:
  - codex/failure
  - robotrace
  - distance-kalman
  - log-decimation
---

# 10ms間隔のオフライン距離KFで観測棄却を例外として扱った

## 要約

実機は1msごとに距離KFを更新するが、Schema 6のCSVは約10msごとの瞬時値しかない。
初回のオフライン試作では4σゲート超過を例外として処理し、検証が停止した。

## 発生した状況

- タスク: 12439・12441・12444の加速度列とエンコーダ速度から、区間対応の距離KFを再計算する。
- 実行環境: `analysis/script/analyze_offline_distance_pose_kalman_schema6.py`。
- 前提条件: CSV間隔は9～12ms、実機の距離KF周期は1ms。CSVの速度と加速度は区間積算ではなく瞬時値。

## 何を試したか

1. 実機と同じ状態`[距離,速度,加速度バイアス]`で、ログ間隔を`dt`にして予測・速度観測更新した。
2. 4σ超過で例外を出した。

## 結果・エラー

```text
ValueError: 10ms KF innovation rejected: 0.3626 m/s
```

## 原因

棄却はフィルタの定義済み動作なのに、初回試作で例外として扱ったことが直接原因。
約10msへ間引かれた瞬時値を使うため、棄却数と推定値を実機1msフィルタと同一視できない。

## 解決・回避策

- 4σ超過時は予測状態を採用して共分散を再初期化し、棄却数を結果へ記録するよう変更した。
- 3本のログでスクリプトが完走し、オフライン棄却数は0・0・1だった。
- 推定結果は「10msオフライン近似」と明記し、実機1ms融合を再現したとは報告しない。

## 今後の予防策

- 間引きログによるKF再生前に、記録周期、各列が瞬時値か区間値か、ゲート超過時の挙動を確認する。
- 回帰確認: `python analysis/script/analyze_offline_distance_pose_kalman_schema6.py`を実行し、各行に`offline_kf_rejects`が出ることを確認する。
- 1ms実機KFとの厳密な比較には、区間に同期した加速度・エンコーダ情報を別途記録する。

## 関連

- `analysis/script/analyze_offline_distance_pose_kalman_schema6.py`
- `robotrace_v2/Core/Src/distanceEstimator.c`
