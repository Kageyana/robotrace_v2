---
type: codex-failure
date: 2026-09-20
task: 左右エンコーダによる一次経路方位推定
status: resolved
severity: medium
tags:
  - codex/failure
---

# エンコーダのpulse/mからmmへの換算を1000倍誤った

## 要約

新規方位推定の初回実装で、区間パルスをmmへ換算する係数を`1000000`としていた。正しくは`1000`。

## 発生した状況

- タスク: 左右エンコーダ差から角速度を計算し、ジャイロ残留バイアスを推定する。
- 実行環境: `headingEstimator.c`の実装レビュー時。実機書き込み前。
- 前提条件: 校正値の単位はpulse/m、計算する車輪移動量の単位はmm。

## 何を試したか

1. 式の単位を見直し、`pulse × 1000 / (pulse/m) = mm`を確認した。
2. 変換係数を修正し、ホスト用の合成テストを追加した。

## 結果・エラー

修正後、約1m/sの左右同数パルスを10ms間隔で3000回与えたテストで、0.24deg/sの残留バイアスを許容差内に推定した。高角速度区間の観測除外と不正時刻の拒否も確認した。

## 原因

pulse/mをpulse/mmへ変える係数と、mからmmへ変える係数を二重に適用した。

## 解決・回避策

`headingEstimator.c`の左右移動量を`pulse × 1000 / pulsePerMeter`へ修正した。

## 今後の予防策

- `gcc -std=c11 -Wall -Wextra -Werror -I robotrace_v2/Core/Inc analysis/script/test_heading_estimator.c robotrace_v2/Core/Src/headingEstimator.c -lm -o <一時実行ファイル>`で合成テストを実行する。
- 新たな距離換算式には入力・出力の単位をコメントに明示し、1m/sのパルス例で検算する。

## 関連

- 関連テスト: `analysis/script/test_heading_estimator.c`
