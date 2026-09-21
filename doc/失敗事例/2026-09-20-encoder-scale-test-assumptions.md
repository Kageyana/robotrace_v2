---
type: codex-failure
date: 2026-09-20
task: 実測エンコーダ換算値の反映
status: resolved
severity: low
tags:
  - codex/failure
  - robotrace
  - tests
---

# 換算値変更時に合成テストの旧上限と校正状態が残った

## 要約

距離換算を55,116 pulse/mへ変更した際、距離推定テストのケース値に旧568 pulse上限が残り、失敗した。また方位推定テストで左右実測値を設定した後、後続ケースの左尺度を元に戻さず失敗した。

## 発生した状況

- タスク: 実測エンコーダ換算値の反映
- 実行環境: Windows PowerShell、Python 3.10、MinGW GCC
- 前提条件: 旧換算56,687 pulse/m、安全上限568 pulseのテストが存在した。

## 何を試したか

1. 距離上限を552 pulseに変更して一括合成テストを実行した。
2. 左右実測尺度を追加してCの方位推定テストを実行した。

## 結果・エラー

- `test_output_guard_checks_float_before_integer_conversion`は旧期待値`-568`と実際の`17`が不一致。
- 方位推定の外れ値テストで`estimator.rejected == 1U`が成立しなかった。

## 原因

距離テストは旧上限をケース内に直書きしていた。方位テストは前のケースで変更した左パルス/mを元に戻していなかった。

## 解決・回避策

距離テストを`MAX_FUSED_DELTA_P`連動にし、方位テストで左右の尺度を明示的に再設定した。一括Pythonテスト39件とCホストテストの成功を確認した。

## 今後の予防策

- `PULSE_METER`または安全上限を変更したら、`rg -n "568|56687|56,687" robotrace_v2/Core analysis/script`で旧値を検索し、履歴用以外の直書きを更新する。
- 複数ケースで可変校正構造体を共有するCテストでは、各ケース開始時に左右両方の尺度とトレッド幅を明示的に初期化する。
- `python -m unittest analysis.script.test_distance_estimator analysis.script.test_heading_route_schema9`とCホストテスト、Debug/Releaseビルドを実行する。

## 関連

- 関連テスト: `analysis/script/test_distance_estimator.py`, `analysis/script/test_heading_estimator.c`
