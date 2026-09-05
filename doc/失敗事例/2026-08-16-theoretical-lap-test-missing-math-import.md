---
type: codex-failure
date: 2026-08-16
task: "理論ラップ計算へ加減速モデルを追加"
status: resolved
severity: low
tags:
  - codex/failure
  - robotrace
  - test
---

# 加減速モデルの単体テストでmath importが不足した

## 要約

加速途中の区間時間を検証する新規テストで`math.sqrt()`を使用したが、テストモジュールへ`math`をimportしておらず、初回テスト実行が`NameError`で失敗した。

## 発生した状況

- タスク: 2次走行の理論ラップへ目標速度変更時の加減速時間を反映
- 実行環境: Windows PowerShell / Python unittest
- 前提条件: `analysis/script/test_theoretical_lap.py`へ加速遷移テストを追加

## 何を試したか

1. Python構文確認後に`python -m unittest analysis.script.test_theoretical_lap`を実行した。

## 結果・エラー

```
NameError: name 'math' is not defined
Ran 11 tests ... FAILED (errors=1)
```

## 原因

新規テストの期待値計算で`math.sqrt()`を追加した際、import一覧の更新を漏らした。

## 解決・回避策

`test_theoretical_lap.py`へ`import math`を追加し、11件すべての単体テストが成功することを確認した。

## 今後の予防策

- テスト追加後は対象テストファイル単体の構文確認だけでなく、必ずunittestを実行する。
- 新しい標準ライブラリ参照を追加した場合はimport一覧も差分確認する。

## 関連

- `analysis/script/test_theoretical_lap.py`
