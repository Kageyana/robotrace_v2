---
type: codex-failure
date: 2026-09-19
task: "一次走行の周期診断・偽ゴール対策の回帰テスト"
status: resolved
severity: low
tags:
  - codex/failure
  - robotrace
  - tests
---

# 閉路補正テストの直接実行でパッケージimportに失敗

## 要約

`test_primary_closed_xy.py`をファイル指定で起動すると、リポジトリ直下の`analysis`パッケージを見つけられない。module形式での実行に切り替えて5件通過した。

## 発生した状況

- タスク: 一次走行の周期診断・偽ゴール対策の回帰確認
- 実行環境: Windows PowerShell、Python 3.10、リポジトリ直下
- 前提条件: テストが`from analysis.script.path_log_recovery import ...`を使用

## 何を試したか

1. `python analysis/script/test_primary_closed_xy.py`
2. `python -m unittest analysis.script.test_primary_closed_xy`

## 結果・エラー

直接実行は`ModuleNotFoundError: No module named 'analysis'`。module実行は5件すべて成功。

## 原因

直接実行では`analysis/script`がimport探索パスの起点となり、リポジトリ直下が含まれないため。製品コードの失敗ではない。

## 解決・回避策

当該テストはリポジトリ直下からmodule形式で実行する。

## 今後の予防策

一括テスト前に各テストの先頭importを確認する。`analysis.script.*`を使用する`test_primary_closed_xy.py`は`python -m unittest analysis.script.test_primary_closed_xy`、ローカルimportの`test_log_header_formats.py`は直接実行する。

## 関連

- 関連ノート: `doc/失敗事例/2026-09-19-python-log-test-import-path.md`
- 参考リンク: `analysis/script/test_primary_closed_xy.py`
