---
type: codex-failure
date: 2026-09-19
task: "一次走行閉路補正の回帰テスト"
status: resolved
severity: low
tags:
  - codex/failure
  - robotrace
  - tests
---

# ログヘッダテストのmodule実行でローカルimport失敗

## 要約

`test_log_header_formats.py`をリポジトリ直下から`python -m unittest analysis.script.test_log_header_formats`で実行すると、同じスクリプトディレクトリの`normalize_log_headers`を見つけられなかった。

## 発生した状況

- タスク: 一次走行閉路補正の回帰確認
- 実行環境: Windows PowerShell、Python 3.10
- 前提条件: 対象テストが`from normalize_log_headers import ...`を使用

## 何を試したか

1. 複数テストをリポジトリ直下から`python -m unittest analysis.script...`で実行。
2. 当該テストを`python analysis/script/test_log_header_formats.py`で直接実行。

## 結果・エラー

前者は`ModuleNotFoundError: No module named 'normalize_log_headers'`、後者は10件すべて成功した。2026-09-20にも一括module実行で同じエラーが再発し、`test_distance_estimator.py`の`robotrace_units`も同じ理由で失敗した。両ファイルを直接実行すると、それぞれ10件・15件成功した。同日、距離換算改修の一括テストで`test_log_header_formats.py`の同じエラーが再発した。既存の「直接実行する」という運用対策だけでは再発を防げなかった。

## 原因

module実行時には`analysis/script`がトップレベルのimport探索パスに入らず、直接実行時には入るため。

## 解決・回避策

当該既存テストは直接実行し、10件成功を確認した。再発後、`test_log_header_formats.py`と`test_distance_estimator.py`およびそれらが使う`repair_cntlog_wrap.py`のimportをpackage/単体の両方に対応させた。一括module実行39件と直接実行10件・15件でimportエラーがないことを確認した。テスト本体や製品コードの失敗ではない。

## 今後の予防策

一括テストと単体実行の両方を回帰コマンドに入れる。`python -m unittest analysis.script.test_log_header_formats analysis.script.test_distance_estimator`と`python analysis/script/test_log_header_formats.py`、`python analysis/script/test_distance_estimator.py`を実行し、importエラーがないことを確認する。

## 関連

- 参考リンク: `analysis/script/test_log_header_formats.py`
