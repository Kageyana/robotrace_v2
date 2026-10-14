---
type: codex-failure
date: 2026-10-14
task: "追加実走ログ13238～13247の探索"
status: resolved
severity: low
tags:
  - codex/failure
---

# 同居する温度校正CSVを走行番号へ変換した

## 要約

走行ログフォルダの全CSV名を数値と仮定し、PowerShellの数値ソートでエラーとなった。ログ内容の異常ではない。

## 発生した状況

- タスク: 前回13237以降の追加走行ログを探す。
- 実行環境: Windows PowerShell、`F:/Dropbox/Document/robotrace/Log/v2`。
- 前提条件: 同じフォルダに`13238.csv`等と`imu_temp_00001.csv`等が存在する。

## 何を試したか

1. `Get-ChildItem -Filter *.csv`の全結果を`[int]$_.BaseName`でソートした。

## 結果・エラー

```text
Cannot convert value "imu_temp_00001" to type "System.Int32".
```

## 原因

拡張子CSVが走行ログを意味するとは限らない。型変換の前に対象ファイルを選別していなかった。

## 解決・回避策

PythonでstemがASCIIかつdecimalであるCSVだけを列挙して数値ソートし、追加13238～13247の10本を同定した。解析スクリプトに`discover()`として保存し、実フォルダ上の実行と追加番号集合の照合が成功した。全20本の解析・原本SHA256不変を確認済み。

## 今後の予防策

走行番号の一覧は`analysis/script/check_additional_logs_13238_13247.py`の`discover()`を使用し、全CSVを直接整数変換しない。スキーマ・列名・行数は別途検証し、温度校正やバイナリを走行CSVとして読み込まない。

## 関連

- 検証結果: `analysis/additional_logs_13238_13247/validation.json`
- 同時に修正した検索glob: `2026-08-11-windows-rg-glob-not-expanded.md`
