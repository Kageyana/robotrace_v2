---
type: codex-failure
date: 2026-09-21
task: "BMI088温度補正ログ解析スクリプトの構文確認"
status: resolved
severity: low
tags:
  - codex/failure
  - test
  - python
  - windows
---

# 通常のPythonランチャーに実体がなく構文確認を開始できなかった

## 要約

温度補正対応後のPATH復元スクリプトを`py -3 -m py_compile`で確認しようとしたが、WindowsのPythonランチャーに実行可能なPython実体がなく失敗した。

## 発生状況

- 対象: `analysis/script/path_log_recovery.py`
- 実行環境: Windows PowerShell / Codexデスクトップ
- 結果: `py`は存在したが、インストール済みPythonがない旨を表示した。

## 原因

通常のPythonランチャーから利用できるPythonランタイムが環境に登録されていなかった。

## 対処

Workspace dependenciesで提示された同梱Pythonの絶対パスを使用し、`-m py_compile`を再実行して終了コード0を確認した。

## 再発防止策

WindowsでPython確認を始める前に、`py`または`python`の実体を確認する。実体がない場合はWorkspace dependenciesのPython絶対パスを使い、構文確認の終了コード0を確認する。

## 確認方法

同梱Pythonで`analysis/script/path_log_recovery.py`の構文確認に成功した。
