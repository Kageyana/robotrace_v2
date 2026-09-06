---
type: codex-failure
date: 2026-09-06
task: "12283〜12288のログ終端集計"
status: resolved
severity: low
tags:
  - codex/failure
  - robotrace/log-analysis
  - powershell
---

# PowerShellのforeach直後のパイプで構文エラー

## 概要

複数ログのマーカー終端を1行コマンドで集計した際、`foreach (...) { ... } | Format-Table` と記述して `An empty pipe element is not allowed` で停止した。CSV解析ファイルや集計結果への影響はない。

## 原因

PowerShellの文として`foreach`構文の出力を直接パイプへ接続する形が不正だった。

## 対処

`foreach`の各結果を配列変数へ追加し、ループ終了後に配列を`Format-Table`へ渡して集計を完了した。

## 再発防止

複数行のPowerShell集計では、`foreach`文の直後へパイプを置かず、結果を明示的な配列変数へ格納する。複雑なログ集計は再利用可能なPythonスクリプトへ実装し、PowerShellは成果物の表示に限定する。
