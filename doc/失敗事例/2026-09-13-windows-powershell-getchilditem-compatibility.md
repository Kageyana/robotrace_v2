---
type: codex-failure
date: 2026-09-13
task: "ログ12384～12394の対象ファイル列挙"
status: resolved
severity: low
tags:
  - codex/failure
  - powershell
  - log-analysis
---

# PowerShell互換差でGet-ChildItemのファイル列挙が停止した

## 要約

対象ログを数値範囲で列挙するときに`Get-ChildItem -File`を使用したところ、この実行環境では`-File`パラメータが認識されず停止した。互換的な`Where-Object { -not $_.PSIsContainer }`へ変更して列挙を完了した。

## 確認結果

- `Get-ChildItem -LiteralPath $logDir -File`は`A parameter cannot be found that matches parameter name 'File'`となった。
- `Get-ChildItem -LiteralPath $logDir | Where-Object { -not $_.PSIsContainer }`ではログ12384～12394を欠落なく列挙できた。
- 最初に参照したSkill既定の`F:`ドライブは存在せず、実ログは`C:\Users\ucawa\Dropbox\Document\robotrace\Log\v2`にあった。

## 原因

実行中のPowerShell互換レイヤーが、想定した`Get-ChildItem`の`-File`動的パラメータを提供していなかった。ログや対象範囲の問題ではない。

## 対処と再発防止

- PowerShellでのファイル列挙は`-File`へ依存せず、`PSIsContainer`でファイルを絞る。
- ログ番号はファイル名を整数化して上下限比較し、正規表現だけで範囲を表現しない。
- Skill既定のログドライブが無い場合は、Dropboxの現在のローカル同期先候補を`Test-Path -LiteralPath`で確認する。
- 互換列挙で対象11ファイルを確認できたため`resolved`とする。
