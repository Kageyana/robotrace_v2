---
type: codex-failure
date: 2026-09-20
task: 距離換算・スキーマ10実装の差分確認
status: resolved
severity: low
tags:
  - codex/failure
  - git
---

# CRLF作業ツリーでautocrlfを一時変更すると差分チェックが誤検出する

## 要約

`git -c core.autocrlf=false diff --check`が、無関係なCRLF行を大量に末尾空白として検出した。

## 発生した状況

- タスク: スキーマ10実装後の差分確認。
- 実行環境: Windows PowerShell、CRLFの作業ツリー。
- 前提条件: 通常設定の`git diff --check`は成功していた。

## 何を試したか

1. 改行警告を抑えるつもりで、チェック時だけ`core.autocrlf=false`を指定した。

## 結果・エラー

終了コード1で多数の`trailing whitespace`を表示した。変更対象外のファイルも含まれた。

## 原因

CRLFを含む作業ツリーで、Gitが期待する改行変換をこのコマンドだけ無効化したため。ファイル内容の実際の末尾空白増加ではない。

## 解決・回避策

設定を上書きしない`git diff --check`を再実行し、終了コード0を確認した。表示される改行変換警告は、この失敗の末尾空白判定とは別物として扱う。

## 今後の予防策

- Windowsの本リポジトリでは、差分チェックに`-c core.autocrlf=false`を付けない。
- 改行警告が出ても、まず通常設定の`git diff --check`の終了コードで判断する。

## 関連

- 確認コマンド: `git diff --check`
