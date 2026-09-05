---
type: codex-failure
date: 2026-08-19
task: "mainへの採用済み制御・ログ解析修正のコミット"
status: open
severity: low
tags:
  - codex/failure
  - git
---

# Git index.lock作成権限エラー

## 要約

変更のステージングとコミット時に、Git管理領域へ`index.lock`を作成できず処理が停止した。

## 発生した状況

- タスク: mainへ曲率FF、7.1V、公称値、2次ログのヘッダ名パーサーを反映してコミット
- 実行環境: `D:\robotrace\robotrace_v2`、PowerShell
- 前提条件: 作業ツリーには対象6ファイルの変更があり、既存の`.git/index.lock`は確認できなかった

## 何を試したか

1. 対象6ファイルを`git add`し、続けて日本語コミットメッセージで`git commit`を実行した。

## 結果・エラー

```text
fatal: Unable to create 'D:/robotrace/robotrace_v2/.git/index.lock': Permission denied
```

## 原因

`.git`配下への書き込み権限、またはGit管理領域をロックしている外部プロセスの有無は未確定。既存の`index.lock`ファイルは見つからなかった。

## 解決・回避策

未解決。作業ツリーの変更は保持している。`.git`のACLとGitクライアント・エクスプローラー等によるロックを確認してから、対象ファイルだけを再ステージ・コミットする。

## 今後の予防策

コミット前に`.git/index.lock`の有無とGit管理領域への書き込み可否を確認し、失敗時は同じコマンドを繰り返す前にロック元・ACLを切り分ける。

## 関連

- 関連ノート:
- 参考リンク:
