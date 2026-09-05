---
type: codex-failure
date: 2026-09-06
task: "codex/shortcut-path の変更を main へマージする"
status: open
severity: medium
tags:
  - codex/failure
  - git
  - permission
---

# Git マージ時の ORIG_HEAD.lock 作成権限エラー

## 要約

`main` へのマージ時に、Git が `.git/ORIG_HEAD.lock` を作成できずマージに失敗した。

## 発生した状況

- タスク: `codex/shortcut-path` を `main` へマージする。
- 実行環境: Windows PowerShell、`D:\robotrace\robotrace_v2`。
- 前提条件: 作業ツリーはコミット済みで、未コミット変更はなかった。

## 何を試したか

1. `main` へ切り替えた。
2. `git merge codex/shortcut-path` を実行した。

## 結果・エラー

```text
fatal: update_ref failed for ref 'ORIG_HEAD': cannot lock ref 'ORIG_HEAD': Unable to create 'D:/robotrace/robotrace_v2/.git/ORIG_HEAD.lock': Permission denied
```

マージ後に `.git/ORIG_HEAD.lock` は残っていなかった。

## 原因

確認できた事実は、Git が `.git/ORIG_HEAD.lock` を作成できなかったことだけである。既存のロックファイル残留ではなく、Git の一時ロック作成時のファイル権限または実行環境の制約が原因である可能性がある。

## 解決・回避策

未確認。`.git` への書き込みが許可された環境または権限でマージを再実行する。

## 今後の予防策

- マージ前に作業ツリーがクリーンであることと、`.git` 内に対象ロックファイルが残っていないことを確認する。
- `ORIG_HEAD.lock` 作成の権限エラーが発生した場合は、ロックファイルを推測で削除せず、既存プロセスとファイル権限を確認してから、Git を書き込み可能な権限で再実行する。
- マージ成功後に `git status`、`git log --decorate`、必要に応じて `git merge-base --is-ancestor` で反映結果を確認する。

## 関連

- 関連ノート: `2026-08-19-git-index-lock-permission-denied.md`
- 参考リンク:
