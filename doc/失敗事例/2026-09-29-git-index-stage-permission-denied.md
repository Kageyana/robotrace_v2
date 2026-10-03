---
type: codex-failure
date: 2026-09-29
task: "SDログ変更を段階的にコミット"
status: resolved
severity: low
tags:
  - codex/failure
  - git
  - sandbox
---

# Gitステージングでindex.lockの権限拒否

## 要約

通常権限の `git add` が `.git/index.lock` の作成権限拒否で失敗した。既存の再発防止策である `require_escalated` を最初のステージングでは適用していなかった。

## 発生した状況

- タスク: SDログ、Debug空転ベンチ、Releaseログ設定の変更を段階的にコミット
- 実行環境: `D:\robotrace\robotrace_v2`、PowerShell、workspace-write
- 前提条件: 作業ツリーに対象変更があり、`.git/index.lock` は存在しなかった

## 何を試したか

1. Releaseプロファイル変更以外を対象に `git add -A` を実行した。

## 結果・エラー

```text
fatal: Unable to create 'D:/robotrace/robotrace_v2/.git/index.lock': Permission denied
```

この失敗ではファイルはステージされず、作業ツリーの変更は保持された。

## 原因

既存の `2026-09-26-git-index-permission-recurrence.md` に記録された `.git` 書込み制限と同じ状況と考えられる。既存ノートの対策はGit更新時に `require_escalated` を使うことだが、今回の初回コマンドでは適用していなかった。

## 解決・回避策

`git add -A -- . ':(exclude)robotrace_v2/CMakePresets.json'` を `require_escalated` で再実行し、ステージが成功した。`git status --short` と `git diff --cached --stat` で対象変更を確認した。ステージ確認中に新規の `.pyc` が含まれたため、ステージを解除し、生成ファイルを削除して `.gitignore` に `__pycache__/` を追加した。

## 今後の予防策

- このリポジトリでは `git add`、`git restore`、`git commit` など `.git` を更新する操作に、初回から `require_escalated` を使う。
- ステージ後に `git status --short` と `git diff --cached --stat` を確認する。

## 関連

- `doc/失敗事例/2026-09-26-git-index-permission-recurrence.md`

## 2026-10-03 再発

IMU yI検証結果の登録時も通常権限のgit addがindex.lock作成拒否で失敗した。既存対策を初回に適用できていなかった。require_escalatedで対象結果ファイルのみを登録し成功した。今後はpermission_profileの.gitへのread指定をGit更新前に確認し、readのみなら初回から昇格する。ステージ結果で登録成功を確認済み。statusはresolved。
