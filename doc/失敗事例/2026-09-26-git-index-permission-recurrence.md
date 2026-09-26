---
type: codex-failure
date: 2026-09-26
task: "PATH追従機能のmain移植"
status: resolved
severity: low
tags:
  - codex/failure
  - git
  - sandbox
---

# Git index 権限拒否の再発

## 要約

通常権限の `git restore` が `.git/index.lock` の作成権限拒否で失敗した。対象を指定した同じ操作を `require_escalated` で再実行すると成功した。

## 発生した状況

- 作業: PATH追従モジュールを参照ブランチから作業ツリーへ移植。
- 実行環境: `D:\robotrace\robotrace_v2`、workspace-write。
- `.git/index` は存在し、`.git/index.lock` は残っていなかった。

## 結果

通常権限では `fatal: Unable to create '.../.git/index.lock': Permission denied`。`git restore --source=codex/imu-distance-kalman --worktree -- robotrace_v2/Core/Src/pathFollower.c robotrace_v2/Core/Inc/pathFollower.h` を権限昇格付きで再実行し、両ファイルの変更を確認した。

## 原因

既存の `2026-08-11-git-index-lock-sandbox-permission.md` と同じく、サンドボックスで `.git` への書き込みが制限されていた。今回は既存の対策を最初の index 更新時に適用しなかった。

## 再発防止策

`.git/index` を変更する `git add`、`git restore`、`git commit` 等は、初回から `sandbox_permissions: require_escalated` を使う。コマンドは対象パスを限定し、昇格後に `git status` で結果を確認する。

## 確認方法

昇格付き `git restore` の後、`git status --short` に指定した2ファイルだけが変更として表示された。

## 関連

- `doc/失敗事例/2026-08-11-git-index-lock-sandbox-permission.md`
