---
type: codex-failure
date: 2026-10-15
task: Skill関連4コミットだけをmainへ統合する
status: resolved
severity: low
tags: [codex/failure, permission, worktree]
---

# workspace外の管理worktreeではGit読取も通常権限で拒否された

## 確認した結果・原因

create_worktreeが返したC:/Users/Natsuki/.codex/worktrees/skill-prevention-main/robotrace_v2で、通常権限のgit statusとrev-parseはcannot change to / Permission deniedとなった。元のD:/workspaceでのGit読取は成功した。対象を限定したrequire_escalatedでは同じworktreeの読取が成功した。OSのACLの詳細は未調査。

## 対処・再発防止

管理worktree作成後は返されたパスを現在のpermission profileと照合する。workspace外ならGit読取も初回から必要権限で実行する。作成ツールの成功やファイル読取成功を、そのディレクトリでのGitプロセス実行権限と同一視しない。共通事前確認へ反映した。

## 確認方法

必要権限付きのstatus/rev-parse、main切替、対象4件の取り込みが成功した。元workspaceの未コミット変更は別途SHA256で保全を確認する。対象パスでの実行手順はresolved。sandboxの内部原因を解明したという意味ではない。
