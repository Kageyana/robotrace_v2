---
type: codex-failure
date: 2026-08-11
task: "Git staging, commit, restore"
status: resolved
severity: low
tags:
  - codex/failure
  - git
  - sandbox
---

# サンドボックス内でGit index更新が権限エラーになった

## 要約

`git add`、`git commit`、`git restore` など `.git/index` を更新する操作で、サンドボックスの権限制限により `.git/index.lock` を作成できず失敗した。権限昇格付きで再実行して解決した。

## 発生した状況

- タスク: AGENTS.md と Skill のコミット、後続の変更戻し
- 実行環境: `D:\robotrace\robotrace_v2`, workspace-write sandbox
- 前提条件: 作業ツリーは書き込み可能だが、`.git` は読み取り扱いのため index 更新には昇格が必要だった

## 何を試したか

1. 通常権限で `git add AGENTS.md .agents` を実行した。
2. 通常権限で `git commit -m ...` を実行した。
3. 変更戻しで通常権限の `git restore -- ...` も実行した。
4. 失敗後、`sandbox_permissions: require_escalated` 付きで再実行した。

## 結果・エラー

```
fatal: Unable to create 'D:/robotrace/robotrace_v2/.git/index.lock': Permission denied
```

## 原因

この環境では `.git` への書き込みが通常サンドボックス権限では許可されていない。作業ファイルの編集と Git index/history の更新は別権限として扱われる。

## 解決・回避策

Git の index や履歴を書き換える操作は、権限昇格付きで再実行した。`git add` と `git commit` は承認済み prefix になった。

## 今後の予防策

- `git add`、`git commit`、`git restore`、`git switch` など `.git` を更新する操作は、必要に応じて最初から昇格付きで実行する。
- 失敗した場合は別手段で回避せず、同じコマンドを昇格付きで再実行する。
- 戻し操作では対象ファイルを明示し、ユーザーの未関連変更を巻き込まない。

## 関連

- 関連ノート:
- 参考リンク:
