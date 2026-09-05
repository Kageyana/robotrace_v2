---
type: codex-failure
date: 2026-08-23
task: "robotrace_v2 ショートカット走行実装"
status: open
severity: medium
tags:
  - codex/failure
---

# exec/apply_patch helper の setup refresh エラー

## 要約

通常権限の `exec_command` と組み込み `apply_patch` が、単発実行でも sandbox helper の初期化に失敗した。昇格した逐次 `exec_command` は成功したが、helper 自体の根本原因は未解決。

## 発生した状況

- 実行環境: Codex Desktop / Windows PowerShell
- 当初は複数 Skill 読込の並列実行で発生
- その後、単発の通常 `exec_command` と組み込み `apply_patch` でも再発

## 結果・エラー

```
helper_unknown_error: setup refresh had errors
```

## 原因

sandbox helper の setup refresh が失敗している。並列実行だけが原因ではなく、詳細原因は未確定。

## 解決・回避策

- 読取・ビルドは、必要性を明示して昇格した逐次 `exec_command` で実行した。
- ファイル編集は一時コピー上で unified diff を作り、`git apply --check` 後に `git apply` した。
- 一時ファイルは対象パスを検証して削除した。

## 今後の予防策

同じエラーでは無意味な通常権限リトライを繰り返さず、権限昇格または検証可能な unified diff 経由へ切り替える。helper の修復後に status を resolved へ変更する。

## 関連

- 関連ノート: [[2026-08-22-shell-helper-setup-refresh-error]]