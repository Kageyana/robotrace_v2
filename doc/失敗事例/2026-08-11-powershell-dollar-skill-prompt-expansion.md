---
type: codex-failure
date: 2026-08-11
task: "repo-scoped Skill の openai.yaml 作成"
status: resolved
severity: low
tags:
  - codex/failure
  - powershell
  - skills
---

# PowerShellの変数展開でSkill名のドル記号が欠落した

## 要約

repo-scoped Skill の `agents/openai.yaml` を作成した際、`default_prompt` に `$robotrace-log-analysis` などの Skill 呼び出し文字列を入れるつもりだったが、PowerShell のダブルクォート内で `$robotrace` が変数として展開され、`Use -log-analysis ...` のようにドル記号と接頭辞が欠落した。

## 発生した状況

- タスク: `.agents/skills/*/agents/openai.yaml` の作成
- 実行環境: Windows PowerShell
- 前提条件: Skill 呼び出し名に `$` を含む文字列を生成していた

## 何を試したか

1. PowerShell から `default_prompt` を含む YAML を生成した。
2. 生成後に `Get-Content` で内容を確認した。
3. `$robotrace-*` が `-*` に変化していることを発見した。
4. `apply_patch` で正しい `$robotrace-*` 表記へ修正した。

## 結果・エラー

```
default_prompt: "Use -log-analysis to analyze robotrace_v2 run logs."
```

期待値は以下だった。

```
default_prompt: "Use $robotrace-log-analysis to analyze robotrace_v2 run logs."
```

## 原因

PowerShell のダブルクォート文字列では `$name` が変数展開される。Skill 名としての `$robotrace-log-analysis` をリテラルとして扱うためのエスケープまたはシングルクォートが不足していた。

## 解決・回避策

生成済み YAML をパッチで修正し、`default_prompt` に `$robotrace-*` が残っていることを確認した。

## 今後の予防策

- PowerShell で `$` を含む設定ファイルを生成する場合は、シングルクォートまたはバッククォートで `$` をエスケープする。
- 生成後は `Get-Content` や `rg` で、意図したリテラルが残っているか必ず確認する。
- Skill メタデータの生成は、可能ならテンプレートやパッチで明示的に作成する。

## 関連

- 関連ノート:
- 参考リンク:
