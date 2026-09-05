---
type: codex-failure
date: 2026-08-11
task: "Skill frontmatter の確認"
status: resolved
severity: low
tags:
  - codex/failure
  - windows
  - shell
---

# PowerShellでUnix風globを前提にしたrg確認が失敗した

## 要約

Skill frontmatter を確認するために `rg -n "^name:|^description:" .agents/skills/*/SKILL.md` のような Unix シェル前提の glob 指定を使い、Windows PowerShell では期待通り展開されず確認コマンドが失敗した。後続で検索対象をディレクトリにする方法へ切り替えて確認した。

## 発生した状況

- タスク: `.agents/skills/*/SKILL.md` の frontmatter 検証
- 実行環境: Windows PowerShell
- 前提条件: Unix 系シェルのように `*` がファイルパスへ展開される想定だった

## 何を試したか

1. `rg` に `.agents/skills/*/SKILL.md` を渡して frontmatter を検索した。
2. パス glob が期待通り処理されず、確認に失敗した。
3. `rg ... .agents/skills` のようにディレクトリを対象にする形へ変更した。

## 結果・エラー

具体的なログは保存していないが、Windows のパス glob 前提違いにより確認コマンドを再実行する必要があった。

## 原因

PowerShell と Unix シェルでワイルドカード展開の挙動が異なる。特に外部コマンドへ渡すパス glob は、期待通りに展開されない場合がある。

## 解決・回避策

`rg` には glob 付きファイルパスではなく、検索ルートディレクトリ `.agents/skills` を渡して確認した。必要な場合は `Get-ChildItem -Recurse -Filter SKILL.md` で対象ファイルを列挙する。

## 今後の予防策

- Windows PowerShell では Unix 風のパス glob を前提にしない。
- `rg` は検索対象ディレクトリを渡し、必要なら `-g` オプションを使う。
- ファイル一覧を厳密に作る場合は `Get-ChildItem` を使う。

## 関連

- 関連ノート:
- 参考リンク:
