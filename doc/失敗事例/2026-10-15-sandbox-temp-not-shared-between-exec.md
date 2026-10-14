---
type: codex-failure
date: 2026-10-15
task: 全失敗事例を確認してSkillの再発防止手順を更新する
status: resolved
severity: low
tags: [codex/failure, sandbox, path, skill]
---

# exec間でsandboxのTEMPを共有できると仮定した

## 確認した結果

最初のexecが表示したTEMPと、ステージファイルを生成したexecのTEMPは異なるsandboxディレクトリだった。後者の表示した絶対パスを次のexecのTest-Pathで確認するとFalseだった。ステージを後続呼出しから利用できるという前提が成立しなかった。Skill本体への変更はまだ行っていなかった。

## 原因と未確定事項

呼出し間でsandboxのTEMPが同じ可視領域にあると仮定したことが手順上の問題。各sandboxのライフサイクルや削除時点は未調査であり、ファイル消失の内部原因は断定しない。

## 対処・再発防止

複数execで共有するステージを、書込可能な固定workspace内のanalysis/skill-hardening-20261015へ作成した。次の独立execでpayload.jsonを読み、変更前コピー・変更後19ファイル・ハッシュ付きmanifestを生成できた。

skill-creatorのWindows更新手順とロボトレース共通事前確認へ、共有ステージをworkspace内へ保存し、次段で存在・SHA256を確認する手順を追加する。単一呼出し内で完結する一時ファイルにまで固定workspaceを要求しない。

## 確認方法・ステータス

prepare.pyの別execでの終了コード0、staged_files=19、manifest.jsonと各変更前/変更後ファイルの読戻しで回避手順を確認した。実行間の共有手順はresolved。sandboxの内部ライフサイクルを解明したという意味ではない。

## 関連

- [共通事前確認](../skill-execution-preflight.md)
- C:/Users/Natsuki/.codex/skills/.system/skill-creator/references/windows-skill-maintenance.md
