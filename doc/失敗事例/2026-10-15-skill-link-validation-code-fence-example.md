---
type: codex-failure
date: 2026-10-15
task: Skill更新後の参照ファイル確認
status: resolved
severity: low
tags: [codex/failure, skill, test]
---

# Skillのコードフェンス内の例示リンクを実参照と誤判定した

## 確認した結果・原因

PyYAML不足を事前確認し、代替の構造検査を実行した。リンク検査がskill-creator内の仮想文書Skill例にあるreferences/redlining.mdを、skill-creator自身の必須参照として検査し停止した。製品Skillの参照切れではなく、検証器がコードフェンス内の記述例を本文参照と区別していなかった。

## 対処・再発防止

リンク抽出前にコードフェンス内の例示を検査対象から外す。通常本文で追加した共通事前確認や分野別参照は引き続き存在・反映予定ファイルとの対応を検査する。skill-creatorのWindows更新手順にも同じ区別を追加する。

次の検査では既存のviewer AGENTS.mdの末尾空白も拒否した。依頼と無関係な既存文書を整形して回避せず、変更前コピーと比較して新規追加した末尾空白だけを拒否する検査へ修正した。

## 確認方法・ステータス

analysis/skill-hardening-20261015/validate.pyで、例示リンクが除外され、本文の実参照17箇所が検査対象に残ることを確認した。19ファイル・8Skillのステージ/反映後検査、142件の原本SHA256照合が成功したためresolved。標準quick_validate.pyを実行したという意味ではない。

Git差分検査も今回の変更対象へ限定する。全作業ツリーの検査には既存の別作業のCRLF差分が含まれていたため、改行の扱いを揃えてAGENTS.mdと.agents/skillsを検査した。既存の失敗記録を整形・上書きして解消しない。
