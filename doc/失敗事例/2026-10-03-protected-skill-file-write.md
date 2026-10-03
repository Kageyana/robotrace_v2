---
type: codex-failure
date: 2026-10-03
task: コースプロット用最小ログへの変更
status: resolved
---

# 保護されたSkillファイルへの直接書き込みが拒否された

## 確認した結果

PythonのPath.write_textで`.agents/skills/robotrace-log-analysis/SKILL.md`を更新するとPermissionErrorになった。先行するAGENTS.mdの更新は完了しており、複数ファイルの処理全体が取り消されたわけではない。

## 原因と対処

現在の権限設定では`.agents`は読み取り対象で、シェルからの直接書き込みが拒否される。許可されたapply_patchで同じ文書を更新できた。最小13列・22バイトの仕様がSkillへ反映された。

## 再発防止と確認方法

保護されたSkillの編集はapply_patchで行う。複数ファイルの更新中に失敗した場合は、git diffで成功済みファイルを確認してから残りだけを更新する。ログスキーマ変更時は両プロファイルのrun_log_schema_tests.ps1と通常・詳細のCMakeビルドを実行する。今回はいずれも成功した。
