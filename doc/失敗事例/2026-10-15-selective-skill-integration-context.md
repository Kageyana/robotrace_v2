---
type: codex-failure
date: 2026-10-15
task: 先行の制御変更を含めずSkill関連4件だけをmainへ反映する
status: resolved
severity: low
tags: [codex/failure, git, skill, integration]
---

# Skillの選択取り込みで未導入機能の周辺説明が競合した

## 確認した結果・原因

83f0414のチューニングSkillには再発防止リンク追加のほか、SHADOW説明内のDWT文言修正があった。mainには先行5件の制御変更と当該説明節がなく、cherry-pickで競合した。相手側を丸ごと採用すると、指定対象外のSHADOW説明までmainへ入る状態だった。

## 対処・再発防止

main側の既存Skillと今回の再発防止リンクを維持し、未導入SHADOW説明の競合部分は取り込まなかった。DWT計測時に対応ELFを確認する一般手順は、同時追加する書込み・デバッグの再発防止参照に保持されている。競合解消後の余分な末尾空行はdiff --cached --checkで検出して除いた。

コミットの親とmainの差を事前確認し、指定コミットの差分と未導入の周辺節を分ける。共通事前確認へ反映した。対応表のmain未収録事例は調査元e9903caのGit参照へ変更し、先行コミットを丸ごと取り込む方法で参照切れを解消しない。

## 確認方法

対象4件の取り込みが完了し、main旧HEAD 85735f0からrobotrace_v2・Circuit・Machineに差分がないことを確認した。Skillの追加参照、競合マーカーなし、元作業ツリーの保持を最終検査する。選択取り込みの競合解消はresolved。先行制御変更を採用したという意味ではない。
