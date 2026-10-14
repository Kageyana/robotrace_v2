---
type: codex-failure
date: 2026-10-14
task: "SLIP計画の先行減速・リスク除去による反実仮想"
status: resolved
severity: low
tags: [codex/failure, log-analysis, slip-planning]
---

# 増速直後の1.05倍を最終SLIP計画にも要求した

解析初版はリスクなし・SLIP段の先行減速なしなら最終計画が元計画の厳密な1.05倍になると仮定し、13212の実C出力でassertが失敗した。readLogDistanceSlip()は増速後にも元計画の区間速度差を上限として前後走査するため、加減速部の増速幅が制限される。実装異常ではなく、途中段階と最終出力を混同した解析上の仮定だった。

直接リスクと拡張リスクが全て0、最終計画が元計画以上・1.05倍以下であることを確認する検証へ変更した。3組×4条件を毎回一次計画から再生成し、保存パラメータ・実際の指令と照合、原本SHA256不変を確認して成功した。

次回のSLIP反実仮想では一次計画、リスク補正、先行減速、前後速度差制限、整数指令の段階を区別する。倍率を最終出力へ直接要求する前に後処理を確認する。チューニングSkillへ反映した。再実行はanalysis/script/prepare_closure_slip_host.pyとanalyze_closure_slip_13212_13227.py。
