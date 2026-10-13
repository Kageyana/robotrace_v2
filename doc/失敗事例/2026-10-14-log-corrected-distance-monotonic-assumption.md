---
type: codex-failure
date: 2026-10-14
task: "ゲイン調整後17本の完走ログ検証"
status: resolved
severity: low
tags: [codex/failure, log-analysis, distance]
---

# マーカー補正済み距離を全モードで単調と仮定して正常ログを拒否

## 事実・原因

check_completed_runs_13195_13211.pyの初版はencTotalOptimalの全差分を正と仮定し、13198でAssertionError。一次やPATHは単調だったが、DISTANCE/SLIPは各5回、約30～49mmの後退段差があった。control.cのControl_ApplyMarkerCorrection_p()はencTotalOptimalから補正パルスを減算するため、正しい実装でも累積距離が後退する。全段差はcourseMarker=0→非0に一致し、200mm上限内。CSV時刻は単調、行数一致、有限だった。

## 対処・具体的な予防策

モードとマーカー遷移・補正上限を確認し、補正段差をログ欠落と区別する検証に修正した。正方向の刻みは5mm要求と1ms速度量子化を確認。全17本の再実行・原本SHA256一致・図生成が成功。以後、補正済み距離と生エンコーダ距離の意味を確認してから単調性を要求する。後退を無条件許可せず、DISTANCEモード・対応するマーカー遷移・設定上限を検証する。

同時に、二次ログのPIDヘッダ2/0/0を全周固定ゲインと解釈しないことを確認した。control.cのchangeGain()は直線時に2/0へ切り替え、カーブで保存値へ戻す。ゲインの比較にはモードと切替コードを確認する。
