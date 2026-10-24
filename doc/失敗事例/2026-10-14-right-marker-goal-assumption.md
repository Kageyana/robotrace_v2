---
type: codex-failure
date: 2026-10-14
task: "既存ログの閉路XとSLIP減速確認"
status: resolved
severity: low
tags: [codex/failure, log-analysis, marker]
---

# 右マーカーの記録回数をSG加算回数と同一視した

解析初版はcourseMarker=1の開始行を5周のSGイベントと仮定し、13212で8回となって停止した。原本異常ではない。markerSensor.hでは1=RIGHTMARKER、2=LEFTMARKER、3=CROSSLINE。checkStartGoalMarker()は右マーカーに加えて前回から1000mm超などの条件を確認してSGmarkerへ加算するため、一対一ではない。通常43列にはsgMarkerCountの時系列がない。

繰り返し対応を確認できるクロス5回を整列基準へ変更し、終端は保存済みgoalMarkerOnset_pを使用した。初回・末尾の部分区間は「周回」と呼ばずsectionとして保存した。元CSV/メタデータの実C XY再現、4本のクロス対応、原本SHA256不変を含む解析が成功し、対処を検証した。

次回はマーカー値の定義とSG加算条件を参照してからイベント数を仮定する。記録がないSG状態を右マーカーだけで補完しない。ログ解析Skillへ反映し、再実行はanalysis/script/analyze_closure_slip_13212_13227.py。結果はanalysis/closure_slip_13212_13227/。
