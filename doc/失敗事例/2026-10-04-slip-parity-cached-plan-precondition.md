---
type: codex-failure
date: 2026-10-04
task: バイナリ/CSVのSLIP速度計画一致検証
status: resolved
severity: low
---

# SLIP比較で前回補正済みの計画をキャッシュ再利用した

## 確認した事実

バイナリ経由のSLIP解析直後にCSV経由のSLIP解析を実行すると、先頭区間速度が2.0999999から2.20499992へ変わり、一致テストが失敗した。ROCは同じ3000だった。

## 原因と対処

readLogDistanceSlip()はBOOST_DISTANCE・解析番号一致時にPPADを一次計画として再利用する。テストは2回目に前回のSLIP補正後PPADを渡し、補正を二重適用していた。2回目のSLIP解析前にreadLogDistance(1)で一次計画を再生成した。ファームウェアのキャッシュ挙動は変更していない。

## 再発防止と確認

速度計画のA/B比較では入力ファイルだけでなく、optimalTrace、analyzedNumber、PPAD、パラメータを同じ初期状態へ戻す。各SLIP試行前に一次距離計画を再生成し、ROCとboostSpeedを全区間比較する。tests/run_log_deferred_tests.ps1で通常・詳細の双方が一致し、resolved。実走行の性能一致は別途確認する。
