---
type: codex-failure
date: 2026-10-04
task: "13005の高音と過去ログの比較"
status: resolved
severity: low
tags: [codex/failure, analysis]
---

# 音声・過去ログ比較前の依存関係と列の確認不足

## 結果

バンドルPythonへSciPy、Matplotlibをimportして未導入エラー。通常PythonにはNumPy・Matplotlibがあり、SciPyなしでNumPy FFTを使って解析できた。過去の最小ログにlineTraceCtrlがなくKeyErrorとなり、そのプロファイルを比較から除外した。初回集計でU16時刻折り返しを時間重みに入れ、負の重みによるsqrt warningを発生させた。

## 原因と対処

解析前の環境・ヘッダ・時刻仕様の確認が不足。SciPyへの依存をなくし、列を事前確認し、折り返しを65536 ms加算して重みを正にした。残る非正時刻は比較から除外する。入力CSVは変更していない。

## 再発防止と確認

`python -c "import numpy,matplotlib"`で必要ライブラリを事前確認する。過去CSVは列名を調べてから診断列を使用する。時間重みを作る前にU16折り返しを補正し、非正差分を排除する。investigate_noise_13005.pyへ反映し、音声FFT・集計・グラフ生成の再実行がwarningなしで成功した。13005は期待4672行、欠落疑い・非有限値・時刻異常0を確認した。
