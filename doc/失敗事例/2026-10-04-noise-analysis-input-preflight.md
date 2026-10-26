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

## 2026-10-26：動画解析のPython実体と局所依存関係

動画002～008とログ13810～13816の解析で、Python312/Python38のディレクトリがあることからpython.exeも存在すると仮定して起動し、実体なしで失敗した。通常Python310と同梱PythonにOpenCV/SciPyがあるという前提も成立しなかった。前回のimport事前確認に加え、実行ファイル自体と過去解析の局所依存先を確認する手順が不足した。

実在する`C:/Users/Natsuki/AppData/Local/Programs/Python/Python310/python.exe`とNumPy2.2.6を確認し、OpenCV4.11.0.86・SciPy1.15.3を`build/video-analysis-runtime`へ限定導入した。明示sys.path後のimportと版照合、7動画の全編復号・床校正・比較・最終CRC監査に成功。過去の`analysis/video_slip_13323_13337/deps`にも局所依存先が存在したため、次回は導入前に既存スクリプトのsys.path/depsと必要版を照合する。グローバルPython環境は変更していない。

次回の最初の処理は`Get-Command python.exe`と`Test-Path -LiteralPath <実体> -PathType Leaf`で実体を確定し、その実体で必要モジュールのspecとimport/版を確認する。バージョン名のフォルダ一覧を実行ファイル存在の代用にしない。依存追加前に既存の解析スクリプトのsys.pathと実在depsを読む。解析Skillの実行前へ反映した。今回の実行環境対策は検証済みでresolved、動画計測の物理精度とは別。
