---
type: codex-failure
date: 2026-09-22
task: "温度補正ON/OFFログの中央値比較"
status: resolved
severity: low
tags:
  - codex/failure
  - powershell
  - log-analysis
---

# PowerShellでCSV列を配列キャストして中央値を誤集計

## 要約

`Import-Csv`のプロパティ列を`[double[]]`へ直接キャストして中央値関数へ渡したところ、期待した3要素の配列として扱われず誤った中央値を表示した。

## 発生した状況

- タスク: 温度・電圧差が許容内の3組についてYaw誤差中央値を比較
- 実行環境: Windows PowerShell
- 前提条件: CSVの数値列は文字列として読み込まれる

## 何を試したか

1. `$valid.off_yaw_error_deg`を`[double[]]`へ直接キャストした。
2. 個別値から求めた期待値と出力が一致しないことを検出した。
3. Pythonの`statistics.median`で再計算した。

## 結果・エラー

```text
PowerShell出力: OFF 7.642 / ON 8.157
期待値: OFF 6.532 / ON 7.597
```

## 原因

CSVオブジェクトのプロパティ列を直接配列キャストする書き方が、想定した要素単位の数値変換にならなかった。

## 解決・回避策

各行を`ForEach-Object { [double]$_.field }`で明示変換するか、分析集計はPythonの`statistics.median`へ統一する。

## 今後の予防策

中央値などの集計後は、少数サンプルでは必ずソート済み個別値も表示して結果を照合する。

## 関連

- 解析表: `analysis/imu_temp_comp_12571_12586/log_12571_12586_compensation_pairs.csv`
