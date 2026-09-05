---
type: codex-failure
date: 2026-08-11
task: "電圧指令化後ログのPWM飽和と周回タイムの比較"
status: resolved
severity: medium
tags:
  - codex/failure
  - robotrace
  - log-analysis
---

# PWM飽和と周回タイムの単純相関を性能改善と誤解しない

## 概要
PWM飽和の多いログほど周回タイムが短く見えたため、PWM飽和が走行性能を改善しているように解釈しそうになった。

## 発生した状況
- タスク: 電圧指令化後の11273〜11371ログ比較。
- 実行環境: `codex/pid-line-speed-tuning` の実機走行ログ。
- 前提条件: `optimalTrace=2` のBOOST_DISTANCE走行を比較。

## 何を確認したか
1. 有効なBOOST_DISTANCEログをPWM飽和行数で分類した。
2. 周回タイム、速度誤差p95、スリップフラグ、バッテリー電圧を同時に比較した。
3. autoStart別と5走セット別にも分けて、単純相関の影響を確認した。

## 結果・エラー

```text
PWM飽和0行:     25本、平均5238.8 ms、速度誤差p95 0.558 m/s
PWM飽和10行超:  23本、平均5176.7 ms、速度誤差p95 0.589 m/s
単純相関:       -0.451
```

## 影響
周回タイムだけを見ると、PWM飽和が多いほど速いという誤った採用判断につながる。実際には、PWM飽和が多い群で速度誤差p95とスリップフラグが増えており、出力余裕が減少している可能性が高い。

## 解決・回避策
PWM飽和は性能向上指標ではなく、バッテリー電圧低下時の出力余裕を示す警告指標として扱う。周回タイムの比較では、同じ走行モード、autoStart、制御ゲイン、バッテリー電圧帯に分け、速度追従、スリップ、完走率と合わせて判断する。

## 今後の予防策
- 比較前に`emcStop`、`cntlog`、`optimalTrace`、autoStart、ビルド時刻、主要ゲインを確認する。
- 採用判断は`batteryVoltage_V >= 7.6 V`のログを優先し、低電圧ログは出力余裕の確認用に分ける。
- PWM飽和行数と周回タイムの相関だけで性能改善と判断しない。
- 制御変更後は最低10本について、周回タイム、速度誤差p95、スリップ、PWM飽和、完走率を同時に確認する。

## 関連

- 解析サマリ: `D:\robotrace\robotrace_v2\analysis\log_11273_11371_compare_summary.csv`
- セット比較: `D:\robotrace\robotrace_v2\analysis\log_11273_11371_compare_sets.csv`
