---
type: codex-failure
date: 2026-08-16
task: "speed_ff=160 の速度追従比較"
status: resolved
severity: medium
tags:
  - codex/failure
  - robotrace/tuning
  - speed-feedforward
---

# speed_ff=160で速度追従改善が再現しなかった

## 要約
`speed_ff=160` は速度FF=150に対して、周回番号別の速度誤差p95とlap短縮を一貫して改善できなかったため不採用とした。

## 発生した状況

- タスク: 速度FF=150から160への中間比較
- 実行環境: robotrace_v2、MOTOR_COMMAND_NOMINAL_V=7.1V、速度PID=12/0/0
- 前提条件: autoStartの周回番号別比較、開始電圧7.3V以上、失敗セット全体除外

## 何を試したか

1. ログ11936〜11965を6セット取得した。
2. 11966〜11970の失敗セットを除外し、11971〜11990の4セットを追加確認した。
3. speed_ff=150基準11921〜11935とautoStart=2〜5を同じ周回番号で比較した。

## 結果・エラー

- 160の追加4セットでも、速度誤差abs p95は2周目・3周目・4周目で基準より悪化した。
- 2周目、4周目ではlapと横スリップも悪化し、改善方向が周回番号間で揃わなかった。
- 走行開始電圧は7.53〜8.15Vで幅があり、電圧差も比較の交絡要因になった。

## 原因

速度FFを増やしても、速度追従誤差の改善が安定して現れなかった。PWM余裕、適応速度計画、バッテリー電圧差の影響が分離できず、160を採用できる根拠が不足した。

## 解決・回避策

speed_ff.txtを既知基準の150へ戻した。速度FFの連続比較は打ち切り、次は速度プロファイルまたはスリップ反映側を1項目だけ比較する。

## 今後の予防策

autoStart=2〜5を混ぜずに比較し、開始電圧帯を揃える。失敗セットは個別完走ログを使わず、セット全体を除外する。

## 関連

- 解析: D:/robotrace/robotrace_v2/analysis/log_11966_11990_speed_ff160_comparison.csv
- 解析: D:/robotrace/robotrace_v2/analysis/log_11921_11965_speed_ff160_comparison.csv
