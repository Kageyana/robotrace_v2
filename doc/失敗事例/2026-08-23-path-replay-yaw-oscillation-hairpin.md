---
type: codex-failure
date: 2026-08-23
task: PATH REPLAYで一次走行経路を再走行する
status: open
severity: medium
tags:
  - codex/failure
  - robotrace
  - path-replay
  - tuning
---

# PATH REPLAYがヘアピン出口でヨー振動してコースアウト

## 要約

最近傍方位によるLOCALIZATION_LOST判定へ修正した後、停止地点は約1.4 mから約2.7 mへ延びた。しかし、ヘアピン出口でヨーレート指令と実ジャイロが大きく振動し、`emcStop=7`となった。

## 発生した状況

- タスク: PATH REPLAYの実機走行
- 実行環境: STM32F446、routeControllerVersion=2、shortcutLevel=0
- 前提条件: 一次走行12167をソースに使用、バッテリー電圧7.88 V

## 何を試したか

1. 先読み方位をLOCALIZATION_LOST判定に使わない修正版を書き込んだ。
2. 一次走行12167を行い、PATH REPLAY 12168を実施した。
3. 動画を1/4速度で確認し、ヘアピン出口とログを同期した。

## 結果・エラー

```
12168: rows=235, cntlog=10-2689, optimalTrace=4, emcStop=7
最大|pathErrorY|=72.3 mm、最大|pathErrorHeading|=122.4 deg
pathStateは記録上1のままだが、停止直前に最近傍方位誤差が70〜90 degへ拡大
目標角速度は±1800 deg/sへ飽和、実gyroは約±1600 deg/s
```

## 原因

一次走行経路の離散点列はヘアピン区間で滑らかに変化しているため、経路生成の破綻ではない。`kLateral=30`、`kHeading=6`の経路操舵に対し、横誤差が増えた際に目標角速度が飽和し、既存の`yawRateCtrl`が遅れて反転することで過大応答している可能性が高い。バッテリー電圧が過去走行より低く、同一条件比較には不利。

## 解決・回避策

未解決。充電後、まず`shortcut.txt`の`kLateral`を3000から1500へ半減し、他の設定を固定して1走ずつ比較する。必要なら次の試行で`kHeading`または`yawRate` PIDを1項目だけ調整する。ショートカット生成はまだ有効化しない。

## 今後の予防策

- PATH REPLAYは充電後（運用下限7.5 V以上、比較時は同程度）に評価する。
- 1回の走行で変更するパラメータは最大2個、原則1個にする。
- 目標角速度の飽和率、gyro、横誤差、最近傍方位誤差をログ比較し、最低10走で再現性を確認する。

## 関連

- 関連ノート: 2026-08-23-path-replay-localization-false-stop.md
- 参考リンク:
