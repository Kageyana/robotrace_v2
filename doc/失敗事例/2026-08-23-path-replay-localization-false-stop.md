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
---

# PATH REPLAYが先読み方位のロスト判定で停止

## 要約

一次走行ログは正常に生成できたが、PATH REPLAYがコースの最初の右カーブ付近で3回連続して `emcStop=7`（`STOP_LOCALIZATION`）になった。

## 発生した状況

- タスク: 固定終点補正を外したPATH REPLAYの実機確認
- 実行環境: STM32F446、PATH REPLAY、AutoCopy停止中に走行後ログを取得
- 前提条件: `routeControllerVersion=2`、`lineAlpha=0`、目標速度はログ上54 pulse/ms

## 何を試したか

1. 一次走行ログ12160/12161、12163、12165を生成した。
2. それぞれをPATH REPLAYのソースにして12162、12164、12166を生成した。
3. 固定ゴール補正を外したファームウェアで再走行した。

## 結果・エラー

```
12162: emcStop=7, pathState 1:114 / 2:26 / 4:1, max|pathErrorY|=77.9 mm, max|heading error|=109.3 deg
12164: emcStop=7, pathState 1:137, max|pathErrorY|=49.6 mm, max|heading error|=98.8 deg
12166: emcStop=7, pathState 1:136, max|pathErrorY|=52.2 mm, max|heading error|=108.1 deg
```

12166では、経路点index 28付近（約610,576 mm）で横誤差は約12 mmなのに、先読み方位を使ったheading errorが約82 degとなり、その後LOCALIZATION_LOSTへ遷移した。

## 原因

一次走行ログの形状不正やAutoCopyによる削除ではない。`pathFollowerUpdateTarget5ms()`のロスト判定が、操舵目標に使う先読み方位の誤差（`headingError`）を使っている。急カーブでは先読み方位と現在接線の差が大きくなるため、機体が経路近傍でもロストと誤判定する可能性が高い。

## 解決・回避策

未解決。ロスト判定だけは最近傍経路点の接線方位誤差を使い、先読み方位誤差は操舵専用に分離してから再走行する。

## 今後の予防策

- PATH REPLAYのテストログでは、`pathErrorY_mm`と最近傍方位誤差を別々に記録して判定する。
- 急カーブで先読み方位誤差だけが大きい場合にLOCALIZATION_LOSTへ遷移しないことを静的テストまたはログ再生で確認する。
- 一次走行ログの削除監視と経路解析エラーを、AutoCopyの動作とは別に確認する。

## 関連

- 関連ノート: 2026-08-23-path-replay-log-11-with-valid-csv.md
- 参考リンク:
