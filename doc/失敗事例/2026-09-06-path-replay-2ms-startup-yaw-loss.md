---
type: codex-failure
date: 2026-09-06
task: Level 0 PATH REPLAYを2.00 m/sで走行する
status: open
severity: high
tags:
  - codex/failure
  - robotrace
  - path-replay
  - tuning
---

# Level 0の2.00 m/s開始直後にヨー応答が崩れて自己位置を喪失

## 要約

経路制御バージョン10でLevel 0専用速度を2.00 m/sへ設定した走行のうち、12334が開始約1.06秒で`STOP_LOCALIZATION`となった。経路indexジャンプではなく、開始73 msで実ジャイロが目標と逆方向へ急変し、その後にモーター出力と目標角速度が飽和して横偏差が拡大した。

## 発生した状況

- タスク: Level 0 PATH REPLAYの速度2.00 m/s検証
- 実行環境: STM32F446、`routeControllerVersion=10`、`shortcutLevel=0`
- 設定: `tgtParam.pathReplay=2.00 m/s`、`lineAlpha_x1000=010`
- 対象ログ: 一次走行12332、PATH REPLAY 12333、12334
- 失敗ログ: 12334、`autoStart=3`、バッテリー電圧7.65 V

## 何を試したか

1. Level 0速度を一次走行の`search`から分離し、`pathReplay=2.00 m/s`で実機走行した。
2. Version 10のPATH REPLAY 12331、12333、12334、12336～12339を同一モードとして比較した。
3. 開始200 msの実ジャイロ、横偏差、モーター飽和、スリップ、indexジャンプを正常走行と比較した。

## 結果・エラー

```text
12334: emcStop=7、cntlog終端=1056 ms、最終pathState=4
最大|pathErrorY|=191.1 mm、最大indexジャンプ=1
cntlog=73 ms: targetYawRate=+199 deg/s、gyro=-932.9 deg/s
開始200 ms: モーター飽和14サンプル、最大|gyro|=933 deg/s、最大|pathErrorY|=63.5 mm
```

同じ2.00 m/s設定で完走した6本は、開始200 msの最大`|gyro|`が15～38 deg/s、最大`|pathErrorY|`が0.7～1.9 mmで、同区間のモーター飽和は0サンプルだった。完走ログの横偏差p95は29.2～33.3 mm、最大値は57.3～60.8 mmだった。

## 原因

経路indexの最大ジャンプは1で、既知の経路対応点ジャンプは再発していない。12334だけで開始直後に目標と逆方向の大きな実ジャイロと駆動飽和が発生しており、機械的な外乱、発進姿勢、IMU過渡、左右駆動差のいずれかは未確定である。

現在の経路速度プロファイルは先頭経路点を2.00 m/sとして生成し、停止速度から先頭速度までの加速境界を持たない。正常走行でも同じ目標値で完走しているため単独原因とは断定できないが、2.00 m/sへの開始時ステップが外乱への余裕を小さくした可能性がある。

## 解決・回避策

未解決。2.00 m/sはまだ採用せず、バッテリー7.5 V以上でVersion 10のLevel 0を最低10本取得する。開始直後の異常が再発する場合は、Level 0速度を下げるか、速度プロファイルへ停止速度からの発進加速境界を追加する。

## 今後の予防策

- Level 0速度変更後は、完走ログだけでなく開始200 msの最大`|gyro|`、最大横偏差、モーター飽和を確認する。
- 速度プロファイル変更時は先頭点と終端点の境界速度を明示し、`acceleF`、`acceleD`が停止状態との境界にも適用されることを確認する。
- バッテリー7.5 V未満の走行は正式な採用判定から除外する。
- 5点以上のindexジャンプ、フォールバック、自己位置喪失、横偏差p95 50 mm以下、最大65 mm以下を最低10本で確認する。

## 関連

- 関連ノート: 2026-08-23-path-replay-yaw-oscillation-hairpin.md
- 関連ノート: 2026-09-06-path-replay-nearest-index-jump-hairpin.md
