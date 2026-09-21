---
type: codex-failure
date: 2026-09-16
task: "距離カルマン融合とコースXY閉路誤差の再確認"
status: resolved
severity: low
tags:
  - codex/failure
  - robotrace
  - bmi088
  - distance-kalman
  - acceleration-sign
---

# BMI088の物理軸方向を誤認して加速度符号不具合と判定した

## 要約

BMI088の物理Y軸正方向が機体前進方向と一致するという誤った前提で、
`BMI088getAccele()`のY軸`-1`倍を符号不具合と判定した。その後、実機では
BMI088の物理Y軸`-`方向が機体前進方向と一致すると訂正された。したがって現行の
`-1`倍は、物理Y軸を機体座標の前進正へ変換する正しい処理である。

## 発生した状況

- タスク: 実装済み距離カルマンとコースXY閉路誤差の再確認
- 実行環境: robotrace_v2実機ログ12439、12441、12444と現行ファームウェア
- 当初の前提: BMI088の物理Y軸正方向が機体前進方向と一致する（誤り）
- 訂正後の実機条件: BMI088の物理Y軸負方向が機体前進方向と一致する

## 何を試したか

1. BMI088の加速度軸変換を確認した。
2. 距離カルマンへ渡す前後加速度の取得経路を確認した。
3. 一次走行3本でIMU Y加速度とエンコーダ速度差分の相関を確認した。
4. 実機のBMI088物理軸方向を再確認した。

## 結果・エラー

```text
BMI088.c: BMI088val.accele.y = raw_y / ACCELELSB * -1
IMU.c:    IMU_GetForwardAccelerationMps2()は線形加速度Yを返す
control.c: DistanceEstimator_Update()へ上記前後加速度を渡す

直線区間・約100 ms平滑化後の相関:
12439: -0.056
12441: -0.077
12444: -0.021
```

物理Y軸負方向が前進なので、前進加速時の生Y値は負となり、コードの`-1`倍後は
`imuVal.accele.y`が正となる。`IMU_GetForwardAccelerationMps2()`および距離カルマンが
前進正を受け取るため、軸変換は整合している。

3本とも`distanceKalman.invalidUpdateCount=0`、`outputGuardCount=0`だった。ログは約10 ms
間隔の瞬時値なので、低い相関は軸符号不具合の証拠にはならない。

## 原因

実機軸方向の確認前に、BMI088の物理Y軸正方向が機体前進方向だと仮定したことが原因。
コードの`-1`倍とAPI名だけから実機搭載方向を逆算し、確認済み事実と推測を分離できて
いなかった。

## 解決・回避策

- `BMI088getAccele()`のY軸`-1`倍は維持し、ファームウェアを変更しない。
- 物理BMI088の`-Y`＝機体前進、変換後の`imuVal.accele.y`＝前進正という対応を
  `AGENTS.md`へ明記した。
- 誤った符号不具合判定を撤回し、本記録を`resolved`とする。

## 今後の予防策

- センサー軸の符号を判断する前に、実機上の物理軸方向を確認済み事実として記録する。
- センサー軸から機体軸への変換を一箇所へ集約し、物理軸とソフトウェア軸の対応を明記する。
- 実機を直線加速・減速させ、1 ms同期のエンコーダ速度差分とIMU前後加速度の符号、
  相関、時間遅れを確認するテストを軸変換変更時の必須項目にする。

## 関連

- `robotrace_v2/Core/Src/BMI088.c`
- `robotrace_v2/Core/Src/IMU.c`
- `robotrace_v2/Core/Src/control.c`
- `analysis/kalman_loop_closure_12439_12445/logs_12439_12445_summary.csv`
