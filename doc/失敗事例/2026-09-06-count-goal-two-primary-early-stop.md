---
type: codex-failure
date: 2026-09-06
task: "COUNT_GOAL=2変更後の一次走行ログ確認"
status: open
severity: high
tags:
  - codex/failure
  - robotrace
  - marker
  - early-goal
---

# COUNT_GOAL=2で一次走行が中間右マーカー終了した

## 要約

ゴールマーカー読み取り回数を2回へ変更した後の一次走行12295が、従来の完走距離約36 mに対して20.48 mで終了した。終了直前に`courseMarker=RIGHTMARKER`、終了ヘッダに`sgMarkerAtLogEnd=2`が記録されており、中間右マーカーをゴールとして扱った可能性が高い。

## 発生した状況

- 対象ログ: 12295
- `optimalTrace=0`、`autoStart=1`、`emcStop=0`
- `batteryVoltage_V=8.14 V`
- `routeControllerVersion=3`
- `sgMarkerAtLogEnd=2`
- ログヘッダは`gitCommit=e56359a`、`branch=main`で、変更後コミットとは一致していない。

## 確認結果

```text
12295: cntlog=20060 ms、距離=20475.4 mm、終了直前courseMarker=1
12289: cntlog=36249 ms、距離=37135.1 mm
12291: cntlog=35261 ms、距離=36062.2 mm
12293: cntlog=35224 ms、距離=36060.8 mm
```

12295は目標速度54 pulse/msの走行中に、20,050 ms、20,465.4 mm地点で`RIGHTMARKER`を記録し、その後10 msでログが終了した。従来3走の同距離付近では`LEFTMARKER`が記録されている。

## 原因または未確定事項

`COUNT_GOAL=2`では、スタート計数後の最初の右マーカーがゴール条件になる。12295ではコース中間のマーカーが右マーカーとして確定したため、一次走行にはPATH系の進捗80%ゲートが適用されず終了条件が成立したと考えられる。

同じ物理マーカーを左右逆に認識した原因は未確定。マーカーセンサー実配線、左右検出値、機体位置の再現性を詳細ログで確認する必要がある。

## 対処または次の調査

- 12295をPATH REPLAYの一次経路として採用しない。
- `LOG_SCHEMA_PROFILE_LIGHT=0`で`markerSensor`、`courseMarker`、`sgMarkerCount`、`encRightMarker_p`を確認する。
- ゴール計数を2のまま使う場合は、一次走行にも誤右マーカーを除外できる追加条件を設計する。
- 追加条件を実装しない場合は、一次走行の`COUNT_GOAL=3`復帰を候補とする。

## 今後の予防策

- 一次走行ログは`emcStop=0`だけで完走扱いせず、過去の正常距離、`sgMarkerAtLogEnd`、終了直前の`courseMarker`を照合する。
- 変更後のファームウェアはログヘッダの`gitCommit`と`branch`が対象コミットに一致することを確認してから比較する。
- ゴール条件変更後は、コース中間のすべての右マーカーで終了しないことを確認する。

## 関連

- `2026-09-03-path-replay-false-goal-localization.md`
- `robotrace_v2/Core/Inc/control.h`
- `robotrace_v2/Core/Src/control.c`
- `robotrace_v2/Core/Src/markerSensor.c`
