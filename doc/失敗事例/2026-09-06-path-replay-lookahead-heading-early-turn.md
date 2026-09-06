---
type: codex-failure
date: 2026-09-06
task: "PATH REPLAYの小Rコースアウト原因調査"
status: open
severity: high
tags:
  - codex/failure
  - robotrace
  - path-replay
  - control
---

# PATH REPLAYが先読み方位FBで小R手前から旋回した

## 要約

経路制御バージョン3のPATH REPLAYが、約120 mm先の方位を曲率FFと方位FBの両方へ使用し、小Rの約100 mm手前から旋回してコースを外れた。

## 発生した状況

- 対象一次走行: 12299
- 対象PATH REPLAY: 12300
- ファームウェア: `gitCommit=9251372`、`branch=codex/path-association-guard`、`routeControllerVersion=3`
- 設定: `shortcutLevel=0`、`lookaheadBaseMm=80`、`lookaheadPerMpsMm=40`、`kLateral=15.00`、`kHeading=16.00`、`lineAlpha=0`
- バッテリー開始電圧: 8.03 V

## 何を試したか

1. index 210～235の経路形状、目標角速度、実角速度、横偏差、方位偏差、モーター出力を確認した。
2. 一次走行12299の同一座標区間と目標角速度を比較した。
3. 40 mm経路を再構成し、120 mm先読み時の曲率FFと方位FBを分離して計算した。

## 結果・エラー

```text
12300: 5点以上のindexジャンプ=0、pathState=1を維持
index 212: PATH目標角速度 約+272 deg/s、一次走行の同一区間 約+4 deg/s
index 217: 横偏差最大 78.13 mm
index 218: 先読み方位偏差最大 62.70 deg
```

index 212では、約120 mm先の方位差12.4 degによる方位FBが約+198 deg/s、曲率FFが約+103 deg/sとなり、経路の現在接線がまだ直線の位置から右旋回指令が立ち上がった。モーター電圧上限到達は対象区間の一部だけで、対応点ジャンプもなかった。

## 原因

`pathFollowerUpdateTarget5ms()`が、先読み点の方位差を曲率FFの算出だけでなく方位FBにも使用していた。小Rでは先読み距離120 mmが旋回半径約83～105 mmと同程度以上になり、現在位置の経路接線が曲がる前から旋回指令が発生した。

## 解決・回避策

経路制御バージョン4として、方位FB、ロスト判定、再合流判定、方位偏差ログには最近傍経路点の接線方位差を使用する。先読み方位は曲率FFの算出だけに使用する。実機で最低10本確認できるまでは未解決とする。

## 今後の予防策

- 小R入口では、現在接線が直線の区間で先読み方位FBが操舵へ加算されていないことを回帰確認する。
- 制御バージョンをまたぐ比較では、バージョン3以前の先読み方位偏差とバージョン4以降の最近傍方位偏差を直接比較しない。
- 実機ではindex 210～235の横偏差、方位偏差、角速度指令の連続性と物理的なコースアウト有無を確認する。

## 関連

- 関連ノート: `2026-09-06-path-replay-nearest-index-jump-hairpin.md`
- 参考リンク: `robotrace_v2/Core/Src/pathFollower.c`
