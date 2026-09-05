---
type: codex-failure
date: 2026-08-23
task: PATH REPLAYのautoStart連続走行
status: open
severity: medium
tags:
  - codex/failure
  - robotrace
  - path-replay
  - battery
---

# PATH REPLAY autoStart 5走目がエンコーダ停止

## 要約

`shortcut.txt`のkLateralを半減した後、12170〜12172では操舵振動が改善したが、12173は`emcStop=3`（エンコーダ停止）で終了した。

## 発生した状況

- ソース一次ログ: 12169
- PATH REPLAY: 12170〜12173
- 設定変更: kLateral 3000→1500
- 12173のバッテリー電圧: 7.62 V

## 結果・エラー

```
12170: emcStop=0, max|pathErrorY|=46.4 mm
12171: emcStop=0, max|pathErrorY|=46.3 mm
12172: emcStop=0, max|pathErrorY|=46.0 mm
12173: emcStop=3, cntlog=2506, encCurrentN=5, targetSpeed=54
```

## 原因

12173は運用下限に近い7.62 Vで、停止時のエンコーダ値が5 pulse/msまで低下している。電圧低下による駆動力不足、またはコース上での機械的停止が候補。12170〜12172は`emcStop=0`だが、目標速度54のままログが終わっており、正常なゴール減速終了とは断定できない。

## 解決・回避策

未解決。充電済みバッテリー（比較時に8 V以上を目安）で再度autoStart 5走を行い、ログ末尾の目標速度低下・ゴールマーカー・走行終了状態を確認する。

## 今後の予防策

- 7.5 V未満または大きく電圧が異なる走行は、制御比較から除外する。
- `emcStop=0`だけで完走扱いせず、ゴールマーカーと正常減速ログを確認する。
- autoStart 5走比較は5本すべての終了条件が揃ってから採用判断する。

## 関連

- 関連ノート: 2026-08-23-path-replay-yaw-oscillation-hairpin.md
- 参考リンク:
