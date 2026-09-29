---
type: codex-failure
date: 2026-09-29
task: "空転負荷ベンチの停止後処理診断"
status: open
severity: low
tags:
  - codex/failure
  - robotrace
  - st-link
  - gdb
---

# STM32診断で同時ハードウェアブレークポイント数を超えた

## 要約

停止後処理の通過点を一度に多数設定したところ、GDBがSTM32のハードウェアブレークポイント枠を使い切り、MCUを再開する前に診断を中止した。

## 発生した状況

- タスク: 空転負荷ベンチの停止後処理診断
- 実行環境: STM32F446、STM32CubeCLT 1.19.0、ST-LINK GDB Server 7.11.0
- 前提条件: モーター電源を切り、Under Resetで接続・停止した状態からGDB診断を開始。

## 何を試したか

1. メインループ、ベンチ開始、停止ラッチ、SD確定、`endLog`、`createLog`、CSV検査、完了地点などへ同時にブレークポイントを設定した。

## 結果・エラー

```
Cannot insert hardware breakpoint
Could not insert hardware breakpoints: You may have requested too many hardware breakpoints/watchpoints.
```

MCU再開前にGDBが終了し、ベンチ要求は実行されていない。

## 原因

STM32の利用可能なハードウェアブレークポイント枠より多いブレークポイントを同時に有効化した。

## 解決・回避策

この試行ではターゲットを再開していない。接続を解放後、Under Resetで再接続してPWMゼロを確認する。次の診断では停止地点ごとに一時ブレークポイントを設定し、同時使用数を抑える。

## 今後の予防策

- GDBスクリプトで多数のブレークポイントを並列設定しない。
- 一度に必要な地点だけを有効にし、停止後に次の一時ブレークポイントを設定する。
- GDB起動ログでブレークポイント挿入成功を確認してからターゲットを再開する。

## 関連

- 関連ノート: `2026-09-29-debug-bench-no-completion-usb-error.md`
- 参考リンク: なし
