---
type: codex-failure
date: 2026-09-29
task: "IWDGリセット後の復旧処理を修正"
status: resolved
severity: low
tags:
  - codex/failure
  - robotrace
  - stm32
---

# Debugベンチのリセット要因復旧がHAL初期化より先に実行される

## 要約

`debugBenchCaptureResetCause()`が`HAL_Init()`より前に実行され、復旧処理が使う`HAL_GetTick()`の前提となるSysTick初期化より先に入っていた。

## 発生した状況

- タスク: IWDGリセット後のDebugベンチ復旧
- 実行環境: STM32F446、STM32 HAL
- 前提条件: IWDGリセット後にリセット要因をRAMへ保存し、IWDGを結果取得用の設定へ戻す。

## 何を試したか

1. IWDGリセット後に起動順と`debugBenchWatchdogRecoverAfterReset()`の待ち処理を確認した。
2. `debugBenchCaptureResetCause()`を`HAL_Init()`の直後へ移し、Debugを書き込んでIWDGリセットと通常起動後のベンチを確認した。

## 結果・エラー

`debugBenchWatchdogRecoverAfterReset()`はLSI/IWDG更新待ちのタイムアウト判定に`HAL_GetTick()`を使う。`HAL_Init()`より先ではSysTickが初期化されておらず、待ち処理の時間判定が成立しない可能性があった。

## 原因

リセット要因の取得自体はクロック非依存だが、同じAPI内で行うウォッチドッグ復旧処理はHAL tickに依存していた。APIを起動直後の`USER CODE BEGIN 1`から、SysTickを初期化する`HAL_Init()`直後へ置く必要があった。

## 解決・回避策

呼び出しだけを`HAL_Init()`直後の`USER CODE BEGIN Init`へ移し、短いコメントを追加した。リセット要因の保存、要求モードのクリア、IWDG復旧処理は変更していない。

実機のIWDGリセットで`state=RESET`、`stopReason=WATCHDOG_RESET`、`resetCauseFlags=0x24000000`（IWDGRSTFを含む）、要求モード0、左右PWM変数とTIM2左右比較値0、自動再開なしを確認した。通常リセット後のDebug PRIMARYも完走し、CSV 3,265行・28列、512 B整列エラー0、SD書込失敗0、バッファオーバーフロー0、1 ms周期超過0だった。

## 今後の予防策

- `HAL_GetTick()`を使う復旧処理は`HAL_Init()`完了後に呼び出す。
- 起動時フックを動かすコードを追加・移動したら、利用するHAL初期化済み要素（SysTickなど）との順序を確認する。
- IWDGリセット後は、保存されたリセット要因、要求モード、PWM変数、タイマ比較値、自動再開の有無を実機で確認する。

## 関連

- 関連ノート: `analysis/debug-bench-results-2026-09-29.md`
- 参考リンク: なし
