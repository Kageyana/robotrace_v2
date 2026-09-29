---
type: codex-failure
date: 2026-09-29
task: "Codex操作で完結する空転負荷検証"
status: resolved
severity: low
tags:
  - codex/failure
  - robotrace
  - stm32
---

# Debugベンチの独立ウォッチドッグ設定順がSTM32 HAL手順と不一致

## 要約

IWDGの更新フラグが残った状態で設定完了を待ち、Debugベンチが開始前にWATCHDOG停止となった。

## 発生した状況

- タスク: Debug専用空転ベンチ
- 実行環境: STM32F446、STM32 HAL IWDG
- 前提条件: ベンチ開始前にLSI起動を確認し、独立ウォッチドッグの分周・リロード値を設定する。

## 何を試したか

1. IWDGの分周・リロード値を設定してから開始キーを書き込んだ。
2. 更新フラグのクリアを待ってベンチ開始可否を判定した。

## 結果・エラー

```
IWDG SRのPVU/RVUが設定状態のままタイムアウトし、ベンチ開始前に停止理由WATCHDOGとなった。
```

## 原因

STM32 HALの初期化順と異なり、IWDG開始キーより前に設定を書いていたため、設定更新待ちが完了しなかった。

## 解決・回避策

LSI ready確認後にIWDG開始キーを書き、書込みアクセスを許可して分周・リロードを設定し、更新フラグ消去後にリロードする順へ変更した。最終6ケースすべてで`watchdogStarted=1`となり、ウォッチドッグ停止・リセットなしで完了した。

2026-09-29の開始直前確認でも、ベンチ開始関数がレジスタ設定を読み戻した後にのみ開始直前ブレークへ進むことを確認した。PWM変数とTIM2 CCR2/CCR3が0の状態でデバッガー停止中にIWDGリセットし、結果に`IWDGRSTF`、モード要求0、停止後PWMゼロが記録された。リセット後に見えるRLR=4095とデバッグ停止フリーズは、結果取得用の回復設定であり、走行中RLR=255の値ではない。

## 今後の予防策

- IWDG設定は対象STM32 HALの開始・設定順に合わせる。
- Debug接続でベンチを実行するときは、起動成功フラグと停止理由を結果構造体で確認する。

## 関連

- 関連ノート: なし
- 参考リンク: [STM32F4 HAL IWDG driver](https://github.com/STMicroelectronics/stm32f4xx-hal-driver/blob/master/Src/stm32f4xx_hal_iwdg.c)
