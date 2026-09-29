---
type: codex-failure
date: 2026-09-29
task: "SD初期化失敗後のDebugベンチ調査"
status: resolved
severity: low
tags:
  - codex/failure
  - robotrace
  - stm32
  - gdb
---

# 停止中のGDBからSD初期化関数を呼び出して応答待ちになった

## 要約

ベンチ終了地点で停止中にGDBの関数呼び出しから`disk_initialize(0)`を実行したところ、呼び出しが戻らずデバッガも応答しなくなった。

## 発生した状況

- タスク: 起動時SD初期化失敗の原因確認
- 実行環境: STM32F446、ST-LINK GDB Server、arm-none-eabi-gdb
- 前提条件: ベンチ開始前のSD初期化失敗を、停止中ターゲットから再試行しようとした。

## 何を試したか

1. GDBで`disk_initialize(0)`を直接呼び出した。
2. 15秒待ってもGDBプロンプトが戻らず、Ctrl-Cにも応答しなかったためGDBとサーバーを終了した。
3. Programmer CLIでUnder Resetハードウェアリセット後、通常起動経路で初期化を再確認した。

## 結果・エラー

直接関数呼び出しは戻り値を得られなかった。PWM変数とTIM2比較値は事前確認で0だった。通常起動経路へ戻した後はSD初期化が成功し、ベンチを完走できた。

## 原因

GDB inferior call中にHAL SPI処理またはtick待ちが完了しなかった可能性があるが、停止位置を回収できておらず原因は未確定。

## 解決・回避策

GDBクライアントとサーバーを終了し、Programmer CLIのUnder Reset手順で安全なリセット・停止を行った。その後は起動時の通常処理を実行してSD初期化を確認した。

## 今後の予防策

- 停止中GDBからブロッキングHAL、SPI、FatFs関数を直接呼ばない。
- 周辺機器の再初期化は通常の起動経路または専用Debug制御フローで実行し、関数入口・戻り値にブレークポイントを置いて確認する。
- GDB inferior callが応答しない場合は長時間入力を重ねず、対象停止状態とPWMゼロを確認してからデバッガを終了し、Programmer CLIでUnder Resetを行う。

## 関連

- 関連ノート: `doc/失敗事例/2026-09-29-log-padding-bench-sd-init-failure.md`
- 参考リンク: なし
