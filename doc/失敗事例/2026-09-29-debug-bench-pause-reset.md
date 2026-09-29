---
type: codex-failure
date: 2026-09-29
task: "空転負荷ベンチの修正"
status: resolved
severity: low
tags:
  - codex/failure
  - robotrace
  - st-link
  - gdb
---

# 実行中ベンチをGDB入力だけで停止できなかった

## 要約

実行中のGDBセッションへCtrl+Cを送っても停止応答を確認できず、ST-LinkをGDBサーバーが保持したままではSTM32CubeProgrammerも接続できなかった。デバッガープロセスを終了してからUnder Resetで接続し、ハードウェアリセット後にコア停止できた。

## 発生した状況

- タスク: Debug空転ベンチの一時停止
- 実行環境: STM32CubeCLT 1.19.0、STM32CubeProgrammer 2.20.0、PowerShell
- 前提条件: STM32F446とST-Linkが接続され、GDBサーバー経由でDebugベンチを実行中。後からモーター電源が未接続だったと判明。

## 何を試したか

1. 実行中GDBセッションへCtrl+Cを複数回送った。
2. ST-Link共有中にSTM32CubeProgrammerでリセットを試した。
3. GDBとGDBサーバーを終了し、STM32CubeProgrammerをUnder Reset接続してハードウェアリセットとコア停止を行った。

## 結果・エラー

```
GDBセッション: Ctrl+C後も停止応答なし
STM32_Programmer_CLI: ST-LINK error (DEV_CONNECT_ERR)
Under Reset接続: STM32F446xx / Device ID 0x421、MCU Reset、Core halted
```

## 原因

GDB実行中のセッションへ送った入力が、実行中ターゲットへのリモート割り込みとして機能しなかった。GDBサーバーがST-Link接続を保持していたため、別プロセスのProgrammer CLIはプローブを取得できなかった。モーター電源は未接続だったため、この測定中にモーターへ駆動電力は供給されていない。

## 解決・回避策

GDBクライアントとGDBサーバーを終了した後、`STM32_Programmer_CLI.exe -c port=SWD mode=UR reset=HWrst freq=100 -rst -halt`を実行した。接続ログで対象MCUを確認し、ハードウェアリセットとコア停止の成功を確認した。

## 今後の予防策

- 実機ベンチ開始前に、モーター電源を含む接続状態を確認する。
- 実行中GDBから停止を確認できない場合は、GDBサーバーを解放してからUnder Resetのハードウェアリセットとコア停止を行う。
- 再開前にST-Link接続、モーター電源、PWM初期値ゼロをそれぞれ確認する。

## 関連

- 関連ノート: `2026-09-29-debug-bench-gdb-server-and-breakpoints.md`
- 参考リンク: なし
