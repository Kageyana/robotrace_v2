---
type: codex-failure
date: 2026-09-29
task: "Codex操作で完結する空転負荷検証"
status: resolved
severity: low
tags:
  - codex/failure
  - robotrace
  - st-link
  - gdb
---

# DebugベンチのGDBサーバー設定とブレーク位置を誤った

## 要約

GDBサーバー起動にProgrammerパス指定が必要で、行番号指定の診断ブレークポイントはソース変更後に意図しないウォッチドッグ更新位置へ移動した。

## 発生した状況

- タスク: ST-Link経由の空転測定
- 実行環境: STM32CubeCLT 1.19.0、ST-LINK GDB Server 7.11.0、PowerShell
- 前提条件: ST-Linkとターゲットは接続済み、STM32F446 Core IDを確認済み。

## 何を試したか

1. GDBサーバーを`-d -g -e`だけで起動した。
2. ソース行番号で測定停止後の状態を確認するブレークポイントを指定した。
3. リセット直後、アプリケーションのメインループ到達前にモード要求を設定した。

## 結果・エラー

```
GDB server: Couldn't locate STM32CubeProgrammer ... use -cp <path>
診断ブレークはdebugBenchWatchdogRefresh()へ解決され、測定中にコアを何度も停止した。
```

## 原因

- GDBサーバーがCubeProgrammer実行ファイルのディレクトリを必要とする。
- ソース行番号は変更後に別関数へ対応し、回転中のコアを停止した。
- リセット直後の要求値は起動初期化で消去される可能性がある。

## 解決・回避策

`-cp "C:\ST\STM32CubeCLT_1.19.0\STM32CubeProgrammer\bin"`を指定し、モード要求は`debugBenchMainLoop()`到達後に設定した。測定中のブレークは完了関数シンボルだけにし、結果取得後にデタッチする手順へ変更した。

2026-09-29に再発し、非表示の`Start-Process`起動で`-cp`を渡し忘れたためサーバーが終了した。前景起動でエラーを確認し、`-cp`を指定して起動し直すとポート61234で待受できた。`monitor reset halt`はこのST-LINK GDB Serverで未対応だった。リセット・停止が必要な場合はGDB接続を解放してからProgrammer CLIのUnder Reset手順を使う。

## 今後の予防策

- GDBサーバー起動時はSWD指定`-d`、ポート、Programmerパス`-cp`を明示する。
- 非表示起動ではプロセス起動だけで成功と判断せず、プロセスとTCP待受ポートを確認する。起動失敗時は前景起動でエラーを確認する。
- ST-LINK GDB Serverで`monitor reset halt`を使わず、GDBサーバーを終了してからProgrammer CLIのUnder Resetリセット・停止を使う。
- モード要求はアプリケーション初期化完了後に設定する。
- 測定中はソース行番号のブレークを置かず、シンボル名の完了地点だけで停止する。

## 関連

- 関連ノート: なし
- 参考リンク: なし
