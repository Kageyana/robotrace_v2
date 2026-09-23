---
name: robotrace-flash-debug
description: STM32F446のビルド、ST-Link接続確認、STM32_Programmer_CLIによるSWD書き込み、静止状態のデバッグを行うときに使う。
---

# 書き込み・デバッグ

- 実機とST-Linkの接続を確認してから書き込む。ST-Link接続中に実走行しない。安全・電圧制約はリポジトリ直下の `AGENTS.md` を参照する。
- `robotrace_v2/` から `cmake --preset Debug` → `cmake --build --preset Debug`（Releaseなら両方とも `Release`）。構成・ビルドは同じpreset・ツールチェーンでサンドボックス外から実行する。VS Codeは `robotrace_v2/.vscode/settings.json` のCube環境を使う。
- Ninjaが無出力・CPU 0のまま30秒以上停止した場合は中断し、同じpreset・生成先でサンドボックス外から構成し直す。`build/` 削除や `git reset` で回避しない。環境起因の停止をソースのビルド失敗と報告しない（`doc/失敗事例/2026-09-06-codex-shell-stm32-vscode-build-hang.md`）。
- VS Codeの `robotrace_v2/.vscode/tasks.json` には `CubeProg: List all available communication interfaces`、`CubeProg: Flash project (SWD)`、`Build + Flash` がある。書き込みタスクは選択中のCMakeターゲットに `STM32_Programmer_CLI --connect port=swd --download <ELF> -hardRst -rst --start` を実行する。CLI利用時も対象ELFと終了コードを確認する。
- `robotrace_v2/.vscode/launch.json` の `Build & Debug Microcontroller - ST-Link` / `Attach to Microcontroller - ST-Link` を使う場合、`preLaunchTask` はコメントアウトされている。デバッグ前に意図したELFをビルド済みか確認する。
- 空転確認は機体を浮かせて静止したまま行う。緊急停止の一時無効化は動かず停止するデバッグまたは低速走行の誤停止に限り、変更条件・理由・残る危険を記録する。ST-Linkデバッグ中に走行確認を兼ねない。
- 書き込み後はCLI成功、起動時の `initSystem()` センサー状態を確認する。高速走行前はバッテリー7.5 V以上を確認する。実機未接続・ツール不足・実機動作未確認はそれぞれ区別して報告する。
