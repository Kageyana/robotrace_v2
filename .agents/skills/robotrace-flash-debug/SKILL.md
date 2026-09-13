---
name: robotrace-flash-debug
description: STM32CubeCLT、STM32_Programmer_CLI、ST-Linkを使ってrobotrace_v2へ書き込みやデバッグを行うときに使う。実機接続条件、ビルド、書き込み、空転確認、緊急停止無効化条件を扱う。
---

# Robotrace Flash Debug

## Overview

Use this skill for firmware build, flash, and ST-Link debug workflows. Keep command definitions and safety policy in `AGENTS.md` as the source of truth.

## Preconditions

- Build before flashing.
- Flash only when the robot and ST-Link are connected.
- Debugging with ST-Link is allowed.
- Real running is not allowed during wired ST-Link debugging.
- The robot may be lifted for no-load motor checks, but it will not move forward and can quickly hit emergency-stop conditions.
- Disable emergency-stop conditions only for no-load, stationary, or non-moving checks where the target observation is possible without real travel.

## Build Commands

Run from `robotrace_v2/`:

- CodexのCMake/Ninjaビルドは、構成生成から最初からサンドボックス外で実行する。`exec_command`では`sandbox_permissions=require_escalated`を指定する。最初にサンドボックス内で試してから切り替える運用はしない。
- `cmake --preset` と `cmake --build` は同じ作業ディレクトリ、同じpreset、同じツールチェーンで実行する。
- サンドボックス外で実行できない場合は、ビルド成功とは報告せず、権限または実行環境の不足を報告する。

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

For release:

```powershell
cmake --preset Release
cmake --build --preset Release
```

If tools are missing, do not report build success. Report the missing tool or failed command.

### Ninjaがサンドボックス内で停止した場合

- サンドボックス内でNinjaを起動してしまった場合は、30秒以上無出力でCPU使用率0、またはコンパイラ子プロセスが起動しない状態を停止事象と判定する。
- 停止事象では同じビルドプロセスを長時間待ち続けず、安全に中断する。`git reset`、`build/`の削除、presetや生成先の変更は行わない。
- 中断後は、同じ作業ディレクトリと同じpresetで、構成生成からビルドまでをサンドボックス外で再実行する。Releaseなら次の順序を使う。

```powershell
cmake --preset Release
cmake --build --preset Release -- -j1
```

- サンドボックス外でコンパイル、リンク、終了コードを確認するまで、サンドボックス内の無出力状態をビルド失敗や成功と断定しない。
- 既存warningは変更箇所由来かを切り分け、リンク成功だけを根拠にwarningなしとは報告しない。

## Flash Workflow

1. Confirm ST-Link and robot connection.
2. Build the intended preset.
3. Run the configured VS Code task or equivalent `STM32_Programmer_CLI` SWD command.
4. Confirm the flash command exits successfully.
5. Report command, success/failure, and connection condition.

## Debug Workflow

1. Confirm robot and ST-Link are wired.
2. Use the configured launch setting:
   - `Build & Debug Microcontroller - ST-Link`
   - `Attach to Microcontroller - ST-Link`
3. Do not perform real running while wired.
4. For lifted no-load checks, clearly state that encoder-stop or movement-related emergency stops may trigger.
5. If emergency-stop conditions are disabled for debug, state which condition, why, and what risk remains.

## Post-Flash Checks

- Confirm the write command completed normally.
- Sensor checks happen through `initSystem()`.
- Motor no-load check is not required by default.
- Before high-speed running, confirm sensors initialized normally.
- High-speed running requires at least `7.5 V` battery voltage by operation policy.
