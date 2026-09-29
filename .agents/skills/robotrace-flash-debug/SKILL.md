---
name: robotrace-flash-debug
description: robotrace_v2をSTM32CubeCLT、STM32_Programmer_CLI、ST-Link/GDBでビルド・書き込み・デバッグするときに使う。Codexからの空転負荷検証、接続確認、停止・復旧と失敗手順の回避も扱う。
---

# robotrace_v2 書き込み・ST-Linkデバッグ

## 基本方針

- 接続・安全条件はリポジトリの `AGENTS.md` を正とする。書き込みは機体とST-Linkが接続されているときだけ実行する。
- ST-Link接続中に実走行しない。空転確認では機体を固定して車輪を浮かせる。緊急停止条件の一時無効化は、静止状態で確認できる対象条件に限る。
- 操作を始める前に `doc/失敗事例/` から関連事例を検索する。失敗した接続・停止操作を繰り返さず、対象MCUと安全状態を確認してから進む。
- CodexからCLI/GDBで操作する、またはDebug空転負荷ベンチを実行する場合は、先に [CodexからのST-Link操作手順](references/codex-stlink-cli-debug.md) を読む。

## ビルド

`robotrace_v2/` で対象構成をビルドする。書き込むELFとデバッグに読み込むELFを同じ構成にする。

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

Releaseの場合:

```powershell
cmake --preset Release
cmake --build --preset Release
```

ツール不足やコマンド失敗をビルド成功として扱わない。

## 書き込み

1. 機体とST-Linkの接続、対象MCUのDevice ID、ビルド成果物を確認する。ST-Linkの列挙とターゲット電圧だけではMCU接続成功と判定しない。
2. VS Codeの `CubeProg: Flash project (SWD)` タスク、または `STM32_Programmer_CLI` で意図したELFを書き込む。通常SWDで接続できない場合は、参照手順にある100 kHz・Under Resetを使う。
3. 書き込み・照合・起動の成否をコマンド出力で確認する。Device IDが取得できない状態で書き込みを続けない。

## デバッグ

VS Codeからは `Build & Debug Microcontroller - ST-Link` または `Attach to Microcontroller - ST-Link` を使う。Codexからの直接操作と空転負荷ベンチは参照手順に従い、GDBサーバーとProgrammer CLIでST-Linkを同時に保持しない。モーター回転中は不要なブレークポイントでコアを停止しない。

## 実行後

- 書き込み成功と `initSystem()` のセンサー初期化結果を確認する。通常の書き込みだけなら空転確認は必須ではない。
- 高速走行前はセンサー初期化成功とバッテリー電圧7.5 V以上を確認する。
- 緊急停止条件を一時変更した場合は、変更条件、理由、残る実機リスクを報告する。
