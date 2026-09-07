---
type: codex-failure
date: 2026-09-06
task: "PATH REPLAY修正後のDebugMarker・Releaseビルド"
status: open
severity: medium
tags:
  - codex/failure
  - build
  - stm32
  - vscode
  - ninja
---

# CodexシェルからSTM32 VS Codeビルド環境を再現するとNinjaが停止した

## 要約

VS Code拡張「STM32CubeIDE for Visual Studio Code」が管理するプロジェクトをCodexの非対話シェルからビルドしたところ、CMakeの依存判定後またはコンパイラABI確認中にNinjaがCPUを使わず停止した。ARM GCCを直接実行したコンパイルとリンクは成功した。

## 発生した状況

- プロジェクト設定: `robotrace_v2/.vscode/settings.json`
- VS Code拡張環境: CMake 4.0.1、Ninja 1.13.1、GNU Arm 13.3.1
- 比較環境: STM32CubeCLT 1.20.0同梱CMake、Ninja、GNU Arm 13.3.1
- 対象preset: DebugMarker、Release
- 2026-09-06の起動時ファームウェア識別表示追加でも、Debug presetのビルドが無出力で停止し、再構成は`Detecting C compiler ABI info`で停止した。

## 結果・エラー

- `cmake --build --preset DebugMarker`はNinja起動後に出力なしで停止した。
- `cube-cmake --preset DebugMarker`は`Detecting C compiler ABI info`で停止した。
- `cube-cmake`に必要な`cube` wrapperと`CUBE_BUNDLE_PATH`を設定しても同じだった。
- 生成済みの全60ビルドコマンドを直列実行すると、コンパイルとリンクは終了コード0で完了した。
- 起動時ファームウェア識別表示追加では、停止したCMake再構成後も`build.ninja`の`GIT_COMMIT`が親コミットのままであり、その設定を使ったELFを採用できなかった。

## 原因または未確定事項

Codexの非対話シェルから起動したNinja固有の停止であることまでは確認した。VS Code拡張内部からのタスク実行との差分は未確定。コンパイラ、ソース、リンカ自体の異常ではない。

## 実施した対処

- DebugMarkerは生成済みビルドコマンドを直列実行し、ELF生成、警告、RAM/FLASHを確認した。
- Releaseは同じGNU Armコンパイラへ`-Os -g0`を指定し、変更対象`pathFollower.c`のコンパイルを確認した。
- Codexセッションではネイティブアプリ制御が無効だったため、VS Code拡張タスクの実行は行っていない。
- 起動時ファームウェア識別表示追加では、生成済みDebugビルドコマンド60件へ現行コミットIDを明示して直列実行し、終了コード0、RAM 75,720 B、FLASH 205,892 BでELFを生成した。新規warningはなかった。
- 経路制御バージョン4の確認では、Release presetの通常構成が再びコンパイラABI確認で停止した。`CMAKE_C_COMPILER_WORKS`、`CMAKE_CXX_COMPILER_WORKS`、`CMAKE_C_COMPILER_FORCED`、`CMAKE_CXX_COMPILER_FORCED`を`TRUE`として既知のコンパイラ確認だけを省略すると構成が完了し、生成済み全コマンドの直列実行でRelease ELFを生成できた。
- 経路制御バージョン11の確認では、通常サンドボックス内のNinjaが子コンパイラを起動せず停止した。同じ`cmake --build --preset Debug/Release -- -j1`をサンドボックス外で実行すると、DebugとReleaseの全60コマンドが終了コード0で完了した。

## 今後の予防策

- このPCでは通常の最終ビルドをVS Code拡張のCMakeタスクから実行する。
- Codexシェルで確認する場合は、先に`robotrace_v2/.vscode/settings.json`とbundleの実体を確認し、CubeCLT環境と混同しない。
- Ninjaが30秒以上無出力かつCPU 0のときは中断し、ARM GCCの直接実行でソース由来か環境由来かを切り分ける。
- 通常サンドボックス内だけで停止し、ARM GCC単体が応答する場合は、同じCMake presetをサンドボックス外で再実行する。preset、作業ディレクトリ、生成先は変更しない。
- Release構成が`Detecting C compiler ABI info`で停止し、同じコンパイラのDebug全ビルドが成功済みの場合に限り、コンパイラ動作確認済みフラグを明示してABI試行を省略する。生成後は全コンパイル・リンクコマンドを実行して実体を検証する。
- VS Code拡張からDebugMarkerとReleaseが成功した後に、本記録を`resolved`へ変更する。

## 確認方法

VS CodeでDebugMarkerとReleaseのclean rebuildを実行し、両方のELF更新時刻、終了コード、新規warning、RAM/FLASH使用量を確認する。
