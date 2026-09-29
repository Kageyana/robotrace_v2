---
type: codex-failure
date: 2026-09-28
task: "Debug専用空転ベンチ"
status: resolved
severity: low
tags:
  - codex/failure
  - robotrace
  - build
---

# fixture生成をファームウェア階層から実行して失敗

## 要約

リポジトリルート基準のfixture生成コマンドをファームウェアサブディレクトリから実行し、生成ヘッダーを更新できなかった。そのままDebugビルドを行ったため、古いヘッダーとの型・マクロ不一致でコンパイルが停止した。

## 発生した状況

- タスク: Debug専用空転ベンチのfixture生成とDebugビルド
- 実行環境: Windows PowerShell、リポジトリ `D:\robotrace\robotrace_v2`
- 前提条件: シェルの作業ディレクトリを `robotrace_v2\` としていた

## 何を試したか

1. `robotrace_v2/tools/generate_debug_bench_fixture.py` を相対パスで実行した。
2. 続けて `cmake --build --preset Debug` を実行した。

## 結果・エラー

```
Python could not open ...\robotrace_v2\robotrace_v2\tools\generate_debug_bench_fixture.py
Debug build found missing fixture setting macros and a stale route fixture type.
```

## 原因

- 確認できた事実: generatorのパスはリポジトリルート基準だが、実行時の作業ディレクトリはファームウェアサブディレクトリだった。生成コマンドが失敗した後もビルドを続け、以前の生成ヘッダーを使った。
- 未確定事項: なし。

## 解決・回避策

リポジトリルートからgeneratorを実行し、生成ヘッダーのfixture件数・設定マクロを確認してからビルドを再実行する。

2026-09-28にリポジトリルートから再生成し、route 925点・replay 3619点を確認した。Debug、DebugMarker、Releaseの各構成もビルド成功したため解決済み。

## 今後の予防策

- generator実行前に `Get-Location` でリポジトリルートを確認する。
- 生成コマンドは `python robotrace_v2/tools/generate_debug_bench_fixture.py ...` の形でリポジトリルートから実行する。
- generatorが非0終了した場合はビルドを続けず、生成ヘッダーが更新されていることを確認する。

## 関連

- 関連ノート: なし
- 参考リンク: なし
