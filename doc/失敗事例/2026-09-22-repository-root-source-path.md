---
type: codex-failure
date: 2026-09-22
task: "IMU温度ログの確認"
status: resolved
severity: low
tags:
  - codex/failure
---

# リポジトリ直下とファームウェアプロジェクト直下の混同

## 要約

リポジトリ直下から `Core/Src` を参照し、ソース検索が失敗した。実際のファームウェアは `robotrace_v2/Core/Src` にある。

## 発生した状況

- タスク: IMU温度ログとCSV生成コードの確認
- 実行環境: `D:\robotrace\robotrace_v2` を作業ディレクトリとするPowerShell
- 前提条件: リポジトリ直下に同名のファームウェアプロジェクトディレクトリがある

## 何を試したか

1. リポジトリ直下から `Core/Src/SDcard.c` を検索した。

## 結果・エラー

```text
Core/Src/SDcard.c: 指定されたパスが見つかりません。
```

## 原因

リポジトリルートと、配下のSTM32プロジェクトルート `robotrace_v2/` を混同した。

## 解決・回避策

`robotrace_v2/Core/Src/SDcard.c` を指定し、ソースを確認できた。

## 今後の予防策

ソース調査・ビルド前にリポジトリ直下の一覧を確認し、ファームウェアは `robotrace_v2/` 配下、解析成果物は `analysis/` 配下という構成を基準にする。

## 関連

- 関連ノート: `AGENTS.md` の「リポジトリ構成」
- 参考リンク:
