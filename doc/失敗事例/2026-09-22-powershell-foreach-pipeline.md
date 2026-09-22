---
type: codex-failure
date: 2026-09-22
task: "IMU温度ログのCSV先頭行確認"
status: resolved
severity: low
tags:
  - codex/failure
---

# PowerShellのforeach文直後へパイプを接続して構文エラー

## 要約

`foreach (...) { ... } | Format-List` と記述し、PowerShellで空のパイプ要素として解釈されて失敗した。

## 発生した状況

- タスク: 複数CSVの先頭3行を比較
- 実行環境: PowerShell
- 前提条件: `foreach` 文から生成したオブジェクトを整形表示する

## 何を試したか

1. `foreach` 文の閉じ括弧直後へ `| Format-List` を接続した。

## 結果・エラー

```text
ParserError: An empty pipe element is not allowed.
```

## 原因

PowerShellの文としての `foreach` を、式として括らずパイプラインの左辺に置いた。

## 解決・回避策

`$(foreach (...) { ... }) | Format-List` のように、先に出力を式として確定してからパイプへ渡す。

## 今後の予防策

複数ファイルの調査では、`ForEach-Object` を使うか、`$(foreach (...) { ... })` と明示的に括る。

## 関連

- 関連ノート:
- 参考リンク:
