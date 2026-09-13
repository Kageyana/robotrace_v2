---
type: codex-failure
date: 2026-09-13
task: "走行ログCSVの2行ヘッダー統一"
status: resolved
severity: low
tags:
  - codex/failure
  - test
  - python
  - sandbox
---

# Python標準一時ディレクトリがサンドボックスの書込範囲外だった

## 要約

CSV変換テストとメモリ上の変換結果検査で`tempfile`の既定保存先を使ったため、Windowsユーザー一時ディレクトリへの書込みが拒否され、テスト5件が処理開始前に失敗した。

## 発生した状況

- タスク: 数値名CSVログのヘッダー変換と回帰テスト
- 実行環境: Windows PowerShell / Codexデスクトップのworkspace-writeサンドボックス
- 前提条件: リポジトリと明示された一時領域だけが書込み可能

## 何を試したか

1. `tempfile.TemporaryDirectory()`でテストCSVを作成した。
2. 変換後バイト列を`tempfile.mkstemp()`へ保存して再検査しようとした。

## 結果・エラー

```text
PermissionError: [Errno 13] Permission denied: C:\Users\ucawa\AppData\Local\Temp\...\1.csv
```

## 原因

Pythonが選んだ既定一時ディレクトリが、このタスクの書込み許可ルート外だった。CSV変換ロジックではなく、テストデータの保存先選択が原因である。

## 解決・回避策

- 変換結果の検査はファイルへ書かず、バイト列を直接解析する処理へ変更した。
- ファイル作成が必要な単体テストは、`TemporaryDirectory(dir=Path.cwd())`でリポジトリ内へ限定した。

## 今後の予防策

- サンドボックス内のPythonテストでは、`tempfile`へ書込可能な`dir`を明示する。
- メモリ上で検証できるデータは一時ファイルへ保存しない。
- 新しい一時ファイル処理は、書込み先がworkspace rootまたは明示された一時領域内であることを事前確認する。

## 確認

リポジトリ内一時領域を使って単体テスト6件を再実行し、全件成功を確認した。変換結果の再検査は一時ファイルを使わずに完了した。

## 関連

- `analysis/script/normalize_log_headers.py`
- `analysis/script/test_log_header_formats.py`
