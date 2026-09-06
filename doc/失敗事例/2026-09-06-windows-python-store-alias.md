---
type: codex-failure
date: 2026-09-06
task: "PATH REPLAY解析スクリプトの回帰実行"
status: resolved
severity: low
tags:
  - codex/failure
  - test
  - python
  - windows
---

# Windows StoreエイリアスをPython実体として実行した

## 要約

解析スクリプトの構文確認で`python`を実行したところ、実体ではなくMicrosoft Storeのアプリ実行エイリアスが呼ばれ、解析を開始できなかった。

## 発生した状況

- 対象: `analysis/script/analyze_path_following.py`
- 実行環境: Windows PowerShell / Codexデスクトップ

## 結果・エラー

```text
Python was not found; run without arguments to install from the Microsoft Store,
or disable this shortcut from Settings > Apps > Advanced app settings > App execution aliases.
```

## 原因

`Get-Command python`が`C:\Users\ucawa\AppData\Local\Microsoft\WindowsApps\python.exe`を返し、Pythonランタイム本体がPATH上に存在しなかった。

## 解決・回避策

Codexデスクトップのworkspace dependency情報から、同梱Pythonの絶対パスを取得して構文確認とログ解析を再実行した。

## 今後の予防策

- WindowsでPython処理を始める前に`Get-Command python`の`Source`を確認する。
- Sourceが`Microsoft\WindowsApps`の場合は実行せず、workspace dependency情報に示されたPython実体を使用する。
- 代替Pythonで`-m py_compile`と対象解析を実行し、終了コード0を確認する。

## 確認

同梱Pythonで構文確認に成功し、ログ12290、12292、12294からindexジャンプ指標を出力できた。
