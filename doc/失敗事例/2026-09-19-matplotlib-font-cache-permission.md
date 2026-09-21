---
type: codex-failure
date: 2026-09-19
task: "一次走行12458～12466のログ比較図作成"
status: resolved
severity: low
tags:
  - codex/failure
---

# Matplotlibのユーザープロファイルへのフォントキャッシュ保存が拒否された

## 要約

解析図自体は保存できたが、Matplotlibが既定のユーザープロファイルへフォントキャッシュを保存する際に権限警告を出した。

## 発生した状況

- タスク: 一次走行ログの距離整列比較と図の出力。
- 実行環境: Windowsの制限付きワークスペース。
- 前提条件: `MPLCONFIGDIR`を指定せずにMatplotlibをインポートした。

## 何を試したか

1. `python analysis/script/analyze_primary_runs_12458_12466.py --log-dir <ログフォルダ>`を実行した。
2. インポート前に`MPLCONFIGDIR`をOSの一時領域へ設定して再実行した。

## 結果・エラー

初回は`Could not save font_manager cache [Errno 13] Permission denied`が出た。図とCSVは正常に出力された。再実行では警告なし、終了コード0で同じ成果物を出力した。

## 原因

既定のフォントキャッシュ保存先が制限付きワークスペースの書込み許可範囲外にあった。解析内容・ログデータの問題ではない。

## 解決・回避策

解析スクリプト内でMatplotlibをインポートする前に、`MPLCONFIGDIR`の既定値を一時領域に指定した。

## 今後の予防策

制限付き環境で新しいMatplotlib解析を追加する場合は、インポート前に書込み可能なキャッシュ先を設定し、同じコマンドを再実行して権限警告が出ないことを確認する。

## 関連

- 関連ノート: `analysis/script/analyze_primary_runs_12458_12466.py`
- 参考リンク: なし
