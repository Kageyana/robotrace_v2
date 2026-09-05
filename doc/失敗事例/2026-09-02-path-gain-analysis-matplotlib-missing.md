---
date: 2026-09-02
status: open
task: PATH REPLAYのゲイン変更前後ログ解析
---

## 発生事象

ゲイン変更前後のログ集計は成功したが、XY軌跡・経路誤差・ヨーレートのグラフ生成スクリプトが `ModuleNotFoundError: No module named 'matplotlib'` で停止した。

## 原因または未確定事項

解析に使用したPython環境にmatplotlibがインストールされていない。

## 対応・次の調査

集計CSVを成果物として残し、グラフは依存関係を追加した環境で再実行する。

## 再発防止

グラフ生成前に、ワークスペース依存ランタイムでmatplotlibのimport確認を行い、不足時はテキスト集計へ切り替える。
