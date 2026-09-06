---
date: 2026-09-02
status: open
task: PATH REPLAYのゲイン変更前後ログ解析
---

## 発生事象

ゲイン変更前後のログ集計は成功したが、XY軌跡・経路誤差・ヨーレートのグラフ生成スクリプトが `ModuleNotFoundError: No module named 'matplotlib'` で停止した。

2026-09-06の再解析では、PATH上の`python`がMicrosoft Storeエイリアスのみで実体がなく、スクリプトを起動できなかった。

## 原因または未確定事項

解析に使用したPython環境にmatplotlibがインストールされていない。

また、このPCでは通常の`python`コマンドを解析ランタイムとして使用できない。Codexのワークスペース依存ランタイムにはPython実体があるが、matplotlibは含まれていない。

## 対応・次の調査

集計CSVを成果物として残し、グラフは依存関係を追加した環境で再実行する。

数値集計は`load_workspace_dependencies`で取得したバンドルPythonの絶対パスから既存スクリプトを実行し、成功した。グラフ生成はmatplotlib不足時にスキップする既存動作を使用した。

## 再発防止

グラフ生成前に、ワークスペース依存ランタイムでmatplotlibのimport確認を行い、不足時はテキスト集計へ切り替える。

`python`コマンドが見つからない場合はMicrosoft Storeの導入案内に進まず、先にワークスペース依存ランタイムのPython実体を確認する。
