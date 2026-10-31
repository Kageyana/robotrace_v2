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

## 2026-10-04 解析環境の再確認

13005高音解析でバンドルPythonにはSciPy・Matplotlibがなくimport失敗が再発した。今回は通常Pythonの実体がPython310でありNumPy・Matplotlibのimportが成功、同環境で音声FFTとグラフを生成できた。追加対策: バンドルに描画ライブラリがない場合、既存の通常Pythonも必要importで検証し、使える環境を選ぶ。SciPyが必要な処理はNumPy FFTへ置換できるか確認する。


## 2026-10-30：追加解析器が既存の動画依存配置を読まなかった

13924の時刻同期器はPython310へSciPyがあると仮定してModuleNotFoundErrorで停止した。同期器はNumPyの粗密グリッド探索へ置換し、実入力で時刻差2.675535秒・角速度相関0.998607を確認した。その後、白線比較器でcv2を先にimportして再び停止。既存カメラ補助はbuild/video-analysis-runtimeをsys.pathへ加えてSciPy/OpenCVを使っていた。

新比較器も最初に同じ依存先を明示し、importlib.util.find_specでnumpy/cv2/scipyを確認してからimportする。追加インストールはしていない。再発防止はPython実体だけでなく既存補助のsys.path設定と必要モジュールの実配置を実行前に読むこと。確認はanalysis/script/check_marked_071234_map.pyの起動・6314骨格点の投影・剛体比較完了。status: resolved（今回のランタイム解決）。
