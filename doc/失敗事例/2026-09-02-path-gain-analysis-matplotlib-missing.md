---
date: 2026-09-02
status: resolved
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

## 2026-09-09 再発

- ログ12363～12367の解析で、既定Python 3.10と`load_workspace_dependencies`が返した同梱Pythonの双方にmatplotlibがなく、描画が失敗した。
- 既定Pythonで描画前の依存確認を省略していたため、既存対策を適用できなかった。同梱ランタイムも描画可能とは限らない。
- 標準ライブラリによる集計を完了し、`analysis/12363_12367/`へ経路追従CSVと追加確認JSONを保存した。グラフは未生成。
- 次回は選択したPython実行ファイルで`-c "import matplotlib"`を実行してから描画処理を開始し、不足なら集計結果と描画未実施を明示する。描画依存を整備してグラフ生成を検証するまでopenを維持する。

## 2026-09-09 解決確認

- ユーザーによるインストール後、既定Python 3.10でmatplotlib 3.10.9のimportに成功した。
- `analysis/script/plot_short_course_12363_12367.py`でXYと追従グラフを生成し、画像を開いて表示を確認した。
- 初回描画時にユーザーフォルダのフォントキャッシュ保存が権限不足となった。スクリプトで未指定時の`MPLCONFIGDIR`を一時フォルダ内の`robotrace-matplotlib`へ設定し、再実行で警告なく生成できることを確認した。
- 描画依存不足とキャッシュ保存の回避を検証できたためresolvedとする。同梱Pythonにインストールされたことは確認していないため、再実行には確認済みの既定Pythonを使用する。

## 2026-09-09 既存解析スクリプトのGUI依存

- 12369～12372を`analyze_path_following.py`で描画すると、既定Tkバックエンドが`Can't find a usable init.tcl`で停止した。CSV集計は先に保存されていた。
- 前回のキャッシュ対策は個別プロットスクリプトのみで、既存の`write_xy_plot()`には未反映だった。
- `write_xy_plot()`でも一時フォルダへのキャッシュ保存と`Agg`バックエンドを設定した。ファイル保存専用のためGUIは不要。
- 同じ4ログのコマンドを再実行し、終了コード0、CSV・PNGの生成、画像の表示を確認した。resolvedを維持する。

## 2026-09-13 再発

- ログ12384～12394の一次走行解析で、`load_workspace_dependencies`が返した同梱Pythonを依存確認なしで使用し、`ModuleNotFoundError: No module named 'matplotlib'`が再発した。
- 既存の再発防止策どおり、通常Pythonの`py -3 -c "import matplotlib"`でmatplotlib 3.11.1を確認し、同じ解析スクリプトを再実行してCSVとPNGを生成した。
- 原因と回避策は既知であり、通常Pythonで成果物生成まで検証できたため`resolved`を維持する。
- 次回は同梱Pythonを優先する前提にせず、描画を伴う解析の最初に実際に使うPythonごとのimport確認を行う。
