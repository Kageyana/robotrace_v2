---
type: codex-failure
date: 2026-10-04
task: 全ログ分類パーサ実装
status: resolved
severity: medium
tags: [codex/failure, log-analysis, parser]
---

# 全CSV走査で短い行とNUL文字を想定せず停止した

## 確認した結果

事前調査スクリプトは列数を確認せず行を添字参照しIndexErrorで停止した。11035.csvの末尾は列定義より短い。最初の全件走査はPython 3.10 csv.readerの`line contains NUL`で停止した。入力側にNULが存在する。NULの発生原因は未調査。次の走査ではStringIOの既定改行処理で`new-line character seen in unquoted field`が発生した。

## 対処と再発防止

classify_logs.pyは列名を検出し、短い行・欠落・非数値を記録する。NULは読み取り時のみ置換して破損フラグを残し、そのログをABNORMALにする。StringIOにはnewline=Noneを指定して改行を正規化する。個別CSV解析例外は対象ログへ理由付きで記録し、全体走査を継続する。元CSVは変更しない。np.nanmaxは有限サンプルの存在を確認してから呼ぶ。

次回はtest_classify_logs.pyで旧/新形式、末尾空列、途中欠落、短い最終行、NULの回帰テストを先に実行する。全12555本の走査が完了し、11035がABNORMAL、移動計画から除外されたことを確認した。NULを含む3ログは入力を維持して解析でき、個別CSV解析例外で全体が止まる事例は残らなかった。

## 確認方法

`python analysis/script/test_classify_logs.py`

`python analysis/script/classify_logs.py --refresh`

classification.csvで11035がABNORMAL、move_plan.csvから除外されていることを確認する。
