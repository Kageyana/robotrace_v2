---
type: codex-failure
date: 2026-09-13
task: "最新ログ12439～12443の解析"
status: resolved
severity: low
tags:
  - codex/failure
  - log-analysis
  - path-analysis
---

# PATH解析へ一次走行と異なるモードを同時入力した

## 要約

`analyze_path_following.py`へ一次走行とPATH系ログを同時入力したため、モード制約で解析が停止した。PATH系でもLevel 0とLevel 1は同時比較できないため、各モードを個別に実行して完了した。

## 確認結果

- 12441、12442、12443の同時入力は`path mode only (3 or 4), got [0, 3, 4]`となった。
- 12442と12443の同時入力は`do not compare different modes: [3, 4]`となった。
- 12440、12442、12443をそれぞれ個別実行すると、経路復元、追従指標、XYグラフを正常に生成できた。

## 原因

解析開始前に`optimalTrace`で入力を分割せず、一次走行、`BOOST_PATH_REPLAY`、`BOOST_SHORTCUT`を一括指定した。ログ不良や復元失敗ではない。

## 対処と再発防止

- `analyze_path_following.py`へ渡す前にヘッダの`optimalTrace`を確認する。
- 一次走行は一次走行用解析へ分離する。
- PATHログは`optimalTrace=4`と`optimalTrace=3`を別コマンドで解析する。
- 個別解析で全対象の成果物生成を確認できたため`resolved`とする。
