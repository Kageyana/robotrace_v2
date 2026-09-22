---
type: codex-failure
date: 2026-08-16
task: "停止位置ずれ原因調査中のログ終端集計"
status: resolved
severity: low
tags:
  - codex/failure
  - analysis
---

# PowerShell経由Python一行スクリプトの引用符エラー

## 要約

停止位置調査のためにログ終端統計を算出する一行Pythonを実行した際、PowerShellの文字列展開とPython f-stringの引用符が衝突し、構文エラーになった。

## 発生した状況

- タスク: 周回終了時の慣性移動による停止位置ずれの原因調査
- 実行環境: PowerShellからpython -cを実行
- 前提条件: CSVログの最終マーカー周辺を一括集計

## 何を試したか

1. pathlibとf-stringを含むPython一行スクリプトをPowerShellのダブルクォート内で実行した。

## 結果・エラー

```text
SyntaxError: unterminated string literal
```

## 原因

PowerShell側の引用符エスケープとPython f-string内の辞書参照が干渉した。

## 解決・回避策

今回の調査では、個別ログの直接確認に切り替えた。

## 今後の予防策

複数行のPython集計は一行python -cにせず、既存解析スクリプトまたは一時スクリプトとして実行する。

## 再発記録

- 2026-09-22: 2行ヘッダーの実ログ読込確認で、PowerShellのダブルクォート内に辞書参照を含むPython f-stringを再び記述し、同じ`unterminated string literal`が発生した。PowerShellのシングルクォートヒアストリングへ複数行Pythonを格納して再実行し、ログ12590を2種類の読込処理で2923行・列数エラー0として確認した。
- 今後はPythonが複数の引用符、Windowsパス、f-stringまたは辞書参照を含む時点で、一行`python -c`を使わずヒアストリングを使用する。

## 関連

- 関連ノート:
- 参考リンク:
