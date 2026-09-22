---
type: codex-failure
date: 2026-09-22
task: "2行ヘッダー専用ログ読込の合成CSV検証"
status: resolved
severity: low
tags:
  - codex/failure
---

# cntlog補修の合成ログを0 msから開始して検証が停止した

## 要約

`repair_cntlog_wrap.py`の2行ヘッダー確認用に作った合成CSVで、先頭データの`cntlog`を0 msとしたため、先頭区間の時間差が0 msとなり入力検証で停止した。

## 発生した状況

- タスク: Version 4ログの2行ヘッダー専用化レビュー
- 実行環境: Windows PowerShell、Python
- 前提条件: 実ログを使わず最小列の合成CSVで読込と補修を確認

## 何を試したか

1. 1行目をパラメータ、2行目を列名、3行目の`cntlog`を0とした合成CSVを作成した。
2. `repair_cntlog_wrap.repair_log()`へ入力した。

## 結果・エラー

```
ValueError: ...valid.csv:3: 不正な時間差です (0 ms)
```

## 原因

補修処理は先頭サンプルでも直前時刻0 msとの差を評価する。合成CSVの先頭`cntlog=0`では正の時間差にならず、仕様どおり不正入力として拒否された。

## 解決・回避策

先頭データを実ログと同様の正の時刻から開始し、後続行も単調増加させて再検証する。

先頭を5 ms、次行を10 msへ修正した合成CSVで、2行ヘッダー読込、補修後CSVの再読込、旧1行ヘッダーの拒否を確認し、検証は成功した。

## 今後の予防策

`repair_cntlog_wrap.py`の合成テストでは、先頭`cntlog`をログ周期以上の正値にし、全データ行で時間差が正になることを事前確認する。

## 関連

- 関連ノート: `2026-09-06-log-cntlog-u16-wrap-derived-xy.md`
- 参考リンク: なし
