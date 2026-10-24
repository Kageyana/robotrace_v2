---
type: codex-failure
date: 2026-10-14
task: "通常72列復帰後ログ13228～13237の検証"
status: resolved
severity: low
tags:
  - codex/failure
---

# 整数相当メタデータの小数表記で解析が停止

## 要約

解析側が`optimalTrace=2.00`を`int()`で直接変換し、正常な実機CSVの検証を停止した。ログ・ファームウェアの異常ではない。

## 発生した状況

- タスク: 13228～13237の72列・診断統計・距離戻りの確認。
- 実行環境: Windows、Python、`analysis/script/check_restored_logs_13228_13237.py`。
- 前提条件: ファームウェアは`optimalTrace`を小数2桁で保存する。

## 何を試したか

1. DISTANCEの補正距離戻りを判定する際、`int(meta['optimalTrace']) == 2`を要求した。

## 結果・エラー

```text
ValueError: invalid literal for int() with base 10: '2.00'
```

## 原因

整数値という意味と、CSV上の整数文字列表記を混同した。既存解析は`float(meta['optimalTrace']) == 2`で確認していたが、新しい検証へ同じ保存形式を反映できていなかった。

## 解決・回避策

`meta_integer()`を追加し、floatへ変換後に有限・整数相当を確認してからintへ変換する。元CSVは変更しない。修正後の全10本検証は成功し、原本SHA256は解析前後で一致した。

## 今後の予防策

整数用途の保存メタデータも、追加パーサーの前に実ファイルの表記と既存の読込経路を確認する。端数を黙って切り捨てない。`2`/`2.00`の受理、`2.5`/`nan`/`inf`の拒否を確認済み。検証再実行は`python analysis/script/check_restored_logs_13228_13237.py`。

## 関連

- 解析証跡: `analysis/restored_logs_13228_13237/validation.json`
- 形式復帰: `doc/log-72-restored.md`
