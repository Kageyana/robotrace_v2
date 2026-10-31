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
# 2026-10-27：完了監査で整数の文字列表記を再び仮定

実走13843～13846の完了監査へ追加した条件がalpha=='100'、emc=='7'を要求し、小数表記の正しいメタデータを拒否した。整数の意味と保存書式を分ける既存対策が、新規assertへ適用されていなかった。数値へ変換して100/7と比較すると原本・全32フレーム・版と設定の監査が成功した。製品の保存書式は変更していない。

再発防止としてログ解析Skillの検証追加前へ、型付き変換と数値判定・書式判定の分離を明記する。analysis/script/audit_path_fusion_completion.pyを実行しlatest13846、候補4本、goal_complete=falseを確認した。表記判定の対処はresolved、全4本が途中停止であるため実走目標は未達のまま。

## 2026-10-28：一次番号の集計型を仮定

13879～13881の集計監査で、run_diagnosticsのprimaryが文字列なのに整数タプルと直接比較し、正しい系列を拒否した。既存の型付き比較を新規監査へ適用できなかった再発。finiteかつ整数相当を検査するinteger()へ統一し、mode/primary/emc・取得版/優先度を数値比較した。全CRC・全項目RAM照合・動画・98再生を含むfinalize_path_acquisition_220117.pyが終了コード0で成功した。保存書式と原本は無変更。status: resolved（監査の型）。

## 2026-10-31：版比較でも整数メタデータを直接intへ渡した

版20/21/22の比較入口でoptimalTraceの保存表記5.00をintへ直接渡し、ValueErrorで停止した。既存の型付き変換対策を、新規比較のassertへ適用しなかった再発。CSV、機体、完成済みの過去解析は未変更。失敗入口はanalysis/path_versions_20_21_22_20261031/before-metadata-integer-fixへ保存した。

compare_controller_20_21_22.pyのmetadata_integerへ全ての整数メタデータ条件を統一し、有限・整数相当を検査してからintへ変換した。版/一次番号/走行方式/停止理由/推定器/FF版を同じ入口で比較する。5/5.00受入、5.1/nan/±inf拒否の6境界と、全21原本ログの所属・SHA一致を確認した。検証は同系列validation.json、比較結果はcomparison.json。

再発防止は新規比較の開始前にこの型付きヘルパーと6境界を実行し、meta文字列への裸のintを追加しないこと。ログ解析Skillの実行前へ具体的な入口を追加した。status: resolved（整数表記の比較契約）。版22の減速改善は別課題で未実装。
