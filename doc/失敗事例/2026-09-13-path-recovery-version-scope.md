---
type: codex-failure
date: 2026-09-13
task: "2行ヘッダー変換後のPATHログ復元確認"
status: resolved
severity: medium
tags:
  - codex/failure
  - robotrace/log-analysis
  - python
  - regression
---

# PATH復元メッセージの経路バージョン変数が未定義だった

## 要約

変換済みPATHログ12385の復元を実行すると、経路生成とヘッダー整合確認後の結果作成で`route_version`が未定義となり停止した。2行ヘッダー解析ではなく、既存のVersion 13対応変更における変数スコープが原因だった。

## 発生した状況

- タスク: 全ログ変換後の一次・二次・PATH復元互換確認
- 対象: `analysis/script/path_log_recovery.py`
- 実ログ: 12385、解析元12384

## 何を試したか

1. 全233本を新2行ヘッダーとして共通読取関数で読み込んだ。
2. PATH走行ログを抽出し、12385で`recover_path_columns()`を実行した。

## 結果・エラー

```text
NameError: name 'route_version' is not defined
```

## 原因

`route_version`が保存済み列用の`_saved_values()`内で取得されていた一方、実際に参照する再生成経路の`recover_path_columns()`内には定義されていなかった。

## 解決・回避策

経路点数、CRC、対応バージョン、生成結果の整合確認を通過した後に、`recover_path_columns()`内で`routeControllerVersion`を取得するよう修正した。不要だった`_saved_values()`内の取得は削除した。

## 今後の予防策

- 復元対応バージョンを追加した際は、保存済み・全行復元・部分欠測・全行欠測の全経路を実行する。
- 実ログを使い、`Recovery.reason`と各行の`recovery_reason`生成まで確認する。
- 構文検査だけで復元機能の完了判定をしない。

## 確認

変換済み実ログ12385で復元を再実行し、`status=restored`、元ログ12384、3574行、欠測0件を確認した。経路復元の既存回帰テスト8件も成功した。

## 関連

- `analysis/script/path_log_recovery.py`
- `doc/失敗事例/2026-09-12-log-recovery-review-fixes.md`
