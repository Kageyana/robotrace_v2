---
type: codex-failure
date: 2026-10-04
task: DISTANCE・SLIP・PATHのRAM共有化とPC回帰検証
status: resolved
severity: low
tags:
  - codex/failure
---

# RAM共通化後のPCテスト依存と模擬ログを修正した

## 要約・発生状況

Windows MinGWで新しい回帰テストを追加し、既存バイナリログテストも再実行した。ファームウェアの全構成ビルドは成功したが、PC検証側の依存・モックに不備があった。

## 確認した結果と原因

- `sd_diskio_spi.h`の直接読込でDiskio_drvTypeDefが未定義。先にfatfs.hを読む必要があった。
- 模擬f_lseekの引数をDWORDにしてff.hのFSIZE_tと不一致。encMMもヘッダのint16_t宣言との不一致を修正した。
- MinGWは未使用関数にも未解決参照を報告した。新しい比較テストでは実際の計画関数を抽出して不要なMCU/UI依存を除いた。
- 模擬CSVに距離解析の必須列が足りず、SLIP最終indexも距離計画要素数を超えていた。列名を実際のパーサーに合わせ、SLIPを計画範囲内に収めた。
- MinGWのassertがダイアログ待ちになった。テスト固有の検証マクロでstderr出力後exit(1)するようにした。
- 既存run_log_deferred_tests.ps1のリンク対象に新規runMemory.cがなく、共有変数・関数が未定義だった。実装ソースをリンク対象へ追加した。

以上はPC検証用ハーネスの問題であり、実機でRAM破壊や走行異常を確認した事例ではない。

## 対処・再発防止

モックの型は対象ヘッダを参照して一致させる。FatFsドライバ宣言は必要な生成ヘッダを含めてから使用する。新規モジュール導入時はCMakeだけでなくローカルtestsの全リンク対象を検索する。CSVの必須列・計画要素数・optimalIndexの依存条件も事前確認する。Windowsの自動テストでは失敗時にGUIダイアログを待たない検証処理とする。

## 確認とステータス

run_distance_slip_memory_tests.pyで変更前後のDISTANCE/SLIP計画一致、run_pc_tests.ps1でPATH切替、run_log_deferred_tests.ps1で通常・詳細の保存/復旧と全方式の読込結果一致を確認した。ハーネス修正を同じ作業で反映済みのためresolved。testsは既存方針によりGit管理対象外。
