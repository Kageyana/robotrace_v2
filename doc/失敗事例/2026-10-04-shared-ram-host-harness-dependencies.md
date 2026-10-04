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

## 2026-10-04 再発: SSD1306非同期DMAのホストテスト

CSV進捗表示の非同期化を検証する際、MinGWは未使用のssd1306_InitからのHAL_Delay参照も要求し、リンクが失敗した。未使用関数の除去だけに依存する対策では再発を防げなかった。HAL_Delayのモックを追加し、呼ばれた場合は検証失敗にすることで、非同期送信経路にdelayが入らないことも確認する。assertは標準エラーへ出力してexit(1)するマクロに置換した。

追加対策: 実装ソース全体を含むMinGWハーネスでは、リンク時に要求されるHAL関数を明示的に模擬し、検証対象で禁止する同期処理は呼出回数0または即失敗で確認する。run_ssd1306_dma_tests.ps1で単発DMAのコマンド→全画面送信、転送中のバッファ保持、開始失敗、他I2Cの完了無視、連続更新への復帰が成功した。対処・検証済み、resolved。

## 2026-10-04 再発: ラインセンサー校正のホストテスト

lineSensor.c全体のPCリンクで、未使用のSD処理とTIM3/ADC変数も未定義参照になった。セクション除去に依存したため、既存の再発防止策を初回ハーネスへ適用できなかった。対処として実ヘッダと型を合わせたTIM3/ADC変数とFatFs関数モックを追加した。SD関数が呼ばれたらstderr出力後exit(1)とし、校正完了処理のSD非依存も検証する。今後は実装全体をリンクするハーネスの作成前に、同一ソース中の外部変数と外部関数を洗い出し、未使用経路を含めてモックを用意する。run_line_calibration_tests.ps1成功によりハーネスの対処はresolved。
