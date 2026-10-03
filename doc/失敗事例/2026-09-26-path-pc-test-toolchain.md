---
type: codex-failure
date: 2026-09-26
task: "PATH REPLAY / SHORTCUT PC tests"
status: resolved
severity: low
tags:
  - codex/failure
---

# PATHのホストテストでSTM32定義不足とFatFsヘッダ競合

## 2026-10-03 ログXY単体テストのリンク依存

courseAnalysis.c全体をMinGWでリンクすると、--gc-sectionsを付けても未使用のFatFs等の未定義参照が残り失敗した。実装関数calcXYcie/clearXYcieと必要なグローバル宣言を、tests/extract_log_xy_source.pyで実ソースから抽出してテストする方式に変更した。計算式の複製はしない。run_log_xy_tests.ps1が警告をエラー扱いで成功し、6周閉路・初期化を検証できた。再発防止として、単体計算テストでは実装本体を抽出し、ファームウェア全体の依存をスタブで無理に埋めない。全体の整合はARMビルドで確認する。対処確認済み。

## 要約

MinGWでファームウェアの経路生成コードをPCテスト用にコンパイルした際、STM32ビルド定義不足とFatFsヘッダのマクロがホスト環境に持ち込まれ、テストコンパイルが失敗した。

## 発生した状況

- タスク: PATH REPLAY / SHORTCUTのPC回帰テスト追加
- 実行環境: Windows、MinGWホストコンパイラ
- 前提条件: STM32向け `pathFollower.c` をFatFsの代替実装とリンク

## 何を試したか

1. ホスト側でファームウェアソースとテストスタブをコンパイルした。
2. STM32のコンパイル定義を加え、FatFs公開ヘッダを避けるホスト用スタブを使った。

## 結果・エラー

```text
初回: STM32デバイス定義不足およびff.h由来マクロ競合でコンパイル失敗
修正後: PATH経路生成とポリシーのPCテストが通過
```

## 原因

MinGWコンパイルに `STM32F446xx` と `USE_HAL_DRIVER` を渡しておらず、さらにFatFsヘッダ内のマクロがホスト標準ヘッダと競合した。

## 解決・回避策

ホストテスト用コンパイルに `-DSTM32F446xx -DUSE_HAL_DRIVER` を追加し、FatFs関数をテスト内でスタブ化して公開ヘッダを含めない構成にした。

## 今後の予防策

- STM32ソースをホストコンパイルするテストでは、必要なデバイス定義をテストランナーのコンパイル引数に明記する。
- ホスト標準ライブラリとマクロ名が衝突する外部ヘッダはテスト対象へ直接持ち込まず、必要な関数をホストスタブで置き換える。
- CI相当の確認では `tests/run_pc_tests.ps1` を使う。

## 関連

- 関連ノート: なし
- 参考リンク: なし
