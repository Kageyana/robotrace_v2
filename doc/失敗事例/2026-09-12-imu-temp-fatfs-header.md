---
type: codex-failure
date: 2026-09-12
task: "BMI088温度補正・自然暖機計測のReleaseビルド"
status: resolved
severity: low
tags:
  - codex/failure
  - build
  - fatfs
  - imu
---

# 新規自然暖機計測モジュールでFatFs型の明示インクルードが不足した

## 要約

自然暖機CSV計測モジュールを追加した初回Releaseビルドで、`FIL`、`FRESULT`、`FA_CREATE_NEW`などのFatFs型・定数が未定義となった。

## 発生した状況

- `imu_temp_log.c` は`SDcard.h`をインクルードしていたが、FatFs型を直接使用していた。
- サンドボックス外の`cmake --build --preset Release -- -j1`でコンパイルした。

## 結果・エラー

- `unknown type name 'FIL'`
- `unknown type name 'FRESULT'`
- `FA_CREATE_NEW`、`FA_WRITE`、`FR_OK`などが未定義

## 原因

`SDcard.h`からの間接インクルードを前提にしており、新規モジュール自身がFatFs定義を明示的にインクルードしていなかった。

## 解決・回避策

`imu_temp_log.c`へ`#include "fatfs.h"`を追加した。その後、同じRelease configure/buildを再実行し、終了コード0とリンク完了を確認した。

## 今後の予防策

- 新規CモジュールでFatFsの`FIL`、`FRESULT`、`f_*` APIを使う場合は、間接依存に頼らず`fatfs.h`を直接インクルードする。
- 新規モジュール追加後は、最小ビルドではなく対象presetのReleaseリンクまで確認する。

## 確認方法

`cmake --preset Release`および`cmake --build --preset Release -- -j1`をサンドボックス外で実行し、終了コード0を確認した。
