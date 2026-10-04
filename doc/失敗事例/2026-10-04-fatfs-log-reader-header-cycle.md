---
type: codex-failure
date: 2026-10-04
task: バイナリログ共通読込APIの追加
status: resolved
severity: low
---

# FatFsの型をmain経由のヘッダへ追加して循環依存した

## 確認した事実

SDcard.hへFIL/FRESULTを使うAPIを追加すると、ff.h → ffconf.h → main.h → SDcard.hの順で、FatFs型定義前にAPI宣言を評価し、Debugビルドがunknown type nameで失敗した。また共通ヘッダからff.hを先に読み込むと、MinGWのWindowsヘッダのLPTR/ERRORマクロがCMSIS型と衝突し、既存PCテストも失敗した。

## 原因と対処

FatFs設定がmain.hを取り込むため、main.hに含まれるSDcard.hでFatFs型を公開できなかった。型付き読込APIをlogSource.hへ分離し、main.hを先に読み、その後にff.hを読む順序にした。SDcard.hにはFatFs型に依存しない保存・進捗APIだけを置いた。

## 再発防止と確認

FatFs型のAPIはmain.hから直接取り込まれるヘッダへ追加しない。読込APIの変更後はARM Debugビルドとtests/run_pc_tests.ps1を両方実行し、Windows/MCUの両方のinclude順序を確認する。修正後はDebugビルドと既存PCテスト、新規通常・詳細ログテストが成功したためresolved。実機I/Oの検証を意味しない。
