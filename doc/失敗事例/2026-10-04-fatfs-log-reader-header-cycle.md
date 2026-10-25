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

## 2026-10-15 実FatFsのホスト検証で再発

新しい実FatFs/exFAT RAMディスク検証でff.c・ccsbcs.cを直接コンパイルすると、各翻訳単位のff.hが先にWindowsのLPTR/ERRORを取り込んだため同じ競合が再発した。テスト本体だけmain.hを先に読む対策では不十分だった。runnerへ`-include Core/Inc/main.h`（実際は絶対パス）を追加し、全翻訳単位でCMSISを先に評価するよう変更した。製品のFatFs・CMSISヘッダは変更しない。実FatFs/exFATで旧f_putsとのバイト一致、CRLF境界、事前確保・切詰め、複数セクター書込が成功した。初回コンパイル前に全翻訳単位のinclude順序を確定することを追加対策とする。

## 2026-10-25 フラグ行検証の単独ヘッダ読込で再発

対象はスリップログの有効性・無効理由・区間統計削除とSLIPフラグ集計への変更。製品の4構成ARMビルドと通常/詳細の保存・計画・PC変換は成功したが、追加した`validation/log_storage/flag_policy.c`の初回コンパイルは`logSource.h`からFatFs→Windowsを先に読み、LPTR/ERROR衝突と_T/_TEXT再定義で停止した。製品のビルドエラーではなく、単独ホストハーネスの依存関係が原因。従来の予防手順は実FatFsの翻訳単位を中心としており、純粋な検証関数で公開ヘッダだけを読むケースを事前確認できなかった。

検証対象はファイルAPIを必要としないため、runnerが実`logSource.h`の`LogSourceRow`定義と実`logSource.c`の`logSourceValidateSlipRow`だけを抽出する構成へ修正した。型の開始・終端が各1件であることを生成前に検査する。製品ヘッダや警告設定を変更せず、式や構造体のテスト用再実装も行わない。

再発防止として、公開ヘッダを使う単独ハーネスでもFatFs/Windows/CMSISへの推移依存を初回コンパイル前に調べる。純粋な行検証は実型・実関数だけを抽出し、ファイルAPIが必要な統合試験とは分ける。書込み・デバッグSkillの実行前参照へこの分岐と実行コマンドを追加した。

確認はリポジトリルート・必要権限で`python validation/log_storage/run.py --converter C:/tool/AutoCopyCsvFromSd/RlogToCsv.exe`を再実行しexit=0。O0/O2・Wall/Wextra/Werrorで全8フラグ、未定義bit、負値、非有限・範囲外行を検証した。通常50列/135 B・詳細56列/144 Bの全CRC、PC変換50行の全数値一致、SLIP増減速・Yaw除外・未観測区間の増速抑制も成功。このホスト依存の対処はresolved。実機走行と割り込み最大時間は未検証。
