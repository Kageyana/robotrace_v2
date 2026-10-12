---
type: codex-failure
date: 2026-10-12
task: "ST-LinkでSD内bin・partの起動時復旧失敗を調査"
status: resolved
severity: high
tags:
  - codex/failure
---

# SD容量不足で起動時CSV復旧が失敗する

## 要約

実機の起動時復旧が13100.binのCSV書き込み途中で失敗した。FatFsの空きクラスタ数0と、f_putsの戻り値-1を同時に取得し、容量不足を確認した。容量確保後に別のexFAT初期化不良を修正し、4本の復旧と再起動後の正常状態を確認したためresolved。

## 発生した状況

- STM32F446（Device ID 0x421）、CubeCLT 1.19.0、ST-LINK GDB Server 7.11.0。
- 現在のFlash先頭248600 Bとbuild/Debug/robotrace_v2.elfのバイナリは全バイト一致。ファーム書き込み・コード変更・実走行は行っていない。
- 起動後のconversionProgressはlogNumber=13100、fileIndex=1、fileCount=4、processedRows=1480、expectedRows=4684、totalPercent=16、FAILED。

## 確認方法と結果

1. リセットなしのattachで状態を取得し、Flashを読み出してELFと照合した。
2. Programmer CLIでリセット・停止後にGDBへ接続したが、接続までに起動時復旧を通過した。GDBサーバーのmonitor helpでreset対応を確認し、ブレーク設定後にmonitor resetを実行して再現した。monitor reset haltは使っていない。
3. 照合済みELFのlogConvertOne+746（cleanup入口）一点にハードウェアブレークを置いた。
4. main→initSystem→finishLogConversions→logConvertPending→logConvertOneの起動経路で停止した。

取得値:

```text
number=13100, inputOpen=true, outputOpen=true, success=false
r0=r3=0xffffffff (直前のf_putsの戻り値-1)
binarySource.row=1481, error=FR_OK
header: profile=1, recordBytes=157, rows=4684, metadataBytes=2181
input.err=0, input.fptr=233029
output.err=0, output.fptr=output.obj.objsize=522240
fs.free_clst=0, fs.n_fatent=16328, fs.csize=4
card_initialized=1, hspi3.ErrorCode=0
motorpwmL=motorpwmR=0
```

## 原因と未確定事項

- FAT16ボリュームのデータ容量は(16328-2)×4×512=33,435,648 B。書込時点の空きは0 B。
- 入力のヘッダ・データCRC・メタデータCRC検証を完了し、CSV整形後のf_putsで失敗している。今回はスキーマ不一致や入力CRC不正が失敗原因ではない。
- FatFsのf_writeはディスク満杯時にFR_OKのまま短い書込となるため、output.err=0でも容量不足を否定できない。f_putsは短い書込を-1として返す。
- 今回の失敗時点でSPIエラーは0。媒体全体の健全性を保証するものではない。
- 前日の13085～13088とは別のログであり、前日の根本原因まで今回の結果から断定しない。

## 対処・次の確認

- 元binの削除、CSVの手動削除、フォーマットは行っていない。partは通常の起動時復旧経路で再作成された。
- 停止後のクローズ処理を再開し、ブレークを削除してdetachした。
- 次は電源断後にSDをPCへ接続し、全bin・part・設定・既存CSVを保全する。不要ログを特定して退避し、空きを確保した後に機体で起動時復旧を確認する。
- CSV行数・CRCメタデータ・正式CSV確定後のbin削除とlogRecoveryReady=trueを確認できた場合だけresolvedへ変更する。

## 再発防止

1. 走行前またはPCへログ回収するときに、カードの実ボリューム容量と空き容量を確認する。カード表記容量だけで判断しない。
2. 未変換binを残したままCSVを生成する分の空きを確保する。容量不足を解消するために元binを先に削除しない。
3. 書込失敗の診断ではFatFs結果だけでなく、要求長・実書込長・fptr・free_clstを同時に確認する。
4. 起動経路のブレークは接続後に設定し、対応を確認したmonitor resetで再現する。CLIのhaltが接続まで維持される前提を置かない。
5. この制限環境ではCubeCLT実行ファイルがシステムDLL再配置エラーとなる場合がある。今回もGDBサーバー起動で発生し、通常環境の承認付き実行で解消した。コード不良と混同しない。

## 関連

- 追加確認: ユーザーの容量確保後はexFAT、free_clst=487890で空きが十分だったが、f_openの末尾断片カウンタ未初期化でメタデータseekが失敗した。別事例2026-10-12-fatfs-exfat-uninitialized-fragment.mdで修正し、13100～13103のCSV確定、DONE 100%、再起動後fileCount=0、logRecoveryReady=trueを実機確認した。前回の容量不足の事実を今回へそのまま当てはめず、別原因を切り分けた。

- 前日事例: 2026-10-11-sd-pending-csv-recovery.md
- 診断記録: analysis/sd-debug-20261012/repro.txt、inspect.txt、repro.gdb、failure-ram.bin、device-flash.bin（ローカル解析成果物）
