---
type: codex-failure
date: 2026-09-29
task: "空転負荷ベンチの6ケース再測定"
status: resolved
severity: medium
tags:
  - codex/failure
  - robotrace
  - st-link
  - gdb
---

# Debugベンチ一次走行の完了値を取得できずST-Link USB通信も失敗した

## 要約

一次走行の開始直前まではGDBで確認できたが、走行上限を過ぎても完了ブレークと結果値が返らなかった。GDBを終了して停止を試みた後、STM32CubeProgrammerはST-LinkのUSB通信エラーとなり、MCU停止状態を再確認できていない。

## 発生した状況

- タスク: 通常56 B・詳細109 Bの空転負荷ベンチ再測定
- 実行環境: STM32CubeCLT 1.19.0、STM32CubeProgrammer 2.20.0、ST-LINK GDB Server 7.11.0
- 前提条件: 当初はモーター電源接続済みと認識していたが、後から未接続と判明。今回の安全確認ではモーター電源を外した状態で実行した。

## 何を試したか

1. 100 kHz・Under Resetでハードウェアリセットし、開始前にPWM変数とTIM2左右比較レジスタが0であることを確認した。
2. モーター電源なしでPRIMARYを要求した診断では、エンコーダ値が異常速度になり、開始後1～2 msで停止理由`OVERSPEED`になった。この結果は負荷測定には使えない。
3. 停止後の完了ブレークへ到達しないままIWDGリセットしたため、ST-Link USBを抜き差しして接続を復旧した。
4. 修正版Debugを書き込み、開始直前でコアを止めてIWDGリセットを確認した。

## 結果・エラー

```
GDB: BENCH_PRESTART mode=1 wd=1 pr=6 rlr=255 pwmL=0 pwmR=0 ccrL=0 ccrR=0
STM32_Programmer_CLI: ST-LINK error (DEV_USB_COMM_ERR)
```

一次走行の有効なCSV検証結果とベンチ結果構造体は取得できていない。最新の開始直前安全確認ではIWDGリセット後の結果構造体を取得し、自動再開しないことを確認した。

## 原因

前回のUSB通信エラーは、ST-Link USBを挿し直した後に解消し、Device ID `0x421`を再取得できた。

モーター電源を外した状態での診断では、エンコーダ換算速度が異常値となり、開始後1～2 msで停止理由`OVERSPEED`になった。この条件では有効な負荷測定にならない。停止後の保存・CSV検査については、完了ブレークへ到達する前にIWDGリセットが発生したことを確認したが、単一FatFs呼び出しの停止か、連続した保存処理の猶予不足かは未確定。

Debugベンチの停止後保存処理では、アプリケーション層のFatFs呼び出し前後にIWDGを更新する変更を加えた。FatFs本体は変更していない。実機の有効な走行データでの再確認は未実施。

## 解決・回避策

ST-Link USBの抜き差し後、100 kHz・Under Reset接続でDevice ID `0x421`を取得し、Debugを書き込み・照合できた。開始直前ブレークでIWDGリセットを確認し、リセット後は結果状態が`RESET`、停止理由が`WATCHDOG_RESET`、リセット要因に`IWDGRSTF`が含まれた。要求モードは0、左右PWM変数とTIM2の左右比較値は0で、ベンチは自動再開しなかった。

保存処理の変更はDebug、DebugMarker、Releaseビルドとホストテストで確認済み。この記録を書いた時点では、実モーター電源を接続した6ケースは未実施だった。

## 今後の予防策

- 次の測定前にモーター電源を切り、ST-Link USBを抜き差しして `-l st-link` とDevice ID取得を確認する。
- 100 kHz・Under ResetのリセットとPWMゼロを確認してからGDBを接続する。
- 6ケースの負荷測定前にモーター電源を接続し、機体固定・車輪浮かせを確認する。電源OFFでのエンコーダ判定結果を負荷測定値に使わない。
- 一次走行は、完了ブレークが返るか、走行上限後にCSV確定が終わるまでの時間を記録し、結果構造体を取得できることを確認してから残りのケースへ進む。

## 追加調査 2026-09-29

モーター電源を接続した状態で再測定したところ、PRIMARYは`TRACE_END`で36,483 ms後に停止し、`endLog()`のCSV確定中にHardFaultした。停止時の`motorpwmL/R`とTIM2比較レジスタは0で、走行中の停止安全条件は満たしていた。CSVは保存・検証完了に至らず、この測定は無効。

GDBでは`endLog()`の2回目の`f_open("temp")`から`ff_memfree()`へ進んだ際に停止し、`HFSR=0x40000000`、`CFSR=0x8200`、`BFAR=0xF44F0236`、LFNポインタ`0x3f0`を確認した。FatFs設定は`_MAX_SS=4096`、SD SPIドライバの`GET_SECTOR_SIZE`は512を返す。Fault時の`FIL`は4,184 B、`endLog()`のスタックフレームは約7 KiBで、ヒープ終端`0x2001e938`とスタック下端約`0x2001e430`が重なっていた。これはヒープ／スタック衝突によるLFN管理領域破損と整合する。

対処としてFatFsの最大セクタサイズを512 Bに合わせ、`endLog()`のクロスライン配列を静的RAMへ移した。Debug、DebugMarker、Releaseは再ビルド成功し、RAM使用量はそれぞれ118,456 B、118,456 B、115,872 B。Debugの`endLog()`スタック割当は逆アセンブルで2,204 Bとなった。ホストテストはサブプロジェクトの`tests/run_pc_tests.ps1`で成功。

その後、IWDG開始前停止試験と、通常56 B・詳細109 BのPRIMARY／PATH REPLAY／SHORTCUT計6ケースを再測定した。全ケースでCSV行数・列数検証と`cntlog`単調性が成功し、走行中書込の512 B整列エラー、短い書込、SD書込失敗、バッファあふれ、1 ms周期超過、PATHロストはいずれも0。複数セクタ書込は最大4セクタでSDドライバへ到達し、全ケースの結果構造体が成功を示した。結果表は`analysis/debug-bench-results-2026-09-29.md`に保存した。今回のCSV終了処理は実機で完了したため、この失敗記録を`resolved`とする。

## 今後の予防策（追加）

- FatFsを構成するときは、`GET_SECTOR_SIZE`の戻り値と`_MAX_SS`を一致させる。
- ベンチを実走行する前に一次走行をCSV確定まで完了させ、完了構造体、CSV行数・列数、`cntlog`連続性を確認する。HardFaultまたは保存検証失敗時は残りのケースへ進まない。

## 関連

- 関連ノート: `2026-09-28-stlink-target-not-found-status-check.md`, `2026-09-29-debug-bench-pause-reset.md`
- 参考リンク: なし
