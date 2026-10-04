---
type: codex-failure
date: 2026-10-04
task: CSV一括変換のRAM・スタック確認
status: open
severity: high
---

# CSV変換フレームより小さいスタック予約を発見した

## 確認した事実

使用中のSTM32F446XX_FLASH.ldは_Min_Stack_Size=0x400（1 KiB）だった。新しいDebugのlogConvertOne()はstmdbで24 Bとsub.wで2592 Bを使用し、下位のlogFormatSourceRow()はさらに532 B、logExistingCsvMatches()は632 Bを割り当てる。sysmem.cの_sbrk()はRAM末尾から_Min_Stack_Sizeを引いた場所までヒープを許可するため、実際のスタックが予約を超える状態を保護できない。

今回実機の衝突・HardFaultは確認していない。過去の2026-09-29-debug-bench-no-completion-usb-error.mdには実際のヒープ/スタック衝突が記録されている。

## 対処と再発防止

使用中リンカの予約を8 KiBへ変更する。CSV変換・解析にFIL、行バッファ、構造体を追加した場合は、全構成のリンク結果とobjdumpの関数プロローグを確認する。リンク成功だけで実機の最大使用量を保証しない。_sbrkの上限と予約領域をnmでも確認する。

## 確認方法・未完了事項

Debug/Release/DebugMarkerを再リンクしてRAMに収まり、_Min_Stack_Size=0x2000となることを確認する。実機の最悪ケースCSV変換・表示・割り込み併用でスタック最大使用量とヒープ終端を確認するまでopenとする。自動予約拡大のために未使用の生成リンカファイルは変更しない。

2026-10-04の再ビルドは全構成で成功した。RAM使用量（予約込み）はDebug 123168 B、Release 123064 B、DebugMarker 123216 Bで、131072 B以内。全ELFのnm出力で_Min_Stack_Size=0x2000、_estack=0x20020000を確認した。実測最大スタック使用量は未確認のため、ステータスはopenを維持する。
