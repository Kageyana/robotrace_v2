# RAMとログレコードのバイト順を混同

- 対象: 実1ms記録13926と同時刻の保存binを直接比較する解析。
- 事実: RAMとRLOGヘッダと同じlittle endianをレコードにも使い、最初の比較がtick2560で不一致となった。誤った読出しでは9行だけが偶然tick範囲に入り、gyro/横残差/パルスが全て不一致だった。CSV再変換・全CRCは既に正常で、機体不具合の証拠として扱っていない。
- 原因: 実serializerのlogBufferWriterAppendU16/U32を読まず、CPUネイティブ順を保存形式へ流用した。レコードは上位バイトから送るbig endian、RLOGヘッダとネイティブRAMはlittle endian。
- 対処: SDcard.cの実シフト順と先頭cntlog=5の00 05を確認し、レコードだけ>H/>Iで読んだ。誤解析JSONと旧解析ソースはbefore-record-byteorder-fix等として保全。
- 再発防止: ヘッダ・レコード・RAMを別の契約としてserializer/実バイト/型を入口で確認する。Byte swapを結果に合わせて推測しない。修正版は実writerのシフト順と先頭時刻をassertし、schemaの保存列だけで154Bを計算する。
- 検証: 2534～3045tick内の237保存レコードでgyro/正式横残差のF32全ビットと補正パルスがRAMへ完全一致。全CRC/CSV再変換/56列は維持。実機取得・書込みは繰り返していない。
- ステータス: resolved（保存形式の読出し）。open（位置ずれの物理原因・補正実走行）。
