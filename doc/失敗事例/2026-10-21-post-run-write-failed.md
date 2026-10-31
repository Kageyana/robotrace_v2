---
status: open
task: 走行後のWrite failed調査
---

# 停止後のWrite failed表示をST-Linkで調査

2026-10-27の再発13854はゼロヘッダtmpの原本保持・境界を除いた部分抽出まで確認した。保存失敗の当時のRAM状態は未採取であり、本件の原因未確定を維持する。詳細は[未確定tmpの再発](2026-10-18-unsealed-tmp-partial-recovery.md)とanalysis/path_fusion_13853_13861/report.md。関連関数を探す際にlogBufferWriter.c/hを仮定して読取が失敗したが、現行の型・実装はSDcard.c内にある。次回は実在Core/Srcディレクトリへの`rg -n 'logBufferWriterBeginRecord|logBufferWriterWriteSucceeded'`から定義位置を確定して読む。

2026-10-21、ユーザーは走行後にWrite failedが出ると報告。調査開始前に機体を再起動済みだったため、走行時のRAMフラグは失われていた。

## 確認済み

- リセットなしattachでDevice ID 0x421。Flash読出しと既存Debug ELFの246872 Bが全バイト一致。
- 初回RAMはlogFileNumber=0、cntSend=0、logBufferWriter全ゼロ、modeLOG=false。左右PWM=0。SD初期化成功。exFAT、csize=256、free_clst=487876。満杯の兆候なし。初回SPI ErrorCode=0。
- ルートの13567.tmpと13568.tmpは各2097152 Bで、先頭64 Bはゼロ。元ファイルは読取のみ。
- 一時診断版を別build/SDWriteDiagnosisにビルドし、モーターを動かさずメイン処理から通常APIを実行。/sddiag21/以下に32/700/720/6000/12000件の合成ログを生成。既存endLogのキュー排出、末尾書込、truncate、全件読込、CRC計算、メタデータ・ヘッダ保存、sync/close/renameは全5条件で成功。2 MiBを超えるログも成功。一次経路検証は合成データなのでreason=2であり、走行成功の証拠にはしない。
- 診断中も左右PWM=0、CFSR/HFSR=0。合成ログは通常走行ログとは別フォルダに保持。

## 現時点の原因と未確定事項

Write failed表示はendLogのfalseを表示している。実f_writeエラーだけでなく、走行中のlogBufferWriter.writeFailed、logOverflow、markerOverflow、FatFs確定処理の各失敗でもfalseになる。再起動後の状態から当時の失敗フラグは判定できない。

2 MiBでゼロヘッダのtmpは確定未完了を示すが、容量不足、バッファあふれ、SPI異常等のどれかを単独では確定できない。停止中の保存5条件成功は実走行中のSD遅延やバッファ耐性を保証しない。

## 次の確認・再発防止

- 次の失敗は電源再投入・リセットをせず、停止したままST-Linkを接続してRAMを回収する。ST-Link接続中に実走行しない。
- logBufferWriter、logOverflow、markerOverflow、dbg_overflow、fil_W.err/fptr/objsize、cntSend、ログ番号を同時回収。必要なら走行中f_writeのFatFs戻り値・要求/実書込長・最大時間をRAMへ保存する。
- 一時診断はメインの通常実行経路からAPIを呼ぶ。停止中GDBからブロッキングHAL/FatFs関数を直接呼ばない。
- GDBからFILのfptr/clustや読取ポインタを変更するサンプル取得案は自動承認レビューで拒否され、未実行。通常FA_READ/f_lseek/f_readの診断処理へ切替。
- 元ELFとソースを保全し、診断後は復元・Flash照合・初期化/PWM確認を行う。

証拠: analysis/sd-write-failed-20261021/。原因未確定のためopen。

## 未確定tmpの追加読取

通常メイン処理のFA_READ/f_lseek/logReadBufferedで13567.tmp/13568.tmpを全域走査し、API結果FR_OK・SPI ErrorCode=0を確認した。2 MiBの事前確保域には末端まで非ゼロの旧内容が残っており、実ファイルサイズや非ゼロ末尾から今回の記録件数は決められない。簡易なcntlog/左右累積値の単調性検査では13567の4757件目付近、13568の526件目付近で不整合が出たが、前者にはASCIIの列定義を数値として読んだ異常値も含まれた。ヘッダ/CRC/期待件数がないため、この件数を有効ログ件数とは扱わない。13568では1585 ms時点の走行データが残っていたが、以後の内容との区別と停止原因は未確定。未確定tmpを正式binへ昇格していない。

境界をさらに観測する条件付きブレークは頻回ヒットによる遅延が大きく、中断した。既知PIDの自分のGDB/サーバーだけを終了。元ファーム復元時にProgrammer CLIがDEV_USB_COMM_ERRで停止し、ユーザーのST-Link USB抜き差し後にDevice ID 0x421を再認識した。USBエラーの根本原因は未確定。

ソースのSDcard.c/main.cは保全ファイルとSHA256一致で元へ復元済み。元ELFも既存Debug ELFとSHA256一致。復元書込み・起動後の結果は以下へ追記する。

## 復元完了

ユーザーのST-Link USB再接続後、元ELFの書込・全照合（Download verified successfully）・リセット・起動に成功。起動後attachでinitMSD=true、initIMU=true、logRecoveryReady=true、patternTrace=0、autoStart=0、modeLOG=false、左右PWM=0、CFSR/HFSR=0を確認しdetach、今回のGDBサーバーも終了した。元ソース2本のSHA256一致を確認済み。一時診断コードは製品ソース・実機ファームに残っていない。変更は失敗記録とflash-debugの追加再発防止参照のみ。/sddiag21以下の合成データは専用フォルダへ分離して保持し、ルートの実走行ログ・設定を変更していない。

今回の実走行失敗の直接原因は未確定。次回の失敗直後、再起動せずにフラグを回収する必要がある。停止中試験成功を根拠に実走行の再現問題をresolvedにしない。
