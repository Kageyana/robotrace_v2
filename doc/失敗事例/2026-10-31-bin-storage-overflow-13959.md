---
status: open
task: 走行後BIN storage failedをリセットなしST-Linkで調査
---

# 13959でログバッファあふれによりbin確定を拒否

対象は2026-10-31のPATH_CURVE走行。ユーザーが再起動せず接続したため、失敗時フラグを保存できた。

事実: 実FlashとDebug ELFの252984 Bが全バイト一致。初回と追加RAMでoverflow=true、logOverflow=true、dbg_overflow=1、writeFailed=false、markerOverflow=false。1103件/154 B、activeLength=1926 B、fil_W.fptr=168448 B。512 Bヘッダを除くSDデータ167936 BとRAM末尾1926 Bの和が全記録169862 Bと一致する。endLogはoverflow判定で確定前にfalseとなる。最終記録は2828 ms。停止理由は別にemcStop=7。PWM/CCR/Fault0、autoStart0。最終detach・所有サーバー終了済み。

原因: 空きバッファ不足が直接原因。SD内部busy/書込遅延とメイン/割り込みによる排出遅延の内訳は未確定。SPI ErrorCode0、ファイルerr0、十分な空き容量を確認したが、一時遅延を否定する情報ではない。

対処: リセット・再書込み・関数call・SD変更をせず、RAM/Flashを保全した。追加接続でOLED I2C割り込み内の待ちも観測したが、デバッガ停止の影響を分離できず今回の走行失敗原因と断定していない。完全ログとして復元/確定はしていない。

再発防止: 次の失敗も再起動せず回収する。時間診断追加時は既存ROBOTRACE_LOG_PERFORMANCEとRAMを確認し、f_write最長時間・戻り値/実書込長・キュー占有最大・書込タスク実行間隔最大・overflow初発時刻をRAMへ保持して実走行で照合する。時間診断を取得するまで根本原因をresolvedにしない。

確認: analysis/bin-storage-failed-20261031/analyze.pyは生RAMと追加RAMのフラグ一致、Flash全バイト一致、記録バイト収支、末尾12件のスキーマ復号・正の時刻差を検証。詳細は同フォルダreport.md、diagnosis.jsonと原ログ。ファーム変更なし、実走行の再検証なし。

## 今回の診断補助の失敗と修正

- 存在確認なしに旧buildフォルダ、logBufferWriter.h、Core/Src/user_diskio.cを指定した検索/読取が失敗した。実在一覧とSDcard.c内定義へ戻した。次回は既知でも今回のチェックアウトでTest-Path/rg --filesを先に確認する。
- 型付きGDBでUSERFatFSをソース宣言だけから選んだが未リンクで中断。file-only GDBのinfo variablesで実体fsを確認し、同様に存在しなかったcntlogをcntRunへ修正した。次回は接続前に全照会式の根シンボルを実ELFで確認する。
- 生GDBログは保全できていたが、表示用decode結果をCP932のstdoutへprintしてUnicodeEncodeErrorになった。capture.pyでstdout/stderrをUTF-8へ明示し、呼出側もPYTHONIOENCODING=utf-8とした。最終回収はexit0、detach成功。
- RAMはLEだがログ記録はBE。初回のLE記録復号は時刻差検証で拒否され、誤値を診断へ採用していない。実AppendU16/U32とReadU16を照合してBEへ修正後、12件の時刻差1～2 msと物理量を確認した。次回はコンテナ/RAM/レコードの各エンディアンを個別に確認する。
- record-failures Skillは今回の提供一覧とローカルSkill検索で見つからず、指定先へ必要項目を直接記録した。
- 最終解析のstdoutをSelect-Object -Firstへ直接つなぐと後段が途中で閉じ、終了コードが非0になった。ファイル出力は保全済み。パイプを外して同じ解析を実行しexit0とdiagnosis.jsonを確認した。検証プロセスの出力を途中で閉じず、表示量はツールのmax_output_tokensで制限する。

診断補助の修正は検証済み。バッファあふれの背景は未確定のため、本記録全体はopen。
