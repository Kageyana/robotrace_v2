---
status: resolved
---

# float版3回目の回収時にST-Link USB通信が失敗

対象: float版の走行後DWT集計をリセットなしで回収する。

確認結果: attachモードのST-LINK GDB Server 7.11.0が「Target USB comms error」「Error in initializing ST-LINK device」で起動失敗した。GDBはlocalhost:61234への接続タイムアウトとなり、今回の集計は未取得。書き込み・リセット・実機メモリ変更は実行していない。

原因: USB通信エラーは確認済み。根本原因は未確定。開始時に他のGDB/Programmer/サーバープロセスは見つからなかった。以前の書き込み前USBエラーはUSB再接続で復旧したが、同種エラーが回収時にも再発した。

対処または次の調査: 自分が起動したサーバーを終了し、再接続待ち。機体電源を維持してST-Link側のUSBのみ抜き差しし、attachで集計を回収する。回収前にUnder Reset・再書き込み・機体再起動を行わない。

再発防止: GDB接続前にサーバーが生存しポートが待受状態か確認する。USBエラー時はGDBへ進まず、プローブ接続を復旧する。走行後の集計保全が必要なときは、接続復旧のためのMCUリセットを行わない。

確認方法: リセットなしattachでisrTimingStatsを読み出し、停止状態とクロックを同時に保存する。現在はユーザーのUSB再接続待ち。

証拠: analysis/fpu-float-audit/read-float-third-server.txt、read-float-third-server-error.txt、read-float-third.txt。

復旧確認: USB再接続後にattach成功。機体をリセットせず、logFileNumber=12921、isrTimingStatsの一次走行22824サンプル、emcStop=0、左右PWM=0を取得した。集計保全と接続解除を確認済み。失敗時はサーバーのUSBエラーを先に確認し、GDBの接続待ちを繰り返さない。

