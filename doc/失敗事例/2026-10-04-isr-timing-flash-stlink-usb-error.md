---
status: resolved
---

# 計測ON版書き込み前のST-Link USB通信エラー

対象: a993603 float化版/Debug/ROBOTRACE_ISR_TIMING=ON ELFの書き込み。

確認結果: SHA256は44ed544d980e099e007f07b1de35b4be7b9c375877076d8f365fe5eadd8db994で保存manifestと一致し、isrTimingStatsを確認した。しかしSTM32_Programmer_CLIの100kHz Under Reset接続がDEV_USB_COMM_ERRで失敗した。続く一覧確認でもST-LinkのSN/FW等はすべて「-」だった。書き込みは実行していない。

原因: ST-LinkのUSB通信失敗は確認済み。USB接続・プローブ状態・ドライバ等の根本原因は未確定。確認開始時、ST-Link GDBサーバーとGDBは稼働していなかった。

対処または次の調査: 一覧取得CLIがUART列挙を継続したため終了。ユーザーがST-LinkのUSBを抜き差し後、Device ID 0x421とCore ID取得を再確認し、成功した場合のみ同じ計測ON ELFを書き込む。

再発防止: プローブ名やターゲット電圧だけで接続成功としない。DEV_USB_COMM_ERR時は書き込みを繰り返さず、プローブ保持プロセスを解放しUSB接続を復旧する。SN/Device IDと接続成否を確認してからダウンロード・照合・起動へ進む。

確認方法: 復旧後に -c port=SWD mode=UR reset=HWrst freq=100 でDevice ID 0x421を取得し、計測ON版のダウンロードとVerify成功を確認する。現在は復旧待ち。

復旧確認: ユーザーのUSB再接続後、ST-Link SNとDevice ID 0x421を取得できた。同じ計測ON ELFのDownload verified successfully、Start operation achieved successfullyを確認した。リセットなしのAttachでIMU/SD初期化成功、左右PWM=0、isrTimingStats全モード0の起動時初期化を確認し、デバッガ接続を解除した。USB再接続による復旧を検証済み。

