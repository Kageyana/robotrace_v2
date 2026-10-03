---
status: resolved
---

# ST-Linkのcompare-sectionsが実Flash一致時にも不一致を報告

対象: FPU単精度化版の走行後ELF照合とDWT計測回収。

確認結果: GDB compare-sectionsは通常Debug ELFの.text/.rodata/.isr_vectorをMIS-MATCHEDと報告した。リセットなしで実Flash先頭256KiBを読み出し、同ELFをobjcopy -O binaryした230212 bytesと比較すると、全バイトが一致した。Device IDは0x421。ST-Link GDB Server 7.11.0を使用した。

原因: compare-sectionsの判定が実Flash内容と矛盾した事実は確認済み。サーバーのCRC応答等、内部原因は未確定。

対処: Flashのバイト比較でELFを確定し、そのELFで停止状態、PWM、emcStop、ログ番号を回収した。

関連結果: 実機はa993603の通常Debug/計測OFF版だったため、isrTimingStatsは存在せず今回の処理時間を回収できなかった。今後は専用計測ON版ELFを指定する。

再発防止: 走行後はリセットしない。compare-sectionsが不一致なら直ちに再書き込みせず、Flash読出しとELFバイナリの直接比較を行う。一致するELFだけで変数を解釈する。書き込み前にnmでisrTimingStatsを確認する。doc/fpu-float-validation.mdへ実行コマンドと照合方法を追加した。

検証: analysis/fpu-float-audit/device-flash.bin、candidate-0.binの比較が0差分。read-matched.txtで静止、左右PWM=0、emcStop=0、logFileNumber=12914を確認した。再走行での計測回収は未実施。
