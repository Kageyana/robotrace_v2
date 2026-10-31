# PATH_CURVEのIRQ・白線更新の追加計測

13870はユーザー完走確認と全CRC/CSV/RAM一致を確認済み。旧計測のPATH_CURVE主要区間は平均381.905µs・最大501.061µs、1ms超過0回だった。一方、白線更新最大age6msは校正・一次・PATHを分けられず、HAL前後も時間集計に含まれていなかった。この2点を区別するため、既存`ROBOTRACE_ISR_TIMING`を拡張した。

## 変更範囲

- `timer.c/h`：従来の主要区間に加え、TIM6の入口フックからHAL処理後の出口フックまでを`isrIrqTimingStats`へ集計する。出口の観測時に次周期の更新フラグが残っていれば`isrIrqPendingTicks`へ加算する。走行範囲は従来と同じpattern12～99、モード0/2/3/4/5。
- `stm32f4xx_it.c`：TIM6のUSER CODE領域に計測ON時だけ入口・出口フックを置く。
- `control.c`：DWT初期化後、計測ON時だけ差分時計の基準を合わせる。
- `lineSensor.c/h`：1ms周期の観測から、モード別の更新age、更新間隔の分布、取得要求の棄却差分を`lineSensorTimingStats`へ保持する。

計測ON時はDWTを連続運転し、周期間隔を符号なし差分で求める。これによりIRQ入口から出口まで時計がリセットされない。カウンタ周回にも対応する。計測OFFは従来のDWTリセット経路のまま。速度、PID、白線の採否条件、緊急停止条件、ログ列/レコード長は変更していない。

## 計測値の範囲

`isrTimingStats`は主要区間、`isrIrqTimingStats`はHALを含む入口・出口間。後者にも出口の集計末尾、関数の末尾、CPU例外入出は含まれない。計測によるRAM集計の追加負荷もあるため、通常版の実行時間と同一とはしない。元の381.905/501.061µsをIRQ全体の値へ置換しない。

白線集計の添字は0=一次、2=DISTANCE/SLIP、3=SHORTCUT、4=PATH、5=PATH_CURVE、6=上記走行外（開始待ち・校正・停止等）、1は予約。更新間隔は同じセンサー電源世代かつ同じ区分の観測が連続した区間だけに集計し、電源OFFやモード境界を長い欠測へ誤算入しない。間隔はHAL tickによる1ms周期からの観測で、ADC完了時刻そのものではない。

`maxAgeMs`と`staleSamples`は補正の鮮度判定に使用するageを観測し、操舵のライン使用条件で除外しない。age3ms以上の標本数を保存する。`gapHistogram`は0～7msを個別、8ms以上を最後へまとめる。最大gapの時刻・pattern・ライン校正モードも残す。取得要求棄却の差分は区間末尾のモードへ割り当てるため、モード境界をまたぐ差分を含む。棄却理由の内訳や完全な1msログではない。

## 検証済みの項目

- 実関数抽出のO0/O2・Wall/Wextra/Werror：DWT/HAL tickの周回、入口モードの保持、1ms境界・次tick保留、64bit合計、鮮度、電源切替・モード切替の除外、カウンタ飽和が成功。
- Debug、Release、DebugMarker、独立PathIntervalTimingがビルド成功。変更前の全体ログと比較し新規warning0。既存warning自体は残る。
- 計測OFFのDebugで、対象4モジュールの全実行セクションのバイト列とシンボル参照先が変更前と一致。
- 計測ONはRAM80376B・Flash250588B、スタック予約8KiB。通常DebugのRAM79528Bを維持。
- 不変ELFのSHA256は`0ae5b57ab26e81ecd9917ef4f7aae6b5ffb17298e1eb92ccc58653623cda5c38`。RAM読出し227項目のDWARF配置、0件を実測済みとしない処理、合成データでの64bit/1ms/区分/不整合拒否を確認した。

実行手順はリポジトリルート・必要権限で`validation/isr_diagnostics/run.py`、`build/isr-interval-validation/preflight_helpers.py`、`finalize.py`。配置確認は`analysis/script/read_isr_interval_snapshot.py prepare`、読出しの合成試験は`self-test`。通常3構成と計測ONは別ディレクトリへ置く。GNUツールの既知の実行制約に従う。

## 実機の状態と次の確認

2026-10-28の最初の確認ではST-Link本体だけを検出し、MCUは`No device found on target`だった。その後、ユーザーから「停止中・電源と接続を確認済み」と連絡を受け、接続が回復した。

リセットなしで旧計測版のFlash249640BとRAM128KiBを保全し、旧ELFとの全バイト一致、pattern0/start0/autoStart0/modeLOG0/PWM左右0を確認した。保存キューも処理待ちなし。追加版SHA0ae5b57a…をDevice ID0x421/Cortex-M4へ100kHzで書込み、ダウンロード約40.5秒・照合約31.4秒で成功した。

通常起動後のsetupでIMU初期化成功、pattern/start/autoStart0、ログ非記録、左右PWM/CCR0、PRIMASK0、CFSR/HFSR0を確認しdetach、所有サーバーを終了した。走行モードの主要時間/IRQ時間の標本は全て0件、白線の走行中区分も0件。区分6の20590標本は停止中/起動後の観測で、走行性能の実測へ数えない。

取得時はSD初期化false・バッテリー電圧有効falseだった。速度やShortcut設定の注入はしていない。走行前にSDカードと機体バッテリーで通常起動し、既存の設定・一次参照を読み込める状態を確認する。高速走行前は7.5V以上を運用確認する。実走はユーザーがST-Linkを外して一次→PATH_CURVEを各1走し、停止後はリセット・電源OFFの前に再接続してRAMを採取する。電源/リセットは走行前の準備と走行後の保全で区別する。

実走のIRQ時間とモード別6ms発生区間、2秒停止保持中の実PWM履歴は未検証。完走の一般化や13854の保存失敗原因を、今回のビルド/合成試験で解決済みとはしない。

証跡は`build/isr-interval-validation/validation.json`、`host_validation.json`、`ram-layout.json`、`decoder_test.json`、`hardware_validation.json`、`connect.log/flash.log/boot.log`。書込み前は同フォルダの`pre-flash-20261028T104701Z/`。過去の未接続ログは`build/path-timing-validation/after-run-20261028T103031Z-server.log`。比較器のwarningゼロ仮定とリンカファイルの既定文字コードによる再発は、既存失敗記録とflash-debug Skillへ反映した。

## 13871～13875走行後の状態

追加計測版に整合する保存ビルド19:23:07の一次13871とPATH_CURVE13872～13875を取得した。13873は自己位置喪失停止、残り3本は動画でも正常終了を確認した。しかしユーザーが走行後に電源を切った/リセットしたため、追加RAM計測は失われ、今回の全IRQ時間・モード別ADC統計は未取得である。静止起動の0件を走行値として扱わない。今回ハードウェアへは接続していない。SDログの全CRC・動画照合は別途analysis/path_fusion_13872_13875/validation.jsonで確認した。

## 追加版の実走計測を回収：13876～13878

2026-10-28 20:32頃、走行後の停止接続からリセット/書込みなしでRAM128KiB・Flash250588Bを回収し、追加版SHA0ae5b57a…と全Flash一致を確認した。CCR/PWM左右0、PRIMASK/Fault0、autoStart0、ログ非記録でdetach・所有サーバー終了。RAMメタデータ全内容はSD13878と一致し、ユーザーも動画203011_00_003＝13878と確認した。一次13876・PATH13877/13878の全CRCを確認した。

一次22,656標本のIRQ平均179.84/最大246.29µs、PATH累積40,134標本は平均376.15/最大502.09µsで1ms超過・次tick保留0。IRQ出口統計/CPU例外入出は計測外。PATHは主要処理最大497.27µsだが、白線更新間隔最大6ms・age3ms以上23.94%であり、制御時間の余裕と取得品質を分ける必要がある。一次は更新間隔最大2ms・鮮度切れ0。最大6msの区分はPATH・pattern12・校正モード0まで特定できたが、2走のどちらかや停止時刻との同時性は分離できない。

PATH2本とも長時間CAUTIONの後に未照合150msで停止。実関数の合成入力でも3ms新鮮/1ms鮮度切れにより100ms連続の減速解除を満たせない。今回の1ms履歴は未保存で、取得窓競合を直接の原因と確定していない。取得方式/優先度と回復条件の整合が次の修正対象。実2秒保持の出力履歴は依然未測定。今回ファーム・設定変更なし。証跡はafter-run-20261028T113242Z/validation.json、analysis/path_fusion_13876_13878/report.md。

## 取得版2の計測版へ更新

886669cc…の独立PathAcquisitionTimingへ更新した。TIM6優先度を運転時3とし、制御途中のraw/正規化公開を保留する。静止負荷は既定0・最大600µs、非走行/開始なし/autoStartなし/ログなしだけで動き、isrIdleIrqTimingStatsへ別集計する。通常構成には含まれない。450µs負荷を使う旧/新交互4区間で新側は1ms公開を維持し、IRQ区間最大606µs/1ms超過0を確認した。実走のモード配列は全0で未計測。

GDBからの直接callはCPU例外状態を変える事例があったため禁止し、通常センサーテストUI経路でON/OFFした。最終の負荷0/優先度3/凍結0/センサーOFF/PWM0/IPSR0と、IMU/SD成功を確認してdetachした。次の実走後はbuild/adc-priority-validation/hardware.py capture-runを使い、read_adc_priority_snapshot.pyで対象ELFと全Flash一致を確認してから234項目を読む。旧0ae5…用の読出し器を新像へ流用しない。詳細は[line-acquisition-priority.md](line-acquisition-priority.md)。
