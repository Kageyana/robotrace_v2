# CodexからのST-Link/GDB操作

この手順は、STM32F446RETxの書き込み・静止デバッグと、`debugBench.c`による車輪を浮かせたSDログ負荷検証に使う。実走行は行わない。リポジトリの`AGENTS.md`にある接続・安全条件を優先する。

## 1. 作業前に確認する

1. `rg -n "stlink|gdb|debug-bench|SD" doc/失敗事例` で関連する失敗と未解決事項を確認する。
2. 機体固定、車輪浮かせ、ST-Link、対象電源、SDカードを確認する。モーターを回す負荷測定では**モーター電源の接続も確認**する。電源なしで`OVERSPEED`となった結果は負荷測定に使えなかった。
3. 使用する構成を決め、`robotrace_v2/`で `cmake --preset Debug` と `cmake --build --preset Debug` を実行する。詳細ログなら両コマンドのpresetを`DebugMarker`、製品用なら`Release`に替える。ELFはそれぞれ`build/<preset>/robotrace_v2.elf`を使う。生成したfixtureが必要なら、生成器を**リポジトリ直下**から実行して終了コードと出力を確認してからビルドする。
4. `Get-Command STM32_Programmer_CLI.exe,ST-LINK_gdbserver.exe,arm-none-eabi-gdb.exe` で実際のツールパスを確認する。CubeCLTの版やPATHを決め打ちしない。

## 2. MCU接続と書き込み

GDBとGDBサーバーを終了した状態で、Programmer CLIからMCUへ接続する。プローブ一覧やターゲット電圧だけでは接続成功とは判定しない。STM32F446のDevice ID `0x421`とCore IDの取得を確認する。通常SWD接続が失敗したときはHot Plugで消去を続けず、次の接続条件を使う。

```powershell
STM32_Programmer_CLI.exe -c port=SWD mode=UR reset=HWrst freq=100
```

書き込みが必要なら、ビルドした**対象構成のELF**を選び、同じUnder Reset条件でダウンロード・照合・起動結果を確認する。例は`robotrace_v2/`からのDebug書き込み。VS Codeの定義は`robotrace_v2/.vscode/tasks.json`にある。

```powershell
STM32_Programmer_CLI.exe -c port=SWD mode=UR reset=HWrst freq=100 -d .\build\Debug\robotrace_v2.elf -v -rst -s
```

Device IDを取得できない状態が続く場合は書き込みと測定を中止し、SWDIO/SWCLK/GND/NRSTとターゲット電源の実接続を調べる。ST-LinkのUSB通信エラー時も盲目的に再書き込みせず、GDB/サーバーを解放し、必要ならUSB接続を復旧して100 kHz Under Resetで再確認する。

## 3. GDBサーバーとベンチ起動

1. Programmer CLIを終了してST-Linkを解放する。ST-LINK GDB ServerはSWD指定`-d`、ポート`-p`、STM32CubeProgrammerの`bin`ディレクトリを指す`-cp`を**毎回指定**する。例: `ST-LINK_gdbserver.exe -d -g -e -p 61234 -cp "C:\ST\STM32CubeCLT_1.19.0\STM32CubeProgrammer\bin"`。パスは手順1で発見した版に置き換える。背景プロセスとして起動する場合は`Start-Process -WindowStyle Hidden`を用い、プロセスの生存とTCP待受を確認する。起動しなければ前景実行でエラーを読む。過去に`-cp`欠落でサーバーが即終了した。
2. 一致するDebug ELFを`arm-none-eabi-gdb`へ読み込み、指定ポートへ接続する。リセット・停止にはこのサーバーで未対応だった`monitor reset halt`を使わない。必要ならGDB/サーバーを終了し、第5節のProgrammer CLI手順で行う。
3. `tbreak debugBenchMainLoop`で**アプリ初期化後**のメインループに到達させる。起動直後に`debugBenchRequestedMode`を設定すると初期化中に消える。SDを使うケースは開始前に`initMSD`、カード初期化、FatFsマウントの成功を確認する。カード検出だけでは足りない。初回起動時のSD初期化失敗は原因未確定なので、発生したら停止理由と各初期化段階を保存し、正常測定として扱わない。
4. `tbreak debugBenchCompletedBreakpoint`を設定し、挿入成功を確認する。`debugBenchRequestedMode`に`1` PRIMARY、`2` PATH REPLAY、`3` SHORTCUTのどれかを設定して続行する。測定中はコアを止めない。開始前安全確認が必要なときだけ`debugBenchPreStartBreakpoint`を使い、PWM変数とTIM2比較値が0、IWDGが動作中であることを確認した後、ブレークを外して続行する。回転中にウォッチドッグ更新点などで止めない。
5. 1ケースごとにMCUをリセットして要求値0から開始する。まずPRIMARYをCSV確定まで通し、結果が有効なら他のモード・プロファイルへ進む。PATH fixtureの再生XYを経路元XYと同一視しない。経路進捗への写像と`pathLostCount`を確認する。

GDBでの最小操作例（PRIMARY）は次の通り。接続先ポートはサーバー起動時と揃える。サーバー接続直後にコアが走行中なら、要求設定前に静止状態と初期化完了を確認する。

```gdb
target remote localhost:61234
tbreak debugBenchMainLoop
continue
print initMSD
print card_initialized
tbreak debugBenchCompletedBreakpoint
set variable debugBenchRequestedMode = 1
continue
print debugBenchResult
```

ブレークポイントは関数シンボル名で設定する。ソース行番号は編集後に別の処理を指し、実際にモーター回転中のウォッチドッグ更新点で停止した。一度に多数のブレークポイントを設定するとSTM32のハードウェア枠を超える。完了地点を基本に、診断が必要な場合も次の一地点だけを一時設定する。

## 4. 結果判定

完了地点で`debugBenchResult`、左右PWM変数、TIM2の左右比較値を読む。結果をCSV保存まで含めて有効とする条件は次の通り。

- `success=1`、`stopReason=TRACE_END`、`csvValidated=1`、実/期待行数一致、`cntlogMonotonic=1`。
- 書込位置・長さが512 B境界/倍数、`writeAlignmentErrors=0`、`sdWriteFailed=0`、短い書込なし、`multiSectorWriteCalls>0`。
- `logOverflowCount=0`、`oneMsPeriodOverruns=0`、PATH系は`pathLostCount=0`。
- 停止後の左右PWM変数とTIM2比較値がすべて0。停止理由、最大割込時間、最大SD待ち時間も記録する。

CSV見出しは`cntlog,`から探す。長いメタデータを固定行数で読み飛ばさず、派生列と末尾カンマによる空列も数える。検証値の例は`analysis/debug-bench-results-2026-09-29.md`。この空転ベンチの成功だけで実走行の安定性は判定しない。

## 5. 応答がないときの停止・復旧

- 実行中GDBへのCtrl+Cが効かない場合、入力を重ねずGDBクライアントとサーバーを終了する。ST-Link保持中にProgrammer CLIを並行接続すると`DEV_CONNECT_ERR`になった。
- ST-Linkを解放してから次を実行し、Device ID `0x421`、`MCU Reset`、`Core halted`を確認する。続行前にPWMゼロと要求モード0を確認する。

```powershell
STM32_Programmer_CLI.exe -c port=SWD mode=UR reset=HWrst freq=100 -rst -halt
```

- 停止中のGDBから`disk_initialize(0)`などブロッキングHAL/SPI/FatFs関数を直接呼ばない。応答不能になった実例がある。初期化は通常起動経路または専用デバッグ処理で実行し、入口・出口を観測する。
- 完了ブレークへ届かない、HardFault、SD保存失敗、IWDGリセットの場合は、そのケースを無効にし、残りの測定へ進まない。復旧後に停止理由、Faultレジスタ、リセット要因、PWMを回収する。IWDGリセット後は要求モード0・PWM0・自動再開なしを確認する。

## 6. ベンチ実装を変更するときの既知の落とし穴

- IWDGはLSI準備後、HALに整合する開始キー→保護解除→PR/RLR設定→更新フラグ待ち→reloadの順で設定する。モーター回転中はデバッガ停止によるIWDG停止を前提にしない。
- リセット要因取得処理で`HAL_GetTick()`を使うなら`HAL_Init()`後に呼ぶ。
- FatFsの`_MAX_SS`は実際の512 Bセクタと整合させ、CSV確定処理のスタックを点検する。過去に大きなセクタ設定とローカル配列によるヒープ/スタック衝突でHardFaultした。
- 不明な再発は`doc/失敗事例/`へ事実、未確定事項、復旧、再発防止の確認方法を記録し、必要ならこの手順も更新する。
