# CodexからのST-Link/GDB操作

この手順は、STM32F446RETxの書き込みと静止状態でのデバッグに使う。実走行は行わない。リポジトリの`AGENTS.md`にある接続・安全条件を優先する。過去のDebug空転ベンチ用コードは現在のmainには含まれないため、その関数や結果変数を前提にしない。

## 1. 作業前に確認する

1. `rg -n "stlink|gdb|SD" doc/失敗事例` で関連する失敗と未解決事項を確認する。
2. ST-Linkと対象電源を確認する。モーターを回す静止試験なら機体固定・車輪浮かせ・モーター電源も確認する。電源なしで得たエンコーダ値を負荷測定結果に使わない。
3. 使用する構成を決め、`robotrace_v2/`で `cmake --preset Debug` と `cmake --build --preset Debug` を実行する。製品用なら両コマンドのpresetを`Release`に替える。ELFは`build/<preset>/robotrace_v2.elf`を使う。
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

Device IDを取得できない状態が続く場合は書き込みとデバッグを中止し、SWDIO/SWCLK/GND/NRSTとターゲット電源の実接続を調べる。ST-LinkのUSB通信エラー時も盲目的に再書き込みせず、GDB/サーバーを解放し、必要ならUSB接続を復旧して100 kHz Under Resetで再確認する。

## 3. GDBサーバーと停止位置

1. Programmer CLIを終了してST-Linkを解放する。ST-LINK GDB ServerはSWD指定`-d`、ポート`-p`、STM32CubeProgrammerの`bin`ディレクトリを指す`-cp`を**毎回指定**する。例: `ST-LINK_gdbserver.exe -d -g -e -p 61234 -cp "C:\ST\STM32CubeCLT_1.19.0\STM32CubeProgrammer\bin"`。パスは手順1で発見した版に置き換える。背景プロセスとして起動する場合は`Start-Process -WindowStyle Hidden`を用い、プロセスの生存とTCP待受を確認する。起動しなければ前景実行でエラーを読む。過去に`-cp`欠落でサーバーが即終了した。
2. 一致するDebug ELFを`arm-none-eabi-gdb`へ読み込み、指定ポートへ接続する。リセット・停止にはこのサーバーで未対応だった`monitor reset halt`を使わない。必要ならGDB/サーバーを終了し、第4節のProgrammer CLI手順で行う。
3. ブレークポイントは現行ELFに存在する**関数シンボル名**で設定し、挿入成功を確認する。ソース行番号は編集後に別の処理を指した実例がある。一度に多数のブレークポイントを設定するとハードウェア枠を超えるため、必要な一点だけを一時設定する。モーター回転中にコアを止めない。
4. SD初期化を調べるときは通常の起動経路で`initMSD`、カード初期化、FatFsマウントの各結果を観測する。カード検出だけでは成功と判定しない。

## 4. 応答がないときの停止・復旧

- 実行中GDBへのCtrl+Cが効かない場合、入力を重ねずGDBクライアントとサーバーを終了する。ST-Link保持中にProgrammer CLIを並行接続すると`DEV_CONNECT_ERR`になった。
- ST-Linkを解放してから次を実行し、Device ID `0x421`、`MCU Reset`、`Core halted`を確認する。続行前に左右PWMとTIM2比較値がゼロであることを確認する。

```powershell
STM32_Programmer_CLI.exe -c port=SWD mode=UR reset=HWrst freq=100 -rst -halt
```

- 停止中のGDBから`disk_initialize(0)`などブロッキングHAL/SPI/FatFs関数を直接呼ばない。応答不能になった実例がある。初期化は通常起動経路または専用デバッグ処理で実行し、入口・出口を観測する。
- HardFault、SD保存失敗、IWDGリセットの場合はそこで調査を止め、復旧後にFaultレジスタ、リセット要因、PWMを回収する。

## 5. 過去の失敗から維持する確認

- IWDGを使う処理を追加するときは、LSI準備後、HALに整合する開始キー→保護解除→PR/RLR設定→更新フラグ待ち→reloadの順を確認する。リセット復旧で`HAL_GetTick()`を使うなら`HAL_Init()`後に呼ぶ。
- FatFsの`_MAX_SS`は実際の512 Bセクタと整合させ、CSV確定処理のスタックを点検する。過去に大きなセクタ設定とローカル配列によるヒープ/スタック衝突でHardFaultした。
- 不明な再発は`doc/失敗事例/`へ事実、未確定事項、復旧、再発防止の確認方法を記録し、必要ならこの手順も更新する。
