# ビルド・デバッグの追加再発防止

## 成果物・PC検証

- ホストテスト等の無変換コピーもコンパイル前にバイト契約を確認する。原本同一を要求する場合はread_bytes/write_bytesまたはCopy-Itemで改行を保存し、SHA256一致を確認してから実行する。read_text/write_textで生成したコピーへ原本バイト一致を仮定しない。内容変換コピーは変換・改行規則を別に保存する。詳細は[改行差の再発](../../../../doc/失敗事例/2026-10-17-shared-header-line-ending-check.md)。

- ファームウェアのソース所在を調べるときは、リポジトリルートから`powershell.exe -NoProfile -ExecutionPolicy Bypass -File analysis/script/search_firmware_sources.ps1 -Pattern 'routeBuildFromLog|pathRouteSourceLog' -FileGlob 'path*.c'`のように固定ルート補助を使う。検索パターンをパスへ渡さず、rg終了コード2以上で停止する。本文中の既知の実在ファイルを読む場合は個別パスを使用できる。詳細は[ソース検索での再発](../../../../doc/失敗事例/2026-08-11-windows-rg-glob-not-expanded.md)。

- 複数リポジトリを扱う読取は絶対パス、Gitはgit -Cで対象を指定する。workdir確定後に対象のTest-Pathを確認し、ファームの相対パスを含む呼出しをPCツールworkdirへ変えない。共有logBinary.hはファーム側から配布し、ビルド前に両SHA256を照合する。詳しくは[workdir再発](../../../../doc/失敗事例/2026-10-16-self-describing-header-test-contract.md)と[共有ヘッダ改行](../../../../doc/失敗事例/2026-10-17-shared-header-line-ending-check.md)。

- 共通事前確認に従い、ビルドはファームウェアルートの明示workdir、generatorはリポジトリルートまたは絶対パスで実行する。PC変換器のビルドとCMakeは呼出しを分け、各workdirを初回から指定する。生成失敗ならビルドせず、件数・型・マクロ・生成時刻を確認する。
- 書込み前にHEAD/branch、構成、build日時、ELF絶対パスとSHA256を確認する。コミット後は再configure/buildして識別情報を更新する。実機初回ログのgitCommit/build情報が一致しなければ新変更の性能評価へ進まない。
- 計測が依頼された場合は、対象ELFのnmでisrTimingStats等の必要シンボルを先に確認する。計測OFF版にない集計を回収できるとは仮定しない。
- ビルド結果のRAM/FLASH表示はB固定で解析しない。リンカのKB表示は丸めを含み得るため、正確なRAM使用端はELFの実RAMセクション配置から集計する。増分ビルドでリンク表示がない場合も成功/失敗と混同しない。単位・セクション・構成名を確認してから検証JSONを確定する。詳細は[メモリ表示単位の解析](../../../../doc/失敗事例/2026-10-18-build-memory-unit-parser.md)。

- 変数追加は既存スコープとの重複を確認し、必要なDebug/Releaseと変更に関係するprofile/構成をビルドする。RAM/スタック・ログ形式の変更は全構成を確認する。
- PCハーネスは実ヘッダの型(FSIZE_t等)、デバイス定義、include順序、全リンク依存を確認する。新モジュールはCMakeだけでなく全該当テストランナーへ追加する。MinGWで未使用関数の参照が消えるとは仮定しない。
- 単体計算は既存extract方式で実装を取り出し、式をテストへ複製しない。ソース全体をリンクする場合は未使用経路を含む外部依存を明示的にmockし、禁止SD/delayが呼ばれたら失敗にする。
- フィクスチャはstatic/公開値/積算値/リセット/呼出元初期化を区別する。合成LogRecordへの値設定と実writeLogBufferPutsのスナップショットは別契約。既知の非ゼロ値の読戻しまで確認する。
- float保存値の完全一致では、コンパイル前に新設CHECKの比較右辺を確認し、最終演算結果をvolatile floatへ実体化してmemcmpする。中間LPF値のfloat宣言だけでは、その後の減算式の保存契約を保証しない。ホストの拡張精度式との直接比較を避け、許容誤差や製品コードを変更して通さない。候補版5では初回から必要権限でanalysis/script/validate_slip_lateral_v5.pyを実行する。詳細は[最終減算式での再発](../../../../doc/失敗事例/2026-10-12-host-float-snapshot-comparison.md)。
- Windows hostテストはassert失敗でGUI待ちにならない構成を使う。Pythonは構文確認だけで成功とせず対象unittest/実処理も実行する。

- 実FatFsのff.c・ccsbcs.cをMinGWで直接検証するときは、全翻訳単位へ`-include`で実在するCore/Inc/main.hの絶対パスを渡し、WindowsのLPTR/ERRORよりCMSISを先に評価する。テスト本体のinclude順序だけでは不十分。実FatFs/exFATのCRLF・事前確保・ブロック書込回帰を確認する。
- 実装抽出ハーネスは、計測ラッパーや内部実装を分けた変更後に公開APIの定義・依存が抽出結果へ含まれることを確認する。ファイル上の固定終端だけに依存せず、必要なラッパーを関数名で抽出し、実際にリンク・実行する。

## ST-Linkの操作順序

- この機体の既知の不安定接続を避け、書込み時の初回ターゲット接続から `port=SWD mode=UR reset=HWrst freq=100` を使う。プローブ列挙・電圧だけで成功としない。Device ID 0x421/Core IDが取れなければ書込みへ進まない。
- Programmerのsession_idが残る間は別のST-Link接続やGDBサーバー起動をしない。exit_code=0と照合/起動の最終メッセージを回収してから進む。非昇格Get-Processに見えないことは昇格Programmer終了の証拠ではない。
- サーバーは発見済みの絶対パスと毎回の-cp、ポート、SWD指定を使い、PID・stdout/stderrを保持する。待受と生存を確認してからGDBを接続する。
- 現CubeCLT ARM GDBはPython非対応。集計/コマンド生成は外部Python/PowerShell、GDB内は標準コマンドを使う。別版へ替えた場合はshow configurationで確認する。
- monitor reset haltを使わない。起動経路の再現が必要ならmonitor helpで対応を確認し、ブレーク設定後のmonitor reset等を選ぶ。CLIでhaltしてもGDB接続まで停止が維持されるとは仮定しない。
- 走行後RAMの回収は**リセットなしattach**を使う。接続復旧目的のUnder Reset・再書込み・機体再起動を行わない。USBエラーはサーバーログから確認し、機体電源を保ったST-Link側USB復旧を選ぶ。
- compare-sectionsのMIS-MATCHEDや単独verify接続失敗をFlash不一致と断定しない。読取可能ならリセットなしでFlashをdumpし、objcopy binaryとロード領域の全バイトを比較する。一致するELFだけでRAMを解釈する。
- 取得周期測定では停止中TIM3/TIM6/TIM7の進行による観測影響を分ける。DBGMCU元値を保存し、必要な凍結設定と計測区間を管理し、最後に復元する。注入件数・停止区間を通常実行統計へ混ぜない。
- 古いDebugベンチAPIは現行mainの存在を検索/nmで確認するまで呼ばない。再生XYは別ソース経路と一致するとは限らず、fixtureの距離進捗を対象driveRouteの弧長へ写像する。
- 開始要求はアプリ初期化後。IWDG初期化順序とHAL tickの開始順を確認し、リセット後の要求0/PWM0・自動再開なしを確認する。PRIMARYの保存/CSV検証が失敗したら残りケースへ進まない。
