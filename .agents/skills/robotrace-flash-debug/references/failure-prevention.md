# ビルド・デバッグの追加再発防止

## 成果物・PC検証

- このWindows制限環境ではST-LINK_gdbserver、Programmer CLI、GDB等のCubeCLT同梱exeにもDLL再配置エラーがある。GNUツールだけに限定せず、help/版表示を含め初回から許可された制限外環境で実行する。再発記録は[2026-10-11事例の2026-10-21追記](../../../../doc/失敗事例/2026-10-11-bmi088-forward-y-sign.md)を確認する。
- PowerShellでファイルパスを含むCMakeの-Dを使う場合、引数全体を単一引用符で囲む。独自構成では存在確認したツールチェーン/Ninja絶対パスを使用し、configure終了コード0を確認してからbuildする。通常は既存presetを使う。[分割失敗](../../../../doc/失敗事例/2026-10-21-cmake-toolchain-argument-split.md)を参照。
- 計測ラッパーの挿入では、ネストするCの関数呼出しを正規表現の最短一致で置換しない。既知の完全な式を個別パッチし、変更した全式とコンパイル成功を確認してからFlashへ進む。[括弧破損](../../../../doc/失敗事例/2026-10-21-diagnostic-wrapper-parentheses.md)を参照。
- 再起動で失敗時RAMが失われた場合、後から取得したSPIエラー0やoverflow=falseで当時の原因を否定しない。既存tmpは読み取りで保全し、静止中の合成保存と実走行時の非同期I/Oを区別する。メインの通常経路からFA_READ/f_lseek/f_readを使い、GDBでFILのfptr/clustや読取ポインタを組み替えない。[Write failed調査](../../../../doc/失敗事例/2026-10-21-post-run-write-failed.md)を参照。

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
- 選択統合ではmain側の関数引数・レコード生成マクロ・任意ソースの存在を先に照合し、他ブランチ専用のハーネス依存を持ち込まない。CMakeの`-D`指定は引数全体を引用し、キャッシュのツールチェーンパスとprofileを確認する。詳細: `doc/失敗事例/2026-10-16-selective-log-main-integration-contract.md`。
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

- レコードごと等の頻回関数へ、ホスト評価の条件付きGDBブレークを置いて大量ヒットさせない。観測が遅延する場合は中断し、通常メイン処理で必要な境界値をRAMへ一度保存する計測へ置き換える。モーター停止を確認し、再利用するブレークは対象関数と実際の停止回数を先に限定する。

- 新規パーサー・整数範囲チェックはARMビルドだけでなくMinGWの`-Wall -Wextra -Werror`でもコンパイル・実行する。int32_tとUINT8_MAX/UINT16_MAXを比べる際は、上限がint32_t内であることを確認し同符号へ明示変換する。検証例は`tests/slip_v2_active_test.c`、詳細は[整数境界の型差](../../../../doc/失敗事例/2026-10-21-slip-v2-host-signed-bounds.md)。

## 現行ログと抽出テスト（2026-10-24）

- 機体CSV変換・旧CSV解析と対応する旧テストは廃止済み。現行binの検証はリポジトリルートから`python validation/log_storage/run.py`。PC変換確認時だけ`--converter C:/tool/AutoCopyCsvFromSd/RlogToCsv.exe`を付け、最初から必要な実行権限を使用する。
- センサー角度関数の抽出では実関数だけでなく`angleSensor`の公開変数定義と`IMU_PI_F`も実ソースから取り込む。抽出対象を追加したら依存する公開値・定数を先に列挙し、O0/O2のWall/Wextra/Werrorでコンパイルする。未宣言のテスト依存を製品コードの変更で回避しない。
- 保存故障は実際のopen/read/closeの段階に注入する。正常openがcloseを呼ぶと仮定しない。詳細は`doc/失敗事例/2026-10-24-cleanup-host-fixture-contract.md`。

## 共通UI・設定定義のホスト比較（2026-10-25）

- 共通ループの追加前に、UI選択値・配列添字・sizeof・unsigned接尾辞を含む比較式の型を確認する。整数上限マクロだけを対象にしない。変換値が対象型の範囲に収まることを確かめて同符号へ明示変換し、実関数をO0/O2・Wall/Wextra/Werrorでコンパイルする。詳細は[符号比較の再発追記](../../../../doc/失敗事例/2026-10-21-slip-v2-host-signed-bounds.md)。
- 変更前スナップショットを比較参照へ追加する場合は、参照側の既存warningを先に識別する。許容が必要な既存診断だけを参照ブロック内のdiagnostic push/popで囲み、理由を記載する。現行コードのWerrorを弱めない。詳細は[参照実装の警告契約](../../../../doc/失敗事例/2026-10-24-cleanup-host-fixture-contract.md)。
- リポジトリルートを明示workdirとして、最初から必要権限で`python validation/control_refactor/run.py`を実行する。任意の作業前Coreがある場合のみ`--baseline build/main-refactor-20261025/before`を付ける。通常実行にスナップショットを必須としない。失敗時は実機検証へ進めず、ビルド診断とアサートを区別して対処する。