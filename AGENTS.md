# AGENTS.md

## 目次

- 1. プロジェクト概要
- 2. Codex の役割と応答方針
- 3. リポジトリ構成
- 4. ハードウェア構成と参照ルール
- 5. 開発環境、ビルド、デバッグ
- 6. 単位系、周期、安全制約
- 7. ファームウェア構成と作業ルール
- 8. 割り込み・周期の責務
- 9. パラメータ管理
- 10. SD カード内設定ファイル
- 11. 初期化、校正、走行開始・終了
- 12. 走行モード、制御改善、チューニング
- 13. ログ、ファームウェアバージョン、解析
- 14. 方針と現行コードの差分管理
- 15. 機体・回路変更時にコードへ反映する項目

## 1. プロジェクト概要

このリポジトリは、マイクロマウス系競技のロボトレース種目に参加するロボットを開発するためのプロジェクトです。

開発対象は以下を含みます。

- 機体設計
- 制御回路設計
- 組み込みファームウェア

主な目的は、安定して完走できる範囲で最速ラップタイムを出し、ロボトレース競技で勝利することです。

## 2. Codex の役割と応答方針

Codex は主にファームウェア開発に使用します。必要に応じて、機体設計や制御回路についても支援します。

このリポジトリで作業するとき、Codex は以下を優先します。

- ラップタイムの短縮
- 安定して再現性のある走行
- 分かりやすいパラメータ調整
- センサー、モーター、制御周期の制約
- 実機で検証できる変更

### コミュニケーション方針

- 日本語で回答する。
- 簡潔で、事実に基づき、実装に直結する説明をする。
- コード変更時は、何を変更したか、なぜ変更したか、どのように確認したか、残っているリスクを報告する。
- 実機の挙動に依存し、ローカル環境で確認できない変更については、そのことを明確に伝える。

### Skill 化した手順

長い手順系の詳細は repo-scoped Skill に分離しています。AGENTS.md には正とする方針、単位、構成、参照先だけを残します。

- ログ解析: `.agents/skills/robotrace-log-analysis/SKILL.md`
- チューニング: `.agents/skills/robotrace-tuning/SKILL.md`
- 書き込み・デバッグ: `.agents/skills/robotrace-flash-debug/SKILL.md`
- 機体・回路確認: `.agents/skills/robotrace-hardware-review/SKILL.md`
- SD カード設定ファイル: `.agents/skills/robotrace-sd-settings/SKILL.md`

該当作業では、AGENTS.md のプロジェクト方針を前提にしつつ、上記 Skill の手順を使用します。

## 3. リポジトリ構成

このリポジトリは、機体データ、回路データ、ファームウェアを同じリポジトリ内で管理します。

- `Machine/`: 機体設計データ。
- `Circuit/`: 制御回路設計データ。
- `robotrace_v2/`: STM32 組み込みファームウェアプロジェクト。
- `.vscode/`: リポジトリ直下を開いたときの VS Code 設定。
- `analysis/`: ログ解析スクリプト、解析グラフ、解析表の保存先。
- `build/`: 生成されたビルド成果物。ソースとして扱わない。

### `Machine/`

`Machine/` には機体の 3D 機械設計データを保存しています。現在の機体 STEP モデルは `Machine/robotrace_v2 v86.step` です。機体寸法、基板配置、センサー位置、クリアランス、重心や搭載スペースを確認するときに参照します。

### `Circuit/`

`Circuit/` には KiCad で作成した制御回路データを保存しています。各基板フォルダには、KiCad プロジェクト、回路図、基板レイアウト、確認用 PDF が含まれます。

- `Circuit/robotrace_v2_main_v2/`: メイン基板。
- `Circuit/robotrace_v2_Linesensor/`: ラインセンサー基板。
- `Circuit/robotrace_v2_Sidemarker/`: サイドマーカー基板。
- `Circuit/robotrace_v2_Extended_UI/`: 拡張 UI 基板。

回路に関する作業では、PDF だけで判断せず、必要に応じて `.kicad_sch` と `.kicad_pcb` も確認します。

### `robotrace_v2/`

`robotrace_v2/` は STM32F446RETx 向けのファームウェアプロジェクトです。STM32CubeMX、STM32CubeCLT、CMake、VS Code の設定を含みます。

主な参照先:

- `robotrace_v2/robotrace_v2.ioc`: ピン、クロック、ペリフェラル、DMA、割り込み設定。
- `robotrace_v2/.vscode/`: CMake、Flash、ST-Link デバッグ設定。
- `robotrace_v2/CMakeLists.txt`, `robotrace_v2/CMakePresets.json`, `robotrace_v2/cmake/`: CMake とツールチェーン設定。
- `robotrace_v2/Core/Src/`, `robotrace_v2/Core/Inc/`: アプリケーション実装とヘッダ。
- `robotrace_v2/FATFS/`, `robotrace_v2/Drivers/`, `robotrace_v2/Middlewares/`: FatFs、HAL/CMSIS、外部ミドルウェア。
- `robotrace_v2/build/`, `robotrace_v2/Debug/`: 生成物。原則として編集しない。

## 4. ハードウェア構成と参照ルール

### 現在使用中の機体・基板

- 機体モデル: `robotrace_v2`
- メイン基板: `robotrace_v2_main_v2`
- ラインセンサー基板: `robotrace_v2_Linesensor`
- サイドマーカー基板: `robotrace_v2_Sidemarker`
- UI 基板: `robotrace_v2_Extended_UI`

### ファームウェアが前提としている実機構成

- ラインセンサー: 10 個。左右 5 個ずつ。
- 駆動モーター: 2 個。
- 減速比: 2。
- マーカーセンサー: 2 個。
- IMU: 1 個。
- ディスプレイ: 1 個。
- ボタン: 5 個。
- microSD カード: 1 個。
- ロータリーエンコーダ: 2 個。

### 座標系・符号規約

- `x`, `y` の原点: スタートマーカ通過時の機体位置。
- 進行方向の角度定義: CW を `+`、CCW を `-` とする。
- 左右モーターの正方向: 前進を `+`、後退を `-` とする。
- `gyroVal_Z` の正方向: 機体のフットプリントに対して CW を `+` とする。
- 左右エンコーダの符号: 前進を `+`、後退を `-` とする。

### 機体寸法パラメータの正

- タイヤ径: `23.5 mm`
- トレッド幅: `11 mm`
- エンコーダ分解能: `512 カウント/回転`、4 逓倍。
- 機体寸法パラメータは、コード内の値を正とする。
- コード、STEP、実測値が一致しない場合はコードを基準に判断し、必要に応じてコードと `AGENTS.md` を更新する。

### ラインセンサー・マーカーセンサーの並び

- ラインセンサーの `sensor[0]` は左端とする。
- `markerSensorL` は実機左側のマーカーセンサーとする。
- `markerSensorR` は実機右側のマーカーセンサーとする。

ハードウェアについて推測する前に、以下のファイルを確認します。

- `Machine/robotrace_v2 v86.step`: 機体モデル。
- `Circuit/robotrace_v2_main_v2/`: メイン基板の KiCad プロジェクトと回路図 PDF。
- `Circuit/robotrace_v2_Linesensor/`: ラインセンサー基板の KiCad プロジェクトと回路図 PDF。
- `Circuit/robotrace_v2_Sidemarker/`: サイドマーカー基板の KiCad プロジェクトと回路図 PDF。
- `Circuit/robotrace_v2_Extended_UI/`: 拡張 UI 基板の KiCad プロジェクトと回路図 PDF。
- `robotrace_v2/robotrace_v2.ioc`: MCU ピン、ペリフェラル、クロック、DMA、割り込み、生成 HAL 設定。

ピン割り当て、センサー配線、電源系統、コネクタの意味を変数名だけから推測しないでください。必ず `.ioc`、回路図、関連ソースを確認します。

### 機体または回路の支援時

- 先に関連する STEP、KiCad、PDF、または `.ioc` ファイルを確認する。
- 確認できた事実と推測を区別する。
- 実機で確認すべき項目を明示する。
- 詳細手順は `.agents/skills/robotrace-hardware-review/SKILL.md` を使う。

## 5. 開発環境、ビルド、デバッグ

このファームウェアは、ARM GCC を使う STM32F446 向けの C11 プロジェクトです。

主な開発環境は VS Code、STM32CubeIDE for Visual Studio Code 拡張機能、STM32CubeCLT、STM32CubeMX です。`.vscode/` には CMake、Flash、ST-Link デバッグ用の設定があります。

想定ツールは CMake 3.22 以上、Ninja、ARM GCC、STM32CubeCLT、`STM32_Programmer_CLI` です。

### 書き込み・デバッグの実行可否

- Codex が `STM32_Programmer_CLI` による SWD 書き込みを実行することは常に許可する。
- ただし、書き込みを実行してよいのは、実機と ST-Link が接続されているときだけとする。
- Codex が ST-Link デバッグを実行することも許可する。
- デバッグ時は実走行を行わない。空転確認や緊急停止条件の一時無効化は、静止状態でも確認できる事項に限る。
- 詳細手順は `.agents/skills/robotrace-flash-debug/SKILL.md` を使う。

Codex が CLI でビルド確認する場合は、`robotrace_v2/` から以下を実行します。Codex のシェルでは `cube-cmake` が PATH に無い場合があるため、通常の `cmake` コマンドを使います。

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

### Release ビルド

```powershell
cmake --preset Release
cmake --build --preset Release
```

ローカル環境でツールが利用できない場合、ビルド成功とは報告しません。不足ツールまたは失敗コマンドを報告します。`Debug`/`Release` preset の出力先は `robotrace_v2/build/Debug` と `robotrace_v2/build/Release` で、生成物として扱います。

`printf` と `scanf` の float 対応リンカフラグ `-u _printf_float`, `-u _scanf_float` は、明確な理由がない限り維持します。

## 6. 単位系、周期、安全制約

このプロジェクトでは、以下の単位系を基準とします。

- 距離: `mm`
- 速度: `m/s`
- 角速度: `deg/s`
- 時間: `ms`
- 制御周期: `1 ms`
- PWM: `-1000` から `1000`
- 電流: `A`
- 電圧: `V`

電圧の最大値は、その時点のバッテリー電圧を上限とします。バッテリーは LiPo 2S を前提とし、満充電時の最大電圧は `8.4 V` です。

制御、ログ解析、パラメータ調整では、上記の単位系を前提に扱います。異なる単位を使う場合は、変数名、コメント、ログヘッダ、または報告内で明示します。

### バッテリー電圧の運用

- `batteryVoltage_V` は `AD2VOLTAGE(batteryAD)` で算出した電圧 `[V]` とする。
- `batteryVoltage_V` はログヘッダへ出力し、速度フィードフォワード計算にも使用する。
- 高速走行前の下限 `7.5 V` は運用上のチェック値とする。
- 現行コードでは、`7.5 V` 未満を理由にした自動走行禁止処理は行わない。

実機を動かす変更では、以下の制約を前提にします。

- 最大速度: `10 m/s`
- 最大加速度: `20 m/s^2`
- 最大 PWM: `-1000` から `1000`
- 吸引ファンは現時点では使用しない。

### 緊急停止を無効化してよい条件

- デバッグ中に走行開始しても動かず、緊急停止してしまう場合。
- 低速走行中に緊急停止してしまう場合。

上記以外では、原則として緊急停止を無効化しない。緊急停止条件を変更する場合は、変更理由、対象条件、実機で確認すべきリスクを明示します。

テストは常に室内で実施します。本番環境も室内です。室内走行を前提に、速度、加速度、PWM、緊急停止条件を扱います。

## 7. ファームウェア構成と作業ルール

関連作業では、該当する `.c` と `.h` を編集前に確認します。主な起点は `battery`, `IMU`/`BMI088`, `encoder`, `motor`, `PIDcontrol`, `control`, `courseAnalysis`, `lineSensor`, `markerSensor`, `emergencyStop`, `SDcard`, `sd_functions`, `setup`, `main` です。

### ファームウェア作業ルール

- 変更範囲は依頼された挙動に必要な範囲へ絞る。
- 明示的に依頼されない限り、ユーザーの変更を戻さない。
- 既存のモジュール境界と命名規則を優先する。
- 生成された STM32CubeMX、HAL、CMSIS、ミドルウェアへの変更は最小限にする。
- 生成ファイルを編集する場合は STM32CubeMX の `USER CODE` 領域を維持する。
- 既存 C ソースの文字化けコメントは修正対象とする。修正時は周辺コードから意味を確認し、新規・修正後コメントは日本語 UTF-8 を基本とする。
- 時間制約の厳しい制御経路や割り込み内では、ブロッキング処理、長い delay、SD カード書き込み、表示更新、重いフォーマット処理を避ける。
- 組み込み制御コードでは、明確で正当な理由がない限り動的メモリ確保を避ける。
- チューニング定数には可能な範囲で明示的な単位を使う。
- モーター出力、センサー値、制御目標には、飽和処理、範囲チェック、フェイルセーフを追加する。
- 制御ロジックを変更するときは、センサーノイズ、エンコーダ分解能、IMU ドリフト、バッテリー電圧、モーター限界、制御周期を考慮する。

### 実装時のレビュー観点

- 制御周期に影響しないか。
- ログ形式を壊していないか。
- 実機安全条件に反していないか。

### 変更時の確認ルール

- 制御変更時は、ビルド確認と割り込み競合確認を行う。
- ログ形式変更時は、変更後最初のログを確認する。
- `.ioc` / STM32CubeMX 変更時は、再生成、ビルド、GPIO 機能競合を確認する。
- SD カード処理、回路・機体データ変更時の詳細手順は該当 Skill を使う。

### コード変更時に `AGENTS.md` 更新が必要な条件

- ログ列を追加、削除、改名、または意味変更したとき。
- 単位系を変更したとき。
- 走行モードを追加、削除、または意味変更したとき。
- ハード構成を変更したとき。
- 安全制約を変更したとき。

### テストと確認の期待値

- ファームウェア変更時は可能な場合は CMake ビルドを実行し、できない場合は理由を報告する。
- 実機なしでは十分に確認できない変更は、想定する実機確認手順を説明する。
- 現行ビルド設定は `-Wall` 有効、`-Werror` 無効。通常の変更では新規 warning を追加しない。

### Git とファイル管理

- 生成されたビルド成果物は編集・コミットしない。
- `robotrace_v2/build/Debug` と `robotrace_v2/build/Release` は Codex のビルド確認で更新される可能性があるが、生成物として扱う。
- 依頼されていない広範囲の整理やフォーマット変更は行わない。
- 作業完了前に、変更したファイルと確認状況を要約する。
- ドキュメント変更とコード変更は、依頼がない限り同じ作業単位で扱ってよい。
- コミットメッセージは日本語で記述する。

## 8. 割り込み・周期の責務

現在のコードから読み取れる主な周期処理の責務は以下です。

- `TIM6` は 1 ms 制御周期を担当し、`HAL_TIM_PeriodElapsedCallback()` から `Interrupt1ms()` を呼ぶ。
- `Interrupt1ms()` ではエンコーダ、ジャイロ、バッテリー、モーター、PID、走行制御、緊急停止、UI 用カウンタなどの主要な周期処理を行う。
- `Interrupt1ms()` 内では SD カードへの実書き込みは行わず、ログデータの蓄積や要求フラグ更新に留める。
- `TIM7` はログ書き込み要求の周期生成に使われ、`Interrupt100us()` から `logWriteReq` を立てる。
- 実際の SD ログ書き込みは `logWriteTask()` が担当し、`loopSystem()` 側で実行される。
- `TIM3` はラインセンサー、マーカーセンサーの LED 点灯と ADC 取得タイミングに使われる。
- `ADC1` 完了割り込みでは、マーカーセンサーとラインセンサーの ADC 値取得処理を行う。
- `TIM1`, `TIM2`, `TIM4`, `TIM5` はエンコーダ入力、PWM、ブザーなどのタイマ用途に使われる。

周期処理を変更する場合は、1 ms 制御周期への影響、割り込み内処理時間、SD 書き込みや表示更新との責務分離を確認する。

`TIM7` の関数名とコメントには `100 us` とあるが、現在の CubeMX 設定では実周期と一致しない可能性がある。この差分は実装未対応箇所として管理する。

## 9. パラメータ管理

速度、制御、センサー、緊急停止に関わる変更では、まず以下を確認します。

- 速度・加速度: `robotrace_v2/Core/Inc/control.h`, `robotrace_v2/Core/Src/control.c`, `robotrace_v2/Core/Src/setup.c` の `speedParamTable`, `./setting/targetSpeeds.txt`
- PID ゲイン、速度フィードフォワード: `robotrace_v2/Core/Inc/PIDcontrol.h`, `robotrace_v2/Core/Src/PIDcontrol.c`, `robotrace_v2/Core/Src/setup.c`, `./setting/line.txt`, `lineomega.txt`, `speed.txt`, `yawRate.txt`, `yaw.txt`, `dist.txt`, `speed_ff.txt`
- ラインセンサー校正・閾値: `robotrace_v2/Core/Inc/lineSensor.h`, `robotrace_v2/Core/Src/lineSensor.c`, `./setting/lsval.txt`
- スリップ判定、クロスライン検出、走行中ゲイン変更: `robotrace_v2/Core/Inc/control.h`, `robotrace_v2/Core/Src/control.c`
- 緊急停止条件: `robotrace_v2/Core/Inc/emergencyStop.h`, `robotrace_v2/Core/Src/emergencyStop.c`
- 走行モード、コース解析、速度計画: `robotrace_v2/Core/Inc/courseAnalysis.h`, `robotrace_v2/Core/Src/courseAnalysis.c`
- エンコーダ換算: `robotrace_v2/Core/Inc/encoder.h` の `PULSE_METER`, `PULSE_MILLIMETER`
- モーター PWM・電流: `robotrace_v2/Core/Inc/motor.h`, `robotrace_v2/Core/Src/motor.c`
- バッテリー電圧換算: `robotrace_v2/Core/Inc/battery.h` の `VBAT_R1`, `VBAT_R2`, `AD2VOLTAGE`

### エンコーダ・距離換算の正

- `robotrace_v2/Core/Inc/encoder.h` の `PULSE_METER = 53424`, `PULSE_MILLIMETER = 54.324F` を正とする。
- `encCurrentN` は左右エンコーダの平均で、1 ms あたりのパルス数とする。距離、速度、XY 座標、速度計画で距離換算を使う場合はこの換算値を基準にする。

`setup.c` には `yawRate`, `yaw`, `dist` の PID 調整画面実装がありますが、現在の表示切替では一部ケースがコメントアウトされており、通常メニューから到達しない可能性があります。

### チューニングパラメータ方針

- チューニングパラメータは見つけやすい場所に置き、意図が明らかでない変更には理由と単位を残す。
- PID、ラインセンサー閾値、マーカー検出、加速度、速度制限の調整手順は `.agents/skills/robotrace-tuning/SKILL.md` を使う。

## 10. SD カード内設定ファイル

`./setting/*.txt` は実機側で使う設定値の正とします。

- コード内デフォルト値よりも、SD カード内設定ファイルの値を優先する。
- SD カード未挿入でも走行は可能とする。ただし、警告を画面に表示する。
- SD カードが挿入されていて設定ファイルが存在しない場合は、対象ファイルを作成し、コード内デフォルト値を書き込む。
- 設定ファイルの読み書き処理を変更する場合は、ファイル欠落時にデフォルト値でファイルが作成されることを確認する。

設定ファイルの保存先は `./setting/` です。詳細なファイル形式、読み書き関数、破損時の扱いは `.agents/skills/robotrace-sd-settings/SKILL.md` を使います。

### 設定ファイル破損時の扱い

- 設定ファイルが途中まで読める場合は、読めた項目だけ部分反映する。
- 数値範囲外の項目は、コード内デフォルト値を使用する。
- 破損を検出した設定ファイルは、デフォルト値または有効値で上書き修復する。
- `lsval.txt` が破損している場合は走行禁止とする。
- 設定ファイルのパーサーを変更する場合は、部分反映、範囲外値のデフォルト使用、破損時の上書き修復、`lsval.txt` 破損時の走行禁止を維持する。

## 11. 初期化、校正、走行開始・終了

### 初期化失敗時の扱い

- SD カード初期化失敗時は、警告を画面に表示したうえで走行可能とする。
- IMU 初期化失敗時は走行禁止とする。
- ラインセンサー異常時は走行禁止とする。
- 電流センサー初期化失敗時は警告のみとし、走行は可能とする。
- 拡張 UI 基板が接続されていない場合でも走行可能とする。
- 初期化失敗時の挙動を変更する場合は、走行可否、警告表示、ログに残す情報、実機で確認すべきリスクを明示する。

### ラインセンサー校正手順

- `lsval.txt` が存在しない場合は走行前に校正する。校正時は超信地旋回を 1 回転行い、ラインを通過させる。
- 校正値は走行前のパラメータ保存タイミングで保存する。全センサーの校正値が `0` または最大値の場合は異常として扱う。
- `lsval.txt` の形式、破損時の扱い、保存確認は `.agents/skills/robotrace-sd-settings/SKILL.md` を使う。

### 走行開始・終了条件

- 走行開始条件は、UI 上でスタート開始動作を行ったときとする。
- `autoStart` は、UI 上でオートスタート開始動作を行ったときに実行する。
- ゴール判定条件は、ゴールマーカーを規定回数通過したときとする。
- 正常終了時、緊急停止時ともに、ログ保存タイミングは機体停止時とする。緊急停止ログでは `emcStop` に停止要因を記録する。
- 走行開始、ゴール判定、ログ終了処理を変更する場合は、正常終了と緊急停止のログ保存タイミングが変わらないか確認する。

### UI 操作とパラメータ保存タイミング

- 走行開始時、カウントダウン完了かつ IMU/電流センサーキャリブレーション完了後に、PID、速度フィードフォワード、目標速度を SD カードへ保存する。
- パラメータ保存は `autoStart <= 1` のときだけ行い、`autoStart >= 2` の連続走行中はスキップする。
- オートスタートは最大 5 走を前提とする。詳細な保存ファイルの扱いは `.agents/skills/robotrace-sd-settings/SKILL.md` を使う。

### 実機検証手順

- 書き込み後は、書き込み成功、`initSystem()` によるセンサー確認、必要な実機条件を確認する。
- 高速走行前は、センサー初期化成功と `7.5 V` 以上のバッテリー電圧を確認する。
- 低速確認は必須手順にはしない。次の速度へ上げる判断は、安定してゴールタイムが速くなったときとする。
- 詳細手順は `.agents/skills/robotrace-flash-debug/SKILL.md` を使う。

### 異常時に最初に見るログ項目

- `cntlog` が単調増加し、速度と距離基準ログ間隔に対して不自然な欠落がないか確認する。
- 目標速度と現在速度が大きく乖離していないか、現在速度が停止に近い速度になっていないか確認する。
- 異常ログの判定は `.agents/skills/robotrace-log-analysis/SKILL.md` を使う。

## 12. 走行モード、制御改善、チューニング

走行モードは `optimalTrace` で区別し、定義は `robotrace_v2/Core/Inc/courseAnalysis.h` の `BOOST_*` を正とします。

- `BOOST_NONE`: 1次走行。距離、角速度、マーカー、曲率半径を記録し、2次走行用のログを取る。
- `BOOST_MARKER`: マーカー位置によって区間速度を決定するモード。現在は使用していない。
- `BOOST_DISTANCE`: 1次走行の走行距離と比較し、現在の走行位置と曲率半径から速度を決定して走行する。
- `BOOST_SHORTCUT`: 1次走行の距離と角速度から走行コースをプロットし、最短経路を算出して走行する。イン側を走る、スラロームを直線として走行する、などを含む。

### 速度計画と確認観点

- `BOOST_DISTANCE` は `PPAD[optimalIndex].boostSpeed`、`BOOST_SHORTCUT` は `tgtParam.shortCut` と `shortCutxycie[optimalIndex]` を正とする。
- 各モードのログ確認観点と速度計画の評価は `.agents/skills/robotrace-log-analysis/SKILL.md` を使う。

### 制御出力の合成ルール

- `motorPwmOutSynth(tPwm, sPwm, yrPwm, dPwm)` は速度、トレース、ヨー、距離の各制御出力を左右 PWM に合成する。
- 右 PWM は `sPwm - tracePwm - yrPwm + dPwm` とする。
- 左 PWM は `sPwm + tracePwm + yrPwm + dPwm` とする。
- 左右 PWM はそれぞれ `-1000` から `1000` に飽和させる。
- 走行モードごとの出力経路を変更する場合は、`control.c` と `PIDcontrol.c` を確認する。

### スリップ判定とチューニング

- スリップ判定は `1 ms` 周期で、`patternTrace >= 12 && patternTrace < 100` の走行中に実行する。`encSpeed < 0.1 m/s` では判定をスキップする。
- `slipFlag` は縦スリップ、`slipFlagLat` は横スリップの判定フラグとする。
- 性能改善の優先順位は、再現性、完走率、ラップタイム短縮の順とする。
- 1 回の走行で変更してよいパラメータ数は最大 2 個までとする。
- PID、速度フィードフォワード、スリップ判定、速度・加速度の調整手順は `.agents/skills/robotrace-tuning/SKILL.md` を使う。

## 13. ログ、ファームウェアバージョン、解析

実機走行ログは `F:\Dropbox\Document\robotrace\Log\v2` に保存されています。

- ログファイルは CSV 形式、ヘッダ有り、文字コード UTF-8、区切り文字はカンマ。
- ログファイル名は通し番号を使う。
- ログスキーマは `robotrace_v2/Core/Inc/log_schema.h` を正とする。実ログ側に古い形式は混在しない前提で扱う。
- ログヘッダにはログデータ名とパラメータが含まれる。パラメータは `パラメータ名=value` 形式で記載される。
- 走行モードはログ内パラメータ `optimalTrace` で区別する。定義は `robotrace_v2/Core/Inc/courseAnalysis.h` の `BOOST_NONE`, `BOOST_MARKER`, `BOOST_DISTANCE`, `BOOST_SHORTCUT` を正とする。
- 新しい走行モードを追加する場合は、`robotrace_v2/Core/Inc/courseAnalysis.h` に定義を追加する。
- `emcStop` が 0 以外の場合は緊急停止しており、ゴールしていない走行として扱う。
- 緊急停止条件は `robotrace_v2/Core/Inc/emergencyStop.h` の `STOP_COUNT_ENCODER_STOP`, `STOP_COUNT_ANGLE_X`, `STOP_COUNT_TIME` などを正とする。
- 新しい緊急停止条件を追加する場合は、`robotrace_v2/Core/Inc/emergencyStop.h` に定義を追加する。
- ログ解析では、ラップタイム、速度追従、角速度、スリップを重視する。
- バッテリー状態はログ内パラメータ `batteryVoltage_V` を参照する。単位は `[V]`。
- ログ同士を比較する場合は、`batteryVoltage_V` の差を考慮する。電圧差によるモーター出力、速度追従、加速性能、スリップ傾向の変化を無視しない。
- ログ形式を安易に変更しない。形式変更が必要な場合は、スキーマまたはドキュメントも合わせて更新する。
- 詳細な解析手順は `.agents/skills/robotrace-log-analysis/SKILL.md` を使う。

### ログ解析、比較、出力

- ログ列の正、物理量、マーカー値、ログ周期、失敗走行分類、必須グラフ・表、解析出力命名は `.agents/skills/robotrace-log-analysis/SKILL.md` を使う。
- 失敗走行は `emcStop != 0` で除外する。`cntlog` 欠落や処理落ちがあるログも無効とし、原因を特定して修正する。
- `BOOST_DISTANCE` と `BOOST_SHORTCUT` は比較しない。1 次走行は 1 次走行同士、2 次走行は同じ種類同士、autoStart は 5 回すべて走行したログ同士で比較する。
- バッテリー電圧差が大きい場合は、充電して再トライする。
- 制御変更後は最低 10 本の実機走行ログを取り、変更前ログと比較する。採用判断は再現性を最優先する。
- 解析スクリプトは `analysis/script/` に置く。Python などの使用は可能。
- 解析グラフ、表、比較結果は `analysis/` に保存する。パラメータ変更内容はログヘッダには記載しない。
- ログには `fwVersion`, `gitCommit`, `buildDate`, `buildTime`, `branch` を残す。`fwVersion` は手動定義、その他はビルド時の自動埋め込みとする。

## 14. 方針と現行コードの差分管理

`AGENTS.md` の方針と現行コードが違う場合は、`AGENTS.md` を優先します。

- 実装未対応箇所は `AGENTS.md` に記述する。
- 修正時は、原則として方針へ合わせる。
- 方針自体が誤っていると判断した場合は、実装前に方針見直しを提案する。

### 実装未対応箇所の扱い

- 実装した項目は、実装未対応一覧から削除する。
- 実装済み履歴は `AGENTS.md` に残す。
- 実装未対応箇所の優先順位は、現時点では自動更新しない。

### AGENTS.md 方針に対する実装未対応箇所

現時点で、方針として明記済みだが実装未対応または要確認の項目は以下です。

- ログヘッダへのファームウェアバージョン情報出力。
  - 目標: `fwVersion`, `gitCommit`, `buildDate`, `buildTime`, `branch` をログに残す。
  - 優先順位: 1
- 設定ファイル欠落時の自動作成。
  - 目標: SD カードが挿入されていて設定ファイルが存在しない場合、デフォルト値で対象ファイルを作成する。
  - 優先順位: 2
- 設定ファイル破損時の修復。
  - 目標: 読み取り可能な値は部分反映し、範囲外値はデフォルトに戻し、破損を検出したファイルを上書き修復する。
- 初期化失敗時の走行禁止制御。
  - 目標: IMU 初期化失敗時とラインセンサー異常時は走行禁止にする。
  - 優先順位: 3
- `lsval.txt` 破損時の走行禁止制御。
  - 目標: ラインセンサー校正値が破損している場合は走行禁止にする。
- `TIM7` ログ要求周期の命名・コメント整理。
  - 現状: 関数名とコメントは `100 us` だが、CubeMX 設定上の実周期と一致しない可能性がある。
  - 目標: 実周期、関数名、コメント、ログ周期の関係を一致させる。

### 実装未対応箇所の対応優先順位

1. ログバージョン情報
2. 設定ファイル自動作成
3. 初期化失敗時の走行禁止

### `AGENTS.md` 更新タイミング

- 実装未対応箇所を実装したときは、必ず `AGENTS.md` を更新する。
- 機体・回路変更時は、`AGENTS.md` 記載のパラメータが変更されたときに必ず更新する。

### 実装済み履歴

- 2026-06-28: 誤字由来の識別子を正規化し、解析済みログ番号ファイル名を `analize.txt` から `analysis.txt` へ変更した。旧名の読み取りフォールバックは設けない。

## 15. 機体・回路変更時にコードへ反映する項目

機体・回路変更時は、STEP、KiCad、PDF、`.ioc`、関連ソースを確認し、確認できた事実と推測を分けて扱います。

- コードへ反映した場合は、関連する定義、ログスキーマ、パラメータ保存形式、`AGENTS.md` の記述に矛盾がないか確認する。
- 詳細な確認項目と報告ルールは `.agents/skills/robotrace-hardware-review/SKILL.md` を使う。
