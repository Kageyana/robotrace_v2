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

### 失敗事例の参照と記録

- 試行錯誤が必要なタスクでは、作業前または方針変更前に `D:\ドキュメント\codex\失敗事例` の既存ノートを確認し、同種の失敗がないか参照する。
- ビルド、テスト、ツール実行、設定、実装仮定、ログ解析、権限、外部連携で再利用価値のある失敗が起きた場合は、`record-failures` Skill の手順で失敗事例を記録する。
- 失敗事例には、発生条件、観測結果、原因または未確定事項、解決策、次回の予防策を分けて記載する。
- 一時的なタイプミスや単発の軽微な操作ミスは、再発防止に役立つ場合だけ記録する。

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

- `batteryVoltage_V` は `AD2VOLTAGE(batteryAD)` の有効サンプルを1ms周期でLPF更新した電圧 `[V]` とする。
- `batteryVoltage_V` はログヘッダへ出力し、モーター電圧指令から実DUTYを算出する補償にも使用する。
- 高速走行前の下限 `7.5 V` は運用上のチェック値とする。
- 調整時のログ比較や採用判断は、ログヘッダの走行開始時点 `batteryVoltage_V >= 7.3 V` を満たす走行を対象にする。これは調整時のプロジェクト運用値であり、Skill に記載された既定の7.6V条件より優先する。走行中の `batteryVoltage_mV` が7.3V未満へ低下しても、この適否判定からは除外しない。
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
- 関数を追加するときは、関数定義の直前に既存コードと同じ関数概要コメントを日本語で記載する。基本書式は以下とする。

```c
/////////////////////////////////////////////////////////////////////
// モジュール名 関数名
// 処理概要     処理内容を日本語で記載
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
```

- 引数や戻り値がある場合は、`引数名: 説明[単位]`、`戻り値       説明` の形で記載する。周辺コードで空白やタブの揃え方が異なる場合は、同じファイル内の既存書式を優先する。
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
- `TIM7` は 0.5 ms 周期のログ書き込み要求タイマとして使われ、`Interrupt500us()` から 10 分周で 5 ms ごとに `logWriteReq` を立てる。
- 実際の SD ログ書き込みは `logWriteTask()` が担当し、`loopSystem()` 側で実行される。
- `TIM3` はラインセンサー、マーカーセンサーの LED 点灯と ADC 取得タイミングに使われる。
- `ADC1` 完了割り込みでは、マーカーセンサーとラインセンサーの ADC 値取得処理を行う。
- `TIM1`, `TIM2`, `TIM4`, `TIM5` はエンコーダ入力、PWM、ブザーなどのタイマ用途に使われる。

周期処理を変更する場合は、1 ms 制御周期への影響、割り込み内処理時間、SD 書き込みや表示更新との責務分離を確認する。

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
- 新しい試行を始める前に、`D:\ドキュメント\codex\失敗事例` の過去失敗事例を参照し、同じパラメータ、電圧条件、ログ除外条件、再発要因を確認する。
- 試行が不採用、失敗、ビルド・テスト・実機連携エラーになり、再利用価値のある知見が得られた場合は、`record-failures` SkillのテンプレートでObsidianへ直ちに記録する。原因未確定でも、事実、除外・回避策、次の確認項目を記録する。
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

- `motorCommandOutSynth(tCmd, sCmd, yrCmd, dCmd)` は速度、トレース、ヨー、距離の各制御出力を左右の正規化指令に合成し、出力層で電圧補償して実DUTYへ変換する。
- 正規化指令 `cmd=1000` は `MOTOR_COMMAND_NOMINAL_V=7.1 V` のモーター指令電圧として扱う。
- 右正規化指令は `sCmd - traceCmd - yrCmd + dCmd` とする。
- 左正規化指令は `sCmd + traceCmd + yrCmd + dCmd` とする。
- 左右の正規化指令と実DUTYはそれぞれ `-1000` から `1000` に飽和させる。
- 走行モードごとの出力経路を変更する場合は、`control.c` と `PIDcontrol.c` を確認する。

### スリップ判定とチューニング

- スリップ判定は `1 ms` 周期で、`patternTrace >= 12 && patternTrace < 100` の走行中に実行する。`encSpeed < 0.1 m/s` では判定をスキップする。
- `slipFlag` は縦スリップ、`slipFlagLat` は横スリップの判定フラグとする。
- `slipLongResidual_mps2` は縦方向、`slipLatResidual_mps2` は横方向のスリップ判定用加速度残差 `[m/s^2]` とする。
- スリップ判定理由を追う内部デバッグ値は制御内で保持するが、現行ログ列には `x`, `y` の後ろの詳細デバッグ列としては出力しない。
- `motorpwmL`, `motorpwmR` は実際にモーターへ出力した飽和後 PWM とする。
- 性能改善の優先順位は、再現性、完走率、ラップタイム短縮の順とする。
- 1 回の走行で変更してよいパラメータ数は最大 2 個までとする。
- 比較用に変更したパラメータまたは定数を不採用として既知の基準値へ戻すだけの場合は、戻し操作そのものの実走行確認は原則不要とする。戻し後の値は SD 読み戻しまたはコード確認で確認する。
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
- `batteryVoltage_mV` は走行中にLPF更新したバッテリー電圧 `[mV]`、`motorVoltageCmdL_mV`, `motorVoltageCmdR_mV` はバッテリー電圧で割る前の左右モーター指令電圧 `[mV]` とする。
- `motorpwmL`, `motorpwmR` は電圧補償後に実際にタイマへ出力した飽和後DUTYとする。
- 通常ログは `LOG_SCHEMA_PROFILE_LIGHT=1` を既定とし、ラップタイム、速度追従、角速度、マーカー、スリップフラグ、電圧指令、実DUTY、XY確認に必要な列だけを残す。
- 加速度、電流、スリップ内部量などの詳細デバッグ列が必要な場合は、ビルド定義で `LOG_SCHEMA_PROFILE_LIGHT=0` にして一時的に出力する。
- ログ同士を比較する場合は、走行開始時点の `batteryVoltage_V` の差を考慮する。走行中の `batteryVoltage_mV` は速度追従、加速性能、PWM飽和、スリップ傾向の原因分析に使うが、7.3V適否判定の基準にはしない。
- ログ形式を安易に変更しない。形式変更が必要な場合は、スキーマまたはドキュメントも合わせて更新する。
- 1次走行ログを読む `courseAnalysis.c` には先頭列順を前提にした処理が残っているため、`cntlog`, `encCurrentN`, `gyroVal_Z`, `courseMarker`, `encTotalOptimal`, `ROC` の6列の前には新しいログ列を挿入しない。
- 2次ログ再解析の `readLogDistanceSlip()` はヘッダ名から必要列を特定する。ログ列追加時は `courseMarker`, `encTotalOptimal`, `ROC`, `targetSpeed`, `optimalIndex`, `slipFlag`, `slipFlagLat` が読めることを確認する。
- `readLogDistanceSlip()` の速度計画反映では、`slipFlag` は軽い減速、`slipFlagLat` は安全側の強い減速として別強度で扱う。
- `readLogDistanceSlip()` は、横スリップが無い高速度帯ストレートでは低速カーブ側の減速伝播で速度計画を落とし過ぎないよう下限を設ける。
- 高速度帯ストレート下限のストレート判定は、2次ログの実測 `ROC` ではなく、1次ログから作った基準速度計画の `ROC` を使う。
- 高速度帯ストレートへ速度変化制限が連続伝播する場合の `v2Max * 0.94` 回復処理は無効化し、最終段の `v2Max * 0.92` ストレート下限だけを使う。
- 詳細な解析手順は `.agents/skills/robotrace-log-analysis/SKILL.md` を使う。

### ログ解析、比較、出力

- ログ列の正、物理量、マーカー値、ログ周期、失敗走行分類、必須グラフ・表、解析出力命名は `.agents/skills/robotrace-log-analysis/SKILL.md` を使う。
- 失敗走行は `emcStop != 0` で除外する。`cntlog` 欠落や処理落ちがあるログも無効とし、原因を特定して修正する。
- `BOOST_DISTANCE` と `BOOST_SHORTCUT` は比較しない。1 次走行は 1 次走行同士、2 次走行は同じ種類同士で比較する。
- `autoStart` の連続走行では、前走のスリップを使って次走の目標速度が更新される。一般に2走目は1次走行を基準に高め、3走目は2走目のスリップ反映で低め、4走目はその反動で高め、5走目は4走目のスリップ反映で低めになり得る。
- `autoStart` の評価は周回番号ごとに行う。`autoStart=2`, `3`, `4`, `5` を別々に比較し、異なる周回番号を混ぜた平均ラップやセット平均を採用判断の根拠にしない。`autoStart=1` は1次走行の基準・妥当性確認として別扱いにする。
- 5走すべてが完走していないautoStartセットは、完走した個別ログがあっても採用比較からセット全体を除外する。
- 2次走行の標準評価では、同じautoStartセットの1次走行ログからコース形状、距離、曲率を生成し、各2次走行の`targetSpeed`へ追従した場合の理論ラップを算出する。速度目標の変更は瞬時切替とせず、スタートマーカー通過後の最初のログ行にある`encCurrentN / PULSE_MILLIMETER`をその走行の実測初速とし、各ログヘッダの`tgtParam.acceleF/acceleD`で区間内の加減速時間を積算する。実測側は既存ラップ定義の最終`cntlog`とし、`theoretical_lap_ms`, `lap_gap_ms`, `lap_gap_pct`を周回番号ごとに比較する。従来の瞬時切替値は`instantaneous_theoretical_lap_ms`として併記する。
- 理論ラップ計算は`analysis/script/theoretical_lap.py`を正とし、標準レポートは`analysis/script/theoretical_lap_report.py`を使う。PPAD相当区間は1次ログを5サンプル単位で分割し、区間の実距離は`encTotalOptimal / PULSE_MILLIMETER`から求める。
- バッテリー電圧差が大きい場合は、充電して再トライする。
- 制御変更後は最低 10 本の実機走行ログを取り、変更前ログと比較する。採用判断は再現性を最優先する。
- 解析スクリプトは `analysis/script/` に置く。Python などの使用は可能。
- 解析グラフ、表、比較結果は `analysis/` に保存する。パラメータ変更内容はログヘッダには記載しない。
- ログには `fwVersion`, `gitCommit`, `buildDate`, `buildTime`, `branch` を残す。`fwVersion` は手動定義、その他はビルド時の自動埋め込みとする。
- ログ負荷切り分けでは、`LOG_SCHEMA_PROFILE_LIGHT=1` の軽量ログプロファイルを使う。軽量プロファイルでも、先頭6列 `cntlog`, `encCurrentN`, `gyroVal_Z`, `courseMarker`, `encTotalOptimal`, `ROC`、2次ログ再解析に必要な `targetSpeed`, `optimalIndex`, `slipFlag`, `slipFlagLat`、および `x/y` 算出に使う `encCurrentCorr_p` は維持する。
- 軽量ログ比較では、ログヘッダ `logRecordSizeBytes`, `logBufferSizeBytes`, `logBufferRecordCapacity`, `dbgOverflowFinal`, `logOverflowFinal` を確認し、ログ量によるバッファ不足やSD書き込み遅延の影響を切り分ける。
- 現行の常用検証ログ上限は `36 bytes` とする。通常の `LOG_SCHEMA_PROFILE_LIGHT=1` はダミー6 bytesを含む36B基準ログとし、追加の診断列が必要な場合だけ36B以内で一時的に既存列やダミー列と入れ替える。
- 36Bを超える常時ログ列追加は行わない。`10445`〜`10453` の2セットで、36B dummyはoverflow/gapなし、lap、PWM飽和、直線 `lineTraceCtrl` が実用範囲に戻ることを確認済みとする。
- 2次走行生成までに必要なログ列は常時維持する。その他の診断列は各タスクで一時的に入れ替えてよいが、タスク終了後は36B dummy基準ログへ戻す。
- 調整終了後の現行ファームは36B dummy基準ログを使う。profile 5/6/7/8 の一時診断列、局所cap/局所scale、差分正規化圧縮の不採用コードは常時ファームから外す。
- 速度PID調整では、ログヘッダの `veloCtrl.kp`, `veloCtrl.ki`, `veloCtrl.kd` に加えて、走行中の左右独立速度制御で実際に使う `veloCtrlL.*`, `veloCtrlR.*` を確認する。
- 走行中ゲイン切替の評価では、ログ列 `lineTraceOmegaFBCtrlkp`, `lineTraceOmegaFBCtrlkd` を使って実際に使われたライン角速度FBゲインを確認する。
- 高速度帯の速度追従評価では、ログ列 `targetSpeedL`, `targetSpeedR`, `speedTargetClip` を使って左右独立速度制御の実目標値とクリップ有無を確認する。
- 速度伝播保護の評価では、ログヘッダ `speedPropagateProtectSteps`, `speedPropagateRecoverScale` を使って、速度変化制限の保護範囲と回復下限を確認する。
- 速度FF内訳、速度PID単体出力、飽和前速度PWM要求値、PWM飽和分類用の詳細列は現行ログには常時出力しない。必要になった場合だけ、36B以内で既存列またはダミー列と一時的に入れ替える。
- PWM飽和評価では、単純な `motorpwmL/R` 飽和率だけで採否を決めない。速度プロファイル由来の加減速中飽和は許容し、`targetSpeed` 前後差、`lineTraceCtrl`、`gyroVal_Z`、横スリップ、lap悪化との相関で判断する。
- profile 5/6/7/8 などのPWM飽和分類・ライン角速度目標診断ログは一時診断ログとして扱い、タスク終了後は36B標準ログへ戻す。
- `emcStop=5` の診断用に緊急停止カウンタ列は常時保持しない。停止要因を追う場合は、通常ログ列の `gyroVal_Z`, `lineTraceCtrl`, `courseMarker`, `x/y`, `emcStop` を優先し、追加診断列が必要なときだけ36B以内で一時的に入れ替える。
- ライン角速度目標の局所cap比較では、`idx10/11/23` だけ速度条件に依存せず `targetAngularvelo` を制限し、`targetAngularveloRaw` と `targetAngularvelo` の差から局所cap適用量を評価する。cap適用範囲のグローバル拡大とは分けて判断する。
- ライン角速度目標の局所スケール比較では、`idx10/11/23/24` だけ `targetAngularveloRaw` から `targetAngularvelo` へのスケール量を見て、hard capより弱い目標丸めで横スリップを下げられるか評価する。`0.90` 比較は不採用であり、今後の調整では index 限定の局所cap/局所scale/局所速度capを原則避ける。
- ライン角速度目標の差分正規化圧縮比較では、`abs(lineOmegaSensorDiff) / lineOmegaSensorSum` が大きい低速カーブ全体で `targetAngularveloRaw` を連続的に丸める。特定 `optimalIndex` の直接指定ではなく、profile 7 の `lineOmegaSensorDiff/Sum`, `targetAngularveloRaw`, `targetAngularvelo` から圧縮候補率、適用率、スケール比率を評価する。
- 不採用になったパラメータや定数を既知の基準値へ戻すだけの場合は、コード確認またはSD読み戻しで復帰確認とし、戻し後だけの実走行確認は必須にしない。
- 速度計画低下の切り分けで使っていた `targetSpeedBase`, `targetSpeedScale` は現行の常時ログには出力しない。再度必要になった場合だけ、36B以内でダミー列と一時的に入れ替える。
- `STOP_ENCODER_STOP` の左右エンコーダ速度ラッチによる切り分けは行わない。`emcStop=3` は平均速度停止条件として扱い、必要時は停止判定カウンタや走行ログの通常列で確認する。

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

現時点で、方針として明記済みだが実装未対応または要確認の項目はありません。

### 実装未対応箇所の対応優先順位

現時点では未対応項目がないため、優先順位は設定しません。

### `AGENTS.md` 更新タイミング

- 実装未対応箇所を実装したときは、必ず `AGENTS.md` を更新する。
- 機体・回路変更時は、`AGENTS.md` 記載のパラメータが変更されたときに必ず更新する。

### 実装済み履歴

- 2026-06-28: 誤字由来の識別子を正規化し、解析済みログ番号ファイル名を `analize.txt` から `analysis.txt` へ変更した。旧名の読み取りフォールバックは設けない。
- 2026-06-28: ログヘッダへ `fwVersion`, `gitCommit`, `buildDate`, `buildTime`, `branch` を出力するようにした。設定ファイル欠落時のデフォルト作成、破損時の部分反映と修復、`lsval.txt` 破損時および IMU/ラインセンサー異常時の走行禁止、TIM7 ログ要求周期の命名整理を実装した。
- 2026-06-28: スリップ判定で実PWMを参照するようにし、IMUバイアス更新を低動的状態に限定した。ログ列 `slipRatio`, `slipRatioLat` は `slipLongResidual_mps2`, `slipLatResidual_mps2` へ変更した。
- 2026-06-28: スリップ判定のデバッグ列 `slipThresholdHigh`, `slipThresholdLow`, `slipLatHigh`, `slipLatLow`, `slipCurScale`, `slipTurningState`, `slipAxBias`, `slipAyBias` を追加し、オフライン評価用スクリプトを追加した。
- 2026-06-29: 縦スリップの低負荷誤検出抑制として、速度誤差と電流によるONカウントゲートを追加し、ログ列 `slipLongOnCountEnabled` を追加した。
- 2026-06-30: 縦スリップON後に低負荷が継続した場合の保持解除を追加し、ログ列 `slipLongLowloadClearCount` を追加した。
- 2026-06-30: PID調整のため直線用 `lineTraceOmegaFBCtrl` ゲインを定数化して `kp=1`, `kd=0` に変更し、SDから読んだ `veloCtrl` を `veloCtrlL/R` へ同期、ログヘッダへ `veloCtrlL.*`, `veloCtrlR.*` を追加した。
- 2026-07-01: ラインPID再評価のため直線用 `lineTraceOmegaFBCtrl` ゲインを `kp=2`, `kd=1` に変更し、ログ列 `lineTraceOmegaFBCtrlkd` を追加した。
- 2026-07-01: カーブ用ゲインが直線区間に残る時間を短くするため、直線復帰判定の先読みを `1`、直線安定距離を `10 mm` に変更した。
- 2026-07-01: ログ容量を抑えるため、`x`, `y` の後ろに出力していたスリップ詳細デバッグ列をログスキーマから削除した。
- 2026-07-02: 高速度帯の左右独立速度目標クリップを評価するため、片輪目標上限を `5.0 m/s` 相当に変更し、ログ列 `targetSpeedL`, `targetSpeedR`, `speedTargetClip` を追加した。
- 2026-07-02: 高速度帯の速度FF評価のため、速度比例FF、Crr相当FF、合計FF、PID単体出力を左右別に記録するログ列を追加し、速度比例FF係数 `SPEED_FEEDFORWARD_VEL_SCALE` を追加した。
- 2026-07-02: 高速度帯の速度追従改善試験として、速度比例FF係数 `SPEED_FEEDFORWARD_VEL_SCALE` を `1.10f` に変更した。
- 2026-07-03: `SPEED_FEEDFORWARD_VEL_SCALE` を `1.00f` に戻し、PWM飽和前要求値と左右速度偏差を切り分けるため、ログ列 `speedErrL/R`, `speedPwmReqL/R`, `speedPwmSatL/R` とログヘッダ `speedFfVelScale` を追加した。
- 2026-07-03: 高速度帯だけ速度比例FFを増やす試験として、`3300-3500 mm/s` で `1.00` から `1.15` へ線形補間する `SPEED_FEEDFORWARD_HIGH_VEL_SCALE` を追加し、ログヘッダへ高速度帯FF設定を追加した。
- 2026-07-03: 高速度帯限定 `SPEED_FEEDFORWARD_HIGH_VEL_SCALE=1.15f` は速度偏差が改善せず不採用とし、`1.00f` へ戻した。直線最高速度過大の切り分けとして、SD設定の `bstStraight=3.30 m/s` 試験を開始した。
- 2026-07-04: 充電後ログ `9930`〜`9943` で高速度帯インデックスの実速度維持とPWM余裕改善を確認し、SD設定の `bstStraight=3.30 m/s` を現行採用候補とした。次回は `120-159 pulse/ms` 帯のPWM飽和再現性を確認する。
- 2026-07-04: 追加ログ `9945`〜`9959` で `bstStraight=3.30 m/s` の再現性と中速帯PWM余裕改善を確認し、現行設定として継続採用した。緊急停止診断用に停止判定カウンタのログ列を追加した。
- 2026-07-04: `encCurrentL/R` 追加後のログ `9960`〜`9974` で2次走行の `targetSpeed` が固定化したため、1次走行解析の固定列順と互換になるよう `encCurrentL/R` を `ROC` の後ろへ移動した。
- 2026-07-04: 2次ログ再解析で追加ログ列を誤読しないよう、`readLogDistanceSlip()` をヘッダ名ベースの列参照へ変更した。
- 2026-07-04: `readLogDistanceSlip()` の速度計画反映で縦スリップと横スリップを別強度に分離し、縦スリップ単独では軽い減速に留めるよう変更した。
- 2026-07-04: `readLogDistanceSlip()` の速度変化制限後、横スリップが無い高速度帯ストレートに `v2Max * 0.95` の下限を追加した。
- 2026-07-04: 高速度帯ストレート下限の判定を2次ログ実測ROCから1次ログ由来の基準ROCへ変更した。
- 2026-07-04: 高速度帯ストレート下限によるPWM飽和とライン制御負荷を下げる比較試験として、下限スケールを `0.92` に変更した。
- 2026-07-05: 下限スケール `0.92` はPWM飽和とライン制御負荷を下げた一方で高速度帯targetが下限ぎりぎりだったため、中間比較として `0.93` に変更した。
- 2026-07-05: 下限スケール `0.93` は高速度帯targetを戻したがlapと直線指標が改善しなかったため `0.92` に戻し、速度計画低下確認用の `targetSpeedBase`, `targetSpeedScale` を追加した。
- 2026-07-05: `0.92` 復帰後の充電ログで高速度帯targetが約92%に張り付いたため、速度変化制限の高速度帯ストレートへの連続伝播を3 index目以降だけ `0.94` 下限で制限した。
- 2026-07-05: 伝播保護 `steps=2` は高速度帯targetを戻したが直線 `gyroVal_Z` と `lineTraceCtrl` が悪化したため、保護を弱める比較として `CA_SLIP_SPEED_PROPAGATE_PROTECT_STEPS=3` へ変更し、ログヘッダへ `speedPropagateProtectSteps`, `speedPropagateRecoverScale` を追加した。
- 2026-07-05: 伝播保護 `steps=3` は充電後2セットでlap、PWM飽和、直線 `gyroVal_Z`、`lineTraceCtrl` が悪化したため不採用とし、`CA_SLIP_SPEED_PROPAGATE_PROTECT_STEPS=OPT_BUFF_SIZE` で `0.94` 伝播回復を無効化して `0.92` 基準へ戻した。
- 2026-07-05: `0.94` 伝播回復無効化後も低速カーブ側のPWM飽和とライン制御負荷が残るため、横スリップ時だけ少し安全側へ寄せる比較として `CA_SLIP_DOWN_LAT_EXTRA=0.35f` に変更した。
- 2026-07-05: `CA_SLIP_DOWN_LAT_EXTRA=0.35f` は採用せず `0.30f` へ戻した。ログ負荷切り分けのため `LOG_SCHEMA_PROFILE_LIGHT=1` を既定にし、軽量ログ列と `logRecordSizeBytes` / `dbgOverflowFinal` / `logOverflowFinal` などの診断ヘッダを追加した。
- 2026-07-05: ログサイズ悪化閾値を調べるため `LOG_SCHEMA_LOAD_TEST_LEVEL` を追加した。次回比較用の既定値は `1` とし、ログレコードサイズを `64 bytes` 相当にする。
- 2026-07-08: `64 bytes` ログで処理落ちやoverflowは出ないがlapと直線指標の悪化が再現したため、中間比較用に `LOG_SCHEMA_LOAD_TEST_LEVEL=4` を追加し、ログレコードサイズを `48 bytes` 相当にした。
- 2026-07-08: `48 bytes` ログでもPWM飽和、直線 `lineTraceCtrl`、`emcStop=5` が残ったため、次の中間比較用に `LOG_SCHEMA_LOAD_TEST_LEVEL=5` を追加し、ログレコードサイズを `40 bytes` 相当にした。
- 2026-07-08: `40 bytes` ログでも直線 `lineTraceCtrl` と中速帯PWM飽和が `33 bytes` 基準まで戻らなかったため、中間比較用に `LOG_SCHEMA_LOAD_TEST_LEVEL=6` を追加し、ログレコードサイズを `36 bytes` 相当にした。
- 2026-07-08: `36 bytes` ログは完走2セットで40Bより改善し33Bに近いため、現行の検証ログ上限候補とした。`emcStop=5` 診断では、必要に応じて36B内のダミー領域をライン未検出カウンタへ一時置換する方針とした。
- 2026-07-09: `emcStop=5` 診断列版36Bはlap、PWM飽和、直線 `lineTraceCtrl` が悪化したため常用不採用とし、`LOG_SCHEMA_LOAD_TEST_LEVEL=6` をダミー3 bytesの36B基準ログへ戻した。
- 2026-07-12: 36B dummy復帰後の追加ログ `10445`〜`10453` で2セットとも `emcStop=0`、overflow/gapなしを確認した。前回低電圧ログ `10435`〜`10443` の悪化はログサイズ起因とは見ず、常用検証ログ上限を36B dummy構成に固定する。
- 2026-07-12: 2次走行生成までに必要なログ列を常時維持し、その他の診断列はタスク単位で一時入れ替え、タスク終了後に36B dummy基準ログへ戻す運用を追加した。
- 2026-07-12: パラメータ調整へ戻る前に、ログサイズ調査用の `LOG_SCHEMA_LOAD_TEST_LEVEL` と40B以上の負荷試験ダミー列を削除し、36B dummy基準ログだけを残した。

- 2026-07-20: カーブ側 `lineTraceOmegaFBCtrl.kd=13` は不採用、`kd=12` を現行固定とした。`kd=12` で残る直線指標悪化の切り分けとして、直線用ゲイン復帰距離 `GAIN_CHANGE_STRAIGHT_STABLE_MM` を `5 mm` に変更した。
- 2026-07-20: 直線用ゲイン復帰距離 `5 mm` は、target curve PWM飽和と `lineTraceCtrl` が悪化したため不採用とし、`GAIN_CHANGE_STRAIGHT_STABLE_MM=10 mm` へ戻した。
- 2026-07-20: カーブ側 `lineTraceOmegaFBCtrl` は `lineomega.txt=004,000,012` を現行固定とした。ログ `10570`〜`10588` の有効2次走行16本で、target curve PWM飽和、直線 `gyroVal_Z`、直線 `lineTraceCtrl`、lapが `kp=5,kd=12` 基準から改善した。
- 2026-07-20: `lineomega.txt=004,000,012` 固定後も `optimalIndex=10,11,23` 付近の横スリップ率が高いため、横スリップ時の2次速度計画を少し安全側へ寄せる比較として `CA_SLIP_DOWN_LAT_EXTRA=0.33f` に変更した。
- 2026-07-20: `CA_SLIP_DOWN_LAT_EXTRA=0.33f` は横スリップ率が下がらず、PWM飽和と `lineTraceCtrl` が悪化したため不採用とし、`0.30f` へ戻した。
- 2026-07-20: 横スリップ原因切り分けのため、36B検証ログから `targetSpeedBase`, `targetSpeedScale`, `lineTraceOmegaFBCtrlkp/kd`, ダミー3Bを一時的に外し、`slipLatResidual_mps2`, `slipLatHigh` を記録する。タスク終了後は36B標準ログへ戻す。
- 2026-07-22: 横スリップ診断列は役割完了のため36B標準ログへ戻した。`idx10/11/23` の実横スリップ残差が大きいため、同indexかつ `abs(baseRoc) <= 160 mm` の区間だけ `CA_SLIP_LOCAL_LAT_SPEED_SCALE=0.85f` で局所速度capする比較へ進む。
- 2026-07-22: 局所速度cap単発適用は `idx10/11/23` の横スリップ改善が薄く、走行ごとのON/OFF交互挙動が見えたため、`CA_SLIP_LOCAL_LAT_HOLD_RUNS=2` を追加してスリップ後2回分capを保持する比較へ進む。
- 2026-07-22: `CA_SLIP_LOCAL_LAT_HOLD_RUNS=2` は横スリップ低減に効いた一方でlap、PWM飽和、直線 `gyroVal_Z` / `lineTraceCtrl` が悪化したため不採用とし、保持を弱めた `CA_SLIP_LOCAL_LAT_HOLD_RUNS=1` の比較へ進む。
- 2026-07-23: 局所横スリップcap方式は単発、保持2走、保持1走のいずれも横スリップ低減とlap/直線指標を両立できなかったため、`CA_SLIP_LOCAL_LAT_ENABLE=0` で無効化する。36BログはPWM飽和分類用に `targetSpeedScale` とダミー3Bを外し、`targetAngularvelo`, `speedPwmSatL/R` を追加する。
- 2026-07-23: PWM飽和分類 profile 1 では `unknown_motor_sat` が多く残ったため、36Bログから `targetAngularvelo`, `lineTraceOmegaFBCtrlkp/kd` を一時的に外し、`speedPwmReqL/R` を追加する profile 2 へ切り替えた。
- 2026-07-26: profile 2ログ `10690`〜`10709` で `unknown_motor_sat=1.42%` が残り、`speedPwmReqL/R` が低いのに `motorpwmL/R` が飽和する行があるため、36Bログから `targetSpeedBase`, `speedPwmSatL/R` を一時的に外し、`speedPwmOutL/R` を追加する profile 3 へ切り替えた。
- 2026-07-26: profile 3ログ `10723`〜`10731` で `motor_out_mismatch=2.45%` が残ったため、`speedPwmOutL/R` をログ時点の `veloCtrlL/R.pwm` 直接参照から、出力適用時にラッチした速度PWM値へ変更する profile 4 へ切り替えた。
- 2026-07-26: profile 5ログ `10747`〜`10758` でPWM飽和分類の `unknown_motor_sat=0%` を確認した。`idx10/11/23` の `targetAngularvelo - gyroVal_Z` 乖離と横スリップを切り分けるため、`BOOST_DISTANCE` の低速カーブ時だけライン角速度目標を `1200 deg/s` に制限する比較へ進む。
- 2026-07-26: `lineomega.txt=005,000,012` と `LINE_TRACE_OMEGA_TARGET_CAP_ABS_DEG_S=1200` を比較ベースにし、`idx10/11/23` に残る `targetAngularvelo > 1200` と横スリップを下げるため、cap適用速度範囲を `90` から `95 pulse/ms` へ広げる比較へ進む。
- 2026-07-27: `LINE_TRACE_OMEGA_TARGET_CAP_MAX_SPEED_PULSE=95` は `emcStop=3` と `idx10/11/23` 横スリップ再発があり採用保留/不採用寄りとし、`90 pulse/ms` へ戻す。36Bログは profile 6 として `targetAngularveloRaw` を追加し、cap適用漏れとcap後目標を比較する一時診断ログへ切り替える。
- 2026-07-27: `max90 + profile6` ログ `10820`〜`10828` で `idx10/11/23` の `cap_missed_due_speed=25.64%` が残り、missed行は主に `targetSpeed=91-92 pulse/ms` だったため、cap適用範囲の中間比較として `LINE_TRACE_OMEGA_TARGET_CAP_MAX_SPEED_PULSE=93` へ変更する。
- 2026-07-27: `LINE_TRACE_OMEGA_TARGET_CAP_MAX_SPEED_PULSE=93` は `idx10/11/23` のcap漏れと横スリップを下げたが、lap、直線 `gyroVal_Z`、target curve `lineTraceCtrl` が悪化したため不採用とし、`90 pulse/ms` へ戻す。cap適用範囲拡大は打ち切り、次はラインセンサー目標または局所経路側を切り分ける。
- 2026-07-27: `max90` 復帰ログ `10850`〜`10858` で復帰成功を確認した。`idx10/11/23` の横スリップは残るがcap適用範囲拡大は打ち切り、36B profile 7 として `lineOmegaSensorDiff/Sum` を記録し、ライン角速度目標のセンサー内訳を診断する。
- 2026-07-27: profile 7ログ `10860`〜`10868` では `lineOmegaSensorSum` は低すぎず、`lineOmegaSensorDiff/Sum` が大きいことを確認した。次は `idx10/11/23` 限定で、速度条件に依存しない `1200 deg/s` のライン角速度目標capを比較する。
- 2026-07-27: `idx10/11/23` 限定ライン角速度目標capは横スリップを下げたが、lap、直線 `gyroVal_Z`、直線 `lineTraceCtrl`、target curve `lineTraceCtrl` が悪化したため不採用とし、`LINE_TRACE_OMEGA_LOCAL_TARGET_CAP_ENABLE=0` へ戻して `max90 + profile7` 復帰を確認する。
- 2026-07-29: `lineomega.txt=006,000,012` は横スリップと直線指標が悪化したため不採用とし、`005,000,012` へ戻した。復帰後ログで `emcStop=3/5` が出たため、36B内の一時診断列で `emcStop=5` のライン未検出要因を切り分けた。
- 2026-07-29: profile 8診断ログ `10917`〜`10936` で `emcStop=5` は再発しなかったため停止条件は変更しない。36Bログをprofile 7へ戻し、`idx10/11/23/24` 限定でライン角速度目標を `0.90` 倍にする比較へ進む。
- 2026-07-30: 局所ライン角速度目標スケール `0.90` はログ `10938`〜`10946` で横スリップ改善が再現しなかったため不採用とし、`LINE_TRACE_OMEGA_LOCAL_TARGET_SCALE_ENABLE=0` へ戻した。次は局所対応ではなく、ラインセンサー目標生成やカーブ全体に効く制御側を検討する。
- 2026-07-31: 復帰後ログ `10948`〜`10956` で局所スケール無効化と profile 7 の正常動作を確認した。局所対応を避ける方針に合わせ、低速カーブ全体で `abs(lineOmegaSensorDiff) / lineOmegaSensorSum` が大きいときだけライン角速度目標を最大90%まで連続圧縮する比較へ進む。
- 2026-07-31: 差分正規化圧縮はログ `10958`〜`10966` で横スリップ、yaw誤差、PWM飽和、直線指標が改善しなかったため不採用とし、`LINE_TRACE_OMEGA_DIFF_COMPRESS_ENABLE=0` へ戻した。次は局所index指定を避け、低速小Rカーブ全体で曲率由来FFとラインセンサーFB縮小を分離する比較へ進む。
- 2026-07-31: 曲率FF実装レビューで、`targetSpeed=0` 時はライン角速度目標のFB縮小を行わないよう修正した。profile 7解析では、曲率FF適用率をcap後差分ではなく曲率FFのpre-cap推定差分で評価し、最終cap込み差分とは分けて集計する。
- 2026-07-31: 曲率由来FF `LINE_TRACE_OMEGA_CURVE_FF_ENABLE=1`, `LINE_TRACE_OMEGA_CURVE_FF_GAIN_PCT=100`, `LINE_TRACE_OMEGA_CURVE_FB_SCALE_PCT=35` を採用した。完走ログ `10968`〜`10991` の比較でlapと `idx10/11/23/24` 横スリップ低減を優先し、`lineTraceCtrl` RMS増加は許容扱いとする。
- 2026-08-01: 調整終了に伴い、一時的に入れた局所横スリップ速度cap、局所ライン角速度cap/scale、差分正規化圧縮、profile 7ライン角速度センサー診断列を削除した。現行ログは36B dummy基準ログへ戻し、採用済みの `max90` ライン角速度目標capと曲率由来FFだけを残す。
- 2026-08-01: `STOP_ENCODER_STOP` 切り分けは不要とし、緊急停止時の `encCurrentN/L/R` ラッチ getter/setter と緊急停止カウンタ getter/log列を削除した。
- 2026-08-02: 調整完了に伴い、速度FF内訳、速度PID単体出力、速度偏差、飽和前速度PWM要求、速度PWM飽和フラグのログ用変数と常時ログ列を削除した。必要時のみ36B以内の一時診断列として再追加する。
- 2026-08-02: 高速度帯速度比例FFスケール構造は不要とし、`SPEED_FEEDFORWARD_VEL_SCALE` / `SPEED_FEEDFORWARD_HIGH_*`、`calcSpeedFeedForwardVelScale()`、速度FFスケール用ログヘッダを削除した。速度FFは従来どおり目標速度とCrr相当バイアスから合計PWMを算出する。
- 2026-08-12: 電圧指令化後の速度追従改善比較として、正規化モーター指令の公称電圧を `MOTOR_COMMAND_NOMINAL_V=7.0V` から `7.2V` へ変更した。速度PID、速度FF、速度表は変更しない。
- 2026-08-16: 前走のスリップを次走の速度計画へ反映する感度を全体的に弱める比較として、`CA_SLIP_FRAC_FULL=0.60f` を `0.70f` へ変更した。局所index、速度PID、速度FF、ライン制御、SD設定は変更しない。
- 2026-08-16: `CA_SLIP_FRAC_FULL=0.70f` の追加完走ログで目標速度の過剰低下が再現しなかったため、次の非局所速度帯比較としてSDの `bst200` を `1.70m/s` から `1.65m/s` へ変更した。`bst100=1.60m/s`、速度PID=12、速度FF=150、ライン制御は固定する。
- 2026-08-16: `bst200=1.65m/s` はautoStart周回別のlap短縮と横スリップ改善が一貫しなかったため不採用とし、SDの `bst200` を基準値 `1.70m/s` へ復帰した。復帰はSD値確認のみで完了とする。
- 2026-08-16: `bst200` 比較後の次候補として、速度表の減速開始先行距離 `decelLeadMm` を `60mm` から `50mm` へ変更した。速度値、速度PID、速度FF、ライン制御は固定する。
- 2026-08-16: `decelLeadMm=50mm` はautoStart周回別に直線指標、速度誤差、lapの悪化が再現したため不採用とし、基準値 `60mm` へSD設定だけ復帰した。
- 2026-08-16: 最新ログで速度目標未達側の速度誤差が残ったため、次の速度追従比較としてSDの速度PID `kp` を `12` から `14` へ変更した。`ki=0`、`kd=0`、速度FF=150、速度表、ライン制御は固定する。
- 2026-08-16: `veloCtrl.kp=14` はautoStart周回別にlap、lineTraceCtrl RMS、横スリップが悪化し、速度誤差p95の改善も一貫しなかったため不採用とし、`kp=12` へSD設定だけ復帰した。
- 2026-08-16: `speed_ff=165` は `CA_SLIP_FRAC_FULL=0.70f` 条件で `11821`〜`11835` を確認したが、autoStart周回別の速度誤差p95、lineTraceCtrl、PWM飽和、横スリップの改善が一貫しなかったため不採用とし、`speed_ff=150` へSD設定だけ復帰した。
- 2026-08-16: `speed_ff=165` 不採用後の次候補として、正規化モーター指令の公称電圧 `MOTOR_COMMAND_NOMINAL_V` を `7.2V` から `7.1V` へ変更した。速度FF=150、速度PID=12、速度表、ライン制御は固定する。
- 2026-08-16: `MOTOR_COMMAND_NOMINAL_V=7.1V` は6セットで候補継続とし、autoStart 2・4のライン負荷と横スリップをさらに確認するため、次候補として `7.0V` へ変更した。速度FF=150、速度PID=12、速度表、ライン制御は固定する。
- 2026-08-16: `MOTOR_COMMAND_NOMINAL_V=7.0V` は開始電圧を揃えた4セットでautoStart 5のlapと速度誤差p95が悪化し、autoStart 4のgyroと横スリップも悪化したため不採用とし、既知候補の `7.1V` へコード値を復帰した。
- 2026-08-16: `MOTOR_COMMAND_NOMINAL_V=7.1V` 復帰後、速度FF=165の不採用と速度PID=12を維持したまま、速度追従の中間比較としてSDの `speed_ff` を `150` から `160` へ変更した。速度表、ライン制御、公称モーター指令電圧は固定する。
- 2026-08-16: `speed_ff=160` は `11966`〜`11970` の失敗セットを除外した追加4完走セットでも周回別の速度誤差p95とlap改善が一貫しなかったため不採用とし、SDの `speed_ff` を基準値 `150` へ復帰した。速度FFの連続比較は打ち切り、次は速度プロファイルまたはスリップ反映側を検討する。
- 2026-08-16: 速度FF調整を打ち切った次の非局所速度計画比較として、スリップリスクの隣接indexへの伝播を弱めるため `CA_SLIP_EXPAND_1` を `0.50f` から `0.40f` へ変更した。`CA_SLIP_FRAC_FULL=0.70f`、速度PID=12、速度FF=150、速度表、ライン制御は固定する。
- 2026-08-16: `CA_SLIP_EXPAND_1=0.40f` は `11991`〜`12005` の完走3セットで周回別の速度誤差p95、lap、横スリップ改善が一貫しなかったため不採用とし、`0.50f` へコード値を復帰した。復帰はコード確認のみで完了とする。
- 2026-08-16: 次の非局所速度計画比較として、スリップリスクの遠方indexへの伝播を弱めるため `CA_SLIP_EXPAND_2` を `0.25f` から `0.20f` へ変更した。`CA_SLIP_EXPAND_1=0.50f`、`CA_SLIP_FRAC_FULL=0.70f`、速度PID=12、速度FF=150、速度表、ライン制御は固定する。
- 2026-08-16: `CA_SLIP_EXPAND_2=0.20f` は `12006`〜`12040` のautoStart周回別比較で速度誤差p95の安定改善がなく、lap改善も再現しなかったため不採用とし、基準値 `0.25f` へコード値だけ復帰した。復帰後の実走行確認は不要とする。
- 2026-08-16: `CA_SLIP_EXPAND_2=0.20f` 復帰後の次候補として、伝播ではなく横スリップ発生区間そのものの減速を弱めるため `CA_SLIP_DOWN_LAT_EXTRA` を `0.30f` から `0.25f` へ変更した。速度PID=12、速度FF=150、速度表、ライン制御、`CA_SLIP_EXPAND_1/2` は固定する。
- 2026-08-16: `CA_SLIP_DOWN_LAT_EXTRA=0.25f` は `12041`〜`12060` のautoStart周回別比較で速度誤差p95と横スリップの安定改善がなく不採用とし、基準値 `0.30f` へコード値だけ復帰した。復帰後の実走行確認は不要とする。
- 2026-08-16: `CA_SLIP_DOWN_LAT_EXTRA=0.25f` 復帰後の次候補として、横スリップriskの基本減速だけを弱めるため `CA_SLIP_DOWN_LAT_RISK` を `0.20f` から `0.15f` へ変更した。`CA_SLIP_DOWN_LAT_EXTRA=0.30f`、`CA_SLIP_FRAC_FULL=0.70f`、`CA_SLIP_EXPAND_1/2`、速度PID=12、速度FF=150、速度表、ライン制御は固定する。
- 2026-08-16: `CA_SLIP_DOWN_LAT_RISK=0.15f` は `12061`〜`12075` のautoStart周回別比較でlapと速度誤差p95が全走行番号で改善しなかったため不採用とし、基準値 `0.20f` へコード値だけ復帰した。復帰後の実走行確認は不要とする。
- 2026-08-16: 横スリップ側の `CA_SLIP_EXPAND_1/2`、`CA_SLIP_DOWN_LAT_EXTRA`、`CA_SLIP_DOWN_LAT_RISK` で安定改善が出なかったため、次の非局所速度計画比較として縦スリップriskの基本減速を `CA_SLIP_DOWN_LONG_RISK=0.04f` から `0.03f` へ変更した。縦スリップ追加減速、横スリップ側、速度PID=12、速度FF=150、速度表、ライン制御は固定する。
- 2026-08-16: `CA_SLIP_DOWN_LONG_RISK=0.03f` は `12076`〜`12097` のautoStart周回別比較で速度誤差p95が全走行番号で悪化し、lap改善も再現しなかったため不採用とし、基準値 `0.04f` へコード値だけ復帰した。`12093`〜`12097` は `12094` の途中終了によりセット全体を除外した。復帰後の実走行確認は不要とする。

## 15. 機体・回路変更時にコードへ反映する項目

機体・回路変更時は、STEP、KiCad、PDF、`.ioc`、関連ソースを確認し、確認できた事実と推測を分けて扱います。

- コードへ反映した場合は、関連する定義、ログスキーマ、パラメータ保存形式、`AGENTS.md` の記述に矛盾がないか確認する。
- 詳細な確認項目と報告ルールは `.agents/skills/robotrace-hardware-review/SKILL.md` を使う。
