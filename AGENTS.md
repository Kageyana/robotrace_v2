# AGENTS.md

## 作業の起点

- 日本語で応答し、ファームウェア変更は変更理由・確認結果・実機で未確認の点を報告する。性能改善は再現性・完走率を優先してからラップタイムを評価する。
- `robotrace_v2/` は STM32F446RETx 向け C11 ファームウェア、`analysis/script/` は PC 側ログ解析・回帰テスト、`Circuit/` は KiCad 基板、`Machine/` は STEP 機体モデル。以下の `Core/` と `CMakeLists.txt` は `robotrace_v2/` 基準。`build/`、`robotrace_v2/build/`、`robotrace_v2/Debug/` は生成物。
- 実行の起点は `Core/Src/main.c` → `loopSystem()` (`control.c`) と HAL コールバック (`main.c`)。走行制御は `control.c` / `PIDcontrol.c`、経路生成・追従は `courseAnalysis.c` / `pathFollower.c`、SD 設定とログは `SDcard.c` / `sd_functions.c` を確認する。
- 関連する既往トラブルを作業前に `doc/失敗事例/` から探す。再利用できる新しい失敗は同ディレクトリの `_template.md` に沿って原因・次回の具体的な確認手順を記録する。実装履歴は `docs/実装済み履歴.md` に置く。

## ビルド・検証

- `robotrace_v2/` で `cmake --preset Debug` → `cmake --build --preset Debug`。Release は `Debug` を `Release` に置換する。通常ログは軽量プロファイル、詳細マーカーログは `DebugMarker` preset (`ROBOTRACE_LOG_SCHEMA_PROFILE_LIGHT=0`)。スリップ1 ms診断は既定 OFF で `-DROBOTRACE_ENABLE_SLIP_UPDATE=ON` のときのみ有効。
- preset は Ninja と `arm-none-eabi-*` の PATH を要求する。VS Code 拡張環境は `robotrace_v2/.vscode/settings.json` の `cube-cmake` / Cube bundle。CLI の CMake/Ninja はサンドボックス外で構成から実行する（サンドボックス内で Ninja が無出力停止した事例あり）。失敗時は `doc/失敗事例/2026-09-06-codex-shell-stm32-vscode-build-hang.md` を参照し、ビルド先削除や preset 変更で回避しない。
- `CMakeLists.txt` の `-u _printf_float -u _scanf_float` は STM32 の float 入出力に必要。CubeMX 再生成ファイルは `USER CODE` 領域を守り、HAL/ミドルウェアを無用に編集しない。
- PC 側の単体テスト例（リポジトリ直下）: `python -m unittest analysis.script.test_path_goal_extension`。対象の `analysis/script/test_*.py` を選んで実行する。制御変更は ARM ビルドと割り込み競合を確認し、ビルド成功を実機動作確認と混同しない。

## 実機の制約と実行経路

- 基本単位は距離 mm、速度 m/s、角速度 deg/s、時間 ms。TIM6 の `Interrupt1ms()` が IMU・エンコーダ・距離融合・制御・緊急停止を担当する。TIM7 の 0.5 ms 割り込みは 10 分周で 5 ms ごとにログ要求を立て、実 SD 書き込みは割り込み外の `loopSystem()` → `logWriteTask()` で行う。割り込み内に待機、SD 書き込み、表示更新を加えない。
- 角度・ジャイロ Z は時計回りが正、左右モーターとエンコーダは前進が正。BMI088 生 Y は前進時に負なので `BMI088getAccele()` で反転する。距離換算は `Core/Inc/encoder.h` の `PULSE_METER`（現行 58019 pulse/m）を基準にし、生エンコーダと融合後距離を混同しない。
- 指令/DUTY は ±1000、速度上限 10 m/s、加速度上限 20 m/s²。正規化指令 1000 は公称 7.1 V 相当で、実 DUTY はバッテリー電圧補償後。高速走行前はセンサー初期化と電圧 7.5 V 以上を確認する（低電圧での自動走行禁止ではない）。IMU・ライン異常時は走行禁止、SD・電流センサー異常は警告扱い。緊急停止を通常の走行検証で無効にしない。
- 書き込みは実機と ST-Link が接続されている場合のみ。ST-Link デバッグ中は実走行しない。詳細は `.agents/skills/robotrace-flash-debug/SKILL.md`。
- 配線・ピン・寸法は変数名から推測せず、`robotrace_v2/robotrace_v2.ioc`、対象の `Circuit/` 内の回路図/基板、`Machine/` の STEP と関連ソースを照合する。手順は `.agents/skills/robotrace-hardware-review/SKILL.md`。

## 設定、走行、ログ

- 実機設定は SD の `./setting/*.txt` がコード既定値に優先する。欠落ファイルは既定値で作成し、破損時は有効項目だけ部分反映して修復する。`lsval.txt` 破損時は走行禁止。保存形式・旧形式互換を変える際は読み書きを同時に確認する。詳細は `.agents/skills/robotrace-sd-settings/SKILL.md`。
- 通常走行の `SGmarker` はスタートで 1 になり、`COUNT_GOAL=COUNT_LAPS+1`（現行 6 周で 7）。PATH REPLAY / SHORTCUT は右マーカー数ではなく、一次経路終端から原点への中間延長点で停止を開始する。正常・緊急停止のログ保存は機体停止後。走行モード定義は `Core/Inc/courseAnalysis.h`、PATH 経路は `pathFollower.c` を正とする。
- ログ列・版・レコード長の正は `Core/Inc/log_schema.h`（現行 schema 10、軽量 36 B）。CSV は UTF-8、1 行目 `key=value` メタデータ、2 行目列名、以降データ。旧形式も列名で解決し、列位置を決め打ちしない。schema 10 の `x/y` は経路用、`x_fused_mm/y_fused_mm` は診断用。新しい PATH 経路元は正常終了・`closureValid=1` 等の検証を満たす一次ログだけ。
- ログ解析・比較は `.agents/skills/robotrace-log-analysis/SKILL.md`。`emcStop != 0` や欠落 `cntlog` を有効走行とせず、同じ走行モード・近い電圧条件で比較する。制御・チューニング変更の採否は実機ログ最低 10 本で確認し、1 走で変えるパラメータは最大 2 個。調整手順は `.agents/skills/robotrace-tuning/SKILL.md`。
- ログ列・単位・走行モード・ハード構成・安全制約を変えたら、このファイルの該当する運用指示とコード・設定形式の整合を確認する。ログ形式変更時は最初の実機ログを確認する。
