# robotrace_v2

## コンパイルオプションについて

`printf` や `scanf` で `float` 型を扱えるようにするため、リンカオプションに以下のフラグを追加しています。

- `-u _printf_float`
- `-u _scanf_float`

STM32 向けの newlib-nano では標準状態で浮動小数点入出力が無効なため、上記のフラグを `CMakeLists.txt` の `CMAKE_EXE_LINKER_FLAGS` に追加することでサポートを有効化しています。

## カスケード制御について

- `PIDcontrol.c` の走行制御は `USE_CASCADE_TRACE`（既定値: 有効）により、外側ループで角速度フィードバック、内側ループで左右速度フィードバックを行うカスケード構成へ更新しました。
- 外側ループはラインセンサ偏差から角速度指令を生成し、IMUが利用可能な場合は実測角速度との偏差をPID制御します。IMUが未初期化の場合は角速度指令のみで制御します。
- 内側ループは左右ホイールの速度PIDで Duty（-1.0〜1.0）を算出し、`motorPwmOutSynth()` を経由して PWM を出力します。
- 調整用パラメータは `Core/Inc/control_params.h` に集約しており、`cascadeControlParams` 経由で変更できます。

### 調整手順の目安

1. `cascadeControlParams.speed_l` / `speed_r` のゲインで左右速度ループを調整し、所望の応答を得てから次へ進みます。
2. `cascadeControlParams.yaw` および `cascadeControlParams.line` を順に調整し、角速度制御とライン偏差→角速度指令のバランスを取ります。
3. `duty_limit` や `v_limit` は安全のための飽和値なので、チューニング時には十分な余裕を持たせてください。

### 旧制御への切り替え

- `Core/Inc/PIDcontrol.h` の `USE_CASCADE_TRACE` を 0 に変更すると従来の PID 構成に戻ります。

