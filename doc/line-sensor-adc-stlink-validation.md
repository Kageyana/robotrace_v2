# ラインセンサADC位相管理のST-Link静止検証

2026-10-13、`codex/line-sensor-adc-phase`。ユーザーのST-Link接続後の追加依頼に基づき実施した。実走行、空転指令、校正旋回、SD設定の変更は行っていない。今回の検証でファームウェアソースは変更していない。

## 対象・照合

- STM32F446、Device ID `0x421`、ST-Link V2、SWD 100kHz。
- 最新の通常Debugをconfigure/buildし、`robotrace_v2/build/Debug/robotrace_v2.elf`を書込み。
- ELF SHA256: `056fbc499161b040ba15b365e88a338b419ad1869a6224acaa52bd3194574e80`。
- GDBでFlashを読み出し、ELFをobjcopyで変換した269568 Bと全バイト一致。GDBの`compare-sections`は過去の誤不一致事例があるため使っていない。
- `initIMU=true`、`initMSD=false`、`patternTrace=0`、走行要求なし、ログ記録なし。モーターPWM変数とTIM2左右比較値は全確認点で0。
- SDの校正値は取得されておらず、最大値0・最小値65535の未校正状態。今回の`lSensorCari[]`は既存の未校正時フォールバックによりraw差分と一致する。校正済み正規化やPIDの実機追従性能の検証とは区別する。

## 連続更新の計測

最初のGDB通常接続区間では、45986制御msで45982回更新（平均1.000087 ms）、スキップ増分5、最大観測間隔3msだった。コア停止中もTIM3が進み、再開時の境界・取得窓失効を診断してしまうため、通常動作のみのスキップや最大間隔としては採用しない。

その後、DBGMCU_APB1_FZのTIM3/TIM6/TIM7停止ビット（`0x32`）を一時設定した。これはデバッガがコアを停止した間だけタイマーを止め、通常実行中のタイマー設定を変えない。意図的な競合注入による長い停止間隔を除くため、区間開始時に最大間隔だけを0へリセットしてからGDBを切り離した。

| 指標 | 開始 | 終了 | 増分・結果 |
|---|---:|---:|---:|
| TIM6 `cntRun` [ms] | 541618 | 593083 | 51465 ms |
| `lineSensorUpdateCount` | 363655 | 415120 | 51465回 |
| `adcSkippedCount` | 86626 | 86626 | 0回 |
| `adcStartErrorCount` | 0 | 0 | 0回 |
| `adcTransferErrorCount` | 0 | 0 | 0回 |
| `lineSensorStaleCount` | 2 | 2 | 追加検出0回 |
| `lineSensorMaxUpdateAgeMs` | 区間開始後0に設定 | 1 | 最大1ms |
| `adcAcquisitionFault` | false | false | 異常ラッチなし |

平均周期は`51465 / 51465 = 1.000 ms`。1ms制御の更新番号監視による計測で、ADC開始・終了時刻のサブms精度や波形を直接測定した値ではない。スキップ・停止検出の累積値は、以下の故障注入とそれ以前のGDB停止操作を含むため、絶対値を通常時の件数として扱わない。

終点でのraw差分/未校正フォールバック値は`{600,857,886,809,663,630,731,1056,703,700}`で、10chすべての更新を確認した。VREFINTは1493～1501程度、`adcVref`は約3.31 Vで有限だった。マーカー出力は0で、白マーカーへ動かした検証は行っていない。

終点の凍結スナップショットでは、TIM3 CNT=487、CCR3=1874、現在LED位相=true、PC14出力ビット=1に対し、最後に取得した位相=false、バッファ=`analogValLSoff`だった。現在位相と取得済み位相が独立して保持されていることを確認できた。物理LED波形のオシロスコープ測定を代替するものではない。

## 取得競合・更新停止と復帰

停止したメインループ位置で、ADC1のRAM上の`hadc1.Lock`だけを`HAL_LOCKED`へ設定してから実行を再開した。モーター指令は変更していない。

- 開始: tick=317901、update=208027、skip=13。
- 競合中: tick=339551、update=208027、skip=86603、age=21649ms、stale=true、staleCount=1、busy=false、fault=false。
- TIM6は21650ms進み、取得要求だけがスキップされた。ADC開始/転送エラーはいずれも0。PWM/左右比較値0、走行開始なし。
- `HAL_UNLOCKED`へ戻すと、MCUリセットや再初期化なしで更新が再開した。回収時はupdate=256616、age=2ms、stale=false、fault=false。

約21.6秒の停止と大量スキップは意図した競合注入による値であり、自然発生したADC故障ではない。新しい強制停止を有効化せず、診断だけで検出する初期方針も維持された。

## 電源切替

GDBから呼んだのは、短い非ブロッキングの`powerLineSensors()`と終了時のゼロ出力関数のみ。HAL待機・SPI・SD処理のinferior callは使用していない。

- OFF設定時: tick=393781、update=262250、skip=86618、power=false、age=0、stale=false、VREFINT公開値0、busy=false。
- OFF確認時: tick=440203、update=262250、skip=86618。約46.4制御秒間、更新も取得スキップも増加しなかった。
- TIM3 DIER=`0x9`（CC1割込み無効）、CCR3=0。基準電圧は直前の有限値約3.309 Vを保持した。
- ONと重複ON要求の後にupdate=262251となり、その後の連続計測でも正常更新した。GDB関数呼出し中に割込みが動くため、呼出し前後の世代値は重複ONの冪等性の証明には使わない。冪等性・旧平均値除外の厳密比較はPCテストで確認済み。

## 終了状態

- `powerLineSensors(0)`、`motorCommandOut(0,0)`を実行し、ライン電源OFF・TIM3 CCR3=0・CC1無効・左右PWM0・TIM2左右比較値0を確認。
- 注入したロックは`HAL_UNLOCKED`、`adcAcquisitionFault=false`、`adcAcquisitionBusy=false`。
- DBGMCU_APB1_FZは元の`0x0`へ復元。
- `patternTrace=0`、開始要求0、autoStart=0、ログ記録false、emcStop=0、CFSR/HFSRとも0。
- GDBをdetachし、今回起動したGDBサーバーだけを終了してST-Linkを解放。機体は通常待機状態で動作を継続する。
- 故障注入で増えたスキップ・検出回数はRAMに残しており、次回起動で初期化される。

## 未検証

SD挿入・有効校正・バッテリー/モーター負荷・実走行中の最大割込み負荷、物理LED/ADC波形、白マーカーの左右検出、走行追従・完走率は未検証。今回の最大1msを走行中の最悪値として採用しない。3msの診断閾値は維持し、強制停止を新しく有効化していない。

ローカルの生ログ・GDBコマンド・ELF/Flash照合用バイナリ・集計JSONは`analysis/line-sensor-adc-stlink/`へ保存した（既存方針でGit管理対象外）。CLI再接続エラーとGDB停止による計測影響は`doc/失敗事例/2026-10-13-stlink-adc-debug-observation.md`に記録した。
