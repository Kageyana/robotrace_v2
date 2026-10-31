---
type: codex-failure
date: 2026-10-13
task: "ラインセンサADC位相管理のC11ホスト検証"
status: resolved
severity: low
tags: [codex/failure, test, adc, float]
---

# ADCホストテストの初期状態と比較精度を合わせた

## 確認結果

- `-Wextra -Werror`で既存マーカー差分のint/unsigned三項演算が警告となった。非負差分を明示的にuint32_tへ変換し、判定結果は維持した。
- VREFINT未取得で基準電圧を保持するテストが、`adcVref == MCU_VOLTAGE`で失敗した。MinGWの拡張精度とfloat保存値の既知問題を期待値へ適用できていなかった。
- 変更前後の平均化回帰比較で、初回公開前の配列一致が失敗した。製品側は電源切替時に最後の公開値を保持するが、固定参照はゼロ初期化であり、テストの開始状態が一致していなかった。
- 最終呼出元レビューで、UIが毎ループONを要求することを確認した。初期実装はON要求のたびに平均化・CC1を破棄しており、呼出側の冪等性契約を保持できていなかった。実機での更新停止は観測していない。

## 原因と対処

マーカー差分はGPIOの0/1をintへ整数昇格した後、三項演算の0Uと混在していた。明示的な符号なし変換で既存意味を保ちつつ警告を解消した。

基準電圧の期待値は`volatile float nominal`に実体化した。許容誤差の拡大や製品電圧式の変更は行っていない。回帰比較は両方の公開配列をゼロから開始させた。製品の電源切替仕様は変更せず、残留平均値は別のOFF/ONテストで検証した。

電源ONの重複要求は、CC1・取得世代・占有・平均化を保持して戻るよう修正した。各取得要求前とDMA完了前にONを再要求する回帰ケースを追加した。今後は公開APIを変更する前に全呼出元を検索し、UIからの連続呼出しと実際の状態遷移を区別してテストする。

## 再発防止・確認方法

ホストテスト作成前に、公開値・内部積算値・static初期化・電源切替の保持契約を分けて確認する。floatの厳密比較では期待値も同じ保存型で実体化する。取得中・処理中・公開時を区別して割込みを挿入し、変更前参照と製品側の開始状態も揃える。

`python validation/line_sensor_adc/run.py`をGCC C11のO0/O2、Wall/Wextra/Werrorで成功確認。要求10ケースと4000取得/1000フレーム回帰比較が通過したため、ホスト検証側の問題はresolved。実機波形・走行性能は別途未確認。

関連: `2026-10-12-host-float-snapshot-comparison.md`、`doc/line-sensor-adc-phase.md`。

## 2026-10-14 コミット分離コピーの確認

分離コピー作成後、補助スクリプトの表示用read_text()だけがWindows既定CP932でUTF-8パッチを読み、UnicodeDecodeErrorとなった。コピー自体は完成しており、UTF-8対応の読み取りで内容を確認して検証を継続した。今後の補助スクリプトもrun.pyと同様、リポジトリの日本語ソース・パッチにはencoding='utf-8'または'utf-8-sig'を明示する。分離コピーのホストO0/O2とDebug/Releaseビルドは成功。生成物や他の未コミット変更を混ぜない検証を完了した。

## 2026-10-28 取得優先度変更時のマクロと公開宣言の不一致

対象は取得IRQ優先と制御フレーム固定の実装。通常3構成はビルド成功したが、新しい計測負荷の式にgetCycleCounter()を入れたところ、実DWT.hのマクロ末尾がセミコロンで式として使用できず、計測ONだけコンパイル停止した。ホスト側は関数に置換しており契約差を隠していた。製品の新しい呼出しを代入文に修正し、ホストも実ヘッダのマクロを取得して読出し先だけモックへ置換した。DWT.h自体は変更していない。

共有markerSensorをvolatileへ変更した際、log_storageの旧非volatileモック定義が実ヘッダと衝突した。モック定義を合わせて通常/詳細の保存・全CRC・変換試験が通った。また補助の自動校正テストが旧4引数のmotorCommandOutSynthとPATH_CURVE未定義のままで停止した。現行3引数のAPIを事前assertし、モード定義をcourseAnalysis.hから抽出するよう修正した。製品APIやモードを古いテストへ戻していない。

再発防止は、抽出/モック化前に実ヘッダのマクロ形・型修飾子・引数・現行モードを確認し、定義を再利用すること。マクロを関数と仮定せず、計測OFF/ON両方をビルドする。実行点はvalidation/isr_diagnostics/run.py、validation/log_storage/check.c、robotrace_v2/tests/run_autostart_calibration_tests.py。flash-debug Skillにも追加した。

O0/O2・Wall/Wextra/WerrorでDWT周回/負荷ガード、ADC4スロット周期欠測/公開一貫性、校正/既存PID、停止後2秒保持が成功。全4構成ビルド・新規warning0、保存2プロファイル・PC変換も成功。証跡はbuild/adc-priority-validation。status: resolved（実装のマクロ使用と試験契約）。実走の採否は別途未確認。

## 2026-10-29：補正無効時も予測するフィクスチャで共分散が非対称になった

方位補正APIの単独試験で、alpha=0のケースにも800msの予測を与えたところ、float丸めの累積で共分散の対称性検査が失敗した。実PATH_CURVEはalpha=0なら融合予測を呼ばず、補正無効または追加勾配だけを適用する。製品の呼出契約と試験の前提が異なっていた。

試験側の予測条件を実呼出元と合わせ、alpha=0・追加勾配0なら出力0/P不変、追加勾配ありなら方位だけ/Pとbias不変を別々に確認した。製品の閾値や許容誤差を広げていない。有効予測のケースでは実出力gainによる独立倍精度Joseph形との一致と、各1000時点の正規化共分散固有値を確認した。

再発防止は、単独APIの条件を決める前に呼出元のenable/予測/補正/欠測時処理を読むこと。無効化条件下の保持契約と、有効な連続予測の数値安定性を別試験にする。validation/path_heading_fusion/run.pyの25組×O0/O2が成功し、最小固有値は約0.0334。status: resolved（フィクスチャ契約）。


## 2026-10-30：減速度0の試験がPATHの識別子を推測

実制御呼出元の新しい試験でBOOST_PATHを使い、コンパイルが停止した。実ヘッダはBOOST_PATH_REPLAYで、before-mode-name-fixへ試験ソース/生成器/ビルドログを保全。実ヘッダへ合わせ、減速度0でも成立5ms後だけ代替制動すること、ガード無効とPATHの速度が変わらないことをO0/O2で確認した。製品は未変更。

次回は生成器が対象private/Core/Inc/courseAnalysis.hを先に読み、BOOST_PATH_REPLAY/BOOST_PATH_CURVEの実宣言を検査してからCを生成する。既存Skillに現行モード確認はあったが、新しいフィクスチャの実行入口に機械検査がなかった再発である。確認はcheck_tracking_guard_zero_decel.pyとzero_deceleration_validation.json。status: resolved（試験の識別子・生成前ゲート）。

## 2026-10-30：識別子を確認しても宣言形式をenumと仮定した再発

- 対象: PATH入力RAM計測のホスト生成。
- 事実: BOOST_PATH_CURVEは実在するが、生成器はenumの代入式を検索し、実際の#defineに一致せずNone参照でコンパイル前に停止した。製品/実機は未変更。
- 原因: 前項の識別子確認を満たすだけで、宣言形式の確認と一致数ゲートが不足していた。
- 対処: 実ヘッダの#define行を読み、行頭と整数値のマクロ形式を照合。宣言・値5・controller21/estimator3をcaller_preflight.jsonへ保存し、不一致なら生成前に停止する。旧生成器をbefore-mode-declaration-fixへ保全。
- 再発防止: 型/マクロ/enumの形式と一致数を実宣言から確定してから抽出・モック化する。識別子名だけの検査を完了条件にしない。flash-debug Skillの生成前へ反映。
- 確認: path_input_capture_20261030/host_validation.json、実timerフック4608条件×O0/O2、6構成の新規ARMビルド。モード5だけが計測し、制御入力は不変。未実行段階から完了した。
- ステータス: resolved（宣言形式と試験生成）。open（実機の入力有効性・処理時間・動的スタック）。

## 2026-10-30：モーター試験のタイマーハンドルを定数で代用

停止専用像の実motorDutyOut等を抽出したPC試験で、MOTOR_TIM_HANDLERを0と定義した。実呼出しは&MOTOR_TIM_HANDLERを渡すため、O0演算3072件・独立性6件・停止ガード1728件が成功した後、モーターモックのコンパイルがlvalue requiredで停止した。機体・製品実装は未変更。既存のマクロ形確認で値だけを見て、呼出し側のアドレス取得要件を検査しなかった再発。

模擬ハンドルをアドレス取得可能な変数へ修正。旧入口・旧ハーネス・完成O0演算exe/出力/ソースSHAを保存し、完成O0演算は再実行せずモーター試験から再開した。O0/O2で実演算6144件と停止ガード3456件、全int16の左右/吸引指令262144件、合成1458件、非有限を含む電圧12件を確認。計測OFFでは通常出力が有効、ARMのOFF構成ではmotor/main/timer命令列が旧試験版と一致した。

再発防止はモック生成前に実マクロの使用式（&handle、handle.Init、handle.Instance等）を調べ、必要なlvalueと構造体/ポインタの参照形を満たすことを確認すること。コマンドはrepo root workdir・絶対Pythonでanalysis/script/validate_path_stopped_bench_host.py、既知のGNU DLL制限に必要な権限を初回から選ぶ。status: resolved（PC模擬ハンドルと再開）。新しい停止演算像の実機時間・実走効果はopen。
