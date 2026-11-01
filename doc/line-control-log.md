# 走行方式別のライン制御ログ（2026-11-01）

DISTANCE/SLIPの現行切替は`lineGainPolicyVersion=2`（計画ROCのみ）。キーがない14033/14034等は旧方式。詳細は[計画ROCによる切替](plan-roc-gain.md)を参照。

一次走行・DISTANCE・SLIPへ、実際に使ったラインゲインと判定状態、ラインP/D項、制限後の左右輪目標を追加する。PATH/SHORTCUT/PATH_CURVE専用11列はこれらの走行で保存しない。PATH系は従来の列定義と記録長を保ち、以下のライン診断14列を追加しない。

|方式|通常 profile=1|詳細 profile=0|
|---|---|---|
|一次・DISTANCE・SLIP|64列 / 166 B|70列 / 175 B|
|PATH・SHORTCUT・PATH_CURVE|61列 / 169 B|67列 / 178 B|

通常のDebug/Releaseはprofile=1を使用する。詳細profileはホスト検証だけで確認し、専用ビルド構成は追加しない。SLIPは内部的に`BOOST_DISTANCE=2`と同じ距離計画・左右独立速度制御を使用するため同じ形式になり、`slipSourceLogNumber`で識別する。

記録間隔は既存の5 mm距離基準を維持する。短区間の1 ms診断バッファや追加ファイルは作らない。制御自体は1 ms更新で、各行は保存時点の最新の計算を記録する。記録間に終了したゲイン遷移や制限は網羅しない。

|追加列|型・単位|意味|
|---|---|---|
|`lineKpUsed` / `lineKdUsed`|各U8、整数ゲイン|その行のラインPID計算で実際に使ったP/Dゲイン|
|`lineGainState`|U8|0=使用周期の判定未確立・角速度FB未計算、1=設定ゲイン、2=直線用P2/D0、3=カーブ用設定ゲイン、4=一次クロス用P2/D0|
|`lineGainReason`|U8、ビット集合|下表の判定条件|
|`lineCurveGainHold`|U8、0/1|使用した判定のカーブ保持状態|
|`lineGainStablePulse`|S32、pulse|使用した判定時点の`encChangeGain`。換算は58019 pulse/m|
|`lineGainRocNow_mm`|F32、mm|判定時点の実測`rocrun`。版2では観測用で、切替判定には使用しない|
|`lineGainPlanEnterRoc_mm` / `lineGainPlanExitRoc_mm`|各S16、mm|進入4点先・復帰2点先の計画ROC。終端は最終要素へ制限。距離計画判定なしは0|
|`linePTerm_x32` / `lineDTerm_x32`|各S32、(pulse/ms)×32|実計算の`Kp×Dev` / `Kd×(Dev−前周期Dev)`。右5 bitシフトと±1000制限の前。I項は含めない|
|`wheelTargetL_p` / `wheelTargetR_p`|各S16、pulse/ms|左右輪速度PIDへ渡した上下限制限後の目標。前進が正|
|`wheelTargetLimitFlags`|U8、ビット集合|bit0=左上限、bit1=左下限、bit2=右上限、bit3=右下限。制限前値が上限/下限を超えたときだけ1|

左右輪上限は既存の`(int16_t)PULSE_MILLIMETER * 5`（±290 pulse/ms、約±5 m/s）を維持する。通常ライン追従で平均速度PIDを使用する開始前・停止動作中は、追加診断を0にする。未校正で角速度FB計算を実行しない場合もP/D診断は0。`lineGainState=0`を設定ゲインP0/D0と解釈しない。

|`lineGainReason`のbit|意味|
|---|---|
|0|版2では予約・常に0。旧方式は実測側ROCが進入閾値500 mm以下|
|1|4点先の計画ROCが進入閾値500 mm以下|
|2|カーブ用ゲインを保持中|
|3|保持中かつ復帰ROC条件（600 mm以上）が成立し、復帰距離を確認中|
|4|この判定で保持を解除|
|5|一次走行のクロスゲインを選択|
|6|DISTANCE/SLIPの距離計画による判定なし（一次走行を含む）|

`changeGain()`はラインPID計算後に実行され、次周期へ適用される。判定結果を別に保持し、次のPID計算の入口でゲイン・理由・ROC・距離と一緒にコピーする。このため後から切り替わったゲインを、その行のP/D項に対応するゲインとして誤記録しない。`rocrun`の更新も距離ログのタイミングで行われるので、この列は同じ行の最新gyroから計算したROCと一致するとは限らない。ログは判定時の値を保持する。版2は進入4点先・復帰2点先の計画ROCだけで切り替え、実測ROCは観測用に残す。

列定義と`optimalTrace`は`initLog()`で走行開始時に固定する。レコード長・列数・スキーマCRCをファイルごとにヘッダへ保存し、停止時全件検証・機体の型付き読込はその形式で処理する。RLOG v2、big-endianレコード、CRC32/IEEE、派生XY/ROC、6 KiB共有I/O、8 KiBスタック予約を維持する。一次ログはPATH元にも使うため、経路生成に必要な校正・閉路等のメタデータは残す。

新しいライン形式と旧PATH形式は現行ビルド内で共存するが、旧PATH列を含む一次/DISTANCE/SLIPは機体の解析元として拒否する。更新後は新しい一次→DISTANCE→SLIPを取得する。過去ログは変更しない。PCは自己記述列定義で復号するが、現環境に配備済み変換器がないため、その実行と最初の実機bin/CSVは未確認。

診断追加時の検証は`validation/mode_log_diagnostics/run.py`と`finalize.py`。当時の旧新制御不変・16000周期比較は切替方式版1の履歴である。版2の検証入口は`validation/plan_roc_gain/run.py`と`finalize.py`で、計画判定の独立性・境界と、両ログ形式の実writer/reader・全CRCを確認する。通常ログDebug/Releaseを全体ビルドし、既存6 warningに対する新規warning、共有I/Oとスタック配置を確認する。実機の割り込み時間と走行安定性、初回ログの版・値・CRC確認は別途必要。
