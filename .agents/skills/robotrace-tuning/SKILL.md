---
name: robotrace-tuning
description: ロボトレースのPID、速度フィードフォワード、スリップ判定、速度・加速度パラメータを調整するときに使う。走行ログ比較、再現性判断、速度向上、変更後10本ログ確認を扱う。
---

# Robotrace Tuning

## 2026-10-25：現行PATH/SHORTCUT更新周期

`routeControllerVersion=18`から自己位置積分→最近傍探索・白線補正・目標更新→PIDを1msごとに実行する。ロスト/白線消失100ms、復帰200ms、混合率切替100msは維持する。SDの位置/方位補正係数は5ms基準の値を保持し、1msでは1/5を適用する。上限は0.2mm/0.02deg毎回。SD値を自動で書き換えない。経路更新の実行頻度増による最長TIM6時間とADC更新欠落は実機で確認する。詳細は[1ms経路更新](../../../doc/path-follower-1ms.md)。

現行ログは通常56列/154 B・詳細62列/163 B。有効性/無効理由/区間統計は保存せず、SLIP計画版3はフラグのON記録数/全記録数を使用する。経路診断6列の最新値は1ms更新、記録は距離間隔。以下の旧サイズ・区間時間率は履歴とし、現行はAGENTS.mdとlog_schema.hを優先する。

## 実行前の再発防止


走行評価集合の「ログ正常終了」と「ユーザー/映像で板内完走確認済み」を分け、未確認を別区分で保持する。emcStop0だけを物理的なはみ出しなしの根拠にしない。現在の23走比較ではemcStop0は21本だが、明示確認済みは13901～13910/13913～13922の20本で13899は未確認。採否・10本の物理確認へ未確認ログを加算しない。[確認範囲の訂正](../../../doc/失敗事例/2026-10-30-normal-log-physical-scope.md)参照。


新しい速度解析の前に、SPEED_PARAM_FIELDSの保存19項目/メタデータ15項目を確認する。search/stop/shortCut/pathReplayはログ非出力。searchが欠けても過去の実SD値を推測せず、凍結C再生の既定を使うなら出典・適用範囲を明記する。保存RAMの停止時値も全過去走へ転用しない。NumPy集計はJSON保存前にint/float/boolへ明示変換し、集計のみ失敗した場合は完成済みC出力を検証して未実行部分から再開する。[非出力項目](../../../doc/失敗事例/2026-10-28-derived-analysis-helper-dependency.md)、[JSON集計型](../../../doc/失敗事例/2026-10-30-tracking-screen-json-scalar.md)参照。


取得版2の13880/13881は正常ログ終了したが、動画13881に板外張出しと5通過Y+16.57mmがある。正常終了数をコースアウトのない実走本数へ加算しない。位置/方位49組×2走の固定入力再生は学習用感度であり、最小変位差を実機採用へ直結しない。独立動画と再生自己位置の変位差、喪失条件、副作用を確認し、実機変更前に今回の設定値・版・対象一次を保持する。詳細は[取得版2の実走](../../../analysis/path_acquisition_220117/report.md)。

調整値を案内する前に、UIの項目名・画面表示値・SD/ログの倍率付き値を対照する。No.8 ShortcutのLine alphaは画面0.005がlineAlpha_x1000=5、画面0.100が100であり、「5へ変更」だけで案内しない。初回の実ログで版・保存設定を確認してから同条件の本数を増やす。13843～13846は全て100で途中停止したため、候補5の実走成功へ数えない。[実走比較](../../../doc/path-confidence-fusion.md)参照。


最初のコマンドの前に [共通事前確認](../../../doc/skill-execution-preflight.md) を読み、パス・workdir・権限・実行環境を確定する。制御やパラメータの試行を設計するときは [分野別の再発防止](references/failure-prevention.md) も読む。関連する失敗記録は最新追記まで確認し、既知の失敗条件を初回操作から回避する。


2026-10-18のユーザー指定：モータ指令電圧上限が同じ走行は電圧差のみで除外せず、速度等は別に記録する。現行横候補版5は、接線補正を除いた同窓残差Rnへ角加速度と同じ一次LPF（前回0.8・今回0.2）を適用し、同窓の適用済み接線項Tbarを引く。ON6.5/OFF4.5m/s²、3/5ms、5/30ms窓と43サンプル充填。ON6.5はユーザー指定で今回テスト・ビルド未実施。最初の有効Rnで初期化し、無効でLPFを消去、区間統計リセットでは保持する。版5の保存候補残差から(R0±Rc)/2でRn/Tbarを逆算しない（この分離は追加LPF前の版1〜4に限る）。版・式・追加LPF係数/初期化方式のメタデータを先に確認する。旧ON5.2での既知16陽性窓保持を現行ON6.5へ流用せず、一次の不要ON・短い滑りの減衰・遅延の実機確認は残る。検出器版3/fast_only・初回SHADOW横4.6/3.8・従来制御は維持。詳細はdoc/slip-lateral-candidate.md。


## 2026-10-21：V2正式採用後の現行仕様

ユーザー指示によりV1互換・SHADOW切替を削除した。縦はFast単独10/8m/s²・3/5ms、横はLatCandidate版5の6.7/4.5m/s²・3/5ms・LPF0.8を使用する（初回ON6.5、2026-10-24の既存変更でON6.7）。保存検出器版4、`slipDetectorMode=ACTIVE_V2`、`slipLatPolicy=matched_candidate_v5`。通常66列/170B、詳細72列/179B（重複整理前の正式採用ログ13571～13577は70列/181B）。旧slipFlag/slipFlagLatと旧詳細診断9列は存在しない。slipV2Flagsの横bitとslipV2Lat/PeakLat/OnLatMsも採用横と同じ意味へ変更した。以下のSHADOW・旧サイズ・旧制御維持の記述は導入時の履歴として扱い、現行は[正式採用とSLIP走行](../../../doc/slip-v2-active.md)を優先する。

横ログは`slipV2Lat/Flags横bit/PeakLat/OnLatMs`へ統一し、`slipV2LatCandidate*`列を廃止した。横独自の有効性は`slipV2LatValid/Reason/ValidMs/IntervalReason`へ改名し、横ON率は`slipV2OnLatMs / slipV2LatValidMs`を使う。横の理由256・再充填で共通有効性と異なるため共通ValidMsへ代用しない。SLIP読込も新列名だけを使い、更新後は新しい一次とDISTANCEを取得する。判定式・閾値・検出器版4・計画版2は維持し、列定義CRCで保存形式を識別する。

SLIPは同じ一次参照・正常ゴールのDISTANCEログから軸別の有効ms/ON msを集計する。不正・飽和・未観測を滑りなしへ置換しない。毎回一次計画から再生成し、DISTANCE→SLIPのオートスタート接続を確認する。未検証の滑り距離推定は制御へ入れない。最初の新形式実機ログと10本以上の実走行再現性は別に確認する。

## Overview

Use this skill for parameter tuning and control-improvement decisions. Keep project-wide limits, units, parameter locations, and safety constraints in `AGENTS.md` as the source of truth.

## Priority

Optimize in this order:

1. Reproducibility
2. Finish rate
3. Lap time reduction

Do not adopt a change that improves finish rate or lap time but worsens reproducibility.

## Workflow

1. Identify the target mode and compare only compatible logs:
   - first runs with first runs
   - secondary runs with the same secondary mode
   - autoStart only against runs where all 5 runs completed
2. Validate logs using the log-analysis skill.
3. Change at most two parameters per run.
4. Tune in this order:
   - PID
   - speed feedforward
   - slip detection
   - speed
5. When increasing speed, start from sections with smaller curvature radius.
6. After a control change, collect at least 10 real-run logs.
7. Compare against logs from before the code or parameter change.
8. Adopt a change only when the goal time improves stably without reducing reproducibility.

## Allowed Risks

Accept only these risks for time reduction:

- mild running slip
- high current
- high voltage

If high-speed behavior gets worse:

- adjust PID parameters first
- lower speed parameters if needed

## Parameter Handling

- Do not rely on the log header to record parameter changes.
- Save comparison results under `analysis/`.
- Include comparison logs, changed values, adoption decision, and remaining issues.
- Confirm relevant parameter locations in `AGENTS.md` before editing code.

## Review Before Editing

For firmware or parameter changes, check:

- whether the 1 ms control period is affected
- whether log format remains compatible
- whether real-machine safety constraints remain satisfied
- whether generated logs can still support the comparison workflow

## Offline SLIP speed-plan diagnosis

Regenerate the primary DISTANCE plan before every trial; do not reuse a previously SLIP-adjusted cache. Match saved speed/lead parameters. Separate base-plan lead, risk scaling, the SLIP-stage lead, forward/backward speed-difference limits and integer target conversion. A 5% increase before postprocessing does not imply the final plan is exactly 1.05 times the base. Label counterfactual lead/risk removal as offline sensitivity, not a proven lap-time gain. Corrected distance can step backward at markers; compare by plan index or validated event alignment.

## Slip detector SHADOW evaluation

閾値選定前に[XY基準・動画の優先順](../../../doc/slip-evaluation-policy.md)で正解ラベルを固定する。XYを先に判定し、その後の動画判定が不確定ならXYを正として閾値を調整する。明確な動画判定を優先し、既存動画ラベルを勝手に不確定へ変えない。走行全体のXYラベルを全ONの正検出や局所時刻の正解へ変換しない。選定用/確認用と他の走行条件を固定し、解析だけの回はファームウェア・設定を変更しない。

For the 2026-10-12 v2 parallel evaluation, keep the legacy detector connected to control, next-run speed planning and PATH distance correction. Inspect the new validity, interval peaks, ON time and invalid reasons before interpreting flag frequency. The user restored the 72-column normal profile on 2026-10-14, so both normal72 and detailed87 record the diagnostics. Temporary normal43 logs retain legacy flags but do not record new detector output. Do not interpret missing diagnostics as zero. Use doc/slip-detector-v2.md and the log-analysis skill for units and schema compatibility. Do not tune the detector and speed profile in the same comparison. DWT timing work, when requested, requires a matching instrumented ELF and separate verification; sparse logs do not prove ISR timing. Real-machine adoption still requires the first new-format log and at least 10 compatible runs; host tests or sparse-log replay do not establish physical slip accuracy.

For the 2026-10-14 longitudinal comparison, use `doc/slip-long-fast-only.md`: version 3 removes Slow from both ON/OFF conditions, keeping 10/8 m/s², 3/5ms and all windows fixed. LongSlow remains diagnostic and legacy control is unchanged. Identify version 2/3 from metadata; do not adjust thresholds, durations or speed plans while isolating this effect.

For the lateral tangential candidate, use `doc/slip-lateral-candidate.md`. Keep thresholds and speed plans fixed while comparing the initial SHADOW lane and candidate in at least 10 same-mode, similar-voltage runs. Verify physical X direction before adoption. Separate steady turns/transients, CW/CCW and speed bands; compare residuals, interval peaks, ON time, validity, angular-acceleration dependence, detection position, completion, speed tracking and lap time. A lower ON rate alone does not establish more accurate slip detection. Existing IMU compensation and control remain connected to the initial paths; adoption is a separate change. Account for remaining angular-acceleration LPF lag.

## 2026-10-27：白線融合の観測点と実制御再生

融合比較の候補を作る前に、伝播する車体基準点・地図の基準点・センサー観測点を区別する。前方白線を車軸XYへ直接照合せず、一次の実センサー観測で小R区間の残差を確認する。全列棚卸しと有限の全候補を明記し、欠測列・未記録鮮度・重力基準を推定済み事実にしない。選定用で候補固定後に確認用を評価する。方法変更後に使い直す系列は新しい未見評価ではない。実制御の最近傍・ゲート・1ms補正上限・停止を含む原本bin再生を実施し、直線評価点の改善だけで採用しない。詳細は[白線観測点の混同](../../../doc/失敗事例/2026-10-27-path-white-observation-reference.md)と[版19](../../../doc/path-confidence-fusion.md)。初回実ログと同条件10走の採否確認は別途必要。

1ms化する前に観測雑音・伝播雑音の時間単位を照合し、疎なログ値を独立した1ms観測として重複融合しない。保存optimalIndexに依存するPC診断と、推定結果で最近傍を選ぶ実制御を分けて検証する。単一線の接線位置や真biasの精度を数値最適化だけで保証しない。既知bias合成試験が期待精度を満たさない場合も、診断を保全し、位置改善・有限性の成功でその失敗を消さない。


## 2026-10-29：白線未照合中の速度方針を変える前

白線の明部と地図への有効照合を分け、未照合の初回周期・継続・回復、残差超過/地図欠測/古い取得、予定減速、方式分離と喪失停止を実Cで検証する。入口はリポジトリルート・必要権限のvalidation/path_unmatched_speed/run.py。失敗13898と同一次の正常13899、成功13901～13910を保持し、後の正常ADC集計を以前の失敗へ帰属させない。固定運動再生は速度指令の性質の検査であり、実閉ループの逸脱防止を証明しない。既に収集済みの版20の10走と、新しい方針の実走確認を分ける。[未照合中の再加速](../../../doc/失敗事例/2026-10-29-path-unmatched-reacceleration.md)参照。
