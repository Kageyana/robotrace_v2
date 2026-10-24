# V2の正式採用とSLIP走行

V1の検出・LEGACY/SHADOW切替・旧距離スケール補正を削除し、V2を正式採用する。

縦はLongFast単独10/8 m/s²・3/5 ms、横は応答整合した候補版5の6.5/4.5 m/s²・3/5 ms・LPF0.8。検出器版4、ACTIVE_V2、fast_only、matched_candidate_v5とする。slipV2Flagsのbit0=縦、bit1=採用横、bit2=Yaw不整合。slipV2Lat/PeakLat/OnLatMsも採用横を表す。

導入時の保存形式は通常70列/181 B、詳細76列/190 B。横Candidateの独立した有効性・理由と重複診断列を保持する。RLOG v2の自己記述スキーマと全CRC検証を維持し、旧V1列へのフォールバックは設けない。

SLIPは同じ一次を参照した正常終了DISTANCEログの軸別ValidMs/ON msと区間無効理由をoptimalIndex別に集計する。横はslipV2LatCandidateValidMs/OnMs/IntervalReasonを使う。一次計画から毎回再生成して二重補正を防ぎ、無効・飽和・未観測を滑りなしに置換しない。最小ON3 ms、既存の速度倍率・近傍拡張・速度変化制限を維持し、上限10 m/sへ飽和する。手動SLIPも元DISTANCE番号をslipSourceLogNumberへ保存する。

縦横有効時間を独立集計するため横有効ms配列2 KiBを追加した。解析は停止中・非記録中・FatFsロック下で行う。1 ms割り込みではセンサー取得→V2更新→制御/保存の順序を維持し、SD処理を追加しない。

Debug/Release/DebugMarker、通常/詳細の保存・CRC・故障注入・復旧・CSV/bin計画一致、実CSV13549参照13550〜13557の8計画生成を確認した。導入時の結果はanalysis/slip-v2-active-validation/。実機の検出精度・ISR最大時間・同条件10本以上の再現性は別途確認する。

更新後は新しい一次とDISTANCEを取得し、既存のDISTANCE→SLIPへ進む。未検証の滑り距離推定モデルは走行制御へ組み込まない。停止中の既存ログ原本を自動削除しない。
