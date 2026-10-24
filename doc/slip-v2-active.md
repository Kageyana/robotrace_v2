# V2の正式採用とSLIP走行

2026-10-21のユーザー指示「スリップ検出をslipv2にアップデート、v1との互換性は不要、SLIP走行を可能にする」に対応する。以前のSHADOW並行評価から、V2だけを使用する構成へ移行した。

## 判定

- 検出器の保存版は`slipDetectorVersion=4`、方式は`slipDetectorMode=ACTIVE_V2`。
- 縦は`slipV2LongFast`だけでON/OFFを判定する。ON 10/OFF 8 m/s²、3/5ms。`slipV2LongSlow`は診断として維持する。
- 横は時間応答を整合した候補版5を正式採用し、`slipV2Lat`へ記録する。ON 6.7/OFF 4.5 m/s²、3/5ms、LPF 0.8、`slipLatPolicy=matched_candidate_v5`。
- `slipV2Flags`のbit0は縦、bit1は採用横、bit2はYaw不整合。`slipV2Lat`・`slipV2PeakLat`・`slipV2OnLatMs`も採用横の残差・ピーク・ON時間へ変更した。採用前の基準横4.6/3.8の意味へ読み替えない。
- 横独自の`slipV2LatValid/Reason/ValidMs/IntervalReason`を保存する。横接線項だけが無効なときも、縦の有効性と分ける。`slipV2LatCandidate`・`…Flag`・`…Peak`・`…OnMs`の重複4列は削除した。内部の候補状態と診断メタデータは判定実装の識別に使うため維持する。
- V1の検出コード、電流/PWMによる旧ゲート、LEGACY/SHADOW選択、旧距離スケール補正を削除した。未検証の滑り量推定モデルは制御へ組み込まない。
- 1msのセンサー取得→V2更新→制御・ログスナップショットの順序を維持する。割り込みでSD読込・書込・表示処理を追加していない。

2026-10-24の既存変更で横ONを6.5から6.7 m/s²へ調整した。保存検出器版4・横OFF4.5・窓/LPF/継続時間は維持する。19:59:54ビルドの13589〜13599で保存閾値6.699999809（floatの6.7相当）を確認。一次の短い不要ONは残り、調査中断時の採用値として記録する。

## SLIP計画

既存オートスタートの`DISTANCE → SLIP`を使用する。既定では一次→DISTANCE→SLIP→PATH→SHORTCUT。`auto_run.txt`の方式と依存関係は変更しない。

1. SLIP開始前に指定一次からDISTANCE計画を毎回再生成する。既にSLIP補正したPPADを再利用しない。
2. 同じ一次参照・正常ゴール・`optimalTrace=2`・`slipSourceLogNumber=0`のDISTANCEログを開く。期待行数一致と有効なV2統計が必要。
3. `optimalIndex`別に縦の`slipV2OnLongMs / slipV2ValidMs`、横の`slipV2OnLatMs / slipV2LatValidMs`を集計する。瞬時フラグがOFFでも区間ON時間を反映する。横の区間無効理由は`slipV2LatIntervalReason`を使う。
4. 混合有効区間や無効理由を含む区間は該当軸の集計から外す。各軸の有効時間が全体で0なら解析失敗とする。軸別に未観測の区間は滑りなしとして増速させない。
5. 最小ON時間は3ms。既存のリスク閾値、近傍拡張、減速・増速係数、先行減速、速度変化制限は維持する。増速後も既定の機体上限10m/sへ飽和させる。これは滑り距離の補正ではなく次走速度計画の調整である。
6. カウンタ不正・飽和・不正行・行数不足・停止・参照違いは負のエラーで返し、呼出側はSLIP走行を開始しない。

SLIP解析用RAMへ横の有効時間配列2KiBを追加した。その他の配列は観測行数から有効ms/ON msへ意味を変更した。すべて停止中・非記録中・FatFsロック下で処理する。

手動解析で作成したSLIP計画も、`slipSourceLogNumber`へ実際の元DISTANCEログ番号を走行開始時に固定して記録する。通常DISTANCEは0。SLIPログを次のDISTANCE解析元へ誤用しない。

## ログ形式と移行

横ログ重複整理後は通常66列/170B、詳細72列/179B。正式採用直後の通常70列/181B、詳細76列/190Bから重複4列・11B/行を削減した。残した横専用4列の型・単位・保存元は変えず名前だけ変更した。旧`slipFlag`, `slipFlagLat`と旧詳細診断9列も存在しない。列名・型・書式をRLOG v2の自己記述スキーマへ保存し、CRC32/IEEE検証を維持する。コンテナのRLOG v2と、スリップ検出器の保存版4は別の版番号である。今回の判定アルゴリズムは同じなので検出器版4と計画版2を維持し、保存形式は列定義・スキーマCRCで区別する。

|変更前の列|整理後|
|---|---|
|slipV2LatCandidate|slipV2Latへ統合|
|slipV2LatCandidateFlag|slipV2Flagsの横bitへ統合|
|slipV2LatCandidatePeak|slipV2PeakLatへ統合|
|slipV2LatCandidateOnMs|slipV2OnLatMsへ統合|
|slipV2LatCandidateValid|slipV2LatValidへ改名|
|slipV2LatCandidateReason|slipV2LatReasonへ改名|
|slipV2LatCandidateValidMs|slipV2LatValidMsへ改名|
|slipV2LatCandidateIntervalReason|slipV2LatIntervalReasonへ改名|

V1・旧Candidate列名へのフォールバックはない。更新前の未取込binは先にPCへ取り込み、原本の保全を確認する。機体のバイナリ読込は現在のスキーマと一致するログだけを使うため、更新後は新ファームで一次・DISTANCEを取り直してSLIPへ進む。起動時に旧スキーマの未取込binが残っていると既存のログ検証で開始禁止になるため、機体側で旧binを自動削除・上書きしない。CSVも現在の横専用列名を必要とする。PC変換器は自己記述列定義に従い、既存ログ原本は元の列のまま保全する。

## 確認と残る実機確認

- ホスト：縦Fast単独、採用横とビット/区間ON時間の一致、無効・再充填、V2列の並替、V1列のみの拒否、区間時間不正の拒否。
- ホスト：通常/詳細の実SDcard/courseAnalysisを使ったCSV・バイナリ計画一致、保存/CRC/I/O/復旧・PATHの回帰。
- 重複整理の前後比較では、実CSV13549～13557の原本を保持し、新列へ投影したテスト専用コピーを使用する。全8本・各469要素のSLIP計画が変更前と一致することを通常/詳細で確認する。これは新形式の実走行ログではない。
- 正式採用直後の70列実機ログ13571～13577では、保存とDISTANCE→SLIPへの接続を確認した。ただし元13574の終端不整合と13575のXY条件外があり、改善の再現性を証明していない。今回の66列/72列の最初の実機ログ、制御周期の最大実測時間、10本以上の同条件走行による再現性は実機で確認する。

今回の再現手順と結果は`analysis/slip-lat-log-dedup/README.md`、導入時の旧70列検証は`analysis/slip-v2-active-validation/README.md`。ホストテストは`robotrace_v2/tests/slip_v2_active_test.c`と`run_log_deferred_tests.ps1`。テスト・解析成果物はリポジトリのGit除外設定に従う。
