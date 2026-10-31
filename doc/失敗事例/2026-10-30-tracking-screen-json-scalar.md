# NumPyの集計整数でJSON証跡の保存が停止した

- 対象: 追従偏差ガード160条件×46系列×O0/O2。
- 確認結果: Cの14720条件は完了し最適化間一致まで確認したが、normal_affected_streamsがNumPy int64となりjson.dumpsでTypeError。JSON保存の直前で停止し、C出力は残っていた。実機・製品は未変更。
- 原因: NumPy boolのsumをPythonの組込みintへ変換していなかった。
- 対処: before-native-json-castへ旧生成器を保持し、当該項目をintで明示変換。--aggregate-onlyを追加して完成済み92集計CSVの行数/有限性/最適化間バイト一致と実在exe/単独試験ログを確認し、Cケースを再実行せず集計を再開した。
- 再発防止: NumPy集計の出力スキーマをint/float/boolへ明示変換し、default=str等で意味や型を隠さない。最終集計だけの失敗を実Cジョブの未完と扱わず、同じ成果物の完了を検証して未実行部分だけ再開する。
- 確認方法: screen_tracking_guard.py --aggregate-only、screen_validation.jsonのaggregation_resumed_from_completed_C_outputs=true、全14720条件と後続実制御276再生の証跡。
- ステータス: resolved（集計保存と再開）。実機安全性は未確認。
