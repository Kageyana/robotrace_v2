# 手動走行の参照一次ログ番号

手動で一次ログを解析し、DISTANCE・SLIP・PATH・SHORTCUTを走らせた場合も、既存メタデータ`primaryLogNumber`へ計画元の一次ログ番号を記録する。

従来の手動ログは、この値が常に0だった。変更後は`initLog()`で、DISTANCE/SLIPは有効な距離計画と`analyzedNumber`、PATH/SHORTCUTは有効経路の`pathRouteSourceLog()`を照合して番号を保持する。停止時の保存は、その開始時点の値を使う。SLIPの一次番号は、スリップ評価元のDISTANCE番号ではなく、距離計画の元となった一次番号。

設定から読んだ古い解析番号だけでは参照を記録しない。対応する共有RAMの所有者と有効計画の要素数が必要。手動一次・計画なし・未取得時は0。自動走行の一次/SLIP参照保存は従来どおり。

`autoRunNumber=0,requestedMode=MANUAL,slipSourceLogNumber=0`という既存の手動属性は維持する。一次参照が非0でも自動走行系列とは限らない。手動DISTANCEとSLIPの方式名を追加する変更は含まない。

例：一次13458の距離計画を手動解析して走行したログでは、`autoRunNumber=0,requestedMode=MANUAL,primaryLogNumber=13458,slipSourceLogNumber=0`となる。

CSVの行データ列・通常72列/183B・詳細87列/220B・スキーマは変更しない。既存キーの値を更新するため、PC変換器の更新は不要。過去ログの0を推測で書き換えない。

## 確認

- `tests/run_manual_primary_metadata_tests.py`：通常/詳細の実C保存・bin読込・CSV変換経路で、手動DISTANCE/SLIP/PATH/SHORTCUT、開始後の解析番号/経路無効化、解析失敗・計画なし、参照切替、自動5方式のメタデータを確認。
- `tests/run_log_deferred_tests.ps1`：既存の保存・CRC・障害注入・復旧・DISTANCE/SLIP/PATH計画回帰が両形式で成功。試験中のf_writeエラー表示は意図した障害注入。
- Debug/Release×通常/詳細の4構成ビルド成功、新規warningなし。開始時番号の保持は2B、静的RAM使用端は前回から増加なし。予約スタック8192B・検出器状態1640Bを維持。

ビルド・ホスト出力・ソースSHA256は`analysis/manual_primary_log_metadata/`へ保存。1ms割り込みの変更はなく、番号取得は停止中の走行開始準備、メタデータ生成は停止後の保存処理で行う。

実機への書込みは実施していない。変更後の初回実ログで、手動解析に選んだ一次番号と`primaryLogNumber`の一致を確認する。実機ログによる確認は未完了。

上記は実装検証時点の状況。その後のユーザー実走では、手動DISTANCE13536に一次13535、13539～13542に一次13538が保存されていることを確認した。方式はユーザー回答と照合し、一次番号だけでDISTANCE/SLIPを推定していない。証跡は`analysis/slip_candidate_v5_13534_13536/`、`analysis/slip_candidate_v5_on65_13537_13542/`のparameters.json・validation.json。
