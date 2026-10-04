# 全ログ分類

入力は `F:\Dropbox\Document\robotrace\Log\v2\old` 直下と `course_XXX` 直下のCSV。参照コース内のCSVは移動対象にしない。ファームウェアの走行方式・CSV出力形式は変更しない。PythonとNumPyが必要。

```powershell
python analysis/script/test_classify_logs.py
python analysis/script/classify_logs.py
python analysis/script/apply_classification.py --dry-run
```

別のローカルコピーでは両スクリプトの `--root` に同じディレクトリを指定する。分類の `--output` を変更した場合は移動側の `--plan` に対応する `move_plan.csv` を指定する。

分類スクリプトには移動処理がない。実移動を明示的に指示された場合のみ次を実行する。

```powershell
python analysis/script/apply_classification.py --apply
```

移動専用スクリプトは、計画自体のSHA256、対象ルート、AUTO、直下の番号付きCSV、元ファイルサイズ/SHA256、移動先重複、パスの範囲を全件確認する。参照ディレクトリからの移動や上書きは禁止する。Windowsでは既存宛先を上書きしないrename、POSIXでは排他的ハードリンク作成と元の名前削除を使い、`move_journal.jsonl` に予定と完了を保存する。途中で失敗した場合、計画をそのまま再実行せず、ジャーナルと両方の名前を確認してから分類を再実行する。

## 出力（analysis/output）

- `log_inventory.csv`: 全CSVの列・メタデータ・行数・サイズ・入力SHA256。
- `schema_summary.csv`: 列定義ごとの件数。
- `classification.csv`: 全件判定と比較要素・根拠。
- `review.csv`: 保留/未分類/異常、既存ラベルとの不一致。
- `move_plan.csv` / `.json`: AUTOの未分類ログのみ、対象のハッシュ付き。
- `classification_summary.md`: 全体集計と検証上の限界。
- `representatives.csv`: 正常候補からモード/年代別に選んだ複数代表。
- `validation.csv`: 自身のファイルを比較対象から外した既存分類との照合。
- `direction_anchors.csv`: 12838–12842=CW、12844–12848=CCWの参照情報。
- `direction_validation.csv`: 既知CW/CCWの25組の反転比較結果。絶対方向推定は記録方式に差があるLEFTを外し、SG/CROSSも照合する。
- `unknown_clusters.csv` / `.html`: 未知候補の仮クラスタと代表形状。
- `classification_cache.json`: サイズ/mtime一致で解析を再利用。疑わしい場合は `--refresh`。

## 判定方針

行長不足、数値欠落、NUL、非有限値、記録された停止/取得エラー/オーバーフロー、期待行数不一致、一次走行の距離逆行、折り返し以外の時刻逆行を異常候補とする。補正距離の戻りは、一次走行以外で総距離の10%以内の場合のみ単調包絡として特徴量化し、戻り回数と最大補正量も保存する。欠落項目を0扱いしない。`emcStop`がなければ完走判定は保留。closureReason=8、PATH/SHORTCUTのreason=1単独は異常扱いしない。過去のゴール回数を現在の定数から推測しない。

距離に対する512点の回転形状、曲率、マーカー位置/系列、旋回区間、総距離を比較する。gyro形状は角速度/エンコーダ速度で速度依存を減らす。ROCは逆数とgyroの符号を使用する。各比較要素の重みはマーカー位置30%、系列20%、旋回20%、gyro15%、ROC10%、距離5%。欠落要素は空欄とし、スコアは存在する要素で正規化するが、マーカー・gyro・旋回が欠落した通常ログはAUTOにしない。raw markerSensorとcourseMarkerは別定義として比較する。

通常、反転、反転+左右交換の全てを計算する。AUTOは総合0.86以上、2位との差0.06以上、マーカー系列完全一致、位置0.80以上、距離比0.94以上、旋回0.65以上、gyro0.75以上が必要。さらに既存ラベル照合でその予測コースのAUTO基準適合が5本以上、誤分類0本のときだけ移動候補を出す。この照合は外部テストセットではなく保守的な内部確認である。

PATH/SHORTCUTは正常候補の確定親から継承する。親番号重複・不明・異常・コース競合は継承せず、距離比92%未満はREVIEW。存在するpathState=4は異常。欠落したpathStateから健全性は証明しない。形状一致と経路の実機性能採否は別の判断である。

NORMAL_CANDIDATEは記録に矛盾がない候補で、実機の完走を保証しない。過去の機体寸法を推測しないため、距離比較はpulse比を使用する。finalDistanceは現行58019 pulse/mで換算した参考mm。既存コースのラベルは保存先を保持し、異常や不一致をレビュー表に併記する。

## 未知コースの登録

未知クラスタは正常候補のみを対象に、代表と総合0.90、マーカー系列完全一致、位置0.85、距離0.96以上でまとめる。クラスタは新コースの証明ではない。速度・記録方式・周回数の違いでも分かれることがある。

`unknown_clusters.html`で代表を確認する。`unknown_clusters.csv`を別の名前にコピーし、採用する行の`course`列へ`course_006`以降を記入する。他の列は変更しない。

```powershell
python analysis/script/classify_logs.py --cluster-map analysis/output/approved_clusters.csv
python analysis/script/apply_classification.py --dry-run
```

代表番号と全メンバーが一致したクラスタだけを明示登録としてAUTOにする。コース名を記入しない行は保留。入力が増減してクラスタが変わった場合は新しい表を再確認する。登録も実ファイルの移動は行わない。
