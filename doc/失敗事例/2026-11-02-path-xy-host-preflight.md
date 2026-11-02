# 停止後XY出力検証の前提確認

- 対象タスク: PATH/SHORTCUT系走行の停止後XY付属ログ。
- 確認した事実: 旧標準配置`C:/tool/AutoCopyCsvFromSd`はTest-Pathで欠落を確認した。後続調査で、実在確認済みの親`validation`から未確認の子`log_storage`をrgへ渡し、パス不存在で終了2になった。またFatFs設定の所在を`FATFS/App/ffconf.h`と推測した検索も終了2だった。変更やビルドには進まず、実在一覧から`validation/mode_log_diagnostics/run.py`と`FATFS/Target/ffconf.h`を特定した。
- 確認した事実: ホスト試験で、常時trueの`logStorageCanRead()`モックを実装抽出へ差し替えたところ、既存makeFileフィクスチャの`modeLOG=true`が読込時にも残り、`logSourceOpen`の停止中ガードで拒否されてアサートが停止した。製品の不具合は確認していない。
- 原因: 親ディレクトリの実在を子や慣例的ファイル名の実在確認に代用した。試験では既存モックが隠していた記録中/停止中の状態遷移を引き継がなかった。
- 対処: 所在は実在一覧から選択。ホスト用makeFileは生成時にmodeLOG=true、出力close後にfalseへ遷移するよう修正した。製品のガードを緩和せず、実logStorageCanReadと実writer/typed reader、実endLogを使用する。
- 再発防止: 親が存在しても未知の子・ファイルは個別Test-Pathまたはrg --filesの結果を確認してから検索する。停止時APIを実装へ差替える前に、記録フラグ、キュー、ファイルのopen/close、共有RAM所有者、ロックの状態遷移をフィクスチャに明記する。録画中の拒否はI/O回数不変まで検査する。既存試験の常時成功モックを実行可否の証拠にしない。
- 確認方法: `validation/path_xy/run.py`の通常/詳細×O0/O2。実writer/readerの全CRCと全XY、344件の付属出力I/O故障/不足、実endLogの6走行モードと188件の保存失敗を検証する。Debug/Releaseビルドも実行する。
- `record-failures` Skillは今回のSkill一覧、個人Skill、プラグインcache内のSKILL.md一覧の検索で見つからず、AGENTS.mdの指定項目を満たす手動記録とした。
- 検証結果: `build/path-xy-20261102/validation.json`の通常/詳細×O0/O2が全件成功。各条件で全4080点のCSV座標照合、344件の付属出力故障/不足、188件の実endLog保存失敗を確認した。Debug/Releaseビルド成功、変更対象のSDcard.c/pathFollower.cにコンパイル診断なし。両mapでI/O領域0x1800 B・スタック予約0x2000 Bを確認した。Skillの再発防止参照2件へ実行前確認を追記済み。
- ステータス: resolved（パス確認とホスト状態遷移の修正）。実SD上の保存時間・実機最大スタック使用量は別途未確認。

## ユーザー指定による保存契約の修正

- 事実: 最初の実装は付属`番号_path.csv`を作成したが、ユーザーは他のログ追加と同様に走行ログbinへ列追加することを指定した。また「最後の2列をXY」とする中間方針はユーザーが取り消し、既存XYの列位置を維持するよう指定した。
- 原因: 比較用途を優先して独立した点列ファイルを選び、既存のログ列追加の運用を満たす出力先・行対応・列位置を実装契約にしなかった。
- 対処: 付属CSVの生成とファイル名メタデータを削除。5列をPATH形式の既存末尾へ追加し、停止後に同じtmpへ補完して通常のCRC確定を行う。既存x/yと他の全既存列の位置・保存値を維持する。一次は補正前の原本から20mm補間し、optimalIndexに対応づけ、一次にない延長はflagsで区別する。
- 再発防止: ログ追加では出力先bin・既存列全体の位置・追加列の位置・行対応・欠測表現を先に固定する。全既存schemaのprefix一致と、全既存保存値の一致を自動検査する。後からのユーザー訂正を古い方針より優先し、既存XYを末尾へ移動しない。
- 確認入口: `validation/path_xy_columns/run.py`。以前の`validation/path_xy/run.py`は付属CSV試作時の履歴で、現行形式の確認には使用しない。通常/詳細×O0/O2で、全既存列のprefix一致、保存値一致、独立XY補間、1380一次点+13延長点・逆行index・複数バッチ、6走行モード、712件の実endLog I/O故障/不足が成功した。
- ステータス: resolved（bin列追加への修正と列位置・保存値の検証）。配備PC変換器、実機の停止後処理時間・初回ログは未確認。
