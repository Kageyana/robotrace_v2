# 走行方式ごとのRAM共通化

2026-10-04、ブランチ `codex/shared-run-ram`。比較元は `4d406d0`。

## 使用期間と実装

DISTANCE走行のPPAD・markerPosはSLIP解析でも使用する。SLIP作業配列と同じ構造体に保持し、PATH/SHORTCUTの経路配列とはunionで重ねる。動的確保は使用しない。

- `runMemory.distance`: PPAD、マーカー、SLIPの集計・リスク・速度配列。44,000 B。
- `runMemory.path`: ライン経路、走行経路、弧長、フラグ。28,766 B。
- `runMemory`: 大きい側の44,000 Bだけを静的確保。
- `runAnalysisLine`: 距離・SLIP・PATHのログ読込行を6,144 Bで共有。距離とSLIPの読込上限4,096 Bは維持。
- 距離解析のROCbuffは、実際に使用する12要素に縮小。24 Bをスタックに置く。

PPADとマーカー、PATHのライン経路と走行経路は、それぞれ同時使用するので重ねない。経路生成・追従算法、速度計画、配列容量、ログ形式は維持する。

`runMemoryPrepare()`は走行中（patternTrace 11～99）またはログ記録中の切替を拒否する。呼出し元でFatFsロックを取得後、optimalTraceをNONEにし、旧計画の要素数・経路追従状態を無効化してから配列を書き換える。SLIPの一次計画再利用にはRAM所有方式も確認する。PATHは終端延長まで成功した後に走行方式を有効にする。ログ用の走行開始時メタデータは共有領域外に保持する。

1ms/5ms割り込みの配列参照は走行方式で分岐している。共有領域を書き換える処理は停止中のメイン処理だけに置き、割り込み内でのクリア・SD処理は追加していない。

## ビルド結果

使用量はリンカが報告するRAM消費量（ヒープ512 B・スタック8 KiBの予約を含む）。物理RAMは131,072 B。

| 構成 | 変更前 | 変更後 | 削減 | 変更後の未割当RAM |
|---|---:|---:|---:|---:|
| Debug | 123,168 B | 85,280 B | 37,888 B | 45,792 B |
| Release | 123,064 B | 85,176 B | 37,888 B | 45,896 B |
| DebugMarker | 123,216 B | 85,328 B | 37,888 B | 45,744 B |

全構成のconfigure/build成功。変更した実装に新規warningなし。既存warningは残る。nmでrunMemory=44,000 B、runAnalysisLine=6,144 B、スタック予約0x2000、スタック上端0x20020000を確認。

## PC検証

リポジトリ方針により `robotrace_v2/tests/` はローカル検証用でGit管理対象外。

- `tests/run_pc_tests.ps1`: 自動走行・CSV・PATH統合、PATH→DISTANCE→PATH再生成の経路CRC一致、走行中/記録中の切替拒否、SLIP作業配列全体更新後のPPAD/マーカー保持、停止時ログメタデータ保持。
- `python tests/run_distance_slip_memory_tests.py`: 変更元と変更後の実装から同じ模擬CSVを解析し、全速度・ROC・マーカー位置が一致。一次計画のキャッシュ再利用、再読込、SLIP読込I/Oエラー後の再試行を含む。
- `tests/run_log_deferred_tests.ps1`: 通常・詳細のバイナリ/CSV変換、CRC、I/O障害、復旧、DISTANCE/SLIP/PATHの読込結果一致。
- `tests/run_log_xy_tests.ps1`: 距離・姿勢からのXYと閉路計算。

## 残る実機確認

2026-10-04の12941～12945で、一次→PATH→SHORTCUT要求（実際はPATH Level 0）→DISTANCE→SLIPの実走行とCSV保存を確認した。詳細は下記。DISTANCE→PATH方向の実機切替、SHORTCUT Level 1、変更前と同条件での速度計画・再現性比較は未確認。CSV変換・表示・割り込み併用時の最大スタック/ヒープ使用量は実測が必要。

## 実走行ログ12941～12945

全ログのbranchは `codex/shared-run-ram`、gitCommitは `4d406d0`、ビルドは2026-10-04 14:12:21。変更は未コミットのためgitCommitだけでは共通化版を識別できず、branch・ビルド時刻も確認した。

| ログ | 要求方式 | 実方式 | 最終記録時刻 | 行数/期待行数 |
|---|---|---|---:|---:|
| 12941 | PRIMARY | 一次 | 22.599 s | 4618/4618 |
| 12942 | PATH | PATH Level 0 | 23.006 s | 4693/4693 |
| 12943 | SHORTCUT | PATH Level 0 | 23.004 s | 4695/4695 |
| 12944 | DISTANCE | DISTANCE | 13.893 s | 4690/4690 |
| 12945 | SLIP | DISTANCE＋SLIP補正 | 14.780 s | 4679/4679 |

全5走でemcStop=0、41列、期待行数一致、有限値、cntlog単調増加、左右累積距離から見た不自然な記録間隔なし。記録時刻はゴール付近のログ時刻で、画面の厳密なgoalTimeではない。

一次12941は閉路X=39.76 mm、closureValid=1、IMU校正・距離換算検証成功。PATH系2走は同じ一次ログを参照し、SLIPは12944を参照している。PATH後のDISTANCE/SLIPはrouteSourceLog=0で、旧経路無効化と一致する。共有RAM切替後の走行開始・正常終了・ログ保存に、今回の1セットで明らかな異常は認められない。これだけでRAM破壊やスタック問題を全条件で否定するものではない。

SHORTCUT要求12943はoptimalTrace=4、shortcutLevel=0で、Level 1短縮は適用されていない。直前の変更前セット12936～12940も同じ実方式だった。新CSVには短縮不採用の理由がないため、今回のログから原因を特定しない。

電圧は今回は8.35～8.25 V、変更前セットは7.59～7.53 V。約0.72～0.76 Vの差があり、タイム・速度追従差をRAM共通化の効果と判断しない。DISTANCE/SLIPの推定XYには周回ごとのずれが見えるが、CSV欠落の証拠ではなく、今回の共通化との因果関係も未確定。

解析スクリプト: `analysis/script/check_shared_ram_runs_12941_12945.py`。成果物: `analysis/shared_ram_runs_12941_12945/report.md`、`summary.csv`、`validation.json`、`run_checks.png`。元CSVは変更せず、SHA-256を保存した。
