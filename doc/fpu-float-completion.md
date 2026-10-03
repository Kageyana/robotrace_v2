# 残りの浮動小数点演算の単精度化

## 対象と維持事項

採用済み制御float版のmain（6d0e21b）から`codex/fpu-float-completion`で検証する。

- UI: `dataTuning`の調整量・上下限をfloatへ統一。リポジトリ内呼出しと宣言を合わせて変更。int16調整値はfloatで全域を正確に表現できる。
- 描画: sin/cosをsinf/cosfへ変更。描画用円周率の従来値3.14は維持し、定数のみ単精度化。
- SD保存: `round`を`roundf`へ変更。対象は既にfloatで評価される目標値×100なので、丸め方式と19項目の保存形式は維持。
- CSV読取: 一時double配列をint32_t/floatの共用体へ変更。整数列はfloatを経由せず、2^24を超える値やINT32上下限の精度を保つ。列名解決、不正値・空欄・非有限値・範囲外値の拒否を維持。
- SD容量: 64bit整数積を2で除してKiBへ換算。floatによる容量精度低下と乗算前の32bitあふれを避ける。
- 残存演算: ライン角度計算のatanfと単精度円周率、fabsf、ラインLED周期・UI等の定数を単精度へ統一。

PIDゲイン、速度・加速度設定、緊急停止条件、ログ列、SD設定形式、制御周期は変更しない。printf系の必須double昇格とfloat対応リンカフラグは維持。ISR計測は既定OFF。一括算術オプションは追加しない。

## 確認

`python analysis/script/validate_fpu_float_completion.py`で実施。成果物は`analysis/fpu-float-completion/`。

- Debug/Release通常ビルド成功。変更前とwarning本文・種類が一致（Debug17件、Release19件）。
- ARM GCCのWdouble-promotion、Wfloat-conversion、Wunsuffixed-float-constantsでCoreを再検査。未指定の浮動定数と通常算術の不要なdouble昇格は解消。残るdouble昇格は表示・CSV出力の可変長引数。
- PC経路、XY、ログスキーマの既存テスト成功。
- 実装から抽出したUI調整関数で短押し、長押し、折り返し、逆順上下限、int16上限を検証。
- CSV列並替、INT32_MIN/MAX、2^24超の整数、不正値、欠落、NaN/Inf、範囲外値を検証。
- 半整数の直前・一致・直後についてround/roundfの整数化一致を60,003値で確認。
- 一次ログ12905と不正閉路ログ12879について、採用済みfloat版の経路結果・整数方位・生成可否・XY・閉路判定とバイト単位で一致。
- Debugの倍精度演算/昇格呼出し箇所はdataTuning26→0、CSV距離行6→0、CSVスリップ行7→0、設定保存writeTgtspeeds38→0、描画DrawArc12→0。表示ライブラリ内のdoubleは残る。

スクリプトの比較は事前保存したbefore-Debug/Release.txt、および第一段階float版のmanifestと一次ログ比較結果を使用する。両版を比べ直す場合は変更前でclean-firstビルドのログを保存してから実行する。既存testsはリポジトリ規約によりローカル管理。

## 実機確認

書き込みはまだ行っていない。既存採用版からの追加変更として、UIの増減・長押し・上下限、円弧描画、SD保存と再起動後の設定値一致、オートスタートの経路解析・5走完了、ログと停止位置を確認する。描画の整数化境界では単精度の丸めにより1px差が出る可能性がある。

未使用getAngleSensorに既存の未初期化index・隣接配列境界の問題があるが、今回有効化していない。用途変更時に別途修正・検証する。UI調整関数の引数型を変更しているため、追加の外部呼出し元がある場合はヘッダに合わせて再ビルドする。

この追加版の実走行時間・安定性改善は未確認。第一段階のDWT改善値を追加版の実績として扱わない。実機で問題が出た場合は保存済み第一段階float版ELFへ戻す。

## 実走行12928～12932（2026-10-04）

全ログのgitCommit=f3c2b85、autoStart=1～5、emcStop=0。PRIMARY→PATH→SHORTCUT要求→DISTANCE→SLIPの5走セットが完了した。行数は4624/4700/4700/4694/4676で期待行数と一致。列数不一致、非有限値、cntlog逆行、距離基準の明確な欠落は0。12928は閉路X=42.43mm、closureValid=1、IMU校正・距離換算検証成功。

ゴール付近の最終記録時刻は22.664/23.076/23.065/13.997/15.101秒。画面goalTimeはログに保存されていないため厳密なゴールタイムとは区別する。

第一段階float版12922～12926と同順番で比較すると、最終記録時刻差は-0.148/-0.007/+0.002/-0.147/-0.549秒。ヘッダ収録の速度・制御設定は一致。追加版の電圧は0.11～0.15V低く、一次ログ経路元も違う。各版1セットなのでfloat化の速度改善・再現性向上はまだ判断できない。

12930はSHORTCUT要求だが実際はoptimalTrace=4、shortcutLevel=0で、前版12924と同様にPATH Level 0だった。Level 1短縮の実機確認には含めない。二次ログのclosureValid=0 / closureReason=1は一次経路元検証の対象外で、異常終了ではない。

DISTANCE/SLIPの速度追従MAEは0.2017/0.1641m/sで前版とほぼ同程度。両版とも周回ごとの推定XYずれが残る。真の停止位置やPATHロスト状態は現行列だけでは直接判定できない。UI操作・描画・設定再読込の目視確認とISR処理時間測定は未確認。今回の実走行結果だけで追加版の採用・main統合は実施しない。

再現スクリプト: `python analysis/script/check_float_runs_12928_12932.py`。
解析結果: `analysis/fpu-float-completion/runs-12928-12932/report.md`、`summary.csv`、`validation.json`、`xy_speed.png`、`gyro_slip.png`。
