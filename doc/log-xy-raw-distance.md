# XYをマーカー補正前の距離から計算する

2026-10-17のユーザー指示により、ログXYの距離入力を`encTotalOptimal`から`(encTotalL + encTotalR) / 2`へ変更した。カーブ/CROSSマーカーの位置補正で補正距離が戻っても、XYにはその段差を加えない。角度は保存済み`imuAngle_Z`の区間両端平均を継続使用する。

左右累積値の和と差分はint64で保持し、差分後に0.5倍してmmへ換算する。半パルス・符号付き後退・大きな左右累積値を扱う。停止中のログ処理と閉路X算出で共用し、1ms制御・マーカー距離補正・スリップ検出・次走速度計画は変更していない。閉路Xの許容値と減速区間も変更しない。

新規ログは`derivedVersion=2`、CSVメタデータは`logDerivedVersion=2,logXYDistance=raw_wheel_average`とする。列数と保存レコード配置は変更しない。PC変換器`C:\tool\AutoCopyCsvFromSd\RlogToCsv.exe`も更新した。互換性不要のユーザー指定に従い、旧計算版1のbinは拒否し、原本は削除しない。既存CSV・実走動画・設定は書き換えていない。

## ローカル確認

- 実Cから抽出したXY関数で、直進・姿勢だけの変化・6周閉路・走行間初期化・半パルス・後退・int32左右和と大きな累積値の後の微小増分を確認。
- 通常/詳細の保存・CRC・故障注入・再起動・DISTANCE/SLIP/PATH計画回帰が成功。補正距離が1000→800→1600pulseと戻り、生左右平均が1000.5→2000.5→3000.5pulseと進む入力で、XYが生左右平均を使うことを確認。
- 同じ補正段差付きbinから、実ファームウェアCとPC変換器のCSVが通常/詳細とも全バイト一致。encTotalOptimalだけの変更がXY/ROCへ影響しないこと、列順変更、左右累積列欠落・旧版1・未知版の拒否を確認。
- Debug/Release×通常/詳細の4構成がビルド成功。変更箇所由来の新規warningなし。既存warningは残る。RAMはDebug通常86424 B、Debug詳細86472 B、Release通常/詳細86224 B。ログI/O共有6144 B、予約スタック8192 Bを維持。

再実行は`robotrace_v2/tests/run_log_xy_tests.ps1`、`run_log_deferred_tests.ps1`、`analysis/script/validate_xy_raw_distance.py`。結果は`analysis/xy_raw_distance/validation.json`、`build-validation.json`へ保存した。tests/analysisは既存方針どおりGit管理対象外。

## 実機確認

現在のmain作業ツリーへ反映。コミット・SWD書込みは未実施。変更後初回のPRIMARY/DISTANCE/SLIPで、版2メタデータ、原本CRC、行列数、マーカー通過前後のXY連続性を確認する。滑り検出精度・対地速度精度の保証とは区別する。
