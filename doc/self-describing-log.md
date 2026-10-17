# 自己記述RLOG v2

2026-10-16のユーザー指示により、保存列の追加・削除・順序変更でPC変換器を更新しない自己記述方式へ移行する。旧RLOG v1の固定6形式は読まない。既存CSVの直接取込は維持する。

## ファイル構成

`512 Bヘッダ → バイナリ記録 → 列定義 → 走行メタデータ`とする。記録部の開始位置とビッグエンディアン保存、レコード内の値、CRC32/IEEEは維持する。列定義を各レコードへ繰り返し保存しない。

ヘッダ整数はリトルエンディアン。magicはRLOG、versionは2。従来の番号・profile・recordBytes・rows・metadataBytes・dataCrc・metadataCrc・schemaの位置は同じ。schemaは列定義そのもののCRCへ変更し、offset 44/48/52へschemaBytes/columns/derivedVersionを追加した。offset 40のヘッダCRCは、その4 Bを除く512 B全体を対象にする。実長は512＋recordBytes×rows＋schemaBytes＋metadataBytesと一致させる。

列定義はUTF-8のASCII部分で、1列1行の`種別,型,列名,書式\n`。

```text
S,U16,cntlog,%d
S,F32,gyroVal_Z,%f
D,F32,ROC,%f
```

Sは保存列、Dは計算列。型はU8/U16/S16/U32/S32/F32。列名は63文字以内のC識別子、同名列は禁止。書式は型別の許可リストで制限し、任意printf指定は実行しない。現在の`%d`・`%ld`・`%f`と符号の扱いを保持し、U32の`%u`/`%lu`、F32の`%.0f`～`%.9f`にも対応する。

上限は256列、列定義4095 B、記録2048 B。実機ではログバッファに1記録が収まることをstatic_assertで確認する。上限を超える列追加や新しい保存型の追加は形式・変換器の更新が必要。

## 保存列と計算列

機体は`log_schema.h`から列定義を自動生成してFlashへ置く。保存列を既存の型で追加・削除・並替する際、PCの列一覧やoffset表の編集・再ビルドは不要。

PCはファイルの列定義から保存位置とCSV列順を復元する。計算版はderivedVersion=2で、ROCの処理は維持し、x/yの距離をマーカー補正前の左右生エンコーダ累積値の平均へ変更した。計算に必要なcntlog/encCurrentN/courseMarker/gyroVal_Z/encTotalL/encTotalR/imuAngle_Z、および計算列ROC/x/yは名前・型・S/D種別を検証する。encTotalL/RはS32で、int64の左右和を差分化してから0.5倍し、半パルス・後退を保持する。角度はimuAngle_Zの両端平均を使う。これらの削除や意味・型の変更、未知の計算列は変換を失敗させ、原本を残す。計算列変更時はPC変換器を更新する。

CSVメタデータにもlogDerivedVersion=2、logXYDistance=raw_wheel_averageを記録する。列数・保存レコード配置は変更しない。encTotalOptimalのマーカー補正は走行制御・次走計画に引き続き使用するが、ログXYへ加えない。新規保存時のゴール閉路Xも同じ生エンコーダ距離を使い、閉路Xの許容値・減速区間・制御中自己位置は変更しない。ユーザー指定により旧計算版1の互換処理は追加せず、旧binは原本を保持して拒否する。以下の計算版1での検証・実機確認は過去記録であり、計算版2の初回実機確認とは区別する。

機体の走行間解析は同じビルドのレコード定義を使い、列定義の完全一致を要求する。PC側の汎用変換とは用途が異なる。新しいビルドで過去の異なる列構成を機体解析元として使うことは保証しない。

## 検証

- 通常・詳細とも、変更前後のCSV列名・全データ行、DISTANCE/SLIP/PATH/SHORTCUTの全計画がバイト一致。メタデータのbinarySchemaは列定義CRCへの仕様変更により変わる。
- 同一のPC変換器exeで、保存列の追加・診断列削除・全列逆順・削除と追加と並替の組合せを変換できた。既存の計算列値は一致した。全6保存型の非ゼロ値、符号、丸めも検証した。
- ヘッダ・記録・列定義・メタデータのCRC、短い入力、重複列名、未知型・計算列・危険な書式、必須列欠落、旧v1を拒否した。失敗時に完成CSVを出力しない。
- 保存・故障注入・原本保持・走行計画・自動走行の既存回帰を通常/詳細で確認した。旧6形式の互換テストは対象外。

ローカル検証は`robotrace_v2/build/rlog-v2/validate.py`、結果は`validation.json`、比較前のコード・PC変換器は`baseline/`へ保全する。既存のtests/build管理対象外方針を維持する。

## 配備と実機確認の状態

`C:\tool\AutoCopyCsvFromSd`の変換器をv2へ更新し、監視を再開した。隔離フォルダから実PowerShellスクリプトを実行し、通常・列順変更・列追加のコピー／変換／保存、原本保持オプション、旧v1・破損列定義の拒否と原本保持を確認した。結果は`build/rlog-v2/import-validation.json`。

Debug/Release×通常/詳細とDebugMarkerの全5構成がビルド成功。新規warningはなく、既存warningは残る。ログI/O共有RAMは6144 B、スタック予約8192 Bを維持した。通常Debugの列定義は終端NUL込み1687 BでFlashへ配置される。

初回のCodexによる書込みは未実施。2026-10-16の接続確認でST-LinkのSN/FWと3.24 Vは取得できたが、`Unable to get core ID`／`No STM32 target found`で停止した。Device ID/Core IDを確認できるまで書込みへ進まない。以後のユーザーによる書込み・実走行と新形式の確認結果は以下のとおり。

## 2026-10-16 実走行ログと実カード取込

ユーザーが新ファームを書込み、走行後にSDをPCへ挿入した。G:のRT_LOG（exFAT）から13288～13297の10本を常駐監視が自動変換・保存した。5走×2セットで、各セットの順はPRIMARY/PATH/SHORTCUT/DISTANCE/SLIP。全ログはRLOG v2、通常プロファイル、183 B/記録、72列、列定義1686 B、derivedVersion=1。ヘッダ・記録・列定義・メタデータの全CRC、temp原本SHA256とディレクトリ名、CSVの全行列数・有限値・CRLF、tempと正式CSVの全バイト一致、期待行数一致を独立に確認した。

行数は順に4675/4742/4741/4738/4717、4670/4741/4738/4720/4715。全ログのemcStop=0、両一次ログのclosureValid=1・closureReason=0。build情報はgitCommit=b809f51、Oct 16 2026 01:09:36。後続ログの一次参照番号も各セットに一致する。これは新形式での保存と機体内解析後の5走進行・PC取込の確認であり、ラップタイム改善や経路精度の評価ではない。

正式CSVは`F:\Dropbox\Document\robotrace\Log\v2`へ保存された。SD上の該当bin/CSVは取り込み後に存在せず、tempの原本binとCSVは保持されている。変換器stderrは全10本とも空。確認スクリプトは`robotrace_v2/build/rlog-v2/verify_actual_card.py`、全CRC・ハッシュ・メタデータの結果は同フォルダの`actual-card-import-13288-13297.json`へ保存した。CodexによるSWD照合・再起動復旧と実測速度比較は今回実施していない。
