# 現行binの保存・解析回帰

ファームウェアの実`SDcard.c`・`courseAnalysis.c`・`pathFollower.c`をRAM上のFatFsへ接続する。保存スキーマや計画式をテスト用に再実装しない。V1/SHADOW切替、機体CSV変換、旧CSVの列名・混在ヘッダ互換は対象外とし、CSVしかない解析元は拒否する。

リポジトリルートで実行する。この環境ではGCCと生成exeを許可された制限外で起動する。

```powershell
python validation/log_storage/run.py
python validation/log_storage/run.py --converter C:/tool/AutoCopyCsvFromSd/RlogToCsv.exe
python validation/line_sensor_adc/run.py
```

通常・詳細の保存、開始時校正値と参照番号の保持、tmp保持、採番予約、binだけの一覧、全CRC・実長・不正スキーマの拒否、短いI/Oと各段階の故障注入、ストリーム終端CRC、軸別SLIP区間時間、DISTANCE/SLIP/PATH/SHORTCUTと経路キャッシュを確認する。別exeで現行V2判定とauto_run設定・依存関係・参照ログ選択・PATH方針も検証する。

共通`logMetadata.c`も実ソースをリンクする。CRC正常なbinのメタデータを置換し、PATH元の整数/小数の同値表記を受理し、重複キー・偽の接頭キー・行数の端数/範囲外などを拒否する。共通読込関数単体の数値境界と不正入力は`validation/control_refactor/run.py`で厳格な警告設定でも確認する。

出力は`build/log-storage-validation/`。`--converter`指定時は既存PC変換器で実保存binをCSV化し、型付き読込で得た50行の値と照合する。直線と閉路カーブのbin・計画ダンプも出力する。PC側の旧CSV取込や自己記述スキーマの汎用変換は変更しない。

2026-10-24の整理では、作業前ソースを`build/main-cleanup-before/`へ保存し、同じ入力の通常・詳細×直線・カーブについて記録・スキーマの全バイトと4方式の計画一致を確認した。比較結果は`build/log-storage-validation/before-after.json`。作業前スナップショットはローカルの比較資料で、通常の回帰実行には不要。

実機の書込み・走行・最大割込み時間・実SDでの停止保存と再起動は別途確認する。ログ列・型・レコード長・CRC方式・検出器閾値は今回変更していない。
