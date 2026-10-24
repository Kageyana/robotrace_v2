---
type: codex-failure
date: 2026-10-24
task: mainの未使用コード・旧CSV処理整理
status: resolved
severity: low
tags: [codex/failure, host-test, fixture]
---

# 抽出依存と故障注入段階の前提を確認した

## 確認した結果

センサー角度の実関数を既存ADCハーネスへ追加した初回コンパイルは、`angleSensor`の未宣言で停止した。抽出済み変数範囲にこの公開変数が含まれると仮定していたが、実際は範囲外だった。製品コードのビルドエラーではない。

bin専用ハーネスでは、close故障を設定してopenが失敗すると期待し、アサートが停止した。正常openはファイルを保持するのでcloseは呼ばれず、テスト側の段階指定が誤っていた。

## 対処と再発防止

ADC抽出へ実ソースの`angleSensor`宣言と`IMU_PI_F`を追加した。抽出する関数の公開値・定数を追加前に列挙し、製品側をモック不足へ合わせない。

open/read/closeの故障注入を分け、close故障は正常open後の実close呼出しで確認する。bin保持と戻り値も同時に確認する。Skillの実行前参照へ、抽出依存と故障注入段階の確認手順を反映した。

## 検証

`python validation/line_sensor_adc/run.py`のO0/O2・Wall/Wextra/Werrorが成功。両端、内側8位置、全ゼロ、全最大値、1000以上の入力を確認した。

`python validation/log_storage/run.py --converter C:/tool/AutoCopyCsvFromSd/RlogToCsv.exe`で通常/詳細の保存・故障注入・CRC・計画生成・PC数値一致と現行V2が成功。このホスト側の前提修正はresolvedとする。実SDや実走行の検証結果を示すものではない。

## 2026-10-25：変更前参照コードの既存warning

速度設定の共通化で変更前の実writerを比較ハーネスへ取り込んだ際、旧writer内の`int16_t`と`sizeof`の比較が`-Wextra -Werror`で停止した。現行writerの問題ではなく、変更前ソースを厳格設定でコンパイルした際の既存診断だった。比較用コードにも現行と同じ警告契約が成立すると仮定していた。

変更前参照ブロックだけを`#pragma GCC diagnostic push/ignored "-Wsign-compare"/pop`で囲み、対象の既存診断を局所的に許容した。現行実関数・共通ヘッダのWerrorは維持し、製品側の警告設定は変更していない。比較参照を追加する前に、既存診断と新規診断を識別し、許容対象と範囲をテストソース内へ記載する。

O0/O2で速度設定178ケースの初期値・保存バイト・部分反映・範囲外・旧18項目修復が変更前と一致した。`validation/control_refactor/run.py`の参照ブロックには局所的なpush/popと理由を残した。検証済みのテスト契約修正としてresolved。旧writer自体の修正を示すものではない。
