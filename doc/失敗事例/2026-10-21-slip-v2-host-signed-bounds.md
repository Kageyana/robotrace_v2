---
type: codex-failure
date: 2026-10-21
task: スリップV2正式採用とSLIP計画の実装
status: resolved
severity: low
---

# ARMとMinGWで整数上限マクロの符号型が異なる

V2のCSV区間カウンタ検証を追加後、ARM Debugビルドは成功したが、MinGWの`-Wall -Wextra -Werror`で`int32_t`と`UINT8_MAX`/`UINT16_MAX`の符号比較エラーとなった。MinGW側の上限マクロがunsignedとなる差によるもので、実機異常ではない。

上限がint32_tへ収まることを確認し、比較上限を`(int32_t)UINT8_MAX`、`(int32_t)UINT16_MAX`へ明示変換した。同様の行数上限にも適用した。警告設定や受理範囲は緩めていない。

再発防止は、新規パーサー・範囲チェックをARMだけでなくMinGWの`-Wall -Wextra -Werror`でもコンパイル・実行すること。整数境界は被比較値と上限マクロの型を確認し、同符号へ明示変換する。flash-debugのホストハーネス実行前参照へ追記する。

`slip_v2_active_test.c`で実検出器と実CSVパーサーを上記厳格設定でコンパイル・実行し、正常・範囲外・飽和・小数整数欄・非有限値・V1列拒否を確認して成功したため、この修正はresolved。

## 2026-10-25：共通PID画面での再発

main整理で3つのPID画面を共通ループへ変更した際、`int16_t`の`pattern.gain`と`i + 1U`の比較がMinGWの`-Wextra -Werror`で停止した。既存の予防手順は整数上限マクロを中心としており、UI選択番号とunsignedループ式の比較まで確認できていなかった。ARMでは通常の`-Wall`であり、この診断を検出できなかった。

項目数が最大4でint16範囲内であることを確認し、比較値を`(int16_t)(i + 1U)`へ明示変換した。選択範囲・表示順・編集範囲は変更していない。PATH行数上限も既存SLIPと同じ`(int32_t)UINT16_MAX`へ揃えた。

予防手順の対象を、整数上限だけでなくUI選択値・配列添字・`sizeof`・unsigned接尾辞を含む比較式へ拡張した。共通化後の実関数を抽出して、O0/O2・Wall/Wextra/Werrorでコンパイルする。`validation/control_refactor/run.py --baseline build/main-refactor-20261025/before`でモーター729ケース・PID画面2,592フレーム・速度設定178ケースが変更前と一致し、メタデータ35ケースと実PATH読込も成功した。対処・再実行を検証した範囲でresolved。実機UI操作は未確認。

## 2026-10-27：uint16_t差分の整数拡張

PATH白線周期地図の二分探索で`high-low > 1U`がMinGWの厳密警告で停止した。両辺がuint16_tでも減算結果はintへ拡張され、unsignedとの比較になっていた。low<=highを維持した有界探索の差分をuint32_tへ明示変換した。O0/O2・Wall/Wextra/Werrorの実モジュール検証が成功。予防の対象を変数宣言だけでなく、減算など整数拡張後の式へ広げる。結果はbuild/path-fusion-port/validation.json。対処検証済みとしてresolvedを維持する。
