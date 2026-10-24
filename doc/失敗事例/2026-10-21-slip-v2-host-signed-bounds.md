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
