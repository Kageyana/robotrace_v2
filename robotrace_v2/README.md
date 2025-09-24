# robotrace_v2

## コンパイルオプションについて

`printf` や `scanf` で `float` 型を扱えるようにするため、リンカオプションに以下のフラグを追加しています。

- `-u _printf_float`
- `-u _scanf_float`

STM32 向けの newlib-nano では標準状態で浮動小数点入出力が無効なため、上記のフラグを `CMakeLists.txt` の `CMAKE_EXE_LINKER_FLAGS` に追加することでサポートを有効化しています。
