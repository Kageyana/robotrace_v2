# CMakeプリセット実行ディレクトリの誤り

- 対象タスク: SGmarkerを6周設定へ変更
- 発生日: 2026-09-22
- ステータス: resolved

## 事実

- リポジトリ直下 `D:\robotrace\robotrace_v2` で `cmake --build --preset Debug` を実行した。
- リポジトリ直下には `CMakePresets.json` がなく、CMakeがプリセットを読み込めず終了した。
- CMakeプリセットは入れ子の `D:\robotrace\robotrace_v2\robotrace_v2` にある。
- 解析スクリプト検査を同じ入れ子ディレクトリから実行すると、`analysis/script/...` が見つからない。

## 原因

- AGENTS.mdの構成上のリポジトリルートと、CMakeプロジェクトのルートが異なるため、作業ディレクトリを取り違えた。

## 対処と再発防止策

- CMake実行時は `robotrace_v2/CMakePresets.json` が存在する入れ子のプロジェクトディレクトリを作業ディレクトリに指定する。
- 解析スクリプト検査時はリポジトリ直下を作業ディレクトリに指定する。
- ビルド前に `Test-Path CMakePresets.json`、解析検査前に `Test-Path analysis/script` で実行ディレクトリを確認する。

## 結果

- `robotrace_v2/` を作業ディレクトリに指定してDebugビルドを再実行し、成功した。
- `GOAL_LAP_COUNT=6U` と `COUNT_GOAL=7U` の設定を確認した。
- リポジトリ直下で解析スクリプトの構文検査と新旧CSVヘッダ解析テストを実行し、成功した。
- `git diff --check` が成功した。
