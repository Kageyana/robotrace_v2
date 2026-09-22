# DebugMarkerログスキーマ固定長assertのプロファイル不一致

- 対象タスク: DebugMarkerログスキーマの互換性修正
- 発生日: 2026-09-22
- ステータス: resolved

## 事実

- 通常ログのVersion 3レコードは42バイトである。
- 詳細デバッグプロファイルは追加フィールドを含むため95バイトである。
- DebugMarkerビルドでは、通常プロファイル用の固定長判定を詳細プロファイルへ適用してしまい、ログレコードサイズの判定がプロファイルと一致しなかった。

## 原因または未確定事項

- `log_schema.h` の固定長assertが通常プロファイルの42バイトだけを前提としていたため、詳細デバッグプロファイルの95バイトを正しく検証できなかった。

## 対処

- 通常プロファイル用42バイトと詳細デバッグプロファイル用95バイトの定数を分離する。
- `LOG_SCHEMA_PROFILE_LIGHT` に応じて対応する `_Static_assert` を選択する。
- Debug、Release、DebugMarkerの全プリセットを再ビルドする。

## 再発防止策

- ログスキーマのレコードサイズを変更した場合は、通常・詳細の全ビルドプロファイルをビルドし、各プロファイルの固定長assertを通す。
- Version 3の解析手順には、通常42バイトと詳細95バイトの両方を明記する。

## 確認方法

- `cmake --preset Debug` / `cmake --build --preset Debug`
- `cmake --preset Release` / `cmake --build --preset Release`
- `cmake --preset DebugMarker` / `cmake --build --preset DebugMarker`
- `path_log_recovery.py` の構文検査とVersion 2/3受付確認
- `git diff --check`

## 結果

- `log_schema.h` に通常プロファイル42バイト、詳細デバッグプロファイル95バイトの定数とプロファイル別 `_Static_assert` を追加した。
- Debug、Release、DebugMarkerの構成・ビルドがすべて成功した。
- Version 3の温度列、旧モーター電圧指令列の省略、先頭列順、Version 2/3のPATH復元受付を静的に確認した。
- `path_log_recovery.py` の構文検査が成功した。
- `git diff --check` が成功した。
