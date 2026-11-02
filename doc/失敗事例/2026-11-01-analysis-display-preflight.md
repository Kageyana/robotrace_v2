---
type: codex-failure
date: 2026-11-01
task: 走行前解析のアニメーション・経過時間追加
status: resolved
---

# UI調査のファイル所在と検証ヘッダの重複定義

確認事実: メインループを探す際、存在一覧にない`robotrace_v2/Core/Src/system.c`をGet-Contentへ渡し、ファイルなしで失敗した。実装は`control.c`の`loopSystem()`だった。さらに新設ホスト試験の模擬`ssd1306.h`が`SSD1306_I2C_ADDR`を定義したが、実フォントヘッダから実`ssd1306_conf.h`が読み込まれ、同じマクロの再定義をWerrorで検出した。ファームのDebug/Releaseビルドは成功しており、この失敗は製品のコンパイル診断ではない。

原因: ディレクトリ確認だけでファイル名の推測を防げていなかったこと、フォントの推移includeが与えるマクロを模擬ヘッダの生成前に照合しなかったこと。既存のパス確定・依存確認策を実行点で適用できなかった再発である。

対処: 実在するCore/Srcへ`rg -n 'loopSystem|Analyzing' ... -g '*.c'`を行って呼出元を確定した。模擬ヘッダからI2C設定の重複定義を除き、実`ssd1306_conf.h`を唯一の定義元とした。製品の設定と警告レベルは変えていない。

再発防止: 関数名は実在する親ディレクトリのrg結果からファイルを選び、未確認の単独ファイルをGet-Contentへ渡さない。UIハーネス生成前に`ssd1306_fonts.h`→`ssd1306_conf.h`のincludeとマクロを確認し、実設定を含める場合は模擬設定を再定義しない。書込み・デバッグSkillの検証前手順へ同じ入口と停止条件を追加する。

確認方法・結果: `validation/analysis_display/run.py`は実フォント・実描画・実DMA状態遷移でO0/O2・Wall/Wextra/Werrorのコンパイルと実行に成功した。修正したcontrol.c/setup.cの所在も読込確認済み。検索・ホスト依存の対処をresolvedとし、実機の表示・解析時間の検証とは区別する。

## 2026-11-02：ホスト入口への引数追加時の変数衝突

負例用CLIを追加した際、argparseのNamespaceを`args`へ格納したが、既存のコンパイル・実行ループも同名の`args`をコマンドリストとして使っていた。初回の現行検証はコンパイル・実行後の分岐で`AttributeError: 'list' object has no attribute 'expect_old_tearing'`となり停止した。ファーム・実機の異常ではない。

CLI結果の保持名を`options`へ分離し、現行・負例の両入口をO0/O2で完走した。CLI追加前に既存ループの代入先を検索し、Namespaceとコマンドリストへ別名を使うことを再発防止とする。入口の対処はresolved。

## 2026-11-02：滑らかさ改善時にも所在確認を省略した

対象: ローディング表示の20fps化。確認事実: 未確認の`Core/Src/i2c.c`と`slipAnalysis.c`、過去文書に記載された`validation/shortcut_speed/run.py`と`analysis/script/search_firmware_sources.ps1`を直接指定し、ファイルなしで失敗した。I2C設定は実際には`main.c`、SLIP解析は`courseAnalysis.c`にある。現行validation一覧にshortcut_speedは存在しない。検索失敗の後続読込が成功し、呼出しの終了コードだけでは検出できなかった。原本ログ・実機への変更はない。

原因: 既読の事前列挙ルールを各実行点へ適用せず、過去文書のパスを現存確認の代わりにした再発。対処: 実在する`Core/Src`全体の検索でI2C 100kHzとSLIPの所在を確定し、`rg --files validation -g '*.py'`で入口を取り直した。`record-failures` SkillはローカルSkill群とプラグイン一覧を検索しても見つからず、AGENTS.mdの記録項目に従って本節へ直接追記した。

追加の再発防止: Skillの手順へ、文書に記載された補助スクリプトも初回の`Test-Path -PathType Leaf`を必須とし、存在しなければディレクトリ検索へ戻すことを追記する。PowerShellのファイル読込には`$ErrorActionPreference='Stop'`、各rgの直後には終了コード2以上のthrowを併用し、失敗した同一呼出しの後段を実行しない。

確認: 実在する`Core/Src`・validationを用いた検索と入口取得は成功。現行表示検証O0/O2、Debug/Releaseは成功。パス確認の対処はresolved。実機の滑らかさと総解析時間は未確認で、このステータスの対象外とする。
