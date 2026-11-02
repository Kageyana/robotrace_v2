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
