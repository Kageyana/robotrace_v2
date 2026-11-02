# 解析ロード画面が未更新のinitLCDで常に無効になる

- 対象タスク: UIで解析を開始しても、ロードアニメーションへ画面が変わらない問題の修正。
- ステータス: resolved（無表示の修正。原因再現・ホスト試験・両構成ビルドと、ユーザーによる実機表示確認済み）。
- 確認した事実: `control.c`の`initLCD`はfalseで初期化され、Core内に以降の代入がない。拡張UI検出時には`modeDSP=true`として`ssd1306_Init()`を呼ぶが、`initLCD`は更新しない。`analysisDisplayBegin()`は`modeDSP && initLCD`を要求していたため、通常UIが表示される状態でも即時returnし、描画・単発DMAを一度も開始しなかった。
- 原因: ロード画面の有効条件に、実起動経路が更新しないフラグを追加したこと。描画・DMA単体の検証記録だけでは、実起動時のフラグで機能が有効になることを保証できていなかった。
- 対処: 解析画面の表示可否は通常UIと同じ`modeDSP`を用いる。停止中`patternTrace==0`・ログ非記録中`!modeLOG`の条件は維持する。初期化APIの成功を確認せずに`initLCD=true`へ書き換えることはしない。
- 再発防止: `validation/analysis_display/run.py`で、実起動と同じ`modeDSP=true/initLCD=false/patternTrace=0/modeLOG=false`から開始する。描画関数への到達だけでなく、実フォントと実SSD1306描画コードから作られたDMA転送内容に文字と円環が入り、200ms後の円環・1秒後の時間表示が変化することを検査する。無表示・走行中・ログ記録中の抑止、終了時復帰、転送開始から完了までの実DMA参照範囲の不変性も確認する。
- 確認方法: リポジトリルートの明示workdir・既知のホスト実行権限で、同梱Pythonから`validation/analysis_display/run.py`を実行する。変更前の`analysisDisplay.c`は`build/analysis-display-activation/before-analysisDisplay.c`に保存済み。旧版O0/O2は`INACTIVE_WITH_REAL_BOOT_DEFAULTS`、修正版O0/O2は全試験PASS（Wall/Wextra/Werror）。結果とソースSHAは同フォルダの`validation.json`。
- ビルド: Debug/Releaseとも成功、今回再コンパイルしたanalysisDisplay.cにwarningなし。RAMはDebug 89,256 B、Release 89,328 Bで前回から増加なし。1ms割込み・走行制御・SD保存形式の差分なし。
- 実機確認: 2026-11-02、修正報告後にユーザーから「表示されました」と回答があり、無表示の解消を確認した。Codexによる書込みは行っていない。
- 確認範囲: 実機の画面更新間隔・時間表示の精度・終了時の復帰・総解析時間への影響を個別に測定したものではない。短時間で完了するキャッシュ解析ではアニメーションが目視できない場合がある。
