# ネイティブ確認ツールをexec内のtoolsへ送った

- 対象: 周回ずれの追加解析後、次回撮影の機体前後マーカーについてユーザーへ確認する操作。
- 確認結果: `functions.exec`内で`tools.request_user_input_async(...)`を呼び、`TypeError: tools.request_user_input_async is not a function`となった。同じセルの先行パッチは完了し、撮影手順書は作成済みだった。誤った呼出しから質問は送信されていない。
- 原因: トップレベルの`functions.request_user_input_async`を、execの入れ子ツールと混同した。
- 対処: `functions.request_user_input_async`を直接呼び、`accepted: true`を確認した。先行パッチを再実行せず、手順書の存在と内容を確認した。実機/ファーム/設定への変更なし。
- 再発防止: ツールの宣言されている名前空間と呼出し入口を先に照合する。exec内はその`tools`に公開された入れ子ツールだけを使い、トップレベルの確認ツールやcollaborationツールを推測して入れない。セル途中の失敗時は、先行した変更の成否を読み戻してから未実行部分だけを再開する。共通事前確認へ追記した。
- 確認方法: 直接呼出しの`accepted: true`、`doc/path-independent-heading-measurement.md`の存在/内容/末尾空白確認。
- status: resolved（確認ツールの入口）。マーカー取付可否のユーザー回答は別途待ち。
