# 一次ROC調査時の検索・集計・ホスト警告

- 対象タスク: 14007・14009・14012の直線減速原因調査。
- 事実1: 存在未確認のCore/Src/interrupt.cをrgへ含めos error 2。その呼出しで後続の読取りも継続した。既存の親ディレクトリ検索と終了コード検査の再発防止が実行点で機能しなかった。
- 対処1: 次の呼出しから実在Core/Src親と-g '*.c'、rg終了2以上のthrowへ変更。実呼出元はtimer.cと確認した。
- 事実2: 新規診断のdictへend_msと**describe()のend_msを重ね、TypeErrorで停止。原本とファームに変更は無かった。
- 対処2: 重複指定を削除。新規合成辞書では**展開されるキーを先に照合する。修正後は7本の診断と全CSV出力が成功した。
- 事実3: 実judgeOverSpeedの抽出ホストCを-Wextra -Werrorで初回コンパイルし、既存のint16_t対uint32_t符号比較warningで停止した。
- 対処3: 製品の式を改変せず-Wno-error=sign-compareを限定指定。既存warningをログへ保全し、他の警告はWerrorのままにした。次の抽出試験では製品とホストの警告設定差を事前照合する。
- 検証: 診断・照合のスクリプト終了0、実C経路計画3本のO0/O2完全一致、実setterの266→10と実停止判定201周期を確認。証拠はbuild/primary-roc-14012とanalysis/primary_roc_14012。
- 再発防止: ログ解析Skillの実行前参照へ、timer.cの実在親検索、辞書キー衝突、ホストの既存warningの扱い、制御/保存型の範囲検査を追記する。
- record-failures Skillは個人Skillとプラグインの一覧に無かったため指定先へ手動記録した。
- status: resolved（調査手順の失敗）。製品の速度目標型不具合は別記録でopen。
