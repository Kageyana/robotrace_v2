# PATH共通化検証での検索入口と白線退避後の回復条件

## 2026-11-01：SHORTCUT_CURVE追加時の検索先の再発

- 事実: 新モードの関連検証を探す際、既知の欠落パスrobotrace_v2/testsを再びrgへ渡しos error 2となった。製品編集・ビルド前の読取段階で、原本とファームへの影響はない。
- 原因: 過去参照のテスト配置を現チェックアウトへ当てはめ、実在検査を呼出し点で省略した。既存の「参照読込後、実在親だけを検索する」対策を適用できなかった再発。
- 対処: 現存validationの一覧から依存する生成器を選び、新検証をvalidation/shortcut_curveへ集約した。新モードの検証入口をflash-debug Skillの実行前手順・再発防止参照へ追記した。
- 再発防止: このチェックアウトの新モード検証はvalidation/shortcut_curve/run.py、entries.py、build.py、check_timing.pyを使う。存在しないtests名を構成せず、別の入口が必要な場合は現在のvalidation一覧から選ぶ。
- 確認: 実在validation経由の実C O0/O2、画面・自動設定・保存/CRC、Debug/Release、診断ONのtimer.c構文検査が全件成功。
- ステータス: resolved（検証入口）。新モードの実走は未確認。

- status: resolved（検索と試験契約）。実走行性能は未検証。
- 対象: PATH／SHORTCUTをPATH_CURVEの経路生成・補正・FF・未照合処理へ統一する作業。
- 事実: 共通事前確認の読取りと同じ呼出し内で、未確認のvalidationとrobotrace_v2/testsをrgへ渡してos error 2、Core/Src/path*等をパスに渡してos error 123が発生した。後続のgit status成功により呼出し全体は終了0だった。製品への変更前の検索失敗で、解析値・実機への影響はない。
- 原因: 既存の存在確認・glob分離策をコマンド生成時に適用できなかった。現セッションの実在チェックアウトはC:/Users/ucawa/Dropbox/Document/robotrace/robotrace_v2で、旧D:のパスやローカル専用検証ファイルは存在しない。record-failures Skillは公開一覧・個人Skill・プラグイン配下に見つからず、本記録を手動作成した。
- 対処: 実在Core/Src・Core/Incを確認し、rgの-g 'path*.c'で検索して実ファイルを読んだ。検証フォルダは今回新規に作成した。PythonはStoreエイリアスを実行せず、load_workspace_dependenciesが返した実行体を使用した。PC用Cコンパイラが無いため、検証フォルダ内にziglang 0.15.2を展開した。
- 追加の確認事実: 初回実C試験は100ms連続照合直後のtargetSpeedMps>1という期待で停止した。信頼度の回復100msとは別に、通常区間では白線退避からの再合流200ms・操舵混合100msも必要だった。
- 試験修正: 製品の回復条件は緩めず、100msでCAUTION解除、その後の再合流・混合完了で通常加速することを順に検証する。短縮保護域では白線退避を禁止し、未照合停止を検証する。
- ビルド集計の初回はPowerShell出力をUTF-16と仮定してBOMエラーで停止した。変更前ビルドは完了済みで、変更後ビルドは開始されていない。先頭の実バイト（5B 31 2F 33 38）を確認し、BOM判別とUTF-8厳密復号へ修正した。保存済みの変更前ビルドをやり直さず、未実行の変更後ビルドから再開する。外部ログは保存したシェルの版から文字コードを決めず実バイトを先に検査する。
- 再発防止: 共通事前確認を読み終えてから検索を組み立て、実在親ディレクトリ＋-gへ限定する。nativeコマンドは個別の終了コードとstderrを確認する。状態遷移試験は呼出元と全速度上限を先に読み、信頼度回復と操舵状態回復を別条件として確認する。flash-debugの再発防止参照にも追加する。
- 確認方法: validation/path_common/run.pyで実pathFollower.c／pathFusion.c／pathObservation.cをO0/O2・Wall/Wextra/Werrorで実行し、共通生成／欠測／キャッシュ、3方式一致、PATH_CURVE旧版トレース一致、未照合・回復・停止、短縮回廊生成・補正抑止・退避禁止を確認した。保存先はbuild/path-common-20261031/host-validation.json。HAL・SD入口は模擬であり、実binの全CRC読込や物理的な完走の検証ではない。
