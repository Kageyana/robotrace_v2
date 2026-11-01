# ショートカット解析ハーネスでassert依存が最適化時に消えた

- 対象: 一次14038から生成するショートカットと14039の追従比較。
- 事実: 初回Zigホストコンパイルでargc未使用warningがWerrorとなり停止。assert内に入力読込も置いていたため、assert無効の構成では検査だけでなくfgets/fscanfも消える構造だった。実行前の停止で、原本・ファーム・設定への影響はない。
- 原因: 既存fixtureを流用し、2026-11-01-target-speed-u16-validation-preflight.mdにあるNDEBUG対策をログ解析の新しい入口へ適用できなかった。record-failures Skillは公開一覧と個人・repo Skill検索に見つからず、プロジェクト指定先へ手動記録した。
- 対処: 生成元analysis/script/analyze_shortcut_14038_14039.pyで、検査失敗時に常に終了するREQUIREを定義し、入力操作・検査がNDEBUGに依存しないようにした。
- 再発防止: ホスト解析を生成する前にassert内の副作用とNDEBUGを確認する。読込・検査は常時有効な条件分岐へ置き、O0/O2の両方で実入力件数と全出力を比較する。ログ解析Skillの実行前参照にも同じ確認を追加した。
- 確認: Wall/Wextra/WerrorでO0/O2とも成功。6794行の一次入力から1724点・短縮1区間・10.390625mm短縮を生成し、全経路CSVがバイト一致。実Cで再計算した一次XYとCSVの最大差0.000015259mm。異常入力のファイル欠落・列不足もO0/O2で終了2となり、常時検査の実行を確認。
- status: resolved（解析ハーネス）。実機RAM経路CRCとbin全CRC、床上の実位置は今回未検証。
