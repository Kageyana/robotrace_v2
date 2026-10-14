# Skill実行前の共通確認

ロボトレース用Skillの最初のコマンドを組み立てる前に読む。対象分野の事例も検索し、本文の最新追記まで確認する。過去の成功は現在の接続・権限・設定を保証しない。

## 1. パスを確定してから実行する

- リポジトリルートは `D:\robotrace\robotrace_v2`、ファームウェアルートは `D:\robotrace\robotrace_v2\robotrace_v2`。ビルドの `workdir` は後者、文書・fixture生成は前者を明示する。文書更新とビルドは別の呼出しにする。
- 初回の検索は実在ディレクトリを `Test-Path -LiteralPath ... -PathType Container` で確認し、ディレクトリ＋`-g`へ限定する。未確認のファイル名やディレクトリ（`.agents`等）を検索パスへ指定しない。
- 外部コマンドのパス引数へ `*`、`?`、Bashの `{a,b}` を渡さない。ファイルを絞るのは `rg --files` または以下の検索結果に実在パスが出てからとする。

```powershell
rg -n 'Interrupt1ms|wheelClick' 'D:\robotrace\robotrace_v2\robotrace_v2\Core\Src' -g '*.c'
powershell.exe -NoProfile -ExecutionPolicy Bypass -File 'D:\robotrace\robotrace_v2\analysis\script\search_failure_notes.ps1' -Pattern '権限|パス|ログ' -FileGlob '*.md'
```

- `search_failure_notes.ps1` は日本語を含むWindows PowerShell 5.1用なのでUTF-8 BOM付き保存を維持する。変更時は実際の `powershell.exe -File` で日本語＋globを確認する。
- ログの標準パスは `F:\Dropbox\Document\robotrace\Log\v2`。無ければ現在のDropboxルートを確認し、旧ユーザー名の `C:\Users\ucawa\...` を推測で使わない。
- CrossDevice動画は存在・`ffprobe`成功を確認してから抽出する。生成失敗後に古い画像や一時成果物を開かない。

## 2. 権限を初回から選ぶ

- 現在のセッションのpermission profileを確認する。作業ツリーのwrite許可は、`.git`、`.agents`、個人Skill、リムーバブルSDへのwrite許可を意味しない。
- 対象がreadのみと分かっている場合、承認済み作業は**最初の書込みから** `exec_command(sandbox_permissions="require_escalated")` へ送る。`git add/commit/restore/switch/branch/merge/rm --cached` 等のindex・refs更新を通常権限で試さない。対象パスと操作を限定する。昇格はCodexのサンドボックス権限要求であり、OS管理者実行やACL変更とは別。
- workspace内の`git status/diff/log`の読取は通常権限を使う。管理worktreeがworkspace外に作られた場合は、Gitのread操作もサンドボックスの対象パス制限を受ける。作成先をpermission profileと照合し、対象外なら初回のGit読取から必要権限で実行する。Git更新前は既存差分・対象・ロックの有無を確認し、ステージ成功後に `git diff --cached --check` と対象差分を確認してからコミットする。失敗後に後続commitへ進まない。
- ロック作成のPermission deniedを、残留ロックと決めつけて削除しない。Git内部ファイル、ACL、グローバル設定を変更して権限制限を迂回しない。
- 権限要求が拒否されたら同じ変更を別ツールから強行しない。安全に進められる作業を完了し、拒否された操作と理由をユーザーへ報告する。
- `helper_unknown_error: setup refresh had errors` はソースの診断と分ける。同じ失敗をパス表記やシェルだけ変えて反復せず、必要最小限の同一処理を許可された環境で実行する。編集の代替には一時コピーのunified diffを使い、`git apply --check`、適用後diffを確認する。
- 自分で開始した長時間処理は返されたsession_id/PIDを保持して対象だけ停止する。CIM照会の権限を前提にせず、全Python・MATLABプロセスを名前で終了しない。

## 3. ランタイム・引用符・文字コード

- Pythonは `Get-Command python.exe` で実体を確認する。Storeエイリアスなら `load_workspace_dependencies` で取得した絶対パスを使う。必要モジュールを `importlib.util.find_spec` で先に検査し、その後importを確認する。同梱PythonにもMatplotlib/SciPy/PyYAMLがあるとは仮定しない。
- この環境ではGNUツールとGCC生成host exeにDLL再配置エラーが繰り返されている。該当するARM/hostビルド・nm/objdump/size・planner exeは、最初から許可された制限外環境で実行する。環境が変わった場合は版表示等で再確認する。ソース変更やテスト閾値緩和で起動エラーを回避しない。
- 日本語テキストの読書きは `encoding='utf-8'`、BOM混在入力は `utf-8-sig` を明示する。Python標準出力も `PYTHONIOENCODING=utf-8` とする。
- Matplotlibはimport前に `MPLCONFIGDIR` を今回の書込可能な解析出力内へ設定する。
- 複数行Pythonやコード置換を、引用符の複雑な `python -c` に押し込まない。スクリプトか範囲の小さいパッチを使う。`$skill-name` をPowerShellで保存するときは単一引用符または単一引用符here-stringでリテラルを保持し、生成後に読み戻す。
- PowerShellの `foreach` 文は結果を配列に格納してからパイプへ渡す。
- 文字化けコメントをパッチアンカーにしない。ASCIIコード行と関数範囲・出現数を確認して編集し、dot-all＋貪欲な `.*` で関数間を横断しない。

## 4. 依存操作の完了を確認する

- `exec_command` の戻り値全体を保持する。session_idが返ったら `write_stdin` で終了コードと最終出力を取得する。待機を呼んだだけでは完了にならない。
- generator→ビルド、書込み→GDB、一時MLAPP生成→起動検証は逐次実行し、前段の成功と新しい成果物を確認してから進む。失敗時はそこで止め、部分成功はdiffで確認する。
- Git履歴変更は実際のHEADと依頼対象を照合する。「今後日本語で」は既存コミットamendの依頼に変換しない。
- 失敗記録の古い36 B上限、電圧7.3/7.6 V、旧ゴール回数、過去のベンチAPIを現行ルールへ戻さない。`AGENTS.md`、現行ソース、ユーザーの最新指定を優先する。未確認の物理原因はopenのままとし、Skill更新だけで解決済みにしない。

複数exec間で共有する一時成果物はworkspace内の明示した作業領域へ保存する。sandboxのTEMPが呼出し間で同じとは仮定せず、次段で存在とハッシュを確認する。

## Git連携と変更範囲

- pushが依頼されたときはstatus/branch/remoteと送信差分を確認する。既知のsandbox HTTPS制限下では初回から必要なネットワーク権限で実行する。port 443接続失敗と認証拒否を分ける。workflow更新を含む場合は使用認証の権限を確認し、without workflow scope拒否はサンドボックス昇格で直ると仮定しない。認可変更や秘密の提示を自動実行せず、必要な認証の更新を案内する。
- 制御API・ログ・設定を統合するときはmotor/PID/FF/保存形式の境界を照合する。競合で片側を丸ごと採用せず、旧PWM出力APIの残存、ログ既存列の意味・順序、両構成ビルドと初回実ログを確認する。
- 文書・候補調査の依頼を無関係な命名正規化や制御実装へ広げない。ユーザーが別タスクと指定した変更は現在の作業へ含めない。

- 特定コミットだけをmainへ取り込む場合は、対象コミットの親とmainの差を先に確認する。Skill競合で周辺節に未導入機能の説明が含まれても、その節全体を持ち込まない。指定コミットの変更意図だけを反映し、ファームウェアソースの差分がないことを確認する。事例対応表にmain未収録の原本がある場合は、調査元のコミットを明示して参照を保つ。
