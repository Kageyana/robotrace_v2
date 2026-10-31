# 派生解析の関数切出しで依存を欠落

- 対象: 動画002～005・ログ13872～13875の解析。
- 確認結果: 再生コードのビルド部分を切り出した生成スクリプトでSHA計算の`sha()`を移し忘れ、実行時に未定義となった。比較スクリプトでは文字列置換でf-stringの引用符が衝突し、保存前のcompileで停止した。原本・ファーム・設定の変更はない。
- 原因: 切出し範囲外の依存関数の確認漏れ。構文検査だけでは未定義グローバルを検出できなかった。
- 対処: ASTで元の`sha()`も取得するよう生成元を修正し、実行ファイル・対象ソースのSHAをbuild_manifestへ保存した。引用符を修正し、生成物の保存前に構文と関数の直接グローバル参照を検査する。
- 再発防止: `analysis/script/preflight_generated_path_analysis.py`を生成時に呼ぶ。symtableで未知名があれば停止する。本解析の単純なトップレベル構造向けで、動的importや入れ子まで保証しない。必要な関数を伴う小さい入力の実行も省略しない。ログ解析Skillの実行前へ追加する。
- 検証: 未定義shaの拒否・修正版の受入に成功。修正後の実ソースビルドと4ログ×4補正率の16再生、比較4本の同期・測定線感度計算は終了コード0。生成元の再実行も構文/直接グローバル検査に合格。
- status: resolved（解析ツール）。13873の実機逸脱原因は未確定。

## RAM配置読出しツールの派生時の停止

新しい234項目の読出し器を作る際、for mode in range(6)という部分文字列が定義部とdecode内の2か所にあり、一意性assertで保存前に停止した。しかし後続処理を同じオーケストレーションで開始し、未生成ファイルの起動にも失敗した。型付き実機読出し・Flash書込みへは進んでいない。対処は改行直後のトップレベル行だけを置換し、生成成功を確認してprepare→self-testを順に実行すること。

ハードウェアdriverのflash入口にも、配置が対象ELFと同一、234項目、expected.binのSHA一致、decoder self-test成功という事前条件を追加した。生成物が未完成でも後段へ進まない。234項目のDWARF準備と合成読出しは終了コード0。status: resolved（生成/依存ゲート）。

## モジュールfor変数を未定義と誤検出

220117の床点棚卸し器を生成する際、事前検査がモジュールのforで束縛したseedを未定義と誤検出し、保存前に停止した。原因はASTの直下Assignだけでグローバル定義を集め、for内の束縛を含めなかったこと。symtableのモジュール記号からassigned/imported/namespaceを追加するよう検査器を修正した。未定義shaの拒否と、for変数を参照する内包表記の受入、実棚卸し/校正/追跡/比較の実行を確認した。status: resolved（検査器）。動的importや任意の入れ子を完全解析する検査ではなく、小さい実入力による起動検証を維持する。

## C補助APIの標準ヘッダ依存

pathFusionWhiteLoopPointでNULLを使った際、pathFusion.cにstddef.hがなく、単独C検証のコンパイルが停止した。他の組込みヘッダからの間接定義を仮定していた。stddef.hを直接追加し、O0/O2/Wall/Wextra/Werrorと4構成のARMビルドで成功した。再発防止は新しい標準識別子の使用元ヘッダを実Cのincludeへ明示し、HAL等を含まない単独モジュール検証を実機書込みより前に実行すること。status: resolved（ヘッダ依存）。機体への書込みはしていない。

## 過去の解析証跡JSONへ新しいキーを仮定

方位更新候補を13848/13849にも検証した際、binary_validation.jsonに後の系列で追加されたcsv_path/csv_sha256があると仮定し、監査がKeyErrorで停止した。過去形式ではCSVの原本ハッシュはvalidation.jsonのoriginalsへ保持されていた。現物のキーを読み、前者が存在すれば使い、旧形式では同じログ番号・CSV拡張子の保存済み原本SHAを後者から解決するよう修正した。両形式で60ケースの入力全件・CSV原本・単独試験・再生を照合し、終了コード0を確認。解析Skillの新規監査前へ証跡JSONの世代確認を追加する。status: resolved（監査の入力契約）。

## 2026-10-29：CSV保存先の世代差を再生入口で見落とした

新FFの15走回帰で、13848/13849もanalysis/<系列>/original/<番号>.csvにあると仮定し、O0の途中でFileNotFoundErrorとなった。原本欠落ではなく、旧系列はvalidation.jsonのoriginalsにF:/Dropbox/.../v2のCSVとSHAを保持していた。同じ世代差の予防を監査だけへ適用し、再生入口へ適用できていなかった再発である。

build/path-yaw-ff-validation/replay.pyへcsv_source()を追加し、csv_path/csv_sha256があればそれを使い、旧系列はoriginalsから番号・CSV拡張子で一意に解決する。全15CSVと5一次binの存在・原本SHAを、コンパイル/再生開始より前に照合する。部分出力はreplay/before-csv-contract-fixへ保持した。修正後の全15走×O0/O2計30再生が終了コード0で完了した。

今後の新規監査/再生はフォルダ名を推測せず、実証跡の世代を読んで共通の原本解決を入口へ置く。解析Skillにも、全入力の事前解決とこの実行点を追記する。status: resolved（入力契約と全件回帰）。

同日の完了監査追加時にも、生成済みvalidation.jsonのpath_yaw_ff_versionをyaw_feedforward_versionと推測してKeyErrorとなった。現物のキーへ合わせ、過去の融合版3のソース照合は現在ファイルではなく保存済みcandidate-sourceへ切り替えた。現在の候補版20のソース・全成果物SHAは別に照合し、監査の終了コード0、最新13894・目標未達を確認した。新規監査は生成元/実JSONの項目名と保存ソースの世代を先に確認する。status: resolved（監査契約）。


## 2026-10-29：検証フェーズの補助ファイル名を推測した再発

版21の準備で、実在を列挙せずbuild/path-yaw-ff-validation/prepare.py、validation/log_storage/path_replay.incを読もうとして失敗した。原本欠落ではなく、実際の入口はbuild.py、再生includeはcheck.cから参照する../path_fusion/replay.incだった。ファーム/実機へ影響する処理は行っていない。既存の存在確認ルールを実行点で守れなかった再発である。

同じ作業でapply_patchへ行全体でない文や未読の見出しを指定し、検証で拒否された。範囲を小さくし、直前に読んだ完全な行・一意なASCIIコード行を使って再適用した。後続へ進む前に対象を読み戻し、成功したパッチだけを確認した。

再発防止は、新しいフェーズの開始時に既存フォルダのrg --filesとcheck.cのincludeを先に読み、生成入口・依存ファイルを一意に確定すること。以後の準備器は全入力のis_file/SHAを先に確認し、旧ソースを別領域へ保全してから変更する。prepare_path_unmatched_speed.pyは全77ソースと旧ELFの対応を照合し、新しいbefore_manifest.jsonへの保存とSHA一致が終了コード0で完了した。status: resolved（今回の入力解決と保全）、再発監視を継続。


## 2026-10-30：凍結C再生へ元CSVを渡した入力契約違反

版21の新10走比較で、real_replayの第3引数を元CSVと推測し、fscanf(tgtParam.bstStraight)で停止した。実入口は15速度値、6整数設定値、dt/パルス/平均gyro/10白線/4slip/flags/markerの19値を並べる専用テキストである。出力列heading_step_cdegも実ハーネスから確認し、推測していた名前を実行前に修正した。

replay_unmatched_first.pyは元CSV/SHAを全件照合後、共通の速度定義から15項目を選び、全10入力を先に生成・列数/有限性/正時刻差を検査してから凍結exeを実行する。失敗した初回出力とソースはanalysis/path_unmatched_20261030_first/replay/before-input-contract-fixに保持。修正後はC20/C21×O0/O2×10走の40再生が終了コード0で完了し、最適化間の全トレース一致、未照合中の再加速なし、固定運動での推定列一致を確認した。status: resolved（入出力契約）。物理的効果の証明とは別である。


## 2026-10-30：13924の補助モジュール名前空間とBOM

対象は新しい走行RAM/映像解析。派生RAM読出し器へmodule.sys.argvを設定したが、そのモジュールはsysをexportせずAttributeErrorでGDB起動前に停止した。呼出し側のsys.argvへ設定して既存mainを呼び、対象ELFの配置照会・保存RAM復号が終了コード0で完了した。補助モジュールのimport/公開名を先に読み、argvは呼出し側で管理する。

静止白線比較器ではBOM付きretained_route.csvをNumPyの既定cp932で読みUnicodeDecodeErrorとなった。encoding=utf-8-sigを明示して全1181行を読み、比較を完了した。Python標準openだけでなくgenfromtxtも文字コードを明示する。確認はanalysis/script/check_marked_071234_map.pyと同系列floor_white_map_comparison.json。status: resolved（今回の解析入出力契約）、物理原因はopen。


## 2026-10-30：追従偏差比較で非出力searchをログ項目と仮定

screen_tracking_guard.pyがtgtParam.searchをメタデータから直接読みKeyErrorでC起動前に停止した。SPEED_PARAM_FIELDSは保存19項目・メタデータ15項目で、search/stop/shortCut/pathReplayは出力フラグ0だった。既存凍結再生はREPLAY_DEFAULTでPARAM_SEARCH=1.0を使っている。

既定値を過去の実SD値へ読み替えず、同じ凍結再生の既定1.0を出典SHA付きで明記した。13924後RAMは全19項目を対象ELFで型付き復号し、停止時search1.0・加減速20、記録15項目とのヘッダ精度一致を確認。これは全過去走の実SD値でも非記録項目の走行開始値の証明でもない。

同じ調査で未確認のlogRunMetadata.hをrgへ指定して失敗した。実際のメタデータ生成はSDcard.cで、存在するSDcard.h/log_schema.h等はrg --filesで確認した。次の調査は実在Core/Srcへsymbol検索し、ファイル名を推測しない。

再発防止としてscreen入口で同じ凍結speedParameters.hの全19項目・出力15・search出力0を確認し、入力値に記録/既定の区分を保持する。確認はtracking_guard_probe/screen_protocol.json、retained_speed_parameters.json、final_validation.json。status: resolved（入力と出典契約）、open（過去の非出力SD値・実走安全性）。

## 2026-10-30：CSV補助の要約と生メタデータを取り違え

- 対象: 13924の重力混入診断。probe_marked_071234_gravity_inputs.pyのprepare。
- 事実: check_path_curve_y_drift_13742_13808.loadの返り値を(meta, columns)と仮定し、meta['binaryLogNumber']でKeyError。Cの起動・実機接続・書込みより前に停止した。
- 原因: この補助は(summary, columns)を返し、生ヘッダはsummary['metadata']。別のCSV補助load_csvは生メタデータを返すが、同一契約ではない。関数名の類似から返り値を推測した。
- 対処: 実際のreturnとsummary生成箇所を読み、info['metadata']のdict型を入口で確認してからログ番号を照合するよう修正。失敗時ソースはgravity_input_probe/before-load-return-fixに保全した。
- 再発防止: 既存補助を使う派生解析は、import名だけでなく実装のreturn、型、メタデータの所在を最初に確認し、原CSV番号・56列数・SHAを入口で検証する。修正後は新たな同一失敗を起こさず、未実行段階から再開する。
- 検証: 修正後prepareは終了コード0、4入力・5578msを生成。正式SlipDetection_Updateによる8応答はO0/O2一致、5ms/30ms平均と正式横残差の0.8 LPFを含む応答を確認。preparation.json、filter_validation.json参照。保存slipV2Latは内部の共通残差ではなくmatched_candidate_v5であり、重力にも正式処理の応答を使用する。
- ステータス: resolved（返り値契約・実行再開）。open（初期姿勢・X物理符号・疎なgyro・未記録有効性・実走効果）。

## 2026-10-30：付加CSVの命名と列名を推測した

- 対象: validate_path_arm_trial_host.pyによる実呼出元の230再生。
- 事実: 最初のC再生が終了コード0で完了した後、Pythonが13898.tracking.csvを開いてFileNotFoundError。出力元replay.incは出力全名へ.tracking.csvを追加するため実在名は13898.csv.tracking.csvだった。同じ消費側に未確認のyaw_ff_dps等もあり、実際の列はff_dps/lateral_fb_dps/heading_fb_dpsだった。
- 原因: 出力元snprintfとCSVヘッダを読まず、Path.with_suffixと列名を推測した。
- 対処: 出力元の命名とヘッダを入口でassertし、Path(str(trace)+'.tracking.csv')と実列名へ修正。C出力・付加CSVの両方を凍結参照と全バイト照合し、exe・単体試験・ソースSHAをrecovery.jsonへ保全した。--resumeで最初のCを再実行せず残りを完了。
- 再発防止: 派生消費側を作る前に、実producerのsnprintf/fputsまたはmanifestの出力名・全使用列を確認する。末尾追加と拡張子置換を混同しない。解析だけの失敗で完成済みC試験を繰り返さず、元成果物のSHA・入力・実行像・参照一致を確認して未実行段階から再開する。
- 検証: path_arm_trial_20261030/host_validation.jsonの230再生、無効化46系列・有効入力46系列の全バイト一致、O0/O2一致。before-output-contract-fixの旧ソースとrecovery.jsonで回収範囲を確認できる。
- ステータス: resolved（入出力契約・再開）。open（人工的な周期欠落時の14完走ログのLOST・実機検証）。

## 2026-10-30：配置JSONの異種値を一括でタプル分解した

- 対象: RAM計測のCダンプを独立Pythonで照合する段階。
- 事実: O0のコンパイル・C単体/呼出元試験は成功した後、配置辞書のend:0を[offset,size]として分解しTypeErrorで停止した。
- 原因: 実producerには通常フィールドとsentinelが混在するのに、内包表記のifより先に全valueを分解していた。
- 対処: 名前でsentinelを除いた後に型/長さを確認して分解する。旧消費側をbefore-layout-sentinel-fixへ保全し、完成O0のexe・ソース・生ダンプ・配置SHAをcompiled.jsonで照合してCを繰り返さず復号から再開した。
- 再発防止: 辞書の実値型・特殊キーを読む。異種辞書は名前を検査してから分解し、配置境界と全使用フィールドを照合する。消費側失敗だけで完成済み実行を繰り返さない。ログ解析Skillの実行前へ反映。
- 確認: O0/O2の32768tick履歴・512frame・128eventが全バイト一致。実ARM DWARFの52フィールド配置と独立ダンプが一致し、38ヘッダ値・全フレーム・全イベント、8異常入力、NaN生ビット保持を確認。CLIは合成由来を明記し、Flash/ELF/RAM SHAとリセット除外の不一致を拒否した。
- ステータス: resolved（異種配置・再開・復号契約）。open（新計測版の実機取得は未実施）。

## 2026-10-30：repo rootのstdinから兄弟解析モジュールをimport

左右姿勢読出し器の7合成試験をrepo rootからstdin Pythonで実行し、analysis/scriptのread_path_input_axis_poseがsys.pathにないためModuleNotFoundErrorとなった。実機処理や保存済み測定は未変更。単独スクリプト実行時の兄弟importをstdinへそのまま移した仮定が原因。validate_path_input_axis_pose.pyを実在する同じscriptディレクトリへ保存し、絶対Pythonとrepo root workdirでファイルとして実行して7ケースを確認。再発防止はimport名だけでなく起動方法とsys.pathを確認し、兄弟依存を使う試験はスクリプト入口を明示する。実機取得を繰り返さず、未実行のPC試験だけ再開した。ログ解析Skillの入口へ追加。status: resolved（試験の起動契約）。実姿勢2回は条件外で不採用のまま。

## 2026-10-30：基準19項目入力を慣性27項目消費側へ渡した

未採用候補の新13926再生で、基準C21の19項目入力を流用した。実replay.incは追加残差8個を要求し、fscanfが次行を消費してdt条件で停止した。原本や機体は未変更。消費側fscanf/各ループの幅を読まず、呼出引数が同じことを入力互換と仮定したことが原因。27項目を既存8残差式から生成し、header15/設定6/各行27/総3042tickを入口でassertした。完成O0コンパイル像と不完了出力を保全し、run_observed_220124_validity.pyから入力修正後の6再生を完了。Cアルゴリズムと既存実行像は変更せず、O0を再コンパイルしていない。O0/O2一致、無効化は基準C21と全バイト一致。再発防止はfscanfのヘッダ/行/追加列を別々に検証すること。status: resolved（再生入力契約）。

## 2026-10-30：差替え外の浮動小数点テキストまで9桁で再出力

停止前509tickだけの実測差替えを検査した際、候補の82msでheading_fb_dpsが1e-6変わり、差替え前の一致ゲートが停止した。全行をPython float64へ変換して.9g出力したため、元の高精度gyro文字列のfloat32への丸めが一部変わった。9桁のfloat32往復性を、未丸めfloat64の再出力へ適用した仮定が誤り。差替え外の語は原文を保持し、dt/パルス/マーカーと実測tailの3値だけを書き換えた。旧ソース/入力/完了C出力はactual_tail_probeへ保全し、変更した入力はactual_tail_precision_probeへ新出力で検証。C本体の変更や許容差の拡大はしない。4再生で最初2533tickの全制御/診断出力が旧入力と全バイト一致、O0/O2一致を確認した。再発防止は変更外のテキスト保持と差替え前の不変性ゲート。status: resolved（入力精度契約）。

## 2026-10-30：派生生成で全import行を同じ綴りと仮定

版22＋入力観測器の派生復号器で、全補助スクリプトが`from prepare_path_input_capture import OUT`を使うと仮定した。実ELFの52配置と8異常復号試験が成功した後、CLI側の実importは`ROOT,OUT`だったため、一意置換ゲートで停止した。機体・製品コード・完成ビルドは未変更。既存の依存入口検査を全対象の生成前へ適用していなかった再発。

修正前の入口をbefore-import-contract-fix.pyへ保存し、全3対象をASTで先に調べ、実ImportFromにOUTが含まれることを確認してからモジュール名だけを置換する。完了済み配置/復号のELF・配置SHA・生成ソース・成功ログを照合し、未実行CLI5件だけを再開した。decoder_pipeline.jsonは再利用順[true,true,false]を明示する。

再発防止は全対象のimport/公開名を最初の生成前に検査すること、生成物と完了証跡を照合した段階別再開を設けること。確認コマンドは絶対Python、repo root workdir、PYTHONPATH=analysis/scriptでanalysis/script/validate_path_trial_capture_decoder.py。修復実行は52配置・8異常復号を再実行せず、CLI5件成功。status: resolved（生成契約と再開）。新融合の実機処理時間・走行効果はopen。

## 2026-10-31：型情報ビルドの失敗後に配置準備を呼んだ

型情報付きReleaseのGDB preflightが失敗した同じorchestration呼出しで、配置準備まで進み、validation.json不在で停止した。実機への書込みや後段のフォルダ生成はなかった。await/textで前段結果を表示することを依存ゲートと誤認した再発。run_path_release_types_pipeline.pyを入口にし、各段のreturncodeと完了証跡/SHAを確認してから次を呼ぶ。前段失敗を注入した2段の模擬呼出しで、後段が実行されないことを確認した。status: resolved（依存起動）、実機効果はopen。詳細は2026-10-31-release-dwarf-effective-flags.md。

## 2026-10-31：未知の準備名と旧走イベント時刻を流用した

- 対象: 13927/13928の追加解析。推測したprepare_marked_071234.pyをGet-Contentして存在せず、後続読取りは実行された。原本・機体への変更はなかった。既存の実在名検索ルールを読取りの実行点で適用していなかった再発。rg --filesで実在のprepare_observed_220124.pyを取得し、全置換検査後に新しいprepare_curve22_070745.pyを生成した。以降の依存PowerShellはErrorActionPreference=Stopを指定した。
- もう一つの事実: 有効性全14257周期の読取りに成功した後、旧13926のイベント[1,6,43]を期待してassertが停止した。実13928は[1,3,43]で、3msの変化はflagsの電流有効bit128。イベント生成は理由だけでなく全flags変化も対象で、フィルタ充填や通信エラーの増加ではなかった。旧走の時刻を新走へコピーした仮定が原因。
- 対処/再発防止: 実events.csvとpathInputCapture.h/cを読み、各イベントの下位4bitと全周期履歴の一致、時刻順序、実理由遷移を検査する入口へ変更した。旧走の一覧へ合わせて実データを変更しない。未知名は実在一覧、producerの複合イベント条件、今回のheaderと値の型を先に確認する。ログ解析Skillの実行前へ追加した。
- 確認: analyze_curve22_070745_inputs.pyは全14257周期、初期42周期以降の内部無効0、末尾512周期とBIN203件のgyro/横残差/補正パルス全ビット一致を確認した。フルFlash/実ELF/RAM SHAも一致し、実機再取得・再書込みは行わなかった。
- status: resolved（入口/イベント比較契約）。open（独立DRDY鮮度・物理誤差原因）。
