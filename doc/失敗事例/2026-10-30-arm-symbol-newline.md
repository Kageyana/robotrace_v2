# ARMシンボル出力のCRLFと部分完了ビルドの再開

- 対象: build_path_arm_trial.py、基準版と試験版の10構成比較。
- 事実: 最初のARM構成のconfigure/buildは成功しELFを生成した。nmの_Min_Stack_Sizeを検索する正規表現がNoneとなり集計だけが停止した。生出力は00002000 A _Min_Stack_Size\r\nだった。
- 原因: bytesからUTF-8へdecodeした後はCRLFが残るが、行末$をLFだけの前提で使用した。read_textのuniversal newlineとbytes.decodeの契約を混同した。
- 対処: 生ログは元バイトを維持し、解析用文字列だけCRLF→LFへ正規化。既存size/nmログはencoding指定のread_textで読み直す。ELF・CMakeCache・compile_commands・全体ビルドログをarm-recovery.jsonへSHA付き保全し、最初のビルドをやり直さずメモリ集計から再開した。
- 再発防止: nm等の外部CLIを行正規表現で読む前に、実バイト・列数・シンボル種別と改行を確認する。splitlinesまたは明示的な解析用改行正規化を使い、生ログの証跡は維持する。完了済みリンクを集計失敗だけで再実行しない。
- 検証: arm_validation.jsonで基準/候補各5構成=10ビルド成功、新規診断0、_Min_Stack_Size=8192、白線推定器68→108B、各ELFをSHA付き不変パスへ保存した。before-nm-newline-fixには失敗時スクリプトと元nm/sizeログを保持。
- ステータス: resolved（CLI解析と部分再開）。open（実機処理時間・動的スタック・入力復帰時のロバスト性）。
