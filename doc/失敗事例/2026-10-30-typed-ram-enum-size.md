# 保存RAMのenumを32bitと仮定した

- 対象タスク: 13924の停止状態と保持経路の型付き復号。
- 確認結果: runMemoryOwner/followerStateを4byteと仮定したassertが停止。対象ELFのDWARFでは両方1byteだった。実機書込み・リセットは行っていない。
- 原因: 固定幅整数の配置確認をenumへ適用せず、ホストのenum幅を仮定した。
- 対処: before-enum-layout-fixへ初版を保持し、ELF照会のsizeofに応じ1/2/4byteの符号なし型で復号。PATH=2/LOST=4と1181経路点を正常読出し。
- 再発防止: 全復号対象についてアドレスとsizeofを対象ELFから取得し、enumも幅を検証してから保存RAMへアクセスする。列挙値の範囲も別に検証する。
- 確認方法: analysis/script/read_marked_071234_state.py、同系列path-state-layout.raw.log/retained_path_state.json/retained_route.csv。
- ステータス: resolved（解析器の型契約）。自己位置喪失の物理原因はopen。
