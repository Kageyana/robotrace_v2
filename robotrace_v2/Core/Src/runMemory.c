#include "main.h"
#include "runMemory.h"
#include "pathFollower.h"

RunMemory runMemory;
char runAnalysisLine[RUN_ANALYSIS_LINE_SIZE];
RunMemoryOwner runMemoryOwner = RUN_MEMORY_NONE;

/////////////////////////////////////////////////////////////////////
// モジュール名 runMemoryPrepare
// 処理概要     停止中に共有RAMの使用先を選び、前の走行計画を無効化する
// 引数         owner: 新しく使用する走行方式
// 戻り値       true: 切替成功、false: 走行中または不正な方式
/////////////////////////////////////////////////////////////////////
bool runMemoryPrepare(RunMemoryOwner owner)
{
	if ((patternTrace > 10U && patternTrace < 100U) || modeLOG ||
		(owner != RUN_MEMORY_DISTANCE && owner != RUN_MEMORY_PATH)) return false;

	// 配列を書き換える前に、1ms割り込みからの計画参照を止める。
	optimalTrace = BOOST_NONE;
	numPPADarry = 0;
	numPPAMarry = 0;
	pathFollowerInvalidateRoute();
	runMemoryOwner = owner;
	return true;
}
