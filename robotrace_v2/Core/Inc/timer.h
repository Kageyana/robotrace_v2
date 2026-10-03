#ifndef TIMER_H_
#define TIMER_H_

//====================================//
// インクルード
//====================================//
#include "main.h"
//====================================//
// シンボル定義
//====================================//

//====================================//
// グローバル変数の宣言
//====================================//
extern float bootTime;
#if defined(ROBOTRACE_ISR_TIMING) && ROBOTRACE_ISR_TIMING
// BOOST_NONE/MARKER/DISTANCE/SHORTCUT/PATH_REPLAYの順。起動からの累積値。
#define ISR_TIMING_MODE_COUNT 5U
typedef struct
{
	uint64_t totalCycles;
	uint32_t samples;
	uint32_t minCycles;
	uint32_t maxCycles;
	uint32_t over1ms;
} IsrTimingStats;
// 走行終了後、リセットせず静止状態でデバッガから読み出す。
extern volatile IsrTimingStats isrTimingStats[ISR_TIMING_MODE_COUNT];
#endif
//====================================//
// プロトタイプ宣言
//====================================//
void Interrupt1ms(void);
void Interrupt500us(void);
void Interrupt300ns(void);
void logWriteTask(void);

#endif // TIMER_H_
