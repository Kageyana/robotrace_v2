#ifndef TIMER_H_
#define TIMER_H_

//====================================//
// インクルード
//====================================//
#include "main.h"
//====================================//
// 調整パラメータ
//====================================//
#define STRAIGHT_STATE_THRESHOLD_MM	70 // 直線状態へ移行するために必要な連続直線距離[mm]。

//====================================//
// グローバル変数の宣言、定義
//====================================//
extern float bootTime;
#if defined(ROBOTRACE_ISR_TIMING) && ROBOTRACE_ISR_TIMING
// モード番号0/予約1/2/3/4の順。廃止した番号1は記録せず、他の添字を維持する。
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
void logWriteTask(void);

#endif // TIMER_H_
