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
#define ISR_TIMING_IDLE_LOAD_MAX_US 600U // 計測ON版の静止試験専用CPU負荷の上限[us/1ms]。要求値0なら無効。

//====================================//
// グローバル変数の宣言、定義
//====================================//
extern float bootTime;
#if defined(ROBOTRACE_ISR_TIMING) && ROBOTRACE_ISR_TIMING
// モード番号0/予約1/2/3/4/5の順。廃止した番号1は記録せず、他の添字を維持する。
#define ISR_TIMING_MODE_COUNT 6U
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
// TIM6のUSER CODE入口からHAL処理後の読取まで。集計の末尾と例外入出は含めない。
extern volatile IsrTimingStats isrIrqTimingStats[ISR_TIMING_MODE_COUNT];
extern volatile uint32_t isrIrqPendingTicks[ISR_TIMING_MODE_COUNT];
// 停止・開始要求なしの計測専用。走行モード別の統計へ混ぜず、既定値0で無効。
extern volatile uint16_t isrTimingIdleLoadUs;
extern volatile IsrTimingStats isrIdleIrqTimingStats;
extern volatile uint32_t isrIdleIrqPendingTicks;
#endif
//====================================//
// プロトタイプ宣言
//====================================//
void Interrupt1ms(void);
void Interrupt500us(void);
void logWriteTask(void);
#if defined(ROBOTRACE_ISR_TIMING) && ROBOTRACE_ISR_TIMING
void timerTimingResetClock(void);
void timerTimingIrqBegin(void);
void timerTimingIrqEnd(void);
#endif

#endif // TIMER_H_
