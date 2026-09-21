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
typedef struct
{
	uint32_t maxIsrCycles;
	uint32_t maxImuReadCycles;
	uint32_t isrOverrunCount;
	uint32_t imuReadErrorCount;
	uint32_t maxLineUpdateIntervalMs;
	uint32_t lineStaleCycleCount;
	uint32_t adcPhaseMismatchCount;
	uint32_t startResetDelayMs;
	int32_t startResetPulseDelta;
	uint8_t startResetMeasured;
} RunTimingDiagnostics;
//====================================//
// プロトタイプ宣言
//====================================//
void Timer_ResetRunDiagnostics(void);
void Timer_StopRunDiagnostics(void);
RunTimingDiagnostics Timer_GetRunDiagnostics(void);
void Timer_NotifyLineUpdate(void);
void Timer_NotifyAdcPhaseMismatch(void);
void Timer_NotifyStartMarkerDetected(int32_t pulse);
void Timer_NotifyStartReferenceReset(int32_t pulse);
bool Timer_IsLineObservationFresh(void);
void Interrupt1ms(void);
void Interrupt500us(void);
void Interrupt300ns(void);
void logWriteTask(void);

#endif // TIMER_H_
