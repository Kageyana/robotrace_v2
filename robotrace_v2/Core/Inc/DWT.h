#ifndef INC_DWT_H_
#define INC_DWT_H_

#define __FPU_PRESENT 1U // エラーが出たので

#include "stm32f446xx.h" // IRQn_Typeが見つからなかったので
#include <core_cm4.h>
#include "main.h"

#define initCycleCounter() \
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

#define resetCycleCounter() \
	DWT->CYCCNT = 0;

#define enableCycleCounter() \
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

#define disableCycleCounter() \
	DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;

#define getCycleCounter() \
	DWT->CYCCNT;

float getTimeUs(uint32_t count);
float getTimeMs(uint32_t count);

#endif /* INC_DWT_H_ */