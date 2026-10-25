#include "DWT.h"

float getTimeMs(uint32_t count)
{
	float ms = 1000 * (float)count / (float)SystemCoreClock;
	return ms;
}