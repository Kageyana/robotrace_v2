#include "DWT.h"

float getTimeUs(uint32_t count)
{
	float us = 1000000 * (float)count / (float)SystemCoreClock;
	return us;
}

float getTimeMs(uint32_t count)
{
	float ms = 1000 * (float)count / (float)SystemCoreClock;
	return ms;
}