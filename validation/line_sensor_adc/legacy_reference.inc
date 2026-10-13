// Regression reference from c25677771e5b546f5755e78845c036b48d0f1ff8 Core/Src/lineSensor.c
// Only function/array names are renamed; averaging and normalization are unchanged.
static uint16_t legacyRaw[10], legacyNorm[10];
void legacyNormalize(void)
{
	const uint32_t baseVal = (uint32_t)BASEVAL;

	for (uint8_t i = 0; i < NUM_SENSORS; i++)
	{
		uint16_t current = (legacyRaw[i] > 0) ? (uint16_t)legacyRaw[i] : 0U;

		if (modeCalLinesensors)
		{
			if (current > lSensorMax[i])
			{
				lSensorMax[i] = current; // 最大値更新
			}
			if (current < lSensorMin[i])
			{
				lSensorMin[i] = current; // 最小値更新
			}
		}

		uint16_t minVal = lSensorMin[i];
		uint16_t maxVal = lSensorMax[i];

		if (maxVal > minVal)
		{
			uint32_t span = (uint32_t)maxVal - (uint32_t)minVal;
			uint32_t offset = (current > minVal) ? (uint32_t)(current - minVal) : 0U;

			if (offset >= span)
			{
				legacyNorm[i] = (uint16_t)baseVal;
			}
			else
			{
				legacyNorm[i] = (uint16_t)((offset * baseVal + (span / 2U)) / span);
			}
		}
		else
		{
			legacyNorm[i] = current;
		}
	}
}
void legacyGetLineSensor(void)
{
	// LED点灯／消灯それぞれの平均を取るためのワークバッファ
	static uint32_t accum[2][NUM_SENSORS] = {{0}};
	static uint32_t average[2][NUM_SENSORS] = {{0}};
	static uint16_t sampleCount[2] = {0};
	static uint8_t readyMask = 0;
	const uint8_t phase = lineSensorState ? 1U : 0U; // 1: LED on, 0: LED off
	const uint8_t div = LS_AVERAGE_SAMPLES / 2;
	uint32_t *acc = accum[phase];
	const uint16_t *sample = (phase == 1U) ? analogValLSon : analogValLSoff;

	// 取得したADC値を位相ごとに積算
	for (uint8_t i = 0; i < NUM_SENSORS; i++)
	{
		acc[i] += sample[i];
	}

	// 所定回数サンプリングしたら平均値に反映
	if (++sampleCount[phase] >= LS_AVERAGE_SAMPLES)
	{
		for (uint8_t i = 0; i < NUM_SENSORS; i++)
		{
			average[phase][i] = acc[i] >> div;
			acc[i] = 0;
		}
		sampleCount[phase] = 0;
		readyMask |= (1U << phase);
	}

	// LED点灯／消灯の両方が揃ったら差分を計算
	if (readyMask == 0x03U)
	{
		for (uint8_t i = 0; i < NUM_SENSORS; i++)
		{
			int32_t diff = (int32_t)average[0][i] - (int32_t)average[1][i];
			legacyRaw[i] = (diff > 0) ? (int16_t)diff : 0;
		}
		readyMask = 0;
		legacyNormalize(); // 最新のライン値でキャリブレーション／正規化を更新
	}
}
