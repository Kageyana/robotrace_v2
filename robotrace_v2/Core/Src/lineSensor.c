//====================================//
// インクルード
//====================================//
#include "lineSensor.h"
#include "IMU.h"
#include "fatfs.h"
#include <stdbool.h>
#include <stdint.h>
//====================================//
// グローバル変数の宣言、定義
//====================================//

// ラインセンサ関連
uint32_t lSensorInt[NUM_SENSORS] = {0};	 // ラインセンサの立ち上がりエッジAD値積算用
uint16_t lSensor[NUM_SENSORS] = {0};	 // ラインセンサの平均AD値
uint16_t lSensorCari[NUM_SENSORS] = {0}; // 正規化したラインセンサのAD値
volatile bool lineSensorState = false; // 現在のPWM位相。取得済みデータの位相には使わない。
volatile bool lineSensorPower = false;
volatile bool adcAcquisitionBusy = false; // DMA開始前から結果処理完了まで占有する。
volatile bool adcAcquisitionFault = false; // HAL異常時は再起動まで再取得を禁止する。
volatile uint32_t adcSkippedCount = 0;
volatile uint32_t adcStartErrorCount = 0;
volatile uint32_t adcLastStartStatus = HAL_OK;
volatile uint32_t adcLastErrorCode = 0;
volatile uint32_t adcTransferErrorCount = 0;
volatile uint32_t lineSensorUpdateCount = 0;
volatile uint32_t lineSensorUpdateAgeMs = 0;
volatile uint32_t lineSensorMaxUpdateAgeMs = 0;
volatile bool lineSensorUpdateStale = false;
volatile uint32_t lineSensorStaleCount = 0;
volatile uint16_t lineSensorVrefSample = 0;
static volatile uint32_t acquisitionEpoch = 0;
static volatile uint32_t sensorPowerEpoch = 0;
static volatile bool sampleAvailable = false;
static volatile bool controlFrameActive = false;
static uint16_t pendingLineRaw[NUM_SENSORS], pendingLineNormalized[NUM_SENSORS];
static uint32_t pendingLineUpdates;
static bool conversionPending = false;
static bool requestedPhase = false;
static uint32_t requestedTick = 0;
static struct {
	bool phase;
	uint16_t *buffer;
	uint8_t markerLeft, markerRight;
	uint32_t epoch;
} adcSample;
static uint32_t accum[2][NUM_SENSORS];
static uint16_t pairSample[2][NUM_SENSORS];
static uint8_t completedPairCount;
static uint32_t oldestPairTimeMs;
static uint8_t readyMask;
#if defined(ROBOTRACE_ISR_TIMING) && ROBOTRACE_ISR_TIMING
volatile LineSensorTimingStats lineSensorTimingStats[LINE_TIMING_SCOPE_COUNT] = {0};
static uint32_t timingPreviousUpdateCount, timingPreviousSkipCount, timingPreviousPowerEpoch;
static uint32_t timingLastUpdateTick, timingLastUpdateScope;
static uint32_t timingPreviousScope = LINE_TIMING_OTHER_SCOPE;
static bool timingGapReady;
#endif
//====================================//
// プロトタイプ宣言
//====================================//

static void normalizeLineSensor(const uint16_t *raw, uint16_t *normalized);

_Static_assert(LS_AVERAGE_SAMPLES == 2U, "Line sensor averaging must retain two samples");

/////////////////////////////////////////////////////////////////////
// モジュール名 skipLineSensorAcquisition
// 処理概要     欠測を記録し、処理中データを含む未完成の組合せを無効化する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void skipLineSensorAcquisition(void)
{
	++adcSkippedCount;
	++acquisitionEpoch;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 lineSensorWindowValid
// 処理概要     CC1要求が同一PWM位相内にあり、全走査の余裕が残っているか確認する
// 引数         なし
// 戻り値       true:取得窓内 false:失効
/////////////////////////////////////////////////////////////////////
static bool lineSensorWindowValid(void)
{
	const uint32_t now = __HAL_TIM_GET_COUNTER(&htim3);
	const uint32_t compare = __HAL_TIM_GET_COMPARE(&htim3, LS_CHANNEL);
	const uint32_t end = requestedPhase ? compare : __HAL_TIM_GET_AUTORELOAD(&htim3) + 1U;
	const uint32_t margin = LS_ADC_SCAN_MARGIN_US * TIMER_TICK / (PSC + 1U);
	// 境界フラグで一周遅れの要求も棄却する。HALはCC1を更新イベントより先に処理する。
	return lineSensorPower && ((htim3.Instance->SR & (TIM_FLAG_UPDATE | TIM_FLAG_CC3)) == 0U)
		&& ((now < compare) == requestedPhase) && now >= requestedTick
		&& now + margin < end;
}
// 仮想センサステア関連
uint16_t lineIndex = 0;
float angleSensor;
// キャリブレーション関連
uint16_t lSensorMax[NUM_SENSORS] = {0};	// 各センサの最大値
uint16_t lSensorMin[NUM_SENSORS] = {[0 ... NUM_SENSORS - 1] = UINT16_MAX};	// 各センサの最小値
uint8_t modeCalLinesensors = 0;
static bool lineSensorCalibrationValid = false;
bool lineSensorSettingCorrupt = false;

/////////////////////////////////////////////////////////////////////
// モジュール名 isLineSensorStoredValueInRange
// 処理概要     lsval.txtの保存値がAD値の許容範囲内か判定する
// 引数         value:判定対象値
// 戻り値       true:範囲内 false:範囲外
/////////////////////////////////////////////////////////////////////
static bool isLineSensorStoredValueInRange(int32_t value)
{
	return value >= 0 && value <= (int32_t)BASEVAL;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 lineSensorStoredValueOr
// 処理概要     保存可能なラインセンサ値を取得し、範囲外なら代替値を返す
// 引数         value:保存候補値, fallback:代替値
// 戻り値       保存するラインセンサ値
/////////////////////////////////////////////////////////////////////
static uint16_t lineSensorStoredValueOr(uint16_t value, uint16_t fallback)
{
	if (isLineSensorStoredValueInRange(value))
	{
		return value;
	}
	return fallback;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 validateLineSensorCalibrationValues
// 処理概要     現在のラインセンサ校正値が全センサで有効か判定する
// 引数         なし
// 戻り値       true:校正値有効 false:校正値無効
/////////////////////////////////////////////////////////////////////
static bool validateLineSensorCalibrationValues(void)
{
	for (uint8_t i = 0; i < NUM_SENSORS; i++)
	{
		if (!isLineSensorStoredValueInRange(lSensorMax[i]) ||
			!isLineSensorStoredValueInRange(lSensorMin[i]) ||
			lSensorMax[i] <= lSensorMin[i])
		{
			return false;
		}
	}
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 readLineSensorStoredValue
// 処理概要     lsval.txtからラインセンサ保存値を1項目読み取る
// 引数         fil:読み取り対象ファイル, value:読み取り値の格納先
// 戻り値       true:読み取り成功 false:読み取り失敗
/////////////////////////////////////////////////////////////////////
static bool readLineSensorStoredValue(FIL *fil, uint16_t *value)
{
	TCHAR str[10] = {0};
	int raw = 0;

	if (f_gets(str, 6, fil) == NULL)
	{
		return false;
	}
	if (sscanf(str, "%d,", &raw) != 1 || !isLineSensorStoredValueInRange(raw))
	{
		return false;
	}

	*value = (uint16_t)raw;
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 isLineSensorCalibrationValid
// 処理概要     ラインセンサ校正値が走行可能な状態か取得する
// 引数         なし
// 戻り値       true:校正値有効 false:校正値無効
/////////////////////////////////////////////////////////////////////
bool isLineSensorCalibrationValid(void)
{
	return lineSensorCalibrationValid;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 finishLineSensorCalibration
// 処理概要     校正を終了し、SD保存とは独立して全センサの校正値を検証する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void finishLineSensorCalibration(void)
{
	modeCalLinesensors = 0;
	lineSensorCalibrationValid = validateLineSensorCalibrationValues();
	lineSensorSettingCorrupt = !lineSensorCalibrationValid;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 powerLineSensors
// 処理概要  	ラインセンサのON/OFF処理
// 引数     	0:OFF 1:ON
// 戻り値    	なし
/////////////////////////////////////////////////////////////////////
void powerLineSensors(uint8_t onoff)
{
	if (onoff > 1U) return;
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	// UIのテスト画面は毎ループONを要求する。同じON要求で平均化を破棄しない。
	if (onoff == 1U && lineSensorPower)
	{
		__set_PRIMASK(primask);
		return;
	}
	__HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC1);
	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC1);
	conversionPending = false;
	++acquisitionEpoch;
	++sensorPowerEpoch;
	sampleAvailable = false;
	pendingLineUpdates = 0U;
	lineSensorUpdateAgeMs = 0;
	lineSensorUpdateStale = false;
	lineSensorVrefSample = 0;
	lineSensorPower = onoff != 0U;
	lineSensorState = false;
	__HAL_TIM_SET_COMPARE(&LS_TIMER, LS_CHANNEL, onoff ? LS_COUNTERPERIOD : 0U);
	powerMarkerSensors(0);
	// 転送中なら占有を保持し、旧世代の完了を棄却してから新規取得を許可する。
	__set_PRIMASK(primask);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 beginLineSensorControlFrame1ms
// 処理概要     ADC取得を継続したまま、制御とログが読む10素子の公開値を固定する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void beginLineSensorControlFrame1ms(void)
{
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	controlFrameActive = true;
	__set_PRIMASK(primask);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 endLineSensorControlFrame1ms
// 処理概要     制御中に完成した最新フレームを、更新件数とともに一括公開する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void endLineSensorControlFrame1ms(void)
{
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	if (lineSensorPower && pendingLineUpdates != 0U)
	{
		memcpy(lSensor, pendingLineRaw, sizeof(pendingLineRaw));
		memcpy(lSensorCari, pendingLineNormalized, sizeof(pendingLineNormalized));
		lineSensorUpdateCount += pendingLineUpdates;
		sampleAvailable = true;
	}
	pendingLineUpdates = 0U;
	controlFrameActive = false;
	__set_PRIMASK(primask);
}
//////////////////////////////////////////////////////////////////////
// モジュール名 lineSensorPhaseChanged
// 処理概要     ADC状態とは独立してLED位相を更新し、同位相内のCC1要求を作成する
// 引数         phase:更新イベントならtrue、PWM比較イベントならfalse
// 戻り値       なし
//////////////////////////////////////////////////////////////////////
void lineSensorPhaseChanged(bool phase)
{
	__HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC1);
	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC1);
	if (conversionPending) skipLineSensorAcquisition();
	conversionPending = false;
	const uint32_t now = __HAL_TIM_GET_COUNTER(&htim3);
	const uint32_t compare = __HAL_TIM_GET_COMPARE(&htim3, LS_CHANNEL);
	lineSensorState = lineSensorPower && now < compare;
	powerMarkerSensors(lineSensorState);
	if (!lineSensorPower) return;
	const uint32_t end = phase ? compare : __HAL_TIM_GET_AUTORELOAD(&htim3) + 1U;
	const uint32_t target = now + LS_ADC_SETTLE_US * TIMER_TICK / (PSC + 1U);
	const uint32_t margin = LS_ADC_SCAN_MARGIN_US * TIMER_TICK / (PSC + 1U);
	if (lineSensorState != phase || target + margin >= end)
	{
		skipLineSensorAcquisition();
		return;
	}
	requestedPhase = phase;
	requestedTick = target;
	conversionPending = true;
	__HAL_TIM_DISABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, target);
	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC1);
	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 startLineSensorAcquisition
// 処理概要     有効なCC1要求だけを消費し、位相・転送先・マーカーGPIOを固定する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void startLineSensorAcquisition(void)
{
	__HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC1);
	if (!conversionPending) return;
	conversionPending = false;
	if (!lineSensorWindowValid() || adcAcquisitionBusy || adcAcquisitionFault ||
		(hadc1.State & HAL_ADC_STATE_BUSY_REG) != 0U ||
		hadc1.Lock == HAL_LOCKED || HAL_DMA_GetState(hadc1.DMA_Handle) != HAL_DMA_STATE_READY)
	{
		skipLineSensorAcquisition();
		return;
	}
	adcAcquisitionBusy = true;
	adcSample.phase = requestedPhase;
	adcSample.buffer = requestedPhase ? analogValLSon : analogValLSoff;
	adcSample.epoch = acquisitionEpoch;
	adcSample.markerLeft = HAL_GPIO_ReadPin(SidesensorL_GPIO_Port, SidesensorL_Pin);
	adcSample.markerRight = HAL_GPIO_ReadPin(SidesensorR_GPIO_Port, SidesensorR_Pin);
	const HAL_StatusTypeDef status = HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcSample.buffer, NUM_SENSORS + 1U);
	if (status != HAL_OK)
	{
		++adcStartErrorCount;
		adcLastStartStatus = status;
		adcLastErrorCode = HAL_ADC_GetError(&hadc1);
		adcAcquisitionFault = true; // HALは失敗時もADCを開始し得るので再使用しない。
		++acquisitionEpoch;
		__DMB();
		adcAcquisitionBusy = false;
	}
	else if (!lineSensorWindowValid() || adcSample.epoch != acquisitionEpoch)
	{
		// HAL開始中のTIM6割込みで取得窓を越えた場合も結果を破棄する。
		skipLineSensorAcquisition();
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 completeLineSensorAcquisition
// 処理概要     固定した取得情報だけで処理し、全処理終了後に占有を解放する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void completeLineSensorAcquisition(void)
{
	if (!adcAcquisitionBusy || adcAcquisitionFault) return;
	if (lineSensorPower && adcSample.epoch == acquisitionEpoch)
	{
		const uint16_t vref = adcSample.buffer[NUM_SENSORS];
		getLineSensor(adcSample.phase, adcSample.buffer, adcSample.epoch);
		const uint32_t primask = __get_PRIMASK();
		__disable_irq();
		if (lineSensorPower && adcSample.epoch == acquisitionEpoch)
		{
			if (vref > 0U && vref <= 4095U) lineSensorVrefSample = vref;
			getMarkerSensor(adcSample.phase, adcSample.markerLeft, adcSample.markerRight, adcSample.epoch);
		}
		__set_PRIMASK(primask);
	}
	__DMB(); // バッファ参照・作業配列更新を完了してから次のTIM3取得へ渡す。
	adcAcquisitionBusy = false;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 failLineSensorAcquisition
// 処理概要     非同期HAL異常を記録し、取得状態を解放して再使用を禁止する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void failLineSensorAcquisition(void)
{
	adcAcquisitionFault = true;
	adcLastErrorCode = HAL_ADC_GetError(&hadc1);
	++adcTransferErrorCount;
	++acquisitionEpoch;
	__DMB();
	adcAcquisitionBusy = false;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 monitorLineSensorUpdate1ms
// 処理概要     有効更新番号から更新間隔を診断する。緊急停止条件には接続しない。
// 引数         steeringUsesLine:現在の操舵がラインセンサを使用するか
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void monitorLineSensorUpdate1ms(bool steeringUsesLine)
{
	static uint32_t previousCount = 0;
	static bool monitoring = false;
	const uint32_t count = lineSensorUpdateCount;
	if (!lineSensorPower || !sampleAvailable)
	{
		previousCount = count;
		monitoring = false;
		lineSensorUpdateAgeMs = 0;
		lineSensorUpdateStale = false;
		return;
	}
	if (!monitoring)
	{
		monitoring = true;
		lineSensorUpdateAgeMs = 0;
	}
	else
	{
		if (lineSensorUpdateAgeMs < UINT32_MAX) ++lineSensorUpdateAgeMs;
		if (lineSensorUpdateAgeMs > lineSensorMaxUpdateAgeMs)
			lineSensorMaxUpdateAgeMs = lineSensorUpdateAgeMs;
		if (count != previousCount) lineSensorUpdateAgeMs = 0;
	}
	previousCount = count;
	const bool stale = steeringUsesLine && lineSensorUpdateAgeMs >= LS_UPDATE_STALE_MS;
	if (stale && !lineSensorUpdateStale) ++lineSensorStaleCount;
	lineSensorUpdateStale = stale;
}
#if defined(ROBOTRACE_ISR_TIMING) && ROBOTRACE_ISR_TIMING
/////////////////////////////////////////////////////////////////////
// モジュール名 addLineTimingCount
// 処理概要     診断カウンタを上限で飽和させて加算する
// 引数         count: 元の値, increment: 加算値
// 戻り値       飽和後の値
/////////////////////////////////////////////////////////////////////
static uint32_t addLineTimingCount(uint32_t count, uint32_t increment)
{
	return increment > UINT32_MAX - count ? UINT32_MAX : count + increment;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 recordLineSensorTiming1ms
// 処理概要     操舵の使用条件によらず、モード別に更新間隔と棄却数を観測する
// 引数         mode: BOOST走行モード, tracePattern: 制御状態, running:走行区間か
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void recordLineSensorTiming1ms(uint8_t mode, uint8_t tracePattern, bool running)
{
	const uint32_t scope = running && mode < LINE_TIMING_OTHER_SCOPE && mode != 1U ? mode : LINE_TIMING_OTHER_SCOPE;
	volatile LineSensorTimingStats *stats = &lineSensorTimingStats[scope];
	const uint32_t count = lineSensorUpdateCount, skipped = adcSkippedCount;
	const uint32_t epoch = sensorPowerEpoch, now = HAL_GetTick();
	const bool updated = count != timingPreviousUpdateCount;
	stats->samples = addLineTimingCount(stats->samples, 1U);
	stats->updateEvents = addLineTimingCount(stats->updateEvents, count - timingPreviousUpdateCount);
	// 区間の末尾で観測したモードへ割当てる。境界をまたぐ要求差分を含む。
	stats->skippedRequests = addLineTimingCount(stats->skippedRequests, skipped - timingPreviousSkipCount);
	if (!lineSensorPower || !sampleAvailable || epoch != timingPreviousPowerEpoch || scope != timingPreviousScope)
		timingGapReady = false;
	if (lineSensorPower) stats->poweredSamples = addLineTimingCount(stats->poweredSamples, 1U);
	if (lineSensorPower && sampleAvailable)
	{
		stats->validSamples = addLineTimingCount(stats->validSamples, 1U);
		const uint32_t age = lineSensorUpdateAgeMs;
		if (age > stats->maxAgeMs) stats->maxAgeMs = age;
		if (age >= LS_UPDATE_STALE_MS) stats->staleSamples = addLineTimingCount(stats->staleSamples, 1U);
		if (updated)
		{
			if (timingGapReady && timingLastUpdateScope == scope)
			{
				const uint32_t gap = now - timingLastUpdateTick;
				const uint32_t bin = gap < LINE_TIMING_GAP_BINS - 1U ? gap : LINE_TIMING_GAP_BINS - 1U;
				stats->gapHistogram[bin] = addLineTimingCount(stats->gapHistogram[bin], 1U);
				if (gap > stats->maxGapMs)
				{
					stats->maxGapMs = gap;
					stats->maxGapTickMs = now;
					stats->maxGapPattern = tracePattern;
					stats->maxGapCalibration = modeCalLinesensors;
				}
			}
			timingLastUpdateTick = now;
			timingLastUpdateScope = scope;
			timingGapReady = true;
		}
	}
	timingPreviousUpdateCount = count;
	timingPreviousSkipCount = skipped;
	timingPreviousPowerEpoch = epoch;
	timingPreviousScope = scope;
}
#endif
/////////////////////////////////////////////////////////////////////
// モジュール名 getLineSensor
// 処理概要  	ラインセンサのAD値を取得し、平均値を計算する
// 引数         phase:取得時LED位相, sample:完了済みDMA値, epoch:連続取得世代
// 戻り値    	なし
/////////////////////////////////////////////////////////////////////
void getLineSensor(bool phase, const uint16_t *sample, uint32_t epoch)
{
	static uint32_t previousEpoch = UINT32_MAX;
	static uint32_t previousPowerEpoch = UINT32_MAX;
	const uint32_t powerEpoch = sensorPowerEpoch;
	const uint32_t nowMs = HAL_GetTick();
	if (previousPowerEpoch != powerEpoch ||
		(completedPairCount != 0U && nowMs - oldestPairTimeMs > LS_AVERAGE_MAX_PAIR_AGE_MS))
	{
		completedPairCount = 0;
		readyMask = 0;
		previousPowerEpoch = powerEpoch;
	}
	if (previousEpoch != epoch)
	{
		// 欠測前の未完成ペアだけを破棄する。完成済みのペアは上限時間内で保持する。
		readyMask = 0;
		previousEpoch = epoch;
	}
	memcpy(pairSample[phase], sample, sizeof(pairSample[phase]));
	readyMask |= (1U << phase);
	if (readyMask != 0x03U) return;
	readyMask = 0;

	// 同じ欠測世代で取得できたON/OFFだけを一つの完成ペアとして積算する。
	uint32_t candidate[2][NUM_SENSORS];
	for (uint8_t p = 0; p < 2U; ++p)
		for (uint8_t i = 0; i < NUM_SENSORS; ++i)
			candidate[p][i] = (completedPairCount ? accum[p][i] : 0U) + pairSample[p][i];

	const uint32_t pairPrimask = __get_PRIMASK();
	__disable_irq();
	if (!lineSensorPower || powerEpoch != sensorPowerEpoch)
	{
		__set_PRIMASK(pairPrimask);
		return;
	}
	if (++completedPairCount < LS_AVERAGE_SAMPLES)
	{
		memcpy(accum, candidate, sizeof(candidate));
		oldestPairTimeMs = nowMs;
		__set_PRIMASK(pairPrimask);
		return;
	}
	completedPairCount = 0;
	__set_PRIMASK(pairPrimask);

	uint16_t raw[NUM_SENSORS], normalized[NUM_SENSORS];
	for (uint8_t i = 0; i < NUM_SENSORS; i++)
	{
		const int32_t diff = (int32_t)(candidate[0][i] / LS_AVERAGE_SAMPLES) -
			(int32_t)(candidate[1][i] / LS_AVERAGE_SAMPLES);
		raw[i] = (diff > 0) ? (uint16_t)diff : 0U;
	}
	normalizeLineSensor(raw, normalized);
	// 後続要求の欠測は完成済みペアを無効化しない。電源世代だけを再確認する。
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	if (lineSensorPower && powerEpoch == sensorPowerEpoch)
	{
		if (controlFrameActive)
		{
			// 高優先度ADCから制御途中の配列を置換せず、完成した最新値だけを保留する。
			memcpy(pendingLineRaw, raw, sizeof(raw));
			memcpy(pendingLineNormalized, normalized, sizeof(normalized));
			if (pendingLineUpdates < UINT32_MAX) ++pendingLineUpdates;
		}
		else
		{
			memcpy(lSensor, raw, sizeof(raw));
			memcpy(lSensorCari, normalized, sizeof(normalized));
			++lineSensorUpdateCount;
			sampleAvailable = true;
		}
	}
	__set_PRIMASK(primask);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 getAngleSensor
// 処理概要  	ラインセンサのAD値からステア角を算出する
// 引数     	なし
// 戻り値    	なし
/////////////////////////////////////////////////////////////////////
void getAngleSensor(void)
{
	uint16_t index = 0U, sen1, sen2, i, min;
	float nsen1, nsen2, phi, dthita;

	// 全入力範囲で最小値を選び、端や無効入力では角度を0にする。
	angleSensor = 0.0F;
	min = lSensor[0];
	for (i = 1U; i < NUM_SENSORS; i++)
	{
		if (lSensor[i] < min)
		{
			min = lSensor[i];
			index = i;
		}
	}

	if (index > 0U && index < NUM_SENSORS - 1U)
	{ // 両端のセンサが白線の上にあるときは無視
		// 白線に一番近いセンサの両隣のセンサ値を取得
		sen1 = lSensor[index - 1];
		sen2 = lSensor[index + 1];
		if (sen1 == 0U && sen2 == 0U) return;
		// 正規化
		nsen1 = (float)sen1 / (sen1 + sen2);
		nsen2 = (float)sen2 / (sen1 + sen2);
		if (index >= NUM_SENSORS / 2)
		{
			phi = atanf((nsen1 - nsen2) / 1); // 偏角φ計算
		}
		else
		{
			phi = atanf((nsen2 - nsen1) / 1); // 偏角φ計算
		}
		dthita = (phi * THITA_SENSOR * (IMU_PI_F / 180.0f) / 2) / (IMU_PI_F / 4); // 微小角度dθ計算

		// センサ角度と微小角度を足す
		if (index >= NUM_SENSORS / 2)
		{
			angleSensor = ((index - 5.5f) * THITA_SENSOR * (IMU_PI_F / 180.0f)) + dthita;
		}
		else
		{
			angleSensor = -(((5.5f - index) * THITA_SENSOR * (IMU_PI_F / 180.0f)) + dthita);
		}
		angleSensor = angleSensor * (180.0f / IMU_PI_F); // 弧度法に変換
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 normalizeLineSensor
// 処理概要  	ラインセンサのAD値を正規化する
// 引数         raw:差分AD値, normalized:正規化値の格納先
// 戻り値    	なし
/////////////////////////////////////////////////////////////////////
static void normalizeLineSensor(const uint16_t *raw, uint16_t *normalized)
{
	const uint32_t baseVal = (uint32_t)BASEVAL;

	for (uint8_t i = 0; i < NUM_SENSORS; i++)
	{
		uint16_t current = raw[i];

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
				normalized[i] = (uint16_t)baseVal;
			}
			else
			{
				normalized[i] = (uint16_t)((offset * baseVal + (span / 2U)) / span);
			}
		}
		else
		{
			normalized[i] = current;
		}
	}
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 writeLinesenval
// 処理概要     ラインセンサの最大値と最小値をSDカードに書き込む
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void writeLinesenval(void)
{
	FIL fil;
	FRESULT fresult;
	char str[10], fileName[20] = PATH_SETTING;
	int16_t i;

	// ファイル読み込み
	strcat(fileName, FILENAME_LS_VAL);							 // ファイル名追加
	strcat(fileName, ".txt");									 // 拡張子追加
	fresult = f_open(&fil, fileName, FA_OPEN_ALWAYS | FA_WRITE); // ファイルを開く

	if (fresult != FR_OK)
	{
		lineSensorCalibrationValid = validateLineSensorCalibrationValues();
		lineSensorSettingCorrupt = !lineSensorCalibrationValid;
		return;
	}

	if (fresult == FR_OK)
	{
		f_lseek(&fil, 0);
		f_truncate(&fil);
		// 最大値を保存
		for (i = 0; i < NUM_SENSORS; i++)
		{
			sprintf(str, "%04u,", lineSensorStoredValueOr(lSensorMax[i], 0U));
			f_puts(str, &fil);
		}
		// 最小値を保存
		for (i = 0; i < NUM_SENSORS; i++)
		{
			sprintf(str, "%04u,", lineSensorStoredValueOr(lSensorMin[i], (uint16_t)BASEVAL));
			f_puts(str, &fil);
		}
	}

	f_close(&fil);
	lineSensorCalibrationValid = validateLineSensorCalibrationValues();
	lineSensorSettingCorrupt = !lineSensorCalibrationValid;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 readLinesenval
// 処理概要     ラインセンサの最大値と最小値をSDカードから読み取る
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void readLinesenval(void)
{
	FIL fil;
	FRESULT fresult;
	char fileName[20] = PATH_SETTING;
	int16_t i;
	bool repair = false;

	// ファイル読み込み
	strcat(fileName, FILENAME_LS_VAL);							  // ファイル名追加
	strcat(fileName, ".txt");									  // 拡張子追加
	fresult = f_open(&fil, fileName, FA_OPEN_EXISTING | FA_READ); // ファイルを開く

	if (fresult == FR_OK)
	{
		for (i = 0; i < NUM_SENSORS; i++)
		{
			uint16_t value = lSensorMax[i];
			if (readLineSensorStoredValue(&fil, &value))
			{
				lSensorMax[i] = value;
			}
			else
			{
				repair = true;
				break;
			}
		}
		for (i = 0; i < NUM_SENSORS && !repair; i++)
		{
			uint16_t value = lSensorMin[i];
			if (readLineSensorStoredValue(&fil, &value))
			{
				lSensorMin[i] = value;
			}
			else
			{
				repair = true;
				break;
			}
		}
		f_close(&fil);
	}
	else
	{
		repair = true;
	}

	lineSensorCalibrationValid = validateLineSensorCalibrationValues();
	lineSensorSettingCorrupt = repair || !lineSensorCalibrationValid;
	if (repair)
	{
		writeLinesenval();
		lineSensorSettingCorrupt = repair || !lineSensorCalibrationValid;
	}
}
