//====================================//
// インクルード
//====================================//
#include "lineSensor.h"
#include "fatfs.h"
#include <stdbool.h>
#include <stdint.h>
//====================================//
// グローバル変数の宣言
//====================================//

// ラインセンサ関連
uint32_t lSensorInt[NUM_SENSORS] = {0};	 // ラインセンサの立ち上がりエッジAD値積算用
uint16_t lSensor[NUM_SENSORS] = {0};	 // ラインセンサの平均AD値
uint16_t lSensorCari[NUM_SENSORS] = {0}; // 正規化したラインセンサのAD値
bool lineSensorState = false;			 // true:ラインセンサ点灯 false:ラインセンサ消灯
bool lineSensorPower = false;			 // ラインセンサ電源状態
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
// モジュール名 isLineSensorSettingCorrupt
// 処理概要     ラインセンサ設定ファイルの破損検出状態を取得する
// 引数         なし
// 戻り値       true:破損検出あり false:破損検出なし
/////////////////////////////////////////////////////////////////////
bool isLineSensorSettingCorrupt(void)
{
	return lineSensorSettingCorrupt;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 powerLineSensors
// 処理概要  	ラインセンサのON/OFF処理
// 引数     	0:OFF 1:ON
// 戻り値    	なし
/////////////////////////////////////////////////////////////////////
void powerLineSensors(uint8_t onoff)
{
	if (onoff == 0)
	{
		lineSensorPower = false;
		__HAL_TIM_SET_COMPARE(&LS_TIMER, LS_CHANNEL, 0);
	}
	else if (onoff == 1)
	{
		lineSensorPower = true;
		__HAL_TIM_SET_COMPARE(&LS_TIMER, LS_CHANNEL, LS_COUNTERPERIOD);
	}
}
//////////////////////////////////////////////////////////////////////
// モジュール名 arm_cc1_after_us
// 処理概要  	TIM3のCC1割り込みを指定したus後に発生させる（ワンショット）
// 引数     	us: 割り込み発生までの時間[us]
// 戻り値    	なし
//////////////////////////////////////////////////////////////////////
void delayLineSensorConversionStart(uint32_t us)
{
	uint32_t ticks;
	// 指定したus後にTIM3のCC1割り込みが発生するようにセット（ワンショット）
    // 安全マージン（レース回避で最低+10tick）
    if(us < 10) us = 10; // レース回避
	ticks = us * TIMER_TICK/(PSC+1);
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
    uint32_t now = __HAL_TIM_GET_COUNTER(&htim3);
    // uint32_t tgt = now + ticks; if(tgt > arr) tgt -= (arr+1);
	uint32_t tgt = (now + ticks) % (arr+1);

    __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC1);				// 念のため無効化
    __HAL_TIM_DISABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_1);	// OC1PE=0を保証
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, tgt);		// 即反映
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC1);					// 念のためクリア	
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);				// 有効化
}
/////////////////////////////////////////////////////////////////////
// モジュール名 getLineSensor
// 処理概要  	ラインセンサのAD値を取得し、平均値を計算する
// 引数     	なし
// 戻り値    	なし
/////////////////////////////////////////////////////////////////////
void getLineSensor(void)
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
			lSensor[i] = (diff > 0) ? (int16_t)diff : 0;
		}
		readyMask = 0;
		calibrationLinesensor(); // 最新のライン値でキャリブレーション／正規化を更新
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 getAngleSensor
// 処理概要  	ラインセンサのAD値からステア角を算出する
// 引数     	なし
// 戻り値    	なし
/////////////////////////////////////////////////////////////////////
void getAngleSensor(void)
{
	uint16_t index, sen1, sen2, i, min;
	float nsen1, nsen2, phi, dthita;

	min = 1000;
	for (i = 0; i < NUM_SENSORS; i++)
	{
		if (lSensor[i] < min)
		{
			min = lSensor[i];
			index = i;
		}
	}

	if (index >= 0 && index <= NUM_SENSORS - 1)
	{ // 両端のセンサが白線の上にあるときは無視
		// 白線に一番近いセンサの両隣のセンサ値を取得
		sen1 = lSensor[index - 1];
		sen2 = lSensor[index + 1];
		// 正規化
		nsen1 = (float)sen1 / (sen1 + sen2);
		nsen2 = (float)sen2 / (sen1 + sen2);
		if (index >= NUM_SENSORS / 2)
		{
			phi = atan((nsen1 - nsen2) / 1); // 偏角φ計算
		}
		else
		{
			phi = atan((nsen2 - nsen1) / 1); // 偏角φ計算
		}
		dthita = (phi * THITA_SENSOR * (M_PI / 180.0) / 2) / (M_PI / 4); // 微小角度dθ計算

		// センサ角度と微小角度を足す
		if (index >= NUM_SENSORS / 2)
		{
			angleSensor = ((index - 5.5) * THITA_SENSOR * (M_PI / 180.0)) + dthita;
		}
		else
		{
			angleSensor = -(((5.5 - index) * THITA_SENSOR * (M_PI / 180.0)) + dthita);
		}
		angleSensor = angleSensor * (180.0 / M_PI); // 弧度法に変換
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 calibrationLinesensor
// 処理概要  	ラインセンサのAD値を正規化する
// 引数     	なし
// 戻り値    	なし
/////////////////////////////////////////////////////////////////////
void calibrationLinesensor(void)
{
	const uint32_t baseVal = (uint32_t)BASEVAL;

	for (uint8_t i = 0; i < NUM_SENSORS; i++)
	{
		uint16_t current = (lSensor[i] > 0) ? (uint16_t)lSensor[i] : 0U;

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
				lSensorCari[i] = (uint16_t)baseVal;
			}
			else
			{
				lSensorCari[i] = (uint16_t)((offset * baseVal + (span / 2U)) / span);
			}
		}
		else
		{
			lSensorCari[i] = current;
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
