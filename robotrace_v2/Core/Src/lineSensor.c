//====================================//
// インクルード
//====================================//
#include "lineSensor.h"
#include "fatfs.h"
//====================================//
// グローバル変数の宣言
//====================================//

// ラインセンサ関連
uint32_t lSensorInt[NUM_SENSORS] = {0};	 // ラインセンサの立ち上がりエッジAD値積算用
uint16_t lSensor[NUM_SENSORS] = {0};	 // ラインセンサの平均AD値
uint16_t lSensorCari[NUM_SENSORS] = {0}; // 正規化したラインセンサのAD値
bool lineSensorState = false;			 // true:ラインセンサ点灯 false:ラインセンサ消灯
// 仮想センサステア関連
uint16_t lineIndex = 0;
float angleSensor;
// キャリブレーション関連
uint16_t lSensorMax[NUM_SENSORS] = {0};	// 各センサの最大値
uint16_t lSensorMin[NUM_SENSORS] = {[0 ... NUM_SENSORS - 1] = UINT16_MAX};	// 各センサの最小値
uint8_t modeCalLinesensors = 0;
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
		lineSensorState = false;
		__HAL_TIM_SET_COMPARE(&LS_TIMER, LS_CHANNEL, 0);
	}
	else if (onoff == 1)
	{
		lineSensorState = true;
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
	// 指定したus後にTIM3のCC1割り込みが発生するようにセット（ワンショット）
    // 安全マージン（レース回避で最低+10tick）
    if(us < 10) us = 10; // レース回避
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
    uint32_t now = __HAL_TIM_GET_COUNTER(&htim3);
    uint32_t tgt = now + us; if(tgt > arr) tgt -= (arr+1);

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
			average[phase][i] = acc[i] / LS_AVERAGE_SAMPLES;
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
	uint8_t str[10], fileName[20] = PATH_SETTING;
	int16_t i, ret = 0;

	// ファイル読み込み
	strcat(fileName, FILENAME_LS_VAL);							 // ファイル名追加
	strcat(fileName, ".txt");									 // 拡張子追加
	fresult = f_open(&fil, fileName, FA_OPEN_ALWAYS | FA_WRITE); // ファイルを開く

	if (fresult == FR_OK)
	{
		// 最大値を保存
		for (i = 0; i < NUM_SENSORS; i++)
		{
			sprintf(str, "%04d,", lSensorMax[i]);
			f_puts(str, &fil);
		}
		// 最小値を保存
		for (i = 0; i < NUM_SENSORS; i++)
		{
			sprintf(str, "%04d,", lSensorMin[i]);
			f_puts(str, &fil);
		}
	}

	f_close(&fil);
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
	uint8_t fileName[20] = PATH_SETTING;
	TCHAR str[10];
	int16_t i, ret = 0;

	// ファイル読み込み
	strcat(fileName, FILENAME_LS_VAL);							  // ファイル名追加
	strcat(fileName, ".txt");									  // 拡張子追加
	fresult = f_open(&fil, fileName, FA_OPEN_EXISTING | FA_READ); // ファイルを開く

	if (fresult == FR_OK)
	{
		// 最大値を読み込む
		for (i = 0; i < NUM_SENSORS; i++)
		{
			f_gets(str, 6, &fil);
			sscanf(str, "%hu,", &lSensorMax[i]);
		}
		// 最小値を読み込む
		for (i = 0; i < NUM_SENSORS; i++)
		{
			f_gets(str, 6, &fil);
			sscanf(str, "%hu,", &lSensorMin[i]);
		}
	}

	f_close(&fil);
}
