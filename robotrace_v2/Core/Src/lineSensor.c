//====================================//
// インクルード
//====================================//
#include "lineSensor.h"
#include "fatfs.h"
#include <stdbool.h>
#include <stdint.h>

#define LS_AVERAGE_SAMPLES 16U
//====================================//
// グローバル変数の宣言
//====================================//

// ラインセンサ関連
uint32_t lSensorInt[NUM_SENSORS] = {0};	 // ラインセンサの立ち上がりエッジAD値積算用
int16_t lSensor[NUM_SENSORS] = {0};	 // ラインセンサの平均AD値
uint16_t lSensorCari[NUM_SENSORS] = {0}; // 正規化したラインセンサのAD値
bool lineSensorState = false;			 // true:ラインセンサ点灯 false:ラインセンサ消灯
// 仮想センサステア関連
uint16_t lineIndex = 0;
float angleSensor = 0.0F;
// キャリブレーション関連
uint16_t lSensorMax[NUM_SENSORS] = {0};	// 各センサの最大値
uint16_t lSensorMin[NUM_SENSORS] = {[0 ... NUM_SENSORS - 1] = UINT16_MAX};	// 各センサの最小値
uint8_t modeCalLinesensors = 0;
/////////////////////////////////////////////////////////////////////
// モジュール名 isCrossLinePattern
// 処理概要  	クロスライン特有の明暗パターンを判定する
// 引数     	sensorValues: 判定に使用するセンサ配列
// 戻り値   	true:クロスライン false:通常ライン
/////////////////////////////////////////////////////////////////////
static bool isCrossLinePattern(const uint16_t *sensorValues)
{
	// センサ配列の最大値を取得し、以降の閾値計算の基準とする
	uint16_t peak = 0U;

	for (uint8_t i = 0; i < NUM_SENSORS; i++)
	{
		if (sensorValues[i] > peak)
		{
			peak = sensorValues[i];
		}
	}

	if (peak == 0U)
	{
		return false;
	}

	// 最大値の70%を閾値とし、白線を検出するための明るさ基準を動的に決定
	const uint16_t crossThreshold = (uint16_t)(((uint32_t)peak * 7U) / 10U);

	if (crossThreshold == 0U)
	{
		return false;
	}

	// 端部センサ4本の合計値が閾値を超える場合はクロスラインと判定する
	const uint32_t edgeThreshold = (uint32_t)crossThreshold * 4U;
	const uint32_t edgeSum = (uint32_t)sensorValues[0] + (uint32_t)sensorValues[1] +
	                         (uint32_t)sensorValues[NUM_SENSORS - 2] + (uint32_t)sensorValues[NUM_SENSORS - 1];

	// Treat bright edges as a cross-line pattern when the sum is large
	if (edgeSum >= edgeThreshold)
	{
		return true;
	}

	uint8_t activeCount = 0;
	uint8_t firstIndex = NUM_SENSORS;
	uint8_t lastIndex = 0;

	// 十分に明るい領域が連続しているかを調べ、クロスラインパターンを検出する
	// Detect consecutive high-intensity sensors as a cross line
	for (uint8_t i = 0; i < NUM_SENSORS; i++)
	{
		if (sensorValues[i] >= crossThreshold)
		{
			activeCount++;
			if (firstIndex == NUM_SENSORS)
			{
				firstIndex = i;
			}
			lastIndex = i;
		}
	}

	return (activeCount >= 5U) && ((lastIndex - firstIndex) >= (NUM_SENSORS / 2));
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
		__HAL_TIM_SET_COMPARE(&LS_TIMER, LS_CHANNEL, 0);
	}
	else if (onoff == 1)
	{
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
// 戻り値    	angleSensor ラインセンサのステア角[rad]
/////////////////////////////////////////////////////////////////////
float getAngleSensor(void)
{
	uint16_t indexL, indexR, index, sen1, sen2, maxVal;
	const uint16_t *sensorValues;
	float nsen1, nsen2, phi, dthita;
	static float beforeAngle = 0.0F;
	static const float thitaRad = THITA_SENSOR * (float)M_PI / 180.0f; // 角度をラジアンへ変換する係数を事前計算
	static const float degPerRad = 180.0f / (float)M_PI; // ラジアン→度変換係数を事前計算
	static const float thitaScale = THITA_SENSOR / 90.0f; // 微小角度の換算係数を事前計算して乗算回数を削減

	// キャリブレーション済みなら正規化済みの値を使用する
	if (lSensorMax[0] > lSensorMin[0])
	{
		sensorValues = lSensorCari;
	}
	else
	{
		sensorValues = lSensor;
	}

	// クロスライン通過中は直前角度を維持してステア角の乱れを防ぐ
	if (isCrossLinePattern(sensorValues))
	{
		angleSensor = beforeAngle;
		return angleSensor;
	}

	// 最大値を用いてライン中心を推定するため、中央から左右の最大値センサを探索する
	index = NUM_SENSORS; // 最大値探索用インデックス初期化
	indexL = NUM_SENSORS / 2 - 1; // 左側の最大値インデックス初期化
	indexR = NUM_SENSORS / 2; // 右側の最大値インデックス初期化
	maxVal = 0U; // 最大値初期化

	for (int16_t i = NUM_SENSORS / 2 - 1; i >= 0; i--)
	{
		uint16_t current = sensorValues[i]; // 配列アクセスを一時変数に保持して再利用
		if (current > maxVal)
		{
			maxVal = current;
			indexL = i;
		}
	}

	maxVal = 0U; // 最大値初期化
	for (int16_t i = NUM_SENSORS / 2; i < NUM_SENSORS; i++)
	{
		uint16_t current = sensorValues[i]; // 配列アクセスを一時変数に保持して再利用
		if (current > maxVal)
		{
			maxVal = current;
			indexR = i;
		}
	}

	// 左右の最大値を比較して全体の最大値インデックスを決定
	if (sensorValues[indexL] > sensorValues[indexR])
	{
		index = indexL;
	}
	else
	{
		index = indexR;
	}

	if (indexL == NUM_SENSORS / 2 - 1 && indexR == NUM_SENSORS / 2)
	{
		// 両端センサで同じ値が検出された場合は0を返す
		index = NUM_SENSORS;
	}

	if (index > 0 && index < NUM_SENSORS - 1)
	{
		// 両端での配列アクセスを避ける
		sen1 = sensorValues[index - 1];
		sen2 = sensorValues[index + 1];
		uint32_t sum = (uint32_t)sen1 + (uint32_t)sen2; // 隣接値合計を32bitで確保し加算回数を削減
		if (sum == 0U)
		{
			angleSensor = beforeAngle;
			return angleSensor; // ゼロ割防止として直前角度を保持
		}
		float invSum = 1.0f / (float)sum; // 逆数を保持して除算回数を削減
		// 正規化
		nsen1 = sen1 * invSum;
		nsen2 = sen2 * invSum;
		if (index >= NUM_SENSORS / 2)
		{
			phi = atanf(nsen2 - nsen1); // 偏角φ計算
		}
		else
		{
			phi = atanf(nsen1 - nsen2); // 偏角φ計算
		}
		dthita = phi * thitaScale; // 微小角度dθ計算（定数計算を簡略化）

		// センサ角度と微小角度を足す
		if (index >= NUM_SENSORS / 2)
		{
			angleSensor = (((float)index - 5.0F + 1) * thitaRad) + dthita;
		}
		else
		{
			angleSensor = -(((4.0F - (float)index + 1) * thitaRad) + dthita);
		}
	}
	else if (index == NUM_SENSORS)
	{
		// センター付近で同じ値が検出された場合
		sen1 = sensorValues[indexL];
		sen2 = sensorValues[indexR];
		uint32_t sum = (uint32_t)sen1 + (uint32_t)sen2; // 隣接値合計を32bitで確保し加算回数を削減
		if (sum == 0U)
		{
			angleSensor = beforeAngle;
			return angleSensor; // ゼロ割防止として直前角度を保持
		}
		float invSum = 1.0f / (float)sum; // 逆数を保持して除算回数を削減
		// 正規化
		nsen1 = sen1 * invSum;
		nsen2 = sen2 * invSum;
		if (sensorValues[indexL] > sensorValues[indexR])
		{
			phi = atanf(nsen1 - nsen2); // 偏角φ計算
		}
		else
		{
			phi = atanf(nsen2 - nsen1); // 偏角φ計算
		}
		dthita = phi * thitaScale; // 微小角度dθ計算（定数計算を簡略化）
		if (sensorValues[indexL] > sensorValues[indexR])
		{
			angleSensor = -dthita;
		}
		else
		{
			angleSensor = dthita;
		}
	}
	else
	{
		angleSensor = beforeAngle; // 端センサで検出されなかった場合は前回値を使用
	}

	beforeAngle = angleSensor;
	return angleSensor;
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
