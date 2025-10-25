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
	// 先頭・末尾センサの合計値が小さい場合は全体的な暗転とみなし、クロスラインを検出
	const uint32_t edgeSum = (uint32_t)sensorValues[0] + (uint32_t)sensorValues[1] +
	                         (uint32_t)sensorValues[NUM_SENSORS - 2] + (uint32_t)sensorValues[NUM_SENSORS - 1];
	const uint32_t edgeThreshold = 4000U;

	if (edgeSum < edgeThreshold)
	{
		return true;
	}

	const uint16_t crossThreshold = (uint16_t)(BASEVAL * 0.3f);
	uint8_t activeCount = 0;
	uint8_t firstIndex = NUM_SENSORS;
	uint8_t lastIndex = 0;

	// 低照度のセンサが一定数以上連続している場合はクロスライン通過と判断する
	for (uint8_t i = 0; i < NUM_SENSORS; i++)
	{
		if (sensorValues[i] <= crossThreshold)
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
/////////////////////////////////////////////////////////////////////
// モジュール名 getLineSensor
// 処理概要  	ラインセンサのAD値を取得し、平均値を計算する
// 引数     	なし
// 戻り値    	なし
/////////////////////////////////////////////////////////////////////
void getLineSensor(void)
{
	uint8_t i;
	static uint16_t cntls=0,cntlson=0,cntlsoff=0; // ラインセンサの立ち上がりエッジ積算回数カウント用
	static uint32_t lSensorOnInt[NUM_SENSORS] = {0};	 // ラインセンサの立ち上がりエッジAD値積算用
	static uint32_t lSensorOffInt[NUM_SENSORS] = {0};	 // ラインセンサの立ち下がりエッジAD値積算用
	static uint32_t lSensorOn[NUM_SENSORS] = {0};	 // ラインセンサの立ち上がりエッジAD値保存用
	static uint32_t lSensorOff[NUM_SENSORS] = {0};	 // ラインセンサの立ち下がりエッジAD値保存用

	cntls++;

	for (i = 0; i < NUM_SENSORS; i++)
	{
		if(lineSensorState == true)
		{
			cntlson++;
			lSensorOnInt[i] += analogValLSon[i];
			if(cntlson >= 16)
			{
				cntlson = 0;
				lSensorOn[i] = lSensorOnInt[i] >> 4; // 平均値算出
				lSensorOnInt[i] = 0;				 // 積算値リセット
			}
		}
		
		if(lineSensorState == false)
		{
			cntlsoff++;
			lSensorOffInt[i] += analogValLSoff[i];
			if(cntlsoff >= 16)
			{
				cntlsoff = 0;
				lSensorOff[i] = lSensorOffInt[i] >> 4; // 平均値算出
				lSensorOffInt[i] = 0;				 // 積算値リセット
			}
		}

		lSensor[i] = lSensorOff[i] - lSensorOn[i];

		// // ラインセンサ値は点灯時と消灯時の差分を使用する
		// if (lSensorOff[i] > lSensorOn[i])
		// {
		// 	lSensor[i] = lSensorOff[i] - lSensorOn[i];
		// }
		// else
		// {
		// 	lSensor[i] = 0;
		// }
		
	// 	if (cntls > 32)
	// 	{
	// 		// ラインセンサ値は点灯時と消灯時の差分を使用する
	// 		if (lSensorOn[i] > lSensorOff[i])
	// 		{
	// 			lSensor[i] = lSensorOn[i] - lSensorOff[i];
	// 		}
	// 		else
	// 		{
	// 			lSensor[i] = 0;
	// 		}

	// 		// // 最大値・最小値を更新
	// 		// if (modeCalLinesensors == 1)
	// 		// {
	// 		// 	if (lSensor[i] > lSensorMax[i])
	// 		// 	{
	// 		// 		lSensorMax[i] = lSensor[i];	// 最大値更新
	// 		// 	}
	// 		// 	if (lSensor[i] < lSensorMin[i])
	// 		// 	{
	// 		// 		lSensorMin[i] = lSensor[i];	// 最小値更新
	// 		// 	}
	// 		// }
	// 		// // 正規化計算
	// 		// uint16_t range = lSensorMax[i] - lSensorMin[i];
	// 		// if (range != 0)
	// 		// {
	// 		// 	lSensorCari[i] = (uint16_t)((lSensor[i] - lSensorMin[i]) * BASEVAL / range);
	// 		// }
	// 		// else
	// 		// {
	// 		// 	lSensorCari[i] = 0;	// 分母ゼロ対策
	// 		// }
			// cntls = 0;
	// 	}
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
	uint16_t indexL, indexR, index, sen1, sen2, min;
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

	index = NUM_SENSORS; // 最小値探索用インデックス初期化
	indexL = NUM_SENSORS/2 - 1;	// 左側の最小値インデックス初期化
	indexR = NUM_SENSORS/2; 	// 右側の最小値インデックス初期化
	min = 4000; // 最小値初期化

	for (int16_t i = NUM_SENSORS/2 - 1; i >= 0; i--)
	{
		uint16_t current = sensorValues[i]; // 配列アクセスを一時変数に保持して再利用
		if (current < min)
		{
			min = current;
			indexL = i;
		}
	}

	min = 4000; // 最小値初期化
	for (int16_t i = NUM_SENSORS/2; i < NUM_SENSORS; i++)
	{
		uint16_t current = sensorValues[i]; // 配列アクセスを一時変数に保持して再利用
		if (current < min)
		{
			min = current;
			indexR = i;
		}
	}

	// 左右の最小値を比較して全体の最小値インデックスを決定
	if (sensorValues[indexL] < sensorValues[indexR])
	{
		index = indexL;
	}
	else
	{
		index = indexR;
	}

	if(indexL == NUM_SENSORS/2 -1 && indexR == NUM_SENSORS/2){
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
			phi = atanf(nsen1 - nsen2); // 偏角φ計算
		}
		else
		{
			phi = atanf(nsen2 - nsen1); // 偏角φ計算
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
		// angleSensor = angleSensor * degPerRad; // 弧度法に変換

	}
	else if(index == NUM_SENSORS)
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
		if (sensorValues[indexL] < sensorValues[indexR])
		{
			phi = atanf(nsen2 - nsen1); // 偏角φ計算
		}
		else
		{
			phi = atanf(nsen1 - nsen2); // 偏角φ計算
		}
		dthita = phi * thitaScale; // 微小角度dθ計算（定数計算を簡略化）
		if(sensorValues[indexL] < sensorValues[indexR])
		{
			angleSensor = -dthita;
		}
		else
		{
			angleSensor = dthita;
		}
		// angleSensor = phi * thitaScale; // 微小角度dθ計算（定数計算を簡略化）

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
	uint8_t i;
	for (i = 0; i < NUM_SENSORS; i++)
	{
		if (lSensor[i] > lSensorMax[i])
		{
			lSensorMax[i] = lSensor[i];	// 最大値更新
		}
		if (lSensor[i] < lSensorMin[i])
		{
			lSensorMin[i] = lSensor[i];	// 最小値更新
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
