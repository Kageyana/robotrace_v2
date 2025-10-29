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
/////////////////////////////////////////////////////////////////////
// モジュール名 getLineSensor
// 処理概要  	ラインセンサのAD値を取得し、平均値を計算する
// 引数     	なし
// 戻り値    	なし
/////////////////////////////////////////////////////////////////////
void getLineSensor(void)
{
	uint8_t i;
	uint16_t s;
	uint32_t ii;
	static uint16_t cntls = 0; // ラインセンサの立ち上がりエッジ積算回数カウント用

	cntls++;
	for (i = 0; i < NUM_SENSORS; i++)
	{
		lSensorInt[i] += analogVal1[i];
		if (cntls > 16)
		{
			lSensor[i] = lSensorInt[i] >> 4; // 平均値算出
			lSensorInt[i] = 0;				 // 積算値リセット
			// 最大値・最小値を更新
			if (modeCalLinesensors == 1)
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
			// 正規化計算
			uint16_t range = lSensorMax[i] - lSensorMin[i];
			if (range != 0)
			{
				lSensorCari[i] = (uint16_t)((lSensor[i] - lSensorMin[i]) * BASEVAL / range);
			}
			else
			{
				lSensorCari[i] = 0;	// 分母ゼロ対策
			}
		}
	}
	if (cntls > 16)
		cntls = 0;
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
	uint8_t lowReflectCount = 0;

	min = 1000;
	index = 0;
	for (i = 0; i < NUM_SENSORS; i++)
	{
		if (lSensor[i] < min)
		{
			min = lSensor[i];
			index = i;
		}
		if (lSensorCari[i] < TRACE_CROSSLINE_LOW_REFLECT_THRESHOLD)
		{
			lowReflectCount++;
		}
	}

	if (lowReflectCount >= TRACE_CROSSLINE_MIN_COUNT)
	{
		// クロスライン通過中は角度更新を停止して急激な姿勢変化を防止
		return;
	}

	if (index >= 0 && index <= NUM_SENSORS - 1)
	{ // 両端のセンサが白線の上にあるときは無視
		if (index == 0 || index >= NUM_SENSORS - 1)
		{
			// クロスライン通過時など端のセンサが最小となった場合は角度計算を行わず、前回値を保持して姿勢乱れを防止
			return;
		}
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
