//====================================//
// インクルード
//====================================//
#include "emergencyStop.h"
//====================================//
// グローバル変数の宣言
//====================================//
uint8_t emcStop = 0;
static uint16_t cntAngleX = 0;
static uint16_t cntAngleY = 0;
static uint16_t cntEncStop = 0;
static uint16_t cntLineSensor = 0;
static uint16_t cntOverSpeed = 0;
/////////////////////////////////////////////////////////////////////
// モジュール名 cntEmcStopAngleX
// 処理概要     緊急停止要因のカウント x軸角速度異常
// 引数         なし
// 戻り値       true:緊急停止 false:異常なし
/////////////////////////////////////////////////////////////////////
bool cntEmcStopAngleX(void)
{
	// 緊急停止条件
	if (fabs(BMI088val.gyro.x) > 2.0f)
	{
		cntAngleX++;
	}
	else
	{
		cntAngleX = 0;
	}

	// 停止条件継続タイマ
	if (cntAngleX > STOP_COUNT_ANGLE_X)
	{
		return true;
	}
	else
	{
		return false;
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 cntEmcStopAngleY
// 処理概要     緊急停止要因のカウント y軸角速度異常
// 引数         なし
// 戻り値       true:緊急停止 false:異常なし
/////////////////////////////////////////////////////////////////////
bool cntEmcStopAngleY(void)
{
	// 緊急停止条件
	if (fabs(BMI088val.gyro.y) > 2.0f)
	{
		cntAngleY++;
	}
	else
	{
		cntAngleY = 0;
	}

	// 停止条件継続タイマ
	if (cntAngleY > STOP_COUNT_ANGLE_Y)
	{
		return true;
	}
	else
	{
		return false;
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 cntEmcStopEncStop
// 処理概要     緊急停止要因のカウント エンコーダストップ
// 引数         なし
// 戻り値       true:緊急停止 false:異常なし
/////////////////////////////////////////////////////////////////////
bool cntEmcStopEncStop(void)
{
	// 緊急停止条件
	if (abs(encCurrentN) < 20)
	{
		cntEncStop++;
	}
	else
	{
		cntEncStop = 0;
	}

	// 停止条件継続タイマ
	if (cntEncStop > STOP_COUNT_ENCODER_STOP)
	{
		return true;
	}
	else
	{
		return false;
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 cntEmcStopLineSensorBright
// 処理概要     緊急停止要因のカウント ラインセンサ反応あり
// 引数         なし
// 戻り値       true:緊急停止 false:異常なし
/////////////////////////////////////////////////////////////////////
bool cntEmcStopLineSensorBright(void)
{
	// 緊急停止条件
	if (lSensorCari[3] > STOP_TH_LINE_SENSOR_BRIGHT
		&& lSensorCari[4]> STOP_TH_LINE_SENSOR_BRIGHT
		&& lSensorCari[5]> STOP_TH_LINE_SENSOR_BRIGHT
		&& lSensorCari[6] > STOP_TH_LINE_SENSOR_BRIGHT)
	{
		cntLineSensor++;
	}
	else
	{
		cntLineSensor = 0;
	}

	// 停止条件継続タイマ
	if (cntLineSensor > STOP_COUNT_LINESENSOR)
	{
		return true;
	}
	else
	{
		return false;
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 cntEmcStopLineSensorUnbright
// 処理概要     緊急停止要因のカウント ラインセンサ反応無し
// 引数         なし
// 戻り値       true:緊急停止 false:異常なし
/////////////////////////////////////////////////////////////////////
bool cntEmcStopLineSensorUnbright(void)
{
	// 緊急停止条件
	if (lSensorCari[3] < STOP_TH_LINE_SENSOR_UNBRIGHT
		&& lSensorCari[4] < STOP_TH_LINE_SENSOR_UNBRIGHT
		&& lSensorCari[5] < STOP_TH_LINE_SENSOR_UNBRIGHT
		&& lSensorCari[6] < STOP_TH_LINE_SENSOR_UNBRIGHT)
	{
		cntLineSensor++;
	}
	else
	{
		cntLineSensor = 0;
	}

	// 停止条件継続タイマ
	if (cntLineSensor > STOP_COUNT_LINESENSOR)
	{
		return true;
	}
	else
	{
		return false;
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 judgeOverSpeed
// 処理概要     緊急停止要因のカウント ラインセンサ飽和
// 引数         なし
// 戻り値       true:緊急停止 false:異常なし
/////////////////////////////////////////////////////////////////////
bool judgeOverSpeed(void)
{
	// 緊急停止条件
	if (encCurrentN > (uint32_t)(targetSpeed * 2))
	{
		cntOverSpeed++;
	}
	else
	{
		cntOverSpeed = 0;
	}

	// 停止条件継続タイマ
	if (cntOverSpeed > STOP_COUNT_OVERSPEED)
	{
		return true;
	}
	else
	{
		return false;
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 resetEmcStop
// 処理概要     緊急停止要因のカウントリセット
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void resetEmcStop(void)
{
	emcStop = 0;
	cntAngleX = 0;
	cntAngleY = 0;
	cntEncStop = 0;
	cntLineSensor = 0;
	cntOverSpeed = 0;
}