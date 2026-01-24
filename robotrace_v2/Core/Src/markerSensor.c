//====================================//
// インクルード
//====================================//
#include "markerSensor.h"
#include "control.h"
#include "encoder.h"
#include <stdint.h>
//====================================//
// グローバル変数の宣言
//====================================//
uint8_t markerSensor = 0;
uint8_t SGmarker = 0;
static int32_t encMarkerL = PALSE_METER/10, encMarkerR = (PALSE_METER/10) + 1;
/////////////////////////////////////////////////////////////////////
// モジュール名 getMarksensor
// 処理概要     マーカーセンサの値を取得
// 引数         なし
// 戻り値       0x1:右センサ反応 0x2:左センサ反応
/////////////////////////////////////////////////////////////////////
void getMarkerSensor(void)
{
	uint8_t ret = 0;
	static uint8_t ron = 1, roff = 1, lon = 1, loff = 1;
	static uint8_t readyMask = 0;
	const uint8_t phase = lineSensorState ? 1U : 0U; // 1: LED on, 0: LED off

	// マーカーセンサ値取得(白:0 黒:1)
	if(phase){
		lon = HAL_GPIO_ReadPin(SidesensorL_GPIO_Port, SidesensorL_Pin);
		ron = HAL_GPIO_ReadPin(SidesensorR_GPIO_Port, SidesensorR_Pin);
	}
	else
	{
		loff = HAL_GPIO_ReadPin(SidesensorL_GPIO_Port, SidesensorL_Pin);
		roff = HAL_GPIO_ReadPin(SidesensorR_GPIO_Port, SidesensorR_Pin);
	}
	readyMask |= (1U << phase);

	if (readyMask == 0x03U)
	{
		uint8_t diffR=0, diffL=0;
		diffR = (roff > ron) ? (roff - ron) : 0U;
		diffL = (loff > lon) ? (loff - lon) : 0U;
		if (diffR == 1)
			ret += RIGHTMARKER;
		if (diffL == 1)
			ret += LEFTMARKER;
		readyMask = 0;
		markerSensor = ret;
	}
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 initMarkerSensor
// 処理概要     マーカーセンサ関連変数初期化
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void initMarkerSensor(void)
{
    markerSensor = 0;
    SGmarker = 0;
    encMarkerL = PALSE_METER/10;
    encMarkerR = (PALSE_METER/10) + 1;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 checkMarker
// 処理概要     クロスラインの読み飛ばし処理を含むマーカー検知
// 引数         なし
// 戻り値       0:マーカなし 0x1:右 0x2:左 0x3:クロスライン
///////////////////////////////////////////////////////////////////////////
uint8_t checkMarker(void)
{
	uint8_t ret = 0;
	static uint8_t checkStart, nowMarker, existMarker;
	static int32_t encMarkerN, nowEncTotalN;
	static int32_t distL, distR;

	nowMarker = markerSensor; // マーカーセンサ値を取得
	nowEncTotalN = encTotalN;

	// 反応があればマーカー幅計測開始
	if (nowMarker > 0 && checkStart == 0)
	{
		existMarker = nowMarker;   // 最初に検知したマーカーを記録
		checkStart = 1;			   // マーカー幅計測開始
		encMarkerN = nowEncTotalN; // 距離計測開始
	}
	if (checkStart == 1)
	{
		if (nowEncTotalN - encMarkerN <= encMM(20))
		{
			// 10mm以内で反応が消えたら誤検出判定
			if (nowMarker == 0 && nowEncTotalN - encMarkerN <= encMM(10))
			{
				existMarker = 0;
				checkStart = 0;
			}
			// クロスラインを検出したら上書き
			if (nowMarker > existMarker)
			{
				existMarker = nowMarker;
			}
		}
		else if (nowEncTotalN - encMarkerN > encMM(20))
		{
			// 20mm以上センサが反応し続けたらマーカーと判定
			// マーカー位置を記録
			if (existMarker == 0x1)
			{
				encMarkerR = nowEncTotalN;
			}
			else if (existMarker == 0x2)
			{
				encMarkerL = nowEncTotalN;
			}
			else if (existMarker == 0x3)
			{
				encMarkerR = nowEncTotalN;
				encMarkerL = nowEncTotalN;
			}
			checkStart = 0;
		}
	}

	// 現在地からマーカー位置までの距離
	distL = nowEncTotalN - encMarkerL;
	distR = nowEncTotalN - encMarkerR;

	// ゴールマーカーを検出してから40~50mm走行後かつカーブマーカーを100mm検出していないとき
	if (distR > encMM(50) && distR <= encMM(60) && distL > encMM(100))
	{
		ret = RIGHTMARKER;
	}
	// カーブマーカーを検出してから20~30mm走行後かつゴールマーカーを40mm検出していないとき
	if (distL > encMM(5) && distL <= encMM(10) && distR > encMM(80))
	{
		ret = LEFTMARKER;
	}

	if (distL > encMM(10) && distL <= encMM(20) && distR == distL)
	{
		ret = CROSSLINE;
	}

	return ret;
}
/////////////////////////////////////////////////////////////
// モジュール名 checkStartGoalMarker
// 処理概要     スタートマーカーとゴールマーカーの検出
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////
void checkStartGoalMarker(void)
{
	if(SGmarker > 0) // スタートマーカー通過後
	{
		if (courseMarker == RIGHTMARKER && encRightMarker > encMM(1000))
		{ // 1000mm以上離れたらゴールマーカー検出可能
			SGmarker++;
			encRightMarker = 0;
		}
	}
	else // スタートマーカー通過前
	{
		if(markerSensor == RIGHTMARKER)
		{
			SGmarker++;
			encRightMarker = 0;
		}
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 powerMarkerSensors
// 処理概要  	マーカーセンサのON/OFF処理
// 引数     	0:OFF 1:ON
// 戻り値    	なし
/////////////////////////////////////////////////////////////////////
void powerMarkerSensors(uint8_t onoff)
{
	if (onoff == 0)
	{
		HAL_GPIO_WritePin(SidemarkerPWR_GPIO_Port, SidemarkerPWR_Pin, GPIO_PIN_RESET);
	}
	else if (onoff == 1)
	{
		HAL_GPIO_WritePin(SidemarkerPWR_GPIO_Port, SidemarkerPWR_Pin, GPIO_PIN_SET);
	}
}