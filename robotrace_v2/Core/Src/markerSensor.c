//====================================//
// インクルード
//====================================//
#include "markerSensor.h"
#include "control.h"
#include "encoder.h"
#include "timer.h"
#include <stdint.h>
//====================================//
// グローバル変数の宣言
//====================================//
uint8_t markerSensor = 0;
uint8_t SGmarker = 0;
volatile uint8_t startMarkerOnsetValid = 0U;
volatile uint8_t goalMarkerOnsetValid = 0U;
volatile int32_t goalMarkerOnset_p = 0;
static int32_t rightMarkerCandidate_p = 0;
static int32_t rightMarkerQualified_p = 0;
static uint8_t rightMarkerQualified = 0U;
static uint8_t startMarkerCandidate = 0U;
static int32_t encMarkerL = PULSE_METER/10, encMarkerR = (PULSE_METER/10) + 1;
static uint8_t markerRon = 1U, markerRoff = 1U, markerLon = 1U, markerLoff = 1U;
static uint8_t markerReadyMask;
/////////////////////////////////////////////////////////////////////
// モジュール名 discardMarkerSensorPendingSamples
// 処理概要     位相不一致時に未完成のマーカー点灯・消灯組を破棄する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void discardMarkerSensorPendingSamples(void)
{
	markerReadyMask = 0U;
	markerSensor = 0U;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 getMarksensor
// 処理概要     マーカーセンサの値を取得
// 引数         phase: ADC開始時に固定したLED位相(0:消灯、1:点灯)
// 戻り値       0x1:右センサ反応 0x2:左センサ反応
/////////////////////////////////////////////////////////////////////
void getMarkerSensor(uint8_t phase)
{
	uint8_t ret = 0;

	// マーカーセンサ値取得(白:0 黒:1)
	if(phase){
		markerLon = HAL_GPIO_ReadPin(SidesensorL_GPIO_Port, SidesensorL_Pin);
		markerRon = HAL_GPIO_ReadPin(SidesensorR_GPIO_Port, SidesensorR_Pin);
	}
	else
	{
		markerLoff = HAL_GPIO_ReadPin(SidesensorL_GPIO_Port, SidesensorL_Pin);
		markerRoff = HAL_GPIO_ReadPin(SidesensorR_GPIO_Port, SidesensorR_Pin);
	}
	markerReadyMask |= (1U << phase);

	if (markerReadyMask == 0x03U)
	{
		uint8_t diffR=0, diffL=0;
		diffR = (markerRoff > markerRon) ? (markerRoff - markerRon) : 0U;
		diffL = (markerLoff > markerLon) ? (markerLoff - markerLon) : 0U;
		if (diffR == 1)
			ret += RIGHTMARKER;
		if (diffL == 1)
			ret += LEFTMARKER;
		markerReadyMask = 0;
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
    discardMarkerSensorPendingSamples();
    markerSensor = 0;
    SGmarker = 0;
    encMarkerL = PULSE_METER/10;
    encMarkerR = (PULSE_METER/10) + 1;
    startMarkerOnsetValid = 0U;
    goalMarkerOnsetValid = 0U;
    goalMarkerOnset_p = 0;
    rightMarkerQualified = 0U;
    rightMarkerCandidate_p = 0;
    rightMarkerQualified_p = 0;
    startMarkerCandidate = 0U;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 markerStartReferenceReset
// 処理概要     スタート検出後の距離原点にマーカー幅検証を同期する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void markerStartReferenceReset(void)
{
    startMarkerCandidate = 1U;
    startMarkerOnsetValid = 0U;
    goalMarkerOnsetValid = 0U;
    goalMarkerOnset_p = 0;
    rightMarkerQualified = 0U;
    rightMarkerCandidate_p = 0;
    rightMarkerQualified_p = 0;
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
		if (nowMarker == RIGHTMARKER && SGmarker > 0U)
		{
			rightMarkerCandidate_p = encTotalOptimal;
		}
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
				rightMarkerQualified_p = rightMarkerCandidate_p;
				rightMarkerQualified = 1U;
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

	// 左右のマーカーを検出してから10~20mm走行後かつ左右走行距離が同じ時
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
		if (startMarkerCandidate != 0U)
		{
			if ((markerSensor == 0U && encTotalN <= encMM(10)) ||
				markerSensor == CROSSLINE)
			{
				startMarkerCandidate = 0U;
			}
			else if (encTotalN > encMM(20))
			{
				startMarkerOnsetValid = 1U;
				startMarkerCandidate = 0U;
			}
		}
		if (courseMarker == RIGHTMARKER && encRightMarker > encMM(1000))
		{ // 1000mm以上離れたらゴールマーカー検出可能
			if (rightMarkerQualified != 0U)
			{
				goalMarkerOnset_p = rightMarkerQualified_p;
				goalMarkerOnsetValid = 1U;
			}
			SGmarker++;
			encRightMarker = 0;
		}
	}
	else // スタートマーカー通過前
	{
		if(markerSensor == RIGHTMARKER)
		{
			Timer_NotifyStartMarkerDetected(encTotalN);
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
