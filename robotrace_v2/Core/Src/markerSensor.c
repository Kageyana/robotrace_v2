//====================================//
// インクルード
//====================================//
#include "markerSensor.h"
//====================================//
// グローバル変数の宣言
//====================================//
uint8_t     SGmarker = 0;
uint8_t  	crossLine = 0;

/////////////////////////////////////////////////////////////////////
// モジュール名 getMarksensor
// 処理概要     マーカーセンサの値を取得
// 引数         なし
// 戻り値       0x1:右センサ反応 0x2:左センサ反応
/////////////////////////////////////////////////////////////////////
uint8_t getMarkerSensor ( void ) {
	uint8_t r=1, l=1, ret=0;

	l = HAL_GPIO_ReadPin(SidesensorL_GPIO_Port,SidesensorL_Pin);
	r = HAL_GPIO_ReadPin(SidesensorR_GPIO_Port,SidesensorR_Pin);

	if (r == 0) ret += RIGHTMARKER;
	if (l == 0) ret += LEFTMARKER;

	return ret;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 checkMarker
// 処理概要     クロスラインの読み飛ばし処理を含むマーカー検知
// 引数         なし
// 戻り値       0:マーカなし 0x1:右 0x2:左 0x3:クロスライン
///////////////////////////////////////////////////////////////////////////
uint8_t checkMarker( void ) {
	uint8_t ret = 0;
	static uint8_t	checkStart, nowMarker, existMarker;
	static int32_t	encMarkerL, encMarkerR, encMarkerN;
	static int32_t  distL, distR, distN;


	nowMarker = getMarkerSensor();	// マーカーセンサ値を取得

	// 反応があればマーカー幅計測開始
	if (nowMarker > 0 && checkStart == 0) {
		existMarker = nowMarker;// 最初に検知したマーカーを記録
		checkStart = 1;			// 読み飛ばし判定開始
		encMarkerN = encTotalN;	// 距離計測開始
	}
	if (checkStart == 1) {
		if (encTotalN - encMarkerN <= encMM(10)) {
			// 10mm以内で反応が消えたら誤検出判定
			if (nowMarker == 0) {
				existMarker = 0;
				checkStart = 0;
			}
			if (nowMarker != existMarker) {
				existMarker = nowMarker;
			}
		} else if(encTotalN - encMarkerN > encMM(10)) {
			// マーカーセンサが反応した位置
			if(existMarker == 0x1) {
				encMarkerR = encTotalN;
			} else if(existMarker == 0x2) {
				encMarkerL = encTotalN;
			} else if (existMarker == 0x3) {
				encMarkerR = encTotalN;
				encMarkerL = encTotalN;
			}
			checkStart = 0;
		}
	}

	// 反応してからの距離
	distL = encTotalN - encMarkerL;
	distR = encTotalN - encMarkerR;

	if (distR > encMM(40) && distR <= encMM(50) && distL > encMM(140)) {
		ret = RIGHTMARKER;
	}
	
	return ret;
}
/////////////////////////////////////////////////////////////
// モジュール名 checkCrossLine
// 処理概要  	ラインセンサのAD値からクロスラインを判定する
// 引数     	なし
// 戻り値    	なし
/////////////////////////////////////////////////////////////
bool checkCrossLine(void) {
	int32_t valLine;
	bool ret = false;

	valLine = lSensorCari[0]+lSensorCari[1]+lSensorCari[8]+lSensorCari[9];

	if (valLine < 4000) {
		ret = true;
	}

	return ret;
}
/////////////////////////////////////////////////////////////
// モジュール名 checkGoalMarker
// 処理概要     クロスラインの読み飛ばし処理を含むマーカー検知
// 引数         なし
// 戻り値       0:マーカなし 0x1:右 0x2:左 0x3:クロスライン
/////////////////////////////////////////////////////////////
void checkGoalMarker (void) {
	if ( courseMarker == RIGHTMARKER ) {
		if (encRightMarker > encMM(600) ) {	// 2回目以降
			SGmarker++;
			encRightMarker = 0;
		}
	}
}