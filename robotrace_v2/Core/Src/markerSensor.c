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
	static int32_t	encMarkerL=0, encMarkerR=1, encMarkerN, nowEncTotalN;
	static int32_t  distL, distR, distN;


	nowMarker = getMarkerSensor();	// マーカーセンサ値を取得
	nowEncTotalN = encTotalN;

	// 反応があればマーカー幅計測開始
	if (nowMarker > 0 && checkStart == 0) {
		existMarker = nowMarker;// 最初に検知したマーカーを記録
		checkStart = 1;			// マーカー幅計測開始
		encMarkerN = nowEncTotalN;	// 距離計測開始
	}
	if (checkStart == 1) {
		if (nowEncTotalN - encMarkerN <= encMM(10)) {
			// 10mm以内で反応が消えたら誤検出判定
			if (nowMarker == 0) {
				existMarker = 0;
				checkStart = 0;
			}
			if (nowMarker != existMarker) {
				existMarker = nowMarker;
			}
		} else if(nowEncTotalN - encMarkerN > encMM(10)) {
			// 10mm以上センサが反応し続けたらマーカーと判定
			// マーカー位置を記録
			if(existMarker == 0x1) {
				encMarkerR = nowEncTotalN;
			} else if(existMarker == 0x2) {
				encMarkerL = nowEncTotalN;
			} else if (existMarker == 0x3) {
				encMarkerR = nowEncTotalN;
				encMarkerL = nowEncTotalN;
			}
			checkStart = 0;
		}
	}

	// 現在地からマーカー位置までの距離
	distL = nowEncTotalN - encMarkerL;
	distR = nowEncTotalN - encMarkerR;

	// ゴールマーカーを検出してから40~50mm走行後かつカーブマーカーを140mm検出していないとき
	if (distR > encMM(50) && distR <= encMM(60) && distL > encMM(140)) {
		ret = RIGHTMARKER;
	}
	// カーブマーカーを検出してから20~30mm走行後かつゴールマーカーを40mm検出していないとき
	if (distL > encMM(10) && distL <= encMM(20) && distR > encMM(40)) {
		ret = LEFTMARKER;
	}

	if (distL > encMM(10) && distL <= encMM(20) && distR == distL) {
		ret = CROSSLINE;
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