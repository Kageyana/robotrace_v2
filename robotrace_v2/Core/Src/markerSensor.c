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
	static int32_t	encMarker;

	// クロスライン判定
	if (checkCrossLine() == true) {
		crossLine = 1;
		encMarker = encTotalN;
		checkStart = 0;
		ret = CROSSLINE;
	} else {
		nowMarker = getMarkerSensor();	// マーカーセンサ値を取得

		if (crossLine == 1 && encTotalN - encMarker >= encMM(120)) {
			// クロスライン通過後180mm(ラインセンサからマーカーセンサまで80mm)以内はマーカー検知をしない
			crossLine = 0;
		} else {
			if (nowMarker > 0 && checkStart == 0) {
				existMarker = nowMarker;// 最初に検知したマーカーを記録
				checkStart = 1;			// 読み飛ばし判定開始
				encMarker = encTotalN;	// 距離計測開始
			}
			
			// マーカー判定
			if (checkStart == 1) {
				if (encTotalN - encMarker <= encMM(40)) {
					if (encTotalN - encMarker <= encMM(10)) {
						// 誤検出防止判定
						if(nowMarker == 0) {
							ret = 100;
						}
					}
					// 反対側のマーカーが反応したらクロスライン
					if(nowMarker != existMarker && nowMarker > 0) {
						crossLine = 1;
						ret = CROSSLINE;
					}
				} else {
					ret = existMarker;
				}
				// マーカー検知終了処理
				if (ret > 0) {
					checkStart = 0;
					encMarker = encTotalN;
					if (ret == 100) {
						ret = 0;
					}
				}
			}
		}	
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
	// int32_t valLine;
	// bool ret = false;

	// valLine = lSensorCari[0]+lSensorCari[1]+lSensorCari[10]+lSensorCari[11];

	// if (valLine < 8000) {
	// 	ret = true;
	// }

	// return ret;
}
/////////////////////////////////////////////////////////////
// モジュール名 checkGoalMarker
// 処理概要     クロスラインの読み飛ばし処理を含むマーカー検知
// 引数         なし
// 戻り値       0:マーカなし 0x1:右 0x2:左 0x3:クロスライン
/////////////////////////////////////////////////////////////
void checkGoalMarker (void) {
	// if ( courseMarker == RIGHTMARKER ) {
	// 	if (encRightMarker > encMM(600) ) {	// 2回目以降
	// 		SGmarker++;
	// 		encRightMarker = 0;
	// 	}
	// }
}