#ifndef EMERGENCYSTOP_H_
#define EMERGENCYSTOP_H_
//====================================//
// インクルード
//====================================//
#include "main.h"
//====================================//
// シンボル定義
//====================================//
// 緊急停止関連
#define STOP_ANGLE_X            1
#define STOP_ANGLE_Y            2
#define STOP_ENCODER_STOP       3
#define STOP_LINESENSOR         4
#define STOP_OVERSPEED          5

#define STOP_COUNT_ENCODER_STOP	200		// エンコーダ停止
#define STOP_COUNT_ANGLE_X	    100		// X方向の角速度変化
#define STOP_COUNT_ANGLE_Y	    100		// Y方向の角速度変化
#define STOP_COUNT_TIME		    1000	// 時間停止
#define STOP_COUNT_LINESENSOR	200	    // ラインセンサが外れる
#define STOP_COUNT_OVERSPEED	200	    // 目標速度を大きく超えている
//====================================//
// グローバル変数の宣言
//====================================//
extern uint8_t emcStop;
//====================================//
// プロトタイプ宣言
//====================================//
bool cntEmcStopAngleX(void);
bool cntEmcStopAngleY(void);
bool cntEmcStopEncStop(void);
bool cntEmcStopLineSensor(void);
bool judgeOverSpeed(void);
#endif // EMERGENCYSTOP_H_
