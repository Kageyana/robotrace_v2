#ifndef EMERGENCYSTOP_H_
#define EMERGENCYSTOP_H_
//====================================//
// インクルード
//====================================//
#include "main.h"
#include "lineSensor.h"
//====================================//
// シンボル定義
//====================================//
// 緊急停止関連
#define STOP_ANGLE_X            1
#define STOP_ANGLE_Y            2
#define STOP_ENCODER_STOP       3
#define STOP_LINESENSOR_BRIGHT		4
#define STOP_LINESENSOR_UNBRIGHT	5
#define STOP_OVERSPEED          6
#define STOP_LOCALIZATION       7

#define STOP_COUNT_ENCODER_STOP	200		// エンコーダ停止
#define STOP_COUNT_ANGLE_X	    100		// X方向の角速度変化
#define STOP_COUNT_ANGLE_Y	    100		// Y方向の角速度変化
#define STOP_COUNT_TIME		    1000	// 時間停止
#define STOP_COUNT_LINESENSOR	150	    // 中央4センサーの明部継続判定[ms]。1ms周期、既存の>判定を使用する。
#define STOP_COUNT_OVERSPEED	200	    // 目標速度を大きく超えている

//====================================//
// 調整パラメータ
//====================================//
#define STOP_LINE_LOST_TIME_MS 20U // 全10センサーの暗部が連続したときの停止時間[ms]。1ms周期で判定する。
#define STOP_TH_LINE_SENSOR_BRIGHT	1000	// ラインセンサ停止閾値
#define STOP_TH_LINE_SENSOR_UNBRIGHT 100U // 完全逸脱とする正規化値の上限（未満）[0～4095]。全10個へ適用する。
//====================================//
// グローバル変数の宣言、定義
//====================================//
extern uint8_t emcStop;
//====================================//
// プロトタイプ宣言
//====================================//
bool cntEmcStopAngleX(void);
bool cntEmcStopAngleY(void);
bool cntEmcStopEncStop(void);
bool cntEmcStopLineSensorBright(void);
bool cntEmcStopLineSensorUnbright(void);
bool judgeOverSpeed(void);
void resetEmcStop(void);
#endif // EMERGENCYSTOP_H_
