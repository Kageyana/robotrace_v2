#ifndef LINESENSOR_H_
#define LINESENSOR_H_

//====================================//
// インクルード
//====================================//
#include "main.h"
//====================================//
// シンボル定義
//====================================//
#define NUM_SENSORS 10
#define THITA_SENSOR 11.0F // ラインセンサの間隔(角度)
#define BASEVAL 4095.0F
#define LS_COUNTERPERIOD htim3.Init.Period
#define LS_TIMER htim3
#define LS_CHANNEL TIM_CHANNEL_3
#define TRACE_CROSSLINE_LOW_REFLECT_THRESHOLD	100	// クロスライン判定用の正規化値しきい値
#define TRACE_CROSSLINE_MIN_COUNT		5	// クロスライン検出の開始条件とする低反射センサ数
#define TRACE_CROSSLINE_RELEASE_COUNT	3	// クロスライン解除条件とする低反射センサ数

// ファイル名
#define FILENAME_LS_VAL "lsval"
//====================================//
// グローバル変数の宣言
//====================================//
extern uint16_t lSensor[NUM_SENSORS];
extern float angleSensor;
extern bool lineSensorState;

extern uint16_t lSensorCari[NUM_SENSORS];
extern uint16_t lSensorMax[NUM_SENSORS];	// 各センサの最大値
extern uint16_t lSensorMin[NUM_SENSORS];	// 各センサの最小値
extern uint8_t modeCalLinesensors;

//====================================//
// プロトタイプ宣言
//====================================//
void powerLineSensors(uint8_t onoff);
void getLineSensor(void);
void getAngleSensor(void);
void calibrationLinesensor(void);
void writeLinesenval(void);
void readLinesenval(void);

#endif // LINESENSOR_H_
