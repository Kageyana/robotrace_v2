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
#define LS_AVERAGE_SAMPLES 2U	 // ラインセンサ平均化サンプル数
#define LS_COUNTERPERIOD htim3.Init.Period * 0.5
#define LS_TIMER htim3
#define LS_CHANNEL TIM_CHANNEL_3
#define PSC htim3.Init.Prescaler
#define TIMER_TICK 90 // タイマクロック周波数[MHz]

// ファイル名
#define FILENAME_LS_VAL "lsval"
//====================================//
// グローバル変数の宣言
//====================================//
extern uint16_t lSensor[NUM_SENSORS];
extern float angleSensor;
extern bool lineSensorState;
extern bool lineSensorPower;

extern uint16_t lSensorCari[NUM_SENSORS];
extern uint16_t lSensorMax[NUM_SENSORS];	// 各センサの最大値
extern uint16_t lSensorMin[NUM_SENSORS];	// 各センサの最小値
extern uint8_t modeCalLinesensors;
extern bool lineSensorSettingCorrupt;

//====================================//
// プロトタイプ宣言
//====================================//
void powerLineSensors(uint8_t onoff);
void delayLineSensorConversionStart(uint32_t us);
void getLineSensor(void);
void getAngleSensor(void);
void calibrationLinesensor(void);
void writeLinesenval(void);
void readLinesenval(void);
bool isLineSensorCalibrationValid(void);
bool isLineSensorSettingCorrupt(void);

#endif // LINESENSOR_H_
