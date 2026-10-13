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
#define LS_ADC_SETTLE_US 120U // 位相切替から取得要求まで[us]
#define LS_ADC_SCAN_MARGIN_US 50U // 11ch走査約34usと開始処理の余裕[us]
#define LS_UPDATE_STALE_MS 3U // 更新停止の診断閾値[ms]。実機計測後に調整する。
#define LS_AVERAGE_MAX_PAIR_AGE_MS 3U // 完成済みペアを平均化に保持できる時間[ms]
#define LS_COUNTERPERIOD htim3.Init.Period * 0.5f
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
extern volatile bool lineSensorState;
extern volatile bool lineSensorPower;
extern volatile bool adcAcquisitionBusy;
extern volatile bool adcAcquisitionFault;
extern volatile uint32_t adcSkippedCount;
extern volatile uint32_t adcStartErrorCount;
extern volatile uint32_t adcLastStartStatus;
extern volatile uint32_t adcLastErrorCode;
extern volatile uint32_t adcTransferErrorCount;
extern volatile uint32_t lineSensorUpdateCount;
extern volatile uint32_t lineSensorUpdateAgeMs;
extern volatile uint32_t lineSensorMaxUpdateAgeMs;
extern volatile bool lineSensorUpdateStale;
extern volatile uint32_t lineSensorStaleCount;
extern volatile uint16_t lineSensorVrefSample;

extern uint16_t lSensorCari[NUM_SENSORS];
extern uint16_t lSensorMax[NUM_SENSORS];	// 各センサの最大値
extern uint16_t lSensorMin[NUM_SENSORS];	// 各センサの最小値
extern uint8_t modeCalLinesensors;
extern bool lineSensorSettingCorrupt;

//====================================//
// プロトタイプ宣言
//====================================//
void powerLineSensors(uint8_t onoff);
void lineSensorPhaseChanged(bool phase);
void startLineSensorAcquisition(void);
void completeLineSensorAcquisition(void);
void failLineSensorAcquisition(void);
void monitorLineSensorUpdate1ms(bool steeringUsesLine);
void getLineSensor(bool phase, const uint16_t *sample, uint32_t epoch);
void getAngleSensor(void);
void finishLineSensorCalibration(void);
void writeLinesenval(void);
void readLinesenval(void);
bool isLineSensorCalibrationValid(void);
bool isLineSensorSettingCorrupt(void);

#endif // LINESENSOR_H_
