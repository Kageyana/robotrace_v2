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
#define LINE_SENSOR_ACQUISITION_POLICY_VERSION 2U // 取得IRQ優先と制御1ms中の公開フレーム固定を識別する版。
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
// グローバル変数の宣言、定義
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
#if defined(ROBOTRACE_ISR_TIMING) && ROBOTRACE_ISR_TIMING
// 0/予約1/2/3/4/5は走行モード、6は校正・開始待ち・停止等。
enum { LINE_TIMING_OTHER_SCOPE = 6, LINE_TIMING_SCOPE_COUNT = 7, LINE_TIMING_GAP_BINS = 9 };
typedef struct
{
	uint32_t samples, poweredSamples, validSamples;
	uint32_t updateEvents, skippedRequests, staleSamples;
	uint32_t maxAgeMs, maxGapMs, maxGapTickMs;
	uint32_t maxGapPattern, maxGapCalibration;
	uint32_t gapHistogram[LINE_TIMING_GAP_BINS]; // 0～7msと8ms以上。制御周期から観測した更新間隔。
} LineSensorTimingStats;
extern volatile LineSensorTimingStats lineSensorTimingStats[LINE_TIMING_SCOPE_COUNT];
#endif

extern uint16_t lSensorCari[NUM_SENSORS];
extern uint16_t lSensorMax[NUM_SENSORS];	// 各センサの最大値
extern uint16_t lSensorMin[NUM_SENSORS];	// 各センサの最小値
extern uint8_t modeCalLinesensors;
extern bool lineSensorSettingCorrupt;

//====================================//
// プロトタイプ宣言
//====================================//
void powerLineSensors(uint8_t onoff);
void beginLineSensorControlFrame1ms(void);
void endLineSensorControlFrame1ms(void);
void lineSensorPhaseChanged(bool phase);
void startLineSensorAcquisition(void);
void completeLineSensorAcquisition(void);
void failLineSensorAcquisition(void);
void monitorLineSensorUpdate1ms(bool steeringUsesLine);
#if defined(ROBOTRACE_ISR_TIMING) && ROBOTRACE_ISR_TIMING
void recordLineSensorTiming1ms(uint8_t mode, uint8_t tracePattern, bool running);
#endif
void getLineSensor(bool phase, const uint16_t *sample, uint32_t epoch);
void getAngleSensor(void);
void finishLineSensorCalibration(void);
void writeLinesenval(void);
void readLinesenval(void);
bool isLineSensorCalibrationValid(void);

#endif // LINESENSOR_H_
