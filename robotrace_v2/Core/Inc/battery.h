#ifndef BATTERY_H_
#define BATTERY_H_
//====================================//
// インクルード
//====================================//
#include "main.h"
//====================================//
// シンボル定義
//====================================//
#define BAT_LV_NONE 100
#define BAT_LV_0 0
#define BAT_LV_1 1
#define BAT_LV_2 2
#define BAT_LV_3 3

#define VBAT_R1 2400.0f			// 抵抗R1の値[Ω]
#define VBAT_R2 620.0f			// 抵抗R2の値[Ω]
#define MCU_VOLTAGE 3.3f	// MCUの基準電圧
#define ADC_MAX 4096.0f		// ADCの分解能
#define AD_VBAT_DIVIDER ((VBAT_R1 + VBAT_R2) / VBAT_R2) // 分圧比
#define AD2VOLTAGE(ad) ((float)(ad)*adcVref / ADC_MAX * AD_VBAT_DIVIDER) // 電圧に変換
//====================================//
// グローバル変数の宣言
//====================================//
extern uint16_t batteryAD;		// バッテリ残量(AD値)
extern uint8_t batteryLevel;	// バッテリ残量レベル
extern float adcVref;			// ADC基準電圧[V]
//====================================//
// プロトタイプ宣言
//====================================//
void getBatteryAD(uint16_t ad);
void SchmittBatery(void);
void showBattery(void);
void showBatMark(void);
void getVref(void);
#endif // BATTERY_H_
