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
//====================================//
// グローバル変数の宣言
//====================================//
extern uint16_t batteryAD;
extern uint8_t batteryLevel;
//====================================//
// プロトタイプ宣言
//====================================//
void getBatteryAD(uint16_t ad);
void SchmittBatery(void);
void showBattery(void);
void showBatMark(void);
#endif // BATTERY_H_
