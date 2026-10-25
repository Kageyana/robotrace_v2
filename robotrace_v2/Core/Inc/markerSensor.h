#ifndef MARKERSENSOR_H_
#define MARKERSENSOR_H_

//====================================//
// インクルード
//====================================//
#include "main.h"
#include <stdbool.h>
#include <stdint.h>
//====================================//
// シンボル定義
//====================================//
#define RIGHTMARKER      0x1
#define LEFTMARKER       0x2
#define CROSSLINE       0x3
#define NUM_MARKER_SENSORS 2

#define SidesensorL_GPIO_Port SIDEMARKER_L_GPIO_Port
#define SidesensorR_GPIO_Port SIDEMARKER_R_GPIO_Port

#define SidesensorL_Pin SIDEMARKER_L_Pin
#define SidesensorR_Pin SIDEMARKER_R_Pin
//====================================//
// グローバル変数の宣言、定義
//====================================//
extern uint8_t  markerSensor;
extern uint8_t  SGmarker;
extern volatile uint8_t goalMarkerOnsetValid;
extern volatile int32_t goalMarkerOnset_p;
//====================================//
// プロトタイプ宣言
//====================================//
void getMarkerSensor(bool phase, uint8_t left, uint8_t right, uint32_t epoch);
void initMarkerSensor(void);
uint8_t checkMarker(void);
void checkStartGoalMarker(void);
bool markerGetGoalOnsetPulse(int32_t *pulse);
void powerMarkerSensors(uint8_t onoff);
#endif // MARKERSENSOR_H_
