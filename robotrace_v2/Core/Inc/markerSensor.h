#ifndef MARKERSENSOR_H_
#define MARKERSENSOR_H_

//====================================//
// インクルード
//====================================//
#include "main.h"
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
// グローバル変数の宣言
//====================================//
extern uint8_t  markerSensor;
extern uint8_t  SGmarker; // スタート検出で1、以後は右マーカー通過ごとに加算
extern volatile uint8_t startMarkerOnsetValid;
extern volatile uint8_t goalMarkerOnsetValid;
extern volatile int32_t goalMarkerOnset_p;
void markerStartReferenceReset(void);
//====================================//
// プロトタイプ宣言
//====================================//
void getMarkerSensor(uint8_t phase);
void discardMarkerSensorPendingSamples(void);
void initMarkerSensor(void);
uint8_t checkMarker(void);
void checkStartGoalMarker(void);
void powerMarkerSensors(uint8_t onoff);
#endif // MARKERSENSOR_H_
