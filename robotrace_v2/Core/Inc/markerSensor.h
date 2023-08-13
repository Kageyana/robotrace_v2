#ifndef MARKERSENSOR_H_
#define MARKERSENSOR_H_

//====================================//
// インクルード
//====================================//
#include "main.h"
//====================================//
// シンボル定義
//====================================//
#define RIGHTMARKER      0x1
#define LEFTMARKER       0x2
#define CROSSLINE       0x3

#define SidesensorL_GPIO_Port SIDEMARKER_L_GPIO_Port
#define SidesensorR_GPIO_Port SIDEMARKER_R_GPIO_Port

#define SidesensorL_Pin SIDEMARKER_L_Pin
#define SidesensorR_Pin SIDEMARKER_R_Pin
//====================================//
// グローバル変数の宣言
//====================================//
extern uint8_t  SGmarker;
extern uint8_t  crossLine;

extern int32_t  distL, distR, distN;
//====================================//
// プロトタイプ宣言
//====================================//
uint8_t getMarkerSensor ( void );
uint8_t checkMarker( void );
void checkGoalMarker(void);
bool checkCrossLine(void);
#endif // MARKERSENSOR_H_
