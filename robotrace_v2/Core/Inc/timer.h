#ifndef TIMER_H_
#define TIMER_H_

//====================================//
// インクルード
//====================================//
#include "main.h"
//====================================//
// シンボル定義
//====================================//

//====================================//
// グローバル変数の宣言
//====================================//
extern float bootTime;
extern int16_t targetSpeedLLog;
extern int16_t targetSpeedRLog;
extern uint8_t speedTargetClipLog;
//====================================//
// プロトタイプ宣言
//====================================//
void Interrupt1ms(void);
void Interrupt500us(void);
void Interrupt300ns(void);
void logWriteTask(void);

#endif // TIMER_H_
