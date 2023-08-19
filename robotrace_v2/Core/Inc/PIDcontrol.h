#ifndef LINETRACE_H_
#define LINETRACE_H_
//====================================//
// インクルード
//====================================//
#include "main.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
//====================================//
// シンボル定義
//====================================//
#define KP1		35
#define KI1		0
#define KD1		150

#define KP2		12
#define KI2		45
#define KD2		0

#define KP3		3
#define KI3		50
#define KD3		50

#define KP4		58
#define KI4		28
#define KD4		0


typedef struct {
    int16_t kp;
    int16_t ki;
    int16_t kd;
    float   Int;
    int16_t pwm;
} pidParam;

//====================================//
// グローバル変数の宣言
//====================================//
extern uint8_t	targetSpeed;	// 目標速度
extern float 	targetAngle;    // 目標角度
extern float    targetAngularVelocity;

extern pidParam lineTraceCtrl;
extern pidParam veloCtrl;
extern pidParam yawRateCtrl;
extern pidParam yawCtrl;
//====================================//
// プロトタイプ宣言
//====================================//
void setTargetSpeed (float speed);
void setTargetAngularVelocity (float angularVelocity);
void setTargetAngle (float angle);
void motorControlTrace( void );
void motorControlSpeed( void );
void motorControlYawRate( void );
void motorControlYaw( void );

#endif // LINETRACE_H_
