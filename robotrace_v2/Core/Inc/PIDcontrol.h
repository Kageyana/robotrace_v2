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
#define KP1		37
#define KI1		0
#define KD1		170

#define KP2		20
#define KI2		80
#define KD2		0

#define KP3		38
#define KI3		100
#define KD3		8

#define KP4		5
#define KI4		1
#define KD4		5

#define KP5		100
#define KI5		1
#define KD5		5

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
extern float    targetAngularVelocity;  // 目標角速度
extern uint16_t targetDist;		        // 目標X座標

extern pidParam lineTraceCtrl;
extern pidParam veloCtrl;
extern pidParam yawRateCtrl;
extern pidParam yawCtrl;
extern pidParam distCtrl;
//====================================//
// プロトタイプ宣言
//====================================//
void setTargetSpeed (float speed);
void setTargetAngularVelocity (float angularVelocity);
void setTargetAngle (float angle);
void setTargetDist (float dist);
void motorControlTrace(void);
void motorControlSpeed(void);
void motorControlYawRate(void);
void motorControlYaw(void);
void motorControldist(void);

#endif // LINETRACE_H_
