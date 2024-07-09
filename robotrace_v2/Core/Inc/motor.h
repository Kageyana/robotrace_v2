#ifndef MOTOR_H_
#define MOTOR_H_

//====================================//
// インクルード
//====================================//
#include "main.h"
//====================================//
// シンボル定義
//====================================//
#define MOTOR_TIM_HANDLER htim2
#define MOTOR_PERIOD MOTOR_TIM_HANDLER.Init.Period
#define MOTOR_TIM_CH_L TIM_CHANNEL_2
#define MOTOR_TIM_CH_R TIM_CHANNEL_3
#define MOTOR_SUCTION_TIM_CH TIM_CHANNEL_4

#define HALFSCAL 2070
#define RREF 2700.0F
//====================================//
// グローバル変数の宣言
//====================================//
extern int16_t motorpwmL, motorpwmR;
extern uint16_t motorCurrentADL, motorCurrentADR;
extern float motorCurrentL, motorCurrentR;
//====================================//
// プロトタイプ宣言
//====================================//
void motorPwmOut(int16_t pwmL, int16_t pwmR);
void motorPwmOutSynth(int16_t tPwm, int16_t sPwm, int16_t yrPwm, int16_t dPwm);
void getMotorCurrent(void);
void MotorSuctionPwmOut(int16_t pwm);

#endif // MOTOR_H_
