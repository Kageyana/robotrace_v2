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

#define VREF_L 1.696F  // 基準出圧
#define VREF_R 1.697F  // 基準出圧
#define RREF_L 1930.0F // 抵抗値
#define RREF_R 1940.0F // 抵抗値

#define MOTOR_AD_WINDOW 64
//====================================//
// グローバル変数の宣言
//====================================//
extern int16_t motorpwmL, motorpwmR;
extern float motorCurrentL, motorCurrentR;
//====================================//
// プロトタイプ宣言
//====================================//
void motorPwmOut(int16_t pwmL, int16_t pwmR);
void motorPwmOutSynth(int16_t tPwm, int16_t sPwm, int16_t yrPwm, int16_t dPwm);
void getMotorAD(uint16_t LAD, uint16_t RAD);
void getMotorCurrent(void);
void MotorFanPwmOut(int16_t pwm);

#endif // MOTOR_H_
