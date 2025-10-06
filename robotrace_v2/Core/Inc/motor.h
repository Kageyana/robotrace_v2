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
// MOTOR_AD_WINDOW は2の冪である必要があるため、変更時は注意
// ビット演算で2の冪かを静的に検証し、条件を満たさない場合はコンパイルエラー
#if (MOTOR_AD_WINDOW & (MOTOR_AD_WINDOW - 1)) != 0
#error "MOTOR_AD_WINDOW must be a power of two"
#endif
//====================================//
// グローバル変数の宣言
//====================================//
extern int16_t motorpwmL, motorpwmR;
extern float motorCurrentL, motorCurrentR;
//====================================//
// プロトタイプ宣言
//====================================//
void motorPwmOut(int16_t pwmL, int16_t pwmR);
void motorPwmOutSynth(float dutyL, float dutyR);
void motorPwmOutCompose(int16_t tPwm, int16_t sPwm, int16_t yrPwm, int16_t dPwm);
void getMotorAD(uint16_t LAD, uint16_t RAD);
void getMotorCurrent(void);
void MotorFanPwmOut(int16_t pwm);

#endif // MOTOR_H_
