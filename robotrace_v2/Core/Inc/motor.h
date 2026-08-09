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
#define MOTOR_COMMAND_NOMINAL_V 7.0f
#define MOTOR_VOLTAGE_CMD_MAX_V 8.4f

#define RREF 2700.0F // 抵抗値[Ω]

#define MOTOR_AD_WINDOW 32
// MOTOR_AD_WINDOW は2の冪である必要があるため、変更時は注意
// ビット演算で2の冪かを静的に検証し、条件を満たさない場合はコンパイルエラー
#if (MOTOR_AD_WINDOW & (MOTOR_AD_WINDOW - 1)) != 0
#error "MOTOR_AD_WINDOW must be a power of two"
#endif
//====================================//
// グローバル変数の宣言
//====================================//
extern int16_t motorpwmL, motorpwmR;
extern float motorVoltageCmdL_V, motorVoltageCmdR_V;
extern float motorCurrentL, motorCurrentR;
extern bool calibrateMotorCurrent;
//====================================//
// プロトタイプ宣言
//====================================//
float motorCommandToVoltage_V(int16_t command);
int16_t motorVoltageToCommand(float voltage_V);
void motorCommandOut(int16_t cmdL, int16_t cmdR);
void motorCommandOutSynth(int16_t tCmd, int16_t sCmd, int16_t yrCmd, int16_t dCmd);
void motorVoltageOut(float voltageL_V, float voltageR_V);
void getMotorAD(uint16_t LAD, uint16_t RAD);
void getMotorCurrent(void);
void calibrationMotorCurrent(void);
void MotorFanPwmOut(int16_t pwm);

#endif // MOTOR_H_
