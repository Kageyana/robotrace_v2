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
#define MOTOR_TIM_CHANNEL_R TIM_CHANNEL_3
#define MOTOR_TIM_CHANNEL_L TIM_CHANNEL_2
#define MOTOR_COUNTERPERIOD MOTOR_TIM_HANDLER.Init.Period

#define PWMOUT_L __HAL_TIM_SET_COMPARE(&MOTOR_TIM_HANDLER, TIM_CHANNEL_2, (int16_t)((float)pwmL / 1000 * MOTOR_COUNTERPERIOD))
#define PWMOUT_R __HAL_TIM_SET_COMPARE(&MOTOR_TIM_HANDLER, TIM_CHANNEL_3, (int16_t)((float)pwmR / 1000 * MOTOR_COUNTERPERIOD))

#define FOWARD_L HAL_GPIO_WritePin(MOTOR_DIR_L_GPIO_Port, MOTOR_DIR_L_Pin, GPIO_PIN_SET)
#define REVERSE_L HAL_GPIO_WritePin(MOTOR_DIR_L_GPIO_Port, MOTOR_DIR_L_Pin, GPIO_PIN_RESET)

#define FOWARD_R HAL_GPIO_WritePin(MOTOR_DIR_R_GPIO_Port, MOTOR_DIR_R_Pin, GPIO_PIN_RESET)
#define REVERSE_R HAL_GPIO_WritePin(MOTOR_DIR_R_GPIO_Port, MOTOR_DIR_R_Pin, GPIO_PIN_SET)

#define HALFSCAL 2070
#define RREF 2700.0F
//====================================//
// グローバル変数の宣言
//====================================//
extern int16_t motorpwmL, motorpwmR;
extern uint16_t motorCurrentValL, motorCurrentValR;
extern float motorCurrentL, motorCurrentR;
//====================================//
// プロトタイプ宣言
//====================================//
void motorPwmOut(int16_t pwmL, int16_t pwmR);
void motorPwmOutSynth(int16_t tPwm, int16_t sPwm, int16_t yrPwm, int16_t dPwm);
void getMotorCurrent(void);

#endif // MOTOR_H_
