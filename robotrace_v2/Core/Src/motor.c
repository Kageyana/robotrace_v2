//====================================//
// インクルード
//====================================//
#include "motor.h"
//====================================//
// グローバル変数の宣言
//====================================//
int16_t motorpwmL = 0;
int16_t motorpwmR = 0;
uint16_t motorCurrentValL, motorCurrentValR;
float motorCurrentL, motorCurrentR;
/////////////////////////////////////////////////////////////////////
// モジュール名 motorPwmOut
// 処理概要     左右のモータにPWMを出力する
// 引数         pwmL: 左モータへの出力(1~1000) pwmR: 右モータへの出力(1~1000)
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void motorPwmOut(int16_t pwmL, int16_t pwmR)
{
	// 0除算回避
	if (pwmL == 0)
		__HAL_TIM_SET_COMPARE(&MOTOR_TIM_HANDLER, MOTOR_TIM_CH_L, 0);
	if (pwmR == 0)
		__HAL_TIM_SET_COMPARE(&MOTOR_TIM_HANDLER, MOTOR_TIM_CH_R, 0);

	if (pwmL != 0)
	{
		if (pwmL > 0)
		{
			HAL_GPIO_WritePin(MOTOR_DIR_L_GPIO_Port, MOTOR_DIR_L_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(MOTOR_DIR_L_GPIO_Port, MOTOR_DIR_L_Pin, GPIO_PIN_RESET);
		}
		pwmL = abs(pwmL);
		__HAL_TIM_SET_COMPARE(&MOTOR_TIM_HANDLER, MOTOR_TIM_CH_L, (int16_t)((float)pwmL / 1000 * MOTOR_PERIOD));
	}

	if (pwmR != 0)
	{
		if (pwmR > 0)
		{
			HAL_GPIO_WritePin(MOTOR_DIR_R_GPIO_Port, MOTOR_DIR_R_Pin, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(MOTOR_DIR_R_GPIO_Port, MOTOR_DIR_R_Pin, GPIO_PIN_SET);
		}
		pwmR = abs(pwmR);
		__HAL_TIM_SET_COMPARE(&MOTOR_TIM_HANDLER, MOTOR_TIM_CH_R, (int16_t)((float)pwmR / 1000 * MOTOR_PERIOD));
	}
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motorPwmOutSynth
// 処理概要     トレースと速度制御のPID制御量をPWMとしてモータに出力する
// 引数         tPwm: トレースのPID制御量 sPwm: 速度のPID制御量
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void motorPwmOutSynth(int16_t tPwm, int16_t sPwm, int16_t yrPwm, int16_t dPwm)
{
	int16_t overpwm, tracePwm = tPwm;

	if (abs(sPwm + tPwm) > 1000 || abs(sPwm - tPwm) > 1000)
	{
		// ライントレースと速度制御の合計制御量が1000を超えたとき
		overpwm = abs(sPwm) + abs(tPwm) - 1000; // 1000を超えた分の制御量を計算

		// トレースの内輪側から越えた分の制御量を引く 正負はtPwmと同じ
		if (tPwm > 0)
		{
			tracePwm = tPwm - (overpwm * (tPwm / abs(tPwm)));
		}
		else
		{
			tracePwm = tPwm + (overpwm * (tPwm / abs(tPwm)));
		}
	}

	motorpwmR = sPwm - tracePwm - yrPwm + dPwm;
	motorpwmL = sPwm + tracePwm + yrPwm + dPwm;

	motorPwmOut(motorpwmL, motorpwmR);
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 getMotorCurrent
// 処理概要     MP6551のCMピン出力[V]を電流値[A]に変換する
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void getMotorCurrent(void)
{
	float vL, vR;

	// AD値を電圧[V]に変換
	vL = (float)(motorCurrentValL - HALFSCAL) / 4095 * 3.3;
	vR = (float)(motorCurrentValR - HALFSCAL) / 4095 * 3.3;

	motorCurrentL = vL / RREF * 10000.0;
	motorCurrentR = vR / RREF * 10000.0;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 MotorSuctionPwmOut
// 処理概要     吸引モータにPWMを出力する
// 引数         pwm: モータへの出力(1~1000)
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void MotorSuctionPwmOut(int16_t pwm)
{
	// 0除算回避
	if (pwm == 0)
		__HAL_TIM_SET_COMPARE(&MOTOR_TIM_HANDLER, MOTOR_SUCTION_TIM_CH, 0);

	if (pwm != 0)
	{
		pwm = abs(pwm);
		__HAL_TIM_SET_COMPARE(&MOTOR_TIM_HANDLER, MOTOR_SUCTION_TIM_CH, (int16_t)((float)pwm / 1000 * MOTOR_PERIOD));
	}
}