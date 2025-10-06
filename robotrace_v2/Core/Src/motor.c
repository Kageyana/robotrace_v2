//====================================//
// インクルード
//====================================//
#include "motor.h"
#include <math.h>

//====================================//
// グローバル変数の宣言
//====================================//
int16_t motorpwmL = 0;
int16_t motorpwmR = 0;
uint16_t motorCurrentADL, motorCurrentADR;
uint16_t motorCurrentADLInt[MOTOR_AD_WINDOW], motorCurrentADRInt[MOTOR_AD_WINDOW];
uint32_t cntMotorAD = 0;
float motorCurrentL, motorCurrentR;
/////////////////////////////////////////////////////////////////////
// モジュール名 motorPwmOut
// 処理概要     左右のモータにPWMを出力する
// 引数         pwmL: 左モータへの出力(1~1000) pwmR: 右モータへの出力(1~1000)
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void motorPwmOut(int16_t pwmL, int16_t pwmR)
{
        // PWM が 0 のとき比較レジスタを 0 に設定
        if (pwmL == 0)
                __HAL_TIM_SET_COMPARE(&MOTOR_TIM_HANDLER, MOTOR_TIM_CH_L, 0); // 左モータ出力を停止
        if (pwmR == 0)
                __HAL_TIM_SET_COMPARE(&MOTOR_TIM_HANDLER, MOTOR_TIM_CH_R, 0); // 右モータ出力を停止

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
// 処理概要     正規化DutyからPWM値を合成してモータに出力する
// 引数         dutyL: 左モータのDuty[-1.0〜1.0]
//              dutyR: 右モータのDuty[-1.0〜1.0]
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void motorPwmOutSynth(float dutyL, float dutyR)
{
	// 入力がNaNになった場合は安全側で停止させる
	if (!isfinite(dutyL)) {
		dutyL = 0.0f;
	}
	if (!isfinite(dutyR)) {
		dutyR = 0.0f;
	}
	if (dutyL > 1.0f) {
		dutyL = 1.0f;
	} else if (dutyL < -1.0f) {
		dutyL = -1.0f;
	}
	if (dutyR > 1.0f) {
		dutyR = 1.0f;
	} else if (dutyR < -1.0f) {
		dutyR = -1.0f;
	}
	motorpwmL = (int16_t)(dutyL * 1000.0f);
	motorpwmR = (int16_t)(dutyR * 1000.0f);
	if (motorpwmL > 1000) {
		motorpwmL = 1000;
	} else if (motorpwmL < -1000) {
		motorpwmL = -1000;
	}
	if (motorpwmR > 1000) {
		motorpwmR = 1000;
	} else if (motorpwmR < -1000) {
		motorpwmR = -1000;
	}
	motorPwmOut(motorpwmL, motorpwmR);
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motorPwmOutCompose
// 処理概要     トレースと速度制御などの制御量を合成してPWMを算出する
// 引数         tPwm: トレースのPID制御量 sPwm: 速度のPID制御量
//              yrPwm: ヨーレートのPID制御量 dPwm : 距離のPID制御量
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void motorPwmOutCompose(int16_t tPwm, int16_t sPwm, int16_t yrPwm, int16_t dPwm)
{
	int16_t overpwm, tracePwm = tPwm;

	if (abs(sPwm + tPwm) > 1000 || abs(sPwm - tPwm) > 1000)
	{
		// ライントレースと速度制御の合計制御量が1000を超えたとき
		overpwm = abs(sPwm) + abs(tPwm) - 1000;
		// トレースの内輪側から越えた分の制御量を引く 正負はtPwmと同じ
		if (tPwm != 0)
		{
			tracePwm = tPwm - (overpwm * (tPwm / abs(tPwm)));
		}
		else
		{
			tracePwm = tPwm;
		}
	}

	motorpwmR = sPwm - tracePwm - yrPwm + dPwm;
	if (motorpwmR > 1000)
		motorpwmR = 1000;
	else if (motorpwmR < -1000)
		motorpwmR = -1000;
	motorpwmL = sPwm + tracePwm + yrPwm + dPwm;
	if (motorpwmL > 1000)
		motorpwmL = 1000;
	else if (motorpwmL < -1000)
		motorpwmL = -1000;

	motorPwmOutSynth((float)motorpwmL / 1000.0f, (float)motorpwmR / 1000.0f);
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 getMotorAD
// 処理概要     MP6551のCMピン出力[AD]を取得する
// 引数         LAD:左モータのAD値 RAD:右モータのAD値
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void getMotorAD(uint16_t LAD, uint16_t RAD)
{
	// MOTOR_AD_WINDOW は2の冪であることを前提にインデックスをマスク
	// 値を変更する場合はこの前提が崩れないよう注意する
	motorCurrentADLInt[cntMotorAD & (MOTOR_AD_WINDOW - 1)] = LAD; // リングバッファに格納
	motorCurrentADRInt[cntMotorAD & (MOTOR_AD_WINDOW - 1)] = RAD; // リングバッファに格納
	cntMotorAD++; // 次回の格納位置を更新
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
	uint32_t Lint = 0, Rint = 0;

	// リングバッファの総和を計算
	for (uint16_t i = 0; i < MOTOR_AD_WINDOW; i++)
	{
		Lint += motorCurrentADLInt[i];
		Rint += motorCurrentADRInt[i];
	}

	// 移動平均を計算
	motorCurrentADL = Lint / MOTOR_AD_WINDOW;
	motorCurrentADR = Rint / MOTOR_AD_WINDOW;

	// AD値を電圧[V]に変換
	vL = (float)(motorCurrentADL) / 4095 * 3.3;
	vR = (float)(motorCurrentADR) / 4095 * 3.3;

	motorCurrentL = 10000.0 * (vL - VREF_L) / RREF_L;
	motorCurrentR = 10000.0 * (vR - VREF_R) / RREF_R;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 MotorFanPwmOut
// 処理概要     吸引モータにPWMを出力する
// 引数         pwm: モータへの出力(1~1000)
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void MotorFanPwmOut(int16_t pwm)
{
        // PWM が 0 のとき比較レジスタを 0 に設定
        if (pwm == 0)
                __HAL_TIM_SET_COMPARE(&MOTOR_TIM_HANDLER, MOTOR_SUCTION_TIM_CH, 0); // 吸引モータ出力を停止

	if (pwm != 0)
	{
		pwm = abs(pwm);
		__HAL_TIM_SET_COMPARE(&MOTOR_TIM_HANDLER, MOTOR_SUCTION_TIM_CH, (int16_t)((float)pwm / 1000 * MOTOR_PERIOD));
	}
}
