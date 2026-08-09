//====================================//
// インクルード
//====================================//
#include "motor.h"
#include "battery.h"
#include <stdint.h>
#include <stdlib.h>

//====================================//
// グローバル変数の宣言
//====================================//
int16_t motorpwmL = 0;
int16_t motorpwmR = 0;
float motorVoltageCmdL_V = 0.0f;
float motorVoltageCmdR_V = 0.0f;

int16_t motorCurrentADL, motorCurrentADR;
uint16_t motorCurrentADLInt[MOTOR_AD_WINDOW], motorCurrentADRInt[MOTOR_AD_WINDOW];
static volatile uint32_t motorCurrentADLSum = 0;
static volatile uint32_t motorCurrentADRSum = 0;
uint32_t cntMotorAD = 0;
float motorCurrentL, motorCurrentR; // モーター電流値[A]
bool calibrateMotorCurrent = false; // 電流センサキャリブレーションフラグ
int16_t motorCurrentADLoffset = 4096/2, motorCurrentADRoffset = 4096/2; // キャリブレーション用オフセット値
/////////////////////////////////////////////////////////////////////
// モジュール名 clampCommand
// 処理概要     正規化モータ指令を-1000..1000に制限する
// 引数         command: 正規化モータ指令
// 戻り値       制限後の正規化モータ指令
/////////////////////////////////////////////////////////////////////
static int16_t clampCommand(int32_t command)
{
	if (command > 1000)
		return 1000;
	if (command < -1000)
		return -1000;
	return (int16_t)command;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 clampVoltage
// 処理概要     モータ指令電圧を許容範囲に制限する
// 引数         voltage_V: モータ指令電圧[V]
// 戻り値       制限後のモータ指令電圧[V]
/////////////////////////////////////////////////////////////////////
static float clampVoltage(float voltage_V)
{
	if (voltage_V > MOTOR_VOLTAGE_CMD_MAX_V)
		return MOTOR_VOLTAGE_CMD_MAX_V;
	if (voltage_V < -MOTOR_VOLTAGE_CMD_MAX_V)
		return -MOTOR_VOLTAGE_CMD_MAX_V;
	return voltage_V;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 motorVoltageToDuty
// 処理概要     指令電圧を現在のバッテリ電圧に対するDUTYへ変換する
// 引数         voltage_V: モータ指令電圧[V]
// 戻り値       実DUTY指令
/////////////////////////////////////////////////////////////////////
static int16_t motorVoltageToDuty(float voltage_V)
{
	float duty;

	if (!batteryVoltageValid || batteryVoltage_V <= 0.0f)
	{
		return 0;
	}

	duty = clampVoltage(voltage_V) / batteryVoltage_V * 1000.0f;
	if (duty > 1000.0f)
		duty = 1000.0f;
	if (duty < -1000.0f)
		duty = -1000.0f;

	return (int16_t)duty;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 motorDutyOut
// 処理概要     左右のモータに実DUTYを出力する
// 引数         pwmL: 左モータへの実DUTY(-1000..1000) pwmR: 右モータへの実DUTY(-1000..1000)
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void motorDutyOut(int16_t pwmL, int16_t pwmR)
{
	pwmL = clampCommand(pwmL);
	pwmR = clampCommand(pwmR);
	motorpwmL = pwmL;
	motorpwmR = pwmR;

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
// モジュール名 motorCommandToVoltage_V
// 処理概要     正規化モータ指令を公称電圧基準の指令電圧へ変換する
// 引数         command: 正規化モータ指令(-1000..1000)
// 戻り値       指令電圧[V]
///////////////////////////////////////////////////////////////////////////
float motorCommandToVoltage_V(int16_t command)
{
	int16_t cmd = clampCommand(command);
	float voltage = (float)cmd / 1000.0f * MOTOR_COMMAND_NOMINAL_V;
	return clampVoltage(voltage);
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motorVoltageToCommand
// 処理概要     指令電圧を公称電圧基準の正規化モータ指令へ変換する
// 引数         voltage_V: 指令電圧[V]
// 戻り値       正規化モータ指令(-1000..1000)
///////////////////////////////////////////////////////////////////////////
int16_t motorVoltageToCommand(float voltage_V)
{
	voltage_V = clampVoltage(voltage_V);
	return clampCommand((int32_t)(voltage_V / MOTOR_COMMAND_NOMINAL_V * 1000.0f));
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motorVoltageOut
// 処理概要     左右の指令電圧を現在のバッテリ電圧で補償して出力する
// 引数         voltageL_V: 左モータ指令電圧[V] voltageR_V: 右モータ指令電圧[V]
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void motorVoltageOut(float voltageL_V, float voltageR_V)
{
	motorVoltageCmdL_V = clampVoltage(voltageL_V);
	motorVoltageCmdR_V = clampVoltage(voltageR_V);
	motorDutyOut(motorVoltageToDuty(motorVoltageCmdL_V), motorVoltageToDuty(motorVoltageCmdR_V));
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motorCommandOut
// 処理概要     左右の正規化モータ指令を指令電圧に変換して出力する
// 引数         cmdL: 左モータ正規化指令 cmdR: 右モータ正規化指令
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void motorCommandOut(int16_t cmdL, int16_t cmdR)
{
	motorVoltageOut(motorCommandToVoltage_V(cmdL), motorCommandToVoltage_V(cmdR));
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motorCommandOutSynth
// 処理概要     各PID制御量を正規化指令として合成し、電圧補償してモータに出力する
// 引数         tCmd: トレース制御量 sCmd: 速度制御量
//              yrCmd: ヨーレート制御量
//              dCmd : 距離制御量
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void motorCommandOutSynth(int16_t tCmd, int16_t sCmd, int16_t yrCmd, int16_t dCmd)
{
	int16_t overCmd, traceCmd = tCmd;

	if (abs(sCmd + tCmd) > 1000 || abs(sCmd - tCmd) > 1000)
	{
		// ライントレースと速度制御の合計制御量が1000を超えたとき
		overCmd = abs(sCmd) + abs(tCmd) - 1000;

		// トレースの内輪側から越えた分の制御量を引く。正負はtCmdと同じ。
		if (tCmd != 0)
		{
			traceCmd = tCmd - (overCmd * (tCmd / abs(tCmd)));
		}
		else
		{
			traceCmd = tCmd;
		}
	}

	int16_t cmdR = clampCommand((int32_t)sCmd - traceCmd - yrCmd + dCmd);
	int16_t cmdL = clampCommand((int32_t)sCmd + traceCmd + yrCmd + dCmd);

	motorCommandOut(cmdL, cmdR);
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
	uint16_t idx = (uint16_t)(cntMotorAD & (MOTOR_AD_WINDOW - 1)); // インデックス計算

    // 古い値を総和から除去（配列がint16_tなので符号影響を避けてuint16_t化してから引く）
    motorCurrentADLSum -= motorCurrentADLInt[idx];
    motorCurrentADRSum -= motorCurrentADRInt[idx];

    // 新しい値を格納
    motorCurrentADLInt[idx] = LAD;
    motorCurrentADRInt[idx] = RAD;

    // 新しい値を総和へ追加
    motorCurrentADLSum += LAD;
    motorCurrentADRSum += RAD;

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
	float dvL, dvR;

	// リングバッファの総和から移動平均を計算
	motorCurrentADL = (int16_t)(motorCurrentADLSum / MOTOR_AD_WINDOW);
    motorCurrentADR = (int16_t)(motorCurrentADRSum / MOTOR_AD_WINDOW);

	// AD値を電圧[V]に変換
	dvL = (float)((int32_t)motorCurrentADL - (int32_t)motorCurrentADLoffset) / 4095 * adcVref;
	dvR = (float)((int32_t)motorCurrentADR - (int32_t)motorCurrentADRoffset) / 4095 * adcVref;

	motorCurrentL = 10000.0 * (dvL) / RREF;
	motorCurrentR = 10000.0 * (dvR) / RREF;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 calibrationMotorCurrent
// 処理概要     電流センサのキャリブレーション
// 引数         なし
// 戻り値       なしs
///////////////////////////////////////////////////////////////////////////
void calibrationMotorCurrent(void)
{
	static uint16_t n = 0;
    static int32_t accL = 0, accR = 0;

    if (n < 100) {
        motorCurrentADL = motorCurrentADLSum / MOTOR_AD_WINDOW;
        motorCurrentADR = motorCurrentADRSum / MOTOR_AD_WINDOW;

        accL += motorCurrentADL;
        accR += motorCurrentADR;
        n++;
    } else {
        motorCurrentADLoffset = accL / 100;
        motorCurrentADRoffset = accR / 100;

        accL = 0;
		accR = 0;
        n = 0;
        calibrateMotorCurrent = false;
    }
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
