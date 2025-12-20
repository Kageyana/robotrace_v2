//====================================//
// インクルード
//====================================//
#include "PIDcontrol.h"
#include "BMI088.h"
#include "control.h"
#include "encoder.h"
#include "fatfs.h"
#include "lineSensor.h"
#include <stdint.h>
//====================================//
// グローバル変数の宣言
//====================================//
pidParam lineTraceCtrl = {"line", KP1, KI1, KD1, 0, 0};
pidParam lineTraceOmegaFBCtrl = {"lineomega", KP6, KI6, KD6, 0, 0};
pidParam veloCtrl = {"speed", KP2, KI2, KD2, 0, 0};
pidParam yawRateCtrl = {"yawRate", KP3, KI3, KD3, 0, 0};
pidParam yawCtrl = {"yaw", KP4, KI4, KD4, 0, 0};
pidParam distCtrl = {"dist", KP5, KI5, KD5, 0, 0};

// 速度フィードフォワード係数(セットアップ画面から変更可能: Crr×1000)
int16_t speedFeedForwardGain = SPEED_FEEDFORWARD_GAIN_DEFAULT;

uint8_t targetSpeed = 0;		 // 目標速度（初期値0）
float targetSpeedCommand_m_s;	// setTargetSpeedで指定した速度指令値[m/s]
float targetAngle;			 // 目標角速度
float targetAngularVelocity; // 目標角度
int16_t targetDist;			 // 目標X座標
static int16_t speedTargetBefore = 0;	// 速度PID用の前回目標値
static int16_t speedEncoderBefore = 0;	// 速度PID用の前回偏差
extern float batteryVoltage_V;	// control.cで保持したバッテリ電圧[V]

int32_t senL, senR;

///////////////////////////////////////////////////////////////////////////
// モジュール名 calcSpeedFeedForward
// 処理概要     目標速度からフィードフォワード項を算出
// 引数         targetSpeed_mm_s:目標速度[mm/s], batteryVoltage:バッテリ電圧[V]
//              pwm_max:PWM最大値, crr:転がり抵抗係数Crr
// 戻り値       フィードフォワードPWM値
///////////////////////////////////////////////////////////////////////////
static int16_t calcSpeedFeedForward(float targetSpeed_mm_s, float batteryVoltage, int16_t pwm_max, float crr)
{
	float sign;
	float kv_ff;              // [V/(mm/s)]
	float wheelDiameter_m;    // [m]
	float tau_wheel_per_mNm;  // [mNm]
	float v_bias;             // [V]
	float voltageRequest;     // [V]
	float pwm;

	/* sgn(v) をデッドバンド付きで算出 */
	if (targetSpeed_mm_s > SPEED_FEEDFORWARD_SIGN_DEADBAND_MM_S)
		sign = 1.0f;
	else if (targetSpeed_mm_s < -SPEED_FEEDFORWARD_SIGN_DEADBAND_MM_S)
		sign = -1.0f;
	else
		sign = 0.0f;

	/* Kv_ff(G) = G*60/(π D Kv) [V/(mm/s)] */
	kv_ff = SPEED_FEEDFORWARD_GEAR_RATIO * 60.0f /
		((float)M_PI * SPEED_FEEDFORWARD_WHEEL_DIAMETER_MM * SPEED_FEEDFORWARD_KV_RPM_PER_V);
	/* τ_wheel_per[mNm] = (Crr*m*g*(D/2)/2)*1000 */
	wheelDiameter_m = SPEED_FEEDFORWARD_WHEEL_DIAMETER_MM * 0.001f;
	tau_wheel_per_mNm = (crr * SPEED_FEEDFORWARD_MASS_KG * SPEED_FEEDFORWARD_GRAVITY * (wheelDiameter_m / 2.0f) / 2.0f) * 1000.0f;
	/* V_bias(G,η) = (S/Kv) * τ_wheel_per / (η*G) */
	v_bias = (SPEED_FEEDFORWARD_S_RPM_PER_MNM / SPEED_FEEDFORWARD_KV_RPM_PER_V) *
	         tau_wheel_per_mNm / (SPEED_FEEDFORWARD_EFFICIENCY * SPEED_FEEDFORWARD_GEAR_RATIO);
	/* Vreq = Kv_ff*v + sgn(v)*V_bias */
	voltageRequest = (kv_ff * targetSpeed_mm_s) + (sign * v_bias);
	/* PWM = Vreq / Vbat * PWM_MAX （バッテリ電圧0V時は0とする）*/
	if (batteryVoltage > 0.0f)
		pwm = voltageRequest / batteryVoltage * (float)pwm_max;
	else
		pwm = 0.0f;

	/* PWM の上限をクリップ */
	if (pwm > (float)pwm_max)
		pwm = (float)pwm_max;
	else if (pwm < -(float)pwm_max)
		pwm = -(float)pwm_max;

	/* 例: Crr=0.02, η=0.90, Vbat=4.5V, PWM_MAX=1000, v=1000mm/s → PWM≈145 */
	return (int16_t)pwm;
}

///////////////////////////////////////////////////////////////////////////
// モジュール名 setTargetSpeed
// 処理概要     目標速度の設定
// 引数         目標速度の整数倍値
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void setTargetSpeed(float speed)
{
	targetSpeedCommand_m_s = speed;	// フィードフォワード計算へ引き渡す速度指令値[m/s]を保存
	targetSpeed = (int16_t)(speed * PALSE_MILLIMETER);	// 速度指令値[m/s]をエンコーダ換算値へ変換
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 setTargetAngularVelocity
// 処理概要     目標角速度の設定
// 引数         目標角速度[deg/s]
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void setTargetAngularVelocity(float angularVelocity)
{
	targetAngularVelocity = angularVelocity;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 setTargetAngle
// 処理概要     目標角度の設定
// 引数         目標角度[deg]
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void setTargetAngle(float angle)
{
	targetAngle = angle;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 setTargetX
// 処理概要     目標x座標の設定
// 引数         目標x座標[mm]
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void setTargetDist(float dist)
{
	targetDist = (int16_t)(dist * PALSE_MILLIMETER);
	encPID = 0;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 resetSpeedPID
// 処理概要	 速度・距離PIDの内部状態をリセットする
// 引数	 なし
// 戻り値	 なし
///////////////////////////////////////////////////////////////////////////
void resetSpeedPID(void)
{
	// 速度・距離PIDの積分項と差分項をクリア
	veloCtrl.Int = 0.0f;
	distCtrl.Int = 0.0f;
	speedTargetBefore = targetSpeed;	// 現在の目標値を基準として保持
	speedEncoderBefore = 0;	// 偏差履歴をリセット
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motorControlTrace
// 処理概要     ライントレース時の制御量の計算
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void motorControlTrace(void)
{
	int32_t iP, iI, iD, iRet, Dev, Dif;
	static int32_t traceBefore;

	// サーボモータ用PWM値計算
	if (lSensorMax[0] > lSensorMin[0])
	{
		// マクロで設定した重みを掛け合わせてセンサ値を合成
		senL = (lSensorCari[4] * TRACE_WEIGHT_CENTER)
			+ (lSensorCari[3] * TRACE_WEIGHT_INNER)
			+ (lSensorCari[2] * TRACE_WEIGHT_MIDDLE)
			+ (lSensorCari[1] * TRACE_WEIGHT_OUTER)
			+ (lSensorCari[0] * TRACE_WEIGHT_FAR);
		senR = (lSensorCari[5] * TRACE_WEIGHT_CENTER)
			+ (lSensorCari[6] * TRACE_WEIGHT_INNER)
			+ (lSensorCari[7] * TRACE_WEIGHT_MIDDLE)
			+ (lSensorCari[8] * TRACE_WEIGHT_OUTER)
			+ (lSensorCari[9] * TRACE_WEIGHT_FAR);
	}
	else
	{
		senL = (lSensor[4] * TRACE_WEIGHT_CENTER)
			+ (lSensor[3] * TRACE_WEIGHT_INNER)
			+ (lSensor[2] * TRACE_WEIGHT_MIDDLE)
			+ (lSensor[1] * TRACE_WEIGHT_OUTER)
			+ (lSensor[0] * TRACE_WEIGHT_FAR);
		senR = (lSensor[5] * TRACE_WEIGHT_CENTER)
			+ (lSensor[6] * TRACE_WEIGHT_INNER)
			+ (lSensor[7] * TRACE_WEIGHT_MIDDLE)
			+ (lSensor[8] * TRACE_WEIGHT_OUTER)
			+ (lSensor[9] * TRACE_WEIGHT_FAR);

	}
	Dev = senR - senL;

	// I成分積算
	lineTraceCtrl.Int += (float)Dev * 0.001;
	if (lineTraceCtrl.Int > 10000.0)
		lineTraceCtrl.Int = 10000.0; // I成分リミット
	else if (lineTraceCtrl.Int < -10000.0)
		lineTraceCtrl.Int = -10000.0;
	Dif = (Dev - traceBefore) * 1; // dゲイン1/1000倍

	iP = lineTraceCtrl.kp * Dev;			   // 比例
	iI = lineTraceCtrl.ki * lineTraceCtrl.Int; // 積分
	iD = lineTraceCtrl.kd * Dif;			   // 微分
	iRet = iP + iI + iD;
	iRet = iRet >> 8; // PWMを0～1000近傍に収める

	// PWMの上限の設定
	if (iRet > 900)
		iRet = 900;
	if (iRet < -900)
		iRet = -900;

	lineTraceCtrl.pwm = iRet;
	traceBefore = Dev; // 次回はこの値が1ms前の値となる
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motorControlTrace
// 処理概要     ライントレース時の制御量の計算
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void motorControlTraceOmegaFB(void)
{
	int32_t iP, iI, iD, iRet, target, Dev, Dif, senL2, senR2;;
	static int32_t traceBefore, encCrossLine = 0, beforeGainP = 0, beforeGainD = 0;
	static bool stateCrossLine = false;

	// サーボモータ用PWM値計算
	if (lSensorMax[0] > lSensorMin[0])
	{
		if(patternTrace < 10)
		{
			encCrossLine = 0;
			beforeGainP = 0;
			beforeGainD = 0;
			stateCrossLine = false;
		}
		// クロスライン検出 (中央2センサが閾値以上かつ、外側1センサが閾値以上)
		if((lSensorCari[4] > TRACE_CROSSLINE_TH&& lSensorCari[5] > TRACE_CROSSLINE_TH)
		&& (lSensorCari[3] > TRACE_CROSSLINE_TH || lSensorCari[6] > TRACE_CROSSLINE_TH))
		{
			stateCrossLine = true;
			encCrossLine = encTotalN;	// クロスライン通過時のエンコーダ値を保存
		}
		
		// クロスライン付近ではゲインを下げる
		if (stateCrossLine)
		{
			if((encTotalN - encCrossLine) < encMM(TRACE_CROSSLINE_DISTANCE))
			{
				if(beforeGainP == 0 && beforeGainD == 0)
				{
					beforeGainP = lineTraceOmegaFBCtrl.kp;
					beforeGainD = lineTraceOmegaFBCtrl.kd;
					lineTraceOmegaFBCtrl.kp = 10;
					lineTraceOmegaFBCtrl.kd = 0;
				}
			}
			else
			{
				// クロスラインから離れたらゲインを元に戻す
				lineTraceOmegaFBCtrl.kp = beforeGainP;
				lineTraceOmegaFBCtrl.kd = beforeGainD;
				beforeGainP = 0;
				beforeGainD = 0;
				stateCrossLine = false;
			}
		}
		
		// マクロで設定した重みを掛け合わせてセンサ値を合成
		senL2 = (lSensorCari[4] * TRACE_WEIGHT_CENTER) 
				+ (lSensorCari[3] * TRACE_WEIGHT_INNER)
				+ (lSensorCari[2] * TRACE_WEIGHT_MIDDLE)
				+ (lSensorCari[1] * TRACE_WEIGHT_OUTER)
				+ (lSensorCari[0] * TRACE_WEIGHT_FAR);
		senR2 = (lSensorCari[5] * TRACE_WEIGHT_CENTER)
				+ (lSensorCari[6] * TRACE_WEIGHT_INNER)
				+ (lSensorCari[7] * TRACE_WEIGHT_MIDDLE)
				+ (lSensorCari[8] * TRACE_WEIGHT_OUTER)
				+ (lSensorCari[9] * TRACE_WEIGHT_FAR);
	}
	else
	{
		senL2 = (lSensor[4] * TRACE_WEIGHT_CENTER)
				+ (lSensor[3] * TRACE_WEIGHT_INNER)
				+ (lSensor[2] * TRACE_WEIGHT_MIDDLE)
				+ (lSensor[1] * TRACE_WEIGHT_OUTER)
				+ (lSensor[0] * TRACE_WEIGHT_FAR);
		senR2 = (lSensor[5] * TRACE_WEIGHT_CENTER)
				+ (lSensor[6] * TRACE_WEIGHT_INNER)
				+ (lSensor[7] * TRACE_WEIGHT_MIDDLE)
				+ (lSensor[8] * TRACE_WEIGHT_OUTER)
				+ (lSensor[9] * TRACE_WEIGHT_FAR);
	}
	target = ((senR2 - senL2) * encCurrentN) >> 9;
	Dev = target - (int32_t)BMI088val.gyro.z;

	// I成分積算
	lineTraceOmegaFBCtrl.Int += (float)Dev * 0.001;
	if (lineTraceOmegaFBCtrl.Int > 10000.0)
		lineTraceOmegaFBCtrl.Int = 10000.0; // I成分リミット
	else if (lineTraceOmegaFBCtrl.Int < -10000.0)
		lineTraceOmegaFBCtrl.Int = -10000.0;
	Dif = (Dev - traceBefore) * 1; // dゲイン1/1000倍

	iP = lineTraceOmegaFBCtrl.kp * Dev;			   // 比例
	iI = lineTraceOmegaFBCtrl.ki * lineTraceOmegaFBCtrl.Int; // 積分
	iD = lineTraceOmegaFBCtrl.kd * Dif;			   // 微分
	iRet = iP + iI + iD;
	iRet = iRet >> 5; // PWMを0～1000近傍に収める

	// PWMの上限の設定
	if (iRet > 900)
		iRet = 900;
	if (iRet < -900)
		iRet = -900;

	lineTraceOmegaFBCtrl.pwm = iRet;
	traceBefore = Dev; // 次回はこの値が1ms前の値となる
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motorControlSpeed
// 処理概要     モーターの制御量の計算
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void motorControlSpeed(void)
{
	int32_t iP, iI, iD, iRet, Dev, Dif;
	static int16_t feedForwardPwm = 0;	/* フィードフォワードPWM値を保持して再利用 */
	float targetSpeed_mm_s;
	float crr;

	// 駆動モーター用PWM値計算
	Dev = (int16_t)targetSpeed - encCurrentN; // 偏差
	// 目標値を変更したタイミングで積分項リセットとフィードフォワード更新を同時に実施
	if (targetSpeed != speedTargetBefore)
	{
		veloCtrl.Int = 0;	/* 目標値変更時に積分項をリセット */
		// 物理パラメータを用いてフィードフォワード電圧を算出
		targetSpeed_mm_s = targetSpeedCommand_m_s * 1000.0f;	// setTargetSpeedで保持した[m/s]を[mm/s]へ換算
		/* control.cで計測済みのバッテリ電圧[V]を使用 */
		crr = (float)speedFeedForwardGain * SPEED_FEEDFORWARD_CRR_SCALE;	// 転がり抵抗係数Crr
		feedForwardPwm = calcSpeedFeedForward(targetSpeed_mm_s, batteryVoltage_V,
				SPEED_FEEDFORWARD_PWM_MAX_DEFAULT, crr);	// フィードフォワード項
	}

	veloCtrl.Int += (float)Dev * 0.001;	// 時間積分
	Dif = Dev - speedEncoderBefore;		// 微分　dゲイン1/1000倍

	iP = veloCtrl.kp * Dev;		// 比例
	iI = veloCtrl.ki * veloCtrl.Int; // 積分
	iD = veloCtrl.kd * Dif;		// 微分
	// PID制御出力にフィードフォワード補償を加えて応答を改善
	iRet = iP + iI + iD + feedForwardPwm;
	// iRet = iRet;

	// PWMの上限の設定
	if (iRet > 900)
		iRet = 900;
	if (iRet < -900)
		iRet = -900;

	veloCtrl.pwm = iRet;
	speedEncoderBefore = Dev;		      // 次回はこの値が1ms前の値となる
	speedTargetBefore = targetSpeed; // 前回の目標値を記録
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motorControlYaw
// 処理概要     角速度制御時の制御量の計算
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void motorControlYawRate(void)
{
	float iP, iI, iD, Dev, Dif;
	static float angularVelocityBefore;
	static float targetAngularVelocityBefore;
	int32_t iRet;

	Dev = (targetAngularVelocity - BMI088val.gyro.z) * 1; // 目標値-現在値
	// I成分積算
	yawRateCtrl.Int += Dev * 0.005;
	// 目標値を変更したらI成分リセット
	if (targetAngularVelocity != targetAngularVelocityBefore)
		yawRateCtrl.Int = 0;
	Dif = (Dev - angularVelocityBefore) * 2; // dゲイン1/500倍

	iP = yawRateCtrl.kp * Dev;			   // 比例
	iI = yawRateCtrl.ki * yawRateCtrl.Int; // 積分
	iD = yawRateCtrl.kd * Dif;			   // 微分
	iRet = (int32_t)iP + iI + iD;
	iRet = iRet >> 4; // PWMを0～1000近傍に収める

	// PWMの上限の設定
	if (iRet > 900)
		iRet = 900;
	if (iRet < -900)
		iRet = -900;

	yawRateCtrl.pwm = iRet;
	angularVelocityBefore = Dev;						 // 次回はこの値が1ms前の値となる
	targetAngularVelocityBefore = targetAngularVelocity; // 前回の目標値を記録
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motorControlYaw
// 処理概要     角速度制御時の制御量の計算
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void motorControlYaw(void)
{
	float iP, iI, iD, Dev, Dif;
	static float angleBefore;
	static float targetAngleBefore;
	int32_t iRet;

	Dev = (targetAngle - BMI088val.angle.z) * 20; // 目標値-現在値
	// I成分積算
	yawCtrl.Int += Dev * 0.005;
	// 目標値を変更したらI成分リセット
	// if ( targetAngle != targetAngleBefore ) yawCtrl.Int = 0;
	Dif = (Dev - angleBefore) * 1; // dゲイン1/1000倍

	iP = yawCtrl.kp * Dev;		   // 比例
	iI = yawCtrl.ki * yawCtrl.Int; // 積分
	iD = yawCtrl.kd * Dif;		   // 微分
	iRet = (int32_t)iP + iI + iD;
	iRet = iRet >> 2; // PWMを0～1000近傍に収める

	// PWMの上限の設定
	if (iRet > 900)
		iRet = 900;
	if (iRet < -900)
		iRet = -900;

	yawCtrl.pwm = iRet;
	angleBefore = Dev;				 // 次回はこの値が1ms前の値となる
	targetAngleBefore = targetAngle; // 前回の目標値を記録
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motorControldist
// 処理概要     距離制御時の制御量の計算
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void motorControldist(void)
{
	int32_t iP, iI, iD, Dev, Dif, iRet;
	static int32_t distBefore, targetDistBefore;

	Dev = (targetDist - encPID) * 1; // 目標値-現在値
	// I成分積算
	distCtrl.Int += Dev * 0.001;
	// 目標値を変更したらI成分リセット
	// if ( targetDist != targetDistBefore ) distCtrl.Int = 0;
	Dif = (Dev - distBefore) * 1; // dゲイン1/1000倍

	iP = distCtrl.kp * Dev;			 // 比例
	iI = distCtrl.ki * distCtrl.Int; // 積分
	iD = distCtrl.kd * Dif;			 // 微分
	iRet = (int32_t)iP + iI + iD;
	iRet = iRet >> 1; // PWMを0～1000近傍に収める

	// PWMの上限の設定
	if (iRet > 900)
		iRet = 900;
	if (iRet < -900)
		iRet = -900;

	distCtrl.pwm = iRet;
	distBefore = Dev;			   // 次回はこの値が1ms前の値となる
	targetDistBefore = targetDist; // 前回の目標値を記録
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 writePIDparameters
// 処理概要     PIDゲインをSDカードに記録する
// 引数         pid:pidParam型の変数
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void writePIDparameters(pidParam *pid)
{
	FIL fil;
	FRESULT fresult;
	char fileName[20] = PATH_SETTING;

	// ファイル読み込み
	strcat(fileName, pid->name);								 // ファイル名追加
	strcat(fileName, ".txt");									 // 拡張子追加
	fresult = f_open(&fil, fileName, FA_OPEN_ALWAYS | FA_WRITE); // ファイルを開く

	if (fresult == FR_OK)
	{
		f_printf(&fil, "%03d,%03d,%03d", pid->kp, pid->ki, pid->kd);
	}

	f_close(&fil);
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 readPIDparameters
// 処理概要     PIDゲインをSDカードから読み取る
// 引数         pid:pidParam型の変数
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void readPIDparameters(pidParam *pid)
{
	FIL fil;
	FRESULT fresult;
	char fileName[20] = PATH_SETTING;
	TCHAR gain[20];

	// ファイル読み込み
	strcat(fileName, pid->name);								  // ファイル名追加
	strcat(fileName, ".txt");									  // 拡張子追加
	fresult = f_open(&fil, fileName, FA_OPEN_EXISTING | FA_READ); // ファイルを開く

	if (fresult == FR_OK)
	{
		f_gets(gain, sizeof(gain), &fil);						// 文字列取得
		sscanf(gain, "%d,%d,%d", &pid->kp, &pid->ki, &pid->kd); // 文字列→数値
	}

	f_close(&fil);
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 writeSpeedFeedForwardGain
// 処理概要     速度フィードフォワード係数をSDカードへ保存
// 引数         gain:保存する係数
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void writeSpeedFeedForwardGain(int16_t gain)
{
	FIL fil;
	FRESULT fresult;
	char fileName[20] = PATH_SETTING;

	// ファイル名を生成して係数を書き込み
	strcat(fileName, "speed_ff.txt");
	fresult = f_open(&fil, fileName, FA_OPEN_ALWAYS | FA_WRITE);
	if (fresult == FR_OK)
	{
		f_lseek(&fil, 0);	// 既存内容を先頭から上書き
		f_truncate(&fil);
		f_printf(&fil, "%03d", gain);
	}

	f_close(&fil);
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 readSpeedFeedForwardGain
// 処理概要     速度フィードフォワード係数をSDカードから読み込む
// 引数         gain:読み込んだ係数の格納先
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void readSpeedFeedForwardGain(int16_t *gain)
{
	FIL fil;
	FRESULT fresult;
	char fileName[20] = PATH_SETTING;
	TCHAR gainStr[10];
	int16_t readGain = *gain;

	// ファイル名を生成して係数を読み込み
	strcat(fileName, "speed_ff.txt");
	fresult = f_open(&fil, fileName, FA_OPEN_EXISTING | FA_READ);
	if (fresult == FR_OK)
	{
		if (f_gets(gainStr, sizeof(gainStr), &fil) != NULL)
		{
			sscanf(gainStr, "%hd", &readGain);
		}
	}

	f_close(&fil);
	*gain = readGain;
}
