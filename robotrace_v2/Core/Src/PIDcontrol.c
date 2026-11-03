//====================================//
// インクルード
//====================================//
#include "PIDcontrol.h"
#include "BMI088.h"
#include "control.h"
#include "courseAnalysis.h"
#include "encoder.h"
#include "fatfs.h"
#include "lineSensor.h"
#include "motor.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
//====================================//
// グローバル変数の宣言、定義
//====================================//
pidParam lineTraceCtrl = {"line", KP1, KI1, KD1, 0, 0};
pidParam lineTraceOmegaFBCtrl = {"lineomega", KP6, KI6, KD6, 0, 0};
pidParam veloCtrl = {"speed", KP2, KI2, KD2, 0, 0};
pidParam veloCtrlL = {"speed", KP2, KI2, KD2, 0, 0};
pidParam veloCtrlR = {"speed", KP2, KI2, KD2, 0, 0};
pidParam yawRateCtrl = {"yawRate", KP3, KI3, KD3, 0, 0};

// 速度フィードフォワード係数(セットアップ画面から変更可能: Crr×1000)
int16_t speedFeedForwardGain = SPEED_FEEDFORWARD_GAIN_DEFAULT;

ControlGainProfile controlGainProfiles[CONTROL_GAIN_GROUP_COUNT] = {
	{{"speed", KP2, KI2, KD2, 0, 0}, {"lineomega", KP6, KI6, KD6, 0, 0}, SPEED_FEEDFORWARD_GAIN_DEFAULT},
	{{"speed", KP2, KI2, KD2, 0, 0}, {"lineomega", KP6, KI6, KD6, 0, 0}, SPEED_FEEDFORWARD_GAIN_DEFAULT}
};

uint16_t targetSpeed = 0;		 // 目標速度[pulse/ms]（初期値0）
float targetSpeedCommand_m_s;	// setTargetSpeedで指定した速度指令値[m/s]
float targetAngularVelocity; // 目標角速度
static int16_t speedTargetBefore = 0;	// 速度PID用の前回目標値
static int16_t speedTargetBeforeL = 0, speedTargetBeforeR = 0;
static int16_t speedEncoderBeforeL = 0, speedEncoderBeforeR = 0;
static int16_t feedForwardPwmL = 0, feedForwardPwmR = 0;
static int16_t speedEncoderBefore = 0;	// 速度PID用の前回偏差

int32_t log_targetAngularVelocity; // ログ用目標角速度
LineTraceLog lineTraceLog;

//====================================//
// プロトタイプ宣言
//====================================//

/////////////////////////////////////////////////////////////////////
// モジュール名 applyLineTraceOmegaCurveFeedForward
// 処理概要     低速小Rカーブで曲率由来FFとラインセンサーFBを分離して目標角速度を作る
// 引数         sensorTarget:センサー偏差由来の目標角速度[deg/s], sensorDiff:左右センサー合成差
// 戻り値       曲率FF反映後の目標角速度[deg/s]
/////////////////////////////////////////////////////////////////////
static int32_t applyLineTraceOmegaCurveFeedForward(int32_t sensorTarget, int32_t sensorDiff)
{
	if (!LINE_TRACE_OMEGA_CURVE_FF_ENABLE ||
			optimalTrace != BOOST_DISTANCE ||
			targetSpeed == 0 ||
			targetSpeed > LINE_TRACE_OMEGA_CURVE_FF_MAX_SPEED_PULSE ||
			numPPADarry <= 0 ||
			optimalIndex >= (uint16_t)numPPADarry ||
			sensorDiff == 0)
	{
		return sensorTarget;
	}

	int32_t roc = PPAD[optimalIndex].ROC;
	if (roc < 0)
		roc = -roc;
	if (roc <= 0 || roc > LINE_TRACE_OMEGA_CURVE_FF_ROC_MAX_MM)
		return sensorTarget;
	if (roc < LINE_TRACE_OMEGA_CURVE_FF_ROC_MIN_MM)
		roc = LINE_TRACE_OMEGA_CURVE_FF_ROC_MIN_MM;

	float targetSpeed_mm_s = ((float)targetSpeed / PULSE_MILLIMETER) * 1000.0f;
	float curveFf = (targetSpeed_mm_s / (float)roc) * RAD2DEG *
			((float)LINE_TRACE_OMEGA_CURVE_FF_GAIN_PCT / 100.0f);
	if (sensorDiff < 0)
		curveFf = -curveFf;

	float sensorFb = (float)sensorTarget *
			((float)LINE_TRACE_OMEGA_CURVE_FB_SCALE_PCT / 100.0f);
	return (int32_t)(curveFf + sensorFb);
}

///////////////////////////////////////////////////////////////////////////
// モジュール名 settingValueInRange
// 処理概要     設定ファイルから読んだ整数値が指定範囲内か判定する
// 引数         value:判定対象値, min:最小値, max:最大値
// 戻り値       true:範囲内 false:範囲外
///////////////////////////////////////////////////////////////////////////
static bool settingValueInRange(int32_t value, int32_t min, int32_t max)
{
	return (value >= min) && (value <= max);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 controlGainGroupForMode
// 処理概要     実走行モードから共用するPID・FFゲインの組を選ぶ
// 引数         mode:走行モード番号（SLIPの内部モードはDISTANCE）
// 戻り値       一次・定速経路用または曲率別速度用のゲイン組
/////////////////////////////////////////////////////////////////////
ControlGainGroup controlGainGroupForMode(uint8_t mode)
{
	return (mode == BOOST_DISTANCE || mode == BOOST_PATH_CURVE || mode == BOOST_SHORTCUT_CURVE) ?
		CONTROL_GAIN_CURVE : CONTROL_GAIN_BASE;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 applyControlGainGroup
// 処理概要     停止中に設定ゲインを平均・左右速度PIDと角速度FBラインPIDへ同時反映する
// 引数         group:反映するゲイン組
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void applyControlGainGroup(ControlGainGroup group)
{
	if ((unsigned)group >= CONTROL_GAIN_GROUP_COUNT ||
		(patternTrace >= 11U && patternTrace != 103U) || modeLOG) return;
	const ControlGainProfile *profile = &controlGainProfiles[group];
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	resetLineGainState(); // 前走の直線ゲイン復元先を次の組へ持ち越さない。
	veloCtrl.kp = profile->speed.kp;
	veloCtrl.ki = profile->speed.ki;
	veloCtrl.kd = profile->speed.kd;
	syncSpeedPIDGains();
	lineTraceOmegaFBCtrl.kp = profile->lineOmega.kp;
	lineTraceOmegaFBCtrl.ki = profile->lineOmega.ki;
	lineTraceOmegaFBCtrl.kd = profile->lineOmega.kd;
	speedFeedForwardGain = profile->speedFF;
	veloCtrl.Int = veloCtrlL.Int = veloCtrlR.Int = lineTraceOmegaFBCtrl.Int = 0.0F;
	speedTargetBefore = speedTargetBeforeL = speedTargetBeforeR = -1; // 同一目標速度でもFFを再計算する。
	speedEncoderBefore = speedEncoderBeforeL = speedEncoderBeforeR = 0;
	feedForwardPwmL = feedForwardPwmR = 0;
	__set_PRIMASK(primask);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 writeGainFile
// 処理概要     ロック取得済みの設定ファイルへ整数ゲイン列を保存し書込・closeを確認する
// 引数         name:既存ファイル名, values:保存値, count:項目数
// 戻り値       true:保存成功 false:I/O異常
/////////////////////////////////////////////////////////////////////
static bool writeGainFile(const char *name, const int16_t *values, unsigned count)
{
	FIL fil;
	char path[40];
	char text[32]; // 最大6項目の255と区切り文字、終端を保持する。
	size_t used = 0U;
	for (unsigned i = 0U; i < count; ++i)
	{
		int length = snprintf(text + used, sizeof(text) - used, i == 0U ? "%03d" : ",%03d", values[i]);
		if (length < 0 || (size_t)length >= sizeof(text) - used) return false;
		used += (size_t)length;
	}
	int length = snprintf(path, sizeof(path), "%s%s.txt", PATH_SETTING, name);
	if (length < 0 || (size_t)length >= sizeof(path)) return false;
	if (f_open(&fil, path, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) return false;
	UINT written = 0U;
	FRESULT result = f_write(&fil, text, (UINT)used, &written);
	FRESULT closed = f_close(&fil);
	return result == FR_OK && written == used && closed == FR_OK;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 readGainFile
// 処理概要     2組の整数ゲインを読み、旧形式を両組へ継承し欠落・範囲外だけ補修する
// 引数         name:既存ファイル名, values:既定値と読込先, count:2組の合計項目数, min/max:許容範囲
// 戻り値       true:読込または修復成功 false:I/O異常（異常時に上書きしない）
/////////////////////////////////////////////////////////////////////
static bool readGainFile(const char *name, int16_t *values, unsigned count, int min, int max)
{
	FIL fil;
	char path[40];
	char text[64];
	int length = snprintf(path, sizeof(path), "%s%s.txt", PATH_SETTING, name);
	if (length < 0 || (size_t)length >= sizeof(path)) return false;
	FRESULT result = f_open(&fil, path, FA_OPEN_EXISTING | FA_READ);
	if (result == FR_NO_FILE || result == FR_NO_PATH) return writeGainFile(name, values, count);
	if (result != FR_OK) return false;
	UINT read = 0U;
	result = f_read(&fil, text, sizeof(text) - 1U, &read);
	bool complete = f_eof(&fil) && !f_error(&fil);
	FRESULT closed = f_close(&fil);
	if (result != FR_OK || closed != FR_OK) return false;
	text[read] = '\0';
	bool repair = !complete || strlen(text) != read;
	unsigned fields = 0U;
	bool valid = true;
	char *token = text;
	for (unsigned i = 0U; i < count && token != NULL; ++i)
	{
		char *next = strchr(token, ',');
		if (next != NULL) *next++ = '\0';
		char *end;
		long raw = strtol(token, &end, 10);
		bool hasNumber = end != token;
		while (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n') ++end;
		if (hasNumber && *end == '\0' && raw >= min && raw <= max)
			values[i] = (int16_t)raw;
		else
		{
			valid = false;
			repair = true;
		}
		++fields;
		token = next;
	}
	if (token != NULL) repair = true;
	if (!repair && valid && fields == count / 2U)
	{
		// 旧PID3項目・FF1項目は既存の実機調整値を両グループへ引き継ぐ。
		for (unsigned i = 0U; i < count / 2U; ++i) values[i + count / 2U] = values[i];
		repair = true;
	}
	if (fields != count) repair = true;
	return !repair || writeGainFile(name, values, count);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 writeControlGainProfiles
// 処理概要     停止中に2組の設定値を既存のspeed・lineomega・speed_ffファイルへ保存する
// 引数         なし
// 戻り値       true:全ファイル保存成功 false:保存禁止またはI/O異常
/////////////////////////////////////////////////////////////////////
bool writeControlGainProfiles(void)
{
	if (!initMSD || patternTrace >= 11U || modeLOG || !sd_fatfs_lock(500U)) return false;
	int16_t speed[6], omega[6], ff[2];
	for (unsigned i = 0U; i < CONTROL_GAIN_GROUP_COUNT; ++i)
	{
		const ControlGainProfile *p = &controlGainProfiles[i];
		speed[3U*i] = p->speed.kp; speed[3U*i+1U] = p->speed.ki; speed[3U*i+2U] = p->speed.kd;
		omega[3U*i] = p->lineOmega.kp; omega[3U*i+1U] = p->lineOmega.ki; omega[3U*i+2U] = p->lineOmega.kd;
		ff[i] = p->speedFF;
	}
	bool ok = writeGainFile("speed", speed, 6U);
	if (!writeGainFile("lineomega", omega, 6U)) ok = false;
	if (!writeGainFile("speed_ff", ff, 2U)) ok = false;
	sd_fatfs_unlock();
	return ok;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 readControlGainProfiles
// 処理概要     既存3ファイルから2組を読み、破損項目にはコード既定値を使用する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void readControlGainProfiles(void)
{
	if (!initMSD || (patternTrace >= 11U && patternTrace != 103U) || modeLOG || !sd_fatfs_lock(500U)) return;
	int16_t speed[6] = {KP2, KI2, KD2, KP2, KI2, KD2};
	int16_t omega[6] = {KP6, KI6, KD6, KP6, KI6, KD6};
	int16_t ff[2] = {SPEED_FEEDFORWARD_GAIN_DEFAULT, SPEED_FEEDFORWARD_GAIN_DEFAULT};
	(void)readGainFile("speed", speed, 6U, PID_GAIN_MIN, PID_GAIN_MAX);
	(void)readGainFile("lineomega", omega, 6U, PID_GAIN_MIN, PID_GAIN_MAX);
	(void)readGainFile("speed_ff", ff, 2U, SPEED_FF_GAIN_MIN, SPEED_FF_GAIN_MAX);
	sd_fatfs_unlock();
	for (unsigned i = 0U; i < CONTROL_GAIN_GROUP_COUNT; ++i)
	{
		ControlGainProfile *p = &controlGainProfiles[i];
		p->speed.kp = speed[3U*i]; p->speed.ki = speed[3U*i+1U]; p->speed.kd = speed[3U*i+2U];
		p->lineOmega.kp = omega[3U*i]; p->lineOmega.ki = omega[3U*i+1U]; p->lineOmega.kd = omega[3U*i+2U];
		p->speedFF = ff[i];
	}
}

///////////////////////////////////////////////////////////////////////////
// モジュール名 calcSpeedFeedForward
// 処理概要     目標速度からフィードフォワード項を算出
// 引数         targetSpeed_mm_s:目標速度[mm/s], command_max:正規化指令最大値
//              crr:転がり抵抗係数Crr
// 戻り値       フィードフォワード正規化指令
///////////////////////////////////////////////////////////////////////////
static int16_t calcSpeedFeedForward(float targetSpeed_mm_s, int16_t command_max, float crr)
{
	float sign;
	float kv_ff;              // [V/(mm/s)]
	float wheelDiameter_m;    // [m]
	float tau_wheel_per_mNm;  // [mNm]
	float v_bias;             // [V]
	float voltageRequest;     // [V]
	int16_t command;

	/* sgn(v) をデッドバンド付きで算出 */
	if (targetSpeed_mm_s > SPEED_FEEDFORWARD_SIGN_DEADBAND_MM_S)
		sign = 1.0f;
	else if (targetSpeed_mm_s < -SPEED_FEEDFORWARD_SIGN_DEADBAND_MM_S)
		sign = -1.0f;
	else
		sign = 0.0f;

	/* Kv_ff(G) = G*60/(π D Kv) [V/(mm/s)] */
	kv_ff = SPEED_FEEDFORWARD_GEAR_RATIO * 60.0f /
		(IMU_PI_F * SPEED_FEEDFORWARD_WHEEL_DIAMETER_MM * SPEED_FEEDFORWARD_KV_RPM_PER_V);
	/* τ_wheel_per[mNm] = (Crr*m*g*(D/2)/2)*1000 */
	wheelDiameter_m = SPEED_FEEDFORWARD_WHEEL_DIAMETER_MM * 0.001f;
	tau_wheel_per_mNm = (crr * SPEED_FEEDFORWARD_MASS_KG * SPEED_FEEDFORWARD_GRAVITY * (wheelDiameter_m / 2.0f) / 2.0f) * 1000.0f;
	/* V_bias(G,η) = (S/Kv) * τ_wheel_per / (η*G) */
	v_bias = (SPEED_FEEDFORWARD_S_RPM_PER_MNM / SPEED_FEEDFORWARD_KV_RPM_PER_V) *
	         tau_wheel_per_mNm / (SPEED_FEEDFORWARD_EFFICIENCY * SPEED_FEEDFORWARD_GEAR_RATIO);
	/* Vreq = Kv_ff*v + sgn(v)*V_bias */
	voltageRequest = (kv_ff * targetSpeed_mm_s) + (sign * v_bias);
	command = motorVoltageToCommand(voltageRequest);
	if (command > command_max)
		command = command_max;
	else if (command < -command_max)
		command = -command_max;

	return command;
}

///////////////////////////////////////////////////////////////////////////
// モジュール名 setTargetSpeed
// 処理概要     目標速度の設定
// 引数         speed: 目標速度[m/s]。0～TARGET_SPEED_MAX_MPSへ制限する。
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void setTargetSpeed(float speed)
{
	if (!isfinite(speed) || speed < 0.0F)
	{
		speed = 0.0F;
	}
	else if (speed > TARGET_SPEED_MAX_MPS)
	{
		speed = TARGET_SPEED_MAX_MPS;
	}
	targetSpeedCommand_m_s = speed;	// フィードフォワード計算へ引き渡す速度指令値[m/s]を保存
	targetSpeed = (uint16_t)(speed * PULSE_MILLIMETER);	// 速度指令値[m/s]をエンコーダ換算値へ変換
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
/////////////////////////////////////////////////////////////////////
// モジュール名 syncSpeedPIDGains
// 処理概要     速度PIDの設定ゲインを左右輪へ同時反映し、各PIDの内部状態は保持する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void syncSpeedPIDGains(void)
{
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	veloCtrlL.kp = veloCtrlR.kp = veloCtrl.kp;
	veloCtrlL.ki = veloCtrlR.ki = veloCtrl.ki;
	veloCtrlL.kd = veloCtrlR.kd = veloCtrl.kd;
	__set_PRIMASK(primask);
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 resetSpeedPID
// 処理概要	 速度PIDの内部状態をリセットする
// 引数	 なし
// 戻り値	 なし
///////////////////////////////////////////////////////////////////////////
void resetSpeedPID(void)
{
	// 速度PIDの積分項と差分項をクリア
	veloCtrl.Int = 0.0f;
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
	int32_t iP, iI, iD, iRet, Dev, Dif, senL, senR;
	static int32_t traceBefore;

	// サーボモータ用PWM値計算
	if (!isLineSensorCalibrationValid())
	{
		lineTraceCtrl.pwm = 0;
		traceBefore = 0;
		return;
	}

	if (isLineSensorCalibrationValid())
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
	lineTraceCtrl.Int += (float)Dev * 0.001f;
	if (lineTraceCtrl.Int > 10000.0f)
		lineTraceCtrl.Int = 10000.0f; // I成分リミット
	else if (lineTraceCtrl.Int < -10000.0f)
		lineTraceCtrl.Int = -10000.0f;
	Dif = (Dev - traceBefore) * 1; // dゲイン1/1000倍

	iP = lineTraceCtrl.kp * Dev;			   // 比例
	iI = lineTraceCtrl.ki * lineTraceCtrl.Int; // 積分
	iD = lineTraceCtrl.kd * Dif;			   // 微分
	iRet = iP + iI + iD;
	iRet = iRet >> 8; // PWMを0～1000近傍に収める

	// PWMの上限の設定
	if (iRet > 1000)
		iRet = 1000;
	if (iRet < -1000)
		iRet = -1000;

	lineTraceCtrl.pwm = iRet;
	traceBefore = Dev; // 次回はこの値が1ms前の値となる
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motorControlTraceOmegaFB
// 処理概要     速度倍率を制限したライン目標と実角速度から制御量を計算する
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void motorControlTraceOmegaFB(void)
{
	int32_t iP, iI, iD, iRet, target, Dev, Dif, senL, senR, sensorDiff;
	static int32_t traceBefore;
	lineTraceLog = (LineTraceLog){0};

	// サーボモータ用PWM値計算
	if (!isLineSensorCalibrationValid())
	{
		lineTraceOmegaFBCtrl.pwm = 0;
		traceBefore = 0;
		return;
	}
	// changeGain()後の値ではなく、この計算に適用した判定とゲインを保持する。
	lineTraceLog.gain = lineGainLog;
	lineTraceLog.kpUsed = (uint8_t)lineTraceOmegaFBCtrl.kp;
	lineTraceLog.kdUsed = (uint8_t)lineTraceOmegaFBCtrl.kd;

	if (isLineSensorCalibrationValid())
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
	sensorDiff = senR - senL;
	// 高速時は目標側の速度倍率だけを抑え、ジャイロFBの強さは維持する。
	int32_t sensorSpeedPulse = encCurrentN;
	const int32_t referenceSpeedPulse = (int32_t)(LINE_TRACE_OMEGA_REFERENCE_SPEED_MPS * PULSE_MILLIMETER);
	if (sensorSpeedPulse > referenceSpeedPulse)
		sensorSpeedPulse = referenceSpeedPulse;
	else if (sensorSpeedPulse < -referenceSpeedPulse)
		sensorSpeedPulse = -referenceSpeedPulse;
	target = (sensorDiff * sensorSpeedPulse) >> 9;
	target = applyLineTraceOmegaCurveFeedForward(target, sensorDiff);
	if (LINE_TRACE_OMEGA_TARGET_CAP_ENABLE &&
			optimalTrace == BOOST_DISTANCE &&
			targetSpeed <= LINE_TRACE_OMEGA_TARGET_CAP_MAX_SPEED_PULSE)
	{
		if (target > LINE_TRACE_OMEGA_TARGET_CAP_ABS_DEG_S)
			target = LINE_TRACE_OMEGA_TARGET_CAP_ABS_DEG_S;
		else if (target < -LINE_TRACE_OMEGA_TARGET_CAP_ABS_DEG_S)
			target = -LINE_TRACE_OMEGA_TARGET_CAP_ABS_DEG_S;
	}
	log_targetAngularVelocity = target; // ログ用に目標角速度を保存
	Dev = target - (int32_t)imuVal.gyro.z;

	// I成分積算
	if(lineTraceOmegaFBCtrl.pwm <= 900 && lineTraceOmegaFBCtrl.pwm >= -900)
	{
		lineTraceOmegaFBCtrl.Int += (float)Dev * 0.001f;
	}
	if (lineTraceOmegaFBCtrl.Int > 10000.0f)
		lineTraceOmegaFBCtrl.Int = 10000.0f; // I成分リミット
	else if (lineTraceOmegaFBCtrl.Int < -10000.0f)
		lineTraceOmegaFBCtrl.Int = -10000.0f;

	Dif = (Dev - traceBefore) * 1; // dゲイン1/1000倍

	iP = lineTraceOmegaFBCtrl.kp * Dev;                    // 比例
	iI = (int32_t)(lineTraceOmegaFBCtrl.ki * lineTraceOmegaFBCtrl.Int); // 積分
	iD = lineTraceOmegaFBCtrl.kd * Dif;                    // 微分
	lineTraceLog.pTerm = iP;
	lineTraceLog.dTerm = iD;
	iRet = iP + iI + iD;

	// PWMを0～1000近傍に収める
	iRet = iRet >> 5;

	// PWMの上限の設定
	if (iRet > 1000)  iRet = 1000;
	if (iRet < -1000) iRet = -1000;

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
	int32_t dEncSpeed_p;

	// 駆動モーター用PWM値計算
dEncSpeed_p = encCurrentN; // 速度制御は生エンコーダを使用する。
	Dev = (int16_t)targetSpeed - dEncSpeed_p; // 偏差
	// 目標値を変更したタイミングで積分項リセットとフィードフォワード更新を同時に実施
	if (targetSpeed != speedTargetBefore)
	{
		veloCtrl.Int = 0;	/* 目標値変更時に積分項をリセット */
		// 物理パラメータを用いてフィードフォワード電圧を算出
		targetSpeed_mm_s = targetSpeedCommand_m_s * 1000.0f;	// setTargetSpeedで保持した[m/s]を[mm/s]へ換算
		crr = (float)speedFeedForwardGain * SPEED_FEEDFORWARD_CRR_SCALE;	// 転がり抵抗係数Crr
		feedForwardPwm = calcSpeedFeedForward(targetSpeed_mm_s,
				SPEED_FEEDFORWARD_PWM_MAX_DEFAULT, crr);	// フィードフォワード項
	}

	// I成分積算 アンチワインドアップ
	if(veloCtrl.pwm <= 1000 && veloCtrl.pwm >= -1000)
	{
		veloCtrl.Int += (float)Dev * 0.001f;
	}
	if (veloCtrl.Int > 10000.0f)
		veloCtrl.Int = 10000.0f; // I成分リミット
	else if (veloCtrl.Int < -10000.0f)
		veloCtrl.Int = -10000.0f;

	// veloCtrl.Int += (float)Dev * 0.001f;	// 時間積分
	Dif = Dev - speedEncoderBefore;		// 微分　dゲイン1/1000倍

	iP = veloCtrl.kp * Dev;		// 比例
	iI = veloCtrl.ki * veloCtrl.Int; // 積分
	iD = veloCtrl.kd * Dif;		// 微分
	// PID制御出力にフィードフォワード補償を加えて応答を改善
	iRet = iP + iI + iD + feedForwardPwm;

	// PWMの上限の設定
	if (iRet > 1000)
		iRet = 1000;
	if (iRet < -1000)
		iRet = -1000;

	veloCtrl.pwm = iRet;
	speedEncoderBefore = Dev;		      // 次回はこの値が1ms前の値となる
	speedTargetBefore = targetSpeed; // 前回の目標値を記録
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 updateWheelSpeedPID
// 処理概要     片輪の速度PIDと目標変更時のFF更新を行い、輪ごとの状態を保持する
// 引数         pid: 対象PID, targetEnc/currentEnc: 目標/現在速度[pulse/ms]
//              previousTarget: 前回目標, previousError: 前回偏差[pulse/ms]
//              feedForward: 保持するFF指令, crr: 転がり抵抗係数
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
static void updateWheelSpeedPID(pidParam *pid, int16_t targetEnc, int16_t currentEnc,
    int16_t *previousTarget, int16_t *previousError, int16_t *feedForward, float crr)
{
    int32_t dev = (int32_t)targetEnc - (int32_t)currentEnc;
    if (targetEnc != *previousTarget)
    {
        pid->Int = 0.0f;
        float target_mm_s = ((float)targetEnc / PULSE_MILLIMETER) * 1000.0f;
        *feedForward = calcSpeedFeedForward(target_mm_s,
            SPEED_FEEDFORWARD_PWM_MAX_DEFAULT, crr);
    }
    // 飽和していないときだけ積分する既存の左右速度PID条件を維持する。
    if (pid->pwm < 1000 && pid->pwm > -1000)
        pid->Int += (float)dev * 0.001f;
    if (pid->Int > 10000.0f) pid->Int = 10000.0f;
    if (pid->Int < -10000.0f) pid->Int = -10000.0f;

    int32_t dif = dev - *previousError;
    int32_t iP = pid->kp * dev;
    int32_t iI = (int32_t)(pid->ki * pid->Int);
    int32_t iD = pid->kd * dif;
    int32_t output = iP + iI + iD + *feedForward;
    if (output > 1000) output = 1000;
    if (output < -1000) output = -1000;
    pid->pwm = output;
    *previousError = (int16_t)dev;
    *previousTarget = targetEnc;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motorControlSpeedLR
// 処理概要     左右独立の速度PIDを同じ処理で更新する
// 引数         targetEncL: 左目標速度, targetEncR: 右目標速度[pulse/ms]
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void motorControlSpeedLR(int16_t targetEncL, int16_t targetEncR)
{
    float crr = (float)speedFeedForwardGain * SPEED_FEEDFORWARD_CRR_SCALE;
    updateWheelSpeedPID(&veloCtrlL, targetEncL, encCurrentL,
        &speedTargetBeforeL, &speedEncoderBeforeL, &feedForwardPwmL, crr);
    updateWheelSpeedPID(&veloCtrlR, targetEncR, encCurrentR,
        &speedTargetBeforeR, &speedEncoderBeforeR, &feedForwardPwmR, crr);
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motorControlYawRate
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

	Dev = (targetAngularVelocity - imuVal.gyro.z) * 1; // 目標値-現在値
	// I成分積算
	yawRateCtrl.Int += Dev * 0.001f;
	// 目標値を変更したらI成分リセット
	if (targetAngularVelocity != targetAngularVelocityBefore)
		yawRateCtrl.Int = 0;
	Dif = Dev - angularVelocityBefore; // 1ms周期の偏差差分

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
// モジュール名 writePIDparameters
// 処理概要     PIDゲインをSDカードに記録する
// 引数         pid:pidParam型の変数
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void writePIDparameters(pidParam *pid)
{
	FIL fil;
	FRESULT fresult;
	char fileName[32] = PATH_SETTING;

	// ファイル読み込み
	strcat(fileName, pid->name);								 // ファイル名追加
	strcat(fileName, ".txt");									 // 拡張子追加
	fresult = f_open(&fil, fileName, FA_CREATE_ALWAYS | FA_WRITE); // ファイルを開く

	if (fresult == FR_OK)
	{
		f_printf(&fil, "%03d,%03d,%03d", pid->kp, pid->ki, pid->kd);
		f_close(&fil);
	}
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
	char fileName[32] = PATH_SETTING;
	TCHAR gain[20] = {0};
	int raw[3] = {pid->kp, pid->ki, pid->kd};
	int parsed = 0;
	bool repair = false;

	// ファイル読み込み
	strcat(fileName, pid->name);								  // ファイル名追加
	strcat(fileName, ".txt");									  // 拡張子追加
	fresult = f_open(&fil, fileName, FA_OPEN_EXISTING | FA_READ); // ファイルを開く

	if (fresult == FR_OK)
	{
		if (f_gets(gain, sizeof(gain), &fil) != NULL)
		{
			parsed = sscanf(gain, "%d,%d,%d", &raw[0], &raw[1], &raw[2]);
			if (parsed >= 1 && settingValueInRange(raw[0], PID_GAIN_MIN, PID_GAIN_MAX))
			{
				pid->kp = (int16_t)raw[0];
			}
			else
			{
				repair = true;
			}
			if (parsed >= 2 && settingValueInRange(raw[1], PID_GAIN_MIN, PID_GAIN_MAX))
			{
				pid->ki = (int16_t)raw[1];
			}
			else
			{
				repair = true;
			}
			if (parsed >= 3 && settingValueInRange(raw[2], PID_GAIN_MIN, PID_GAIN_MAX))
			{
				pid->kd = (int16_t)raw[2];
			}
			else
			{
				repair = true;
			}
		}
		else
		{
			repair = true;
		}
		f_close(&fil);
	}
	else
	{
		repair = true;
	}

	if (repair)
	{
		writePIDparameters(pid);
	}
}
