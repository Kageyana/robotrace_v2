//====================================//
// インクルード
//====================================//
#include "PIDcontrol.h"
#include "fatfs.h"
#include "motor.h"
#include "encoder.h"
#include "lineSensor.h"
#include "BMI088.h"
#include "control.h"
//====================================//
// グローバル変数の宣言
//====================================//
pidParam lineTraceCtrl = {"line", KP1, KI1, KD1, 0, 0};
pidParam veloCtrl = {"speed", KP2, KI2, KD2, 0, 0};
pidParam yawRateCtrl = {"yawRate", KP3, KI3, KD3, 0, 0};
pidParam yawCtrl = {"yaw", KP4, KI4, KD4, 0, 0};
pidParam distCtrl = {"dist", KP5, KI5, KD5, 0, 0};

// 速度フィードフォワード係数(セットアップ画面から変更可能: Crr×1000)
int16_t speedFeedForwardGain = SPEED_FEEDFORWARD_GAIN_DEFAULT;

const CascadeControlParams cascadeControlParamsDefault = {
	.line = {
		.kp = 4.0f,
		.ki = 0.0f,
		.kd = 0.2f,
		.i_limit = 0.5f,
		.d_lpf = 0.02f,
		.ref_limit = 6.0f
	},
	.yaw = {
		.kp = 2.0f,
		.ki = 0.5f,
		.kd = 0.02f,
		.i_limit = 1.0f,
		.out_limit = 6.0f
	},
	.speed_l = {
		.kp = 8.0f,
		.ki = 40.0f,
		.kd = 0.1f,
		.i_limit = 2.0f
	},
	.speed_r = {
		.kp = 8.0f,
		.ki = 40.0f,
		.kd = 0.1f,
		.i_limit = 2.0f
	},
	.duty_limit = 1.0f,
	.tread_half = 0.045f,
	.v_limit = 3.0f
};

CascadeControlParams cascadeControlParams = {
	.line = {
		.kp = 4.0f,
		.ki = 0.0f,
		.kd = 0.2f,
		.i_limit = 0.5f,
		.d_lpf = 0.02f,
		.ref_limit = 6.0f
	},
	.yaw = {
		.kp = 2.0f,
		.ki = 0.5f,
		.kd = 0.02f,
		.i_limit = 1.0f,
		.out_limit = 6.0f
	},
	.speed_l = {
		.kp = 8.0f,
		.ki = 40.0f,
		.kd = 0.1f,
		.i_limit = 2.0f
	},
	.speed_r = {
		.kp = 8.0f,
		.ki = 40.0f,
		.kd = 0.1f,
		.i_limit = 2.0f
	},
	.duty_limit = 1.0f,
	.tread_half = 0.045f,
	.v_limit = 3.0f
};

/**
 * @brief カスケード制御パラメータを既定値に戻す。
 * @param[out] params リセット対象の構造体
 */
void CascadeControlParamsResetDefaults(CascadeControlParams *params)
{
	if (params == NULL) {
		return;
	}
	*params = cascadeControlParamsDefault;
}

uint8_t targetSpeed = 0;		 // 目標速度（初期値0）
float targetSpeedCommand_m_s;	// setTargetSpeedで指定した速度指令値[m/s]
float targetAngle;			 // 目標角速度
float targetAngularVelocity; // 目標角度
int16_t targetDist;			 // 目標X座標
static int16_t speedTargetBefore = 0;	// 速度PID用の前回目標値
static int16_t speedEncoderBefore = 0;	// 速度PID用の前回偏差
extern float batteryVoltage_V;	// control.cで保持したバッテリ電圧[V]

#if USE_CASCADE_TRACE

typedef struct {
	float integral;
	float prev_error;
	float d_lpf_state;
	float omega_ref;
} LineCascadeState;

typedef struct {
	float integral;
	float prev_error;
	float last_output;
} YawCascadeState;

typedef struct {
	float integral;
	float prev_error;
	float last_output;
} SpeedCascadeState;

static LineCascadeState lineCascadeState;
static YawCascadeState yawCascadeState;
static SpeedCascadeState speedCascadeState[2];
static float outerLoopElapsed = 0.0f;

/**
 * @brief 値を指定範囲へ収める簡易クリッピング関数。
 */
static float clampf(float value, float min, float max)
{
	if (!isfinite(value)) {
		return 0.0f;
	}
	if (value > max) {
		value = max;
	} else if (value < min) {
		value = min;
	}
	return value;
}

/**
 * @brief 1次遅れローパスフィルタを適用する。
 */
static float apply_lowpass(float input, float prev, float dt, float tau)
{
	if (!isfinite(input)) {
		return prev;
	}
	if (!isfinite(dt) || dt <= 0.0f || tau <= 0.0f) {
		return input;
	}
	float alpha = dt / (tau + dt);
	return prev + alpha * (input - prev);
}

/**
 * @brief ラインセンサ値を左右5本ずつ取得する。
 */
static bool read_line_sensor_pairs(uint16_t left[5], uint16_t right[5])
{
	bool calibrated = (lSensorMax[0] > lSensorMin[0]);
	uint32_t sum = 0U;
	if (calibrated) {
		left[0] = lSensorCari[4];
		left[1] = lSensorCari[3];
		left[2] = lSensorCari[2];
		left[3] = lSensorCari[1];
		left[4] = lSensorCari[0];
		right[0] = lSensorCari[5];
		right[1] = lSensorCari[6];
		right[2] = lSensorCari[7];
		right[3] = lSensorCari[8];
		right[4] = lSensorCari[9];
	} else {
		left[0] = lSensor[4];
		left[1] = lSensor[3];
		left[2] = lSensor[2];
		left[3] = lSensor[1];
		left[4] = lSensor[0];
		right[0] = lSensor[5];
		right[1] = lSensor[6];
		right[2] = lSensor[7];
		right[3] = lSensor[8];
		right[4] = lSensor[9];
	}
	for (uint32_t i = 0U; i < 5U; i++) {
		sum += (uint32_t)left[i];
		sum += (uint32_t)right[i];
	}
	return (sum > 0U);
}

/**
 * @brief 左右センサの差を正規化してライン誤差を算出する。
 */
static float compute_line_error(const uint16_t left[5], const uint16_t right[5])
{
	static const float weights[5] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
	float sum_left = 0.0f;
	float sum_right = 0.0f;
	for (uint32_t i = 0U; i < 5U; i++) {
		sum_left += (float)left[i] * weights[i];
		sum_right += (float)right[i] * weights[i];
	}
	float diff = sum_left - sum_right;
	float total = sum_left + sum_right;
	if (total > 1.0e-6f) {
		diff /= total;
	} else {
		diff = 0.0f;
	}
	return diff;
}

/**
 * @brief 旧来のログ変数へ最新状態を反映するヘルパー。
 */
static void update_legacy_logs(float duty_left, float duty_right, float yaw_cmd, float omega_ref)
{
	const float duty_avg = (duty_left + duty_right) * 0.5f;
	veloCtrl.pwm = (int16_t)clampf(duty_avg * 1000.0f, -1000.0f, 1000.0f);
	veloCtrl.Int = (speedCascadeState[MOTOR_SIDE_LEFT].integral + speedCascadeState[MOTOR_SIDE_RIGHT].integral) * 0.5f;
	lineTraceCtrl.pwm = (int16_t)clampf(omega_ref * 1000.0f, -1000.0f, 1000.0f);
	lineTraceCtrl.Int = lineCascadeState.integral;
	yawRateCtrl.pwm = (int16_t)clampf(yaw_cmd * 1000.0f, -1000.0f, 1000.0f);
	yawRateCtrl.Int = yawCascadeState.integral;
}

/**
 * @brief 角速度誤差に対するPID制御を実行する。
 * @param yaw_error 角速度誤差[rad/s]
 * @param dt 角速度ループの周期[s]
 * @return 角速度制御出力[rad/s相当]
 */
float motorControlYawRate(float yaw_error, float dt)
{
	if (!isfinite(dt) || dt <= 0.0f) {
		return yawCascadeState.last_output;
	}
	if (!isfinite(yaw_error)) {
		yaw_error = 0.0f;
	}
	float derivative = 0.0f;
	if (dt > 1.0e-6f) {
		derivative = (yaw_error - yawCascadeState.prev_error) / dt;
	}
	float integral_candidate = clampf(yawCascadeState.integral + yaw_error * dt,
		- cascadeControlParams.yaw.i_limit,
		  cascadeControlParams.yaw.i_limit);
	float output_unsat = (cascadeControlParams.yaw.kp * yaw_error) +
		(cascadeControlParams.yaw.ki * integral_candidate) +
		(cascadeControlParams.yaw.kd * derivative);
	float output = clampf(output_unsat,
		- cascadeControlParams.yaw.out_limit,
		  cascadeControlParams.yaw.out_limit);
	if ((output != output_unsat) && (yaw_error * output > 0.0f)) {
		integral_candidate = yawCascadeState.integral;
		output_unsat = (cascadeControlParams.yaw.kp * yaw_error) +
		(cascadeControlParams.yaw.ki * integral_candidate) +
		(cascadeControlParams.yaw.kd * derivative);
		output = clampf(output_unsat,
		- cascadeControlParams.yaw.out_limit,
		  cascadeControlParams.yaw.out_limit);
	}
	yawCascadeState.integral = integral_candidate;
	yawCascadeState.prev_error = yaw_error;
	yawCascadeState.last_output = output;
	return output;
}

/**
 * @brief 左右ホイールの速度PIDを1ステップ更新する。
 * @param motor 制御対象のモータID
 * @param v_ref 目標速度[m/s]
 * @param v_meas 実測速度[m/s]
 * @param dt サンプリング周期[s]
 * @return PWMデューティ（-1.0〜1.0）
 */
float motorControlSpeed(motor_side_t motor, float v_ref, float v_meas, float dt)
{
	if (motor >= MOTOR_SIDE_RIGHT + 1) {
		return 0.0f;
	}
	if (!isfinite(dt) || dt <= 0.0f) {
		return speedCascadeState[motor].last_output;
	}
	if (!isfinite(v_ref)) {
		v_ref = 0.0f;
	}
	if (!isfinite(v_meas)) {
		v_meas = 0.0f;
	}
	const SpeedPidParams *param = (motor == MOTOR_SIDE_LEFT) ?
		&cascadeControlParams.speed_l : &cascadeControlParams.speed_r;
	SpeedCascadeState *state = &speedCascadeState[motor];
	float error = v_ref - v_meas;
	float derivative = 0.0f;
	if (dt > 1.0e-6f) {
		derivative = (error - state->prev_error) / dt;
	}
	float integral_candidate = clampf(state->integral + error * dt,
		- param->i_limit,
		  param->i_limit);
	float output_unsat = (param->kp * error) +
		(param->ki * integral_candidate) +
		(param->kd * derivative);
	float output = clampf(output_unsat,
		- cascadeControlParams.duty_limit,
		  cascadeControlParams.duty_limit);
	if ((output != output_unsat) && (error * output > 0.0f)) {
		integral_candidate = state->integral;
		output_unsat = (param->kp * error) +
		(param->ki * integral_candidate) +
		(param->kd * derivative);
		output = clampf(output_unsat,
		- cascadeControlParams.duty_limit,
		  cascadeControlParams.duty_limit);
	}
	state->integral = integral_candidate;
	state->prev_error = error;
	state->last_output = output;
	return output;
}

/**
 * @brief カスケード制御の外側ループを実行する。
 * @param dt_outer 外側ループの周期[s]
 * @param dt_inner 内側ループの周期[s]
 */
void motorControlTrace(float dt_outer, float dt_inner)
{
	if (!isfinite(dt_inner) || dt_inner <= 0.0f) {
		dt_inner = CASCADE_DT_INNER_DEFAULT;
	}
	if (!isfinite(dt_outer) || dt_outer <= 0.0f) {
		dt_outer = CASCADE_DT_OUTER_DEFAULT;
	}
	outerLoopElapsed += dt_inner;

	float v_target = targetSpeedCommand_m_s;
	if (!isfinite(v_target)) {
		v_target = (float)targetSpeed / PALSE_MILLIMETER;
	}
	v_target = clampf(v_target,
		- cascadeControlParams.v_limit,
		  cascadeControlParams.v_limit);

	bool update_outer = false;
	float applied_dt_outer = dt_inner;
	if (outerLoopElapsed >= dt_outer) {
		update_outer = true;
		applied_dt_outer = outerLoopElapsed;
		outerLoopElapsed = 0.0f;
	}

	float yaw_cmd = yawCascadeState.last_output;
	float omega_ref = lineCascadeState.omega_ref;

	if (update_outer) {
		uint16_t left[5] = {0};
		uint16_t right[5] = {0};
		bool valid = read_line_sensor_pairs(left, right);
		float e_line = valid ? compute_line_error(left, right) : 0.0f;

		if (!valid) {
			lineCascadeState.integral = 0.0f;
			lineCascadeState.prev_error = 0.0f;
			lineCascadeState.d_lpf_state = 0.0f;
		}

		if (valid) {
			lineCascadeState.integral = clampf(lineCascadeState.integral + e_line * applied_dt_outer,
			- cascadeControlParams.line.i_limit,
			  cascadeControlParams.line.i_limit);
		} else {
			lineCascadeState.integral = 0.0f;
		}
		float derivative = 0.0f;
		if (applied_dt_outer > 1.0e-6f) {
			derivative = (e_line - lineCascadeState.prev_error) / applied_dt_outer;
		}
		derivative = apply_lowpass(derivative, lineCascadeState.d_lpf_state, applied_dt_outer,
		cascadeControlParams.line.d_lpf);
		lineCascadeState.d_lpf_state = derivative;
		lineCascadeState.prev_error = e_line;

		omega_ref = (cascadeControlParams.line.kp * e_line) +
		(cascadeControlParams.line.ki * lineCascadeState.integral) +
		(cascadeControlParams.line.kd * derivative);
		omega_ref = clampf(omega_ref,
		- cascadeControlParams.line.ref_limit,
		  cascadeControlParams.line.ref_limit);
		lineCascadeState.omega_ref = omega_ref;

		float omega_meas = 0.0f;
		if (initIMU && BMI088val.Initialized) {
			omega_meas = BMI088val.gyro.z * ((float)M_PI / 180.0f);
		}
		float yaw_error = omega_ref - omega_meas;
		yaw_cmd = motorControlYawRate(yaw_error, applied_dt_outer);
	}

	float v_left_ref = clampf(v_target - cascadeControlParams.tread_half * yaw_cmd,
		- cascadeControlParams.v_limit,
		  cascadeControlParams.v_limit);
	float v_right_ref = clampf(v_target + cascadeControlParams.tread_half * yaw_cmd,
		- cascadeControlParams.v_limit,
		  cascadeControlParams.v_limit);

	float v_left_meas = (float)encCurrentL / PALSE_MILLIMETER;
	float v_right_meas = (float)encCurrentR / PALSE_MILLIMETER;

	float duty_left = motorControlSpeed(MOTOR_SIDE_LEFT, v_left_ref, v_left_meas, dt_inner);
	float duty_right = motorControlSpeed(MOTOR_SIDE_RIGHT, v_right_ref, v_right_meas, dt_inner);

	float duty_left_cmd = clampf(duty_left, -1.0f, 1.0f);
	float duty_right_cmd = clampf(duty_right, -1.0f, 1.0f);

	// PWM出力を合成
	motorPwmOutSynth(duty_left_cmd, duty_right_cmd);
	update_legacy_logs(duty_left_cmd, duty_right_cmd, yaw_cmd, omega_ref);
}

#else
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
	if (lSensorMax[0] > lSensorMin[0])
	{
		// マクロで設定した重みを掛け合わせてセンサ値を合成
		senL = (lSensorCari[4] * TRACE_WEIGHT_CENTER) + (lSensorCari[3] * TRACE_WEIGHT_INNER) + (lSensorCari[2] * TRACE_WEIGHT_MIDDLE) + (lSensorCari[1] * TRACE_WEIGHT_OUTER) + (lSensorCari[0] * TRACE_WEIGHT_FAR);
		senR = (lSensorCari[5] * TRACE_WEIGHT_CENTER) + (lSensorCari[6] * TRACE_WEIGHT_INNER) + (lSensorCari[7] * TRACE_WEIGHT_MIDDLE) + (lSensorCari[8] * TRACE_WEIGHT_OUTER) + (lSensorCari[9] * TRACE_WEIGHT_FAR);
	}
	else
	{
		// senL = (lSensor[4]) + (lSensor[3]*0.8) + (lSensor[2]*0.7) + (lSensor[1]*0.5) + (lSensor[0]*0.3);
		// senR = (lSensor[5]) + (lSensor[6]*0.8) + (lSensor[7]*0.7) + (lSensor[8]*0.5) + (lSensor[9]*0.3);
		senL = (lSensor[5]);
		senR = (lSensor[6]);
	}
	Dev = senL - senR;

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
	iRet = iRet;

	// PWMの上限の設定
	if (iRet > 900)
		iRet = 900;
	if (iRet < -900)
		iRet = -900;

	veloCtrl.pwm = iRet;
	speedEncoderBefore = Dev;		      // 次回はこの値が1ms前の値となる
	speedTargetBefore = targetSpeed; // 前回の目標値を記録
}
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
#endif
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
#if USE_CASCADE_TRACE
	// カスケード制御用状態量も初期化
	lineCascadeState.integral = 0.0f;
	lineCascadeState.prev_error = 0.0f;
	lineCascadeState.d_lpf_state = 0.0f;
	lineCascadeState.omega_ref = 0.0f;
	yawCascadeState.integral = 0.0f;
	yawCascadeState.prev_error = 0.0f;
	yawCascadeState.last_output = 0.0f;
	speedCascadeState[MOTOR_SIDE_LEFT].integral = 0.0f;
	speedCascadeState[MOTOR_SIDE_LEFT].prev_error = 0.0f;
	speedCascadeState[MOTOR_SIDE_LEFT].last_output = 0.0f;
	speedCascadeState[MOTOR_SIDE_RIGHT].integral = 0.0f;
	speedCascadeState[MOTOR_SIDE_RIGHT].prev_error = 0.0f;
	speedCascadeState[MOTOR_SIDE_RIGHT].last_output = 0.0f;
	outerLoopElapsed = 0.0f;
#endif
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motorControlYaw
// 処理概要     角速度制御時の制御量の計算
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
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
	uint8_t fileName[20] = PATH_SETTING;
	int16_t ret = 0;

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
	uint8_t fileName[20] = PATH_SETTING;
	TCHAR gain[20];
	int16_t ret = 0;

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
	uint8_t fileName[20] = PATH_SETTING;

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
	uint8_t fileName[20] = PATH_SETTING;
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
