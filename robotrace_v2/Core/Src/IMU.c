//====================================//
// インクルード
//====================================//
#include "IMU.h"
#include <math.h>
#include <string.h>
//====================================//
// グローバル変数の定義
//====================================//
bool calibratIMU = false;		// IMUキャリブレーション中フラグ
static volatile bool imuCalibrationResetRequested = false;
static volatile bool imuCalibrationReady = false;
static volatile uint16_t imuCalibrationSamples = 0U;
static volatile uint16_t imuCalibrationReadErrors = 0U;
float imuTempCoeff_dpsPerC = 0.0F;
bool imuTempCalibrationValid = false;
float imuTempCalibrationStart_C = BMI088_TEMP_INVALID_C;
float imuTempCalibration_C = BMI088_TEMP_INVALID_C;
float imuTempCalibrationEnd_C = BMI088_TEMP_INVALID_C;
uint16_t imuTempCalibrationSamples = 0U;
uint16_t imuTempCalibrationReadErrors = 0U;
bool imuTempCorrectionEnabled = false;
float imuTempEnd_C = BMI088_TEMP_INVALID_C;
static float imuTempCalibrationSum_C = 0.0F;
static uint16_t imuTempCalibrationAttempts = 0U;
volatile IMUval imuVal = {0};	// IMUの実行時変数（加速度、角速度、角度などを保持）
float angleOffset[3] = {0.0F, 0.0F, 0.0F};	// ジャイロオフセット[deg/s]（calibrationIMU()で算出される）
#ifdef USE_ACCELE
float acceleOffset[3] = {0.0F, 0.0F, 0.0F};	// 加速度オフセット[g]（calibrationIMU()で算出される）
static float gravityReference[3] = {0.0F, 0.0F, 0.0F};	// 静止時の重力基準ベクトル[g]
#ifdef USE_IMU_ROT_CENTER_CORRECTION
static float prevGyroZRad = 0.0F;		// 角加速度算出用の前回ジャイロz値[rad/s]
static float alphaZFiltered = 0.0F;		// 角加速度のLPF後値[rad/s^2]
static bool gyroZInitialized = false;	// ジャイロzの初期化フラグ
#endif
#endif

//====================================//
// ローカル関数
//====================================//
#ifdef USE_ACCELE
#ifdef USE_IMU_ROT_CENTER_CORRECTION
static void applyRotCenterCorrectionIMU(void);
#endif
#endif
static void captureImuTempCalibration(void);
static void finalizeImuTempCalibration(void);
/////////////////////////////////////////////////////////////////////
// モジュール名 applyRotCenterCorrectionIMU
// 処理概要     旋回中心ずれによる加速度成分を2D(yaw軸まわり)で補正する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
#ifdef USE_ACCELE
#ifdef USE_IMU_ROT_CENTER_CORRECTION
static void applyRotCenterCorrectionIMU(void)
{
	// IMUが旋回中心からずれていると、加速度には接線加速度項 α×r と向心加速度項 ω×(ω×r) が混ざる
	// この関数ではヨー軸まわりの2D近似でx/yのみ補正する
	// 補正オフセットはrawセンサ軸ではなく、applyOffsetIMU()後のimuVal座標系基準で与える
	// ジャイロz[deg/s]を[rad/s]へ変換する
	float omegaZ = imuVal.gyro.z * DEG2RAD;
	// 角加速度[rad/s^2]（初回は0扱い）
	float alphaZ = 0.0F;
	// 旋回中心ずれにより加算する補正加速度[m/s^2]
	float corrX_mps2 = 0.0F;
	float corrY_mps2 = 0.0F;

	if(gyroZInitialized)
	{
		// 差分から角加速度を算出する
		alphaZ = (omegaZ - prevGyroZRad) / DEFF_TIME;
	}
	else
	{
		// 初回は差分が取れないため次回以降用の初期化のみ行う
		gyroZInitialized = true;
	}

	// 角加速度ノイズ低減のため1次LPFを適用する
	alphaZFiltered = IMU_ALPHA_LPF_COEF * alphaZFiltered + (1.0F - IMU_ALPHA_LPF_COEF) * alphaZ;

	// 接線加速度項(α×r)と向心加速度項(ω×(ω×r))を2Dで合成する
	corrX_mps2 = alphaZFiltered * IMU_OFFSET_Y_M + omegaZ * omegaZ * IMU_OFFSET_X_M;
	corrY_mps2 = -alphaZFiltered * IMU_OFFSET_X_M + omegaZ * omegaZ * IMU_OFFSET_Y_M;

	// [m/s^2]から[g]へ戻してimuValの加速度x/yへ反映する（zは補正しない）
	imuVal.accele.x += corrX_mps2 / GRAVITY_MPS2;
	imuVal.accele.y += corrY_mps2 / GRAVITY_MPS2;

	// 次回の角加速度算出用に現在値を保持する
	prevGyroZRad = omegaZ;
}
#endif
#endif
/////////////////////////////////////////////////////////////////////
// モジュール名 applyOffsetIMU
// 処理概要     BMI088の出力値にオフセット補正を適用し、imuValを更新する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void applyOffsetIMU(void)
{
	imuVal.Aid = BMI088val.Aid;
	imuVal.Gid = BMI088val.Gid;
	imuVal.Initialized = BMI088val.Initialized;
	imuVal.temp = BMI088val.temp;
	imuVal.tempRaw = BMI088val.tempRaw;
	imuVal.tempValid = BMI088val.tempValid;

	// ジャイロ補正（物理量オフセット除去後に方向係数を適用）
	imuVal.gyro.x = (BMI088val.gyro.x - angleOffset[0]) * COEFF_DPD;
	imuVal.gyro.y = (BMI088val.gyro.y - angleOffset[1]) * COEFF_DPD;
	float gyroZ = BMI088val.gyro.z;
	if (imuTempCorrectionEnabled)
	{
		if (!BMI088val.tempValid || !isfinite(BMI088val.temp))
		{
			// 走行中に温度が無効になった場合は、その走行の補正を停止する。
			imuTempCorrectionEnabled = false;
		}
		else
		{
			gyroZ -= imuTempCoeff_dpsPerC *
				(BMI088val.temp - imuTempCalibration_C);
		}
	}
	imuVal.gyro.z = (gyroZ - angleOffset[2]) * COEFF_DPD;

#ifdef USE_ACCELE
	// 加速度補正（物理量オフセットをそのまま除去）
	imuVal.accele.x = BMI088val.accele.x - acceleOffset[0];
	imuVal.accele.y = BMI088val.accele.y - acceleOffset[1];
	imuVal.accele.z = BMI088val.accele.z - acceleOffset[2];
#ifdef USE_IMU_ROT_CENTER_CORRECTION
	applyRotCenterCorrectionIMU();
#endif
#else
	imuVal.accele = BMI088val.accele;
#endif
}
/////////////////////////////////////////////////////////////////////
// モジュール名 calcDegrees
// 処理概要     ジャイロと加速度から角度を算出する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void calcDegrees(void)
{
	if(!BMI088val.Initialized)
	{
		return;
	}

	// オフセット補正はIMUモジュール側で実施
	applyOffsetIMU();

#ifdef USE_ACCELE
	// 加速度からpitch/roll角を推定
	volatile float pitchAccele = atan2f(imuVal.accele.y, imuVal.accele.z) * 180.0f / M_PI;
	volatile float rollAccele = atan2f(imuVal.accele.x, sqrtf(imuVal.accele.y * imuVal.accele.y + imuVal.accele.z * imuVal.accele.z)) * 180.0f / M_PI;

	// ドリフト低減のためコンプリメンタリフィルタを適用
	imuVal.angle.x = COEFF_COMPFILTER * imuVal.angle.x + (1.0f - COEFF_COMPFILTER) * pitchAccele;
	imuVal.angle.y = COEFF_COMPFILTER * imuVal.angle.y + (1.0f - COEFF_COMPFILTER) * rollAccele;
#else
	// ジャイロ積分で角度を更新
	imuVal.angle.x += imuVal.gyro.x * DEFF_TIME; // X軸角度(pitch)
	imuVal.angle.y += imuVal.gyro.y * DEFF_TIME; // Y軸角度(roll)
#endif

	// Z軸角度(yaw)は積分のみ
	imuVal.angle.z += imuVal.gyro.z * DEFF_TIME;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 calcVelocity
// 処理概要     加速度から速度を算出する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void calcVelocity(void)
{
#ifdef USE_ACCELE
	if(!BMI088val.Initialized)
	{
		return;
	}

	// この実装は、calcDegrees()内でapplyOffsetIMU()を実行後のimuValを積分する前提
	imuVal.velo.x += imuVal.accele.x * 9.81f * DEFF_TIME;
	imuVal.velo.y += imuVal.accele.y * 9.81f * DEFF_TIME;
	imuVal.velo.z += imuVal.accele.z * 9.81f * DEFF_TIME;
#endif
}
/////////////////////////////////////////////////////////////////////
// モジュール名 IMU_GetLinearAccelerationXMps2
// 処理概要     静止時重力基準を除去した機体X軸線形加速度を取得する
// 引数         なし
// 戻り値       X軸線形加速度[m/s^2]
/////////////////////////////////////////////////////////////////////
float IMU_GetLinearAccelerationXMps2(void)
{
#ifdef USE_ACCELE
	return (imuVal.accele.x - gravityReference[0]) * GRAVITY_MPS2;
#else
	return 0.0F;
#endif
}
/////////////////////////////////////////////////////////////////////
// モジュール名 IMU_GetLinearAccelerationYMps2
// 処理概要     静止時重力基準を除去した機体Y軸線形加速度を取得する
// 引数         なし
// 戻り値       Y軸線形加速度[m/s^2]
/////////////////////////////////////////////////////////////////////
float IMU_GetLinearAccelerationYMps2(void)
{
#ifdef USE_ACCELE
	return (imuVal.accele.y - gravityReference[1]) * GRAVITY_MPS2;
#else
	return 0.0F;
#endif
}
/////////////////////////////////////////////////////////////////////
// モジュール名 IMU_GetLinearAccelerationZMps2
// 処理概要     静止時重力基準を除去した機体Z軸線形加速度を取得する
// 引数         なし
// 戻り値       Z軸線形加速度[m/s^2]
/////////////////////////////////////////////////////////////////////
float IMU_GetLinearAccelerationZMps2(void)
{
#ifdef USE_ACCELE
	return (imuVal.accele.z - gravityReference[2]) * GRAVITY_MPS2;
#else
	return 0.0F;
#endif
}
/////////////////////////////////////////////////////////////////////
// モジュール名 IMU_GetForwardAccelerationMps2
// 処理概要     Y軸線形加速度を前後加速度として取得する互換ラッパー
// 引数         なし
// 戻り値       前後加速度[m/s^2]
/////////////////////////////////////////////////////////////////////
float IMU_GetForwardAccelerationMps2(void)
{
	return IMU_GetLinearAccelerationYMps2();
}
/////////////////////////////////////////////////////////////////////
// モジュール名 clearIMUval
// 処理概要     IMUの実行時変数を初期化する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void clearIMUval(void)
{
	imuVal.accele.x = 0.0F;
	imuVal.accele.y = 0.0F;
	imuVal.accele.z = 0.0F;

	imuVal.velo.x = 0.0F;
	imuVal.velo.y = 0.0F;
	imuVal.velo.z = 0.0F;

	imuVal.gyro.x = 0.0F;
	imuVal.gyro.y = 0.0F;
	imuVal.gyro.z = 0.0F;

	imuVal.angle.x = 0.0F;
	imuVal.angle.y = 0.0F;
	imuVal.angle.z = 0.0F;

#ifdef USE_ACCELE
#ifdef USE_IMU_ROT_CENTER_CORRECTION
	prevGyroZRad = 0.0F;
	alphaZFiltered = 0.0F;
	gyroZInitialized = false;
#endif
#endif
}

/////////////////////////////////////////////////////////////////////
// モジュール名 IMU_SetTempCompensationCoefficient
// 処理概要     SDカードから読み込んだBMI088温度係数を実行時値へ設定する
// 引数         coeffX1000000: 温度係数を1,000,000倍した値
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void IMU_SetTempCompensationCoefficient(int32_t coeffX1000000)
{
	if (coeffX1000000 < IMU_TEMP_COEFF_MIN_X1000000 ||
		coeffX1000000 > IMU_TEMP_COEFF_MAX_X1000000)
	{
		coeffX1000000 = 0;
	}
	imuTempCoeff_dpsPerC = (float)coeffX1000000 /
		(float)IMU_TEMP_COEFF_SCALE;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 updateImuTempEndTemperature
// 処理概要     ログヘッダー用に終了時点の有効なBMI088温度を保存する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void updateImuTempEndTemperature(void)
{
	if (BMI088getTemp() && BMI088val.tempValid && isfinite(BMI088val.temp))
	{
		imuTempEnd_C = BMI088val.temp;
	}
	else
	{
		imuTempEnd_C = BMI088_TEMP_INVALID_C;
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 captureImuTempCalibration
// 処理概要     IMU校正周期に合わせて温度を取得し、統計値を更新する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void captureImuTempCalibration(void)
{
	if (imuTempCalibrationAttempts >= IMU_TEMP_CALIBRATION_TOTAL_SAMPLES)
	{
		return;
	}
	imuTempCalibrationAttempts++;

	if (BMI088getTemp() && BMI088val.tempValid && isfinite(BMI088val.temp))
	{
		if (imuTempCalibrationSamples == 0U)
		{
			imuTempCalibrationStart_C = BMI088val.temp;
		}
		imuTempCalibrationEnd_C = BMI088val.temp;
		imuTempCalibrationSum_C += BMI088val.temp;
		imuTempCalibrationSamples++;
	}
	else if (imuTempCalibrationReadErrors < UINT16_MAX)
	{
		imuTempCalibrationReadErrors++;
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 finalizeImuTempCalibration
// 処理概要     2秒間の温度統計を確定し、温度補正の有効状態を更新する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void finalizeImuTempCalibration(void)
{
	if (imuTempCalibrationSamples > 0U)
	{
		imuTempCalibration_C = imuTempCalibrationSum_C /
			(float)imuTempCalibrationSamples;
	}
	else
	{
		imuTempCalibrationStart_C = BMI088_TEMP_INVALID_C;
		imuTempCalibration_C = BMI088_TEMP_INVALID_C;
		imuTempCalibrationEnd_C = BMI088_TEMP_INVALID_C;
	}
	imuTempCalibrationValid =
		imuTempCalibrationSamples >= IMU_TEMP_CALIBRATION_MIN_VALID_SAMPLES;
	imuTempCorrectionEnabled = imuTempCalibrationValid &&
		imuTempCoeff_dpsPerC != 0.0F;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 calibrationIMU
// 処理概要     ジャイロと加速度のオフセットを校正する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void calibrationIMU(void)
{
	static uint16_t sampleCount = 0;
	static uint16_t sampleIntervalMs = 0;
	static float angleInt[3];
#ifdef USE_ACCELE
	static float acceleInt[3];
#endif
	if (imuCalibrationResetRequested)
	{
		sampleCount = 0U;
		sampleIntervalMs = 0U;
		memset(angleInt, 0, sizeof(angleInt));
#ifdef USE_ACCELE
		memset(acceleInt, 0, sizeof(acceleInt));
#endif
		imuTempCalibrationAttempts = 0U;
		imuTempCalibrationSum_C = 0.0F;
		imuCalibrationResetRequested = false;
	}

	if(!BMI088val.Initialized)
	{
		if (imuCalibrationReadErrors < UINT16_MAX) imuCalibrationReadErrors++;
		return;
	}

	sampleIntervalMs++;
	if (sampleIntervalMs < IMU_CALIBRATION_SAMPLE_INTERVAL_MS)
	{
		return;
	}
	sampleIntervalMs = 0;

	// 20msごとにジャイロの物理量を積算する
	if (!BMI088getGyro() || !isfinite(BMI088val.gyro.x) ||
		!isfinite(BMI088val.gyro.y) || !isfinite(BMI088val.gyro.z))
	{
		if (imuCalibrationReadErrors < UINT16_MAX) imuCalibrationReadErrors++;
		return;
	}
#ifdef USE_ACCELE
	// 加速度の物理量を積算する
	if (!BMI088getAccele() || !isfinite(BMI088val.accele.x) ||
		!isfinite(BMI088val.accele.y) || !isfinite(BMI088val.accele.z))
	{
		if (imuCalibrationReadErrors < UINT16_MAX) imuCalibrationReadErrors++;
		return;
	}
	acceleInt[0] += BMI088val.accele.x;
	acceleInt[1] += BMI088val.accele.y;
	acceleInt[2] += BMI088val.accele.z;
#endif
	angleInt[0] += BMI088val.gyro.x;
	angleInt[1] += BMI088val.gyro.y;
	angleInt[2] += BMI088val.gyro.z;
	sampleCount++;
	imuCalibrationSamples = sampleCount;
	if ((sampleCount % IMU_TEMP_CALIBRATION_INTERVAL_SAMPLES) == 0U)
	{
		captureImuTempCalibration();
	}
	if (sampleCount < IMU_CALIBRATION_SAMPLE_COUNT)
	{
		return;
	}

	angleOffset[0] = angleInt[0] / sampleCount;
	angleOffset[1] = angleInt[1] / sampleCount;
	angleOffset[2] = angleInt[2] / sampleCount;
	angleInt[0] = 0;
	angleInt[1] = 0;
	angleInt[2] = 0;
#ifdef USE_ACCELE
	// 平均加速度から静止時の重力基準ベクトルを保存する。
	float acceleAvgX = acceleInt[0] / sampleCount;
	float acceleAvgY = acceleInt[1] / sampleCount;
	float acceleAvgZ = acceleInt[2] / sampleCount;
	float gravityScale = sqrtf((acceleAvgX * acceleAvgX) + (acceleAvgY * acceleAvgY) + (acceleAvgZ * acceleAvgZ));
	float gravityCompX = 0.0f;
	float gravityCompY = 0.0f;
	float gravityCompZ = 0.0f;

	if (gravityScale > 0.0f)
	{
		float normCoef = 1.0f / gravityScale;
		gravityCompX = acceleAvgX * normCoef;
		gravityCompY = acceleAvgY * normCoef;
		gravityCompZ = acceleAvgZ * normCoef;
	}
	gravityReference[0] = gravityCompX;
	gravityReference[1] = gravityCompY;
	gravityReference[2] = gravityCompZ;

	// センサー固有のオフセットを除去し、既存の姿勢推定用には1gの重力を残す。
	acceleOffset[0] = acceleAvgX - gravityCompX;
	acceleOffset[1] = acceleAvgY - gravityCompY;
	acceleOffset[2] = acceleAvgZ - gravityCompZ;
	acceleInt[0] = 0;
	acceleInt[1] = 0;
	acceleInt[2] = 0;
#endif
	finalizeImuTempCalibration();
	sampleCount = 0;
	sampleIntervalMs = 0;
	imuCalibrationReady = imuCalibrationReadErrors == 0U &&
		imuCalibrationSamples == IMU_CALIBRATION_SAMPLE_COUNT &&
		isfinite(angleOffset[0]) && isfinite(angleOffset[1]) && isfinite(angleOffset[2]);
	calibratIMU = false;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 IMU_StartCalibration
// 処理概要     IMU校正の診断値と積算状態をリセットして開始する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void IMU_StartCalibration(void)
{
	imuCalibrationReady = false;
	imuCalibrationSamples = 0U;
	imuCalibrationReadErrors = 0U;
	imuTempCalibrationValid = false;
	imuTempCalibrationStart_C = BMI088_TEMP_INVALID_C;
	imuTempCalibration_C = BMI088_TEMP_INVALID_C;
	imuTempCalibrationEnd_C = BMI088_TEMP_INVALID_C;
	imuTempCalibrationSamples = 0U;
	imuTempCalibrationReadErrors = 0U;
	imuTempCalibrationSum_C = 0.0F;
	imuTempCalibrationAttempts = 0U;
	imuTempCorrectionEnabled = false;
	imuTempEnd_C = BMI088_TEMP_INVALID_C;
	imuCalibrationResetRequested = true;
	calibratIMU = true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 IMU_CalibrationReady
// 処理概要     直近のIMU校正が読出し異常なしで完了したか返す
// 引数         なし
// 戻り値       true:有効 false:未完了または異常
/////////////////////////////////////////////////////////////////////
bool IMU_CalibrationReady(void) { return imuCalibrationReady; }

/////////////////////////////////////////////////////////////////////
// モジュール名 IMU_CalibrationSamples
// 処理概要     直近のIMU校正で採用したサンプル数を返す
// 引数         なし
// 戻り値       採用サンプル数
/////////////////////////////////////////////////////////////////////
uint16_t IMU_CalibrationSamples(void) { return imuCalibrationSamples; }

/////////////////////////////////////////////////////////////////////
// モジュール名 IMU_CalibrationReadErrors
// 処理概要     直近のIMU校正で検出した読出し異常回数を返す
// 引数         なし
// 戻り値       読出し異常回数
/////////////////////////////////////////////////////////////////////
uint16_t IMU_CalibrationReadErrors(void) { return imuCalibrationReadErrors; }
