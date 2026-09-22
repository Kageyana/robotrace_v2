//====================================//
// インクルード
//====================================//
#include "IMU.h"
#include "SDcard.h"
#include "fatfs.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
//====================================//
// グローバル変数の定義
//====================================//
bool calibratIMU = false;		// IMUキャリブレーション中フラグ
volatile IMUval imuVal = {0};	// IMUの実行時変数（加速度、角速度、角度などを保持）
float angleOffset[3] = {0.0F, 0.0F, 0.0F};	// ジャイロオフセット[deg/s]（calibrationIMU()で算出される）
float imuTempCoeff_dpsPerC = 0.0F;	// ジャイロZ温度係数[deg/s/°C]
bool imuTempCalibrationValid = false;	// 走行前温度校正の有効状態
float imuTempCalibrationStart_C = BMI088_TEMP_INVALID_C;	// 最初の有効温度[°C]
float imuTempCalibration_C = BMI088_TEMP_INVALID_C;	// 走行前校正温度[°C]
float imuTempCalibrationEnd_C = BMI088_TEMP_INVALID_C;	// 最後の有効温度[°C]
uint16_t imuTempCalibrationSamples = 0U;	// 有効温度サンプル数
uint16_t imuTempCalibrationReadErrors = 0U;	// 無効温度サンプル数
float imuTempEnd_C = BMI088_TEMP_INVALID_C;	// ログ終了時温度[°C]
bool imuTempCorrectionEnabled = false;	// 走行中の温度補正有効状態
static uint16_t imuCalibrationSampleCount = 0U;
static float imuAngleInt[3] = {0.0F, 0.0F, 0.0F};
static float imuTempCalibrationSum_C = 0.0F;
static bool imuTempCalibrationHasValue = false;
#ifdef USE_ACCELE
float acceleOffset[3] = {0.0F, 0.0F, 0.0F};	// 加速度オフセット[g]（calibrationIMU()で算出される）
static float imuAcceleInt[3] = {0.0F, 0.0F, 0.0F};
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
		if (!BMI088val.tempValid)
		{
			// 走行中に温度が無効になった場合は、その走行中は従来処理へ戻す。
			imuTempCorrectionEnabled = false;
		}
		else
		{
			gyroZ -= imuTempCoeff_dpsPerC * (BMI088val.temp - imuTempCalibration_C);
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
// モジュール名 startCalibrationIMU
// 処理概要     IMUの2秒間校正と温度統計を開始する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void startCalibrationIMU(void)
{
	if (calibratIMU)
	{
		return;
	}

	imuCalibrationSampleCount = 0U;
	imuAngleInt[0] = 0.0F;
	imuAngleInt[1] = 0.0F;
	imuAngleInt[2] = 0.0F;
#ifdef USE_ACCELE
	imuAcceleInt[0] = 0.0F;
	imuAcceleInt[1] = 0.0F;
	imuAcceleInt[2] = 0.0F;
#endif
	imuTempCalibrationSum_C = 0.0F;
	imuTempCalibrationHasValue = false;
	imuTempCalibrationValid = false;
	imuTempCalibrationStart_C = BMI088_TEMP_INVALID_C;
	imuTempCalibration_C = BMI088_TEMP_INVALID_C;
	imuTempCalibrationEnd_C = BMI088_TEMP_INVALID_C;
	imuTempCalibrationSamples = 0U;
	imuTempCalibrationReadErrors = 0U;
	imuTempCorrectionEnabled = false;
	imuTempEnd_C = BMI088_TEMP_INVALID_C;
	calibratIMU = true;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 calibrationIMU
// 処理概要     ジャイロと加速度のオフセットを2秒間校正し、温度統計を取得する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void calibrationIMU(void)
{
	if(!BMI088val.Initialized)
	{
		return;
	}

	BMI088getGyro();
	imuAngleInt[0] += BMI088val.gyro.x;
	imuAngleInt[1] += BMI088val.gyro.y;
	imuAngleInt[2] += BMI088val.gyro.z;
#ifdef USE_ACCELE
	BMI088getAccele();
	imuAcceleInt[0] += BMI088val.accele.x;
	imuAcceleInt[1] += BMI088val.accele.y;
	imuAcceleInt[2] += BMI088val.accele.z;
#endif
	imuCalibrationSampleCount++;

	if ((imuCalibrationSampleCount % IMU_TEMP_CALIBRATION_INTERVAL_SAMPLES) == 0U)
	{
		BMI088getTemp();
		if (BMI088val.tempValid)
		{
			if (!imuTempCalibrationHasValue)
			{
				imuTempCalibrationStart_C = BMI088val.temp;
				imuTempCalibrationHasValue = true;
			}
			imuTempCalibrationEnd_C = BMI088val.temp;
			imuTempCalibrationSum_C += BMI088val.temp;
			imuTempCalibrationSamples++;
		}
		else
		{
			imuTempCalibrationReadErrors++;
		}
	}

	if (imuCalibrationSampleCount >= IMU_CALIBRATION_SAMPLE_COUNT)
	{
		angleOffset[0] = imuAngleInt[0] / (float)imuCalibrationSampleCount;
		angleOffset[1] = imuAngleInt[1] / (float)imuCalibrationSampleCount;
		angleOffset[2] = imuAngleInt[2] / (float)imuCalibrationSampleCount;
#ifdef USE_ACCELE
		// 平均加速度から重力成分を差し引いてオフセットを算出
		float acceleAvgX = imuAcceleInt[0] / (float)imuCalibrationSampleCount;
		float acceleAvgY = imuAcceleInt[1] / (float)imuCalibrationSampleCount;
		float acceleAvgZ = imuAcceleInt[2] / (float)imuCalibrationSampleCount;
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

		acceleOffset[0] = acceleAvgX - gravityCompX;
		acceleOffset[1] = acceleAvgY - gravityCompY;
		acceleOffset[2] = acceleAvgZ - gravityCompZ;
#endif
		captureImuTempCalibration();
		imuCalibrationSampleCount = 0U;
		imuAngleInt[0] = 0.0F;
		imuAngleInt[1] = 0.0F;
		imuAngleInt[2] = 0.0F;
#ifdef USE_ACCELE
		imuAcceleInt[0] = 0.0F;
		imuAcceleInt[1] = 0.0F;
		imuAcceleInt[2] = 0.0F;
#endif
		calibratIMU = false;
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 readImuTempCompensation
// 処理概要     SDカードからBMI088温度係数を読み出し、異常時は0へ修復する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void readImuTempCompensation(void)
{
	const char *fileName = PATH_SETTING "imu_temp.txt";
	int32_t coefficientX1000000 = 0;
	bool valid = false;
	FIL file;
	FRESULT result;

	imuTempCoeff_dpsPerC = 0.0F;
	if (!initMSD)
	{
		return;
	}

	result = f_open(&file, fileName, FA_OPEN_EXISTING | FA_READ);
	if (result == FR_OK)
	{
		char buffer[32] = {0};
		UINT bytesRead = 0;
		result = f_read(&file, buffer, sizeof(buffer) - 1U, &bytesRead);
		f_close(&file);
		if (result == FR_OK && bytesRead < sizeof(buffer) - 1U)
		{
			char *end = NULL;
			long parsed = strtol(buffer, &end, 10);
			while (end != NULL && (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n'))
			{
				end++;
			}
			if (end != buffer && end != NULL && *end == '\0' &&
				parsed >= IMU_TEMP_COEFF_MIN_X1000000 && parsed <= IMU_TEMP_COEFF_MAX_X1000000)
			{
				coefficientX1000000 = (int32_t)parsed;
				valid = true;
			}
		}
	}

	if (!valid)
	{
		// 欠落・破損・範囲外は、既定値を保存して次回の再発を防ぐ。
		result = f_open(&file, fileName, FA_CREATE_ALWAYS | FA_WRITE);
		if (result == FR_OK)
		{
			f_printf(&file, "%ld", (long)coefficientX1000000);
			f_sync(&file);
			f_close(&file);
		}
	}

	imuTempCoeff_dpsPerC = (float)coefficientX1000000 / (float)IMU_TEMP_COEFF_SCALE;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 captureImuTempCalibration
// 処理概要     2秒間の有効温度平均と統計から補正基準を確定する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void captureImuTempCalibration(void)
{
	if (imuTempCalibrationSamples > 0U)
	{
		imuTempCalibration_C = imuTempCalibrationSum_C / (float)imuTempCalibrationSamples;
	}
	else
	{
		imuTempCalibrationStart_C = BMI088_TEMP_INVALID_C;
		imuTempCalibration_C = BMI088_TEMP_INVALID_C;
		imuTempCalibrationEnd_C = BMI088_TEMP_INVALID_C;
	}
	imuTempCalibrationValid =
		imuTempCalibrationSamples >= IMU_TEMP_CALIBRATION_MIN_VALID_SAMPLES;
	imuTempCorrectionEnabled =
		imuTempCalibrationValid && imuTempCoeff_dpsPerC != 0.0F;
	imuTempEnd_C = BMI088_TEMP_INVALID_C;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 updateImuTempEndTemperature
// 処理概要     ログ終了時のBMI088温度を保存する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void updateImuTempEndTemperature(void)
{
	imuTempEnd_C = BMI088val.tempValid ? BMI088val.temp : BMI088_TEMP_INVALID_C;
}
