//====================================//
// インクルード
//====================================//
#include "IMU.h"
#include <math.h>
//====================================//
// グローバル変数の定義
//====================================//
bool calibratIMU = false;		// IMUキャリブレーション中フラグ
volatile IMUval imuVal = {0};	// IMUの実行時変数（加速度、角速度、角度などを保持）
float angleOffset[3] = {0.0F, 0.0F, 0.0F};	// ジャイロオフセット[deg/s]（calibrationIMU()で算出される）
#ifdef USE_ACCELE
float acceleOffset[3] = {0.0F, 0.0F, 0.0F};	// 加速度オフセット[g]（calibrationIMU()で算出される）
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

	// ジャイロ補正（物理量オフセット除去後に方向係数を適用）
	imuVal.gyro.x = (BMI088val.gyro.x - angleOffset[0]) * COEFF_DPD;
	imuVal.gyro.y = (BMI088val.gyro.y - angleOffset[1]) * COEFF_DPD;
	imuVal.gyro.z = (BMI088val.gyro.z - angleOffset[2]) * COEFF_DPD;

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
// モジュール名 calibrationIMU
// 処理概要     ジャイロと加速度のオフセットを校正する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void calibrationIMU(void)
{
	static uint16_t i = 0;
	static float angleInt[3];
#ifdef USE_ACCELE
	static float acceleInt[3];
#endif

	if(!BMI088val.Initialized)
	{
		return;
	}

	if (i < (uint32_t)(1.0 / DEFF_TIME))
	{
		// ジャイロの物理量を積算
		BMI088getGyro();
		angleInt[0] += BMI088val.gyro.x;
		angleInt[1] += BMI088val.gyro.y;
		angleInt[2] += BMI088val.gyro.z;
#ifdef USE_ACCELE
		// 加速度の物理量を積算
		BMI088getAccele();
		acceleInt[0] += BMI088val.accele.x;
		acceleInt[1] += BMI088val.accele.y;
		acceleInt[2] += BMI088val.accele.z;
#endif
		i++;
	}
	else
	{
		angleOffset[0] = angleInt[0] / i;
		angleOffset[1] = angleInt[1] / i;
		angleOffset[2] = angleInt[2] / i;
		angleInt[0] = 0;
		angleInt[1] = 0;
		angleInt[2] = 0;
#ifdef USE_ACCELE
		// 平均加速度から重力成分を差し引いてオフセットを算出
		float acceleAvgX = acceleInt[0] / i;
		float acceleAvgY = acceleInt[1] / i;
		float acceleAvgZ = acceleInt[2] / i;
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
		acceleInt[0] = 0;
		acceleInt[1] = 0;
		acceleInt[2] = 0;
#endif
		i = 0;
		calibratIMU = false;
	}
}
