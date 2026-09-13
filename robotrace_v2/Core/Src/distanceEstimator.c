#include "distanceEstimator.h"

#include <math.h>
#include <stdint.h>

typedef struct
{
	float state[3];
	float covariance[3][3];
	float fusedDeltaM;
	float innovationMps;
	float innovationSigmaMps;
	uint32_t innovationRejectCount;
	uint32_t fallbackCount;
	bool fallbackActive;
	bool initialized;
} DistanceEstimatorState;

static DistanceEstimatorState estimator;

/////////////////////////////////////////////////////////////////////
// モジュール名 clampFloat
// 処理概要     浮動小数点値を指定範囲へ制限する
// 引数         value: 対象値, minimum: 下限, maximum: 上限
// 戻り値       制限後の値
/////////////////////////////////////////////////////////////////////
static float clampFloat(float value, float minimum, float maximum)
{
	if (value < minimum)
	{
		return minimum;
	}
	if (value > maximum)
	{
		return maximum;
	}
	return value;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 estimatorStateIsFinite
// 処理概要     距離推定器の状態、共分散、診断値の有限性を確認する
// 引数         なし
// 戻り値       全値が有限ならtrue
/////////////////////////////////////////////////////////////////////
static bool estimatorStateIsFinite(void)
{
	for (uint8_t i = 0U; i < 3U; i++)
	{
		if (!isfinite(estimator.state[i]))
		{
			return false;
		}
		for (uint8_t j = 0U; j < 3U; j++)
		{
			if (!isfinite(estimator.covariance[i][j]))
			{
				return false;
			}
		}
	}
	return isfinite(estimator.fusedDeltaM) &&
		isfinite(estimator.innovationMps) &&
		isfinite(estimator.innovationSigmaMps);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 resetCovariance
// 処理概要     距離推定器の共分散を初期値へ戻す
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void resetCovariance(void)
{
	for (uint8_t i = 0U; i < 3U; i++)
	{
		for (uint8_t j = 0U; j < 3U; j++)
		{
			estimator.covariance[i][j] = 0.0F;
		}
	}
	estimator.covariance[1][1] =
		DISTANCE_ESTIMATOR_SIGMA_ENCODER_MPS * DISTANCE_ESTIMATOR_SIGMA_ENCODER_MPS;
	estimator.covariance[2][2] =
		DISTANCE_ESTIMATOR_INITIAL_BIAS_SIGMA_MPS2 * DISTANCE_ESTIMATOR_INITIAL_BIAS_SIGMA_MPS2;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 resetVelocityCovariance
// 処理概要     速度状態の共分散と相関を再初期化する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void resetVelocityCovariance(void)
{
	for (uint8_t i = 0U; i < 3U; i++)
	{
		estimator.covariance[1][i] = 0.0F;
		estimator.covariance[i][1] = 0.0F;
	}
	estimator.covariance[1][1] =
		DISTANCE_ESTIMATOR_SIGMA_ENCODER_MPS * DISTANCE_ESTIMATOR_SIGMA_ENCODER_MPS;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 symmetrizeCovariance
// 処理概要     共分散行列の対称性と対角要素の非負性を補正する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void symmetrizeCovariance(void)
{
	for (uint8_t i = 0U; i < 3U; i++)
	{
		if (estimator.covariance[i][i] < 0.0F || !isfinite(estimator.covariance[i][i]))
		{
			estimator.covariance[i][i] = 0.0F;
		}
		for (uint8_t j = (uint8_t)(i + 1U); j < 3U; j++)
		{
			float average = 0.5F * (estimator.covariance[i][j] + estimator.covariance[j][i]);
			estimator.covariance[i][j] = average;
			estimator.covariance[j][i] = average;
		}
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 updateFallback
// 処理概要     IMU無効時に生エンコーダだけで距離状態を更新する
// 引数         encoderSpeedMps: 平均エンコーダ速度[m/s]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void updateFallback(float encoderSpeedMps)
{
	float safeSpeed = isfinite(encoderSpeedMps) ?
		clampFloat(encoderSpeedMps, -DISTANCE_ESTIMATOR_MAX_SPEED_MPS,
			DISTANCE_ESTIMATOR_MAX_SPEED_MPS) : 0.0F;
	float previousDistance = isfinite(estimator.state[0]) ? estimator.state[0] : 0.0F;

	if (!estimator.fallbackActive && estimator.fallbackCount < UINT32_MAX)
	{
		estimator.fallbackCount++;
	}
	estimator.fallbackActive = true;
	estimator.state[0] = previousDistance + safeSpeed * DISTANCE_ESTIMATOR_DT_S;
	estimator.state[1] = safeSpeed;
	estimator.state[2] = 0.0F;
	estimator.fusedDeltaM = estimator.state[0] - previousDistance;
	estimator.innovationMps = 0.0F;
	estimator.innovationSigmaMps = DISTANCE_ESTIMATOR_SIGMA_ENCODER_MPS;
	resetCovariance();
}

/////////////////////////////////////////////////////////////////////
// モジュール名 predict
// 処理概要     加速度とバイアスを用いて状態と共分散を予測する
// 引数         accelerationMps2: 前後加速度[m/s^2]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void predict(float accelerationMps2)
{
	const float dt = DISTANCE_ESTIMATOR_DT_S;
	const float dt2 = dt * dt;
	const float acceleration = clampFloat(accelerationMps2,
		-DISTANCE_ESTIMATOR_MAX_ACCEL_MPS2, DISTANCE_ESTIMATOR_MAX_ACCEL_MPS2);
	const float effectiveAcceleration = acceleration - estimator.state[2];
	const float transition[3][3] = {
		{1.0F, dt, -0.5F * dt2},
		{0.0F, 1.0F, -dt},
		{0.0F, 0.0F, 1.0F}
	};
	float predictedState[3];
	float firstProduct[3][3];
	float predictedCovariance[3][3];

	predictedState[0] = estimator.state[0] + estimator.state[1] * dt +
		0.5F * effectiveAcceleration * dt2;
	predictedState[1] = estimator.state[1] + effectiveAcceleration * dt;
	predictedState[2] = estimator.state[2];

	for (uint8_t i = 0U; i < 3U; i++)
	{
		for (uint8_t j = 0U; j < 3U; j++)
		{
			firstProduct[i][j] = 0.0F;
			for (uint8_t k = 0U; k < 3U; k++)
			{
				firstProduct[i][j] += transition[i][k] * estimator.covariance[k][j];
			}
		}
	}
	for (uint8_t i = 0U; i < 3U; i++)
	{
		for (uint8_t j = 0U; j < 3U; j++)
		{
			predictedCovariance[i][j] = 0.0F;
			for (uint8_t k = 0U; k < 3U; k++)
			{
				predictedCovariance[i][j] += firstProduct[i][k] * transition[j][k];
			}
		}
	}

	const float processAccelerationVariance =
		DISTANCE_ESTIMATOR_SIGMA_ACCEL_MPS2 * DISTANCE_ESTIMATOR_SIGMA_ACCEL_MPS2;
	const float processBiasVariance =
		DISTANCE_ESTIMATOR_BIAS_RANDOM_WALK_MPS2_SQRT_S *
		DISTANCE_ESTIMATOR_BIAS_RANDOM_WALK_MPS2_SQRT_S * dt;
	const float noiseVector[3] = {0.5F * dt2, dt, 0.0F};
	for (uint8_t i = 0U; i < 3U; i++)
	{
		for (uint8_t j = 0U; j < 3U; j++)
		{
			predictedCovariance[i][j] += processAccelerationVariance *
				noiseVector[i] * noiseVector[j];
		}
	}
	predictedCovariance[2][2] += processBiasVariance;

	for (uint8_t i = 0U; i < 3U; i++)
	{
		estimator.state[i] = predictedState[i];
		for (uint8_t j = 0U; j < 3U; j++)
		{
			estimator.covariance[i][j] = predictedCovariance[i][j];
		}
	}
	symmetrizeCovariance();
	if (estimator.state[1] > DISTANCE_ESTIMATOR_MAX_SPEED_MPS ||
		estimator.state[1] < -DISTANCE_ESTIMATOR_MAX_SPEED_MPS)
	{
		estimator.state[1] = clampFloat(estimator.state[1],
			-DISTANCE_ESTIMATOR_MAX_SPEED_MPS, DISTANCE_ESTIMATOR_MAX_SPEED_MPS);
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 DistanceEstimator_Initialize
// 処理概要     距離推定器を初期化する
// 引数         initialSpeedMps: 初期速度[m/s]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void DistanceEstimator_Initialize(float initialSpeedMps)
{
	DistanceEstimator_Reset(initialSpeedMps);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 DistanceEstimator_Reset
// 処理概要     走行開始または校正完了時に距離推定器を再初期化する
// 引数         encoderSpeedMps: エンコーダ速度[m/s]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void DistanceEstimator_Reset(float encoderSpeedMps)
{
	estimator.state[0] = 0.0F;
	estimator.state[1] = isfinite(encoderSpeedMps) ?
		clampFloat(encoderSpeedMps, -DISTANCE_ESTIMATOR_MAX_SPEED_MPS,
			DISTANCE_ESTIMATOR_MAX_SPEED_MPS) : 0.0F;
	estimator.state[2] = 0.0F;
	estimator.fusedDeltaM = 0.0F;
	estimator.innovationMps = 0.0F;
	estimator.innovationSigmaMps = DISTANCE_ESTIMATOR_SIGMA_ENCODER_MPS;
	estimator.innovationRejectCount = 0U;
	estimator.fallbackCount = 0U;
	estimator.fallbackActive = false;
	estimator.initialized = true;
	resetCovariance();
}

/////////////////////////////////////////////////////////////////////
// モジュール名 DistanceEstimator_Update
// 処理概要     加速度予測とエンコーダ速度観測更新を実行する
// 引数         encoderSpeedMps: 平均エンコーダ速度[m/s]
//              accelerationMps2: 前後加速度[m/s^2]
//              imuValid: 加速度入力の有効フラグ
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void DistanceEstimator_Update(float encoderSpeedMps, float accelerationMps2, bool imuValid)
{
	if (!estimator.initialized)
	{
		DistanceEstimator_Initialize(encoderSpeedMps);
	}

	if (!imuValid || !isfinite(encoderSpeedMps) || !isfinite(accelerationMps2))
	{
		updateFallback(encoderSpeedMps);
		return;
	}

	float previousDistance = estimator.state[0];
	if (estimator.fallbackActive)
	{
		// フォールバック中に生エンコーダへ追従していた速度状態を再同期する。
		estimator.state[1] = clampFloat(encoderSpeedMps,
			-DISTANCE_ESTIMATOR_MAX_SPEED_MPS, DISTANCE_ESTIMATOR_MAX_SPEED_MPS);
		resetVelocityCovariance();
		estimator.fallbackActive = false;
	}

	predict(accelerationMps2);
	if (!estimatorStateIsFinite())
	{
		updateFallback(encoderSpeedMps);
		return;
	}

	float innovation = encoderSpeedMps - estimator.state[1];
	float innovationVariance = estimator.covariance[1][1] +
		(DISTANCE_ESTIMATOR_SIGMA_ENCODER_MPS * DISTANCE_ESTIMATOR_SIGMA_ENCODER_MPS);
	if (!isfinite(innovation) || !isfinite(innovationVariance) || innovationVariance <= 0.0F)
	{
		updateFallback(encoderSpeedMps);
		return;
	}

	float varianceScale = 1.0F;
	float normalizedInnovationSquared = (innovation * innovation) / innovationVariance;
	if (normalizedInnovationSquared >
		(DISTANCE_ESTIMATOR_INNOVATION_GATE_SIGMA * DISTANCE_ESTIMATOR_INNOVATION_GATE_SIGMA))
	{
		varianceScale = normalizedInnovationSquared /
			(DISTANCE_ESTIMATOR_INNOVATION_GATE_SIGMA * DISTANCE_ESTIMATOR_INNOVATION_GATE_SIGMA);
		if (varianceScale > DISTANCE_ESTIMATOR_MAX_MEASUREMENT_VARIANCE_SCALE)
		{
			varianceScale = DISTANCE_ESTIMATOR_MAX_MEASUREMENT_VARIANCE_SCALE;
		}
		if (estimator.innovationRejectCount < UINT32_MAX)
		{
			estimator.innovationRejectCount++;
		}
	}
	innovationVariance = estimator.covariance[1][1] +
		(DISTANCE_ESTIMATOR_SIGMA_ENCODER_MPS * DISTANCE_ESTIMATOR_SIGMA_ENCODER_MPS) * varianceScale;
	if (!isfinite(innovationVariance) || innovationVariance <= 0.0F)
	{
		updateFallback(encoderSpeedMps);
		return;
	}

	float gain[3];
	for (uint8_t i = 0U; i < 3U; i++)
	{
		gain[i] = estimator.covariance[i][1] / innovationVariance;
	}
	for (uint8_t i = 0U; i < 3U; i++)
	{
		estimator.state[i] += gain[i] * innovation;
	}
	float priorCovariance[3][3];
	for (uint8_t i = 0U; i < 3U; i++)
	{
		for (uint8_t j = 0U; j < 3U; j++)
		{
			priorCovariance[i][j] = estimator.covariance[i][j];
			estimator.covariance[i][j] -= gain[i] * priorCovariance[1][j];
		}
	}
	symmetrizeCovariance();
	estimator.state[1] = clampFloat(estimator.state[1],
		-DISTANCE_ESTIMATOR_MAX_SPEED_MPS, DISTANCE_ESTIMATOR_MAX_SPEED_MPS);
	estimator.innovationMps = innovation;
	estimator.innovationSigmaMps = sqrtf(innovationVariance);
	estimator.fusedDeltaM = estimator.state[0] - previousDistance;

	if (!estimatorStateIsFinite())
	{
		updateFallback(encoderSpeedMps);
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 DistanceEstimator_GetFusedDeltaM
// 処理概要     直前の更新で推定した距離差分を取得する
// 引数         なし
// 戻り値       融合後の距離差分[m]
/////////////////////////////////////////////////////////////////////
float DistanceEstimator_GetFusedDeltaM(void)
{
	return estimator.fusedDeltaM;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 DistanceEstimator_GetDistanceM
// 処理概要     融合後の累積距離を取得する
// 引数         なし
// 戻り値       累積距離[m]
/////////////////////////////////////////////////////////////////////
float DistanceEstimator_GetDistanceM(void)
{
	return estimator.state[0];
}

/////////////////////////////////////////////////////////////////////
// モジュール名 DistanceEstimator_GetVelocityMps
// 処理概要     融合後の速度を取得する
// 引数         なし
// 戻り値       速度[m/s]
/////////////////////////////////////////////////////////////////////
float DistanceEstimator_GetVelocityMps(void)
{
	return estimator.state[1];
}

/////////////////////////////////////////////////////////////////////
// モジュール名 DistanceEstimator_ApplyDistanceCorrectionM
// 処理概要     マーカー補正量を距離状態へ反映する
// 引数         correctionM: 距離補正量[m]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void DistanceEstimator_ApplyDistanceCorrectionM(float correctionM)
{
	if (isfinite(correctionM) && isfinite(estimator.state[0]))
	{
		estimator.state[0] += correctionM;
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 DistanceEstimator_GetDiagnostics
// 処理概要     距離推定器の診断値を取得する
// 引数         なし
// 戻り値       距離推定器診断値
/////////////////////////////////////////////////////////////////////
DistanceEstimatorDiagnostics DistanceEstimator_GetDiagnostics(void)
{
	DistanceEstimatorDiagnostics diagnostics = {
		.distance_m = estimator.state[0],
		.velocity_mps = estimator.state[1],
		.accelerationBias_mps2 = estimator.state[2],
		.innovation_mps = estimator.innovationMps,
		.innovationSigma_mps = estimator.innovationSigmaMps,
		.innovationRejectCount = estimator.innovationRejectCount,
		.fallbackCount = estimator.fallbackCount,
		.fallbackActive = estimator.fallbackActive};
	return diagnostics;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 DistanceEstimator_GetInnovationRejectCount
// 処理概要     イノベーション棄却相当の観測回数を取得する
// 引数         なし
// 戻り値       観測分散を増加させた回数
/////////////////////////////////////////////////////////////////////
uint32_t DistanceEstimator_GetInnovationRejectCount(void)
{
	return estimator.innovationRejectCount;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 DistanceEstimator_GetFallbackCount
// 処理概要     IMU無効フォールバックへ遷移した回数を取得する
// 引数         なし
// 戻り値       フォールバック遷移回数
/////////////////////////////////////////////////////////////////////
uint32_t DistanceEstimator_GetFallbackCount(void)
{
	return estimator.fallbackCount;
}
