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
	uint32_t invalidUpdateCount;
	float maxAbsFusedDeltaM;
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
// 引数         state: 検証対象の距離推定状態
// 戻り値       全値が有限ならtrue
/////////////////////////////////////////////////////////////////////
static bool estimatorStateIsFinite(const DistanceEstimatorState *state)
{
	for (uint8_t i = 0U; i < 3U; i++)
	{
		if (!isfinite(state->state[i]))
		{
			return false;
		}
		for (uint8_t j = 0U; j < 3U; j++)
		{
			if (!isfinite(state->covariance[i][j]))
			{
				return false;
			}
		}
	}
	return isfinite(state->fusedDeltaM) &&
		isfinite(state->innovationMps) &&
		isfinite(state->innovationSigmaMps) &&
		isfinite(state->maxAbsFusedDeltaM);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 resetCovariance
// 処理概要     距離推定器の共分散を初期値へ戻す
// 引数         state: 初期化対象の距離推定状態
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void resetCovariance(DistanceEstimatorState *state)
{
	for (uint8_t i = 0U; i < 3U; i++)
	{
		for (uint8_t j = 0U; j < 3U; j++)
		{
			state->covariance[i][j] = 0.0F;
		}
	}
	state->covariance[1][1] =
		DISTANCE_ESTIMATOR_SIGMA_ENCODER_MPS * DISTANCE_ESTIMATOR_SIGMA_ENCODER_MPS;
	state->covariance[2][2] =
		DISTANCE_ESTIMATOR_INITIAL_BIAS_SIGMA_MPS2 * DISTANCE_ESTIMATOR_INITIAL_BIAS_SIGMA_MPS2;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 resetVelocityCovariance
// 処理概要     速度状態の共分散と相関を再初期化する
// 引数         state: 初期化対象の距離推定状態
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void resetVelocityCovariance(DistanceEstimatorState *state)
{
	for (uint8_t i = 0U; i < 3U; i++)
	{
		state->covariance[1][i] = 0.0F;
		state->covariance[i][1] = 0.0F;
	}
	state->covariance[1][1] =
		DISTANCE_ESTIMATOR_SIGMA_ENCODER_MPS * DISTANCE_ESTIMATOR_SIGMA_ENCODER_MPS;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 symmetrizeCovariance
// 処理概要     共分散行列の対称性だけを補正する
// 引数         state: 補正対象の距離推定状態
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void symmetrizeCovariance(DistanceEstimatorState *state)
{
	for (uint8_t i = 0U; i < 3U; i++)
	{
		for (uint8_t j = (uint8_t)(i + 1U); j < 3U; j++)
		{
			float average = 0.5F * (state->covariance[i][j] + state->covariance[j][i]);
			state->covariance[i][j] = average;
			state->covariance[j][i] = average;
		}
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 covarianceDiagonalIsValid
// 処理概要     共分散対角要素が有限かつ非負であることを確認する
// 引数         state: 検証対象の距離推定状態
// 戻り値       全対角要素が有効ならtrue
/////////////////////////////////////////////////////////////////////
static bool covarianceDiagonalIsValid(const DistanceEstimatorState *state)
{
	for (uint8_t i = 0U; i < 3U; i++)
	{
		if (!isfinite(state->covariance[i][i]) || state->covariance[i][i] < 0.0F)
		{
			return false;
		}
	}
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 covarianceMatrixIsFinite
// 処理概要     共分散行列の全要素が有限であることを確認する
// 引数         matrix: 検証対象の3x3行列
// 戻り値       全要素が有限ならtrue
/////////////////////////////////////////////////////////////////////
static bool covarianceMatrixIsFinite(const float matrix[3][3])
{
	for (uint8_t i = 0U; i < 3U; i++)
	{
		for (uint8_t j = 0U; j < 3U; j++)
		{
			if (!isfinite(matrix[i][j]))
			{
				return false;
			}
		}
	}
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 recordFusedDelta
// 処理概要     融合後距離差分の最大絶対値を診断値へ記録する
// 引数         state: 記録対象の距離推定状態
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void recordFusedDelta(DistanceEstimatorState *state)
{
	if (!isfinite(state->maxAbsFusedDeltaM) || state->maxAbsFusedDeltaM < 0.0F)
	{
		state->maxAbsFusedDeltaM = 0.0F;
	}
	if (isfinite(state->fusedDeltaM))
	{
		float absoluteDelta = fabsf(state->fusedDeltaM);
		if (absoluteDelta > state->maxAbsFusedDeltaM)
		{
			state->maxAbsFusedDeltaM = absoluteDelta;
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
	resetCovariance(&estimator);
	recordFusedDelta(&estimator);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 updateInvalid
// 処理概要     更新異常時に距離を戻し、生エンコーダ速度へ同期する
// 引数         encoderSpeedMps: 平均エンコーダ速度[m/s]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void updateInvalid(float encoderSpeedMps)
{
	float safeSpeed = isfinite(encoderSpeedMps) ?
		clampFloat(encoderSpeedMps, -DISTANCE_ESTIMATOR_MAX_SPEED_MPS,
			DISTANCE_ESTIMATOR_MAX_SPEED_MPS) : 0.0F;
	float previousDistance = isfinite(estimator.state[0]) ? estimator.state[0] : 0.0F;

	if (estimator.invalidUpdateCount < UINT32_MAX)
	{
		estimator.invalidUpdateCount++;
	}
	estimator.state[0] = previousDistance + safeSpeed * DISTANCE_ESTIMATOR_DT_S;
	estimator.state[1] = safeSpeed;
	estimator.state[2] = 0.0F;
	estimator.fusedDeltaM = estimator.state[0] - previousDistance;
	estimator.innovationMps = 0.0F;
	estimator.innovationSigmaMps = DISTANCE_ESTIMATOR_SIGMA_ENCODER_MPS;
	estimator.fallbackActive = false;
	resetCovariance(&estimator);
	recordFusedDelta(&estimator);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 predict
// 処理概要     加速度とバイアスを用いて状態と共分散を予測する
// 引数         accelerationMps2: 前後加速度[m/s^2]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void predict(DistanceEstimatorState *state, float accelerationMps2)
{
	const float dt = DISTANCE_ESTIMATOR_DT_S;
	const float dt2 = dt * dt;
	const float acceleration = clampFloat(accelerationMps2,
		-DISTANCE_ESTIMATOR_MAX_ACCEL_MPS2, DISTANCE_ESTIMATOR_MAX_ACCEL_MPS2);
	const float effectiveAcceleration = acceleration - state->state[2];
	const float transition[3][3] = {
		{1.0F, dt, -0.5F * dt2},
		{0.0F, 1.0F, -dt},
		{0.0F, 0.0F, 1.0F}
	};
	float predictedState[3];
	float firstProduct[3][3];
	float predictedCovariance[3][3];

	predictedState[0] = state->state[0] + state->state[1] * dt +
		0.5F * effectiveAcceleration * dt2;
	predictedState[1] = state->state[1] + effectiveAcceleration * dt;
	predictedState[2] = state->state[2];

	for (uint8_t i = 0U; i < 3U; i++)
	{
		for (uint8_t j = 0U; j < 3U; j++)
		{
			firstProduct[i][j] = 0.0F;
			for (uint8_t k = 0U; k < 3U; k++)
			{
				firstProduct[i][j] += transition[i][k] * state->covariance[k][j];
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
		state->state[i] = predictedState[i];
		for (uint8_t j = 0U; j < 3U; j++)
		{
			state->covariance[i][j] = predictedCovariance[i][j];
		}
	}
	symmetrizeCovariance(state);
	if (state->state[1] > DISTANCE_ESTIMATOR_MAX_SPEED_MPS ||
		state->state[1] < -DISTANCE_ESTIMATOR_MAX_SPEED_MPS)
	{
		state->state[1] = clampFloat(state->state[1],
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
	DistanceEstimator_ResetState(encoderSpeedMps, false);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 DistanceEstimator_ResetState
// 処理概要     状態と共分散を初期化し、必要に応じて走行診断値を保持する
// 引数         encoderSpeedMps: エンコーダ速度[m/s]
//              preserveDiagnostics: trueなら棄却・異常・最大差分を保持
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void DistanceEstimator_ResetState(float encoderSpeedMps, bool preserveDiagnostics)
{
	uint32_t innovationRejectCount = preserveDiagnostics ? estimator.innovationRejectCount : 0U;
	uint32_t fallbackCount = preserveDiagnostics ? estimator.fallbackCount : 0U;
	uint32_t invalidUpdateCount = preserveDiagnostics ? estimator.invalidUpdateCount : 0U;
	float maxAbsFusedDeltaM = preserveDiagnostics &&
		isfinite(estimator.maxAbsFusedDeltaM) && estimator.maxAbsFusedDeltaM >= 0.0F ?
		estimator.maxAbsFusedDeltaM : 0.0F;

	estimator.state[0] = 0.0F;
	estimator.state[1] = isfinite(encoderSpeedMps) ?
		clampFloat(encoderSpeedMps, -DISTANCE_ESTIMATOR_MAX_SPEED_MPS,
			DISTANCE_ESTIMATOR_MAX_SPEED_MPS) : 0.0F;
	estimator.state[2] = 0.0F;
	estimator.fusedDeltaM = 0.0F;
	estimator.innovationMps = 0.0F;
	estimator.innovationSigmaMps = DISTANCE_ESTIMATOR_SIGMA_ENCODER_MPS;
	estimator.innovationRejectCount = innovationRejectCount;
	estimator.fallbackCount = fallbackCount;
	estimator.invalidUpdateCount = invalidUpdateCount;
	estimator.maxAbsFusedDeltaM = maxAbsFusedDeltaM;
	estimator.fallbackActive = false;
	estimator.initialized = true;
	resetCovariance(&estimator);
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

	if (!imuValid)
	{
		updateFallback(encoderSpeedMps);
		return;
	}

	if (!isfinite(encoderSpeedMps) || !isfinite(accelerationMps2))
	{
		updateInvalid(encoderSpeedMps);
		return;
	}

	DistanceEstimatorState candidate = estimator;
	float previousDistance = candidate.state[0];
	if (!isfinite(previousDistance))
	{
		updateInvalid(encoderSpeedMps);
		return;
	}
	// 更新前の状態・共分散が壊れている場合も予測を実行せず、距離ジャンプを防ぐ。
	if (!estimatorStateIsFinite(&candidate) || !covarianceDiagonalIsValid(&candidate))
	{
		updateInvalid(encoderSpeedMps);
		return;
	}
	if (candidate.fallbackActive)
	{
		// フォールバック中に生エンコーダへ追従していた速度状態を再同期する。
		candidate.state[1] = clampFloat(encoderSpeedMps,
			-DISTANCE_ESTIMATOR_MAX_SPEED_MPS, DISTANCE_ESTIMATOR_MAX_SPEED_MPS);
		resetVelocityCovariance(&candidate);
		candidate.fallbackActive = false;
	}

	predict(&candidate, accelerationMps2);
	if (!estimatorStateIsFinite(&candidate) || !covarianceDiagonalIsValid(&candidate))
	{
		updateInvalid(encoderSpeedMps);
		return;
	}

	float encoderVariance =
		DISTANCE_ESTIMATOR_SIGMA_ENCODER_MPS * DISTANCE_ESTIMATOR_SIGMA_ENCODER_MPS;
	float innovationVariance = candidate.covariance[1][1] + encoderVariance;
	if (!isfinite(innovationVariance) || innovationVariance <= 0.0F)
	{
		updateInvalid(encoderSpeedMps);
		return;
	}

	float innovation = encoderSpeedMps - candidate.state[1];
	if (!isfinite(innovation))
	{
		updateInvalid(encoderSpeedMps);
		return;
	}

	const float initialSpeedVariance = encoderVariance;
	const float maximumGateVariance = initialSpeedVariance + encoderVariance;
	float gateVariance = innovationVariance;
	if (gateVariance > maximumGateVariance)
	{
		gateVariance = maximumGateVariance;
	}
	if (!isfinite(gateVariance) || gateVariance <= 0.0F)
	{
		updateInvalid(encoderSpeedMps);
		return;
	}
	float gateSigma = sqrtf(gateVariance);
	if (!isfinite(gateSigma))
	{
		updateInvalid(encoderSpeedMps);
		return;
	}

	candidate.innovationMps = innovation;
	candidate.innovationSigmaMps = gateSigma;
	// 物理上限を超える観測、または固定された4σを超える観測は、
	// 予測状態だけを採用し、共分散を初期化してゲートを拡大させない。
	if (encoderSpeedMps > DISTANCE_ESTIMATOR_MAX_SPEED_MPS ||
		encoderSpeedMps < -DISTANCE_ESTIMATOR_MAX_SPEED_MPS ||
		fabsf(innovation) > DISTANCE_ESTIMATOR_INNOVATION_GATE_SIGMA * gateSigma)
	{
		if (candidate.innovationRejectCount < UINT32_MAX)
		{
			candidate.innovationRejectCount++;
		}
		candidate.fusedDeltaM = candidate.state[0] - previousDistance;
		if (!isfinite(candidate.fusedDeltaM) ||
			fabsf(candidate.fusedDeltaM) > DISTANCE_ESTIMATOR_MAX_FUSED_DELTA_M)
		{
			updateInvalid(encoderSpeedMps);
			return;
		}
		resetCovariance(&candidate);
		recordFusedDelta(&candidate);
		estimator = candidate;
		return;
	}

	// 更新前共分散を完全に退避し、Joseph形式で別行列へ共分散を算出する。
	// 速度観測の観測行列はH=[0, 1, 0]とする。
	float priorCovariance[3][3];
	for (uint8_t i = 0U; i < 3U; i++)
	{
		for (uint8_t j = 0U; j < 3U; j++)
		{
			priorCovariance[i][j] = candidate.covariance[i][j];
		}
	}
	float gain[3];
	for (uint8_t i = 0U; i < 3U; i++)
	{
		gain[i] = priorCovariance[i][1] / innovationVariance;
	}
	for (uint8_t i = 0U; i < 3U; i++)
	{
		candidate.state[i] += gain[i] * innovation;
	}

	const float observationMatrix[3] = {0.0F, 1.0F, 0.0F};
	float identityMinusKH[3][3];
	float firstJosephProduct[3][3];
	float josephCovariance[3][3];
	for (uint8_t i = 0U; i < 3U; i++)
	{
		for (uint8_t j = 0U; j < 3U; j++)
		{
			identityMinusKH[i][j] = (i == j ? 1.0F : 0.0F) -
				gain[i] * observationMatrix[j];
		}
	}
	for (uint8_t i = 0U; i < 3U; i++)
	{
		for (uint8_t j = 0U; j < 3U; j++)
		{
			firstJosephProduct[i][j] = 0.0F;
			for (uint8_t k = 0U; k < 3U; k++)
			{
				firstJosephProduct[i][j] += identityMinusKH[i][k] *
					priorCovariance[k][j];
			}
		}
	}
	for (uint8_t i = 0U; i < 3U; i++)
	{
		for (uint8_t j = 0U; j < 3U; j++)
		{
			josephCovariance[i][j] = gain[i] * encoderVariance * gain[j];
			for (uint8_t k = 0U; k < 3U; k++)
			{
				josephCovariance[i][j] += firstJosephProduct[i][k] *
					identityMinusKH[j][k];
			}
		}
	}
	if (!covarianceMatrixIsFinite(josephCovariance))
	{
		updateInvalid(encoderSpeedMps);
		return;
	}
	for (uint8_t i = 0U; i < 3U; i++)
	{
		for (uint8_t j = 0U; j < 3U; j++)
		{
			candidate.covariance[i][j] = josephCovariance[i][j];
		}
	}
	symmetrizeCovariance(&candidate);
	candidate.state[1] = clampFloat(candidate.state[1],
		-DISTANCE_ESTIMATOR_MAX_SPEED_MPS, DISTANCE_ESTIMATOR_MAX_SPEED_MPS);
	candidate.fusedDeltaM = candidate.state[0] - previousDistance;

	if (!estimatorStateIsFinite(&candidate) || !covarianceDiagonalIsValid(&candidate) ||
		!isfinite(candidate.fusedDeltaM) ||
		fabsf(candidate.fusedDeltaM) > DISTANCE_ESTIMATOR_MAX_FUSED_DELTA_M)
	{
		updateInvalid(encoderSpeedMps);
		return;
	}
	recordFusedDelta(&candidate);
	estimator = candidate;
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
		float correctedDistance = estimator.state[0] + correctionM;
		if (isfinite(correctedDistance))
		{
			estimator.state[0] = correctedDistance;
		}
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
		.invalidUpdateCount = estimator.invalidUpdateCount,
		.maxAbsFusedDeltaM = estimator.maxAbsFusedDeltaM,
		.fallbackActive = estimator.fallbackActive};
	return diagnostics;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 DistanceEstimator_GetInnovationRejectCount
// 処理概要     4σ超過または物理上限超過で観測更新を完全スキップした回数を取得する
// 引数         なし
// 戻り値       観測更新を完全スキップした回数
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
