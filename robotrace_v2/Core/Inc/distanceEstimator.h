#ifndef DISTANCE_ESTIMATOR_H_
#define DISTANCE_ESTIMATOR_H_

#include <stdbool.h>
#include <stdint.h>

// 距離推定器の単位と制限。
#define DISTANCE_ESTIMATOR_DT_S                         0.001F
#define DISTANCE_ESTIMATOR_SIGMA_ACCEL_MPS2             1.5F
#define DISTANCE_ESTIMATOR_SIGMA_ENCODER_MPS            0.08F
#define DISTANCE_ESTIMATOR_BIAS_RANDOM_WALK_MPS2_SQRT_S 0.05F
#define DISTANCE_ESTIMATOR_INITIAL_BIAS_SIGMA_MPS2      0.5F
#define DISTANCE_ESTIMATOR_MAX_ACCEL_MPS2               20.0F
#define DISTANCE_ESTIMATOR_MAX_SPEED_MPS                10.0F
#define DISTANCE_ESTIMATOR_INNOVATION_GATE_SIGMA       4.0F
#define DISTANCE_ESTIMATOR_MAX_FUSED_DELTA_M \
	(DISTANCE_ESTIMATOR_MAX_SPEED_MPS * DISTANCE_ESTIMATOR_DT_S + \
	0.5F * DISTANCE_ESTIMATOR_MAX_ACCEL_MPS2 * \
	DISTANCE_ESTIMATOR_DT_S * DISTANCE_ESTIMATOR_DT_S)
#define DISTANCE_ESTIMATOR_MAX_FUSED_DELTA_P 535

typedef struct
{
	float distance_m;
	float velocity_mps;
	float accelerationBias_mps2;
	float innovation_mps;
	float innovationSigma_mps;
	uint32_t innovationRejectCount; // 4σ超過または物理上限超過による観測更新スキップ回数
	uint32_t fallbackCount;
	uint32_t invalidUpdateCount; // 有限性、共分散、距離差分の検証失敗回数
	float maxAbsFusedDeltaM; // 融合後1ms距離差分の最大絶対値[m]
	bool fallbackActive;
} DistanceEstimatorDiagnostics;

/////////////////////////////////////////////////////////////////////
// モジュール名 DistanceEstimator_Initialize
// 処理概要     距離推定器の状態と共分散を初期化する
// 引数         initialSpeedMps: 初期速度[m/s]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void DistanceEstimator_Initialize(float initialSpeedMps);

/////////////////////////////////////////////////////////////////////
// モジュール名 DistanceEstimator_Reset
// 処理概要     走行開始または加速度校正完了時に距離推定器を再初期化する
// 引数         encoderSpeedMps: エンコーダ速度[m/s]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void DistanceEstimator_Reset(float encoderSpeedMps);

/////////////////////////////////////////////////////////////////////
// モジュール名 DistanceEstimator_ResetState
// 処理概要     状態と共分散を初期化し、必要に応じて走行診断値を保持する
// 引数         encoderSpeedMps: エンコーダ速度[m/s]
//              preserveDiagnostics: trueなら棄却・異常・最大差分を保持
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void DistanceEstimator_ResetState(float encoderSpeedMps, bool preserveDiagnostics);

/////////////////////////////////////////////////////////////////////
// モジュール名 DistanceEstimator_Update
// 処理概要     加速度予測とエンコーダ速度観測更新を1ms周期で実行する
// 引数         encoderSpeedMps: 平均エンコーダ速度[m/s]
//              accelerationMps2: 重力除去後の前後加速度[m/s^2]
//              imuValid: 加速度入力が有効であるか
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void DistanceEstimator_Update(float encoderSpeedMps, float accelerationMps2, bool imuValid);

/////////////////////////////////////////////////////////////////////
// モジュール名 DistanceEstimator_GetFusedDeltaM
// 処理概要     直前の1ms更新で推定した移動距離差分を取得する
// 引数         なし
// 戻り値       融合後の移動距離[m]
/////////////////////////////////////////////////////////////////////
float DistanceEstimator_GetFusedDeltaM(void);

/////////////////////////////////////////////////////////////////////
// モジュール名 DistanceEstimator_GetDistanceM
// 処理概要     融合後の累積距離を取得する
// 引数         なし
// 戻り値       累積距離[m]
/////////////////////////////////////////////////////////////////////
float DistanceEstimator_GetDistanceM(void);

/////////////////////////////////////////////////////////////////////
// モジュール名 DistanceEstimator_GetVelocityMps
// 処理概要     融合後の速度を取得する
// 引数         なし
// 戻り値       速度[m/s]
/////////////////////////////////////////////////////////////////////
float DistanceEstimator_GetVelocityMps(void);

/////////////////////////////////////////////////////////////////////
// モジュール名 DistanceEstimator_ApplyDistanceCorrectionM
// 処理概要     マーカー補正量を内部距離状態へ反映する
// 引数         correctionM: 距離補正量[m]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void DistanceEstimator_ApplyDistanceCorrectionM(float correctionM);

/////////////////////////////////////////////////////////////////////
// モジュール名 DistanceEstimator_GetDiagnostics
// 処理概要     推定器の状態と診断値を取得する
// 引数         なし
// 戻り値       距離推定器診断値
/////////////////////////////////////////////////////////////////////
DistanceEstimatorDiagnostics DistanceEstimator_GetDiagnostics(void);

uint32_t DistanceEstimator_GetInnovationRejectCount(void);
uint32_t DistanceEstimator_GetFallbackCount(void);

#endif // DISTANCE_ESTIMATOR_H_
