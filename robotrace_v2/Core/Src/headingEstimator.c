#include "headingEstimator.h"
#include <math.h>

#define HEADING_BIAS_PROCESS_VARIANCE_PER_S 0.0004f
#define HEADING_BIAS_MEASUREMENT_VARIANCE   225.0f
#define HEADING_BIAS_MAX_ABS_DPS            1.5f
#define HEADING_BIAS_MIN_ACCEPTED           200U
#define HEADING_RAD_TO_DEG                   57.2957795131f
#define HEADING_DEG_TO_RAD                   0.017453292519943f

/////////////////////////////////////////////////////////////////////
// モジュール名 HeadingBiasEstimator_Reset
// 処理概要     一次ログ用のジャイロ残留バイアス推定を初期化する
// 引数         estimator: 推定状態
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void HeadingBiasEstimator_Reset(HeadingBiasEstimator *estimator)
{
	estimator->biasDps = 0.0f;
	estimator->varianceDps2 = 1.0f;
	estimator->accepted = 0U;
	estimator->rejected = 0U;
	estimator->invalid = false;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 HeadingBiasEstimator_Update
// 処理概要     低曲率区間の左右車輪差でジャイロバイアスを観測する
// 引数         estimator: 推定状態, calibration: 左右車輪校正値,
//              gyroDps: ログ区間平均角速度[deg/s],
//              intervalL/R: ログ区間の左右パルス[pulse],
//              dtSeconds: ログ区間長[s]
// 戻り値       true: 入力と推定状態が正常 false: 異常
/////////////////////////////////////////////////////////////////////
bool HeadingBiasEstimator_Update(HeadingBiasEstimator *estimator,
	const HeadingCalibration *calibration, float gyroDps,
	int16_t intervalL, int16_t intervalR, float dtSeconds)
{
	if (estimator->invalid || calibration->pulsePerMeterL == 0U ||
		calibration->pulsePerMeterR == 0U || calibration->effectiveTreadCentiMm == 0U ||
		!isfinite(gyroDps) || !isfinite(dtSeconds) ||
		dtSeconds <= 0.0f || dtSeconds > 0.1f)
	{
		estimator->invalid = true;
		return false;
	}
	float leftMm = (float)intervalL * 1000.0f / (float)calibration->pulsePerMeterL;
	float rightMm = (float)intervalR * 1000.0f / (float)calibration->pulsePerMeterR;
	float treadMm = (float)calibration->effectiveTreadCentiMm * 0.01f;
	float wheelDps = (leftMm - rightMm) * HEADING_RAD_TO_DEG / (treadMm * dtSeconds);
	float speedMps = (leftMm + rightMm) * 0.0005f / dtSeconds;
	estimator->varianceDps2 += HEADING_BIAS_PROCESS_VARIANCE_PER_S * dtSeconds;
	if (!isfinite(wheelDps) || !isfinite(speedMps) ||
		!isfinite(estimator->varianceDps2) || estimator->varianceDps2 <= 0.0f)
	{
		estimator->invalid = true;
		return false;
	}
	if (speedMps < 0.3f || speedMps > 2.0f ||
		fabsf(gyroDps) > 30.0f || fabsf(wheelDps) > 30.0f)
	{
		return true;
	}
	float innovation = gyroDps - wheelDps - estimator->biasDps;
	float innovationVariance = estimator->varianceDps2 + HEADING_BIAS_MEASUREMENT_VARIANCE;
	if (fabsf(innovation) > 4.0f * sqrtf(innovationVariance))
	{
		estimator->rejected++;
		return true;
	}
	float gain = estimator->varianceDps2 / innovationVariance;
	estimator->biasDps += gain * innovation;
	estimator->varianceDps2 *= 1.0f - gain;
	estimator->accepted++;
	if (!isfinite(estimator->biasDps) || !isfinite(estimator->varianceDps2) ||
		estimator->varianceDps2 <= 0.0f)
	{
		estimator->invalid = true;
		return false;
	}
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 HeadingBiasEstimator_Ready
// 処理概要     一次経路に適用できる推定結果か確認する
// 引数         estimator: 推定状態
// 戻り値       true: 採用条件を満たす false: 不採用
/////////////////////////////////////////////////////////////////////
bool HeadingBiasEstimator_Ready(const HeadingBiasEstimator *estimator)
{
	return !estimator->invalid && estimator->accepted >= HEADING_BIAS_MIN_ACCEPTED &&
		isfinite(estimator->biasDps) && fabsf(estimator->biasDps) <= HEADING_BIAS_MAX_ABS_DPS;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 HeadingPose_Advance
// 処理概要     区間平均角速度と距離を中点方位でXYへ積分する
// 引数         pose: 積分状態, deltaMm: 区間距離[mm],
//              gyroDps: バイアス除去後の区間角速度[deg/s], dtSeconds: 区間長[s]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void HeadingPose_Advance(HeadingPose *pose, float deltaMm,
	float gyroDps, float dtSeconds)
{
	float deltaHeading = gyroDps * dtSeconds;
	float middleRad = (pose->heading_deg + 0.5f * deltaHeading) * HEADING_DEG_TO_RAD;
	pose->x_mm += deltaMm * sinf(middleRad);
	pose->y_mm += deltaMm * cosf(middleRad);
	pose->heading_deg += deltaHeading;
}
