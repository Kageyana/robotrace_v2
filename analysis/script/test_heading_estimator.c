#include "headingEstimator.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
	HeadingCalibration calibration = {1U, 56687U, 56687U, 10900U};
	HeadingBiasEstimator estimator;
	HeadingBiasEstimator_Reset(&estimator);
	// 10msに左右約10mm進む直線では、残差0.24deg/sを推定できる。
	for (int i = 0; i < 3000; i++)
	{
		assert(HeadingBiasEstimator_Update(&estimator, &calibration,
			0.24f, 567, 567, 0.01f));
	}
	assert(estimator.accepted == 3000U);
	assert(HeadingBiasEstimator_Ready(&estimator));
	assert(fabsf(estimator.biasDps - 0.24f) < 0.03f);
	// 車輪差から求める高角速度区間はバイアス観測に混ぜない。
	uint32_t before = estimator.accepted;
	assert(HeadingBiasEstimator_Update(&estimator, &calibration,
		200.0f, 675, 459, 0.01f));
	assert(estimator.accepted == before);
	// 時刻の欠落は経路生成を無効にする。
	assert(!HeadingBiasEstimator_Update(&estimator, &calibration,
		0.0f, 567, 567, 0.0f));
	assert(!HeadingBiasEstimator_Ready(&estimator));
	// 左右でパルス/mが少し異なり、記録間隔が揺れても正常な観測を使える。
	calibration.pulsePerMeterR = 56500U;
	HeadingBiasEstimator_Reset(&estimator);
	for (int i = 0; i < 1000; i++)
	{
		float dt = (i % 2 == 0) ? 0.009f : 0.011f;
		int16_t left = (int16_t)lroundf(56687.0f * dt);
		int16_t right = (int16_t)lroundf(56500.0f * dt);
		assert(HeadingBiasEstimator_Update(&estimator, &calibration,
			0.24f, left, right, dt));
	}
	assert(estimator.accepted == 1000U);
	assert(fabsf(estimator.biasDps - 0.24f) < 0.2f);
	// 実測の左右尺度差を個別換算して直進の見かけの角速度を除く。
	calibration.pulsePerMeterL = 55036U;
	calibration.pulsePerMeterR = 55196U;
	calibration.effectiveTreadCentiMm = 10900U;
	HeadingBiasEstimator_Reset(&estimator);
	for (int i = 0; i < 3000; i++)
	{
		float dt = (i % 2 == 0) ? 0.05f : 0.06f;
		int16_t left = (int16_t)lroundf(55036.0f * dt);
		int16_t right = (int16_t)lroundf(55196.0f * dt);
		assert(HeadingBiasEstimator_Update(&estimator, &calibration,
			0.24f, left, right, dt));
	}
	assert(estimator.accepted == 3000U);
	assert(HeadingBiasEstimator_Ready(&estimator));
	assert(fabsf(estimator.biasDps - 0.24f) < 0.2f);
	// 観測域の端でも4σを超える不整合は棄却する。
	calibration.pulsePerMeterL = 56687U;
	calibration.pulsePerMeterR = 56687U;
	HeadingBiasEstimator_Reset(&estimator);
	estimator.biasDps = -1.5f;
	assert(HeadingBiasEstimator_Update(&estimator, &calibration,
		30.0f, 551, 583, 0.01f));
	assert(estimator.rejected == 1U && estimator.accepted == 0U);
	assert(!HeadingBiasEstimator_Update(&estimator, &calibration,
		0.0f, 567, 567, 0.101f));
	assert(estimator.invalid);
	// 速度・角速度・採取間隔が変動しても中点方位積分は倍精度基準に一致する。
	HeadingPose pose = {0.0f, 0.0f, 0.0f};
	HeadingPose raw = {0.0f, 0.0f, 0.0f};
	double referenceX = 0.0, referenceY = 0.0, referenceHeading = 0.0;
	for (int i = 0; i < 400; i++)
	{
		float dt = 0.007f + (float)(i % 9) * 0.001f;
		float deltaMm = (0.5f + 0.2f * sinf((float)i * 0.07f)) * 1000.0f * dt;
		float angularDps = 20.0f * sinf((float)i * 0.03f);
		HeadingPose_Advance(&pose, deltaMm, angularDps, dt);
		HeadingPose_Advance(&raw, deltaMm, angularDps + 0.24f, dt);
		double deltaHeading = (double)angularDps * (double)dt;
		double middleRad = (referenceHeading + 0.5 * deltaHeading) * 0.017453292519943295;
		referenceX += (double)deltaMm * sin(middleRad);
		referenceY += (double)deltaMm * cos(middleRad);
		referenceHeading += deltaHeading;
	}
	assert(fabs((double)pose.x_mm - referenceX) < 0.05);
	assert(fabs((double)pose.y_mm - referenceY) < 0.05);
	assert(fabs((double)pose.heading_deg - referenceHeading) < 0.001);
	assert(fabsf(raw.heading_deg - pose.heading_deg) > 0.8f);
	// ログ点間のマーカー比率を変えても走行積分結果は変わらない。
	HeadingPose beforePose = pose;
	HeadingPose_Advance(&pose, 10.0f, 1.0f, 0.01f);
	float markerX1 = beforePose.x_mm + 0.25f * (pose.x_mm - beforePose.x_mm);
	float markerX2 = beforePose.x_mm + 0.75f * (pose.x_mm - beforePose.x_mm);
	assert(fabsf(markerX1 - markerX2) > 0.00001f);
	assert(fabsf(pose.x_mm - beforePose.x_mm) > 0.00001f);
	puts("heading estimator: ok");
	return 0;
}
