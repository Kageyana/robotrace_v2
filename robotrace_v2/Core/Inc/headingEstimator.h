#ifndef HEADING_ESTIMATOR_H_
#define HEADING_ESTIMATOR_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
	uint8_t enabled;
	uint32_t pulsePerMeterL;
	uint32_t pulsePerMeterR;
	uint16_t effectiveTreadCentiMm;
} HeadingCalibration;

typedef struct
{
	float biasDps;
	float varianceDps2;
	uint32_t accepted;
	uint32_t rejected;
	bool invalid;
} HeadingBiasEstimator;

typedef struct
{
	float x_mm;
	float y_mm;
	float heading_deg;
} HeadingPose;

void HeadingBiasEstimator_Reset(HeadingBiasEstimator *estimator);
bool HeadingBiasEstimator_Update(HeadingBiasEstimator *estimator,
	const HeadingCalibration *calibration, float gyroDps,
	int16_t intervalL, int16_t intervalR, float dtSeconds);
bool HeadingBiasEstimator_Ready(const HeadingBiasEstimator *estimator);
void HeadingPose_Advance(HeadingPose *pose, float deltaMm,
	float gyroDps, float dtSeconds);

#endif // HEADING_ESTIMATOR_H_
