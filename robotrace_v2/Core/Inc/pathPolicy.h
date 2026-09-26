#ifndef PATH_POLICY_H_
#define PATH_POLICY_H_

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

static inline bool pathPolicyAssociationAllowed(float candidateArcMm,
	float maximumArcMm, float headingErrorDeg, float maximumHeadingErrorDeg)
{
	return isfinite(candidateArcMm) && isfinite(maximumArcMm) &&
		isfinite(headingErrorDeg) && candidateArcMm <= maximumArcMm &&
		fabsf(headingErrorDeg) <= maximumHeadingErrorDeg;
}

static inline bool pathPolicyPoseCorrectionAllowed(float positionCorrectionMm,
	float headingCorrectionDeg, float maximumPositionCorrectionMm,
	float maximumHeadingCorrectionDeg)
{
	return isfinite(positionCorrectionMm) && isfinite(headingCorrectionDeg) &&
		positionCorrectionMm >= 0.0F && maximumPositionCorrectionMm >= 0.0F &&
		fabsf(headingCorrectionDeg) <= maximumHeadingCorrectionDeg &&
	positionCorrectionMm <= maximumPositionCorrectionMm;
}

static inline bool pathPolicyComputeGoalExtension(float endXmm, float endYmm,
	float spacingMm, uint16_t availablePoints, float *goalXmm, float *goalYmm,
	float *extensionMm, uint16_t *appendCount)
{
	if (goalXmm == NULL || goalYmm == NULL || extensionMm == NULL || appendCount == NULL ||
		!isfinite(endXmm) || !isfinite(endYmm) || !isfinite(spacingMm) || spacingMm <= 0.0F)
	{
		return false;
	}
	float distanceToOriginMm = hypotf(endXmm, endYmm);
	float requiredExtensionMm = distanceToOriginMm * 0.5F;
	if (!isfinite(requiredExtensionMm) || requiredExtensionMm < 1.0F) return false;
	float requiredPointsF = ceilf(requiredExtensionMm / spacingMm);
	if (!isfinite(requiredPointsF) || requiredPointsF < 1.0F || requiredPointsF > availablePoints ||
		requiredPointsF > UINT16_MAX)
	{
		return false;
	}
	*goalXmm = endXmm * 0.5F;
	*goalYmm = endYmm * 0.5F;
	*extensionMm = requiredExtensionMm;
	*appendCount = (uint16_t)requiredPointsF;
	return true;
}

static inline bool pathPolicySourceMetadataValid(bool versionSupported,
	bool closureValid, uint32_t closureReason, bool primaryMode, bool normalStop,
	uint32_t pulsePerMeter, bool imuCalibrationValid, uint32_t imuCalibrationSamples,
	uint32_t imuCalibrationReadErrors, bool distanceScaleVerified, uint32_t expectedRows)
{
	return versionSupported && closureValid && closureReason == 0U && primaryMode && normalStop &&
		pulsePerMeter == 58019U && imuCalibrationValid && imuCalibrationSamples == 100U &&
		imuCalibrationReadErrors == 0U && distanceScaleVerified && expectedRows > 0U;
}

static inline bool pathPolicyGoalReached(bool goalValid, uint16_t routeIndex,
	uint16_t routeCount, uint16_t currentArcMm, uint16_t goalArcMm)
{
	return goalValid && routeIndex < routeCount && currentArcMm >= goalArcMm;
}

static inline uint8_t pathPolicyGenerationLevel(bool requestShortcut, uint8_t configuredMaxLevel)
{
	return requestShortcut ? configuredMaxLevel : 0U;
}

static inline bool pathPolicyGoalRequest(bool pathMode, bool markerGoalReached,
	bool pathGoalReached)
{
	return pathMode ? pathGoalReached : markerGoalReached;
}

static inline bool pathPolicyMergeSparseSettings(int *values, const bool *present,
	const int *defaults, const int *minimums, const int *maximums, size_t count)
{
	bool repaired = false;
	for (size_t i = 0U; i < count; i++)
	{
		if (!present[i] || values[i] < minimums[i] || values[i] > maximums[i])
		{
			values[i] = defaults[i];
			repaired = true;
		}
	}
	return repaired;
}

#endif // PATH_POLICY_H_
