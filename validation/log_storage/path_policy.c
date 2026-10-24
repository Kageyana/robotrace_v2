#include "pathPolicy.h"

#include <stdio.h>
#include <stdlib.h>

#define CHECK(condition) \
	do \
	{ \
		if (!(condition)) \
		{ \
			fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
			exit(EXIT_FAILURE); \
		} \
	} while (0)

void testPathPolicy(void)
{
	CHECK(pathPolicyAssociationAllowed(500.0F, 620.0F, 5.0F, 60.0F));
	CHECK(!pathPolicyAssociationAllowed(2000.0F, 1120.0F, 0.0F, 60.0F));
	CHECK(!pathPolicyAssociationAllowed(1000.0F, 1120.0F, 179.0F, 60.0F));

	CHECK(pathPolicyPoseCorrectionAllowed(1.0F, 0.1F, 1.0F, 0.1F));
	CHECK(!pathPolicyPoseCorrectionAllowed(1.01F, 0.0F, 1.0F, 0.1F));
	CHECK(!pathPolicyPoseCorrectionAllowed(0.5F, 0.11F, 1.0F, 0.1F));

	float goalX = 0.0F;
	float goalY = 0.0F;
	float extensionMm = 0.0F;
	uint16_t appendCount = 0U;
	CHECK(pathPolicyComputeGoalExtension(300.0F, 400.0F, 40.0F, 7U,
		&goalX, &goalY, &extensionMm, &appendCount));
	CHECK(goalX == 150.0F && goalY == 200.0F);
	CHECK(extensionMm == 250.0F && appendCount == 7U);
	CHECK(!pathPolicyComputeGoalExtension(300.0F, 400.0F, 40.0F, 6U,
		&goalX, &goalY, &extensionMm, &appendCount));
	CHECK(!pathPolicyComputeGoalExtension(0.5F, 0.0F, 40.0F, 10U,
		&goalX, &goalY, &extensionMm, &appendCount));

	CHECK(pathPolicySourceMetadataValid(true, true, 0U, true, true,
		58019U, true, 100U, 0U, true, 2000U));
	CHECK(!pathPolicySourceMetadataValid(false, true, 0U, true, true,
		58019U, true, 100U, 0U, true, 2000U));
	CHECK(!pathPolicySourceMetadataValid(true, false, 2U, true, true,
		58019U, true, 100U, 0U, true, 2000U));
	CHECK(!pathPolicySourceMetadataValid(true, true, 0U, false, true,
		58019U, true, 100U, 0U, true, 2000U));
	CHECK(!pathPolicySourceMetadataValid(true, true, 0U, true, true,
		58019U, true, 99U, 0U, true, 2000U));
	CHECK(!pathPolicySourceMetadataValid(true, true, 0U, true, true,
		58019U, true, 100U, 0U, false, 2000U));
	CHECK(!pathPolicySourceMetadataValid(true, true, 0U, true, true,
		58019U, true, 100U, 0U, true, 0U));

	CHECK(!pathPolicyGoalReached(true, 3U, 10U, 999U, 1000U));
	CHECK(pathPolicyGoalReached(true, 3U, 10U, 1000U, 1000U));
	CHECK(!pathPolicyGoalReached(false, 3U, 10U, 1000U, 1000U));
	CHECK(pathPolicyGenerationLevel(false, 1U) == 0U);
	CHECK(pathPolicyGenerationLevel(true, 1U) == 1U);
	CHECK(pathPolicyGoalRequest(false, true, false));
	CHECK(!pathPolicyGoalRequest(true, true, false));
	CHECK(pathPolicyGoalRequest(true, false, true));

	int shortcutValues[7] = {1, 80, 40, 3000, 600, 10, 0};
	const bool shortcutPresent[7] = {true, true, true, true, true, true, false};
	const int shortcutDefaults[7] = {1, 80, 40, 3000, 600, 10, 0};
	const int shortcutMinimums[7] = {0, 40, 0, 0, 0, 0, 0};
	const int shortcutMaximums[7] = {1, 300, 200, 10000, 3000, 100, 1000};
	CHECK(pathPolicyMergeSparseSettings(shortcutValues, shortcutPresent,
		shortcutDefaults, shortcutMinimums, shortcutMaximums, 7U));
	CHECK(shortcutValues[0] == 1 && shortcutValues[5] == 10 && shortcutValues[6] == 0);

	int targetSpeeds[19];
	int targetDefaults[19];
	int targetMinimums[19] = {0};
	int targetMaximums[19];
	bool targetPresent[19];
	for (int i = 0; i < 19; i++)
	{
		targetSpeeds[i] = 100 + i;
		targetDefaults[i] = 1000 + i;
		targetMaximums[i] = (i == 14 || i == 15) ? 2000 : ((i == 17) ? 9900 : 1000);
		targetPresent[i] = i < 18;
	}
	CHECK(pathPolicyMergeSparseSettings(targetSpeeds, targetPresent,
		targetDefaults, targetMinimums, targetMaximums, 19U));
	for (int i = 0; i < 18; i++) CHECK(targetSpeeds[i] == 100 + i);
	CHECK(targetSpeeds[18] == targetDefaults[18]);
	targetSpeeds[4] = 3000;
	CHECK(pathPolicyMergeSparseSettings(targetSpeeds, targetPresent,
		targetDefaults, targetMinimums, targetMaximums, 19U));
	CHECK(targetSpeeds[4] == targetDefaults[4]);
}
