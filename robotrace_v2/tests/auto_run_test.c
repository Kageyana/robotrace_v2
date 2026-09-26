#include "autoRun.h"
#include "courseLogCsv.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(condition) \
	do \
	{ \
		if (!(condition)) \
		{ \
			fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
			exit(EXIT_FAILURE); \
		} \
	} while (0)

typedef struct
{
	bool exists;
	bool writeAllowed;
	bool shortRead;
	bool closeFailure;
	AutoRunConfigReadResult readResult;
	char text[AUTO_RUN_CONFIG_TEXT_CAPACITY];
	size_t length;
	unsigned int writeCount;
} TestStore;

static AutoRunConfigReadResult testRead(void *context, char *buffer, size_t capacity, size_t *length)
{
	TestStore *store = (TestStore *)context;
	if (!store->exists) return AUTO_RUN_CONFIG_READ_MISSING;
	if (store->readResult != AUTO_RUN_CONFIG_READ_SUCCESS) return store->readResult;
	if (store->shortRead || store->closeFailure) return AUTO_RUN_CONFIG_READ_IO_ERROR;
	if (store->length >= capacity) return AUTO_RUN_CONFIG_READ_INVALID;
	memcpy(buffer, store->text, store->length);
	*length = store->length;
	return AUTO_RUN_CONFIG_READ_SUCCESS;
}

static bool testWrite(void *context, const char *text, size_t length)
{
	TestStore *store = (TestStore *)context;
	if (!store->writeAllowed || length >= sizeof(store->text)) return false;
	memcpy(store->text, text, length);
	store->text[length] = '\0';
	store->length = length;
	store->exists = true;
	store->writeCount++;
	return true;
}

static void checkIoReadFailure(TestStore *store, AutoRunState *state, const char *original)
{
	state->configReady = true;
	store->writeCount = 0U;
	CHECK(autoRunLoadConfig(state, testRead, testWrite, store) == AUTO_RUN_CONFIG_LOAD_IO_ERROR);
	CHECK(!state->configReady);
	CHECK(store->writeCount == 0U);
	CHECK(store->exists && strcmp(store->text, original) == 0);
}

static void testConfigParsingAndRepair(void)
{
	static const char defaults[] =
		"2,DISTANCE\n3,SLIP\n4,PATH\n5,SHORTCUT\n";
	AutoRunMode modes[AUTO_RUN_CONFIG_SLOT_COUNT];
	CHECK(autoRunParseConfig(defaults, modes));
	CHECK(modes[0] == AUTO_RUN_MODE_DISTANCE);
	CHECK(modes[1] == AUTO_RUN_MODE_SLIP);
	CHECK(modes[2] == AUTO_RUN_MODE_PATH);
	CHECK(modes[3] == AUTO_RUN_MODE_SHORTCUT);

	CHECK(autoRunParseConfig("2,DISTANCE\n3,DISTANCE\n4,SLIP\n5,PATH", modes));
	CHECK(modes[0] == AUTO_RUN_MODE_DISTANCE && modes[1] == AUTO_RUN_MODE_DISTANCE);
	CHECK(!autoRunParseConfig("2,SLIP\n3,DISTANCE\n4,PATH\n5,SHORTCUT", modes));
	CHECK(!autoRunParseConfig("2,DISTANCE\n3,PATH\n4,SLIP\n5,SHORTCUT", modes));
	CHECK(!autoRunParseConfig("2,DISTANCE\n3,SLIP\n4,PATH", modes));
	CHECK(!autoRunParseConfig("2,DISTANCE\n3,UNKNOWN\n4,PATH\n5,SHORTCUT", modes));

	TestStore store = {0};
	store.writeAllowed = true;
	AutoRunState state = {0};
	CHECK(autoRunLoadConfig(&state, testRead, testWrite, &store) == AUTO_RUN_CONFIG_LOAD_READY);
	CHECK(store.exists && store.writeCount == 1U);
	CHECK(strcmp(store.text, defaults) == 0);
	CHECK(state.configReady);

	store.writeCount = 0U;
	strcpy(store.text, "2,DISTANCE\n3,PATH\n4,SLIP\n5,SHORTCUT\n");
	store.length = strlen(store.text);
	CHECK(autoRunLoadConfig(&state, testRead, testWrite, &store) == AUTO_RUN_CONFIG_LOAD_READY);
	CHECK(store.writeCount == 1U && strcmp(store.text, defaults) == 0);
	CHECK(state.modeByRun[1] == AUTO_RUN_MODE_SLIP);

	store.writeCount = 0U;
	memcpy(store.text, defaults, sizeof(defaults) - 1U);
	store.text[sizeof(defaults) - 1U] = '\0';
	memcpy(store.text + sizeof(defaults), "junk", 4U);
	store.length = sizeof(defaults) + 3U;
	store.exists = true;
	CHECK(autoRunLoadConfig(&state, testRead, testWrite, &store) == AUTO_RUN_CONFIG_LOAD_READY);
	CHECK(store.writeCount == 1U && strcmp(store.text, defaults) == 0);

	strcpy(store.text, defaults);
	store.length = strlen(store.text);
	store.exists = true;
	store.writeCount = 0U;
	CHECK(autoRunLoadConfig(&state, testRead, testWrite, &store) == AUTO_RUN_CONFIG_LOAD_READY);
	CHECK(state.configReady && store.writeCount == 0U);

	strcpy(store.text, "2,DISTANCE\n3,PATH\n4,SLIP\n5,SHORTCUT\n");
	store.length = strlen(store.text);
	store.writeAllowed = false;
	store.writeCount = 0U;
	char invalidOriginal[AUTO_RUN_CONFIG_TEXT_CAPACITY];
	strcpy(invalidOriginal, store.text);
	CHECK(autoRunLoadConfig(&state, testRead, testWrite, &store) == AUTO_RUN_CONFIG_LOAD_REPAIR_FAILED);
	CHECK(!state.configReady);
	CHECK(store.exists && store.writeCount == 0U && strcmp(store.text, invalidOriginal) == 0);

	strcpy(store.text, defaults);
	store.length = strlen(store.text);
	store.writeAllowed = true;
	store.readResult = AUTO_RUN_CONFIG_READ_IO_ERROR;
	checkIoReadFailure(&store, &state, defaults);

	store.readResult = AUTO_RUN_CONFIG_READ_SUCCESS;
	store.shortRead = true;
	checkIoReadFailure(&store, &state, defaults);

	store.shortRead = false;
	store.closeFailure = true;
	checkIoReadFailure(&store, &state, defaults);

	store.closeFailure = false;
	store.readResult = AUTO_RUN_CONFIG_READ_INVALID;
	store.writeCount = 0U;
	CHECK(autoRunLoadConfig(&state, testRead, testWrite, &store) == AUTO_RUN_CONFIG_LOAD_READY);
	CHECK(state.configReady && store.writeCount == 1U && strcmp(store.text, defaults) == 0);
}

static void testRunLogSelection(void)
{
	AutoRunState state = {0};
	AutoRunPlan plan;
	CHECK(autoRunParseConfig("2,DISTANCE\n3,SLIP\n4,PATH\n5,SHORTCUT\n", state.modeByRun));
	state.configReady = true;
	autoRunBeginSeries(&state);

	CHECK(autoRunPrepareRun(&state, 1U, &plan));
	CHECK(plan.requestedMode == AUTO_RUN_MODE_PRIMARY);
	CHECK(autoRunPrimaryLogForHeader(&state, 100) == 100);
	CHECK(autoRunCompleteRun(&state, &plan, 100, true, true));
	CHECK(state.firstLogNumber == 100 && state.previousLogNumber == 100);

	CHECK(autoRunPrepareRun(&state, 2U, &plan));
	CHECK(plan.requestedMode == AUTO_RUN_MODE_DISTANCE);
	CHECK(plan.primaryLogNumber == 100 && plan.slipSourceLogNumber == 0);
	CHECK(autoRunCompleteRun(&state, &plan, 101, true, true));
	CHECK(state.firstLogNumber == 100 && state.previousLogNumber == 101);

	CHECK(autoRunPrepareRun(&state, 3U, &plan));
	CHECK(plan.requestedMode == AUTO_RUN_MODE_SLIP);
	CHECK(plan.primaryLogNumber == 100 && plan.slipSourceLogNumber == 101);
	CHECK(!autoRunCompleteRun(&state, &plan, 102, false, true));
	autoRunAbortSeries(&state);

	CHECK(autoRunParseConfig("2,DISTANCE\n3,DISTANCE\n4,SLIP\n5,PATH\n", state.modeByRun));
	state.configReady = true;
	autoRunBeginSeries(&state);
	CHECK(autoRunPrepareRun(&state, 1U, &plan));
	CHECK(autoRunCompleteRun(&state, &plan, 200, true, true));
	CHECK(autoRunPrepareRun(&state, 2U, &plan) && plan.primaryLogNumber == 200);
	CHECK(autoRunCompleteRun(&state, &plan, 201, true, true));
	CHECK(autoRunPrepareRun(&state, 3U, &plan) && plan.primaryLogNumber == 200);
	CHECK(autoRunCompleteRun(&state, &plan, 202, true, true));
	CHECK(autoRunPrepareRun(&state, 4U, &plan));
	CHECK(plan.requestedMode == AUTO_RUN_MODE_SLIP && plan.slipSourceLogNumber == 202);
	CHECK(autoRunCompleteRun(&state, &plan, 203, true, true));
	CHECK(autoRunPrepareRun(&state, 5U, &plan));
	CHECK(plan.requestedMode == AUTO_RUN_MODE_PATH && plan.primaryLogNumber == 200);
	autoRunAbortSeries(&state);

	CHECK(autoRunParseConfig("2,DISTANCE\n3,SLIP\n4,PATH\n5,SHORTCUT\n", state.modeByRun));
	state.configReady = true;
	autoRunBeginSeries(&state);
	CHECK(autoRunPrepareRun(&state, 1U, &plan));
	CHECK(autoRunCompleteRun(&state, &plan, 300, true, true));

	CHECK(autoRunPrepareRun(&state, 2U, &plan));
	CHECK(plan.requestedMode == AUTO_RUN_MODE_DISTANCE && plan.primaryLogNumber == 300);
	CHECK(autoRunPrimaryLogForHeader(&state, 301) == 300);
	CHECK(autoRunCompleteRun(&state, &plan, 301, true, true));

	CHECK(autoRunPrepareRun(&state, 3U, &plan));
	CHECK(plan.requestedMode == AUTO_RUN_MODE_SLIP && plan.primaryLogNumber == 300);
	CHECK(plan.slipSourceLogNumber == 301);
	CHECK(autoRunCompleteRun(&state, &plan, 302, true, true));

	CHECK(autoRunPrepareRun(&state, 4U, &plan));
	CHECK(plan.requestedMode == AUTO_RUN_MODE_PATH && plan.primaryLogNumber == 300);
	CHECK(plan.slipSourceLogNumber == 0);
	CHECK(autoRunCompleteRun(&state, &plan, 303, true, true));

	CHECK(autoRunPrepareRun(&state, 5U, &plan));
	CHECK(plan.requestedMode == AUTO_RUN_MODE_SHORTCUT && plan.primaryLogNumber == 300);
	CHECK(plan.slipSourceLogNumber == 0);
	CHECK(!autoRunCompleteRun(&state, &plan, 304, true, false));
	autoRunAbortSeries(&state);
	CHECK(!autoRunPrepareRun(&state, 5U, &plan));
}

static void testNewAndLegacyCsvRows(void)
{
	const char *newMetadata = "fwVersion=test,autoRunNumber=3\n";
	const char *newHeader = "cntlog,encCurrentN,gyroVal_Z,courseMarker,encTotalOptimal,ROC,targetSpeed,optimalIndex,slipFlag,slipFlagLat\n";
	const char *newRow = "12,128,12.5,3,550,124.5,200,4,1,0\n";
	CourseLogColumnMap map;
	CourseLogDistanceRow distance;
	CourseLogSlipRow slip;
	CHECK(courseLogResolveHeader(newMetadata, newHeader, false, &map));
	CHECK(courseLogParseDistanceRow(newRow, &map, &distance));
	CHECK(distance.cntlog == 12 && distance.encCurrentN == 128);
	CHECK(distance.courseMarker == 3 && distance.encTotalOptimal == 550);
	CHECK(distance.ROC > 124.4f && distance.ROC < 124.6f);
	CHECK(courseLogResolveHeader(newMetadata, newHeader, true, &map));
	CHECK(courseLogParseSlipRow(newRow, &map, &slip));
	CHECK(slip.optimalIndex == 4 && slip.slipFlag == 1 && slip.slipFlagLat == 0);

	const char *legacyMixedHeader = "slipFlagLat,optimalIndex,cntlog,encTotalOptimal,courseMarker,ROC,encCurrentN,targetSpeed,gyroVal_Z,slipFlag,autoRunNumber=4\n";
	const char *legacyRow = "1,2,500,250,2,3000.0,170,70,3.25,1,\n";
	CHECK(courseLogResolveHeader(legacyMixedHeader, NULL, false, &map));
	CHECK(courseLogParseDistanceRow(legacyRow, &map, &distance));
	CHECK(distance.cntlog == 500 && distance.encCurrentN == 170);
	CHECK(distance.courseMarker == 2 && distance.encTotalOptimal == 250);
	CHECK(distance.ROC == 3000.0f);
	CHECK(courseLogResolveHeader(legacyMixedHeader, NULL, true, &map));
	CHECK(courseLogParseSlipRow(legacyRow, &map, &slip));
	CHECK(slip.optimalIndex == 2 && slip.slipFlag == 1 && slip.slipFlagLat == 1);
	CHECK(!courseLogParseSlipRow("1,2,500,250,2,invalid,170,70,3.25,1,\n", &map, &slip));
}

int main(void)
{
	testConfigParsingAndRepair();
	testRunLogSelection();
	testNewAndLegacyCsvRows();
	puts("All auto-run and CSV parsing tests passed.");
	return EXIT_SUCCESS;
}
