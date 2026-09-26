#include "pathFollower.h"
#include "control.h"
#include "courseAnalysis.h"
#include "encoder.h"
#include "pathPolicy.h"

#include <math.h>
#include <stdarg.h>
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

#define MOCK_CSV_CAPACITY 120000U
#define MOCK_FR_OK 0
#define MOCK_FR_NO_FILE 4
static char mockCsv[MOCK_CSV_CAPACITY];
static size_t mockCsvLength = 0U;
static size_t mockCsvPosition = 0U;
static uint16_t mockRowCount = 0U;

speedParam tgtParam = {0};
uint8_t optimalTrace = BOOST_NONE;
uint16_t optimalIndex = 0U;
int16_t indexSC = 0;
int16_t analyzedNumber = 0;
int32_t log_targetAngularVelocity = 0;
bool stateCrossLine = false;
uint16_t lSensorCari[NUM_SENSORS] = {0};

typedef int MockFResult;
typedef unsigned char MockByte;
typedef char MockTchar;

bool sd_fatfs_lock(uint32_t timeout_ms)
{
	(void)timeout_ms;
	return true;
}

void sd_fatfs_unlock(void) {}
void saveLogNumber(int16_t fileNumber) { (void)fileNumber; }
float asignVelocity(int16_t roc) { (void)roc; return 1.0F; }
void setTargetAngularVelocity(float target) { (void)target; }
bool isLineSensorCalibrationValid(void) { return false; }

MockFResult f_open(void *file, const MockTchar *path, MockByte mode)
{
	(void)file;
	(void)mode;
	if (strcmp(path, "1.csv") != 0) return MOCK_FR_NO_FILE;
	mockCsvPosition = 0U;
	return MOCK_FR_OK;
}

MockFResult f_close(void *file)
{
	(void)file;
	return MOCK_FR_OK;
}

MockTchar *f_gets(MockTchar *buffer, int length, void *file)
{
	(void)file;
	if (mockCsvPosition >= mockCsvLength || length < 2) return NULL;
	int copied = 0;
	while (copied < length - 1 && mockCsvPosition < mockCsvLength)
	{
		char value = mockCsv[mockCsvPosition++];
		buffer[copied++] = value;
		if (value == '\n') break;
	}
	buffer[copied] = '\0';
	return buffer;
}

int f_printf(void *file, const char *format, ...)
{
	(void)file;
	(void)format;
	return 0;
}

static void mockAppend(const char *text)
{
	size_t length = strlen(text);
	CHECK(mockCsvLength + length < MOCK_CSV_CAPACITY);
	memcpy(&mockCsv[mockCsvLength], text, length + 1U);
	mockCsvLength += length;
}

static void mockBeginLog(uint16_t rows, bool currentMetadata, uint32_t expectedRows)
{
	mockCsvLength = 0U;
	mockRowCount = rows;
	char header[256];
	if (currentMetadata)
	{
		(void)snprintf(header, sizeof(header),
			"pathSourceFormatVersion=1,closureValid=1,closureReason=0,"
			"optimalTrace=0.00,emcStop=0.00,encoderPulsePerMeter=58019,"
			"imuCalibrationValid=1,imuCalibrationSamples=100,"
			"imuCalibrationReadErrors=0,distanceScaleVerified=1,logExpectedRows=%lu,\n",
			(unsigned long)expectedRows);
	}
	else
	{
		(void)snprintf(header, sizeof(header),
			"closureValid=1,closureReason=0,optimalTrace=0.00,emcStop=0.00,"
			"encoderPulsePerMeter=58019,imuCalibrationValid=1,"
			"imuCalibrationSamples=100,imuCalibrationReadErrors=0,"
			"distanceScaleVerified=1,logExpectedRows=%u,\n", rows);
	}
	mockAppend(header);
	mockAppend("x,y,encTotalOptimal\n");
}

static void mockBuildStraightPrimary(void)
{
	const uint16_t rows = 21U;
	mockBeginLog(rows, true, rows);
	for (uint16_t i = 0U; i < rows; i++)
	{
		char row[64];
		float y = (float)i * 40.0F;
		long pulse = lroundf(y * PULSE_MILLIMETER);
		(void)snprintf(row, sizeof(row), "0.0,%.1f,%ld\n", (double)y, pulse);
		mockAppend(row);
	}
}

static void mockBuildShortcutPrimary(void)
{
	const uint16_t rows = 101U;
	mockBeginLog(rows, true, rows);
	float previousX = 0.0F;
	float previousY = 0.0F;
	float cumulativeMm = 0.0F;
	for (uint16_t i = 0U; i < rows; i++)
	{
		float y = (float)i * 20.0F;
		float phase = (float)(2.0 * 3.14159265358979323846 * (double)y / 200.0);
		float x = 15.0F * (1.0F - cosf(phase));
		if (i > 0U) cumulativeMm += hypotf(x - previousX, y - previousY);
		long pulse = lroundf(cumulativeMm * PULSE_MILLIMETER);
		char row[64];
		(void)snprintf(row, sizeof(row), "%.3f,%.3f,%ld\n", (double)x, (double)y, pulse);
		mockAppend(row);
		previousX = x;
		previousY = y;
	}
}

static void mockBuildClosedPrimary(void)
{
	const float points[4][2] = {{0.0F, 0.0F}, {0.0F, 200.0F},
		{200.0F, 200.0F}, {0.0F, 0.0F}};
	const uint16_t rows = 4U;
	mockBeginLog(rows, true, rows);
	float cumulativeMm = 0.0F;
	for (uint16_t i = 0U; i < rows; i++)
	{
		if (i > 0U) cumulativeMm += hypotf(points[i][0] - points[i - 1U][0],
			points[i][1] - points[i - 1U][1]);
		long pulse = lroundf(cumulativeMm * PULSE_MILLIMETER);
		char row[64];
		(void)snprintf(row, sizeof(row), "%.1f,%.1f,%ld\n",
			(double)points[i][0], (double)points[i][1], pulse);
		mockAppend(row);
	}
}

static void mockBuildPointLimitPrimary(void)
{
	const uint16_t rows = 1510U;
	mockBeginLog(rows, true, rows);
	float previousX = 0.0F;
	float previousY = 0.0F;
	float cumulativeMm = 0.0F;
	for (uint16_t i = 0U; i < rows; i++)
	{
		float angle = (float)i * 0.04F;
		float x = 1000.0F * sinf(angle);
		float y = 1000.0F * cosf(angle);
		if (i == 0U) cumulativeMm = hypotf(x, y);
		else cumulativeMm += hypotf(x - previousX, y - previousY);
		long pulse = lroundf(cumulativeMm * PULSE_MILLIMETER);
		char row[64];
		(void)snprintf(row, sizeof(row), "%.3f,%.3f,%ld\n", (double)x, (double)y, pulse);
		mockAppend(row);
		previousX = x;
		previousY = y;
	}
}

static void prepareRouteSettings(void)
{
	shortcutSettings = (ShortcutSettings){1U, 80U, 40U, 3000U, 600U, 10U, 0U};
	tgtParam.pathReplay = 1.0F;
	tgtParam.shortCut = 0.5F;
	tgtParam.acceleF = 20.0F;
	tgtParam.acceleD = 20.0F;
}

void testPathRouteBuilder(void)
{
	prepareRouteSettings();
	mockBuildStraightPrimary();
	CHECK(routeBuildFromLog(1, pathPolicyGenerationLevel(false, shortcutSettings.maxLevel)) > 0);
	CHECK(optimalTrace == BOOST_PATH_REPLAY);
	CHECK(pathRouteCount() == 31U);
	pathFollowerReset();
	CHECK(!pathFollowerGoalReached());

	prepareRouteSettings();
	mockBuildShortcutPrimary();
	CHECK(routeBuildFromLog(1, pathPolicyGenerationLevel(true, shortcutSettings.maxLevel)) > 0);
	CHECK(pathRouteShortcutBuildStatus() == PATH_SHORTCUT_BUILD_SUCCESS);
	CHECK(optimalTrace == BOOST_SHORTCUT);
	CHECK(pathRouteCount() > mockRowCount / 2U);
	pathFollowerReset();
	CHECK(!pathFollowerGoalReached());

	prepareRouteSettings();
	mockBuildClosedPrimary();
	CHECK(routeBuildFromLog(1, 0U) == -15);

	prepareRouteSettings();
	mockBuildPointLimitPrimary();
	CHECK(routeBuildFromLog(1, 0U) == -7);

	prepareRouteSettings();
	mockBuildStraightPrimary();
	mockCsvLength = strlen(mockCsv);
	char *version = strstr(mockCsv, "pathSourceFormatVersion=1,");
	CHECK(version != NULL);
	memmove(version, version + strlen("pathSourceFormatVersion=1,"),
		mockCsvLength - (size_t)(version - mockCsv) - strlen("pathSourceFormatVersion=1,") + 1U);
	mockCsvLength -= strlen("pathSourceFormatVersion=1,");
	CHECK(routeBuildFromLog(1, 0U) < 0);

	prepareRouteSettings();
	mockBuildStraightPrimary();
	mockCsvLength = strlen(mockCsv);
	char *expectedRows = strstr(mockCsv, "logExpectedRows=21,");
	CHECK(expectedRows != NULL);
	memcpy(expectedRows + strlen("logExpectedRows="), "22", 2U);
	CHECK(routeBuildFromLog(1, 0U) < 0);
	CHECK(f_open(NULL, "missing.csv", 0U) == MOCK_FR_NO_FILE);
}
