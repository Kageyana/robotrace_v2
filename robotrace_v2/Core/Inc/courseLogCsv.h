#ifndef COURSE_LOG_CSV_H_
#define COURSE_LOG_CSV_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
	int16_t cntlog;
	int16_t encCurrentN;
	int16_t gyroValZ;
	int16_t courseMarker;
	int16_t encTotalOptimal;
	int16_t ROC;
	int16_t targetSpeed;
	int16_t optimalIndex;
	int16_t slipV2TotalMs;
	int16_t slipV2ValidMs;
	int16_t slipV2OnLongMs;
	int16_t slipV2LatValidMs;
	int16_t slipV2OnLatMs;
	int16_t slipV2IntervalReason;
	int16_t slipV2LatIntervalReason;
} CourseLogColumnMap;

typedef struct
{
	int32_t cntlog;
	int32_t encCurrentN;
	float gyroValZ;
	int32_t courseMarker;
	int32_t encTotalOptimal;
	float ROC;
} CourseLogDistanceRow;

typedef struct
{
	int32_t courseMarker;
	int32_t encTotalOptimal;
	float ROC;
	float targetSpeed;
	int32_t optimalIndex;
	int32_t slipV2TotalMs;
	int32_t slipV2ValidMs;
	int32_t slipV2OnLongMs;
	int32_t slipV2LatValidMs;
	int32_t slipV2OnLatMs;
	int32_t slipV2IntervalReason;
	int32_t slipV2LatIntervalReason;
} CourseLogSlipRow;

void courseLogInitColumnMap(CourseLogColumnMap *map);
bool courseLogParseHeaderLine(const char *line, CourseLogColumnMap *map);
bool courseLogHasDistanceColumns(const CourseLogColumnMap *map);
bool courseLogHasSlipColumns(const CourseLogColumnMap *map);
bool courseLogResolveHeader(const char *firstLine, const char *secondLine, bool requireSlipColumns, CourseLogColumnMap *map);
bool courseLogParseDistanceRow(const char *line, const CourseLogColumnMap *map, CourseLogDistanceRow *row);
bool courseLogParseSlipRow(const char *line, const CourseLogColumnMap *map, CourseLogSlipRow *row);
bool courseLogValidateSlipRow(const CourseLogSlipRow *row);

#endif // COURSE_LOG_CSV_H_
