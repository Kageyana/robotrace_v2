#include "courseLogCsv.h"

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/////////////////////////////////////////////////////////////////////
// モジュール名 csvFieldEquals
// 処理概要     CSVヘッダのフィールド名が指定名と一致するか判定する
// 引数         start: フィールド開始位置, end: フィールド終端位置, name: 比較する列名
// 戻り値       true: 一致 false: 不一致
/////////////////////////////////////////////////////////////////////
static bool csvFieldEquals(const char *start, const char *end, const char *name)
{
	while (start < end && (*start == ' ' || *start == '\t')) start++;
	while (end > start && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n')) end--;
	size_t nameLength = strlen(name);
	return (size_t)(end - start) == nameLength && strncmp(start, name, nameLength) == 0;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 courseLogInitColumnMap
// 処理概要     走行ログCSVの列位置を未設定に初期化する
// 引数         map: 列位置格納先
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void courseLogInitColumnMap(CourseLogColumnMap *map)
{
	if (map == NULL) return;
	map->cntlog = -1;
	map->encCurrentN = -1;
	map->gyroValZ = -1;
	map->courseMarker = -1;
	map->encTotalOptimal = -1;
	map->ROC = -1;
	map->targetSpeed = -1;
	map->optimalIndex = -1;
	map->slipFlag = -1;
	map->slipFlagLat = -1;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 setColumn
// 処理概要     CSVヘッダの列名に対応する列番号を記録する
// 引数         map: 列位置格納先, start: フィールド開始位置, end: 終端位置, column: 列番号
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void setColumn(CourseLogColumnMap *map, const char *start, const char *end, int16_t column)
{
	if (map->cntlog < 0 && (csvFieldEquals(start, end, "cntlog") || csvFieldEquals(start, end, "time"))) map->cntlog = column;
	else if (map->encCurrentN < 0 && (csvFieldEquals(start, end, "encCurrentN") || csvFieldEquals(start, end, "velo"))) map->encCurrentN = column;
	else if (map->gyroValZ < 0 && (csvFieldEquals(start, end, "gyroVal_Z") || csvFieldEquals(start, end, "gyro") || csvFieldEquals(start, end, "angVelo"))) map->gyroValZ = column;
	else if (map->courseMarker < 0 && (csvFieldEquals(start, end, "courseMarker") || csvFieldEquals(start, end, "marker"))) map->courseMarker = column;
	else if (map->encTotalOptimal < 0 && (csvFieldEquals(start, end, "encTotalOptimal") || csvFieldEquals(start, end, "distance"))) map->encTotalOptimal = column;
	else if (map->ROC < 0 && (csvFieldEquals(start, end, "ROC") || csvFieldEquals(start, end, "roc"))) map->ROC = column;
	else if (map->targetSpeed < 0 && csvFieldEquals(start, end, "targetSpeed")) map->targetSpeed = column;
	else if (map->optimalIndex < 0 && csvFieldEquals(start, end, "optimalIndex")) map->optimalIndex = column;
	else if (map->slipFlag < 0 && csvFieldEquals(start, end, "slipFlag")) map->slipFlag = column;
	else if (map->slipFlagLat < 0 && csvFieldEquals(start, end, "slipFlagLat")) map->slipFlagLat = column;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 courseLogParseHeaderLine
// 処理概要     CSVヘッダから走行ログ列を名前で解決する
// 引数         line: ヘッダ文字列, map: 列位置格納先
// 戻り値       true: 走行データ列を検出 false: 対象列なし
/////////////////////////////////////////////////////////////////////
bool courseLogParseHeaderLine(const char *line, CourseLogColumnMap *map)
{
	if (line == NULL || map == NULL) return false;
	courseLogInitColumnMap(map);
	const char *fieldStart = line;
	const char *p = line;
	int16_t column = 0;
	while (*p != '\0' && *p != '\n' && *p != '\r')
	{
		if (*p == ',')
		{
			setColumn(map, fieldStart, p, column++);
			fieldStart = p + 1;
		}
		p++;
	}
	setColumn(map, fieldStart, p, column);
	return courseLogHasDistanceColumns(map) || courseLogHasSlipColumns(map);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 courseLogHasDistanceColumns
// 処理概要     距離解析に必要なCSV列がすべて解決済みか確認する
// 引数         map: 列位置
// 戻り値       true: 必要列あり false: 不足
/////////////////////////////////////////////////////////////////////
bool courseLogHasDistanceColumns(const CourseLogColumnMap *map)
{
	return map != NULL && map->cntlog >= 0 && map->encCurrentN >= 0 && map->gyroValZ >= 0 &&
		map->courseMarker >= 0 && map->encTotalOptimal >= 0 && map->ROC >= 0;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 courseLogHasSlipColumns
// 処理概要     スリップ解析に必要なCSV列がすべて解決済みか確認する
// 引数         map: 列位置
// 戻り値       true: 必要列あり false: 不足
/////////////////////////////////////////////////////////////////////
bool courseLogHasSlipColumns(const CourseLogColumnMap *map)
{
	return map != NULL && map->courseMarker >= 0 && map->encTotalOptimal >= 0 && map->ROC >= 0 &&
		map->targetSpeed >= 0 && map->optimalIndex >= 0 && map->slipFlag >= 0 && map->slipFlagLat >= 0;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 courseLogResolveHeader
// 処理概要     新旧CSVのヘッダ行から解析に必要な列を解決する
// 引数         firstLine: 1行目, secondLine: 2行目, requireSlipColumns: スリップ列要否, map: 列位置
// 戻り値       true: 必要列を解決 false: 必要列が不足
/////////////////////////////////////////////////////////////////////
bool courseLogResolveHeader(const char *firstLine, const char *secondLine, bool requireSlipColumns, CourseLogColumnMap *map)
{
	if (map == NULL) return false;
	if (courseLogParseHeaderLine(firstLine, map) &&
		(requireSlipColumns ? courseLogHasSlipColumns(map) : courseLogHasDistanceColumns(map)))
	{
		return true;
	}
	if (courseLogParseHeaderLine(secondLine, map) &&
		(requireSlipColumns ? courseLogHasSlipColumns(map) : courseLogHasDistanceColumns(map)))
	{
		return true;
	}
	courseLogInitColumnMap(map);
	return false;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 maxColumn
// 処理概要     指定解析に必要な最大列番号を取得する
// 引数         map: 列位置, slip: trueならスリップ解析用
// 戻り値       最大列番号
/////////////////////////////////////////////////////////////////////
static int16_t maxColumn(const CourseLogColumnMap *map, bool slip)
{
	int16_t max = slip ? map->courseMarker : map->cntlog;
#define COURSE_LOG_MAX_COLUMN(field) if (map->field > max) max = map->field
	if (slip)
	{
		COURSE_LOG_MAX_COLUMN(encTotalOptimal);
		COURSE_LOG_MAX_COLUMN(ROC);
		COURSE_LOG_MAX_COLUMN(targetSpeed);
		COURSE_LOG_MAX_COLUMN(optimalIndex);
		COURSE_LOG_MAX_COLUMN(slipFlag);
		COURSE_LOG_MAX_COLUMN(slipFlagLat);
	}
	else
	{
		COURSE_LOG_MAX_COLUMN(encCurrentN);
		COURSE_LOG_MAX_COLUMN(gyroValZ);
		COURSE_LOG_MAX_COLUMN(courseMarker);
		COURSE_LOG_MAX_COLUMN(encTotalOptimal);
		COURSE_LOG_MAX_COLUMN(ROC);
	}
#undef COURSE_LOG_MAX_COLUMN
	return max;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 parseField
// 処理概要     CSV値を整数または有限浮動小数点数へ変換する
// 引数         start: 値の開始位置, end: 終端位置, asFloat: 浮動小数点指定, value: 変換結果
// 戻り値       true: 変換成功 false: 空欄または不正値
/////////////////////////////////////////////////////////////////////
static bool parseField(const char *start, const char *end, bool asFloat, double *value)
{
	char token[64];
	while (start < end && (*start == ' ' || *start == '\t')) start++;
	while (end > start && (* (end - 1) == ' ' || *(end - 1) == '\t' || *(end - 1) == '\r' || *(end - 1) == '\n')) end--;
	size_t length = (size_t)(end - start);
	if (length == 0U || length >= sizeof(token)) return false;
	memcpy(token, start, length);
	token[length] = '\0';
	errno = 0;
	char *parseEnd = NULL;
	if (asFloat)
	{
		float parsed = strtof(token, &parseEnd);
		if (parseEnd == token || *parseEnd != '\0' || errno == ERANGE || !isfinite(parsed)) return false;
		*value = parsed;
	}
	else
	{
		long parsed = strtol(token, &parseEnd, 10);
		if (parseEnd == token || *parseEnd != '\0' || errno == ERANGE || parsed < INT32_MIN || parsed > INT32_MAX) return false;
		*value = (double)parsed;
	}
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 parseMappedRow
// 処理概要     列位置に従って走行データ行の必要な値を抽出する
// 引数         line: CSV行, map: 列位置, slip: trueならスリップ解析, values: 値格納先, found: 読取状態
// 戻り値       true: 必須値をすべて読取 false: 行または値が不正
/////////////////////////////////////////////////////////////////////
static bool parseMappedRow(const char *line, const CourseLogColumnMap *map, bool slip, double values[10], bool found[10])
{
	if (line == NULL || map == NULL || (slip ? !courseLogHasSlipColumns(map) : !courseLogHasDistanceColumns(map))) return false;
	memset(values, 0, sizeof(double) * 10U);
	memset(found, 0, sizeof(bool) * 10U);
	int16_t needed = maxColumn(map, slip);
	int16_t column = 0;
	const char *fieldStart = line;
	const char *p = line;
	while (1)
	{
		if (*p == ',' || *p == '\n' || *p == '\r' || *p == '\0')
		{
			int mapIndex = -1;
			bool asFloat = false;
			if (slip)
			{
				if (column == map->courseMarker) mapIndex = 3;
				else if (column == map->encTotalOptimal) mapIndex = 4;
				else if (column == map->ROC) { mapIndex = 5; asFloat = true; }
				else if (column == map->targetSpeed) { mapIndex = 6; asFloat = true; }
				else if (column == map->optimalIndex) mapIndex = 7;
				else if (column == map->slipFlag) mapIndex = 8;
				else if (column == map->slipFlagLat) mapIndex = 9;
			}
			else
			{
				if (column == map->cntlog) mapIndex = 0;
				else if (column == map->encCurrentN) mapIndex = 1;
				else if (column == map->gyroValZ) { mapIndex = 2; asFloat = true; }
				else if (column == map->courseMarker) mapIndex = 3;
				else if (column == map->encTotalOptimal) mapIndex = 4;
				else if (column == map->ROC) { mapIndex = 5; asFloat = true; }
			}
			if (mapIndex >= 0)
			{
				if (!parseField(fieldStart, p, asFloat, &values[mapIndex])) return false;
				found[mapIndex] = true;
			}
			if (*p != ',') break;
			column++;
			if (column > needed) break;
			fieldStart = p + 1;
		}
		p++;
	}
	if (slip)
	{
		for (int i = 3; i <= 9; i++) if (!found[i]) return false;
	}
	else
	{
		for (int i = 0; i <= 5; i++) if (!found[i]) return false;
	}
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 courseLogParseDistanceRow
// 処理概要     CSV行から距離解析用の値を抽出する
// 引数         line: CSV行, map: 列位置, row: 抽出結果
// 戻り値       true: 抽出成功 false: 必須値が不正または不足
/////////////////////////////////////////////////////////////////////
bool courseLogParseDistanceRow(const char *line, const CourseLogColumnMap *map, CourseLogDistanceRow *row)
{
	double values[10];
	bool found[10];
	if (row == NULL || !parseMappedRow(line, map, false, values, found)) return false;
	row->cntlog = (int32_t)values[0];
	row->encCurrentN = (int32_t)values[1];
	row->gyroValZ = (float)values[2];
	row->courseMarker = (int32_t)values[3];
	row->encTotalOptimal = (int32_t)values[4];
	row->ROC = (float)values[5];
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 courseLogParseSlipRow
// 処理概要     CSV行からスリップ解析用の値を抽出する
// 引数         line: CSV行, map: 列位置, row: 抽出結果
// 戻り値       true: 抽出成功 false: 必須値が不正または不足
/////////////////////////////////////////////////////////////////////
bool courseLogParseSlipRow(const char *line, const CourseLogColumnMap *map, CourseLogSlipRow *row)
{
	double values[10];
	bool found[10];
	if (row == NULL || !parseMappedRow(line, map, true, values, found)) return false;
	row->courseMarker = (int32_t)values[3];
	row->encTotalOptimal = (int32_t)values[4];
	row->ROC = (float)values[5];
	row->targetSpeed = (float)values[6];
	row->optimalIndex = (int32_t)values[7];
	row->slipFlag = (int32_t)values[8];
	row->slipFlagLat = (int32_t)values[9];
	return true;
}
