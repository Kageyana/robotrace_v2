#include "pathFollower.h"
#include "PIDcontrol.h"
#include "SDcard.h"
#include "control.h"
#include "courseAnalysis.h"
#include "encoder.h"
#include "ff.h"
#include "lineSensor.h"
#include "sd_functions.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define PATH_SETTING_FILE                    "./setting/shortcut.txt"
#define PATH_DEFAULT_MAX_LEVEL               1U
#define PATH_DEFAULT_LOOKAHEAD_BASE_MM        80U
#define PATH_DEFAULT_LOOKAHEAD_PER_MPS_MM     40U
#define PATH_DEFAULT_KLATERAL_X100            3000U
#define PATH_DEFAULT_KHEADING_X100            600U
#define PATH_DEFAULT_LINE_ALPHA_X1000         0U

#define PATH_LEVEL_MAX                        1U // 初期実機検証はLevel 1だけを許可する
#define PATH_LOOKAHEAD_BASE_MIN_MM            40U
#define PATH_LOOKAHEAD_BASE_MAX_MM            300U
#define PATH_LOOKAHEAD_SPEED_MAX_MM            200U
#define PATH_KLATERAL_MAX_X100                10000U
#define PATH_KHEADING_MAX_X100                3000U
#define PATH_LINE_ALPHA_MAX_X1000             100U

#define PATH_CSV_LINE_SIZE                    2048U
#define PATH_ANCHOR_HALF_WIDTH_MM             100.0f
#define PATH_SMOOTH_MOVE_MAX_MM               2.0f
#define PATH_SMOOTH_CONVERGED_MM              0.1f
#define PATH_SMOOTH_ITERATIONS                100U

// 寸法入力値はすべて[mm]。機体座標の原点は左右駆動輪の車軸中心とし、
// 上面から見て右を+X、左を-X、前方を+Y、後方を-Yとする。
#define PATH_TRACKING_ERROR_BUDGET_MM         15.0f
#define PATH_LINE_HALF_WIDTH_MM               9.5f   // コースライン実幅の1/2
#define PATH_OCCUPIED_HALF_WIDTH_MM           65.0f  // 車軸中心から左右投影端までの短い側の距離
#define PATH_OUTER_RADIUS_MM                  100.0f  // 車軸中心から全投影外形の最遠点までの距離
#define PATH_BOARD_CLEARANCE_MM               200.0f // コース中心線から走行可能領域端までの最小距離
#define PATH_LEGAL_RESERVE_MM                 10.0f
#define PATH_LOST_DISTANCE_MM                 80.0f
#define PATH_REJOIN_DISTANCE_MM               35.0f
#define PATH_LOST_HEADING_DEG                 60.0f
#define PATH_REJOIN_HEADING_DEG               30.0f
#define PATH_LOST_COUNT_5MS                   20U
#define PATH_REJOIN_COUNT_5MS                 40U
#define PATH_LINE_LOST_COUNT_5MS              20U
// 走行中の自己位置を維持できる範囲で、終端付近だけゴールマーカーを有効にする。
#define PATH_GOAL_MIN_PROGRESS_PERMILLE      800U
#define PATH_REJOIN_BLEND_STEP                50U
#define PATH_SENSOR_ACTIVE_TH                 800U
#define PATH_SENSOR_MIN_SUM                   1200U
#define PATH_SENSOR_MAX_CLUSTER_WIDTH         3U
#define PATH_SENSOR_PITCH_MM                  9.5f   // 隣接ラインセンサーの受光中心間隔
#define PATH_SENSOR_FOV_MM                    35.0f  // ライン位置補正を許可する予測横偏差範囲

#define PATH_FLAG_ANCHOR                      0x01U

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t marker;
} RouteCsvColumns;

typedef struct
{
	float x_mm;
	float y_mm;
	float heading_deg;
} PathPose;

static RoutePoint lineRoute[PATH_ROUTE_MAX_POINTS];
static RoutePoint driveRoute[PATH_ROUTE_MAX_POINTS];
static uint8_t routeFlags[PATH_ROUTE_MAX_POINTS];
static uint8_t routeAnchorScratch[PATH_ROUTE_MAX_POINTS];
static char routeCsvLine[PATH_CSV_LINE_SIZE];
static uint16_t routeCount = 0U;
static int16_t routeSourceLog = 0;
static uint8_t routeShortcutLevel = 0U;
static PathPose pathPose;
static PathFollowerState followerState = PATH_STATE_INACTIVE;
static uint16_t routeIndex = 0U;
static uint16_t lostCount = 0U;
static uint16_t rejoinCount = 0U;
static uint16_t lineLostCount = 0U;
static uint16_t pathBlendPermille = 1000U;
static float targetSpeedMps = 0.0f;
static bool currentLineValid = false;

ShortcutSettings shortcutSettings = {0U, PATH_DEFAULT_LOOKAHEAD_BASE_MM,
	PATH_DEFAULT_LOOKAHEAD_PER_MPS_MM, PATH_DEFAULT_KLATERAL_X100,
	PATH_DEFAULT_KHEADING_X100, PATH_DEFAULT_LINE_ALPHA_X1000};
float pathLogLinePointX_mm = 0.0f;
float pathLogLinePointY_mm = 0.0f;
uint8_t pathLogLineValid = 0U;
float pathLogErrorY_mm = 0.0f;
int16_t pathLogErrorHeading_cdeg = 0;
uint8_t pathLogState = PATH_STATE_INACTIVE;
float pathLogLegalMargin_mm = 0.0f;

/////////////////////////////////////////////////////////////////////
// モジュール名 pathWrapDeg
// 処理概要     角度を-180～180degへ正規化する
// 引数         angleDeg: 正規化前の角度[deg]
// 戻り値       正規化後の角度[deg]
/////////////////////////////////////////////////////////////////////
static float pathWrapDeg(float angleDeg)
{
	while (angleDeg > 180.0f) angleDeg -= 360.0f;
	while (angleDeg < -180.0f) angleDeg += 360.0f;
	return angleDeg;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathPointDistance
// 処理概要     2点間の直線距離を算出する
// 引数         x0,y0,x1,y1: 2点の座標[mm]
// 戻り値       2点間距離[mm]
/////////////////////////////////////////////////////////////////////
static float pathPointDistance(float x0, float y0, float x1, float y1)
{
	float dx = x1 - x0;
	float dy = y1 - y0;
	return sqrtf((dx * dx) + (dy * dy));
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathFloatToInt16
// 処理概要     浮動小数値をint16_t範囲へ飽和して丸める
// 引数         value: 変換前の値
// 戻り値       飽和・丸め後の値
/////////////////////////////////////////////////////////////////////
static int16_t pathFloatToInt16(float value)
{
	if (value > 32767.0f) return INT16_MAX;
	if (value < -32768.0f) return INT16_MIN;
	return (int16_t)lroundf(value);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathHeaderFieldEquals
// 処理概要     CSVヘッダ項目名を空白除去後に比較する
// 引数         start,end: 項目範囲, name: 比較名
// 戻り値       true: 一致 false: 不一致
/////////////////////////////////////////////////////////////////////
static bool pathHeaderFieldEquals(const char *start, const char *end, const char *name)
{
	while (start < end && (*start == ' ' || *start == '\t')) start++;
	while (end > start && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n')) end--;
	return (size_t)(end - start) == strlen(name) && strncmp(start, name, (size_t)(end - start)) == 0;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathParseHeader
// 処理概要     CSVヘッダから経路生成に必要な列番号を解決する
// 引数         line:CSVヘッダ, columns:列番号の格納先
// 戻り値       true:必要列あり false:必要列なし
/////////////////////////////////////////////////////////////////////
static bool pathParseHeader(const char *line, RouteCsvColumns *columns)
{
	const char *start = line;
	const char *p = line;
	int16_t column = 0;
	columns->x = -1;
	columns->y = -1;
	columns->marker = -1;

	while (*p != '\0' && *p != '\r' && *p != '\n')
	{
		if (*p == ',')
		{
			if (pathHeaderFieldEquals(start, p, "x")) columns->x = column;
			else if (pathHeaderFieldEquals(start, p, "y")) columns->y = column;
			else if (pathHeaderFieldEquals(start, p, "courseMarker")) columns->marker = column;
			column++;
			start = p + 1;
		}
		p++;
	}
	if (pathHeaderFieldEquals(start, p, "x")) columns->x = column;
	else if (pathHeaderFieldEquals(start, p, "y")) columns->y = column;
	else if (pathHeaderFieldEquals(start, p, "courseMarker")) columns->marker = column;
	return columns->x >= 0 && columns->y >= 0 && columns->marker >= 0;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathCsvFloatAt
// 処理概要     CSV行の指定列を浮動小数値として読む
// 引数         line: CSV行, targetColumn: 列番号, value: 読取先
// 戻り値       true: 読取成功 false: 読取失敗
/////////////////////////////////////////////////////////////////////
static bool pathCsvFloatAt(const char *line, int16_t targetColumn, float *value)
{
	const char *start = line;
	int16_t column = 0;
	while (column < targetColumn)
	{
		start = strchr(start, ',');
		if (start == NULL) return false;
		start++;
		column++;
	}
	char *end = NULL;
	float parsed = strtof(start, &end);
	if (end == start) return false;
	*value = parsed;
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathReadCsvPoint
// 処理概要     CSV行から座標とマーカー値を読む
// 引数         line: CSV行, columns: 列情報, xMm,yMm: 座標[mm], marker: マーカー値
// 戻り値       true: 読取成功 false: 読取失敗
/////////////////////////////////////////////////////////////////////
static bool pathReadCsvPoint(const char *line, const RouteCsvColumns *columns,
	float *xMm, float *yMm, uint8_t *marker)
{
	float markerValue = 0.0f;
	if (!pathCsvFloatAt(line, columns->x, xMm) ||
		!pathCsvFloatAt(line, columns->y, yMm) ||
		!pathCsvFloatAt(line, columns->marker, &markerValue))
	{
		return false;
	}
	if (!isfinite(*xMm) || !isfinite(*yMm)) return false;
	*marker = (uint8_t)markerValue;
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathComputeHeadings
// 処理概要     経路点列から各点の接線方位を算出する
// 引数         route: 経路点列, count: 経路点数
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void pathComputeHeadings(RoutePoint *route, uint16_t count)
{
	if (count < 2U) return;
	for (uint16_t i = 0U; i < count; i++)
	{
		uint16_t before = (i == 0U) ? 0U : (uint16_t)(i - 1U);
		uint16_t after = (i + 1U < count) ? (uint16_t)(i + 1U) : (uint16_t)(count - 1U);
		float dx = (float)route[after].x_mm - (float)route[before].x_mm;
		float dy = (float)route[after].y_mm - (float)route[before].y_mm;
		float heading = atan2f(dx, dy) * RAD2DEG;
		route[i].heading_cdeg = pathFloatToInt16(pathWrapDeg(heading) * 100.0f);
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathBuildSpeedProfile
// 処理概要     曲率と加減速制約から経路速度を設定する
// 引数         route: 経路点列, count: 経路点数, shortcutLevel: 短縮レベル
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void pathBuildSpeedProfile(RoutePoint *route, uint16_t count, uint8_t shortcutLevel)
{
	if (count == 0U) return;
	float speedCap = (shortcutLevel == 0U) ? tgtParam.search : tgtParam.shortCut;
	if (speedCap <= 0.0f) speedCap = tgtParam.search;
	for (uint16_t i = 0U; i < count; i++)
	{
		float speed = speedCap;
		if (i > 0U && i + 1U < count)
		{
			float h0 = (float)route[i - 1U].heading_cdeg * 0.01f;
			float h1 = (float)route[i + 1U].heading_cdeg * 0.01f;
			float dHeadingRad = fabsf(pathWrapDeg(h1 - h0)) * DEG2RAD;
			float dsMm = pathPointDistance((float)route[i - 1U].x_mm, (float)route[i - 1U].y_mm,
				(float)route[i + 1U].x_mm, (float)route[i + 1U].y_mm);
			if (dHeadingRad > 1.0e-4f && dsMm > 1.0f)
			{
				float radiusMm = dsMm / dHeadingRad;
				float curveSpeed = asignVelocity(pathFloatToInt16(radiusMm));
				if (curveSpeed < speed) speed = curveSpeed;
			}
		}
		route[i].speed_cms = (uint16_t)lroundf(fmaxf(0.0f, speed) * 100.0f);
	}

	for (uint16_t i = 1U; i < count; i++)
	{
		float previous = (float)route[i - 1U].speed_cms * 0.01f;
		float dsM = pathPointDistance((float)route[i - 1U].x_mm, (float)route[i - 1U].y_mm,
			(float)route[i].x_mm, (float)route[i].y_mm) * 0.001f;
		float limit = sqrtf(fmaxf(0.0f, (previous * previous) + (2.0f * tgtParam.acceleF * dsM)));
		if ((float)route[i].speed_cms * 0.01f > limit) route[i].speed_cms = (uint16_t)lroundf(limit * 100.0f);
	}
	for (int32_t i = (int32_t)count - 2; i >= 0; i--)
	{
		float next = (float)route[i + 1].speed_cms * 0.01f;
		float dsM = pathPointDistance((float)route[i].x_mm, (float)route[i].y_mm,
			(float)route[i + 1].x_mm, (float)route[i + 1].y_mm) * 0.001f;
		float limit = sqrtf(fmaxf(0.0f, (next * next) + (2.0f * tgtParam.acceleD * dsM)));
		if ((float)route[i].speed_cms * 0.01f > limit) route[i].speed_cms = (uint16_t)lroundf(limit * 100.0f);
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathExpandAnchors
// 処理概要     マーカー前後100mmを形状変更禁止区間へ展開する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void pathExpandAnchors(void)
{
	memcpy(routeAnchorScratch, routeFlags, routeCount);
	uint16_t radius = (uint16_t)ceilf(PATH_ANCHOR_HALF_WIDTH_MM / PATH_ROUTE_SPACING_MM);
	for (uint16_t i = 0U; i < routeCount; i++)
	{
		if ((routeAnchorScratch[i] & PATH_FLAG_ANCHOR) == 0U) continue;
		uint16_t begin = (i > radius) ? (uint16_t)(i - radius) : 0U;
		uint16_t end = (i + radius < routeCount) ? (uint16_t)(i + radius) : (uint16_t)(routeCount - 1U);
		for (uint16_t j = begin; j <= end; j++) routeFlags[j] |= PATH_FLAG_ANCHOR;
	}
}

#if PATH_SHORTCUT_GEOMETRY_ENABLE
/////////////////////////////////////////////////////////////////////
// モジュール名 pathSegmentsIntersect
// 処理概要     離れた2経路区間の交差を判定する
// 引数         route: 経路点列, i,j: 区間先頭インデックス
// 戻り値       true: 交差あり false: 交差なし
/////////////////////////////////////////////////////////////////////
static bool pathSegmentsIntersect(const RoutePoint *route, uint16_t i, uint16_t j)
{
	float ax = route[i].x_mm;
	float ay = route[i].y_mm;
	float bx = route[i + 1U].x_mm;
	float by = route[i + 1U].y_mm;
	float cx = route[j].x_mm;
	float cy = route[j].y_mm;
	float dx = route[j + 1U].x_mm;
	float dy = route[j + 1U].y_mm;
	float abC = ((bx - ax) * (cy - ay)) - ((by - ay) * (cx - ax));
	float abD = ((bx - ax) * (dy - ay)) - ((by - ay) * (dx - ax));
	float cdA = ((dx - cx) * (ay - cy)) - ((dy - cy) * (ax - cx));
	float cdB = ((dx - cx) * (by - cy)) - ((dy - cy) * (bx - cx));
	return ((abC > 0.0f && abD < 0.0f) || (abC < 0.0f && abD > 0.0f)) &&
		((cdA > 0.0f && cdB < 0.0f) || (cdA < 0.0f && cdB > 0.0f));
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathLength
// 処理概要     経路点列の全長を算出する
// 引数         route: 経路点列
// 戻り値       経路長[mm]
/////////////////////////////////////////////////////////////////////
static float pathLength(const RoutePoint *route)
{
	float length = 0.0f;
	for (uint16_t i = 1U; i < routeCount; i++)
	{
		length += pathPointDistance(route[i - 1U].x_mm, route[i - 1U].y_mm, route[i].x_mm, route[i].y_mm);
	}
	return length;
}
#endif

/////////////////////////////////////////////////////////////////////
// モジュール名 routeGenerateShortcut
// 処理概要     一次経路を制約付きElastic Bandで短縮する
// 引数         shortcutLevel:短縮レベル(1..3)
// 戻り値       true:短縮経路生成成功 false:再走行経路を使用
/////////////////////////////////////////////////////////////////////
bool routeGenerateShortcut(uint8_t shortcutLevel)
{
	memcpy(driveRoute, lineRoute, sizeof(RoutePoint) * routeCount);
	routeShortcutLevel = 0U;
	if (shortcutLevel == 0U || routeCount < 3U) return false;
#if !PATH_SHORTCUT_GEOMETRY_ENABLE
	(void)shortcutLevel;
	return false;
#else
	if (shortcutLevel > PATH_LEVEL_MAX) shortcutLevel = PATH_LEVEL_MAX;
	float legalOffsetMm = PATH_LINE_HALF_WIDTH_MM + PATH_OCCUPIED_HALF_WIDTH_MM -
		PATH_TRACKING_ERROR_BUDGET_MM - PATH_LEGAL_RESERVE_MM;
	if (legalOffsetMm <= 0.0f || PATH_OUTER_RADIUS_MM + legalOffsetMm > PATH_BOARD_CLEARANCE_MM) return false;
	float maxOffsetMm = legalOffsetMm * ((float)shortcutLevel * 0.25f);
	for (uint16_t iteration = 0U; iteration < PATH_SMOOTH_ITERATIONS; iteration++)
	{
		float maxMove = 0.0f;
		for (uint16_t i = 1U; i + 1U < routeCount; i++)
		{
			if ((routeFlags[i] & PATH_FLAG_ANCHOR) != 0U) continue;
			float x = driveRoute[i].x_mm;
			float y = driveRoute[i].y_mm;
			float targetX = 0.5f * ((float)driveRoute[i - 1U].x_mm + (float)driveRoute[i + 1U].x_mm);
			float targetY = 0.5f * ((float)driveRoute[i - 1U].y_mm + (float)driveRoute[i + 1U].y_mm);
			float dx = targetX - x;
			float dy = targetY - y;
			float move = sqrtf((dx * dx) + (dy * dy));
			if (move > PATH_SMOOTH_MOVE_MAX_MM)
			{
				dx *= PATH_SMOOTH_MOVE_MAX_MM / move;
				dy *= PATH_SMOOTH_MOVE_MAX_MM / move;
				move = PATH_SMOOTH_MOVE_MAX_MM;
			}
			x += dx;
			y += dy;
			float fromLineX = x - (float)lineRoute[i].x_mm;
			float fromLineY = y - (float)lineRoute[i].y_mm;
			float fromLine = sqrtf((fromLineX * fromLineX) + (fromLineY * fromLineY));
			if (fromLine > maxOffsetMm)
			{
				x = (float)lineRoute[i].x_mm + (fromLineX * maxOffsetMm / fromLine);
				y = (float)lineRoute[i].y_mm + (fromLineY * maxOffsetMm / fromLine);
			}
			driveRoute[i].x_mm = pathFloatToInt16(x);
			driveRoute[i].y_mm = pathFloatToInt16(y);
			if (move > maxMove) maxMove = move;
		}
		if (maxMove < PATH_SMOOTH_CONVERGED_MM) break;
	}

	for (uint16_t i = 0U; i < routeCount; i++)
	{
		float offset = pathPointDistance(lineRoute[i].x_mm, lineRoute[i].y_mm,
			driveRoute[i].x_mm, driveRoute[i].y_mm);
		if (offset > maxOffsetMm + 0.5f) return false;
	}
	for (uint16_t i = 0U; i + 1U < routeCount; i++)
	{
		for (uint16_t j = (uint16_t)(i + 3U); j + 1U < routeCount; j++)
		{
			if (pathSegmentsIntersect(driveRoute, i, j) && !pathSegmentsIntersect(lineRoute, i, j)) return false;
		}
	}
	float sourceLength = pathLength(lineRoute);
	float shortcutLength = pathLength(driveRoute);
	if (sourceLength <= 0.0f || shortcutLength > sourceLength * 0.995f)
	{
		memcpy(driveRoute, lineRoute, sizeof(RoutePoint) * routeCount);
		return false;
	}
	pathComputeHeadings(driveRoute, routeCount);
	pathBuildSpeedProfile(driveRoute, routeCount, shortcutLevel);
	routeShortcutLevel = shortcutLevel;
	return true;
#endif
}

/////////////////////////////////////////////////////////////////////
// モジュール名 routeBuildFromLog
// 処理概要     一次走行CSVから40mm間隔の再走行経路を生成する
// 引数         logNumber:ログ番号, shortcutLevel:短縮レベル
// 戻り値       経路点数、負値はエラー
/////////////////////////////////////////////////////////////////////
int16_t routeBuildFromLog(int logNumber, uint8_t shortcutLevel)
{
	FIL file;
	FRESULT result;
	RouteCsvColumns columns;
	char fileName[16];
	float firstX = 0.0f, firstY = 0.0f, lastX = 0.0f, lastY = 0.0f;
	float previousX = 0.0f, previousY = 0.0f, totalLength = 0.0f;
	bool havePoint = false;
	uint8_t marker = 0U;
	bool lockAcquired = sd_fatfs_lock(500U);
	if (!lockAcquired) return -9;

	snprintf(fileName, sizeof(fileName), "%d.csv", logNumber);
	result = f_open(&file, fileName, FA_OPEN_EXISTING | FA_READ);
	if (result != FR_OK)
	{
		sd_fatfs_unlock();
		return -5;
	}
	if (f_gets(routeCsvLine, sizeof(routeCsvLine), &file) == NULL || !pathParseHeader(routeCsvLine, &columns))
	{
		f_close(&file);
		sd_fatfs_unlock();
		return -10;
	}
	while (f_gets(routeCsvLine, sizeof(routeCsvLine), &file) != NULL)
	{
		float x, y;
		if (!pathReadCsvPoint(routeCsvLine, &columns, &x, &y, &marker)) continue;
		if (!havePoint)
		{
			firstX = previousX = x;
			firstY = previousY = y;
			havePoint = true;
		}
		else
		{
			totalLength += pathPointDistance(previousX, previousY, x, y);
			previousX = x;
			previousY = y;
		}
		lastX = x;
		lastY = y;
	}
	if (!havePoint || totalLength < PATH_ROUTE_SPACING_MM)
	{
		f_close(&file);
		sd_fatfs_unlock();
		return -11;
	}
	/* 一次走行の終点を固定ゴールへ補正せず、ログの経路形状を保持する。 */
	const float closureX = 0.0f;
	const float closureY = 0.0f;

	/*
	 * 1パス目でEOFまで読み込んだFILをf_lseek()だけで巻き戻すと、
	 * SPI接続のSDカードやFatFsの状態によって2パス目が空読みに
	 * なることがある。いったん閉じて同じCSVを再オープンし、
	 * 2パス目の読込状態を確実に初期化する。
	 */
	f_close(&file);
	result = f_open(&file, fileName, FA_OPEN_EXISTING | FA_READ);
	if (result != FR_OK)
	{
		sd_fatfs_unlock();
		return -13;
	}
	if (f_gets(routeCsvLine, sizeof(routeCsvLine), &file) == NULL ||
		!pathParseHeader(routeCsvLine, &columns))
	{
		f_close(&file);
		sd_fatfs_unlock();
		return -14;
	}
	memset(lineRoute, 0, sizeof(lineRoute));
	memset(driveRoute, 0, sizeof(driveRoute));
	memset(routeFlags, 0, sizeof(routeFlags));
	routeCount = 1U;
	lineRoute[0].x_mm = 0;
	lineRoute[0].y_mm = 0;
	routeFlags[0] = PATH_FLAG_ANCHOR;
	float rawTraversed = 0.0f;
	float accumulated = 0.0f;
	float previousRawX = 0.0f;
	float previousRawY = 0.0f;
	float previousCorrectedX = 0.0f;
	float previousCorrectedY = 0.0f;
	bool firstCorrected = true;
	bool markerPending = false;
	bool routeOverflow = false;
	while (f_gets(routeCsvLine, sizeof(routeCsvLine), &file) != NULL)
	{
		float rawX, rawY;
		if (!pathReadCsvPoint(routeCsvLine, &columns, &rawX, &rawY, &marker)) continue;
		if (!firstCorrected)
		{
			rawTraversed += pathPointDistance(previousRawX, previousRawY, rawX, rawY);
		}
		previousRawX = rawX;
		previousRawY = rawY;
		float progress = fminf(1.0f, rawTraversed / totalLength);
		float blend = progress * progress * (3.0f - (2.0f * progress));
		float correctedX = (rawX - firstX) + (closureX * blend);
		float correctedY = (rawY - firstY) + (closureY * blend);
		if (marker != 0U) markerPending = true;
		if (firstCorrected)
		{
			previousCorrectedX = correctedX;
			previousCorrectedY = correctedY;
			firstCorrected = false;
			continue;
		}

		float segmentStartX = previousCorrectedX;
		float segmentStartY = previousCorrectedY;
		float remainingSegment = pathPointDistance(segmentStartX, segmentStartY, correctedX, correctedY);
		while (accumulated + remainingSegment >= PATH_ROUTE_SPACING_MM)
		{
			if (routeCount >= PATH_ROUTE_MAX_POINTS)
			{
				routeOverflow = true;
				break;
			}
			float needed = PATH_ROUTE_SPACING_MM - accumulated;
			float ratio = needed / remainingSegment;
			segmentStartX += (correctedX - segmentStartX) * ratio;
			segmentStartY += (correctedY - segmentStartY) * ratio;
			lineRoute[routeCount].x_mm = pathFloatToInt16(segmentStartX);
			lineRoute[routeCount].y_mm = pathFloatToInt16(segmentStartY);
			if (markerPending)
			{
				routeFlags[routeCount] |= PATH_FLAG_ANCHOR;
				markerPending = false;
			}
			routeCount++;
			remainingSegment -= needed;
			accumulated = 0.0f;
		}
		accumulated += remainingSegment;
		previousCorrectedX = correctedX;
		previousCorrectedY = correctedY;
		if (routeOverflow) break;
	}
	f_close(&file);
	sd_fatfs_unlock();
	if (routeOverflow) return -7;
	if (routeCount < 2U) return -11;
	lineRoute[routeCount - 1U].x_mm = pathFloatToInt16(lastX - firstX);
	lineRoute[routeCount - 1U].y_mm = pathFloatToInt16(lastY - firstY);
	routeFlags[routeCount - 1U] |= PATH_FLAG_ANCHOR;
	pathExpandAnchors();
	pathComputeHeadings(lineRoute, routeCount);
	pathBuildSpeedProfile(lineRoute, routeCount, 0U);
	memcpy(driveRoute, lineRoute, sizeof(RoutePoint) * routeCount);
	routeSourceLog = (int16_t)logNumber;
	if (shortcutLevel > shortcutSettings.maxLevel) shortcutLevel = shortcutSettings.maxLevel;
	bool shortcutOk = routeGenerateShortcut(shortcutLevel);
	if (!shortcutOk)
	{
		pathBuildSpeedProfile(driveRoute, routeCount, 0U);
		optimalTrace = BOOST_PATH_REPLAY;
	}
	else
	{
		optimalTrace = BOOST_SHORTCUT;
	}
	indexSC = (int16_t)routeCount;
	optimalIndex = 0U;
	saveLogNumber((int16_t)logNumber);
	analyzedNumber = (int16_t)logNumber;
	return (int16_t)routeCount;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathSenseLine
// 処理概要     単一ラインを検出し横偏差を推定する
// 引数         predictedErrorMm: 予測横偏差[mm], measuredErrorMm: 計測偏差の格納先[mm]
// 戻り値       true: 有効な単一ライン false: 無効
/////////////////////////////////////////////////////////////////////
static bool pathSenseLine(float predictedErrorMm, float *measuredErrorMm)
{
	if (!isLineSensorCalibrationValid() || stateCrossLine || fabsf(predictedErrorMm) > PATH_SENSOR_FOV_MM) return false;
	uint32_t sum = 0U;
	float weighted = 0.0f;
	uint8_t first = NUM_SENSORS;
	uint8_t last = 0U;
	uint8_t active = 0U;
	for (uint8_t i = 0U; i < NUM_SENSORS; i++)
	{
		uint16_t value = lSensorCari[i];
		if (value >= PATH_SENSOR_ACTIVE_TH)
		{
			if (first == NUM_SENSORS) first = i;
			last = i;
			active++;
		}
		sum += value;
		weighted += (float)value * (float)i;
	}
	if (sum < PATH_SENSOR_MIN_SUM || active == 0U || (uint8_t)(last - first + 1U) > PATH_SENSOR_MAX_CLUSTER_WIDTH) return false;
	float linePositionMm = ((weighted / (float)sum) - 4.5f) * PATH_SENSOR_PITCH_MM;
	*measuredErrorMm = -linePositionMm;
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathFollowerReset
// 処理概要     スタートマーカー基準で経路追従状態を初期化する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void pathFollowerReset(void)
{
	memset(&pathPose, 0, sizeof(pathPose));
	routeIndex = 0U;
	lostCount = 0U;
	rejoinCount = 0U;
	lineLostCount = 0U;
	pathBlendPermille = 1000U;
	targetSpeedMps = (routeCount > 0U) ? (float)driveRoute[0].speed_cms * 0.01f : 0.0f;
	followerState = (routeCount > 1U) ? PATH_STATE_TRACKING : PATH_STATE_INACTIVE;
	pathLogLinePointX_mm = 0.0f;
	pathLogLinePointY_mm = 0.0f;
	pathLogLineValid = 0U;
	pathLogErrorY_mm = 0.0f;
	pathLogErrorHeading_cdeg = 0;
	pathLogState = (uint8_t)followerState;
	pathLogLegalMargin_mm = PATH_LINE_HALF_WIDTH_MM + PATH_OCCUPIED_HALF_WIDTH_MM;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathFollowerUpdatePose1ms
// 処理概要     エンコーダとジャイロから1ms周期で自己位置を更新する
// 引数         encoderPulse:1ms移動パルス, gyroDegPerSec:角速度[deg/s]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void pathFollowerUpdatePose1ms(int32_t encoderPulse, float gyroDegPerSec)
{
	if (followerState == PATH_STATE_INACTIVE || followerState == PATH_STATE_LOCALIZATION_LOST) return;
	pathPose.heading_deg = pathWrapDeg(pathPose.heading_deg + (gyroDegPerSec * 0.001f));
	float distanceMm = (float)encoderPulse / PULSE_MILLIMETER;
	float headingRad = pathPose.heading_deg * DEG2RAD;
	pathPose.x_mm += distanceMm * sinf(headingRad);
	pathPose.y_mm += distanceMm * cosf(headingRad);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathFollowerUpdateTarget5ms
// 処理概要     近傍経路を投影し、速度と角速度目標を5ms周期で更新する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void pathFollowerUpdateTarget5ms(void)
{
	if (routeCount < 2U || followerState == PATH_STATE_INACTIVE || followerState == PATH_STATE_LOCALIZATION_LOST) return;
	uint16_t begin = (routeIndex > 4U) ? (uint16_t)(routeIndex - 4U) : 0U;
	uint16_t end = (routeIndex + 24U < routeCount) ? (uint16_t)(routeIndex + 24U) : (uint16_t)(routeCount - 1U);
	uint16_t nearest = begin;
	float nearestDistance = pathPointDistance(pathPose.x_mm, pathPose.y_mm, driveRoute[begin].x_mm, driveRoute[begin].y_mm);
	for (uint16_t i = (uint16_t)(begin + 1U); i <= end; i++)
	{
		float distance = pathPointDistance(pathPose.x_mm, pathPose.y_mm, driveRoute[i].x_mm, driveRoute[i].y_mm);
		if (distance < nearestDistance)
		{
			nearestDistance = distance;
			nearest = i;
		}
	}
	if (nearest + 2U < routeIndex) nearest = routeIndex - 2U;
	routeIndex = nearest;
	optimalIndex = routeIndex;

	float lineHeadingDeg = (float)lineRoute[nearest].heading_cdeg * 0.01f;
	float lineHeadingRad = lineHeadingDeg * DEG2RAD;
	float lineDx = pathPose.x_mm - (float)lineRoute[nearest].x_mm;
	float lineDy = pathPose.y_mm - (float)lineRoute[nearest].y_mm;
	float lineError = (lineDx * cosf(lineHeadingRad)) - (lineDy * sinf(lineHeadingRad));
	float measuredError = 0.0f;
	currentLineValid = pathSenseLine(lineError, &measuredError);
	if (currentLineValid)
	{
		float alpha = (float)shortcutSettings.lineAlpha_x1000 * 0.001f;
		float correction = alpha * (measuredError - lineError);
		pathPose.x_mm += correction * cosf(lineHeadingRad);
		pathPose.y_mm -= correction * sinf(lineHeadingRad);
		lineError += correction;
	}
	pathLogLineValid = currentLineValid ? 1U : 0U;
	pathLogLinePointX_mm = lineRoute[nearest].x_mm;
	pathLogLinePointY_mm = lineRoute[nearest].y_mm;

	float lookaheadMm = (float)shortcutSettings.lookaheadBaseMm +
		((float)shortcutSettings.lookaheadPerMpsMm * targetSpeedMps);
	uint16_t lookaheadPoints = (uint16_t)fmaxf(1.0f, ceilf(lookaheadMm / PATH_ROUTE_SPACING_MM));
	uint16_t targetIndex = (nearest + lookaheadPoints < routeCount) ?
		(uint16_t)(nearest + lookaheadPoints) : (uint16_t)(routeCount - 1U);
	float targetHeadingDeg = (float)driveRoute[targetIndex].heading_cdeg * 0.01f;
	float nearestHeadingRad = (float)driveRoute[nearest].heading_cdeg * 0.01f * DEG2RAD;
	float nearestHeadingError = pathWrapDeg(
		((float)driveRoute[nearest].heading_cdeg * 0.01f) - pathPose.heading_deg);
	float dx = pathPose.x_mm - (float)driveRoute[nearest].x_mm;
	float dy = pathPose.y_mm - (float)driveRoute[nearest].y_mm;
	float lateralError = (dx * cosf(nearestHeadingRad)) - (dy * sinf(nearestHeadingRad));
	float headingError = pathWrapDeg(targetHeadingDeg - pathPose.heading_deg);
	float segmentLength = pathPointDistance(driveRoute[nearest].x_mm, driveRoute[nearest].y_mm,
		driveRoute[targetIndex].x_mm, driveRoute[targetIndex].y_mm);
	float curvature = 0.0f;
	if (segmentLength > 1.0f)
	{
		float currentRouteHeading = (float)driveRoute[nearest].heading_cdeg * 0.01f;
		curvature = pathWrapDeg(targetHeadingDeg - currentRouteHeading) * DEG2RAD / segmentLength;
	}
	targetSpeedMps = (float)driveRoute[nearest].speed_cms * 0.01f;
	float feedForwardDegPerSec = (targetSpeedMps * 1000.0f) * curvature * RAD2DEG;
	float kLateral = (float)shortcutSettings.kLateral_x100 * 0.01f;
	float kHeading = (float)shortcutSettings.kHeading_x100 * 0.01f;
	float targetYawRate = feedForwardDegPerSec - (kLateral * lateralError) + (kHeading * headingError);
	if (targetYawRate > 1800.0f) targetYawRate = 1800.0f;
	if (targetYawRate < -1800.0f) targetYawRate = -1800.0f;
	setTargetAngularVelocity(targetYawRate);
	log_targetAngularVelocity = (int32_t)targetYawRate;

	/*
	 * 先読み方位差は操舵目標専用とし、ロスト判定には最近傍経路点の
	 * 接線方位差を使う。急カーブでは先読み方位差が大きくなるため、
	 * これをロスト判定へ流用すると経路近傍でも誤って停止する。
	 */
	if (nearestDistance > PATH_LOST_DISTANCE_MM || fabsf(nearestHeadingError) > PATH_LOST_HEADING_DEG)
	{
		if (lostCount < UINT16_MAX) lostCount++;
	}
	else
	{
		lostCount = 0U;
	}
	if (followerState == PATH_STATE_TRACKING && lostCount >= PATH_LOST_COUNT_5MS)
	{
		if (currentLineValid)
		{
			followerState = PATH_STATE_LINE_FALLBACK;
			pathBlendPermille = 0U;
			lineLostCount = 0U;
		}
		else
		{
			followerState = PATH_STATE_LOCALIZATION_LOST;
		}
	}
	else if (followerState == PATH_STATE_LINE_FALLBACK)
	{
		if (!currentLineValid)
		{
			if (lineLostCount < UINT16_MAX) lineLostCount++;
			if (lineLostCount >= PATH_LINE_LOST_COUNT_5MS) followerState = PATH_STATE_LOCALIZATION_LOST;
		}
		else
		{
			lineLostCount = 0U;
			if (nearestDistance < PATH_REJOIN_DISTANCE_MM && fabsf(headingError) < PATH_REJOIN_HEADING_DEG)
			{
				if (rejoinCount < UINT16_MAX) rejoinCount++;
				if (rejoinCount >= PATH_REJOIN_COUNT_5MS)
				{
					followerState = PATH_STATE_REJOIN;
					pathBlendPermille = 0U;
				}
			}
			else
			{
				rejoinCount = 0U;
			}
		}
	}
	else if (followerState == PATH_STATE_REJOIN)
	{
		if (pathBlendPermille + PATH_REJOIN_BLEND_STEP >= 1000U)
		{
			pathBlendPermille = 1000U;
			followerState = PATH_STATE_TRACKING;
			lostCount = 0U;
			rejoinCount = 0U;
		}
		else
		{
			pathBlendPermille += PATH_REJOIN_BLEND_STEP;
		}
	}

	float lineOffset = pathPointDistance(lineRoute[nearest].x_mm, lineRoute[nearest].y_mm,
		driveRoute[nearest].x_mm, driveRoute[nearest].y_mm);
	pathLogErrorY_mm = lateralError;
	pathLogErrorHeading_cdeg = pathFloatToInt16(headingError * 100.0f);
	pathLogState = (uint8_t)followerState;
	pathLogLegalMargin_mm = PATH_LINE_HALF_WIDTH_MM + PATH_OCCUPIED_HALF_WIDTH_MM -
		lineOffset - PATH_TRACKING_ERROR_BUDGET_MM;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathFollowerGetStatus
// 処理概要     経路追従状態を返す
// 引数         なし
// 戻り値       経路追従状態
/////////////////////////////////////////////////////////////////////
PathFollowerState pathFollowerGetStatus(void) { return followerState; }
/////////////////////////////////////////////////////////////////////
// モジュール名 pathFollowerGetPathBlendPermille
// 処理概要     経路操舵の混合率を返す
// 引数         なし
// 戻り値       経路操舵混合率[permille]
/////////////////////////////////////////////////////////////////////
uint16_t pathFollowerGetPathBlendPermille(void) { return pathBlendPermille; }
/////////////////////////////////////////////////////////////////////
// モジュール名 pathFollowerGetTargetSpeedMps
// 処理概要     現在の経路目標速度を返す
// 引数         なし
// 戻り値       目標速度[m/s]
/////////////////////////////////////////////////////////////////////
float pathFollowerGetTargetSpeedMps(void) { return targetSpeedMps; }
/////////////////////////////////////////////////////////////////////
// モジュール名 pathFollowerLineIsValid
// 処理概要     位置補正用ライン検出の有効状態を返す
// 引数         なし
// 戻り値       true: 有効 false: 無効
/////////////////////////////////////////////////////////////////////
bool pathFollowerLineIsValid(void) { return currentLineValid; }
/////////////////////////////////////////////////////////////////////
// モジュール名 pathRouteCount
// 処理概要     現在の経路点数を返す
// 引数         なし
// 戻り値       経路点数
/////////////////////////////////////////////////////////////////////
uint16_t pathRouteCount(void) { return routeCount; }
/////////////////////////////////////////////////////////////////////
// モジュール名 pathFollowerGoalWindowOpen
// 処理概要     経路終端付近に到達してゴールマーカー終了を許可できるか判定する
// 引数         なし
// 戻り値       true:終端付近 false:走行途中
/////////////////////////////////////////////////////////////////////
bool pathFollowerGoalWindowOpen(void)
{
	if (routeCount < 2U) return false;
	return ((uint32_t)routeIndex * 1000U) >= ((uint32_t)(routeCount - 1U) * PATH_GOAL_MIN_PROGRESS_PERMILLE);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 pathRouteSourceLog
// 処理概要     経路生成元のログ番号を返す
// 引数         なし
// 戻り値       ログ番号
/////////////////////////////////////////////////////////////////////
int16_t pathRouteSourceLog(void) { return routeSourceLog; }
/////////////////////////////////////////////////////////////////////
// モジュール名 pathRouteShortcutLevel
// 処理概要     適用済みショートカットレベルを返す
// 引数         なし
// 戻り値       ショートカットレベル
/////////////////////////////////////////////////////////////////////
uint8_t pathRouteShortcutLevel(void) { return routeShortcutLevel; }

/////////////////////////////////////////////////////////////////////
// モジュール名 pathSettingInRange
// 処理概要     設定値が許容範囲内か判定する
// 引数         value: 設定値, minimum: 下限, maximum: 上限
// 戻り値       true: 範囲内 false: 範囲外
/////////////////////////////////////////////////////////////////////
static bool pathSettingInRange(int value, int minimum, int maximum)
{
	return value >= minimum && value <= maximum;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 writeShortcutSettings
// 処理概要     ショートカット設定を固定CSV形式でSDカードへ保存する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void writeShortcutSettings(void)
{
	FIL file;
	if (f_open(&file, PATH_SETTING_FILE, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
	{
		f_printf(&file, "%u,%03u,%03u,%04u,%04u,%03u",
			shortcutSettings.maxLevel, shortcutSettings.lookaheadBaseMm,
			shortcutSettings.lookaheadPerMpsMm, shortcutSettings.kLateral_x100,
			shortcutSettings.kHeading_x100, shortcutSettings.lineAlpha_x1000);
		f_close(&file);
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 readShortcutSettings
// 処理概要     ショートカット設定を部分反映し、不正時は修復する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void readShortcutSettings(void)
{
	FIL file;
	char text[64] = {0};
	int values[6] = {PATH_DEFAULT_MAX_LEVEL, PATH_DEFAULT_LOOKAHEAD_BASE_MM,
		PATH_DEFAULT_LOOKAHEAD_PER_MPS_MM, PATH_DEFAULT_KLATERAL_X100,
		PATH_DEFAULT_KHEADING_X100, PATH_DEFAULT_LINE_ALPHA_X1000};
	ShortcutSettings defaults = {PATH_DEFAULT_MAX_LEVEL, PATH_DEFAULT_LOOKAHEAD_BASE_MM,
		PATH_DEFAULT_LOOKAHEAD_PER_MPS_MM, PATH_DEFAULT_KLATERAL_X100,
		PATH_DEFAULT_KHEADING_X100, PATH_DEFAULT_LINE_ALPHA_X1000};
	shortcutSettings = defaults;
	bool repair = false;
	int parsed = 0;
	if (f_open(&file, PATH_SETTING_FILE, FA_OPEN_EXISTING | FA_READ) == FR_OK)
	{
		if (f_gets(text, sizeof(text), &file) != NULL)
		{
			parsed = sscanf(text, "%d,%d,%d,%d,%d,%d", &values[0], &values[1], &values[2],
				&values[3], &values[4], &values[5]);
		}
		else repair = true;
		f_close(&file);
	}
	else repair = true;

	if (parsed >= 1 && pathSettingInRange(values[0], 0, PATH_LEVEL_MAX)) shortcutSettings.maxLevel = (uint8_t)values[0]; else repair = true;
	if (parsed >= 2 && pathSettingInRange(values[1], PATH_LOOKAHEAD_BASE_MIN_MM, PATH_LOOKAHEAD_BASE_MAX_MM)) shortcutSettings.lookaheadBaseMm = (uint16_t)values[1]; else repair = true;
	if (parsed >= 3 && pathSettingInRange(values[2], 0, PATH_LOOKAHEAD_SPEED_MAX_MM)) shortcutSettings.lookaheadPerMpsMm = (uint16_t)values[2]; else repair = true;
	if (parsed >= 4 && pathSettingInRange(values[3], 0, PATH_KLATERAL_MAX_X100)) shortcutSettings.kLateral_x100 = (uint16_t)values[3]; else repair = true;
	if (parsed >= 5 && pathSettingInRange(values[4], 0, PATH_KHEADING_MAX_X100)) shortcutSettings.kHeading_x100 = (uint16_t)values[4]; else repair = true;
	if (parsed >= 6 && pathSettingInRange(values[5], 0, PATH_LINE_ALPHA_MAX_X1000)) shortcutSettings.lineAlpha_x1000 = (uint16_t)values[5]; else repair = true;
	if (repair) writeShortcutSettings();
}
