#include "pathFollower.h"
#include "runGuard.h"
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
#define PATH_DEFAULT_LINE_ALPHA_X1000         10U
#define PATH_DEFAULT_LINE_THETA_GAIN_X1E9      0U

#define PATH_LEVEL_MAX                        1U // 初期実機検証はLevel 1だけを許可する
#define PATH_LOOKAHEAD_BASE_MIN_MM            40U
#define PATH_LOOKAHEAD_BASE_MAX_MM            300U
#define PATH_LOOKAHEAD_SPEED_MAX_MM            200U
#define PATH_KLATERAL_MAX_X100                10000U
#define PATH_KHEADING_MAX_X100                3000U
#define PATH_LINE_ALPHA_MAX_X1000             100U
#define PATH_LINE_THETA_GAIN_MAX_X1E9         1000U

#define PATH_CSV_LINE_SIZE                    6144U
#define PATH_CORRIDOR_MIN_SPAN_POINTS         15U   // 600mm
#define PATH_CORRIDOR_MAX_SPAN_POINTS         40U   // 1600mm
#define PATH_CORRIDOR_TRANSITION_POINTS       3U    // 120mm
#define PATH_CORRIDOR_END_GUARD_POINTS        4U
#define PATH_CORRIDOR_OVERLAP_GUARD_POINTS    5U
#define PATH_CORRIDOR_MAX_COUNT               16U
#define PATH_CORRIDOR_HEADING_LIMIT_DEG       12.0f
#define PATH_CORRIDOR_TURN_THRESHOLD_DEG      2.0f
#define PATH_CORRIDOR_MIN_SIGN_CHANGES        3U
#define PATH_CORRIDOR_MAX_OFFSET_MM           34.0f
#define PATH_CORRIDOR_OFFSET_TOLERANCE_MM     0.5f
#define PATH_CORRIDOR_MIN_PROJECTION          -0.02f
#define PATH_CORRIDOR_MAX_PROJECTION          1.02f
#define PATH_CORRIDOR_MIN_SAVING_MM           5.0f

// 寸法入力値はすべて[mm]。機体座標原点は前車軸中心と後車軸中心の中点とし、
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
#define PATH_ASSOCIATION_PROGRESS_MARGIN_MM   120.0f
#define PATH_ASSOCIATION_HEADING_MAX_DEG      60.0f
#define PATH_GOAL_RESERVED_POINTS             13U  // 終端から原点まで1040mm以下なら延長点を確保
#define PATH_REJOIN_BLEND_STEP                50U
#define PATH_SENSOR_ACTIVE_TH                 800U
#define PATH_SENSOR_MIN_SUM                   1200U
#define PATH_SENSOR_MAX_CLUSTER_WIDTH         3U
#define PATH_SENSOR_FOV_MM                    35.0f  // ライン位置補正を許可する予測横偏差範囲
#define PATH_LINE_MATCH_BACK_POINTS           2U
#define PATH_LINE_MATCH_FORWARD_POINTS        4U
#define PATH_LINE_MATCH_RESIDUAL_MAX_MM        30.0f
#define PATH_LINE_POSITION_CORRECTION_MAX_MM    1.0f
#define PATH_LINE_HEADING_CORRECTION_MAX_DEG    0.1f

#define PATH_FLAG_CORRIDOR_RESERVED           0x01U
#define PATH_FLAG_CORRIDOR_APPLIED            0x02U

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t pulse;
	uint32_t expectedRows;
	bool closureValid;
} RouteCsvColumns;

typedef struct
{
	float x_mm;
	float y_mm;
	float heading_deg;
} PathPose;

typedef struct
{
	float lateral_mm;
	float forward_mm;
} PathLineObservation;

typedef struct
{
	float x_mm;
	float y_mm;
	float residual_mm;
} PathLineMatch;

static RoutePoint lineRoute[PATH_ROUTE_MAX_POINTS];
static RoutePoint driveRoute[PATH_ROUTE_MAX_POINTS];
static uint16_t driveRouteArcMm[PATH_ROUTE_MAX_POINTS];
static uint8_t routeFlags[PATH_ROUTE_MAX_POINTS];
static char routeCsvLine[PATH_CSV_LINE_SIZE];
static uint16_t routeCount = 0U;
static int16_t routeSourceLog = 0;
static uint8_t routeShortcutRequestedLevel = 0U;
static uint8_t routeShortcutLevel = 0U;
static uint8_t routeShortcutBuildStatus = PATH_SHORTCUT_BUILD_NOT_REQUESTED;
static uint8_t routeShortcutCorridorCount = 0U;
static float routeShortcutReductionMm = 0.0f;
static uint32_t routeGeometryCrc32 = 0U;
static ShortcutSettings routeGenerationSettings = {0U, 0U, 0U, 0U, 0U, 0U, 0U};
static ShortcutSettings runStartSettings = {0U, 0U, 0U, 0U, 0U, 0U, 0U};
static ShortcutSettings runGenerationSettings = {0U, 0U, 0U, 0U, 0U, 0U, 0U};
static int16_t runRouteSourceLog = 0;
static uint8_t runRouteRequestedLevel = 0U;
static uint8_t runRouteShortcutLevel = 0U;
static uint8_t runRouteShortcutBuildStatus = PATH_SHORTCUT_BUILD_NOT_REQUESTED;
static uint8_t runRouteShortcutCorridorCount = 0U;
static float runRouteShortcutReductionMm = 0.0f;
static uint16_t runRouteCount = 0U;
static uint32_t runRouteGeometryCrc32 = 0U;
static bool runStartSettingsValid = false;
static PathPose pathPose;
static PathFollowerState followerState = PATH_STATE_INACTIVE;
static uint16_t routeIndex = 0U;
static uint16_t lostCount = 0U;
static uint16_t rejoinCount = 0U;
static uint16_t lineLostCount = 0U;
static uint16_t pathBlendPermille = 1000U;
static float targetSpeedMps = 0.0f;
static float pathTravelMm = 0.0f;
static uint16_t pathGoalArcMm = 0U;
static bool pathGoalValid = false;
static bool currentLineValid = false;

/////////////////////////////////////////////////////////////////////
// モジュール名 pathCrc32UpdateByte
// 処理概要     CRC32へ1バイトをリトルエンディアン順で追加する
// 引数         crc:現在のCRC, value:追加するバイト
// 戻り値       更新後のCRC
/////////////////////////////////////////////////////////////////////
static uint32_t pathCrc32UpdateByte(uint32_t crc, uint8_t value)
{
	crc ^= value;
	for (uint8_t bit = 0U; bit < 8U; bit++)
	{
		crc = (crc & 1U) ? ((crc >> 1U) ^ 0xEDB88320UL) : (crc >> 1U);
	}
	return crc;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathCrc32UpdateU16LE
// 処理概要     16bit値を固定リトルエンディアン形式でCRC32へ追加する
// 引数         crc:現在のCRC, value:追加する16bit値
// 戻り値       更新後のCRC
/////////////////////////////////////////////////////////////////////
static uint32_t pathCrc32UpdateU16LE(uint32_t crc, uint16_t value)
{
	crc = pathCrc32UpdateByte(crc, (uint8_t)(value & 0xFFU));
	return pathCrc32UpdateByte(crc, (uint8_t)(value >> 8U));
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathComputeRouteGeometryCrc32
// 処理概要    生成済み点数と元経路・走行経路の整数XY座標からCRC32を計算する
// 引数         なし
// 戻り値       経路形状CRC32
/////////////////////////////////////////////////////////////////////
static uint32_t pathComputeRouteGeometryCrc32(void)
{
	uint32_t crc = 0xFFFFFFFFUL;
	crc = pathCrc32UpdateU16LE(crc, routeCount);
	for (uint16_t i = 0U; i < routeCount; i++)
	{
		crc = pathCrc32UpdateU16LE(crc, (uint16_t)lineRoute[i].x_mm);
		crc = pathCrc32UpdateU16LE(crc, (uint16_t)lineRoute[i].y_mm);
		crc = pathCrc32UpdateU16LE(crc, (uint16_t)driveRoute[i].x_mm);
		crc = pathCrc32UpdateU16LE(crc, (uint16_t)driveRoute[i].y_mm);
	}
	return crc ^ 0xFFFFFFFFUL;
}

// KiCad基板座標と実機のsensor[0]=左端、sensor[9]=右端を照合した受光中心横座標[mm]。
static const float pathSensorLateralMm[NUM_SENSORS] = {
	-41.70f, -35.04f, -27.29f, -18.68f, -9.49f,
	9.50f, 18.68f, 27.29f, 35.05f, 41.70f
};

// 中央受光中心の実測前方距離95mmとKiCad相対座標から求めた受光中心前方座標[mm]。
static const float pathSensorForwardMm[NUM_SENSORS] = {
	76.07f, 82.88f, 88.42f, 92.50f, 95.00f,
	95.00f, 92.50f, 88.42f, 82.88f, 76.07f
};

ShortcutSettings shortcutSettings = {0U, PATH_DEFAULT_LOOKAHEAD_BASE_MM,
	PATH_DEFAULT_LOOKAHEAD_PER_MPS_MM, PATH_DEFAULT_KLATERAL_X100,
	PATH_DEFAULT_KHEADING_X100, PATH_DEFAULT_LINE_ALPHA_X1000,
	PATH_DEFAULT_LINE_THETA_GAIN_X1E9};
float pathLogLinePointX_mm = 0.0f;
float pathLogLinePointY_mm = 0.0f;
uint8_t pathLogLineValid = 0U;
float pathLogErrorY_mm = 0.0f;
int16_t pathLogErrorHeading_cdeg = 0;
uint8_t pathLogState = PATH_STATE_INACTIVE;
float pathLogLegalMargin_mm = 0.0f;
float pathLogLineMatchResidual_mm = 0.0f;
uint16_t pathLogPoseCorrection_um = 0U;
int16_t pathLogPoseCorrectionHeading_cdeg = 0;

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
// モジュール名 pathBuildDriveRouteArcLength
// 処理概要     実走行経路の各点までの累積弧長を算出する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void pathBuildDriveRouteArcLength(void)
{
	memset(driveRouteArcMm, 0, sizeof(driveRouteArcMm));
	pathGoalArcMm = 0U;
	pathGoalValid = false;
	float accumulatedMm = 0.0f;
	for (uint16_t i = 1U; i < routeCount; i++)
	{
		accumulatedMm += pathPointDistance(driveRoute[i - 1U].x_mm, driveRoute[i - 1U].y_mm,
			driveRoute[i].x_mm, driveRoute[i].y_mm);
		float storedMm = fminf(accumulatedMm, (float)UINT16_MAX);
		driveRouteArcMm[i] = (uint16_t)lroundf(storedMm);
	}
	if (routeCount >= 2U)
	{
		pathGoalArcMm = driveRouteArcMm[routeCount - 1U];
		pathGoalValid = true;
	}
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
	columns->pulse = -1;

	while (*p != '\0' && *p != '\r' && *p != '\n')
	{
		if (*p == ',')
		{
			if (pathHeaderFieldEquals(start, p, "x")) columns->x = column;
			else if (pathHeaderFieldEquals(start, p, "y")) columns->y = column;
			else if (pathHeaderFieldEquals(start, p, "encTotalOptimal")) columns->pulse = column;
			column++;
			start = p + 1;
		}
		p++;
	}
	if (pathHeaderFieldEquals(start, p, "x")) columns->x = column;
	else if (pathHeaderFieldEquals(start, p, "y")) columns->y = column;
	else if (pathHeaderFieldEquals(start, p, "encTotalOptimal")) columns->pulse = column;
	return columns->x >= 0 && columns->y >= 0 && columns->pulse >= 0;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathReadColumnHeader
// 処理概要     旧混在形式または新2行形式から経路ログ列名を読み込む
// 引数         file:読込ファイル, columns:列番号の格納先
// 戻り値       true:必要列あり false:読込失敗または必要列なし
/////////////////////////////////////////////////////////////////////
static bool pathReadColumnHeader(FIL *file, RouteCsvColumns *columns)
{
	if (f_gets(routeCsvLine, sizeof(routeCsvLine), file) == NULL ||
		(strchr(routeCsvLine, '\n') == NULL && strchr(routeCsvLine, '\r') == NULL))
	{
		return false;
	}
	const bool supportedVersion = strstr(routeCsvLine, "logSchemaVersion=10,") != NULL;
	const char *valid = strstr(routeCsvLine, "closureValid=1,");
	const char *reason = strstr(routeCsvLine, "closureReason=0,");
	const char *mode = strstr(routeCsvLine, "optimalTrace=0.00,");
	const char *stop = strstr(routeCsvLine, "emcStop=0.00,");
	const char *scale = strstr(routeCsvLine, "encoderPulsePerMeter=");
	char *scaleEnd = NULL;
	const unsigned long sourcePulsePerMeter = (scale != NULL) ?
		strtoul(scale + strlen("encoderPulsePerMeter="), &scaleEnd, 10) : 0UL;
	const char *calibrated = strstr(routeCsvLine, "imuCalibrationValid=1,");
	const char *calibrationSamples = strstr(routeCsvLine, "imuCalibrationSamples=100,");
	const char *calibrationErrors = strstr(routeCsvLine, "imuCalibrationReadErrors=0,");
	const char *distanceVerified = strstr(routeCsvLine, "distanceScaleVerified=1,");
	const char *expectedRows = strstr(routeCsvLine, "logExpectedRows=");
	columns->closureValid = (supportedVersion && valid != NULL && reason != NULL &&
		mode != NULL && stop != NULL && scaleEnd != NULL && *scaleEnd == ',' &&
		sourcePulsePerMeter == PULSE_METER && calibrated != NULL &&
		calibrationSamples != NULL && calibrationErrors != NULL && distanceVerified != NULL);
	if (!columns->closureValid) return false;
	columns->expectedRows = (expectedRows != NULL) ?
		(uint32_t)strtoul(expectedRows + strlen("logExpectedRows="), NULL, 10) : 0U;
	if (expectedRows == NULL || columns->expectedRows == 0U) return false;
	if (f_gets(routeCsvLine, sizeof(routeCsvLine), file) == NULL ||
		(strchr(routeCsvLine, '\n') == NULL && strchr(routeCsvLine, '\r') == NULL))
	{
		return false;
	}
	return pathParseHeader(routeCsvLine, columns);
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
	if (*end != ',' && *end != '\r' && *end != '\n' && *end != '\0') return false;
	*value = parsed;
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathReadCsvPoint
// 処理概要     CSV行からジャイロ経路座標と累積距離を読む
// 引数         line: CSV行, columns: 列情報, xMm/yMm: 座標[mm], pulse: 累積パルス
// 戻り値       true: 読取成功 false: 読取失敗
/////////////////////////////////////////////////////////////////////
static bool pathReadCsvPoint(const char *line, const RouteCsvColumns *columns,
	float *xMm, float *yMm, float *pulse)
{
	if (!pathCsvFloatAt(line, columns->x, xMm) ||
		!pathCsvFloatAt(line, columns->y, yMm) ||
		!pathCsvFloatAt(line, columns->pulse, pulse))
	{
		return false;
	}
	if (!isfinite(*xMm) || !isfinite(*yMm) || !isfinite(*pulse)) return false;
	return true;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 pathAppendRouteSample
// 処理概要     経路を40mm間隔で再標本化する
// 引数         xMm/yMm: 終点[mm], previousX/Y: 前点,
//              accumulated: 残距離[mm]
// 戻り値       true:追加成功 false:点数上限
/////////////////////////////////////////////////////////////////////
static bool pathAppendRouteSample(float xMm, float yMm,
	float *previousX, float *previousY, float *accumulated)
{
	float startX = *previousX;
	float startY = *previousY;
	float remaining = pathPointDistance(startX, startY, xMm, yMm);
	while (remaining > 0.0f && *accumulated + remaining >= PATH_ROUTE_SPACING_MM)
	{
		if (routeCount >= PATH_ROUTE_MAX_POINTS - PATH_GOAL_RESERVED_POINTS) return false;
		float needed = PATH_ROUTE_SPACING_MM - *accumulated;
		float ratio = needed / remaining;
		startX += (xMm - startX) * ratio;
		startY += (yMm - startY) * ratio;
		lineRoute[routeCount].x_mm = pathFloatToInt16(startX);
		lineRoute[routeCount].y_mm = pathFloatToInt16(startY);
		routeCount++;
		remaining -= needed;
		*accumulated = 0.0f;
	}
	*accumulated += remaining;
	*previousX = xMm;
	*previousY = yMm;
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
	float speedCap = (shortcutLevel == 0U) ? tgtParam.pathReplay : tgtParam.shortCut;
	if (speedCap <= 0.0f) speedCap = tgtParam.pathReplay;
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
// モジュール名 pathExtendDriveRouteTowardOrigin
// 処理概要     一次走行終端と座標原点の中間点まで実走行経路を延長する
// 引数         shortcutLevel: 実走行経路へ適用済みの短縮レベル
// 戻り値       true:延長成功 false:延長不可
/////////////////////////////////////////////////////////////////////
static bool pathExtendDriveRouteTowardOrigin(uint8_t shortcutLevel)
{
	if (routeCount < 2U) return false;
	float endX = (float)driveRoute[routeCount - 1U].x_mm;
	float endY = (float)driveRoute[routeCount - 1U].y_mm;
	float distanceToOriginMm = sqrtf((endX * endX) + (endY * endY));
	float goalExtensionMm = distanceToOriginMm * 0.5f;
	if (!isfinite(goalExtensionMm) || goalExtensionMm < 1.0f) return false;
	if (goalExtensionMm > (float)(PATH_ROUTE_MAX_POINTS - routeCount) * PATH_ROUTE_SPACING_MM) return false;
	uint16_t appendCount = (uint16_t)ceilf(goalExtensionMm / PATH_ROUTE_SPACING_MM);
	if ((uint32_t)routeCount + appendCount > PATH_ROUTE_MAX_POINTS) return false;

	float unitX = -endX / distanceToOriginMm;
	float unitY = -endY / distanceToOriginMm;
	float advancedMm = 0.0f;
	for (uint16_t i = 0U; i < appendCount; i++)
	{
		float nextAdvancedMm = fminf(goalExtensionMm,
			advancedMm + PATH_ROUTE_SPACING_MM);
		int16_t extensionX = pathFloatToInt16(endX + (unitX * nextAdvancedMm));
		int16_t extensionY = pathFloatToInt16(endY + (unitY * nextAdvancedMm));
		driveRoute[routeCount].x_mm = extensionX;
		driveRoute[routeCount].y_mm = extensionY;
		driveRoute[routeCount].heading_cdeg = 0;
		driveRoute[routeCount].speed_cms = 0U;
		lineRoute[routeCount].x_mm = extensionX;
		lineRoute[routeCount].y_mm = extensionY;
		lineRoute[routeCount].heading_cdeg = 0;
		lineRoute[routeCount].speed_cms = 0U;
		routeFlags[routeCount] = 0U;
		routeCount++;
		advancedMm = nextAdvancedMm;
	}
	pathComputeHeadings(lineRoute, routeCount);
	pathComputeHeadings(driveRoute, routeCount);
	pathBuildSpeedProfile(driveRoute, routeCount, shortcutLevel);
	return true;
}

#if PATH_SHORTCUT_GEOMETRY_ENABLE
typedef struct
{
	uint16_t begin;
	uint16_t end;
	float saving_mm;
	bool valid;
} PathCorridorCandidate;

/////////////////////////////////////////////////////////////////////
// モジュール名 pathCorridorSmoothstep
// 処理概要     直線回廊の入口と出口を滑らかに接続する係数を算出する
// 引数         value: 0～1の進捗
// 戻り値       0～1の補間係数
/////////////////////////////////////////////////////////////////////
static float pathCorridorSmoothstep(float value)
{
	value = fminf(1.0f, fmaxf(0.0f, value));
	return value * value * (3.0f - (2.0f * value));
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathComputeCorridorHeadings
// 処理概要     直線回廊抽出用に前後2点から一次経路の接線方位を算出する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void pathComputeCorridorHeadings(void)
{
	for (uint16_t i = 0U; i < routeCount; i++)
	{
		uint16_t before = (i > 2U) ? (uint16_t)(i - 2U) : 0U;
		uint16_t after = (i + 2U < routeCount) ? (uint16_t)(i + 2U) : (uint16_t)(routeCount - 1U);
		float dx = (float)lineRoute[after].x_mm - (float)lineRoute[before].x_mm;
		float dy = (float)lineRoute[after].y_mm - (float)lineRoute[before].y_mm;
		driveRoute[i].heading_cdeg = pathFloatToInt16(pathWrapDeg(atan2f(dx, dy) * RAD2DEG) * 100.0f);
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathCorridorRangeIsFree
// 処理概要     候補区間が既採用回廊の保護範囲と重ならないことを確認する
// 引数         begin,end: 候補区間の先頭と終端index
// 戻り値       true:重なりなし false:重なりあり
/////////////////////////////////////////////////////////////////////
static bool pathCorridorRangeIsFree(uint16_t begin, uint16_t end)
{
	for (uint16_t i = begin; i <= end; i++)
	{
		if ((routeFlags[i] & PATH_FLAG_CORRIDOR_RESERVED) != 0U) return false;
	}
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathEvaluateCorridorCandidate
// 処理概要     一次経路の区間が直線回廊の抽出条件を満たすか評価する
// 引数         begin,end: 候補区間の先頭と終端index, savingMm:短縮量格納先[mm]
// 戻り値       true:候補成立 false:候補不成立
/////////////////////////////////////////////////////////////////////
static bool pathEvaluateCorridorCandidate(uint16_t begin, uint16_t end, float *savingMm)
{
	float dx = (float)lineRoute[end].x_mm - (float)lineRoute[begin].x_mm;
	float dy = (float)lineRoute[end].y_mm - (float)lineRoute[begin].y_mm;
	float chordLengthSquared = (dx * dx) + (dy * dy);
	if (chordLengthSquared <= 0.0f) return false;
	float chordHeadingDeg = atan2f(dx, dy) * RAD2DEG;
	float beginHeadingDeg = (float)driveRoute[begin].heading_cdeg * 0.01f;
	float endHeadingDeg = (float)driveRoute[end].heading_cdeg * 0.01f;
	if (fabsf(pathWrapDeg(beginHeadingDeg - chordHeadingDeg)) > PATH_CORRIDOR_HEADING_LIMIT_DEG ||
		fabsf(pathWrapDeg(endHeadingDeg - chordHeadingDeg)) > PATH_CORRIDOR_HEADING_LIMIT_DEG) return false;

	float sourceLength = 0.0f;
	for (uint16_t i = begin; i <= end; i++)
	{
		float pointX = (float)lineRoute[i].x_mm - (float)lineRoute[begin].x_mm;
		float pointY = (float)lineRoute[i].y_mm - (float)lineRoute[begin].y_mm;
		float progress = ((pointX * dx) + (pointY * dy)) / chordLengthSquared;
		if (progress < PATH_CORRIDOR_MIN_PROJECTION || progress > PATH_CORRIDOR_MAX_PROJECTION) return false;
		float projectedX = (float)lineRoute[begin].x_mm + (progress * dx);
		float projectedY = (float)lineRoute[begin].y_mm + (progress * dy);
		float offset = pathPointDistance(lineRoute[i].x_mm, lineRoute[i].y_mm, projectedX, projectedY);
		if (offset > PATH_CORRIDOR_MAX_OFFSET_MM) return false;
		if (i > begin)
		{
			sourceLength += pathPointDistance(lineRoute[i - 1U].x_mm, lineRoute[i - 1U].y_mm,
				lineRoute[i].x_mm, lineRoute[i].y_mm);
		}
	}

	uint8_t signChanges = 0U;
	int8_t previousSign = 0;
	for (uint16_t i = (uint16_t)(begin + 2U); i < (uint16_t)(end - 1U); i++)
	{
		float beforeHeading = (float)driveRoute[i - 2U].heading_cdeg * 0.01f;
		float afterHeading = (float)driveRoute[i + 2U].heading_cdeg * 0.01f;
		float delta = pathWrapDeg(afterHeading - beforeHeading);
		if (fabsf(delta) < PATH_CORRIDOR_TURN_THRESHOLD_DEG) continue;
		int8_t sign = (delta > 0.0f) ? 1 : -1;
		if (previousSign != 0 && sign != previousSign) signChanges++;
		previousSign = sign;
	}
	if (signChanges < PATH_CORRIDOR_MIN_SIGN_CHANGES) return false;

	*savingMm = sourceLength - sqrtf(chordLengthSquared);
	return *savingMm >= PATH_CORRIDOR_MIN_SAVING_MM;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathFindBestCorridorCandidate
// 処理概要     未使用区間から短縮量が最大の直線回廊候補を取得する
// 引数         candidate:候補格納先
// 戻り値       true:候補あり false:候補なし
/////////////////////////////////////////////////////////////////////
static bool pathFindBestCorridorCandidate(PathCorridorCandidate *candidate)
{
	candidate->valid = false;
	for (uint16_t begin = PATH_CORRIDOR_END_GUARD_POINTS;
		begin + PATH_CORRIDOR_MIN_SPAN_POINTS + PATH_CORRIDOR_END_GUARD_POINTS < routeCount; begin++)
	{
		for (uint16_t span = PATH_CORRIDOR_MIN_SPAN_POINTS; span <= PATH_CORRIDOR_MAX_SPAN_POINTS; span++)
		{
			uint16_t end = (uint16_t)(begin + span);
			if (end + PATH_CORRIDOR_END_GUARD_POINTS >= routeCount) break;
			if (!pathCorridorRangeIsFree(begin, end)) continue;
			float savingMm = 0.0f;
			if (!pathEvaluateCorridorCandidate(begin, end, &savingMm)) continue;
			uint16_t bestSpan = candidate->valid ? (uint16_t)(candidate->end - candidate->begin) : 0U;
			if (!candidate->valid || savingMm > candidate->saving_mm ||
				(savingMm == candidate->saving_mm && span > bestSpan))
			{
				candidate->begin = begin;
				candidate->end = end;
				candidate->saving_mm = savingMm;
				candidate->valid = true;
			}
		}
	}
	return candidate->valid;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathApplyCorridorCandidate
// 処理概要     候補区間を直線へ寄せて入口と出口を120mmで滑らかに接続する
// 引数         candidate:適用する直線回廊候補
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void pathApplyCorridorCandidate(const PathCorridorCandidate *candidate)
{
	uint16_t span = (uint16_t)(candidate->end - candidate->begin);
	for (uint16_t i = candidate->begin; i <= candidate->end; i++)
	{
		float progress = (float)(i - candidate->begin) / (float)span;
		float chordX = (float)lineRoute[candidate->begin].x_mm +
			((float)(lineRoute[candidate->end].x_mm - lineRoute[candidate->begin].x_mm) * progress);
		float chordY = (float)lineRoute[candidate->begin].y_mm +
			((float)(lineRoute[candidate->end].y_mm - lineRoute[candidate->begin].y_mm) * progress);
		float entryWeight = pathCorridorSmoothstep((float)(i - candidate->begin) /
			(float)PATH_CORRIDOR_TRANSITION_POINTS);
		float exitWeight = pathCorridorSmoothstep((float)(candidate->end - i) /
			(float)PATH_CORRIDOR_TRANSITION_POINTS);
		float weight = fminf(entryWeight, exitWeight);
		float moveX = (chordX - (float)lineRoute[i].x_mm) * weight;
		float moveY = (chordY - (float)lineRoute[i].y_mm) * weight;
		float move = sqrtf((moveX * moveX) + (moveY * moveY));
		if (move > PATH_CORRIDOR_MAX_OFFSET_MM)
		{
			moveX *= PATH_CORRIDOR_MAX_OFFSET_MM / move;
			moveY *= PATH_CORRIDOR_MAX_OFFSET_MM / move;
		}
		driveRoute[i].x_mm = pathFloatToInt16((float)lineRoute[i].x_mm + moveX);
		driveRoute[i].y_mm = pathFloatToInt16((float)lineRoute[i].y_mm + moveY);
		routeFlags[i] |= PATH_FLAG_CORRIDOR_APPLIED;
	}

	uint16_t reserveBegin = (candidate->begin > PATH_CORRIDOR_OVERLAP_GUARD_POINTS) ?
		(uint16_t)(candidate->begin - PATH_CORRIDOR_OVERLAP_GUARD_POINTS) : 0U;
	uint16_t reserveEnd = (candidate->end + PATH_CORRIDOR_OVERLAP_GUARD_POINTS < routeCount) ?
		(uint16_t)(candidate->end + PATH_CORRIDOR_OVERLAP_GUARD_POINTS) : (uint16_t)(routeCount - 1U);
	for (uint16_t i = reserveBegin; i <= reserveEnd; i++) routeFlags[i] |= PATH_FLAG_CORRIDOR_RESERVED;
}

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
// 処理概要     一次経路のスラローム区間を直線回廊へ置換する
// 引数         shortcutLevel:短縮レベル(1)
// 戻り値       true:短縮経路生成成功 false:再走行経路を使用
/////////////////////////////////////////////////////////////////////
bool routeGenerateShortcut(uint8_t shortcutLevel)
{
	memcpy(driveRoute, lineRoute, sizeof(RoutePoint) * routeCount);
	routeShortcutLevel = 0U;
	routeShortcutBuildStatus = PATH_SHORTCUT_BUILD_NOT_REQUESTED;
	routeShortcutCorridorCount = 0U;
	routeShortcutReductionMm = 0.0f;
	if (shortcutLevel == 0U) return false;
	if (routeCount < (PATH_CORRIDOR_MIN_SPAN_POINTS + (2U * PATH_CORRIDOR_END_GUARD_POINTS) + 1U))
	{
		routeShortcutBuildStatus = PATH_SHORTCUT_BUILD_NO_CORRIDOR;
		return false;
	}
#if !PATH_SHORTCUT_GEOMETRY_ENABLE
	(void)shortcutLevel;
	routeShortcutBuildStatus = PATH_SHORTCUT_BUILD_LEGAL_GEOMETRY_INVALID;
	return false;
#else
	if (shortcutLevel > PATH_LEVEL_MAX) shortcutLevel = PATH_LEVEL_MAX;
	float legalOffsetMm = PATH_LINE_HALF_WIDTH_MM + PATH_OCCUPIED_HALF_WIDTH_MM -
		PATH_TRACKING_ERROR_BUDGET_MM - PATH_LEGAL_RESERVE_MM;
	if (legalOffsetMm <= 0.0f || PATH_CORRIDOR_MAX_OFFSET_MM > legalOffsetMm ||
		PATH_OUTER_RADIUS_MM + legalOffsetMm > PATH_BOARD_CLEARANCE_MM)
	{
		routeShortcutBuildStatus = PATH_SHORTCUT_BUILD_LEGAL_GEOMETRY_INVALID;
		return false;
	}

	memset(routeFlags, 0, sizeof(routeFlags));
	pathComputeCorridorHeadings();
	for (uint8_t count = 0U; count < PATH_CORRIDOR_MAX_COUNT; count++)
	{
		PathCorridorCandidate candidate;
		if (!pathFindBestCorridorCandidate(&candidate)) break;
		pathApplyCorridorCandidate(&candidate);
		routeShortcutCorridorCount++;
	}
	if (routeShortcutCorridorCount == 0U)
	{
		memcpy(driveRoute, lineRoute, sizeof(RoutePoint) * routeCount);
		routeShortcutBuildStatus = PATH_SHORTCUT_BUILD_NO_CORRIDOR;
		return false;
	}

	for (uint16_t i = 0U; i < routeCount; i++)
	{
		float offset = pathPointDistance(lineRoute[i].x_mm, lineRoute[i].y_mm,
			driveRoute[i].x_mm, driveRoute[i].y_mm);
		if (offset > PATH_CORRIDOR_MAX_OFFSET_MM + PATH_CORRIDOR_OFFSET_TOLERANCE_MM)
		{
			memcpy(driveRoute, lineRoute, sizeof(RoutePoint) * routeCount);
			routeShortcutBuildStatus = PATH_SHORTCUT_BUILD_OFFSET_VIOLATION;
			return false;
		}
	}
	for (uint16_t i = 0U; i + 1U < routeCount; i++)
	{
		for (uint16_t j = (uint16_t)(i + 3U); j + 1U < routeCount; j++)
		{
			if (pathSegmentsIntersect(driveRoute, i, j) && !pathSegmentsIntersect(lineRoute, i, j))
			{
				memcpy(driveRoute, lineRoute, sizeof(RoutePoint) * routeCount);
				routeShortcutBuildStatus = PATH_SHORTCUT_BUILD_NEW_INTERSECTION;
				return false;
			}
		}
	}
	float sourceLength = pathLength(lineRoute);
	float shortcutLength = pathLength(driveRoute);
	routeShortcutReductionMm = sourceLength - shortcutLength;
	if (sourceLength <= 0.0f || routeShortcutReductionMm < PATH_CORRIDOR_MIN_SAVING_MM)
	{
		memcpy(driveRoute, lineRoute, sizeof(RoutePoint) * routeCount);
		routeShortcutBuildStatus = PATH_SHORTCUT_BUILD_INSUFFICIENT_REDUCTION;
		return false;
	}
	pathComputeHeadings(driveRoute, routeCount);
	pathBuildSpeedProfile(driveRoute, routeCount, shortcutLevel);
	routeShortcutLevel = shortcutLevel;
	routeShortcutBuildStatus = PATH_SHORTCUT_BUILD_SUCCESS;
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
	analysisSetSourceLog(0);
	analysisSetSlipSourceLog(0);
	FIL file;
	FRESULT result;
	RouteCsvColumns columns;
	ShortcutSettings generationSettings = shortcutSettings;
	uint8_t requestedShortcutLevel = shortcutLevel;
	char fileName[16];
	float lastX = 0.0f, lastY = 0.0f;
	float previousX = 0.0f, previousY = 0.0f, totalLength = 0.0f;
	bool havePoint = false;
	bool parseError = false;
	uint32_t parsedRows = 0U;
	float pulse = 0.0f;
	bool lockAcquired = sd_fatfs_lock(500U);
	if (!lockAcquired) return -9;

	snprintf(fileName, sizeof(fileName), "%d.csv", logNumber);
	result = f_open(&file, fileName, FA_OPEN_EXISTING | FA_READ);
	if (result != FR_OK)
	{
		sd_fatfs_unlock();
		return -5;
	}
	if (!pathReadColumnHeader(&file, &columns))
	{
		f_close(&file);
		sd_fatfs_unlock();
		return -10;
	}
	while (f_gets(routeCsvLine, sizeof(routeCsvLine), &file) != NULL)
	{
		float x, y;
		if (!RunGuard_CompleteCsvRow(routeCsvLine) ||
			!pathReadCsvPoint(routeCsvLine, &columns, &x, &y, &pulse))
		{
			parseError = true;
			break;
		}
		parsedRows++;
		if (!havePoint)
		{
			havePoint = true;
		}
		totalLength += pathPointDistance(previousX, previousY, x, y);
		previousX = x;
		previousY = y;
		lastX = x;
		lastY = y;
	}
	if (parseError || !havePoint || totalLength < PATH_ROUTE_SPACING_MM ||
		!RunGuard_CsvRowsMatch(columns.expectedRows, parsedRows))
	{
		f_close(&file);
		sd_fatfs_unlock();
		return -11;
	}
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
	if (!pathReadColumnHeader(&file, &columns))
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
	routeFlags[0] = 0U;
	float accumulated = 0.0f;
	float previousFusedX = 0.0f;
	float previousFusedY = 0.0f;
	bool routeOverflow = false;
	float previousPulse = 0.0f;
	uint32_t secondPassRows = 0U;
	while (f_gets(routeCsvLine, sizeof(routeCsvLine), &file) != NULL)
	{
		float rawX, rawY;
		if (!RunGuard_CompleteCsvRow(routeCsvLine) ||
			!pathReadCsvPoint(routeCsvLine, &columns, &rawX, &rawY, &pulse))
		{
			routeOverflow = true;
			break;
		}
		secondPassRows++;
		if (pulse < previousPulse)
		{
			routeOverflow = true;
			break;
		}
		if (!pathAppendRouteSample(rawX, rawY,
			&previousFusedX, &previousFusedY, &accumulated))
		{
			routeOverflow = true;
			break;
		}
		previousPulse = pulse;
	}
	f_close(&file);
	sd_fatfs_unlock();
	if (routeOverflow || secondPassRows != parsedRows) return -7;
	if (pathPointDistance((float)lineRoute[routeCount - 1U].x_mm,
		(float)lineRoute[routeCount - 1U].y_mm, lastX, lastY) >= 0.5f)
	{
		if (routeCount >= PATH_ROUTE_MAX_POINTS - PATH_GOAL_RESERVED_POINTS) return -7;
		lineRoute[routeCount].x_mm = pathFloatToInt16(lastX);
		lineRoute[routeCount].y_mm = pathFloatToInt16(lastY);
		routeCount++;
	}
	if (routeCount < 2U) return -11;
	pathComputeHeadings(lineRoute, routeCount);
	pathBuildSpeedProfile(lineRoute, routeCount, 0U);
	memcpy(driveRoute, lineRoute, sizeof(RoutePoint) * routeCount);
	bool shortcutOk = false;
	if (requestedShortcutLevel > 0U && generationSettings.maxLevel == 0U)
	{
		routeShortcutLevel = 0U;
		routeShortcutBuildStatus = PATH_SHORTCUT_BUILD_DISABLED_BY_SETTING;
		routeShortcutCorridorCount = 0U;
		routeShortcutReductionMm = 0.0f;
	}
	else
	{
		if (shortcutLevel > generationSettings.maxLevel) shortcutLevel = generationSettings.maxLevel;
		shortcutOk = routeGenerateShortcut(shortcutLevel);
	}
	if (!shortcutOk)
	{
		pathBuildSpeedProfile(driveRoute, routeCount, 0U);
		optimalTrace = BOOST_PATH_REPLAY;
	}
	else
	{
		optimalTrace = BOOST_SHORTCUT;
	}
	if (!pathExtendDriveRouteTowardOrigin(routeShortcutLevel)) return -15;
	pathBuildDriveRouteArcLength();
	routeSourceLog = (int16_t)logNumber;
	routeShortcutRequestedLevel = requestedShortcutLevel;
	routeGenerationSettings = generationSettings;
	routeGeometryCrc32 = pathComputeRouteGeometryCrc32();
	analysisSetSourceLog((int16_t)logNumber);
	analysisSetSlipSourceLog(0);
	indexSC = (int16_t)routeCount;
	optimalIndex = 0U;
	saveLogNumber((int16_t)logNumber);
	analyzedNumber = (int16_t)logNumber;
	return (int16_t)routeCount;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathSenseLine
// 処理概要     単一ラインを検出し機体座標系の受光位置を推定する
// 引数         predictedErrorMm: 予測横偏差[mm], observation: 受光位置の格納先[mm]
// 戻り値       true: 有効な単一ライン false: 無効
/////////////////////////////////////////////////////////////////////
static bool pathSenseLine(float predictedErrorMm, PathLineObservation *observation)
{
	if (!isLineSensorCalibrationValid() || stateCrossLine || fabsf(predictedErrorMm) > PATH_SENSOR_FOV_MM) return false;
	uint32_t sum = 0U;
	float weightedLateral = 0.0f;
	float weightedForward = 0.0f;
	uint8_t first = NUM_SENSORS;
	uint8_t last = 0U;
	uint8_t active = 0U;
	uint8_t clusters = 0U;
	bool previousActive = false;
	for (uint8_t i = 0U; i < NUM_SENSORS; i++)
	{
		uint16_t value = lSensorCari[i];
		bool sensorActive = value >= PATH_SENSOR_ACTIVE_TH;
		if (sensorActive)
		{
			if (!previousActive) clusters++;
			if (first == NUM_SENSORS) first = i;
			last = i;
			active++;
		}
		previousActive = sensorActive;
		sum += value;
		weightedLateral += (float)value * pathSensorLateralMm[i];
		weightedForward += (float)value * pathSensorForwardMm[i];
	}
	if (sum < PATH_SENSOR_MIN_SUM || active == 0U || clusters != 1U ||
		(uint8_t)(last - first + 1U) > PATH_SENSOR_MAX_CLUSTER_WIDTH) return false;
	observation->lateral_mm = weightedLateral / (float)sum;
	observation->forward_mm = weightedForward / (float)sum;
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathFindLineMatch
// 処理概要     検出ライン点に最も近い一次経路上の連続点を探索する
// 引数         sensorX_mm,sensorY_mm: 検出点世界座標[mm], nearest: 機体最近傍index,
//              maximumArcMm: 対応を許可する最大進捗[mm], match: 対応点の格納先
// 戻り値       true: 対応点あり false: 対応点なし
/////////////////////////////////////////////////////////////////////
static bool pathFindLineMatch(float sensorX_mm, float sensorY_mm, uint16_t nearest,
	float maximumArcMm, PathLineMatch *match)
{
	if (routeCount < 2U) return false;
	uint16_t begin = (nearest > PATH_LINE_MATCH_BACK_POINTS) ?
		(uint16_t)(nearest - PATH_LINE_MATCH_BACK_POINTS) : 0U;
	uint16_t lastPoint = (nearest + PATH_LINE_MATCH_FORWARD_POINTS < routeCount) ?
		(uint16_t)(nearest + PATH_LINE_MATCH_FORWARD_POINTS) : (uint16_t)(routeCount - 1U);
	bool found = false;
	float bestDistanceSquared = 0.0f;
	for (uint16_t i = begin; i < lastPoint; i++)
	{
		if ((float)driveRouteArcMm[i] > maximumArcMm) continue;
		float segmentHeadingError = pathWrapDeg(
			((float)lineRoute[i].heading_cdeg * 0.01f) - pathPose.heading_deg);
		if (fabsf(segmentHeadingError) > PATH_ASSOCIATION_HEADING_MAX_DEG) continue;
		float x0 = (float)lineRoute[i].x_mm;
		float y0 = (float)lineRoute[i].y_mm;
		float segmentX = (float)lineRoute[i + 1U].x_mm - x0;
		float segmentY = (float)lineRoute[i + 1U].y_mm - y0;
		float segmentLengthSquared = (segmentX * segmentX) + (segmentY * segmentY);
		if (segmentLengthSquared <= 1.0f) continue;
		float projection = (((sensorX_mm - x0) * segmentX) + ((sensorY_mm - y0) * segmentY)) /
			segmentLengthSquared;
		if (projection < 0.0f) projection = 0.0f;
		if (projection > 1.0f) projection = 1.0f;
		float candidateX = x0 + (segmentX * projection);
		float candidateY = y0 + (segmentY * projection);
		float residualX = sensorX_mm - candidateX;
		float residualY = sensorY_mm - candidateY;
		float distanceSquared = (residualX * residualX) + (residualY * residualY);
		if (!found || distanceSquared < bestDistanceSquared)
		{
			found = true;
			bestDistanceSquared = distanceSquared;
			match->x_mm = candidateX;
			match->y_mm = candidateY;
		}
	}
	if (!found) return false;
	match->residual_mm = sqrtf(bestDistanceSquared);
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
	pathTravelMm = 0.0f;
	targetSpeedMps = (routeCount > 0U) ? (float)driveRoute[0].speed_cms * 0.01f : 0.0f;
	followerState = (routeCount > 1U) ? PATH_STATE_TRACKING : PATH_STATE_INACTIVE;
	pathLogLinePointX_mm = 0.0f;
	pathLogLinePointY_mm = 0.0f;
	pathLogLineValid = 0U;
	pathLogErrorY_mm = 0.0f;
	pathLogErrorHeading_cdeg = 0;
	pathLogState = (uint8_t)followerState;
	pathLogLegalMargin_mm = PATH_LINE_HALF_WIDTH_MM + PATH_OCCUPIED_HALF_WIDTH_MM;
	pathLogLineMatchResidual_mm = 0.0f;
	pathLogPoseCorrection_um = 0U;
	pathLogPoseCorrectionHeading_cdeg = 0;
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
	if (distanceMm > 0.0f) pathTravelMm += distanceMm;
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
	const ShortcutSettings *activeSettings = runStartSettingsValid ? &runStartSettings : &shortcutSettings;
	uint16_t begin = (routeIndex > 2U) ? (uint16_t)(routeIndex - 2U) : 0U;
	uint16_t end = (routeIndex + 24U < routeCount) ? (uint16_t)(routeIndex + 24U) : (uint16_t)(routeCount - 1U);
	uint16_t nearest = routeIndex;
	float nearestDistance = pathPointDistance(pathPose.x_mm, pathPose.y_mm,
		driveRoute[routeIndex].x_mm, driveRoute[routeIndex].y_mm);
	float maximumArcMm = pathTravelMm + PATH_ASSOCIATION_PROGRESS_MARGIN_MM;
	bool associationValid = false;
	for (uint16_t i = begin; i <= end; i++)
	{
		if ((float)driveRouteArcMm[i] > maximumArcMm) continue;
		float candidateHeadingError = pathWrapDeg(
			((float)driveRoute[i].heading_cdeg * 0.01f) - pathPose.heading_deg);
		if (fabsf(candidateHeadingError) > PATH_ASSOCIATION_HEADING_MAX_DEG) continue;
		float distance = pathPointDistance(pathPose.x_mm, pathPose.y_mm, driveRoute[i].x_mm, driveRoute[i].y_mm);
		if (!associationValid || distance < nearestDistance)
		{
			nearestDistance = distance;
			nearest = i;
			associationValid = true;
		}
	}
	if (!associationValid)
	{
		nearest = routeIndex;
		nearestDistance = pathPointDistance(pathPose.x_mm, pathPose.y_mm,
			driveRoute[nearest].x_mm, driveRoute[nearest].y_mm);
	}
	routeIndex = nearest;
	optimalIndex = routeIndex;

	float lineHeadingRad = (float)lineRoute[nearest].heading_cdeg * 0.01f * DEG2RAD;
	float lineDx = pathPose.x_mm - (float)lineRoute[nearest].x_mm;
	float lineDy = pathPose.y_mm - (float)lineRoute[nearest].y_mm;
	float lineError = (lineDx * cosf(lineHeadingRad)) - (lineDy * sinf(lineHeadingRad));
	PathLineObservation observation = {0.0f, 0.0f};
	currentLineValid = pathSenseLine(lineError, &observation);
	bool corridorLineCorrectionBlocked = routeShortcutLevel > 0U &&
		((routeFlags[nearest] & PATH_FLAG_CORRIDOR_APPLIED) != 0U);
	PathLineMatch lineMatch = {(float)lineRoute[nearest].x_mm,
		(float)lineRoute[nearest].y_mm, 0.0f};
	bool lineMatchValid = false;
	pathLogPoseCorrection_um = 0U;
	pathLogPoseCorrectionHeading_cdeg = 0;
	if (currentLineValid && associationValid)
	{
		float poseHeadingRad = pathPose.heading_deg * DEG2RAD;
		float headingSin = sinf(poseHeadingRad);
		float headingCos = cosf(poseHeadingRad);
		float sensorX = pathPose.x_mm + (observation.lateral_mm * headingCos) +
			(observation.forward_mm * headingSin);
		float sensorY = pathPose.y_mm - (observation.lateral_mm * headingSin) +
			(observation.forward_mm * headingCos);
		lineMatchValid = pathFindLineMatch(sensorX, sensorY, nearest, maximumArcMm, &lineMatch);
		if (lineMatchValid && lineMatch.residual_mm <= PATH_LINE_MATCH_RESIDUAL_MAX_MM &&
			!corridorLineCorrectionBlocked)
		{
			float residualX = sensorX - lineMatch.x_mm;
			float residualY = sensorY - lineMatch.y_mm;
			float alpha = (float)activeSettings->lineAlpha_x1000 * 0.001f;
			float correctionX = -alpha * residualX;
			float correctionY = -alpha * residualY;
			float correctionDistance = sqrtf((correctionX * correctionX) +
				(correctionY * correctionY));
			float sensorDerivativeX = (-observation.lateral_mm * headingSin) +
				(observation.forward_mm * headingCos);
			float sensorDerivativeY = (-observation.lateral_mm * headingCos) -
				(observation.forward_mm * headingSin);
			float thetaGradient = (residualX * sensorDerivativeX) +
				(residualY * sensorDerivativeY);
			float thetaGain = (float)activeSettings->lineThetaGain_x1e9 * 1.0e-9f;
			float headingCorrectionDeg = (-thetaGain * thetaGradient) * RAD2DEG;
			if (correctionDistance <= PATH_LINE_POSITION_CORRECTION_MAX_MM &&
				fabsf(headingCorrectionDeg) <= PATH_LINE_HEADING_CORRECTION_MAX_DEG)
			{
				pathPose.x_mm += correctionX;
				pathPose.y_mm += correctionY;
				pathPose.heading_deg = pathWrapDeg(pathPose.heading_deg + headingCorrectionDeg);
				pathLogPoseCorrection_um = (uint16_t)lroundf(correctionDistance * 1000.0f);
				pathLogPoseCorrectionHeading_cdeg = pathFloatToInt16(headingCorrectionDeg * 100.0f);
			}
		}
	}
	pathLogLineValid = currentLineValid ? 1U : 0U;
	pathLogLinePointX_mm = lineMatch.x_mm;
	pathLogLinePointY_mm = lineMatch.y_mm;
	pathLogLineMatchResidual_mm = lineMatchValid ? lineMatch.residual_mm : 0.0f;
	nearestDistance = pathPointDistance(pathPose.x_mm, pathPose.y_mm,
		driveRoute[nearest].x_mm, driveRoute[nearest].y_mm);

	float lookaheadMm = (float)activeSettings->lookaheadBaseMm +
		((float)activeSettings->lookaheadPerMpsMm * targetSpeedMps);
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
	float kLateral = (float)activeSettings->kLateral_x100 * 0.01f;
	float kHeading = (float)activeSettings->kHeading_x100 * 0.01f;
	float targetYawRate = feedForwardDegPerSec - (kLateral * lateralError) + (kHeading * nearestHeadingError);
	if (targetYawRate > 1800.0f) targetYawRate = 1800.0f;
	if (targetYawRate < -1800.0f) targetYawRate = -1800.0f;
	setTargetAngularVelocity(targetYawRate);
	log_targetAngularVelocity = (int32_t)targetYawRate;

	/*
	 * 先読み方位は曲率FFだけに使い、方位FB、ロスト判定、再合流判定には
	 * 最近傍経路点の接線方位差を使う。急カーブ手前で先読み方位差を
	 * 方位FBへ加えると、経路の接線が曲がる前から旋回して内側へ外れる。
	 */
	if (!associationValid || nearestDistance > PATH_LOST_DISTANCE_MM ||
		fabsf(nearestHeadingError) > PATH_LOST_HEADING_DEG)
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
			if (nearestDistance < PATH_REJOIN_DISTANCE_MM && fabsf(nearestHeadingError) < PATH_REJOIN_HEADING_DEG)
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
	pathLogErrorHeading_cdeg = pathFloatToInt16(nearestHeadingError * 100.0f);
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
// モジュール名 pathFollowerGoalReached
// 処理概要     一次走行終端と原点の中間点への到達を判定する
// 引数         なし
// 戻り値       true:停止開始位置へ到達 false:走行途中
/////////////////////////////////////////////////////////////////////
bool pathFollowerGoalReached(void)
{
	if (!pathGoalValid || routeIndex >= routeCount) return false;
	return driveRouteArcMm[routeIndex] >= pathGoalArcMm;
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
// モジュール名 pathRouteShortcutBuildStatus
// 処理概要     Level 1経路生成結果を取得する
// 引数         なし
// 戻り値       PathShortcutBuildStatus
/////////////////////////////////////////////////////////////////////
uint8_t pathRouteShortcutBuildStatus(void) { return routeShortcutBuildStatus; }

/////////////////////////////////////////////////////////////////////
// モジュール名 pathRouteShortcutCorridorCount
// 処理概要     生成した直線回廊数を取得する
// 引数         なし
// 戻り値       直線回廊数
/////////////////////////////////////////////////////////////////////
uint8_t pathRouteShortcutCorridorCount(void) { return routeShortcutCorridorCount; }

/////////////////////////////////////////////////////////////////////
// モジュール名 pathRouteShortcutReductionMm
// 処理概要     Level 1経路の全長短縮量を取得する
// 引数         なし
// 戻り値       短縮量[mm]
/////////////////////////////////////////////////////////////////////
float pathRouteShortcutReductionMm(void) { return routeShortcutReductionMm; }

/////////////////////////////////////////////////////////////////////
// モジュール名 pathFollowerCaptureRunStartSettings
// 処理概要     走行開始時のショートカット設定と経路生成情報を固定する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void pathFollowerCaptureRunStartSettings(void)
{
	runStartSettings = shortcutSettings;
	runStartSettingsValid = true;
	runRouteSourceLog = 0;
	runRouteRequestedLevel = 0U;
	runRouteShortcutLevel = 0U;
	runRouteShortcutBuildStatus = PATH_SHORTCUT_BUILD_NOT_REQUESTED;
	runRouteShortcutCorridorCount = 0U;
	runRouteShortcutReductionMm = 0.0f;
	runRouteCount = 0U;
	runRouteGeometryCrc32 = 0U;
	runGenerationSettings = (ShortcutSettings){0U, 0U, 0U, 0U, 0U, 0U, 0U};
	if (optimalTrace == BOOST_PATH_REPLAY || optimalTrace == BOOST_SHORTCUT)
	{
		runRouteSourceLog = routeSourceLog;
		runRouteRequestedLevel = routeShortcutRequestedLevel;
		runRouteShortcutLevel = routeShortcutLevel;
		runRouteShortcutBuildStatus = routeShortcutBuildStatus;
		runRouteShortcutCorridorCount = routeShortcutCorridorCount;
		runRouteShortcutReductionMm = routeShortcutReductionMm;
		runRouteCount = routeCount;
		runRouteGeometryCrc32 = routeGeometryCrc32;
		runGenerationSettings = routeGenerationSettings;
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathRouteGenerationSettings
// 処理概要     経路生成成功時に保存したショートカット設定を返す
// 引数         なし
// 戻り値       経路生成時設定
/////////////////////////////////////////////////////////////////////
ShortcutSettings pathRouteGenerationSettings(void) { return routeGenerationSettings; }

/////////////////////////////////////////////////////////////////////
// モジュール名 pathRunStartSettings
// 処理概要     走行開始時に固定したショートカット設定を返す
// 引数         なし
// 戻り値       走行開始時設定
/////////////////////////////////////////////////////////////////////
ShortcutSettings pathRunStartSettings(void) { return runStartSettings; }

/////////////////////////////////////////////////////////////////////
// モジュール名 pathRunGenerationSettings
// 処理概要     走行開始時に固定した経路生成時設定を返す
// 引数         なし
// 戻り値       経路生成時設定
/////////////////////////////////////////////////////////////////////
ShortcutSettings pathRunGenerationSettings(void) { return runGenerationSettings; }

/////////////////////////////////////////////////////////////////////
// モジュール名 pathRunRouteSourceLog
// 処理概要     走行開始時に固定した経路元ログ番号を返す
// 引数         なし
// 戻り値       ログ番号。不明または非PATH走行は0
/////////////////////////////////////////////////////////////////////
int16_t pathRunRouteSourceLog(void) { return runRouteSourceLog; }

/////////////////////////////////////////////////////////////////////
// モジュール名 pathRunRouteRequestedLevel
// 処理概要     走行開始時に固定した要求ショートカットレベルを返す
// 引数         なし
// 戻り値       要求レベル
/////////////////////////////////////////////////////////////////////
uint8_t pathRunRouteRequestedLevel(void) { return runRouteRequestedLevel; }

/////////////////////////////////////////////////////////////////////
// モジュール名 pathRunRouteShortcutLevel
// 処理概要     走行開始時に固定した採用ショートカットレベルを返す
// 引数         なし
// 戻り値       採用レベル
/////////////////////////////////////////////////////////////////////
uint8_t pathRunRouteShortcutLevel(void) { return runRouteShortcutLevel; }

/////////////////////////////////////////////////////////////////////
// モジュール名 pathRunRouteShortcutBuildStatus
// 処理概要     走行開始時に固定した経路生成結果を返す
// 引数         なし
// 戻り値       生成結果
/////////////////////////////////////////////////////////////////////
uint8_t pathRunRouteShortcutBuildStatus(void) { return runRouteShortcutBuildStatus; }

/////////////////////////////////////////////////////////////////////
// モジュール名 pathRunRouteShortcutCorridorCount
// 処理概要     走行開始時に固定した直線回廊数を返す
// 引数         なし
// 戻り値       回廊数
/////////////////////////////////////////////////////////////////////
uint8_t pathRunRouteShortcutCorridorCount(void) { return runRouteShortcutCorridorCount; }

/////////////////////////////////////////////////////////////////////
// モジュール名 pathRunRouteShortcutReductionMm
// 処理概要     走行開始時に固定した経路短縮量を返す
// 引数         なし
// 戻り値       短縮量[mm]
/////////////////////////////////////////////////////////////////////
float pathRunRouteShortcutReductionMm(void) { return runRouteShortcutReductionMm; }

/////////////////////////////////////////////////////////////////////
// モジュール名 pathRunRouteCount
// 処理概要     走行開始時に固定した経路点数を返す
// 引数         なし
// 戻り値       経路点数
/////////////////////////////////////////////////////////////////////
uint16_t pathRunRouteCount(void) { return runRouteCount; }

/////////////////////////////////////////////////////////////////////
// モジュール名 pathRunRouteGeometryCrc32
// 処理概要     走行開始時に固定した経路形状CRC32を返す
// 引数         なし
// 戻り値       経路形状CRC32。不明または非PATH走行は0
/////////////////////////////////////////////////////////////////////
uint32_t pathRunRouteGeometryCrc32(void) { return runRouteGeometryCrc32; }

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
		f_printf(&file, "%u,%03u,%03u,%04u,%04u,%03u,%04u",
			shortcutSettings.maxLevel, shortcutSettings.lookaheadBaseMm,
			shortcutSettings.lookaheadPerMpsMm, shortcutSettings.kLateral_x100,
			shortcutSettings.kHeading_x100, shortcutSettings.lineAlpha_x1000,
			shortcutSettings.lineThetaGain_x1e9);
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
	int values[7] = {PATH_DEFAULT_MAX_LEVEL, PATH_DEFAULT_LOOKAHEAD_BASE_MM,
		PATH_DEFAULT_LOOKAHEAD_PER_MPS_MM, PATH_DEFAULT_KLATERAL_X100,
		PATH_DEFAULT_KHEADING_X100, PATH_DEFAULT_LINE_ALPHA_X1000,
		PATH_DEFAULT_LINE_THETA_GAIN_X1E9};
	ShortcutSettings defaults = {PATH_DEFAULT_MAX_LEVEL, PATH_DEFAULT_LOOKAHEAD_BASE_MM,
		PATH_DEFAULT_LOOKAHEAD_PER_MPS_MM, PATH_DEFAULT_KLATERAL_X100,
		PATH_DEFAULT_KHEADING_X100, PATH_DEFAULT_LINE_ALPHA_X1000,
		PATH_DEFAULT_LINE_THETA_GAIN_X1E9};
	shortcutSettings = defaults;
	bool repair = false;
	int parsed = 0;
	if (f_open(&file, PATH_SETTING_FILE, FA_OPEN_EXISTING | FA_READ) == FR_OK)
	{
		if (f_gets(text, sizeof(text), &file) != NULL)
		{
			parsed = sscanf(text, "%d,%d,%d,%d,%d,%d,%d", &values[0], &values[1], &values[2],
				&values[3], &values[4], &values[5], &values[6]);
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
	if (parsed >= 7 && pathSettingInRange(values[6], 0, PATH_LINE_THETA_GAIN_MAX_X1E9)) shortcutSettings.lineThetaGain_x1e9 = (uint16_t)values[6]; else repair = true;
	if (repair) writeShortcutSettings();
}
