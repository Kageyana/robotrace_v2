#ifndef RUN_MEMORY_H_
#define RUN_MEMORY_H_

//====================================//
// インクルード
//====================================//
#include <stdbool.h>
#include <stdint.h>

#define OPT_BUFF_SIZE 1000
#define PATH_ROUTE_MAX_POINTS 1514U
#define PATH_MARKER_CAPACITY 128U

typedef struct
{
	int16_t ROC;
	float boostSpeed;
} AnalysisData;

typedef struct
{
	int32_t distance;
	int32_t indexPPAD;
} EventPos;

typedef struct
{
	int16_t x_mm;
	int16_t y_mm;
	int16_t heading_cdeg;
	uint16_t speed_cms;
} RoutePoint;

_Static_assert(sizeof(RoutePoint) == 8U, "RoutePoint must remain 8 bytes");

typedef struct
{
    uint16_t routeIndex;
    uint16_t arcMm;
    int16_t x_mm;
    int16_t y_mm;
    uint8_t type;
    uint8_t reserved;
} PathMarkerAnchor;

typedef struct
{
    int16_t x_mm;
    int16_t y_mm;
    uint16_t valid;
} PathWhitePoint;

typedef struct
{
	float v2Max[OPT_BUFF_SIZE];
	float rocAbsSum[OPT_BUFF_SIZE];
	uint16_t rocCnt[OPT_BUFF_SIZE];
	uint16_t longOnCount[OPT_BUFF_SIZE];
	uint16_t latOnCount[OPT_BUFF_SIZE];
	float risk[OPT_BUFF_SIZE];
	float riskExpanded[OPT_BUFF_SIZE];
	float v3[OPT_BUFF_SIZE];
} SlipAnalysisMemory;

// DISTANCE計画とSLIP解析は同時使用する。PATH経路とは停止中に切り替える。
typedef union
{
	struct
	{
		AnalysisData ppad[OPT_BUFF_SIZE];
		EventPos markers[OPT_BUFF_SIZE];
		SlipAnalysisMemory slip;
	} distance;
	struct
	{
		RoutePoint line[PATH_ROUTE_MAX_POINTS];
		RoutePoint drive[PATH_ROUTE_MAX_POINTS];
		uint16_t arcMm[PATH_ROUTE_MAX_POINTS];
		uint8_t flags[PATH_ROUTE_MAX_POINTS];
		PathMarkerAnchor markers[PATH_MARKER_CAPACITY];
		PathWhitePoint white[PATH_ROUTE_MAX_POINTS];
	} path;
} RunMemory;

_Static_assert(sizeof(((RunMemory *)0)->path) <= sizeof(((RunMemory *)0)->distance),
    "PATH observations must fit in the existing shared RAM");

typedef enum
{
	RUN_MEMORY_NONE = 0,
	RUN_MEMORY_DISTANCE,
	RUN_MEMORY_PATH
} RunMemoryOwner;

//====================================//
// グローバル変数の宣言、定義
//====================================//
extern RunMemory runMemory;
// 解析元は型付きbin読込とし、CSV行バッファは持たない。
extern RunMemoryOwner runMemoryOwner;

//====================================//
// プロトタイプ宣言
//====================================//
bool runMemoryPrepare(RunMemoryOwner owner);

#endif
