#ifndef PATH_FOLLOWER_H_
#define PATH_FOLLOWER_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

// 60mコースを40mm間隔で保持する。1501点で開始点を含む60mまでを表現できる。
#define PATH_ROUTE_SPACING_MM              40.0f
#define PATH_ROUTE_MAX_POINTS              1501U
#define PATH_ROUTE_CONTROLLER_VERSION      2U

// 実測した機体投影寸法と合法余裕を確認済みのため、形状変更を許可する。
#define PATH_SHORTCUT_GEOMETRY_ENABLE      1

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
	uint8_t maxLevel;
	uint16_t lookaheadBaseMm;
	uint16_t lookaheadPerMpsMm;
	uint16_t kLateral_x100;
	uint16_t kHeading_x100;
	uint16_t lineAlpha_x1000;
} ShortcutSettings;

typedef enum
{
	PATH_STATE_INACTIVE = 0,
	PATH_STATE_TRACKING = 1,
	PATH_STATE_LINE_FALLBACK = 2,
	PATH_STATE_REJOIN = 3,
	PATH_STATE_LOCALIZATION_LOST = 4
} PathFollowerState;

extern ShortcutSettings shortcutSettings;
extern float pathLogLinePointX_mm;
extern float pathLogLinePointY_mm;
extern uint8_t pathLogLineValid;
extern float pathLogErrorY_mm;
extern int16_t pathLogErrorHeading_cdeg;
extern uint8_t pathLogState;
extern float pathLogLegalMargin_mm;

int16_t routeBuildFromLog(int logNumber, uint8_t shortcutLevel);
bool routeGenerateShortcut(uint8_t shortcutLevel);
uint16_t pathRouteCount(void);
bool pathFollowerGoalWindowOpen(void);
int16_t pathRouteSourceLog(void);
uint8_t pathRouteShortcutLevel(void);
void pathFollowerReset(void);
void pathFollowerUpdatePose1ms(int32_t encoderPulse, float gyroDegPerSec);
void pathFollowerUpdateTarget5ms(void);
PathFollowerState pathFollowerGetStatus(void);
uint16_t pathFollowerGetPathBlendPermille(void);
float pathFollowerGetTargetSpeedMps(void);
bool pathFollowerLineIsValid(void);
void readShortcutSettings(void);
void writeShortcutSettings(void);

#endif // PATH_FOLLOWER_H_
