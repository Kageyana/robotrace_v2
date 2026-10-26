#ifndef PATH_FOLLOWER_H_
#define PATH_FOLLOWER_H_

//====================================//
// インクルード
//====================================//
#include "runMemory.h"
#include "main.h"
#include <stdbool.h>
#include <stdint.h>

//====================================//
// 調整パラメータ
//====================================//
// 経路の標本化と形状変更。
#define PATH_ROUTE_SPACING_MM              20.0f // 経路点を再標本化する距離間隔[mm]。
#define PATH_CURVE_MAX_SPEED_MPS            10.0f // PATH_CURVEの機体速度上限[m/s]。曲率別速度はDISTANCEの有効なSD設定を共用。
// 実測した機体投影寸法と合法余裕を確認済みのため、形状変更を許可する。
#define PATH_SHORTCUT_GEOMETRY_ENABLE      1 // ショートカット形状変更の許可[0:無効、1:有効]。

// SD設定の既定値。有効な ./setting/shortcut.txt の値を優先する。
#define PATH_DEFAULT_MAX_LEVEL               1U // ショートカットレベルの既定値[段階]。0で形状変更なし、1でLevel 1。
#define PATH_DEFAULT_LOOKAHEAD_BASE_MM        80U // 速度0での先読み距離の既定値[mm]。
#define PATH_DEFAULT_LOOKAHEAD_PER_MPS_MM     40U // 速度1m/sあたりの先読み距離加算量の既定値[mm/(m/s)]。
#define PATH_DEFAULT_KLATERAL_X100            3000U // 横偏差ゲインの既定値[(deg/s)/mm ×100]。
#define PATH_DEFAULT_KHEADING_X100            600U // 方位偏差ゲインの既定値[(deg/s)/deg ×100]。
#define PATH_DEFAULT_LINE_ALPHA_X1000         10U // 5ms基準の位置補正率の既定値[無次元 ×1000]。1ms更新では1/5を適用。
#define PATH_DEFAULT_LINE_THETA_GAIN_X1E9      0U // 5ms基準の方位補正ゲインの既定値[1/mm^2 ×10^9]。1ms更新では1/5、0で補正なし。

// SD設定の許容範囲。範囲外の項目は既定値で修復する。
#define PATH_LEVEL_MAX                        1U // 許可するショートカットレベルの上限[段階]。初期実機検証はLevel 1まで。
#define PATH_LOOKAHEAD_BASE_MIN_MM            40U // 基本先読み距離の下限[mm]。
#define PATH_LOOKAHEAD_BASE_MAX_MM            300U // 基本先読み距離の上限[mm]。
#define PATH_LOOKAHEAD_SPEED_MAX_MM            200U // 速度による先読み距離加算係数の上限[mm/(m/s)]。
#define PATH_KLATERAL_MAX_X100                10000U // 横偏差ゲインの上限[(deg/s)/mm ×100]。
#define PATH_KHEADING_MAX_X100                3000U // 方位偏差ゲインの上限[(deg/s)/deg ×100]。
#define PATH_LINE_ALPHA_MAX_X1000             100U // 位置補正率の上限[無次元 ×1000]。
#define PATH_LINE_THETA_GAIN_MAX_X1E9         1000U // 方位補正ゲインの上限[1/mm^2 ×10^9]。

// ショートカットの直線回廊候補。点数指定の距離はPATH_ROUTE_SPACING_MMに従う。
#define PATH_CORRIDOR_MIN_SPAN_POINTS         15U // 候補区間の最小長[点間隔数]。40mm間隔では600mm。
#define PATH_CORRIDOR_MAX_SPAN_POINTS         40U // 候補区間の最大長[点間隔数]。40mm間隔では1600mm。
#define PATH_CORRIDOR_TRANSITION_POINTS       3U // 回廊端で元経路へ滑らかに接続する長さ[点間隔数]。40mm間隔では120mm。
#define PATH_CORRIDOR_END_GUARD_POINTS        4U // 経路の両端に残す回廊生成の保護幅[点数]。
#define PATH_CORRIDOR_OVERLAP_GUARD_POINTS    5U // 回廊同士の重複を防ぐ前後の予約幅[点数]。
#define PATH_CORRIDOR_MAX_COUNT               16U // 1経路に適用する直線回廊の上限[個]。
#define PATH_CORRIDOR_HEADING_LIMIT_DEG       12.0f // 候補区間の端点方位と直線方位の許容差[deg]。
#define PATH_CORRIDOR_TURN_THRESHOLD_DEG      2.0f // 蛇行の旋回方向変化として数える最小方位変化[deg]。
#define PATH_CORRIDOR_MIN_SIGN_CHANGES        3U // 蛇行候補に必要な旋回方向の反転回数[回]。
#define PATH_CORRIDOR_MAX_OFFSET_MM           34.0f // 元ラインからの直線回廊の最大横変位[mm]。
#define PATH_CORRIDOR_OFFSET_TOLERANCE_MM     0.5f // 生成後の最大横変位検査に加える許容誤差[mm]。
#define PATH_CORRIDOR_MIN_PROJECTION          -0.02f // 候補点の直線区間への射影位置の下限[無次元、始点0・終点1]。
#define PATH_CORRIDOR_MAX_PROJECTION          1.02f // 候補点の直線区間への射影位置の上限[無次元、始点0・終点1]。
#define PATH_CORRIDOR_MIN_SAVING_MM           5.0f // 回廊候補と生成経路の採用に必要な最小短縮量[mm]。

// 機体寸法と合法余裕。寸法変更時は実測値・走行可能領域を再確認する。
// 機体座標原点は前後車軸中心の中点。上面から右が+X、前が+Y。
#define PATH_TRACKING_ERROR_BUDGET_MM         15.0f // 合法横変位の算出で確保する追従誤差の余裕[mm]。
#define PATH_LINE_HALF_WIDTH_MM               9.5f // コースライン実幅の半分[mm]。
#define PATH_OCCUPIED_HALF_WIDTH_MM           65.0f // 車軸中心から左右投影端までの短い側の距離[mm]。
#define PATH_OUTER_RADIUS_MM                  100.0f // 車軸中心から全投影外形の最遠点までの距離[mm]。
#define PATH_BOARD_CLEARANCE_MM               200.0f // コース中心線から走行可能領域端までの最小距離[mm]。
#define PATH_LEGAL_RESERVE_MM                 10.0f // 合法横変位の算出で追加確保する余裕[mm]。

// 経路逸脱・ライン追従への退避・経路復帰。判定周期は1ms。
#define PATH_LOST_DISTANCE_MM                 80.0f // ライン追従への退避を判定する経路からの距離上限[mm]。
#define PATH_REJOIN_DISTANCE_MM               35.0f // 経路復帰を許可する経路からの距離上限[mm]。
#define PATH_LOST_HEADING_DEG                 60.0f // ライン追従への退避を判定する方位偏差上限[deg]。
#define PATH_REJOIN_HEADING_DEG               30.0f // 経路復帰を許可する方位偏差上限[deg]。
#define PATH_LOST_TIME_MS                     100U // 経路逸脱から退避を判定する連続時間[ms]。
#define PATH_REJOIN_TIME_MS                   200U // 復帰条件の連続成立時間[ms]。
#define PATH_LINE_LOST_TIME_MS                100U // 退避中に自己位置喪失とするライン未検出の連続時間[ms]。
#define PATH_ASSOCIATION_PROGRESS_MARGIN_MM   120.0f // 走行積算距離より先の経路候補を許可する余裕[mm]。
#define PATH_ASSOCIATION_HEADING_MAX_DEG      60.0f // 経路候補と機体の方位差の許容上限[deg]。
#define PATH_GOAL_RESERVED_POINTS             13U // 終端から原点方向への延長用に確保する点数[点]。40mm間隔で最大520mmの延長分。
#define PATH_REJOIN_BLEND_STEP                10U // 復帰中に1msごとに増やす経路操舵の混合率[permille/更新]。100msで0→1000。

// ライン観測と自己位置補正。判定周期は1ms。SDの補正係数は従来の5ms基準を維持する。
#define PATH_SENSOR_ACTIVE_TH                 800U // 明部として数える各ラインセンサーの閾値[正規化AD値]。
#define PATH_SENSOR_MIN_SUM                   1200U // ライン観測を有効とする全センサー値の合計下限[正規化AD値の合計]。
#define PATH_SENSOR_MAX_CLUSTER_WIDTH         3U // 単一ラインとして許可する連続した明部の最大幅[センサー数]。
#define PATH_SENSOR_FOV_MM                    35.0f // ライン位置補正を許可する予測横偏差の絶対値上限[mm]。
#define PATH_LINE_MATCH_BACK_POINTS           2U // ライン観測と照合する最近傍点より後方の探索幅[点数]。
#define PATH_LINE_MATCH_FORWARD_POINTS        4U // ライン観測と照合する最近傍点より前方の探索幅[点数]。
#define PATH_LINE_MATCH_RESIDUAL_MAX_MM        30.0f // 自己位置補正を許可するライン観測残差の上限[mm]。
#define PATH_LINE_POSITION_CORRECTION_MAX_MM    0.2f // 1msあたりの位置補正上限[mm/更新]。従来の1mm/5ms相当。
#define PATH_LINE_HEADING_CORRECTION_MAX_DEG    0.02f // 1msあたりの方位補正上限[deg/更新]。従来の0.1deg/5ms相当。

//====================================//
// シンボル定義
//====================================//
#define PATH_ROUTE_CONTROLLER_VERSION      18U
#define PATH_SOURCE_FORMAT_VERSION          1U

typedef struct
{
	uint8_t maxLevel; // ショートカット上限[段階]。0:形状変更なし、1:Level 1。
	uint16_t lookaheadBaseMm; // 基本先読み距離[mm]。
	uint16_t lookaheadPerMpsMm; // 速度に応じた先読み距離加算係数[mm/(m/s)]。
	uint16_t kLateral_x100; // 横偏差ゲイン[(deg/s)/mm ×100]。
	uint16_t kHeading_x100; // 方位偏差ゲイン[(deg/s)/deg ×100]。
	uint16_t lineAlpha_x1000; // 5ms基準の位置補正率[無次元 ×1000]。1ms更新では1/5を適用。
	uint16_t lineThetaGain_x1e9; // 5ms基準の方位補正ゲイン[1/mm^2 ×10^9]。1ms更新では1/5を適用。
} ShortcutSettings;

typedef enum
{
	PATH_STATE_INACTIVE = 0,
	PATH_STATE_TRACKING = 1,
	PATH_STATE_LINE_FALLBACK = 2,
	PATH_STATE_REJOIN = 3,
	PATH_STATE_LOCALIZATION_LOST = 4
} PathFollowerState;

typedef enum
{
	PATH_SHORTCUT_BUILD_NOT_REQUESTED = 0,
	PATH_SHORTCUT_BUILD_SUCCESS = 1,
	PATH_SHORTCUT_BUILD_DISABLED_BY_SETTING = 2,
	PATH_SHORTCUT_BUILD_LEGAL_GEOMETRY_INVALID = 3,
	PATH_SHORTCUT_BUILD_NO_CORRIDOR = 4,
	PATH_SHORTCUT_BUILD_OFFSET_VIOLATION = 5,
	PATH_SHORTCUT_BUILD_NEW_INTERSECTION = 6,
	PATH_SHORTCUT_BUILD_INSUFFICIENT_REDUCTION = 7
} PathShortcutBuildStatus;

//====================================//
// グローバル変数の宣言、定義
//====================================//
extern ShortcutSettings shortcutSettings;
extern float pathLogLinePointX_mm;
extern float pathLogLinePointY_mm;
extern uint8_t pathLogLineValid;
extern float pathLogErrorY_mm;
extern int16_t pathLogErrorHeading_cdeg;
extern uint8_t pathLogState;
// 経路操舵の角速度指令成分[deg/s]。符号付き、合計の±1800deg/s制限前。
extern float pathLogYawFF_dps;
extern float pathLogYawLateralFB_dps;
extern float pathLogYawHeadingFB_dps;
extern float pathLogLegalMargin_mm;
extern float pathLogLineMatchResidual_mm;
extern uint16_t pathLogPoseCorrection_um;
extern int16_t pathLogPoseCorrectionHeading_cdeg;

//====================================//
// プロトタイプ宣言
//====================================//

int16_t routeBuildFromLog(int logNumber, uint8_t shortcutLevel);
int16_t routeBuildCurvatureFromLog(int logNumber);
bool routeGenerateShortcut(uint8_t shortcutLevel);
uint16_t pathRouteCount(void);
bool pathFollowerGoalReached(void);
int16_t pathRouteSourceLog(void);
uint8_t pathRouteShortcutLevel(void);
uint8_t pathRouteShortcutBuildStatus(void);
uint8_t pathRouteShortcutCorridorCount(void);
float pathRouteShortcutReductionMm(void);
void pathFollowerCaptureRunStartSettings(void);
ShortcutSettings pathRouteGenerationSettings(void);
ShortcutSettings pathRunStartSettings(void);
ShortcutSettings pathRunGenerationSettings(void);
int16_t pathRunRouteSourceLog(void);
uint8_t pathRunRouteRequestedLevel(void);
uint8_t pathRunRouteShortcutLevel(void);
uint8_t pathRunRouteShortcutBuildStatus(void);
uint8_t pathRunRouteShortcutCorridorCount(void);
float pathRunRouteShortcutReductionMm(void);
uint16_t pathRunRouteCount(void);
uint32_t pathRunRouteGeometryCrc32(void);
void pathFollowerInvalidateRoute(void);
void pathFollowerReset(void);
void pathFollowerUpdatePose1ms(int32_t encoderPulse, float gyroDegPerSec);
void pathFollowerUpdateTarget1ms(void);
PathFollowerState pathFollowerGetStatus(void);
uint16_t pathFollowerGetPathBlendPermille(void);
float pathFollowerGetTargetSpeedMps(void);
bool pathFollowerLineIsValid(void);
void readShortcutSettings(void);
void writeShortcutSettings(void);

#endif // PATH_FOLLOWER_H_
