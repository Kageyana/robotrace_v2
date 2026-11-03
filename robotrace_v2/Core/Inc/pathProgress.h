#ifndef PATH_PROGRESS_H_
#define PATH_PROGRESS_H_

//====================================//
// インクルード
//====================================//
#include "runMemory.h"
#include <stdbool.h>
#include <stdint.h>

//====================================//
// 調整パラメータ
//====================================//
#define PATH_PROGRESS_HISTORY_COUNT 20U // 距離標本の保持上限[個]。静的RAMのみ使用する。
#define PATH_PROGRESS_SAMPLE_MM 15.0f // 別の新鮮な観測を保持する最小移動量[mm]。
#define PATH_PROGRESS_MIN_SPAN_MM 220.0f // 形状照合に必要な履歴の最小移動量[mm]。
#define PATH_PROGRESS_HISTORY_MAX_MS 400U // 最古の観測を許可する時間[ms]。
#define PATH_PROGRESS_EVALUATE_MS 20U // 照合開始の最小間隔[ms]。1回の反復だけを各1msへ分割する。
#define PATH_PROGRESS_ITERATIONS 4U // 1候補あたりの最小二乗反復上限[回]。
#define PATH_PROGRESS_SEARCH_POINTS 5U // 各履歴の予測位相から白線線分を探す前後幅[経路点]。
#define PATH_PROGRESS_SEED_MM 30.0f // 3候補を開始する前後変位[mm]。中央候補も評価する。
#define PATH_PROGRESS_MAX_ALONG_MM 60.0f // 受理する前後位置差の上限[mm]。
#define PATH_PROGRESS_MAX_LATERAL_MM 35.0f // 横位置を前後誤差へ取り違えないための同時推定上限[mm]。
#define PATH_PROGRESS_MAX_HEADING_RAD 0.13962634f // 同時推定する方位差の上限[rad]（8deg）。方位へ直接適用しない。
#define PATH_PROGRESS_ROTATION_SCALE_MM 100.0f // 方位変数の数値条件を位置と揃える換算長[mm/rad]。
#define PATH_PROGRESS_RESIDUAL_MAX_MM 8.0f // 最終照合で許可する各点の法線残差[mm]。
#define PATH_PROGRESS_INITIAL_RESIDUAL_MM 40.0f // 反復途中に許可する各点の地図からの距離[mm]。
#define PATH_PROGRESS_RMS_MAX_MM 2.5f // 最終照合で許可する法線残差RMS[mm]。
#define PATH_PROGRESS_INFORMATION_MIN 0.08f // 横位置・方位を分離した前後方向の情報量下限[無次元]。
#define PATH_PROGRESS_AMBIGUITY_MM 8.0f // 別解とみなす前後差[mm]。
#define PATH_PROGRESS_AMBIGUITY_MSE_MM2 1.0f // 別解を棄却しきれない平均二乗残差の差[mm^2]。
#define PATH_PROGRESS_CONFIRM_MM 6.0f // 別時点で行った連続照合の補正先の許容差[mm]。重なる履歴を独立観測とは数えない。
#define PATH_PROGRESS_CONFIRM_HEADING_RAD 0.03490659f // 連続照合の方位差上限[rad]（2deg）。
#define PATH_PROGRESS_GOAL_MAX_MS 60U // 受理した補正先を使用できる時間[ms]。
#define PATH_PROGRESS_STEP_MAX_MM 0.1f // 1msあたり前後位置補正上限[mm]。白線補正全体の0.2mm上限も守る。
#define PATH_PROGRESS_DEADBAND_MM 2.0f // 地図の整数丸め等による微小な前後補正を保留する幅[mm]。
#define PATH_PROGRESS_INVALID_CLEAR_MS 10U // 有効な白線観測なしで履歴を破棄する時間[ms]。

//====================================//
// グローバル変数の宣言、定義
//====================================//
#define PATH_PROGRESS_POLICY_VERSION 1U

typedef enum
{
    PATH_PROGRESS_DISABLED = 0,
    PATH_PROGRESS_COLLECTING,
    PATH_PROGRESS_MATCHING,
    PATH_PROGRESS_UNOBSERVABLE,
    PATH_PROGRESS_MAP_GAP,
    PATH_PROGRESS_RESIDUAL,
    PATH_PROGRESS_AMBIGUOUS,
    PATH_PROGRESS_CONFIRMING,
    PATH_PROGRESS_ACCEPTED,
    PATH_PROGRESS_NO_LINE
} PathProgressReason;

typedef struct
{
    float poseX_mm, poseY_mm, heading_rad, tangent_rad;
    float lateral_mm, forward_mm;
    uint32_t observationId;
    uint16_t nearest;
    bool enabled, lineValid;
} PathProgressInput;

typedef struct
{
    float applied_mm, absoluteApplied_mm, lastAlong_mm, lastRms_mm, lastInformation;
    uint32_t accepted, rejected;
    PathProgressReason reason;
    uint16_t matchAge_ms;
    uint8_t matchValid;
} PathProgressDiagnostics;

// 位置補正量・件数・理由集合だけを保存成功時にリセットし、最新診断は保持する。
typedef struct
{
    float applied_mm, absoluteApplied_mm, total_mm, along_mm, rms_mm, information;
    uint16_t reasonMask, accepted, rejected, matchAge_ms;
    uint8_t reason, matchValid;
} PathProgressLog;
extern PathProgressLog pathProgressLog;

//====================================//
// プロトタイプ宣言
//====================================//
void pathProgressReset(void);
void pathProgressPredict1ms(float distanceMm, float gyroRadPerSec);
float pathProgressStep1ms(const PathProgressInput *input, const PathWhitePoint *white,
    const uint16_t *arcMm, const uint8_t *flags, uint16_t count, uint8_t blockedFlags);
void pathProgressRecordApplied(float alongMm);
void pathProgressResetLog(void);
const PathProgressDiagnostics *pathProgressDiagnostics(void);

#endif
