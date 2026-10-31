#ifndef PATH_FUSION_H_
#define PATH_FUSION_H_
//====================================//
// インクルード
//====================================//
#include "runMemory.h"
#include <stdbool.h>
#include <stdint.h>

//====================================//
// 調整パラメータ
//====================================//
#define PATH_FUSION_BACKGROUND 400U // PATH_CURVEの白線重心から差し引く背景[正規化AD値]。
#define PATH_FUSION_ACTIVE_TH 800U // 明部として数える閾値[正規化AD値]。
#define PATH_FUSION_MAX_WIDTH 4U // 単一の白線として許可する連続した明部数[個]。
#define PATH_FUSION_WHITE_FORWARD_POINTS 4U // 白線観測地図の前方位相探索[点]。センサーの前方距離は地図に含む。
#define PATH_FUSION_WHITE_SAMPLE_MAX_MM 20.0f // 一次の白線観測を割り当てる車軸経路点との距離上限[mm]。
#define PATH_FUSION_RESIDUAL_CLIP_MM 15.0f // 任意の直接方位補正に使用する白線残差の飽和値[mm]。
#define PATH_FUSION_LOOP_MIN_MM 1000.0f // 周回共通地図を作成できる最短周期[mm]。
#define PATH_FUSION_LOOP_MAX_MM 10000.0f // 周回共通地図を作成できる最長周期[mm]。
#define PATH_FUSION_LOOP_VARIATION 0.05f // 一次のCROSS間距離に許す中央値からの相対差[無次元]。
#define PATH_FUSION_LOOP_CLOSURE_MAX_MM 50.0f // 一次1周目に許す並進閉じずれ[mm]。
#define PATH_FUSION_MARKER_SEARCH_POINTS 8U // マーカーの進行位相に許す経路点差[点]。
#define PATH_FUSION_MARKER_HEADING_DEG 20.0f // マーカーの方向照合に許す方位差[deg]。
#define PATH_FUSION_MARKER_RESIDUAL_MAX_MM 150.0f // マーカーの接線方向残差の許容上限[mm]。
#define PATH_FUSION_UNKNOWN_SLIP_WEIGHT 0.25f // スリップ診断が無効な場合の暫定補正重み[無次元]。
#define PATH_FUSION_BIAS_MAX_RADPS 0.0872664626f // 白線で推定するジャイロずれの上限[rad/s]（5deg/s）。
#define PATH_FUSION_ALPHA_REFERENCE 5.0f // EKFの観測強度1に対応するSD lineAlpha_x1000。0は補正無効。
#define PATH_FUSION_LINE_VARIANCE_MM2 25.0f // 基準の白線観測分散[mm^2]。SD係数と品質重みで割る。
#define PATH_FUSION_REFERENCE_STEP_RATIO 5.0f // 比較モデルの5ms基準を実機1msへ換算する倍率[無次元]。
#define PATH_FUSION_MARKER_VARIANCE_MM2 100.0f // 位相確認済みマーカーの接線観測分散[mm^2]。
#define PATH_FUSION_POSITION_STEP_MAX_MM 0.2f // 全観測を合わせた1msあたり位置補正上限[mm]。
#define PATH_FUSION_HEADING_STEP_MAX_RAD 0.0003490658504f // 白線による方位補正上限[rad/1ms]（0.02deg）。
#define PATH_FUSION_DISTANCE_NOISE_SCALE 0.000225f // 移動量の二乗へ掛ける伝播分散係数[無次元]。
#define PATH_FUSION_POSITION_NOISE_MM2PS 1.0f // 位置の時間あたり伝播分散[mm^2/s]。
#define PATH_FUSION_HEADING_NOISE_RAD2PS 0.000076154355f // 方位の伝播分散[rad^2/s]（0.25 deg^2/s）。
#define PATH_FUSION_BIAS_NOISE_RAD2PS3 0.00000012184697f // ジャイロずれの伝播分散[rad^2/s^3]。

//====================================//
// グローバル変数の宣言、定義
//====================================//
#define PATH_WHITE_MAP_VERSION 2U // 既存の周期観測をゴール延長区間へ割り当てる白線地図版。
#define PATH_FUSION_ESTIMATOR_VERSION 3U // 位置・方位・gyroずれを同じ有効gainで更新する推定器版。

typedef struct
{
    float lateralMm;
    float forwardMm;
} PathFusionLine;

typedef struct
{
    float covariance[4][4];
    float biasRadPerSec;
} PathFusionFilter;

//====================================//
// プロトタイプ宣言
//====================================//
bool pathFusionSenseLine(const uint16_t values[10], const float lateralMm[10],
    const float forwardMm[10], PathFusionLine *line);
float pathFusionPeriod(const float *intervalMm, uint8_t count);
bool pathFusionLoopPoint(const RoutePoint *source, const uint16_t *arcMm,
    uint16_t count, float periodMm, float progressMm, float *xMm, float *yMm);
bool pathFusionNormalizeLoop(RoutePoint *destination, const RoutePoint *source,
    const uint16_t *arcMm, uint16_t count, float periodMm);
bool pathFusionNormalizeWhite(PathWhitePoint *white, const RoutePoint *source,
    const uint16_t *arcMm, uint16_t count, float periodMm);
bool pathFusionWhiteLoopPoint(const PathWhitePoint *white, const RoutePoint *source,
    const uint16_t *arcMm, uint16_t count, float periodMm, float progressMm, PathWhitePoint *point);
float pathFusionSlipWeight(float longitudinalFast, float longitudinalSlow,
    float lateral, float yawDegPerSec, uint8_t flags, bool valid);
void pathFusionFilterReset(PathFusionFilter *filter);
void pathFusionFilterPredict(PathFusionFilter *filter, float distanceMm, float headingRad);
bool pathFusionFilterCorrect(PathFusionFilter *filter, float normalX, float normalY,
    float headingDerivativeMm, float residualMm, float quality, uint16_t alphaX1000,
    uint16_t thetaGainX1e9, float *correctionXmm, float *correctionYmm, float *correctionHeadingRad);
bool pathFusionFilterMarker(PathFusionFilter *filter, float tangentX, float tangentY,
    float residualMm, float *correctionXmm, float *correctionYmm);

#endif
