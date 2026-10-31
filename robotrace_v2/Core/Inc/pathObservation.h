#ifndef PATH_OBSERVATION_H_
#define PATH_OBSERVATION_H_
//====================================//
// インクルード
//====================================//
#include <stdbool.h>
#include <stdint.h>

//====================================//
// 調整パラメータ
//====================================//
#define PATH_OBSERVATION_CAUTION_MS 20U // 新鮮な明部がないと減速する連続時間[ms]。1ms周期。
#define PATH_OBSERVATION_STOP_MS 60U // 新鮮な明部がないと自己位置喪失で停止する連続時間[ms]。
#define PATH_OBSERVATION_CAUTION_MM 40.0f // 新鮮な明部なしで減速する移動量[mm]。
#define PATH_OBSERVATION_STOP_MM 120.0f // 新鮮な明部なしで停止する移動量[mm]。
#define PATH_OBSERVATION_MATCH_CAUTION_MS 60U // 有効な白線照合なしで減速する連続時間[ms]。
#define PATH_OBSERVATION_MATCH_STOP_MS 150U // 有効な白線照合なしで停止する連続時間[ms]。
#define PATH_OBSERVATION_MATCH_CAUTION_MM 120.0f // 有効な白線照合なしで減速する移動量[mm]。
#define PATH_OBSERVATION_MATCH_STOP_MM 300.0f // 有効な白線照合なしで停止する移動量[mm]。
#define PATH_OBSERVATION_RECOVERY_MS 100U // 有効な白線照合が連続してから通常速度へ戻す時間[ms]。
#define PATH_OBSERVATION_CAUTION_SPEED_MPS 1.0f // 信頼度低下時の速度上限[m/s]。SDのsearchが低い場合はその値を優先。

//====================================//
// グローバル変数の宣言、定義
//====================================//
typedef struct
{
    uint16_t noLineMs;
    uint16_t noMatchMs;
    uint16_t recoveryMs;
    uint16_t confidencePermille;
    float blindDistanceMm;
    float unmatchedDistanceMm;
    bool caution;
    bool lost;
} PathObservationTrust;

//====================================//
// プロトタイプ宣言
//====================================//
void pathObservationReset(PathObservationTrust *state);
void pathObservationStep1ms(PathObservationTrust *state, bool fresh, bool visible,
    bool matched, float distanceMm);

#endif
