//====================================//
// インクルード
//====================================//
#include "pathObservation.h"
#include <math.h>
#include <string.h>

//====================================//
// グローバル変数の宣言、定義
//====================================//

//====================================//
// プロトタイプ宣言
//====================================//

/////////////////////////////////////////////////////////////////////
// モジュール名 pathObservationIncrement
// 処理概要     1msカウンタを上限で飽和させる
// 引数         value: 現在の連続時間[ms]
// 戻り値       更新後の連続時間[ms]
/////////////////////////////////////////////////////////////////////
static uint16_t pathObservationIncrement(uint16_t value)
{
    return value < UINT16_MAX ? (uint16_t)(value + 1U) : UINT16_MAX;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathObservationReset
// 処理概要     走行開始時に白線観測の信頼度と欠測履歴を初期化する
// 引数         state: 白線観測の状態
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void pathObservationReset(PathObservationTrust *state)
{
    memset(state, 0, sizeof(*state));
    state->confidencePermille = 1000U;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathObservationStep1ms
// 処理概要     白線の可視性と照合成功を分け、欠測時間と移動量で信頼度を更新する
// 引数         state: 白線観測の状態, fresh: 取得値が新鮮, visible: 明部を1個以上検出
//              matched: 有効な白線照合が成立, distanceMm: 今回の移動量[mm/1ms]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void pathObservationStep1ms(PathObservationTrust *state, bool fresh, bool visible,
    bool matched, float distanceMm)
{
    if (state->lost) return;
    if (!isfinite(distanceMm))
    {
        state->lost = true;
        state->caution = true;
        state->confidencePermille = 0U;
        return;
    }
    float distance = fabsf(distanceMm);
    if (fresh && visible)
    {
        state->noLineMs = 0U;
        state->blindDistanceMm = 0.0f;
    }
    else
    {
        state->noLineMs = pathObservationIncrement(state->noLineMs);
        state->blindDistanceMm = fminf(1000000.0f, state->blindDistanceMm + distance);
    }
    if (fresh && visible && matched)
    {
        state->noMatchMs = 0U;
        state->unmatchedDistanceMm = 0.0f;
        state->recoveryMs = pathObservationIncrement(state->recoveryMs);
    }
    else
    {
        state->noMatchMs = pathObservationIncrement(state->noMatchMs);
        state->unmatchedDistanceMm = fminf(1000000.0f, state->unmatchedDistanceMm + distance);
        state->recoveryMs = 0U;
    }
    bool caution = state->noLineMs >= PATH_OBSERVATION_CAUTION_MS ||
        state->blindDistanceMm >= PATH_OBSERVATION_CAUTION_MM ||
        state->noMatchMs >= PATH_OBSERVATION_MATCH_CAUTION_MS ||
        state->unmatchedDistanceMm >= PATH_OBSERVATION_MATCH_CAUTION_MM;
    state->lost = state->noLineMs >= PATH_OBSERVATION_STOP_MS ||
        state->blindDistanceMm >= PATH_OBSERVATION_STOP_MM ||
        state->noMatchMs >= PATH_OBSERVATION_MATCH_STOP_MS ||
        state->unmatchedDistanceMm >= PATH_OBSERVATION_MATCH_STOP_MM;
    if (caution || state->lost) state->caution = true;
    else if (state->recoveryMs >= PATH_OBSERVATION_RECOVERY_MS) state->caution = false;

    float pressure = fmaxf((float)state->noLineMs / PATH_OBSERVATION_STOP_MS,
        state->blindDistanceMm / PATH_OBSERVATION_STOP_MM);
    pressure = fmaxf(pressure, (float)state->noMatchMs / PATH_OBSERVATION_MATCH_STOP_MS);
    pressure = fmaxf(pressure, state->unmatchedDistanceMm / PATH_OBSERVATION_MATCH_STOP_MM);
    uint16_t confidence = (uint16_t)(1000.0f * (1.0f - fminf(1.0f, pressure)));
    if (state->caution && state->recoveryMs < PATH_OBSERVATION_RECOVERY_MS)
    {
        uint16_t recoveryConfidence = (uint16_t)(500U + (500U * state->recoveryMs) / PATH_OBSERVATION_RECOVERY_MS);
        if (confidence > recoveryConfidence) confidence = recoveryConfidence;
    }
    state->confidencePermille = state->lost ? 0U : confidence;
}
