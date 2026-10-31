//====================================//
// インクルード
//====================================//
#include "pathFusion.h"
#include <math.h>
#include <limits.h>
#include <stddef.h>

//====================================//
// グローバル変数の宣言、定義
//====================================//

//====================================//
// プロトタイプ宣言
//====================================//

/////////////////////////////////////////////////////////////////////
// モジュール名 pathFusionSenseLine
// 処理概要     背景を除いた重心を求め、4個までの単一白線を観測する
// 引数         values: 正規化値, lateralMm/forwardMm: 素子座標[mm], line: 観測出力
// 戻り値       true: 単一白線として有効 false: 不明瞭
/////////////////////////////////////////////////////////////////////
bool pathFusionSenseLine(const uint16_t values[10], const float lateralMm[10],
    const float forwardMm[10], PathFusionLine *line)
{
    uint8_t active = 0U, clusters = 0U;
    bool previous = false;
    uint32_t sum = 0U;
    float lateral = 0.0f, forward = 0.0f;
    for (uint8_t i = 0U; i < 10U; i++)
    {
        if (values[i] > 4095U) return false;
        bool bright = values[i] >= PATH_FUSION_ACTIVE_TH;
        if (bright) { active++; if (!previous) clusters++; }
        previous = bright;
        uint16_t weight = values[i] > PATH_FUSION_BACKGROUND ?
            (uint16_t)(values[i] - PATH_FUSION_BACKGROUND) : 0U;
        sum += weight;
        lateral += (float)weight * lateralMm[i];
        forward += (float)weight * forwardMm[i];
    }
    if (clusters != 1U || active == 0U || active > PATH_FUSION_MAX_WIDTH || sum < 1200U)
        return false;
    line->lateralMm = lateral / (float)sum;
    line->forwardMm = forward / (float)sum;
    return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathFusionPeriod
// 処理概要     一次の周回間隔から、距離のばらつきを検証して周期を求める
// 引数         intervalMm: 連続CROSS間距離[mm], count: 有効区間数（2～8）
// 戻り値       周期[mm]、検証不能時0
/////////////////////////////////////////////////////////////////////
float pathFusionPeriod(const float *intervalMm, uint8_t count)
{
    if (count < 2U || count > 8U) return 0.0f;
    float sorted[8];
    for (uint8_t i = 0U; i < count; i++)
    {
        if (!isfinite(intervalMm[i]) || intervalMm[i] < PATH_FUSION_LOOP_MIN_MM ||
            intervalMm[i] > PATH_FUSION_LOOP_MAX_MM) return 0.0f;
        uint8_t j = i;
        while (j > 0U && sorted[j - 1U] > intervalMm[i])
        {
            sorted[j] = sorted[j - 1U];
            j--;
        }
        sorted[j] = intervalMm[i];
    }
    float period = (count & 1U) ? sorted[count / 2U] :
        (sorted[count / 2U - 1U] + sorted[count / 2U]) * 0.5f;
    for (uint8_t i = 0U; i < count; i++)
        if (fabsf(sorted[i] - period) > period * PATH_FUSION_LOOP_VARIATION) return 0.0f;
    return period;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathFusionSample
// 処理概要     一次経路上の指定弧長位置を有界二分探索で補間する
// 引数         source/arcMm/count: 一次経路, distanceMm: 弧長[mm], xMm/yMm: 出力
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void pathFusionSample(const RoutePoint *source, const uint16_t *arcMm,
    uint16_t count, float distanceMm, float *xMm, float *yMm)
{
    uint16_t low = 0U, high = (uint16_t)(count - 1U);
    while ((uint32_t)(high - low) > 1U)
    {
        uint16_t middle = (uint16_t)(low + (high - low) / 2U);
        if ((float)arcMm[middle] <= distanceMm) low = middle;
        else high = middle;
    }
    float length = (float)arcMm[high] - (float)arcMm[low];
    float u = length > 0.0f ? (distanceMm - (float)arcMm[low]) / length : 0.0f;
    u = fminf(1.0f, fmaxf(0.0f, u));
    *xMm = (float)source[low].x_mm + ((float)source[high].x_mm - source[low].x_mm) * u;
    *yMm = (float)source[low].y_mm + ((float)source[high].y_mm - source[low].y_mm) * u;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathFusionLoopPoint
// 処理概要     一次1周目の閉じずれを弧長へ分配し、周回共通の位置を求める
// 引数         source/arcMm/count: 一次経路, periodMm/progressMm: 周期・進捗[mm]
//              xMm/yMm: 出力位置[mm]
// 戻り値       true: 有効 false: 周期・閉じずれ・進捗が不適切
/////////////////////////////////////////////////////////////////////
bool pathFusionLoopPoint(const RoutePoint *source, const uint16_t *arcMm,
    uint16_t count, float periodMm, float progressMm, float *xMm, float *yMm)
{
    if (count < 3U || !isfinite(periodMm) || !isfinite(progressMm) || progressMm < 0.0f ||
        periodMm < PATH_FUSION_LOOP_MIN_MM || periodMm > PATH_FUSION_LOOP_MAX_MM ||
        periodMm >= (float)arcMm[count - 1U]) return false;
    float endX, endY;
    pathFusionSample(source, arcMm, count, periodMm, &endX, &endY);
    float closureX = endX - (float)source[0].x_mm;
    float closureY = endY - (float)source[0].y_mm;
    if (hypotf(closureX, closureY) > PATH_FUSION_LOOP_CLOSURE_MAX_MM) return false;
    float phase = fmodf(progressMm, periodMm);
    pathFusionSample(source, arcMm, count, phase, xMm, yMm);
    *xMm -= closureX * (phase / periodMm);
    *yMm -= closureY * (phase / periodMm);
    return isfinite(*xMm) && isfinite(*yMm);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathFusionNormalizeLoop
// 処理概要     元の配列を保全しながら全周の経路を同じ1周目の地図へ変換する
// 引数         destination: 出力, source/arcMm/count: 一次経路, periodMm: 周期[mm]
// 戻り値       true: 変換成功 false: 無効な周期または入出力の重複
/////////////////////////////////////////////////////////////////////
bool pathFusionNormalizeLoop(RoutePoint *destination, const RoutePoint *source,
    const uint16_t *arcMm, uint16_t count, float periodMm)
{
    if (destination == source) return false;
    float x, y;
    if (!pathFusionLoopPoint(source, arcMm, count, periodMm, 0.0f, &x, &y)) return false;
    for (uint16_t i = 0U; i < count; i++)
    {
        if (i > 0U && arcMm[i] < arcMm[i - 1U]) return false;
        if (!pathFusionLoopPoint(source, arcMm, count, periodMm, (float)arcMm[i], &x, &y)) return false;
        if (x < INT16_MIN || x > INT16_MAX || y < INT16_MIN || y > INT16_MAX) return false;
        destination[i] = source[i];
        destination[i].x_mm = (int16_t)lroundf(x);
        destination[i].y_mm = (int16_t)lroundf(y);
    }
    return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathFusionNormalizeWhite
// 処理概要     一次で観測した白線位置を車軸経路と同じ周期地図へ変換する
// 引数         white: 観測点（同じ配列へ出力）, source/arcMm/count: 元の車軸経路
//              periodMm: 周回周期[mm]
// 戻り値       true: 有効 false: 周期が不適切
/////////////////////////////////////////////////////////////////////
bool pathFusionNormalizeWhite(PathWhitePoint *white, const RoutePoint *source,
    const uint16_t *arcMm, uint16_t count, float periodMm)
{
    float originX, originY;
    if (!pathFusionLoopPoint(source, arcMm, count, periodMm, 0.0f, &originX, &originY)) return false;
    float endX, endY;
    pathFusionSample(source, arcMm, count, periodMm, &endX, &endY);
    float closureX = endX - originX, closureY = endY - originY;
    // 高いindexから書き換え、後半の周回が参照する未変換の1周目を保全する。
    for (uint16_t remaining = count; remaining > 0U; remaining--)
    {
        uint16_t i = (uint16_t)(remaining - 1U);
        float phase = fmodf((float)arcMm[i], periodMm), x, y;
        if ((float)arcMm[i] < periodMm)
        {
            if (white[i].valid == 0U) continue;
            x = white[i].x_mm; y = white[i].y_mm;
        }
        else
        {
            uint16_t low = 0U, high = (uint16_t)(count - 1U);
            while ((uint32_t)(high - low) > 1U)
            {
                uint16_t middle = (uint16_t)(low + (high - low) / 2U);
                if ((float)arcMm[middle] <= phase) low = middle; else high = middle;
            }
            if (white[low].valid == 0U || white[high].valid == 0U)
                { white[i].valid = 0U; continue; }
            float length = (float)arcMm[high] - arcMm[low];
            float u = length > 0.0f ? (phase - arcMm[low]) / length : 0.0f;
            x = white[low].x_mm + u * ((float)white[high].x_mm - white[low].x_mm);
            y = white[low].y_mm + u * ((float)white[high].y_mm - white[low].y_mm);
        }
        x -= closureX * phase / periodMm;
        y -= closureY * phase / periodMm;
        if (!isfinite(x) || !isfinite(y) || x < INT16_MIN || x > INT16_MAX || y < INT16_MIN || y > INT16_MAX)
            { white[i].valid = 0U; continue; }
        white[i] = (PathWhitePoint){(int16_t)lroundf(x), (int16_t)lroundf(y), 1U};
    }
    return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathFusionWhiteLoopPoint
// 処理概要     未変換の一次観測から指定周回位相の白線位置を求め、欠測を保持する
// 引数         white/source/arcMm/count: 元の白線観測・車軸経路・弧長と点数
//              periodMm/progressMm: 検証済み周期・対象進捗[mm], point: 出力観測
// 戻り値       true: 既存の有効観測から補間成功 false: 周期・入力不正または欠測
/////////////////////////////////////////////////////////////////////
bool pathFusionWhiteLoopPoint(const PathWhitePoint *white, const RoutePoint *source,
    const uint16_t *arcMm, uint16_t count, float periodMm, float progressMm, PathWhitePoint *point)
{
    if (point == NULL) return false;
    *point = (PathWhitePoint){0};
    if (white == NULL || source == NULL || arcMm == NULL ||
        !isfinite(progressMm) || progressMm < 0.0f) return false;
    float originX, originY;
    if (!pathFusionLoopPoint(source, arcMm, count, periodMm, 0.0f, &originX, &originY)) return false;
    float phase = fmodf(progressMm, periodMm);
    uint16_t low = 0U, high = (uint16_t)(count - 1U);
    while ((uint32_t)(high - low) > 1U)
    {
        uint16_t middle = (uint16_t)(low + (high - low) / 2U);
        if ((float)arcMm[middle] <= phase) low = middle; else high = middle;
    }
    if (white[low].valid == 0U || white[high].valid == 0U) return false;
    float length = (float)arcMm[high] - arcMm[low];
    if (length <= 0.0f) return false;
    float u = (phase - (float)arcMm[low]) / length;
    if (u < 0.0f || u > 1.0f) return false;
    float endX, endY;
    pathFusionSample(source, arcMm, count, periodMm, &endX, &endY);
    float x = white[low].x_mm + u * ((float)white[high].x_mm - white[low].x_mm) -
        (endX - originX) * phase / periodMm;
    float y = white[low].y_mm + u * ((float)white[high].y_mm - white[low].y_mm) -
        (endY - originY) * phase / periodMm;
    if (!isfinite(x) || !isfinite(y) || x < INT16_MIN || x > INT16_MAX || y < INT16_MIN || y > INT16_MAX)
        return false;
    *point = (PathWhitePoint){(int16_t)lroundf(x), (int16_t)lroundf(y), 1U};
    return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathFusionSlipWeight
// 処理概要     相関するスリップ指標を最大リスクにまとめ、白線補正の重みを求める
// 引数         longitudinalFast/longitudinalSlow/lateral: 加速度残差[m/s^2]
//              yawDegPerSec: 角速度残差[deg/s], flags: 判定bit, valid: 診断有効
// 戻り値       0～1の補正重み
/////////////////////////////////////////////////////////////////////
float pathFusionSlipWeight(float longitudinalFast, float longitudinalSlow,
    float lateral, float yawDegPerSec, uint8_t flags, bool valid)
{
    if (!valid || !isfinite(longitudinalFast) || !isfinite(longitudinalSlow) ||
        !isfinite(lateral) || !isfinite(yawDegPerSec)) return PATH_FUSION_UNKNOWN_SLIP_WEIGHT;
    float risk = fmaxf(fabsf(longitudinalFast) / 10.0f, fabsf(longitudinalSlow) / 10.0f);
    risk = fmaxf(risk, fabsf(lateral) / 6.7f);
    risk = fmaxf(risk, fabsf(yawDegPerSec) / 60.0f);
    if (flags != 0U) risk = fmaxf(risk, 1.0f);
    return fmaxf(0.02f, 1.0f / (1.0f + risk * risk));
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathFusionFilterReset
// 処理概要     位置・方位・ジャイロずれの共分散を走行開始時に初期化する
// 引数         filter: 推定状態
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void pathFusionFilterReset(PathFusionFilter *filter)
{
    const float rad = 0.017453292519943295f;
    *filter = (PathFusionFilter){0};
    filter->covariance[0][0] = filter->covariance[1][1] = 25.0f;
    filter->covariance[2][2] = rad * rad;
    filter->covariance[3][3] = 0.04f * rad * rad;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathFusionFilterPredict
// 処理概要     1msの移動による共分散を予測し、白線欠測時の不確かさを増やす
// 引数         filter: 推定状態, distanceMm: 移動量[mm], headingRad: 伝播方位[rad]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void pathFusionFilterPredict(PathFusionFilter *filter, float distanceMm, float headingRad)
{
    const float dt = 0.001f;
    float transition[4][4] = {{0}}, intermediate[4][4] = {{0}}, next[4][4] = {{0}};
    for (uint8_t i = 0U; i < 4U; i++) transition[i][i] = 1.0f;
    transition[0][2] = distanceMm * cosf(headingRad);
    transition[1][2] = -distanceMm * sinf(headingRad);
    transition[2][3] = -dt;
    transition[0][3] = -0.5f * dt * transition[0][2];
    transition[1][3] = -0.5f * dt * transition[1][2];
    for (uint8_t i = 0U; i < 4U; i++)
        for (uint8_t j = 0U; j < 4U; j++)
            for (uint8_t k = 0U; k < 4U; k++)
                intermediate[i][j] += transition[i][k] * filter->covariance[k][j];
    for (uint8_t i = 0U; i < 4U; i++)
        for (uint8_t j = 0U; j < 4U; j++)
            for (uint8_t k = 0U; k < 4U; k++)
                next[i][j] += intermediate[i][k] * transition[j][k];
    next[0][0] += PATH_FUSION_DISTANCE_NOISE_SCALE * PATH_FUSION_REFERENCE_STEP_RATIO * distanceMm * distanceMm + PATH_FUSION_POSITION_NOISE_MM2PS * dt;
    next[1][1] += PATH_FUSION_DISTANCE_NOISE_SCALE * PATH_FUSION_REFERENCE_STEP_RATIO * distanceMm * distanceMm + PATH_FUSION_POSITION_NOISE_MM2PS * dt;
    next[2][2] += PATH_FUSION_HEADING_NOISE_RAD2PS * dt;
    next[3][3] += PATH_FUSION_BIAS_NOISE_RAD2PS3 * dt;
    for (uint8_t i = 0U; i < 4U; i++)
        for (uint8_t j = 0U; j < 4U; j++) filter->covariance[i][j] = next[i][j];
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathFusionFilterUpdate
// 処理概要     観測を位置・方位・ジャイロずれへ融合し、適用後のgainで共分散も更新する
// 引数         filter: 推定状態, normalX/normalY: 白線の単位法線
//              headingDerivativeMm: 方位による観測位置の変化[mm/rad], residualMm: 残差[mm]
//              variance: 観測分散[mm^2], updateBias: 白線の方位・ずれ推定を更新
//              extraHeadingGain: 追加の方位補正gain[rad/mm], correctionXmm/Ymm: 補正[mm]
//              correctionHeadingRad: 方位補正出力[rad]
// 戻り値       true: 有効な補正 false: 無効入力または補正無効
/////////////////////////////////////////////////////////////////////
static bool pathFusionFilterUpdate(PathFusionFilter *filter, float normalX, float normalY,
    float headingDerivativeMm, float residualMm, float variance, bool updateBias, float extraHeadingGain,
    float *correctionXmm, float *correctionYmm, float *correctionHeadingRad)
{
    *correctionXmm = *correctionYmm = 0.0f;
    *correctionHeadingRad = 0.0f;
    if (!isfinite(normalX) || !isfinite(normalY) || !isfinite(headingDerivativeMm) ||
        !isfinite(residualMm) || !isfinite(variance) || variance <= 0.0f || !isfinite(extraHeadingGain))
        return false;
    float h[4] = {normalX, normalY, headingDerivativeMm, 0.0f};
    float ph[4] = {0}, gain[4], next[4][4];
    float innovation = variance;
    for (uint8_t i = 0U; i < 4U; i++)
    {
        for (uint8_t j = 0U; j < 4U; j++) ph[i] += filter->covariance[i][j] * h[j];
        innovation += h[i] * ph[i];
    }
    if (!isfinite(innovation) || innovation <= 0.0f) { pathFusionFilterReset(filter); return false; }
    for (uint8_t i = 0U; i < 4U; i++)
        gain[i] = ((i == 2U || i == 3U) && !updateBias) ? 0.0f : ph[i] / innovation;
    if (updateBias) gain[2] += extraHeadingGain;
    // 単一の線では接線方向を観測できない。共分散の相関だけで進行位相を動かさない。
    float normSquared = normalX * normalX + normalY * normalY;
    if (normSquared < 0.5f || normSquared > 1.5f) return false;
    float observedGain = (gain[0] * normalX + gain[1] * normalY) / normSquared;
    gain[0] = observedGain * normalX;
    gain[1] = observedGain * normalY;
    float residual = -residualMm;
    float length = hypotf(gain[0] * residual, gain[1] * residual);
    if (length > PATH_FUSION_POSITION_STEP_MAX_MM)
    {
        float scale = PATH_FUSION_POSITION_STEP_MAX_MM / length;
        for (uint8_t i = 0U; i < 4U; i++) gain[i] *= scale;
    }
    float headingChange = gain[2] * residual;
    if (fabsf(headingChange) > PATH_FUSION_HEADING_STEP_MAX_RAD)
        gain[2] = copysignf(PATH_FUSION_HEADING_STEP_MAX_RAD, headingChange) / residual;
    float biasChange = gain[3] * residual;
    float limitedBias = fminf(PATH_FUSION_BIAS_MAX_RADPS,
        fmaxf(-PATH_FUSION_BIAS_MAX_RADPS, filter->biasRadPerSec + biasChange));
    if (fabsf(biasChange) > 1.0e-12f) gain[3] = (limitedBias - filter->biasRadPerSec) / residual;
    // Joseph形と等価な更新。上限で縮めたgainを共分散にも適用する。
    for (uint8_t i = 0U; i < 4U; i++)
        for (uint8_t j = 0U; j < 4U; j++)
            next[i][j] = filter->covariance[i][j] - gain[i] * ph[j] - ph[i] * gain[j] +
                gain[i] * innovation * gain[j];
    for (uint8_t i = 0U; i < 4U; i++)
        for (uint8_t j = 0U; j < 4U; j++)
        {
            float value = 0.5f * (next[i][j] + next[j][i]);
            if (!isfinite(value)) { pathFusionFilterReset(filter); return false; }
            filter->covariance[i][j] = value;
        }
    filter->biasRadPerSec = limitedBias;
    *correctionXmm = gain[0] * residual;
    *correctionYmm = gain[1] * residual;
    *correctionHeadingRad = gain[2] * residual;
    return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathFusionFilterCorrect
// 処理概要     白線の位置・方位・ジャイロずれを補正し、追加方位勾配もgainへ合成する
// 引数         filter: 推定状態, normalX/Y: 法線, headingDerivativeMm: 方位感度[mm/rad]
//              residualMm: 残差[mm], quality: 品質0～1, alphaX1000: SD位置係数
//              thetaGainX1e9: SD追加方位勾配[1/mm^2 ×10^9]
//              correctionXmm/Ymm: 補正出力[mm], correctionHeadingRad: 方位補正出力[rad]
// 戻り値       true: 補正成立 false: 無効
/////////////////////////////////////////////////////////////////////
bool pathFusionFilterCorrect(PathFusionFilter *filter, float normalX, float normalY,
    float headingDerivativeMm, float residualMm, float quality, uint16_t alphaX1000,
    uint16_t thetaGainX1e9, float *correctionXmm, float *correctionYmm, float *correctionHeadingRad)
{
    if (filter == NULL || correctionXmm == NULL || correctionYmm == NULL || correctionHeadingRad == NULL)
        return false;
    *correctionXmm = *correctionYmm = 0.0f;
    *correctionHeadingRad = 0.0f;
    float normSquared = normalX * normalX + normalY * normalY;
    if (!isfinite(quality) || quality <= 0.0f || !isfinite(headingDerivativeMm) || !isfinite(residualMm) ||
        !isfinite(normalX) || !isfinite(normalY) || normSquared < 0.5f || normSquared > 1.5f)
        return false;
    quality = fminf(1.0f, quality);
    float clippedResidual = fminf(PATH_FUSION_RESIDUAL_CLIP_MM,
        fmaxf(-PATH_FUSION_RESIDUAL_CLIP_MM, residualMm));
    float extraHeadingGain = (float)thetaGainX1e9 * 1.0e-9f / PATH_FUSION_REFERENCE_STEP_RATIO *
        quality * headingDerivativeMm;
    if (fabsf(residualMm) > 1.0e-12f) extraHeadingGain *= clippedResidual / residualMm;
    if (alphaX1000 == 0U)
    {
        // alpha=0ではEKF状態を更新せず、既存の追加方位勾配だけを許可する。
        if (thetaGainX1e9 == 0U) return false;
        float correction = -extraHeadingGain * residualMm;
        if (!isfinite(correction)) return false;
        *correctionHeadingRad = fminf(PATH_FUSION_HEADING_STEP_MAX_RAD,
            fmaxf(-PATH_FUSION_HEADING_STEP_MAX_RAD, correction));
        return true;
    }
    float variance = PATH_FUSION_LINE_VARIANCE_MM2 * PATH_FUSION_REFERENCE_STEP_RATIO /
        (fmaxf(0.02f, fminf(1.0f, quality)) * ((float)alphaX1000 / PATH_FUSION_ALPHA_REFERENCE));
    return pathFusionFilterUpdate(filter, normalX, normalY, headingDerivativeMm, residualMm,
        variance, true, extraHeadingGain, correctionXmm, correctionYmm, correctionHeadingRad);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathFusionFilterMarker
// 処理概要     位相確認済みマーカーで進行方向の位置だけを補正する
// 引数         filter: 推定状態, tangentX/Y: 接線, residualMm: 残差[mm]
//              correctionXmm/Ymm: 補正出力[mm]
// 戻り値       true: 補正成立 false: 無効
/////////////////////////////////////////////////////////////////////
bool pathFusionFilterMarker(PathFusionFilter *filter, float tangentX, float tangentY,
    float residualMm, float *correctionXmm, float *correctionYmm)
{
    if (filter == NULL || correctionXmm == NULL || correctionYmm == NULL) return false;
    float unusedHeadingRad;
    return pathFusionFilterUpdate(filter, tangentX, tangentY, 0.0f, residualMm, PATH_FUSION_MARKER_VARIANCE_MM2,
        false, 0.0f, correctionXmm, correctionYmm, &unusedHeadingRad);
}
