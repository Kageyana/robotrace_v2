//====================================//
// インクルード
//====================================//
#include "pathProgress.h"
#include <math.h>
#include <string.h>

//====================================//
// グローバル変数の宣言、定義
//====================================//
typedef struct { float x, y, distance; uint32_t tick; } ProgressSample;
typedef struct { float x, y; uint16_t index; } ProgressPoint;
typedef struct { float along, lateral, rotation, mse, information; bool valid, evaluated; } ProgressFit;
typedef struct { float x, y, rotation; } ProgressTransform;
static struct
{
    ProgressSample history[PATH_PROGRESS_HISTORY_COUNT];
    ProgressPoint points[PATH_PROGRESS_HISTORY_COUNT];
    ProgressFit fits[3];
    ProgressFit working;
    ProgressTransform candidate, goal;
    float odomX, odomY, odomHeading, distance;
    float anchorX, anchorY, anchorOdomX, anchorOdomY, anchorOdomHeading, anchorHeading;
    float tx, ty, nx, ny;
    uint32_t tick, lastEvaluation, lastObservation, candidateTick, goalTick;
    uint16_t used, head, invalidMs;
    uint8_t seed, iteration;
    bool solving, candidateValid, goalValid;
    PathProgressDiagnostics diagnostics;
} progress;
PathProgressLog pathProgressLog;

//====================================//
// プロトタイプ宣言
//====================================//

/////////////////////////////////////////////////////////////////////
// モジュール名 progressSetReason
// 処理概要     最新状態と、保存区間内に発生した理由の集合を同時更新する
// 引数         reason: 処理状態
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void progressSetReason(PathProgressReason reason)
{
    progress.diagnostics.reason = reason;
    pathProgressLog.reasonMask |= (uint16_t)(1U << (unsigned)reason);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 progressWrap
// 処理概要     方位差を正負piの範囲へ揃える
// 引数         angle: 方位[rad]
// 戻り値       正規化した方位[rad]
/////////////////////////////////////////////////////////////////////
static float progressWrap(float angle)
{
    return remainderf(angle, 6.28318530718f);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 progressClearHistory
// 処理概要     欠測・保護域で観測履歴と未適用の補正先を破棄する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void progressClearHistory(void)
{
    progress.used = progress.head = 0U;
    progress.solving = progress.candidateValid = progress.goalValid = false;
    progress.diagnostics.matchValid = 0U;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathProgressReset
// 処理概要     走行開始時に独立した相対オドメトリと診断を初期化する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void pathProgressReset(void)
{
    memset(&progress, 0, sizeof(progress));
    memset(&pathProgressLog, 0, sizeof(pathProgressLog));
    progress.diagnostics.matchAge_ms = pathProgressLog.matchAge_ms = UINT16_MAX;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathProgressPredict1ms
// 処理概要     位置補正を含めない観測履歴用の相対移動を更新する
// 引数         distanceMm: 生車輪距離[mm/1ms], gyroRadPerSec: 実角速度[rad/s]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void pathProgressPredict1ms(float distanceMm, float gyroRadPerSec)
{
    progress.tick++;
    if (progress.diagnostics.matchAge_ms < UINT16_MAX) progress.diagnostics.matchAge_ms++;
    if (!isfinite(distanceMm) || !isfinite(gyroRadPerSec) || distanceMm < 0.0f)
    {
        progressClearHistory();
        return;
    }
    progress.odomHeading = progressWrap(progress.odomHeading + gyroRadPerSec * 0.001f);
    progress.odomX += distanceMm * sinf(progress.odomHeading);
    progress.odomY += distanceMm * cosf(progress.odomHeading);
    progress.distance += distanceMm;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 progressArcIndex
// 処理概要     予測弧長を対応する経路区間へ有界二分探索で変換する
// 引数         arc: 弧長配列[mm], count: 点数, distance: 予測弧長[mm]
// 戻り値       区間先頭の経路点番号
/////////////////////////////////////////////////////////////////////
static uint16_t progressArcIndex(const uint16_t *arc, uint16_t count, float distance)
{
    uint16_t low = 0U, high = (uint16_t)(count - 1U);
    while ((uint32_t)(high - low) > 1U)
    {
        uint16_t middle = (uint16_t)((low + high) / 2U);
        if ((float)arc[middle] <= distance) low = middle;
        else high = middle;
    }
    return low;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 progressInverse
// 処理概要     横位置と方位を分離する3変数の情報行列を反転する
// 引数         h: 対称情報行列, inverse: 逆行列の出力
// 戻り値       true: 可逆 false: 形状が識別不能または非有限
/////////////////////////////////////////////////////////////////////
static bool progressInverse(float h[3][3], float inverse[3][3])
{
    float a = h[0][0], b = h[0][1], c = h[0][2];
    float d = h[1][1], e = h[1][2], f = h[2][2];
    float determinant = a * (d * f - e * e) - b * (b * f - c * e) + c * (b * e - c * d);
    if (!isfinite(determinant) || determinant <= 1.0e-6f * fmaxf(1.0f, a * d * f)) return false;
    inverse[0][0] = (d * f - e * e) / determinant;
    inverse[0][1] = inverse[1][0] = (c * e - b * f) / determinant;
    inverse[0][2] = inverse[2][0] = (b * e - c * d) / determinant;
    inverse[1][1] = (a * f - c * c) / determinant;
    inverse[1][2] = inverse[2][1] = (b * c - a * e) / determinant;
    inverse[2][2] = (a * d - b * b) / determinant;
    return isfinite(inverse[0][0]) && inverse[0][0] > 0.0f;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 progressStart
// 処理概要     観測履歴を固定コピーし、3候補の照合を開始する
// 引数         input: 最新の補正済み位置, arc: 経路弧長[mm], flags: 保護域,
//              count: 経路点数, blocked: 使用禁止フラグ
// 戻り値       true: 開始 false: 履歴不足または保護域
/////////////////////////////////////////////////////////////////////
static bool progressStart(const PathProgressInput *input, const uint16_t *arc,
    const uint8_t *flags, uint16_t count, uint8_t blocked)
{
    if (progress.used < PATH_PROGRESS_HISTORY_COUNT) return false;
    ProgressSample *oldest = &progress.history[progress.head];
    if (progress.distance - oldest->distance < PATH_PROGRESS_MIN_SPAN_MM ||
        progress.tick - oldest->tick > PATH_PROGRESS_HISTORY_MAX_MS) return false;
    float angle = progressWrap(input->heading_rad - progress.odomHeading);
    float sine = sinf(angle), cosine = cosf(angle);
    for (uint16_t k = 0U; k < PATH_PROGRESS_HISTORY_COUNT; k++)
    {
        const ProgressSample *sample = &progress.history[(progress.head + k) % PATH_PROGRESS_HISTORY_COUNT];
        float arcAtSample = (float)arc[input->nearest] - (progress.distance - sample->distance);
        if (arcAtSample < 0.0f) return false;
        uint16_t index = progressArcIndex(arc, count, arcAtSample);
        float x = sample->x - progress.odomX, y = sample->y - progress.odomY;
        progress.points[k] = (ProgressPoint){cosine * x + sine * y, -sine * x + cosine * y, index};
    }
    uint16_t begin = progress.points[0].index;
    begin = begin > PATH_PROGRESS_SEARCH_POINTS ? (uint16_t)(begin - PATH_PROGRESS_SEARCH_POINTS) : 0U;
    uint16_t end = (uint16_t)fminf((float)(count - 1U), (float)input->nearest + PATH_PROGRESS_SEARCH_POINTS);
    for (uint16_t k = begin; k <= end; k++) if ((flags[k] & blocked) != 0U) return false;
    progress.anchorX = input->poseX_mm; progress.anchorY = input->poseY_mm;
    progress.anchorHeading = input->heading_rad;
    progress.anchorOdomX = progress.odomX; progress.anchorOdomY = progress.odomY;
    progress.anchorOdomHeading = progress.odomHeading;
    progress.tx = sinf(input->tangent_rad); progress.ty = cosf(input->tangent_rad);
    progress.nx = progress.ty; progress.ny = -progress.tx;
    memset(progress.fits, 0, sizeof(progress.fits));
    progress.seed = progress.iteration = 0U;
    progress.working = (ProgressFit){.along = -PATH_PROGRESS_SEED_MM};
    progress.solving = true;
    progressSetReason(PATH_PROGRESS_MATCHING);
    progress.lastEvaluation = progress.tick;
    return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 progressIterate
// 処理概要     前後・横・方位の同時照合を1反復だけ実行する
// 引数         white: 白線地図, flags: 保護域, count: 地図点数, blocked: 禁止フラグ
// 戻り値       true: 反復成功 false: 欠測・特異形状・範囲外
/////////////////////////////////////////////////////////////////////
static bool progressIterate(const PathWhitePoint *white, const uint8_t *flags,
    uint16_t count, uint8_t blocked)
{
    ProgressFit *fit = &progress.working;
    fit->evaluated = false;
    float angle = fit->rotation / PATH_PROGRESS_ROTATION_SCALE_MM;
    float sine = sinf(angle), cosine = cosf(angle);
    float h[3][3] = {{0}}, g[3] = {0}, inverse[3][3];
    float sum = 0.0f, maximum = 0.0f;
    for (uint16_t k = 0U; k < PATH_PROGRESS_HISTORY_COUNT; k++)
    {
        const ProgressPoint *point = &progress.points[k];
        float rx = cosine * point->x + sine * point->y;
        float ry = -sine * point->x + cosine * point->y;
        float x = progress.anchorX + rx + fit->along * progress.tx + fit->lateral * progress.nx;
        float y = progress.anchorY + ry + fit->along * progress.ty + fit->lateral * progress.ny;
        uint16_t begin = point->index > PATH_PROGRESS_SEARCH_POINTS ? (uint16_t)(point->index - PATH_PROGRESS_SEARCH_POINTS) : 0U;
        uint16_t end = (uint16_t)fminf((float)(count - 2U), (float)point->index + PATH_PROGRESS_SEARCH_POINTS);
        float best = INFINITY, normalX = 0.0f, normalY = 0.0f, residual = 0.0f;
        for (uint16_t j = begin; j <= end; j++)
        {
            if (!white[j].valid || !white[j + 1U].valid || ((flags[j] | flags[j + 1U]) & blocked) != 0U) continue;
            float dx = white[j + 1U].x_mm - white[j].x_mm, dy = white[j + 1U].y_mm - white[j].y_mm;
            float lengthSquared = dx * dx + dy * dy;
            if (lengthSquared <= 1.0f) continue;
            float u = ((x - white[j].x_mm) * dx + (y - white[j].y_mm) * dy) / lengthSquared;
            // 線分端だけの一致で、存在しない接線方向の情報を作らない。
            if (u < 0.0f || u > 1.0f) continue;
            float ex = x - white[j].x_mm - u * dx, ey = y - white[j].y_mm - u * dy;
            float squared = ex * ex + ey * ey;
            if (squared < best)
            {
                float inverseLength = 1.0f / sqrtf(lengthSquared);
                best = squared; normalX = dy * inverseLength; normalY = -dx * inverseLength;
                residual = ex * normalX + ey * normalY;
            }
        }
        if (!isfinite(best) || best > PATH_PROGRESS_INITIAL_RESIDUAL_MM * PATH_PROGRESS_INITIAL_RESIDUAL_MM)
        { progressSetReason(PATH_PROGRESS_MAP_GAP); return false; }
        float jacobian[3] = {normalX * progress.tx + normalY * progress.ty,
            normalX * progress.nx + normalY * progress.ny,
            (normalX * ry - normalY * rx) / PATH_PROGRESS_ROTATION_SCALE_MM};
        for (uint8_t i = 0U; i < 3U; i++)
        {
            g[i] += jacobian[i] * residual;
            for (uint8_t j = 0U; j < 3U; j++) h[i][j] += jacobian[i] * jacobian[j];
        }
        sum += residual * residual;
        maximum = fmaxf(maximum, fabsf(residual));
    }
    fit->mse = sum / PATH_PROGRESS_HISTORY_COUNT;
    fit->information = 0.0f;
    fit->evaluated = true;
    if (!progressInverse(h, inverse)) { progressSetReason(PATH_PROGRESS_UNOBSERVABLE); return false; }
    fit->information = 1.0f / inverse[0][0];
    if (fit->information < PATH_PROGRESS_INFORMATION_MIN) { progressSetReason(PATH_PROGRESS_UNOBSERVABLE); return false; }
    if (progress.iteration == PATH_PROGRESS_ITERATIONS - 1U)
    {
        fit->valid = fit->mse <= PATH_PROGRESS_RMS_MAX_MM * PATH_PROGRESS_RMS_MAX_MM && maximum <= PATH_PROGRESS_RESIDUAL_MAX_MM;
        if (!fit->valid) progressSetReason(PATH_PROGRESS_RESIDUAL);
        return fit->valid;
    }
    float delta[3] = {0};
    fit->evaluated = false;
    for (uint8_t i = 0U; i < 3U; i++) for (uint8_t j = 0U; j < 3U; j++) delta[i] -= inverse[i][j] * g[j];
    fit->along += delta[0]; fit->lateral += delta[1]; fit->rotation += delta[2];
    if (!isfinite(fit->along) || !isfinite(fit->lateral) || !isfinite(fit->rotation) ||
        fabsf(fit->along) >= PATH_PROGRESS_MAX_ALONG_MM || fabsf(fit->lateral) > PATH_PROGRESS_MAX_LATERAL_MM ||
        fabsf(fit->rotation) > PATH_PROGRESS_MAX_HEADING_RAD * PATH_PROGRESS_ROTATION_SCALE_MM)
    { progressSetReason(PATH_PROGRESS_RESIDUAL); return false; }
    return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 progressTransformPoint
// 処理概要     相対オドメトリ座標を、照合で求めた地図座標へ変換する
// 引数         transform: 変換, x/y: 相対位置[mm], outX/outY: 地図位置[mm]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void progressTransformPoint(ProgressTransform transform, float x, float y, float *outX, float *outY)
{
    float sine = sinf(transform.rotation), cosine = cosf(transform.rotation);
    *outX = transform.x + cosine * x + sine * y;
    *outY = transform.y - sine * x + cosine * y;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 progressFinish
// 処理概要     別解を拒否し、連続した照合が一致した場合だけ補正先を受理する
// 引数         input: 最新の位置と経路接線
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void progressFinish(const PathProgressInput *input)
{
    int best = -1;
    int diagnostic = -1;
    for (int i = 0; i < 3; i++)
    {
        if (progress.fits[i].evaluated && isfinite(progress.fits[i].mse) &&
            isfinite(progress.fits[i].along) && isfinite(progress.fits[i].information) &&
            (diagnostic < 0 || progress.fits[i].mse < progress.fits[diagnostic].mse)) diagnostic = i;
    }
    progress.diagnostics.matchValid = 0U;
    if (diagnostic >= 0)
    {
        progress.diagnostics.lastAlong_mm = progress.fits[diagnostic].along;
        progress.diagnostics.lastRms_mm = sqrtf(progress.fits[diagnostic].mse);
        progress.diagnostics.lastInformation = progress.fits[diagnostic].information;
        progress.diagnostics.matchAge_ms = 0U;
    }
    for (int i = 0; i < 3; i++) if (progress.fits[i].valid && (best < 0 || progress.fits[i].mse < progress.fits[best].mse)) best = i;
    progress.solving = false;
    if (best < 0) goto rejected;
    ProgressFit fit = progress.fits[best];
    for (int i = 0; i < 3; i++)
    {
        if (progress.fits[i].valid && fabsf(progress.fits[i].along - fit.along) > PATH_PROGRESS_AMBIGUITY_MM &&
            progress.fits[i].mse <= fit.mse + PATH_PROGRESS_AMBIGUITY_MSE_MM2)
        { progressSetReason(PATH_PROGRESS_AMBIGUOUS); goto rejected; }
    }
    ProgressTransform transform = {0.0f, 0.0f,
        progressWrap(progress.anchorHeading - progress.anchorOdomHeading + fit.rotation / PATH_PROGRESS_ROTATION_SCALE_MM)};
    float x, y;
    progressTransformPoint(transform, progress.anchorOdomX, progress.anchorOdomY, &x, &y);
    transform.x = progress.anchorX + fit.along * progress.tx + fit.lateral * progress.nx - x;
    transform.y = progress.anchorY + fit.along * progress.ty + fit.lateral * progress.ny - y;
    progress.diagnostics.lastAlong_mm = fit.along;
    progress.diagnostics.lastRms_mm = sqrtf(fit.mse);
    progress.diagnostics.lastInformation = fit.information;
    progress.diagnostics.matchValid = 1U;
    if (progress.candidateValid && progress.tick - progress.candidateTick <= PATH_PROGRESS_GOAL_MAX_MS)
    {
        float oldX, oldY, newX, newY;
        progressTransformPoint(progress.candidate, progress.odomX, progress.odomY, &oldX, &oldY);
        progressTransformPoint(transform, progress.odomX, progress.odomY, &newX, &newY);
        float difference = (newX - oldX) * sinf(input->tangent_rad) + (newY - oldY) * cosf(input->tangent_rad);
        if (fabsf(difference) <= PATH_PROGRESS_CONFIRM_MM &&
            fabsf(progressWrap(transform.rotation - progress.candidate.rotation)) <= PATH_PROGRESS_CONFIRM_HEADING_RAD)
        {
            progress.goal = transform; progress.goalValid = true; progress.goalTick = progress.tick;
            progress.diagnostics.accepted++;
            if (pathProgressLog.accepted < UINT16_MAX) pathProgressLog.accepted++;
            progressSetReason(PATH_PROGRESS_ACCEPTED);
        }
        else { progress.goalValid = false; progressSetReason(PATH_PROGRESS_CONFIRMING); }
    }
    else progressSetReason(PATH_PROGRESS_CONFIRMING);
    progress.candidate = transform; progress.candidateValid = true; progress.candidateTick = progress.tick;
    return;
rejected:
    progress.diagnostics.rejected++;
    if (pathProgressLog.rejected < UINT16_MAX) pathProgressLog.rejected++;
    progress.candidateValid = progress.goalValid = false;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 progressStep1ms
// 処理概要     観測の保持と有界反復を行い、経路接線方向だけの補正候補を返す
// 引数         input: 位置・新鮮な白線観測, white: 白線地図, arcMm: 弧長[mm],
//              flags: 保護域, count: 地図点数, blockedFlags: 照合禁止フラグ
// 戻り値       前後方向の補正候補[mm/1ms]。位置へ適用した量は別途記録する。
/////////////////////////////////////////////////////////////////////
static float progressStep1ms(const PathProgressInput *input, const PathWhitePoint *white,
    const uint16_t *arcMm, const uint8_t *flags, uint16_t count, uint8_t blockedFlags)
{
    if (input == NULL || white == NULL || arcMm == NULL || flags == NULL || count < 2U || input->nearest >= count ||
        !input->enabled || !isfinite(input->poseX_mm) || !isfinite(input->poseY_mm) ||
        !isfinite(input->heading_rad) || !isfinite(input->tangent_rad) || (flags[input->nearest] & blockedFlags) != 0U)
    { progressClearHistory(); progressSetReason(PATH_PROGRESS_DISABLED); return 0.0f; }
    if (!input->lineValid || !isfinite(input->lateral_mm) || !isfinite(input->forward_mm))
    {
        progress.goalValid = progress.candidateValid = progress.solving = false;
        progress.diagnostics.matchValid = 0U;
        if (progress.invalidMs < PATH_PROGRESS_INVALID_CLEAR_MS) progress.invalidMs++;
        if (progress.invalidMs >= PATH_PROGRESS_INVALID_CLEAR_MS) progressClearHistory();
        progressSetReason(PATH_PROGRESS_NO_LINE);
        return 0.0f;
    }
    progress.invalidMs = 0U;
    if (input->observationId != progress.lastObservation)
    {
        progress.lastObservation = input->observationId;
        uint16_t previous = (uint16_t)((progress.head + PATH_PROGRESS_HISTORY_COUNT - 1U) % PATH_PROGRESS_HISTORY_COUNT);
        if (progress.used == 0U || progress.distance - progress.history[previous].distance >= PATH_PROGRESS_SAMPLE_MM)
        {
            float sine = sinf(progress.odomHeading), cosine = cosf(progress.odomHeading);
            progress.history[progress.head] = (ProgressSample){
                progress.odomX + input->lateral_mm * cosine + input->forward_mm * sine,
                progress.odomY - input->lateral_mm * sine + input->forward_mm * cosine,
                progress.distance, progress.tick};
            progress.head = (uint16_t)((progress.head + 1U) % PATH_PROGRESS_HISTORY_COUNT);
            if (progress.used < PATH_PROGRESS_HISTORY_COUNT) progress.used++;
        }
    }
    if (!progress.solving && progress.tick - progress.lastEvaluation >= PATH_PROGRESS_EVALUATE_MS)
    {
        if (!progressStart(input, arcMm, flags, count, blockedFlags))
        { progressSetReason(PATH_PROGRESS_COLLECTING); progress.goalValid = false; }
    }
    if (progress.solving)
    {
        progressSetReason(PATH_PROGRESS_MATCHING);
        bool valid = progressIterate(white, flags, count, blockedFlags);
        progress.iteration++;
        if (!valid || progress.iteration >= PATH_PROGRESS_ITERATIONS)
        {
            progress.fits[progress.seed] = progress.working;
            progress.seed++; progress.iteration = 0U;
            if (progress.seed >= 3U) progressFinish(input);
            else progress.working = (ProgressFit){.along = progress.seed == 1U ? 0.0f : PATH_PROGRESS_SEED_MM};
        }
    }
    if (!progress.goalValid || progress.tick - progress.goalTick > PATH_PROGRESS_GOAL_MAX_MS) return 0.0f;
    float x, y;
    progressTransformPoint(progress.goal, progress.odomX, progress.odomY, &x, &y);
    float along = (x - input->poseX_mm) * sinf(input->tangent_rad) + (y - input->poseY_mm) * cosf(input->tangent_rad);
    if (!isfinite(along) || fabsf(along) >= PATH_PROGRESS_MAX_ALONG_MM) { progress.goalValid = false; return 0.0f; }
    if (fabsf(along) <= PATH_PROGRESS_DEADBAND_MM) return 0.0f;
    return fminf(PATH_PROGRESS_STEP_MAX_MM, fmaxf(-PATH_PROGRESS_STEP_MAX_MM, along));
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathProgressStep1ms
// 処理概要     補正処理後の同じ周期の最新診断をログ用へ固定する
// 引数         input: 位置・観測, white: 白線地図, arcMm: 弧長[mm],
//              flags: 保護域, count: 点数, blockedFlags: 禁止フラグ
// 戻り値       前後方向の補正候補[mm/1ms]
/////////////////////////////////////////////////////////////////////
float pathProgressStep1ms(const PathProgressInput *input, const PathWhitePoint *white,
    const uint16_t *arcMm, const uint8_t *flags, uint16_t count, uint8_t blockedFlags)
{
    float correction = progressStep1ms(input, white, arcMm, flags, count, blockedFlags);
    pathProgressLog.along_mm = progress.diagnostics.lastAlong_mm;
    pathProgressLog.rms_mm = progress.diagnostics.lastRms_mm;
    pathProgressLog.information = progress.diagnostics.lastInformation;
    pathProgressLog.reason = (uint8_t)progress.diagnostics.reason;
    pathProgressLog.reasonMask |= (uint16_t)(1U << (unsigned)progress.diagnostics.reason);
    pathProgressLog.matchValid = progress.diagnostics.matchValid;
    pathProgressLog.matchAge_ms = progress.diagnostics.matchAge_ms;
    return correction;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathProgressRecordApplied
// 処理概要     全白線補正の合計制限後に実適用した前後補正だけを集計する
// 引数         alongMm: 実適用量[mm/1ms]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void pathProgressRecordApplied(float alongMm)
{
    if (!isfinite(alongMm)) return;
    progress.diagnostics.applied_mm += alongMm;
    progress.diagnostics.absoluteApplied_mm += fabsf(alongMm);
    pathProgressLog.applied_mm += alongMm;
    pathProgressLog.absoluteApplied_mm += fabsf(alongMm);
    pathProgressLog.total_mm = progress.diagnostics.applied_mm;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathProgressResetLog
// 処理概要     ログの保存成功後に区間集計だけをリセットする
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void pathProgressResetLog(void)
{
    pathProgressLog.applied_mm = pathProgressLog.absoluteApplied_mm = 0.0f;
    pathProgressLog.accepted = pathProgressLog.rejected = pathProgressLog.reasonMask = 0U;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 pathProgressDiagnostics
// 処理概要     走行終了ヘッダと検証用の集計診断を取得する
// 引数         なし
// 戻り値       走行開始から保持した診断
/////////////////////////////////////////////////////////////////////
const PathProgressDiagnostics *pathProgressDiagnostics(void)
{
    return &progress.diagnostics;
}
