#include "slipDetection.h"
#include <math.h>
#include <string.h>

SlipDetectionState slipV2;
_Static_assert(SLIP_V2_SLOW_MS + SLIP_V2_ENCODER_YAW_DELAY_MS < SLIP_V2_HISTORY,
    "slip history must include delayed windows");
_Static_assert(SLIP_V2_ACCEL_DELAY_MS == 0U && SLIP_V2_GYRO_DELAY_MS == 0U,
    "nonzero IMU delay needs an aligned history implementation");
_Static_assert(SLIP_LAT_CANDIDATE_ON > SLIP_LAT_CANDIDATE_OFF && SLIP_LAT_CANDIDATE_OFF > 0.0F,
    "candidate hysteresis thresholds must be positive and ordered");
_Static_assert(SLIP_LAT_CANDIDATE_LPF_COEF >= 0.0F && SLIP_LAT_CANDIDATE_LPF_COEF < 1.0F,
    "candidate LPF coefficient must be stable");

/////////////////////////////////////////////////////////////////////
// モジュール名 SlipDetection_Reset
// 処理概要     検出器と区間統計を初期化する
// 引数         state: 検出器状態
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void SlipDetection_Reset(SlipDetectionState *state)
{
    memset(state, 0, sizeof(*state));
    state->result.reason = SLIP_V2_WARMUP;
    state->lateralCandidate.result.reason = state->result.reason;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 SlipDetection_ResetInterval
// 処理概要     保存成功したログ区間の統計だけを初期化する
// 引数         state: 検出器状態
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void SlipDetection_ResetInterval(SlipDetectionState *state)
{
    memset(&state->interval, 0, sizeof(state->interval));
    memset(&state->lateralCandidate.interval, 0, sizeof(state->lateralCandidate.interval));
}

/////////////////////////////////////////////////////////////////////
// モジュール名 incrementSaturated
// 処理概要     長いログ間隔でも16bitカウンタを折り返さない
// 引数         value: 加算先
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void incrementSaturated(uint16_t *value)
{
    if (*value < UINT16_MAX) ++*value;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 SlipDetection_LateralCandidateMps2
// 処理概要     右X・前Y・CW正の候補接線項へ置換し向心補正を維持する
// 引数         currentMps2: 共通補正済み横加速度[m/s^2], appliedTangentialMps2: 適用済み接線項[m/s^2]
// 戻り値       現行接線項だけを逆符号へ置換した横加速度[m/s^2]
/////////////////////////////////////////////////////////////////////
float SlipDetection_LateralCandidateMps2(float currentMps2, float appliedTangentialMps2)
{
    return currentMps2 - 2.0F * appliedTangentialMps2;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 invalidateCandidate
// 処理概要     候補窓のみを初期化し未保存の区間統計を保持する
// 引数         state: 候補状態, reason: 無効理由
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void invalidateCandidate(SlipLateralCandidateState *state, uint16_t reason)
{
    state->filterIndex = state->filterCount = state->historyIndex = state->historyCount = state->sampleCount = 0U;
    state->highMs = state->lowMs = state->result.flag = state->result.valid = 0U;
    state->result.residual = 0.0F;
    state->neutralFiltered = 0.0F;
    state->neutralStarted = false;
    state->result.reason = reason;
    state->interval.invalidReason |= reason;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 updateCandidate
// 処理概要     同じ5ms平滑化と30ms窓で候補横残差・継続判定を更新する
// 引数         state: 候補状態, input: 同一1ms入力, reason: 共通入力の無効理由
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void updateCandidate(SlipLateralCandidateState *state, const SlipDetectionInput *input, uint16_t reason)
{
    bool finite = isfinite(input->yawAlphaRadps2) && isfinite(input->tangentialAppliedXMps2);
    state->result.alphaRadps2 = finite && input->tangentialOk ? input->yawAlphaRadps2 : 0.0F;
    state->result.tangentialAppliedXMps2 = finite && input->tangentialOk ? input->tangentialAppliedXMps2 : 0.0F;
    if (!input->tangentialOk) reason |= SLIP_V2_TANGENTIAL_UNAVAILABLE;
    if (!finite) reason |= SLIP_V2_NONFINITE;
    if (reason) { invalidateCandidate(state, reason); return; }
    float a = SlipDetection_LateralCandidateMps2(input->lateralMps2, input->tangentialAppliedXMps2);
    if (!isfinite(a)) { invalidateCandidate(state, SLIP_V2_NONFINITE); return; }
    state->filter[state->filterIndex] = a;
    state->filterIndex = (uint16_t)((state->filterIndex + 1U) % SLIP_V2_FILTER_MS);
    if (state->filterCount < SLIP_V2_FILTER_MS) ++state->filterCount;
    if (state->sampleCount < SLIP_V2_READY_SAMPLES) ++state->sampleCount;
    state->result.valid = 0U;
    state->result.reason = SLIP_V2_WARMUP;
    if (state->filterCount == SLIP_V2_FILTER_MS) {
        float mean = 0.0F;
        for (unsigned k = 0; k < SLIP_V2_FILTER_MS; ++k) mean += state->filter[k];
        mean /= SLIP_V2_FILTER_MS;
        if (!isfinite(mean)) { invalidateCandidate(state, SLIP_V2_NONFINITE); return; }
        // baselineと同じ順で平滑化したvとomegaを使う。積の平均へ置換しない。
        state->history[state->historyIndex] = mean;
        state->historyIndex = (uint16_t)((state->historyIndex + 1U) % SLIP_V2_SLOW_MS);
        if (state->historyCount < SLIP_V2_SLOW_MS) ++state->historyCount;
    }
    if (state->sampleCount < SLIP_V2_READY_SAMPLES || state->historyCount < SLIP_V2_SLOW_MS)
        state->interval.invalidReason |= SLIP_V2_WARMUP;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 finishCandidate
// 処理概要     接線補正を除いた残差へ同じ一次LPFを適用し補正項との応答をそろえる
// 引数         state: 全検出器状態
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void finishCandidate(SlipDetectionState *state)
{
    SlipLateralCandidateState *c = &state->lateralCandidate;
    if (c->sampleCount < SLIP_V2_READY_SAMPLES || c->historyCount < SLIP_V2_SLOW_MS) return;
    float sum = 0.0F;
    // baselineの新しい値から古い値への加算順も揃える。
    for (unsigned k = 0; k < SLIP_V2_SLOW_MS; ++k) {
        const SlipDetectionFrame *f = &state->history[(state->historyIndex + SLIP_V2_HISTORY - 1U - k) % SLIP_V2_HISTORY];
        sum += c->history[(c->historyIndex + SLIP_V2_SLOW_MS - 1U - k) % SLIP_V2_SLOW_MS] - f->v * f->yawRad;
    }
    // 同じ窓の共通残差と符号置換残差から、接線補正前の残差と適用済み項を分離する。
    float unaligned = sum / SLIP_V2_SLOW_MS;
    float neutral = 0.5F * state->result.lateral + 0.5F * unaligned;
    float tangent = 0.5F * state->result.lateral - 0.5F * unaligned;
    if (!isfinite(unaligned) || !isfinite(neutral) || !isfinite(tangent)) {
        invalidateCandidate(c, SLIP_V2_NONFINITE); return;
    }
    // 開始・無効復帰では最初の有効値で初期化する。区間統計リセットでは保持する。
    c->neutralFiltered = c->neutralStarted ?
        SLIP_LAT_CANDIDATE_LPF_COEF * c->neutralFiltered +
        (1.0F - SLIP_LAT_CANDIDATE_LPF_COEF) * neutral : neutral;
    c->neutralStarted = true;
    c->result.residual = c->neutralFiltered - tangent;
    if (!isfinite(c->result.residual)) { invalidateCandidate(c, SLIP_V2_NONFINITE); return; }
    c->result.valid = 1U;
    c->result.reason = 0U;
    float magnitude = fabsf(c->result.residual);
    if (!c->result.flag) {
        c->lowMs = 0U;
        c->highMs = magnitude > SLIP_LAT_CANDIDATE_ON ? (uint8_t)(c->highMs + 1U) : 0U;
        if (c->highMs >= SLIP_V2_ON_MS) { c->result.flag = 1U; c->highMs = 0U; }
    } else {
        c->highMs = 0U;
        c->lowMs = magnitude < SLIP_LAT_CANDIDATE_OFF ? (uint8_t)(c->lowMs + 1U) : 0U;
        if (c->lowMs >= SLIP_V2_OFF_MS) { c->result.flag = 0U; c->lowMs = 0U; }
    }
    c->interval.peak = fmaxf(c->interval.peak, magnitude);
    incrementSaturated(&c->interval.validMs);
    if (c->result.flag) incrementSaturated(&c->interval.onMs);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 frameAgo
// 処理概要     最新の平滑化サンプルから指定時間前の値を取得する
// 引数         state: 状態, ago: 過去時間[ms]
// 戻り値       指定サンプル
/////////////////////////////////////////////////////////////////////
static const SlipDetectionFrame *frameAgo(const SlipDetectionState *state, unsigned ago)
{
    return &state->history[(state->historyIndex + SLIP_V2_HISTORY - 1U - ago) % SLIP_V2_HISTORY];
}

/////////////////////////////////////////////////////////////////////
// モジュール名 updateFlag
// 処理概要     有効な1msサンプルでON/OFFヒステリシスを更新する
// 引数         state: 状態, axis: 判定番号, high: ON条件, low: OFF条件
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void updateFlag(SlipDetectionState *state, unsigned axis, bool high, bool low)
{
    uint8_t bit = (uint8_t)(1U << axis);
    if (!(state->result.flags & bit)) {
        state->lowMs[axis] = 0U;
        state->highMs[axis] = high ? (uint8_t)(state->highMs[axis] + 1U) : 0U;
        if (state->highMs[axis] >= SLIP_V2_ON_MS) {
            state->result.flags |= bit;
            state->highMs[axis] = 0U;
        }
    } else {
        state->highMs[axis] = 0U;
        state->lowMs[axis] = low ? (uint8_t)(state->lowMs[axis] + 1U) : 0U;
        if (state->lowMs[axis] >= SLIP_V2_OFF_MS) {
            state->result.flags &= (uint8_t)~bit;
            state->lowMs[axis] = 0U;
        }
    }
}

/////////////////////////////////////////////////////////////////////
// モジュール名 invalidateWindow
// 処理概要     無効サンプルを窓へ混ぜず区間統計を保持して再充填へ移る
// 引数         state: 状態, reason: 無効理由ビット
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void invalidateWindow(SlipDetectionState *state, uint16_t reason)
{
    state->filterCount = state->filterIndex = state->historyCount = state->historyIndex = 0U;
    memset(state->highMs, 0, sizeof(state->highMs));
    memset(state->lowMs, 0, sizeof(state->lowMs));
    state->result.flags = state->result.valid = 0U;
    state->result.longFast = state->result.longSlow = state->result.lateral = state->result.yawDps = 0.0F;
    state->result.reason = reason;
    state->interval.invalidReason |= reason;
    invalidateCandidate(&state->lateralCandidate, reason |
        (state->lateralCandidate.result.reason & (SLIP_V2_NONFINITE | SLIP_V2_TANGENTIAL_UNAVAILABLE)));
}

/////////////////////////////////////////////////////////////////////
// モジュール名 SlipDetection_Update
// 処理概要     同一区間の縦横残差と旋回整合を計算し診断統計を蓄積する
// 引数         state: 状態, input: 1ms入力（位置補正・静止重力除去済み）
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void SlipDetection_Update(SlipDetectionState *state, const SlipDetectionInput *input)
{
    SlipDetectionResult *r = &state->result;
    bool consecutive = !state->timeStarted || input->timeMs - state->previousTime == 1U;
    state->previousTime = input->timeMs;
    state->timeStarted = true;
    float v = ((float)input->pulseL + (float)input->pulseR) * 500.0F / SLIP_V2_PULSE_PER_METER;
    uint16_t reason = 0U;
    if (!input->running) reason |= SLIP_V2_NOT_RUNNING;
    if (input->calibrating) reason |= SLIP_V2_CALIBRATING;
    if (!input->gyroOk || !input->accelOk || input->timeMs - input->gyroTimeMs > 1U ||
        input->timeMs - input->accelTimeMs > 1U) reason |= SLIP_V2_IMU_ERROR;
    if (!isfinite(input->forwardMps2) || !isfinite(input->lateralMps2) ||
        !isfinite(input->yawDps)) reason |= SLIP_V2_NONFINITE;
    if (fabsf(v) < 0.1F) reason |= SLIP_V2_LOW_SPEED;
    if (!consecutive) reason |= SLIP_V2_TIME_GAP;

    int32_t pwm = (input->pwmL < 0 ? -(int32_t)input->pwmL : input->pwmL) +
        (input->pwmR < 0 ? -(int32_t)input->pwmR : input->pwmR);
    r->pwmSum = (uint16_t)(pwm > 2000 ? 2000 : pwm);
    r->currentValid = input->currentOk && isfinite(input->currentL) && isfinite(input->currentR) &&
        input->timeMs - input->currentTimeMs <= 20U;
    if (!r->currentValid) {
        r->currentHighSamples = 0U;
        state->currentStarted = false;
    } else if (!state->currentStarted || input->currentSequence != state->currentSequence) {
        if (state->currentStarted && input->currentSequence - state->currentSequence != 1U)
            r->currentHighSamples = 0U;
        state->currentSequence = input->currentSequence;
        state->currentStarted = true;
        if (fabsf(input->currentL) + fabsf(input->currentR) > 0.60F)
            incrementSaturated(&r->currentHighSamples);
        else r->currentHighSamples = 0U;
    }
    if (input->running) incrementSaturated(&state->interval.totalMs);
    updateCandidate(&state->lateralCandidate, input, reason);
    if (reason != 0U) {
        invalidateWindow(state, reason);
        return;
    }
    state->filter[state->filterIndex] = (SlipDetectionFrame){v, input->forwardMps2,
        input->lateralMps2, input->yawDps / SLIP_V2_RAD_TO_DEG,
        ((float)input->pulseL - (float)input->pulseR) * 1000.0F / SLIP_V2_PULSE_PER_METER};
    state->filterIndex = (uint16_t)((state->filterIndex + 1U) % SLIP_V2_FILTER_MS);
    if (state->filterCount < SLIP_V2_FILTER_MS) ++state->filterCount;
    r->valid = 0U;
    r->reason = SLIP_V2_WARMUP;
    if (state->filterCount < SLIP_V2_FILTER_MS) {
        state->interval.invalidReason |= SLIP_V2_WARMUP;
        return;
    }
    SlipDetectionFrame mean = {0};
    for (unsigned k = 0; k < SLIP_V2_FILTER_MS; ++k) {
        mean.v += state->filter[k].v;
        mean.a += state->filter[k].a;
        mean.lateral += state->filter[k].lateral;
        mean.yawRad += state->filter[k].yawRad;
        mean.wheelDiff += state->filter[k].wheelDiff;
    }
    mean.v /= SLIP_V2_FILTER_MS;
    mean.a /= SLIP_V2_FILTER_MS;
    mean.lateral /= SLIP_V2_FILTER_MS;
    mean.yawRad /= SLIP_V2_FILTER_MS;
    mean.wheelDiff /= SLIP_V2_FILTER_MS;
    if (!isfinite(mean.a) || !isfinite(mean.lateral) || !isfinite(mean.yawRad)) {
        invalidateWindow(state, SLIP_V2_NONFINITE);
        return;
    }
    state->history[state->historyIndex] = mean;
    state->historyIndex = (uint16_t)((state->historyIndex + 1U) % SLIP_V2_HISTORY);
    if (state->historyCount < SLIP_V2_HISTORY) ++state->historyCount;
    if (state->historyCount < SLIP_V2_SLOW_MS + SLIP_V2_ENCODER_YAW_DELAY_MS + 1U) {
        state->interval.invalidReason |= SLIP_V2_WARMUP;
        return;
    }

    float aFast = 0.0F, aSlow = 0.0F, lat = 0.0F, yaw = 0.0F;
    for (unsigned k = 0; k < SLIP_V2_SLOW_MS; ++k) {
        const SlipDetectionFrame *f = frameAgo(state, k);
        const SlipDetectionFrame *old = frameAgo(state, k + SLIP_V2_ENCODER_YAW_DELAY_MS);
        aSlow += f->a;
        if (k < SLIP_V2_FAST_MS) aFast += f->a;
        lat += f->lateral - f->v * f->yawRad;
        // 保存モデルは過去8msの車輪差と現在のgyro・平均速度を比較する。
        yaw += (old->wheelDiff - SLIP_V2_YAW_OFFSET_MPS - SLIP_V2_YAW_SCALE_DIFF * f->v) /
            SLIP_V2_YAW_WIDTH_M - f->yawRad;
    }
    r->longFast = (mean.v - frameAgo(state, SLIP_V2_FAST_MS)->v) *
        (1000.0F / SLIP_V2_FAST_MS) - aFast / SLIP_V2_FAST_MS;
    r->longSlow = (mean.v - frameAgo(state, SLIP_V2_SLOW_MS)->v) *
        (1000.0F / SLIP_V2_SLOW_MS) - aSlow / SLIP_V2_SLOW_MS;
    r->lateral = lat / SLIP_V2_SLOW_MS;
    r->yawDps = yaw * (SLIP_V2_RAD_TO_DEG / SLIP_V2_SLOW_MS);
    if (!isfinite(r->longFast) || !isfinite(r->longSlow) || !isfinite(r->lateral) || !isfinite(r->yawDps)) {
        invalidateWindow(state, SLIP_V2_NONFINITE);
        return;
    }
    r->valid = 1U;
    r->reason = 0U;
    finishCandidate(state);
    // 縦判定は10ms窓だけで継続判定し、30ms窓は比較診断として計算・記録を続ける。
    updateFlag(state, 0U, fabsf(r->longFast) > SLIP_V2_LONG_ON,
        fabsf(r->longFast) < SLIP_V2_LONG_OFF);
    // 横は時間応答を整合した候補版5を正式採用する。基準残差はfinishCandidateの内部計算まで使用する。
    r->lateral = state->lateralCandidate.result.valid ? state->lateralCandidate.result.residual : 0.0F;
    r->flags &= (uint8_t)~SLIP_V2_LAT;
    if (state->lateralCandidate.result.valid && state->lateralCandidate.result.flag) r->flags |= SLIP_V2_LAT;
    updateFlag(state, 2U, fabsf(r->yawDps) > SLIP_V2_YAW_ON_DPS, fabsf(r->yawDps) < SLIP_V2_YAW_OFF_DPS);
    state->interval.peakLong = fmaxf(state->interval.peakLong, fmaxf(fabsf(r->longFast), fabsf(r->longSlow)));
    state->interval.peakLat = fmaxf(state->interval.peakLat, fabsf(r->lateral));
    state->interval.peakYawDps = fmaxf(state->interval.peakYawDps, fabsf(r->yawDps));
    incrementSaturated(&state->interval.validMs);
    if (r->flags & SLIP_V2_LONG) incrementSaturated(&state->interval.onLongMs);
    if (r->flags & SLIP_V2_LAT) incrementSaturated(&state->interval.onLatMs);
    if (r->flags & SLIP_V2_YAW) incrementSaturated(&state->interval.onYawMs);
}
