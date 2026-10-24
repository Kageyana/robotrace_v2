#include "slipDetection.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "FAIL %d: %s\n", __LINE__, #x); exit(1); } } while (0)
static SlipDetectionState state;
static SlipDetectionInput input;

/////////////////////////////////////////////////////////////////////
// モジュール名 tick
// 処理概要     新鮮な1ms入力で採用V2を実行し横フラグと統計の一致を検査する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void tick(void)
{
    ++input.timeMs;
    input.gyroTimeMs = input.accelTimeMs = input.timeMs;
    SlipDetection_Update(&state, &input);
    CHECK(((state.result.flags & SLIP_V2_LAT) != 0U) ==
        (state.lateralCandidate.result.valid && state.lateralCandidate.result.flag));
    CHECK(state.interval.onLatMs == state.lateralCandidate.interval.onMs);
    if (state.lateralCandidate.result.valid)
        CHECK(state.result.lateral == state.lateralCandidate.result.residual);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 reset
// 処理概要     モーター電流・PWMに依存しない定速走行を設定する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void reset(void)
{
    SlipDetection_Reset(&state);
    memset(&input, 0, sizeof(input));
    input.running = input.gyroOk = input.accelOk = input.tangentialOk = true;
    input.pulseL = input.pulseR = 100;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 detector
// 処理概要     縦Fast単独・採用横・無効復帰・区間リセットを確認する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void detector(void)
{
    reset();
    for (unsigned i=0;i<42;i++) { tick(); CHECK(!state.result.valid && state.result.flags==0); }
    tick(); CHECK(state.result.valid);
    input.forwardMps2=-30.0F;
    bool seen=false;
    for (unsigned i=0;i<20;i++) {
        tick();
        if ((state.result.flags & SLIP_V2_LONG) && fabsf(state.result.longSlow)<=10.0F) seen=true;
    }
    CHECK(seen);
    reset(); input.lateralMps2=8.0F;
    for(unsigned i=0;i<80;i++) tick();
    CHECK(state.result.flags & SLIP_V2_LAT);
    CHECK(state.interval.onLatMs>0);
    SlipDetection_ResetInterval(&state);
    CHECK(state.result.flags & SLIP_V2_LAT);
    CHECK(state.interval.totalMs==0 && state.lateralCandidate.interval.onMs==0);
    input.tangentialOk=false; tick();
    CHECK(state.result.valid && !(state.result.flags & SLIP_V2_LAT));
    CHECK(state.lateralCandidate.result.reason & SLIP_V2_TANGENTIAL_UNAVAILABLE);
    input.gyroOk=false; tick(); CHECK(!state.result.valid && state.result.flags==0);
    input.gyroOk=input.tangentialOk=true;
    for(unsigned i=0;i<42;i++) { tick(); CHECK(!state.result.valid); }
    tick(); CHECK(state.result.valid);
    input.running=false; tick(); CHECK(!state.result.valid && state.result.flags==0);
}



/////////////////////////////////////////////////////////////////////
// モジュール名 main
// 処理概要     正式採用V2とSLIP入力契約を検証する
// 引数         なし
// 戻り値       成功時0
/////////////////////////////////////////////////////////////////////
int main(void)
{
    detector();
    puts("PASS: active V2 Fast-only, adopted lateral, validity, interval counters");
    return 0;
}
