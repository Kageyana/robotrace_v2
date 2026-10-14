#ifndef LOG_SCHEMA_CANDIDATE_V1_H_
#define LOG_SCHEMA_CANDIDATE_V1_H_
#include "log_schema_shadow_v2.h"

// 2026-10-12の横補正候補付き72列/183 B・87列/220 Bを固定する。
// 固定済みの初回SHADOW列へ当時の10列を追加し、現行プロファイルから独立させる。
#define LOG_CANDIDATE_V1_FIELD_LIST_EXTRA(STORED, DERIVED) \
    STORED(F32, slipV2LatCandidate, "%f", slipShadow.lateralCandidate.result.residual) \
    STORED(U8, slipV2LatCandidateFlag, "%d", slipShadow.lateralCandidate.result.flag) \
    STORED(U8, slipV2LatCandidateValid, "%d", slipShadow.lateralCandidate.result.valid) \
    STORED(U16, slipV2LatCandidateReason, "%d", slipShadow.lateralCandidate.result.reason) \
    STORED(F32, slipV2LatCandidatePeak, "%f", slipShadow.lateralCandidate.interval.peak) \
    STORED(U16, slipV2LatCandidateOnMs, "%d", slipShadow.lateralCandidate.interval.onMs) \
    STORED(U16, slipV2LatCandidateValidMs, "%d", slipShadow.lateralCandidate.interval.validMs) \
    STORED(U16, slipV2LatCandidateIntervalReason, "%d", slipShadow.lateralCandidate.interval.invalidReason) \
    STORED(F32, imuYawAlphaFiltered, "%f", slipShadow.lateralCandidate.result.alphaRadps2) \
    STORED(F32, imuLatTangentialApplied, "%f", slipShadow.lateralCandidate.result.tangentialAppliedXMps2)

#define LOG_CANDIDATE_V1_FIELD_LIST_LIGHT(STORED, DERIVED) \
    LOG_V2_FIELD_LIST_LIGHT(STORED, DERIVED) \
    LOG_CANDIDATE_V1_FIELD_LIST_EXTRA(STORED, DERIVED)
#define LOG_CANDIDATE_V1_FIELD_LIST_DEBUG_PROFILE(STORED, DERIVED) \
    LOG_V2_FIELD_LIST_DEBUG_PROFILE(STORED, DERIVED) \
    LOG_CANDIDATE_V1_FIELD_LIST_EXTRA(STORED, DERIVED)
#endif
