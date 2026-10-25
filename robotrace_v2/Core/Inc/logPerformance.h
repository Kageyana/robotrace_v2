#ifndef LOG_PERFORMANCE_H_
#define LOG_PERFORMANCE_H_

//====================================//
// インクルード
//====================================//
#include <stdint.h>

#ifndef ROBOTRACE_LOG_PERFORMANCE
#define ROBOTRACE_LOG_PERFORMANCE 0
#endif

// 停止中の処理だけで集計する。時間は割込み待ちを含む経過msで、各段階は重複する。
typedef struct {
    uint32_t scanMs, readMs, processMs;
    uint32_t distanceMs, slipMs, pathMs, shortcutMs;
    uint32_t fullScans, cacheHits, readCalls;
    uint64_t readBytes;
    uint32_t diskReadSingle, diskReadMulti, diskWriteSingle, diskWriteMulti;
    uint64_t diskReadBytes, diskWriteBytes;
} LogPerformanceStats;

//====================================//
// グローバル変数の宣言、定義
//====================================//
#if ROBOTRACE_LOG_PERFORMANCE
extern volatile LogPerformanceStats logPerformanceStats;
#define LOG_PERF_BEGIN(name) uint32_t name = HAL_GetTick()
#define LOG_PERF_END(field, name) (logPerformanceStats.field += (uint32_t)(HAL_GetTick() - (name)))
#define LOG_PERF_ADD(field, value) (logPerformanceStats.field += (value))
#else
#define LOG_PERF_BEGIN(name) ((void)0)
#define LOG_PERF_END(field, name) ((void)0)
#define LOG_PERF_ADD(field, value) ((void)0)
#endif

//====================================//
// プロトタイプ宣言
//====================================//

#endif
