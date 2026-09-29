#ifndef DEBUG_BENCH_H_
#define DEBUG_BENCH_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef DEBUG

#define DEBUG_BENCH_PRIMARY_DURATION_MS 36483U
#define DEBUG_BENCH_REPLAY_DURATION_MS 36734U

typedef enum
{
	DEBUG_BENCH_MODE_NONE = 0U,
	DEBUG_BENCH_MODE_PRIMARY = 1U,
	DEBUG_BENCH_MODE_PATH_REPLAY = 2U,
	DEBUG_BENCH_MODE_SHORTCUT = 3U
} DebugBenchMode;

typedef enum
{
	DEBUG_BENCH_STOP_NONE = 0U,
	DEBUG_BENCH_STOP_TRACE_END = 1U,
	DEBUG_BENCH_STOP_TIMEOUT = 2U,
	DEBUG_BENCH_STOP_SD_ERROR = 3U,
	DEBUG_BENCH_STOP_BUFFER_OVERFLOW = 4U,
	DEBUG_BENCH_STOP_PATH_LOST = 5U,
	DEBUG_BENCH_STOP_OVERSPEED = 6U,
	DEBUG_BENCH_STOP_EMERGENCY = 7U,
	DEBUG_BENCH_STOP_ROUTE_SETUP = 8U,
	DEBUG_BENCH_STOP_SD_OPEN = 9U,
	DEBUG_BENCH_STOP_WATCHDOG = 10U,
	DEBUG_BENCH_STOP_INVALID_MODE = 11U,
	DEBUG_BENCH_STOP_WRITE_METRICS_FULL = 12U,
	DEBUG_BENCH_STOP_SENSOR_INIT = 13U,
	DEBUG_BENCH_STOP_NOT_STOPPED = 14U,
	DEBUG_BENCH_STOP_WATCHDOG_RESET = 15U
} DebugBenchStopReason;

typedef enum
{
	DEBUG_BENCH_OUTPUT_NORMAL = 0U,
	DEBUG_BENCH_OUTPUT_FIXED_100 = 1U,
	DEBUG_BENCH_OUTPUT_STOPPED = 2U
} DebugBenchMotorOutputMode;

typedef struct
{
	uint32_t magic;
	uint32_t version;
	uint32_t state;
	uint32_t mode;
	uint32_t stopReason;
	uint32_t success;
	uint32_t elapsedMs;
	uint32_t expectedRows;
	uint32_t csvRows;
	uint32_t csvColumns;
	uint32_t csvColumnMismatchRows;
	uint32_t csvFirstBadRow;
	uint32_t csvFirstBadColumnCount;
	uint32_t csvValidated;
	uint32_t firstCntlog;
	uint32_t lastCntlog;
	uint32_t cntlogMonotonic;
	uint32_t writeCount;
	uint32_t sdWriteCallCount;
	uint32_t maxSdWriteSectors;
	uint32_t multiSectorWriteCalls;
	uint32_t writeAlignmentErrors;
	uint32_t writeMetricOverflow;
	uint32_t maxWriteWaitMs;
	uint32_t lastWritePosition;
	uint32_t lastWriteLength;
	uint32_t lastWriteWritten;
	uint32_t lastWriteResult;
	uint32_t maxInterruptCycles;
	uint32_t maxSyntheticPoseCycles;
	uint32_t maxRunEquivalentInterruptCycles;
	uint32_t maxPeriodCycles;
	uint32_t oneMsPeriodOverruns;
	uint32_t pathLostCount;
	uint32_t lineBrightJudgeCount;
	uint32_t lineUnbrightJudgeCount;
	uint32_t overSpeedJudgeCount;
	uint32_t peakEncoderPulsesPerMs;
	uint32_t logOverflowCount;
	uint32_t sdWriteFailed;
	uint32_t watchdogStarted;
	uint32_t watchdogPrescaler;
	uint32_t watchdogReload;
	uint32_t resetCauseFlags;
} DebugBenchResult;

// GDB: 1=PRIMARY, 2=PATH REPLAY, 3=SHORTCUT。起動後は値0で停止状態。
extern volatile uint32_t debugBenchRequestedMode;
extern volatile DebugBenchResult debugBenchResult;

bool debugBenchMainLoop(void);
void debugBenchCaptureResetCause(void);
bool debugBenchIsRunning(void);
uint32_t debugBenchAdvance1ms(void);
uint32_t debugBenchElapsedMs(void);
uint8_t debugBenchMode(void);
uint8_t debugBenchMotorOutputMode(void);
void debugBenchApplyPlaybackHeading(uint32_t elapsedMs);
void debugBenchRecordSyntheticPoseCycles(uint32_t poseCycles);
void debugBenchRecordJudgements(bool lineBright, bool lineUnbright, bool overSpeed);
void debugBenchRecordPeriod(uint32_t periodCycles);
void debugBenchRecordInterruptTime(uint32_t interruptCycles, uint32_t syntheticPoseCycles);
void debugBenchLatchStop(DebugBenchStopReason reason);
void debugBenchWatchdogRefresh(void);
void debugBenchCompletedBreakpoint(void);
void debugBenchPreStartBreakpoint(void);

#endif /* DEBUG */

#endif /* DEBUG_BENCH_H_ */
