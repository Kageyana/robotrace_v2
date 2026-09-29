#ifndef SDCARD_H_
#define SDCARD_H_
//====================================//
// インクルード
//====================================//
#include "main.h"
#include "autoRun.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#ifdef DEBUG
#include "debugBench.h"
#endif
//====================================//
// シンボル定義
//====================================//
#include "log_schema.h" // フィールド順とレコードサイズを定義。

#define BUFFER_SIZE_LOG 2048U
#define LOG_SIZE LOG_RECORD_SIZE_BYTES // スキーマ由来のレコードサイズ。

#define BUFFER_SIZE_MARKER 500
#define FILENUMBER_NUM 1000		// ログファイルナンバーの上限
#define FILENUMBER_ALARM 50	// 書き込み不備を警告する数
#define FILENUMBER_LIMIT 100	// 超過時に古いログを削除する上限数

#define PATH_SETTING "./setting/"
#define FILENAME_LOGNUMBER "lognum"	// 保存ログ番号ファイル名(拡張子なし)
#define FILENAME_AUTORUN "./setting/auto_run.txt"

//====================================//
// グローバル変数の宣言
//====================================//
extern int16_t fileNumbers[1000],fileIndexLog, endFileIndex;
extern uint8_t cntLog;
extern int32_t encLog;
extern bool logOverflow;     // ログバッファ上限超過フラグ
extern bool markerOverflow;  // マーカーバッファ上限超過フラグ
extern bool getFileNumbersError; // getFileNumbersでエラーが発生した際のフラグ
extern volatile uint32_t dbg_overflow;
//====================================//
// プロトタイプ宣言
//====================================//
// MicroSD
bool insertSD(void);
bool initMicroSD(void);
void createLog(void);
void readImuTempCompensation(void);
AutoRunConfigLoadResult readAutoRunSettings(void);
void endTempFile(void);
bool endLog(void);
bool logLastPrimaryRouteValid(void);
uint8_t logLastPrimaryRouteReason(void);
void writeMarkerPos(uint32_t distance, uint8_t marker);
void initLog(void);
// スキーマ順で1レコードを書き込む。
void writeLogBufferPuts(void);
void writeLogPuts(void);
void send8bit(uint8_t data);
void send16bit(uint16_t data);
void send32bit(uint32_t data);
int16_t getFileNumbers(void);
int16_t getNextLogNumber(void);
int16_t getLastLogNumber(void);
void setLogStr(char *column, char *format);
void setLogHeaderStr(char *name, int32_t value);
void setLogHeaderStrF(char *name, float value);
void setLogHeaderStrS(char *name, const char *value);
void SDtest(void);
void createDir(char *dirName);
bool sd_fatfs_lock(uint32_t timeout_ms);
void sd_fatfs_unlock(void);
bool sd_fatfs_is_locked(void);
bool sd_fatfs_try_lock(void);
void sd_set_analysis_active(bool active);
bool sd_is_analysis_active(void);
void sd_flush_log(void);
#ifdef DEBUG
typedef struct
{
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
	uint32_t writeAlignmentErrors;
	uint32_t writeMetricOverflow;
	uint32_t maxWriteWaitMs;
	uint32_t lastWritePosition;
	uint32_t lastWriteLength;
	uint32_t lastWriteWritten;
	uint32_t lastWriteResult;
	uint32_t logOverflowCount;
	uint32_t writeFailed;
	uint32_t metricsCsvSaved;
} SdBenchStorageResult;

typedef struct
{
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
	uint32_t sdWriteCallCount;
	uint32_t maxSdWriteSectors;
	uint32_t multiSectorWriteCalls;
} SdBenchRuntimeMetrics;

bool sdBenchStart(const char *csvName, const char *metricsName, const char *modeName);
void sdBenchSetRuntimeMetrics(const SdBenchRuntimeMetrics *metrics);
bool sdBenchFinish(uint32_t stopReason, uint32_t elapsedMs, SdBenchStorageResult *result);
bool sdBenchHasWriteFailure(void);
bool sdBenchHasMetricsOverflow(void);
#endif
#endif // SDCARD_H_
