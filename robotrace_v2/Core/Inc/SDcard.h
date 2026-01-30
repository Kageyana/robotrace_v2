#ifndef SDCARD_H_
#define SDCARD_H_
//====================================//
// インクルード
//====================================//
#include "main.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
//====================================//
// シンボル定義
//====================================//
#include "log_schema.h" // フィールド順とレコードサイズを定義。

#define BUFFER_SIZE_LOG 512
#define LOG_SIZE LOG_RECORD_SIZE_BYTES // スキーマ由来のレコードサイズ。

#define BUFFER_SIZE_MARKER 500
#define FILENUMBER_NUM 1000		// ログファイルナンバーの上限
#define FILENUMBER_LIMIT 300	// 書き込み不備を警告する数

#define PATH_SETTING "./setting/"

//====================================//
// グローバル変数の宣言
//====================================//
extern int16_t fileNumbers[1000],fileIndexLog, endFileIndex;
extern uint8_t cntLog;
extern int32_t encLog;
extern bool logOverflow;     // ログバッファ上限超過フラグ
extern bool markerOverflow;  // マーカーバッファ上限超過フラグ
extern bool getFileNumbersError; // getFileNumbersでエラーが発生した際のフラグ
//====================================//
// プロトタイプ宣言
//====================================//
// MicroSD
bool insertSD(void);
bool initMicroSD(void);
void createLog(void);
void endTempFile(void);
void endLog(void);
void writeMarkerPos(uint32_t distance, uint8_t marker);
void initLog(void);
// スキーマ順で1レコードを書き込む。
void writeLogBufferPuts(void);
void writeLogPuts(void);
void send8bit(uint8_t data);
void send16bit(uint16_t data);
void send32bit(uint32_t data);
int16_t getFileNumbers(void);
void setLogStr(char *column, char *format);
void setLogHeaderStr(char *name, int32_t value);
void setLogHeaderStrF(char *name, float value);
void SDtest(void);
void createDir(char *dirName);
bool sd_fatfs_lock(uint32_t timeout_ms);
void sd_fatfs_unlock(void);
bool sd_fatfs_is_locked(void);
bool sd_fatfs_try_lock(void);
void sd_set_analysis_active(bool active);
bool sd_is_analysis_active(void);
#endif // SDCARD_H_
