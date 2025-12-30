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
#define LOG_RUNNING_WRITE

#ifdef LOG_RUNNING_WRITE
#define BUFFER_SIZE_LOG 512
#define LOG_NUM_8BIT 3
#define LOG_NUM_16BIT 7
#define LOG_NUM_32BIT 1
#define LOG_NUM_FLOAT 8
#define LOG_SIZE (LOG_NUM_8BIT * sizeof(uint8_t)) + (LOG_NUM_16BIT * sizeof(uint16_t)) + (LOG_NUM_32BIT * sizeof(uint32_t)) + (LOG_NUM_FLOAT * sizeof(float))
#else
#define BUFFER_SIZE_LOG 5000 // 綴りの誤りを修正
#endif

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
#ifdef LOG_RUNNING_WRITE
void writeLogBufferPuts(uint8_t c, uint8_t s, uint8_t i, uint8_t f, ...);
void writeLogPuts(void);
void send8bit(uint8_t data);
void send16bit(uint16_t data);
void send32bit(uint32_t data);
uint8_t logPut8bit(void);
uint16_t logPut16bit(void);
uint32_t logPut32bit(void);
#else
void writeLogBufferPrint(void);
void writeLogPrint(void);
#endif
int16_t getFileNumbers(void);
void setLogStr(char *column, char *format);
void setLogHeaderStr(char *name, int32_t value);
void setLogHeaderStrF(char *name, float value);
void SDtest(void);
void createDir(char *dirName);
#endif // SDCARD_H_
