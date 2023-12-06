#ifndef SDCARD_H_
#define SDCARD_H_
//====================================//
// インクルード
//====================================//
#include "main.h"
#include "fatfs_sd.h"
//====================================//
// シンボル定義
//====================================//
#define NUM_LOGDATA         7
#define BUFFER_SIZW_LOG     1024

#define PATH_SETTING    "./setting/"
//====================================//
// グローバル変数の宣言
//====================================//
extern int16_t  fileNumbers[1000], fileIndexLog, endFileIndex;
extern uint16_t cntLog;
extern int32_t  encLog;
extern uint32_t logIndex;
//====================================//
// プロトタイプ宣言
//====================================//
// MicroSD
bool initMicroSD(void);
void initLog(void);
void createLog(void);
void endLog(void);
void setLogStr(uint8_t* column, uint8_t* format);
void writeLogBufferPuts (uint8_t valNum, ...);
void writeLogPuts(void);
void writeLogBufferPrint (void);
void writeLogPrint(void);
void getFileNumbers(void);
void SDtest(void);
void createDir(uint8_t *dirName);
#endif // SDCARD_H_
