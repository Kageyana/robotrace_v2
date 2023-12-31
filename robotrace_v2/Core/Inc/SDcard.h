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
#define LOG_RUNNING_WRITE


#ifdef LOG_RUNNING_WRITE

#define BUFFER_SIZE_LOG     512
#define LOG_NUM_8BIT        2
#define LOG_NUM_16BIT       7
#define LOG_NUM_32BIT       1
#define LOG_NUM_FLOAT       1
#define LOG_SIZE            (LOG_NUM_8BIT*1)+(LOG_NUM_16BIT*2)+(LOG_NUM_32BIT*4)+(LOG_NUM_FLOAT*4)

#else

#define BUFFER_SIZW_LOG     3000

#endif

#define PATH_SETTING    "./setting/"

//====================================//
// グローバル変数の宣言
//====================================//
extern int16_t  fileNumbers[1000], fileIndexLog, endFileIndex;
extern uint16_t cntLog;
extern int32_t  encLog;
//====================================//
// プロトタイプ宣言
//====================================//
// MicroSD
bool initMicroSD(void);
void createLog(void);
void endLog(void);
#ifdef	LOG_RUNNING_WRITE
void initLog(void);
void writeLogBufferPuts(uint8_t c, uint8_t s, uint8_t i, uint8_t f, ...);
void writeLogPuts(void);
void send8bit(uint8_t data);
void send16bit(uint16_t data);
void send32bit(uint32_t data);
uint8_t logPut8bit(void);
uint16_t logPut16bit(void);
uint32_t logPut32bit(void);
#else
void writeLogBufferPrint (void);
void writeLogPrint(void);
#endif
void getFileNumbers(void);
void setLogStr(uint8_t* column, uint8_t* format);
void SDtest(void);
void createDir(uint8_t *dirName);
#endif // SDCARD_H_
