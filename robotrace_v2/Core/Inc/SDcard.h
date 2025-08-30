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

#define BUFFER_SIZE_LOG 512
#define LOG_NUM_8BIT 2
#define LOG_NUM_16BIT 7
#define LOG_NUM_32BIT 1
#define LOG_NUM_FLOAT 1

typedef struct
{
	uint8_t targetSpeed;	// 目標速度
	uint8_t courseMarker;	// コースマーカー種別
	uint16_t time;		// 経過時間[ms]
	int16_t speed;		// 現在速度
	uint16_t optimalIndex;	// ショートカット用インデックス
	int16_t currentL;	// 左モータ電流×10000
	int16_t currentR;	// 右モータ電流×10000
	int16_t lineTracePwm;	// ライントレースPWM
	int16_t veloPwm;		// 速度制御PWM
	uint32_t encTotal;	// 総エンコーダカウント
	float gyroZ;		// ジャイロZ軸角速度
} LogRecord;

#define LOG_SIZE sizeof(LogRecord)

#endif

#define BUFFER_SIZW_LOG 5000
#define BUFFER_SIZW_MARKER 500

#define PATH_SETTING "./setting/"

//====================================//
// グローバル変数の宣言
//====================================//
extern int16_t fileNumbers[1000],fileIndexLog, endFileIndex;
extern uint8_t cntLog;
extern int32_t encLog;
//====================================//
// プロトタイプ宣言
//====================================//
// MicroSD
bool insertSD(void);
bool initMicroSD(void);
void createLog(void);
void endLog(void);
void writeMarkerPos(uint32_t distance, uint8_t marker);
void initLog(void);
#ifdef LOG_RUNNING_WRITE
void writeLogBufferPuts(const LogRecord *rec);
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
void getFileNumbers(void);
void setLogStr(uint8_t *column, uint8_t *format);
void SDtest(void);
void createDir(uint8_t *dirName);
#endif // SDCARD_H_
