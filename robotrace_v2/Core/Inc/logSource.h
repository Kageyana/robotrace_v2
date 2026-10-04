#ifndef LOG_SOURCE_H_
#define LOG_SOURCE_H_
#include "main.h"
#include "ff.h"
#include "courseLogCsv.h"

// 停止中、FatFsロック下で1ファイルずつ使用する共通ログ読込。
FRESULT logSourceOpen(FIL *file, const char *csvName, BYTE mode);
FRESULT logSourceClose(FIL *file);
char *logSourceGets(char *line, int capacity, FIL *file);
int logSourceError(FIL *file);
int logSourceEof(FIL *file);
bool logSourceDistanceRow(FIL *file, const char *line, const CourseLogColumnMap *columns, CourseLogDistanceRow *row);
bool logSourceSlipRow(FIL *file, const char *line, const CourseLogColumnMap *columns, CourseLogSlipRow *row);
bool logSourceBinaryPoint(FIL *file, float *x, float *y, float *pulse);
bool logSourceIsBinary(FIL *file);
#endif
