#ifndef LOG_SOURCE_H_
#define LOG_SOURCE_H_
//====================================//
// インクルード
//====================================//
#include <stdbool.h>
#include <stdint.h>
#include "ff.h"
#include "logBinary.h"

typedef enum { LOG_SOURCE_DISTANCE, LOG_SOURCE_SLIP, LOG_SOURCE_PATH } LogSourcePurpose;

// 現行binから復元した解析行。距離はpulse、速度はpulse/ms、角速度はdeg/s、ROCとXYはmm。
typedef struct {
    int32_t cntlog, encCurrentN;
    float gyroValZ;
    int32_t courseMarker, encTotalOptimal;
    float ROC, targetSpeed;
    int32_t optimalIndex;
    int32_t slipV2TotalMs, slipV2ValidMs, slipV2OnLongMs;
    int32_t slipV2LatValidMs, slipV2OnLatMs;
    int32_t slipV2IntervalReason, slipV2LatIntervalReason;
    float x, y;
} LogSourceRow;

//====================================//
// グローバル変数の宣言、定義
//====================================//

//====================================//
// プロトタイプ宣言
//====================================//
// 停止中・FatFsロック下で1ファイルずつ使用する。開く時に全CRCを検証する。
FRESULT logSourceOpen(FIL *file, int logNumber, LogSourcePurpose purpose);
bool logSourceInfo(FIL *file, LogBinaryHeader *header);
// 戻り値はcloseまたは次のopenまで有効。呼出側から変更しない。
const char *logSourceMetadata(FIL *file);
bool logSourceNext(FIL *file, LogSourceRow *row);
FRESULT logSourceClose(FIL *file);
int logSourceError(FIL *file);
int logSourceEof(FIL *file);
bool logSourceValidateSlipRow(const LogSourceRow *row);
// メタデータから重複のない整数相当値を範囲検証して取得する。
bool logMetadataReadInt(const char *line, const char *key, int32_t *value);
#endif
