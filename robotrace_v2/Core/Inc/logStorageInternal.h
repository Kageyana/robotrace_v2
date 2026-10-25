#ifndef LOG_STORAGE_INTERNAL_H_
#define LOG_STORAGE_INTERNAL_H_

// SDcard.cとlogSource.c専用の内部ヘッダ。
// 保存と読込だけで共有する停止時検証・作業領域の契約。外部モジュールから使用しない。
#include "logBinary.h"
#include "log_schema.h"
#include "ff.h"

// 保存と停止中読込で共用するI/O領域の寸法。書込キューの実装はSDcard.c内に置く。
#define LOG_BUFFER_SECTOR_SIZE_BYTES 512U
#define LOG_BUFFER_SIZE_BYTES (4U * LOG_BUFFER_SECTOR_SIZE_BYTES)
#define LOG_BUFFER_COUNT 3U

#define LOG_COLUMN_TITLE_BUFFER_SIZE 4096U
typedef union {
    uint8_t logging[LOG_BUFFER_COUNT][LOG_BUFFER_SIZE_BYTES];
    struct { uint8_t input[4096]; } stopped;
} LogIoMemory;
extern LogIoMemory logIoMemory;
extern char columnTitle[LOG_COLUMN_TITLE_BUFFER_SIZE];

#define LOG_SCHEMA_STORED(type, name, fmt, expr) "S," #type "," #name "," fmt "\n"
#define LOG_SCHEMA_DERIVED(type, name, fmt, expr) "D," #type "," #name "," fmt "\n"
enum { LOG_SCHEMA_TEXT_BYTES = sizeof(LOG_FIELD_LIST(LOG_SCHEMA_STORED, LOG_SCHEMA_DERIVED)) - 1U };
extern const char logSchemaText[LOG_SCHEMA_TEXT_BYTES + 1U];
#undef LOG_SCHEMA_STORED
#undef LOG_SCHEMA_DERIVED
#define LOG_COLUMN_ADD(type, name, fmt, expr) + 1U
enum { LOG_BINARY_COLUMNS = 0 LOG_FIELD_LIST(LOG_COLUMN_ADD, LOG_COLUMN_ADD) };
#undef LOG_COLUMN_ADD
_Static_assert(sizeof(logSchemaText) - 1U <= LOG_BINARY_SCHEMA_MAX_BYTES, "schema description too large");
_Static_assert(LOG_BINARY_COLUMNS <= LOG_BINARY_MAX_COLUMNS, "too many log columns");
typedef struct
{
	bool closureValid;
	bool goalMarkerOnsetValid;
	bool distanceScaleVerified;
	int32_t goalMarkerOnset_p;
	int32_t distanceScaleError_p;
	float goalMarkerX_mm;
	uint8_t reason;
} PrimaryRouteValidation;

// 保存処理が固定した状態。読込検証ではNULLを渡し、一次経路判定を行わない。
typedef struct {
    bool writeFailed, imuCalibrationValid;
    uint16_t imuCalibrationSamples, imuCalibrationReadErrors;
} LogSaveCheck;

bool logStorageCanRead(void);
uint32_t logStorageSchemaCrc(void);
bool logStorageReadContainerHeader(FIL *file, uint16_t number, LogBinaryHeader *header);
bool logStorageScanRecords(FIL *file, uint16_t rows, const LogSaveCheck *saved,
    PrimaryRouteValidation *validation, uint32_t *dataCrc);
#endif
