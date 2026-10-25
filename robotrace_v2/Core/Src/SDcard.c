//====================================//
// インクルード
//====================================//
#include "SDcard.h"
#include "logStorageInternal.h"
#include "autoRun.h"
#include "logBinary.h"
#include "courseAnalysis.h"
#include "firmware_version.h"
#include "IMU.h"
#include "calibrationConstants.h"
#include "lineSensor.h"
#include "logPerformance.h"
#include "markerSensor.h"
#include "sd_functions.h"
#include "stdio.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <limits.h>
//====================================//
// グローバル変数の宣言
//====================================//
// MicroSD
FIL fil_W;
FIL fil_R;

// ログヘッダー
// 走行ごとのメタデータを保存・検証する共通領域。
#define LOG_TEMP_PREALLOC_BYTES (2UL * 1024UL * 1024UL)
char columnTitle[LOG_COLUMN_TITLE_BUFFER_SIZE] = "";

// ログバッファ
// Log buffers

_Static_assert(BUFFER_SIZE_LOG == LOG_BUFFER_SIZE_BYTES, "log buffer size mismatch");
_Static_assert((BUFFER_SIZE_LOG % LOG_BUFFER_SECTOR_SIZE_BYTES) == 0U, "log buffer must be sector aligned");
_Static_assert(LOG_RECORD_SIZE_BYTES <= LOG_BUFFER_SIZE_BYTES, "log record must fit in one log buffer");
_Static_assert(SLIP_LAT_CANDIDATE_LPF_COEF == IMU_ALPHA_LPF_COEF,
    "lateral candidate LPF must match the applied angular acceleration LPF");
// 列定義はFlashへ一度だけ生成し、停止時に記録部とメタデータの間へ保存する。
#define LOG_SCHEMA_STORED(type, name, fmt, expr) "S," #type "," #name "," fmt "\n"
#define LOG_SCHEMA_DERIVED(type, name, fmt, expr) "D," #type "," #name "," fmt "\n"
const char logSchemaText[LOG_SCHEMA_TEXT_BYTES + 1U] = LOG_FIELD_LIST(LOG_SCHEMA_STORED, LOG_SCHEMA_DERIVED);
#undef LOG_SCHEMA_STORED
#undef LOG_SCHEMA_DERIVED
// ログ書込キューの状態は、このファイル内だけで保持・更新する。
typedef struct
{
	uint8_t (*buffers)[LOG_BUFFER_SIZE_BYTES];
	uint8_t *activeBuffer;
	uint8_t *flushBuffer;
	uint8_t *pendingBuffer;
	uint8_t *reservedBuffer;
	uint32_t activeLength;
	uint32_t flushLength;
	uint32_t pendingLength;
	volatile bool flushPending;
	volatile bool pendingPending;
	volatile bool overflow;
	volatile bool writeFailed;
	bool recordFailed;
} LogBufferWriter;

static LogBufferWriter logBufferWriter;
_Alignas(LOG_BUFFER_SECTOR_SIZE_BYTES) LogIoMemory logIoMemory;
#define logBuffers (logIoMemory.logging)
_Static_assert(sizeof(logIoMemory) == 6144U, "shared log I/O memory must remain 6 KiB");
#if ROBOTRACE_LOG_PERFORMANCE
volatile LogPerformanceStats logPerformanceStats;
#endif


uint16_t cntSend = 0;
uint16_t logValIndex = 0;
bool logOverflow = false;
bool markerOverflow = false;
volatile uint32_t dbg_overflow = 0;

int16_t fileNumbers[FILENUMBER_NUM];
static uint16_t logFileNumber = 0; // log file number sequence
int16_t fileIndexLog = 0; // 現在使用しているログ番号
int16_t endFileIndex = 0; // ログの最終番号

// カウンタ
uint8_t cntLog = 0;
int32_t encLog = 0;
bool getFileNumbersError = false; // getFileNumbersでエラーが発生した際のフラグ

static volatile bool sd_fatfs_locked = false;
static volatile bool sd_analysis_active = false;
static bool logRecoveryReady = true;
static bool logTempWarning = false;
static bool logTempOpen = false;
static uint32_t savedDataCrc = 0U;
static bool lastRunSaved = false;
static bool lastPrimaryRouteValid = false;
static bool runImuCalibrationValid = false;
static uint16_t runImuCalibrationSamples = 0U;
static uint16_t runImuCalibrationReadErrors = 0U;
// 校正完了後、走行開始時点の値を保持し、後続走行の再校正から分離する。
static uint16_t runLineSensorMax[NUM_SENSORS];
static uint16_t runLineSensorMin[NUM_SENSORS];
static bool runLineSensorCalibrationValid = false;
// 手動走行の開始時点で固定した計画元。停止後の解析番号変更をログへ混入させない。
static int16_t runPrimaryLogNumber = 0;
static int16_t runSlipSourceLogNumber = 0;

static PrimaryRouteValidation primaryRouteValidation = {false, false, false, 0, 0, 0.0F, 1U};

// スキーマ関連のヘルパー宣言。
static void logSendFloat(float value);

static bool readSavedLogNumber(int16_t *outNumber);
static void writeSavedLogNumber(int16_t fileNumber);
static int16_t calcNextLogNumber(void);
static void setLogHeaderStrFPrecision(char *name, float value, unsigned precision);

//====================================//
// ログバッファの内部処理
//====================================//
/////////////////////////////////////////////////////////////////////
// モジュール名 logBufferWriterInit
// 処理概要     3本の整列ログバッファとキュー状態を初期化する
// 引数         writer: ログバッファ状態
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static inline void logBufferWriterInit(
	LogBufferWriter *writer, uint8_t buffers[][LOG_BUFFER_SIZE_BYTES])
{
	writer->buffers = buffers;
	writer->activeBuffer = writer->buffers[0];
	writer->flushBuffer = writer->buffers[1];
	writer->pendingBuffer = writer->buffers[2];
	writer->reservedBuffer = NULL;
	writer->activeLength = 0U;
	writer->flushLength = 0U;
	writer->pendingLength = 0U;
	writer->flushPending = false;
	writer->pendingPending = false;
	writer->overflow = false;
	writer->writeFailed = false;
	writer->recordFailed = false;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logBufferWriterFindFreeBuffer
// 処理概要     書き込み中・書き込み待ちではないバッファを探す
// 引数         writer: ログバッファ状態
// 戻り値       空きバッファ。空きがない場合はNULL
/////////////////////////////////////////////////////////////////////
static inline uint8_t *logBufferWriterFindFreeBuffer(LogBufferWriter *writer)
{
	for (uint32_t i = 0U; i < LOG_BUFFER_COUNT; i++)
	{
		uint8_t *buffer = writer->buffers[i];
		if (buffer == writer->activeBuffer || buffer == writer->reservedBuffer)
		{
			continue;
		}
		if (writer->flushPending && buffer == writer->flushBuffer)
		{
			continue;
		}
		if (writer->pendingPending && buffer == writer->pendingBuffer)
		{
			continue;
		}
		return buffer;
	}
	return NULL;
}

static inline bool logBufferWriterRotateFullBuffer(LogBufferWriter *writer);

/////////////////////////////////////////////////////////////////////
// モジュール名 logBufferWriterBeginRecord
// 処理概要     満杯バッファを切り替え、境界をまたぐレコードの空きを事前確保する
// 引数         writer: ログバッファ状態 / recordLength: レコード長[B]
// 戻り値       true: 書き込み可能 / false: 書き込み失敗
/////////////////////////////////////////////////////////////////////
static inline bool logBufferWriterBeginRecord(LogBufferWriter *writer, uint32_t recordLength)
{
	if (writer->writeFailed || writer->overflow || recordLength == 0U ||
		recordLength > LOG_BUFFER_SIZE_BYTES || writer->activeLength > LOG_BUFFER_SIZE_BYTES)
	{
		return false;
	}

	writer->recordFailed = false;
	writer->reservedBuffer = NULL;
	if (writer->activeLength == LOG_BUFFER_SIZE_BYTES)
	{
		writer->reservedBuffer = logBufferWriterFindFreeBuffer(writer);
		if (writer->reservedBuffer == NULL || !logBufferWriterRotateFullBuffer(writer))
		{
			writer->overflow = true;
			writer->reservedBuffer = NULL;
			return false;
		}
	}

	uint32_t available = LOG_BUFFER_SIZE_BYTES - writer->activeLength;
	if (recordLength > available)
	{
		writer->reservedBuffer = logBufferWriterFindFreeBuffer(writer);
		if (writer->reservedBuffer == NULL)
		{
			writer->overflow = true;
			return false;
		}
	}
	else if (recordLength == available)
	{
		// 満杯時に即時切替できるよう空きがあれば確保する。空きがなくても記録は受け付ける。
		writer->reservedBuffer = logBufferWriterFindFreeBuffer(writer);
	}
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logBufferWriterRotateFullBuffer
// 処理概要     満杯のバッファを待ち行列へ渡して空きバッファへ切り替える
// 引数         writer: ログバッファ状態
// 戻り値       true: 切替成功 / false: 切替失敗
/////////////////////////////////////////////////////////////////////
static inline bool logBufferWriterRotateFullBuffer(LogBufferWriter *writer)
{
	if (writer->reservedBuffer == NULL || writer->activeLength != LOG_BUFFER_SIZE_BYTES ||
		(writer->flushPending && writer->pendingPending))
	{
		writer->overflow = true;
		writer->recordFailed = true;
		return false;
	}

	if (writer->flushPending)
	{
		writer->pendingBuffer = writer->activeBuffer;
		writer->pendingLength = LOG_BUFFER_SIZE_BYTES;
		writer->pendingPending = true;
	}
	else
	{
		writer->flushBuffer = writer->activeBuffer;
		writer->flushLength = LOG_BUFFER_SIZE_BYTES;
		writer->flushPending = true;
	}

	writer->activeBuffer = writer->reservedBuffer;
	writer->reservedBuffer = NULL;
	writer->activeLength = 0U;
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logBufferWriterAppendBytes
// 処理概要     バイト列をバッファ境界で分割して格納する
// 引数         writer: ログバッファ状態 / data: 格納データ / length: データ長[B]
// 戻り値       true: 格納成功 / false: 格納失敗
/////////////////////////////////////////////////////////////////////
static inline bool logBufferWriterAppendBytes(LogBufferWriter *writer, const uint8_t *data, size_t length)
{
	while (length > 0U)
	{
		if (writer->activeLength == LOG_BUFFER_SIZE_BYTES &&
			(writer->reservedBuffer == NULL || !logBufferWriterRotateFullBuffer(writer)))
		{
			writer->recordFailed = true;
			return false;
		}

		uint32_t available = LOG_BUFFER_SIZE_BYTES - writer->activeLength;
		size_t chunkLength = (length < available) ? length : available;
		memcpy(writer->activeBuffer + writer->activeLength, data, chunkLength);
		writer->activeLength += (uint32_t)chunkLength;
		data += chunkLength;
		length -= chunkLength;

		if (writer->activeLength == LOG_BUFFER_SIZE_BYTES && writer->reservedBuffer != NULL &&
			!logBufferWriterRotateFullBuffer(writer))
		{
			writer->recordFailed = true;
			return false;
		}
	}
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logBufferWriterEndRecord
// 処理概要     レコードの格納完了状態を確認する
// 引数         writer: ログバッファ状態
// 戻り値       true: レコード全体を格納 / false: 格納失敗
/////////////////////////////////////////////////////////////////////
static inline bool logBufferWriterEndRecord(LogBufferWriter *writer)
{
	bool complete = writer->reservedBuffer == NULL && !writer->recordFailed;
	if (!complete)
	{
		writer->reservedBuffer = NULL;
		writer->overflow = true;
	}
	writer->recordFailed = false;
	return complete;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logBufferWriterAppendU8
// 処理概要     8bit値をログバッファへ格納する
// 引数         writer: ログバッファ状態 / value: 格納値
// 戻り値       true: 格納成功 / false: 格納失敗
/////////////////////////////////////////////////////////////////////
static inline bool logBufferWriterAppendU8(LogBufferWriter *writer, uint8_t value)
{
	if (writer->activeLength < LOG_BUFFER_SIZE_BYTES)
	{
		writer->activeBuffer[writer->activeLength++] = value;
		return (writer->activeLength != LOG_BUFFER_SIZE_BYTES || writer->reservedBuffer == NULL) ||
			logBufferWriterRotateFullBuffer(writer);
	}
	return logBufferWriterAppendBytes(writer, &value, sizeof(value));
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logBufferWriterAppendU16
// 処理概要     16bit値を従来のビッグエンディアン形式で格納する
// 引数         writer: ログバッファ状態 / value: 格納値
// 戻り値       true: 格納成功 / false: 格納失敗
/////////////////////////////////////////////////////////////////////
static inline bool logBufferWriterAppendU16(LogBufferWriter *writer, uint16_t value)
{
	if (writer->activeLength <= LOG_BUFFER_SIZE_BYTES - 2U)
	{
		writer->activeBuffer[writer->activeLength++] = (uint8_t)(value >> 8);
		writer->activeBuffer[writer->activeLength++] = (uint8_t)value;
		return (writer->activeLength != LOG_BUFFER_SIZE_BYTES || writer->reservedBuffer == NULL) ||
			logBufferWriterRotateFullBuffer(writer);
	}
	uint8_t bytes[2] = {(uint8_t)(value >> 8), (uint8_t)value};
	return logBufferWriterAppendBytes(writer, bytes, sizeof(bytes));
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logBufferWriterAppendU32
// 処理概要     32bit値を従来のビッグエンディアン形式で格納する
// 引数         writer: ログバッファ状態 / value: 格納値
// 戻り値       true: 格納成功 / false: 格納失敗
/////////////////////////////////////////////////////////////////////
static inline bool logBufferWriterAppendU32(LogBufferWriter *writer, uint32_t value)
{
	if (writer->activeLength <= LOG_BUFFER_SIZE_BYTES - 4U)
	{
		writer->activeBuffer[writer->activeLength++] = (uint8_t)(value >> 24);
		writer->activeBuffer[writer->activeLength++] = (uint8_t)(value >> 16);
		writer->activeBuffer[writer->activeLength++] = (uint8_t)(value >> 8);
		writer->activeBuffer[writer->activeLength++] = (uint8_t)value;
		return (writer->activeLength != LOG_BUFFER_SIZE_BYTES || writer->reservedBuffer == NULL) ||
			logBufferWriterRotateFullBuffer(writer);
	}
	uint8_t bytes[4] = {
		(uint8_t)(value >> 24), (uint8_t)(value >> 16), (uint8_t)(value >> 8), (uint8_t)value};
	return logBufferWriterAppendBytes(writer, bytes, sizeof(bytes));
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logBufferWriterWriteSucceeded
// 処理概要     FatFs書き込み結果を確認し、失敗時は待ち行列を破棄する
// 引数         writer: ログバッファ状態 / ioSucceeded: FatFs結果 / expectedLength: 要求長[B] / writtenLength: 書込長[B]
// 戻り値       true: 全量書き込み成功 / false: 書き込み失敗
/////////////////////////////////////////////////////////////////////
static inline bool logBufferWriterWriteSucceeded(
	LogBufferWriter *writer, bool ioSucceeded, uint32_t expectedLength, uint32_t writtenLength)
{
	if (ioSucceeded && writtenLength == expectedLength)
	{
		return true;
	}

	writer->writeFailed = true;
	writer->flushPending = false;
	writer->pendingPending = false;
	writer->flushLength = 0U;
	writer->pendingLength = 0U;
	writer->reservedBuffer = NULL;
	return false;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logBufferWriterCompleteWrite
// 処理概要     書き込み済みバッファを解放して次の待ちバッファを昇格する
// 引数         writer: ログバッファ状態
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static inline void logBufferWriterCompleteWrite(LogBufferWriter *writer)
{
	if (writer->pendingPending)
	{
		writer->flushBuffer = writer->pendingBuffer;
		writer->flushLength = writer->pendingLength;
		writer->pendingPending = false;
		writer->pendingLength = 0U;
		writer->flushPending = true;
	}
	else
	{
		writer->flushPending = false;
		writer->flushLength = 0U;
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logBufferWriterPrepareFinalWrite
// 処理概要     残りの有効データを512 B境界までゼロ埋めする
// 引数         writer: ログバッファ状態
// 戻り値       書き込み長[B]。残データなしは0
/////////////////////////////////////////////////////////////////////
static inline uint32_t logBufferWriterPrepareFinalWrite(LogBufferWriter *writer)
{
	if (writer->activeLength == 0U)
	{
		return 0U;
	}

	uint32_t finalLength =
		(writer->activeLength + LOG_BUFFER_SECTOR_SIZE_BYTES - 1U) & ~(LOG_BUFFER_SECTOR_SIZE_BYTES - 1U);
	memset(writer->activeBuffer + writer->activeLength, 0, finalLength - writer->activeLength);
	return finalLength;
}

void readImuTempCompensation(void)
{
	const char fileName[] = PATH_SETTING "imu_temp.txt";
	char buffer[24] = {0};
	char *end = NULL;
	FIL file;
	UINT bytesRead = 0U;
	uint32_t fileSize = 0U;
	long coeffX1000000 = 0L;
	bool valid = false;

	IMU_SetTempCompensationCoefficient(0);
	if (f_open(&file, fileName, FA_OPEN_EXISTING | FA_READ) == FR_OK)
	{
		fileSize = (uint32_t)f_size(&file);
		if (fileSize > 0U && fileSize < sizeof(buffer) &&
			f_read(&file, buffer, (UINT)fileSize, &bytesRead) == FR_OK &&
			bytesRead == (UINT)fileSize)
		{
			buffer[bytesRead] = '\0';
			coeffX1000000 = strtol(buffer, &end, 10);
			valid = end != buffer && *end == '\0' &&
				coeffX1000000 >= IMU_TEMP_COEFF_MIN_X1000000 &&
				coeffX1000000 <= IMU_TEMP_COEFF_MAX_X1000000;
		}
		f_close(&file);
	}
	if (!valid)
	{
		coeffX1000000 = 0L;
		if (f_open(&file, fileName, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
		{
			f_printf(&file, "%ld", coeffX1000000);
			f_close(&file);
		}
	}
	IMU_SetTempCompensationCoefficient((int32_t)coeffX1000000);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 readAutoRunConfigFile
// 処理概要     SDカードからオートスタート方式設定を読み込む
// 引数         context: 未使用, buffer: 読込先, capacity: 読込容量, length: 読込文字数
// 戻り値       読込成功、欠落、内容不正、またはI/Oエラー
/////////////////////////////////////////////////////////////////////
static AutoRunConfigReadResult readAutoRunConfigFile(void *context, char *buffer, size_t capacity, size_t *length)
{
	FIL file;
	FSIZE_t fileSize;
	UINT bytesRead = 0U;
	FRESULT result;
	FRESULT closeResult;
	(void)context;
	if (buffer == NULL || length == NULL || capacity < 2U)
	{
		return AUTO_RUN_CONFIG_READ_IO_ERROR;
	}
	result = f_open(&file, FILENAME_AUTORUN, FA_OPEN_EXISTING | FA_READ);
	if (result == FR_NO_FILE)
	{
		return AUTO_RUN_CONFIG_READ_MISSING;
	}
	if (result != FR_OK)
	{
		return AUTO_RUN_CONFIG_READ_IO_ERROR;
	}
	fileSize = f_size(&file);
	if (fileSize >= capacity)
	{
		closeResult = f_close(&file);
		return (closeResult == FR_OK) ? AUTO_RUN_CONFIG_READ_INVALID : AUTO_RUN_CONFIG_READ_IO_ERROR;
	}
	result = f_read(&file, buffer, (UINT)fileSize, &bytesRead);
	closeResult = f_close(&file);
	if (result != FR_OK || bytesRead != (UINT)fileSize || closeResult != FR_OK)
	{
		return AUTO_RUN_CONFIG_READ_IO_ERROR;
	}
	buffer[bytesRead] = '\0';
	*length = (size_t)bytesRead;
	return AUTO_RUN_CONFIG_READ_SUCCESS;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 writeAutoRunConfigFile
// 処理概要     SDカード上のオートスタート方式設定を全体置換で保存する
// 引数         context: 未使用, text: 保存内容, length: 保存文字数
// 戻り値       true: 保存成功 false: 書込または同期エラー
/////////////////////////////////////////////////////////////////////
static bool writeAutoRunConfigFile(void *context, const char *text, size_t length)
{
	FIL file;
	UINT bytesWritten = 0U;
	FRESULT result;
	(void)context;
	if (text == NULL || length > UINT_MAX || f_open(&file, FILENAME_AUTORUN, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
	{
		return false;
	}
	result = f_write(&file, text, (UINT)length, &bytesWritten);
	if (result == FR_OK && bytesWritten == (UINT)length)
	{
		result = f_sync(&file);
	}
	FRESULT closeResult = f_close(&file);
	return result == FR_OK && bytesWritten == (UINT)length && closeResult == FR_OK;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 readAutoRunSettings
// 処理概要     オートスタート方式設定を読み込み、欠落・不正時は既定順へ修復する
// 引数         なし
// 戻り値       設定利用可能、修復失敗、またはSD読込エラー
/////////////////////////////////////////////////////////////////////
AutoRunConfigLoadResult readAutoRunSettings(void)
{
	return autoRunLoadConfig(&autoRunState, readAutoRunConfigFile, writeAutoRunConfigFile, NULL);
}


bool sd_fatfs_lock(uint32_t timeout_ms)
{
	if (timeout_ms == 0)
	{
		return sd_fatfs_try_lock();
	}

	uint32_t start = HAL_GetTick();
	while (sd_fatfs_locked)
	{
		if ((HAL_GetTick() - start) > timeout_ms)
		{
			return false;
		}
	}
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	if (sd_fatfs_locked)
	{
		__set_PRIMASK(primask);
		return false;
	}
	sd_fatfs_locked = true;
	__set_PRIMASK(primask);
	return true;
}

void sd_fatfs_unlock(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	sd_fatfs_locked = false;
	__set_PRIMASK(primask);
}

bool sd_fatfs_is_locked(void)
{
	return sd_fatfs_locked;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 sd_set_analysis_active
// 処理概要     解析中フラグを設定する
// 引数         active: true=解析中 / false=解析終了
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void sd_set_analysis_active(bool active)
{
	sd_analysis_active = active;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 sd_is_analysis_active
// 処理概要     解析中フラグを取得する
// 引数         なし
// 戻り値       true=解析中 / false=解析中ではない
/////////////////////////////////////////////////////////////////////
bool sd_is_analysis_active(void)
{
	return sd_analysis_active;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 sd_fatfs_try_lock
// 処理概要     FatFsロックをノンブロッキングで取得する
// 引数         なし
// 戻り値       true=取得成功 / false=取得失敗
/////////////////////////////////////////////////////////////////////
bool sd_fatfs_try_lock(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	if (sd_fatfs_locked)
	{
		__set_PRIMASK(primask);
		return false;
	}
	sd_fatfs_locked = true;
	__set_PRIMASK(primask);
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 insertSD
// 処理概要     SDカード挿入状況確認
// 引数         なし
// 戻り値       true:挿入されている false:未挿入
/////////////////////////////////////////////////////////////////////
bool insertSD(void)
{
	if (HAL_GPIO_ReadPin(SD_SW_GPIO_Port, SD_SW_Pin))
	{
		return true;
	}
	else
	{
		return false;
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 initMicroSD
// 処理概要     SDカードの初期化
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
bool initMicroSD(void)
{
	FATFS *pfs;
	FRESULT fresult;		// f_write status
	DWORD fre_clust;
	uint32_t total, free_space;

	// SDcardをマウント
	fresult = sd_mount();
	if (fresult == FR_OK)
	{
		// マウント成功
		initMSD = true;
		printf("SD CARD mounted successfully...\r\n");

		// 空き容量を計算
		fresult = f_getfree("", &fre_clust, &pfs);	// cluster size
		if (fresult != FR_OK)
		{
			// 空き容量取得に失敗した場合はエラーメッセージを出力して終了
			initMSD = false;
			printf("error in getting SD CARD free space...\r\n");
			return false;
		}
		total = (uint32_t)((uint64_t)(pfs->n_fatent - 2U) * pfs->csize / 2U); // total capacity
		printf("SD_SIZE: \t%lu\r\n", total);
		free_space = (uint32_t)((uint64_t)fre_clust * pfs->csize / 2U); // empty capacity
		printf("SD free space: \t%lu\r\n", free_space);

		// ディレクトリを作成
		createDir("setting");
		createDir("plot");

		return true;
	}
	else
	{
		// マウント失敗
		initMSD = false;
		printf("error in mounting SD CARD...\r\n");
		return false;
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 readSavedLogNumber
// 処理概要     保存済みログ番号ファイルを読み込み、取得できた番号を返す
// 引数         outNumber: 取得先ポインタ
// 戻り値       true: 読み込み成功 / false: ファイル無し・不正値・エラー
/////////////////////////////////////////////////////////////////////
static bool readSavedLogNumber(int16_t *outNumber)
{
	FRESULT fresult;
	FIL fil;
	char fileName[32] = PATH_SETTING;
	char buf[16] = {0};
	int value = 0;

	// 引数が無効なら失敗扱い
	if (outNumber == NULL)
	{
		return false;
	}

	// ログ番号保存ファイル名を組み立て（読込）
	strncat(fileName, FILENAME_LOGNUMBER, sizeof(fileName) - strlen(fileName) - 1);
	strncat(fileName, ".txt", sizeof(fileName) - strlen(fileName) - 1);

	// ファイルが無ければ失敗
	fresult = f_open(&fil, fileName, FA_OPEN_EXISTING | FA_READ);
	if (fresult != FR_OK)
	{
		return false;
	}

	// 1行読み込み
	if (f_gets(buf, (int)sizeof(buf), &fil) == NULL)
	{
		f_close(&fil);
		return false;
	}

	// 数値に変換。0以下は無効
	if (sscanf(buf, "%d", &value) != 1 || value <= 0)
	{
		f_close(&fil);
		return false;
	}

	// 正常値を返す
	*outNumber = (int16_t)value;
	f_close(&fil);
	return true;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 writeSavedLogNumber
// 処理概要     保存ログ番号ファイルへ番号を書き込む（上書き保存）
// 引数         fileNumber: 保存するログ番号
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void writeSavedLogNumber(int16_t fileNumber)
{
	FRESULT fresult;
	FIL fil;
	char fileName[32] = PATH_SETTING;

	// ログ番号保存ファイル名を組み立て（保存）
	strncat(fileName, FILENAME_LOGNUMBER, sizeof(fileName) - strlen(fileName) - 1);
	strncat(fileName, ".txt", sizeof(fileName) - strlen(fileName) - 1);

	fresult = f_open(&fil, fileName, FA_OPEN_ALWAYS | FA_WRITE);
	if (fresult == FR_OK)
	{
		// 先頭から書き込み、古い残りを削除
		f_lseek(&fil, 0);
		// 5桁ゼロ埋めで保存
		f_printf(&fil, "%05d", fileNumber);
		f_truncate(&fil);
	}
	f_close(&fil);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 calcNextLogNumber
// 処理概要     次のログ番号を決定する（保存値優先→SD内最大→1）
// 引数         なし
// 戻り値       次のログ番号
/////////////////////////////////////////////////////////////////////
static int16_t calcNextLogNumber(void)
{
	DIR dir;
	FILINFO info;
	int16_t saved = 0;
	uint32_t maximum = logFileNumber;
	if (readSavedLogNumber(&saved) && saved > 0 && (uint32_t)saved > maximum) maximum = (uint32_t)saved;
	if (f_opendir(&dir, "/") != FR_OK) return 0;
	FRESULT result;
	do {
		result = f_readdir(&dir, &info);
		uint16_t number;
		if (result == FR_OK && logBinaryReservedName(info.fname, &number) && number > maximum) maximum = number;
	} while (result == FR_OK && info.fname[0] != '\0');
	FRESULT closeResult = f_closedir(&dir);
	if (result != FR_OK || closeResult != FR_OK || maximum >= INT16_MAX) return 0;
	return (int16_t)(maximum + 1U);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 getLastLogNumber
// 処理概要     バイナリ保存用に予約した直前ログ番号を取得する
// 引数         なし
// 戻り値       直前ログ番号。未採番の場合は0
/////////////////////////////////////////////////////////////////////
int16_t getLastLogNumber(void)
{
	return (logFileNumber <= INT16_MAX) ? (int16_t)logFileNumber : 0;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 logLastPrimaryRouteValid
// 処理概要     直前に保存した一次走行ログの経路採用可否を返す
// 引数         なし
// 戻り値       true:採用可能 false:保存失敗または検証不成立
/////////////////////////////////////////////////////////////////////
bool logLastPrimaryRouteValid(void)
{
	return lastRunSaved && lastPrimaryRouteValid;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 logLastPrimaryRouteReason
// 処理概要     直前の一次走行ログの経路不採用理由を返す
// 引数         なし
// 戻り値       経路検証の理由コード
/////////////////////////////////////////////////////////////////////
uint8_t logLastPrimaryRouteReason(void)
{
	return primaryRouteValidation.reason;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 logBuildMetadata
// 処理概要     停止時点の走行メタデータを生成する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void logBuildMetadata(void)
{
	columnTitle[0] = 0; // バッファを安全に初期化


	updateBatteryVoltage(); // ログヘッダへ停止時点の電圧を残す
	updateImuTempEndTemperature(); // ログ終了時の温度を取得

	setLogHeaderStr("binaryLogNumber", logFileNumber);
	setLogHeaderStr("binaryDataCrc", (int32_t)savedDataCrc);
	setLogHeaderStr("binarySchema", (int32_t)logStorageSchemaCrc());
	setLogHeaderStr("logDerivedVersion", LOG_BINARY_DERIVED_VERSION);
	setLogHeaderStrS("logXYDistance", "raw_wheel_average");
	// 1行目: 走行条件と校正値
	setLogHeaderStrS("fwVersion", FW_VERSION);
	setLogHeaderStrS("gitCommit", GIT_COMMIT);
	setLogHeaderStrS("buildDate", BUILD_DATE);
	setLogHeaderStrS("buildTime", BUILD_TIME);
	setLogHeaderStrS("branch", GIT_BRANCH);
	// 正式採用したV2の方式・閾値と、SLIP計画が使用する統計の意味を保存する。
	setLogHeaderStr("slipDetectorVersion", SLIP_V2_VERSION);
    setLogHeaderStrS("slipLongPolicy", SLIP_V2_LONG_POLICY);
	setLogHeaderStrS("slipDetectorMode", "ACTIVE_V2");
	setLogHeaderStrS("slipLatPolicy", SLIP_V2_LAT_POLICY);
	setLogHeaderStr("slipPlannerVersion", 2);
	setLogHeaderStr("slipLatCandidateVersion", SLIP_LAT_CANDIDATE_VERSION);
	setLogHeaderStrFPrecision("slipLatCandidateOn_mps2", SLIP_LAT_CANDIDATE_ON, 9U);
	setLogHeaderStrFPrecision("slipLatCandidateOff_mps2", SLIP_LAT_CANDIDATE_OFF, 9U);
	setLogHeaderStrS("slipLatCandidateFormula", "lpf_neutral_minus_windowed_tangent_X");
	setLogHeaderStrFPrecision("slipLatCandidateLpf", SLIP_LAT_CANDIDATE_LPF_COEF, 6U);
	setLogHeaderStrS("slipLatCandidateLpfInit", "first_valid_neutral_reset_on_invalid");
	setLogHeaderStrS("slipLatCandidateAxes", "Xright_Yforward_CWpositive_unverified");
	setLogHeaderStrFPrecision("slipLatOffsetX_m", IMU_OFFSET_X_M, 8U);
	setLogHeaderStrFPrecision("slipLatOffsetY_m", IMU_OFFSET_Y_M, 8U);
	setLogHeaderStrFPrecision("slipLatAlphaLpf", IMU_ALPHA_LPF_COEF, 6U);
	setLogHeaderStrS("slipLatAlphaUnit", "rad/s2");
	setLogHeaderStrS("slipLatTangentUnit", "m/s2");
	setLogHeaderStr("slipFastMs", SLIP_V2_FAST_MS);
	setLogHeaderStr("slipSlowMs", SLIP_V2_SLOW_MS);
	setLogHeaderStr("slipFilterMs", SLIP_V2_FILTER_MS);
	setLogHeaderStr("slipAccelDelayMs", SLIP_V2_ACCEL_DELAY_MS);
	setLogHeaderStr("slipGyroDelayMs", SLIP_V2_GYRO_DELAY_MS);
	setLogHeaderStr("slipYawEncDelayMs", SLIP_V2_ENCODER_YAW_DELAY_MS);
	setLogHeaderStr("slipOnMs", SLIP_V2_ON_MS);
	setLogHeaderStr("slipOffMs", SLIP_V2_OFF_MS);
	setLogHeaderStrFPrecision("slipLongOn_mps2", SLIP_V2_LONG_ON, 9U);
	setLogHeaderStrFPrecision("slipLongOff_mps2", SLIP_V2_LONG_OFF, 9U);
	setLogHeaderStrFPrecision("slipLatOn_mps2", SLIP_LAT_CANDIDATE_ON, 9U);
	setLogHeaderStrFPrecision("slipLatOff_mps2", SLIP_LAT_CANDIDATE_OFF, 9U);
	setLogHeaderStrFPrecision("slipYawOn_dps", SLIP_V2_YAW_ON_DPS, 9U);
	setLogHeaderStrFPrecision("slipYawOff_dps", SLIP_V2_YAW_OFF_DPS, 9U);
	setLogHeaderStrFPrecision("slipYawWidth_m", SLIP_V2_YAW_WIDTH_M, 9U);
	setLogHeaderStrFPrecision("slipYawOffset_mps", SLIP_V2_YAW_OFFSET_MPS, 9U);
	setLogHeaderStrFPrecision("slipYawScaleDiff", SLIP_V2_YAW_SCALE_DIFF, 9U);
	setLogHeaderStrS("slipYawCalibration", "13074-13083_empirical");
	setLogHeaderStrFPrecision("gyroScaleCoeff", COEFF_DPD, 6U);
	setLogHeaderStr("imuTempCalibrationValid", imuTempCalibrationValid ? 1 : 0);
	setLogHeaderStrFPrecision("imuTempCalibrationStart_C", imuTempCalibrationStart_C, 3U);
	setLogHeaderStrFPrecision("imuTempCalibration_C", imuTempCalibration_C, 3U);
	setLogHeaderStrFPrecision("imuTempCalibrationEnd_C", imuTempCalibrationEnd_C, 3U);
	setLogHeaderStr("imuTempCalibrationSamples", imuTempCalibrationSamples);
	setLogHeaderStr("imuTempCalibrationReadErrors", imuTempCalibrationReadErrors);
	setLogHeaderStrFPrecision("imuGyroOffsetZ_dps", angleOffset[2], 3U);
	setLogHeaderStr("imuTempCompEnabled", imuTempCorrectionEnabled ? 1 : 0);
	setLogHeaderStrFPrecision("imuTempCoeff_dpsPerC", imuTempCoeff_dpsPerC, 6U);
	setLogHeaderStrFPrecision("imuTempEnd_C", imuTempEnd_C, 3U);
	setLogHeaderStr("encoderPulsePerMeter", PULSE_METER);
	setLogHeaderStr("logDistanceTargetMm", (int32_t)LOG_DISTANCE_MM);
	setLogHeaderStr("pathSourceFormatVersion", PATH_SOURCE_FORMAT_VERSION);
	setLogHeaderStr("closureValid", primaryRouteValidation.closureValid ? 1 : 0);
	setLogHeaderStr("closureReason", primaryRouteValidation.reason);
	setLogHeaderStr("goalMarkerOnsetValid", primaryRouteValidation.goalMarkerOnsetValid ? 1 : 0);
	setLogHeaderStr("goalMarkerOnset_p", primaryRouteValidation.goalMarkerOnset_p);
	setLogHeaderStrFPrecision("goalMarkerX_mm", primaryRouteValidation.goalMarkerX_mm, 2U);
	setLogHeaderStr("logExpectedRows", cntSend);
	setLogHeaderStr("imuCalibrationValid", runImuCalibrationValid ? 1 : 0);
	setLogHeaderStr("imuCalibrationSamples", runImuCalibrationSamples);
	setLogHeaderStr("imuCalibrationReadErrors", runImuCalibrationReadErrors);
	setLogHeaderStr("lineSensorCalibrationValid", runLineSensorCalibrationValid ? 1 : 0);
	for (uint8_t i = 0U; i < NUM_SENSORS; i++)
	{
		char name[24];
		snprintf(name, sizeof(name), "lSensorMax%u", (unsigned)i);
		setLogHeaderStr(name, runLineSensorMax[i]);
		snprintf(name, sizeof(name), "lSensorMin%u", (unsigned)i);
		setLogHeaderStr(name, runLineSensorMin[i]);
	}
	setLogHeaderStr("distanceScaleVerified", primaryRouteValidation.distanceScaleVerified ? 1 : 0);
	setLogHeaderStr("distanceScalePulsePerMeter", PULSE_METER);
	setLogHeaderStr("distanceScaleError_p", primaryRouteValidation.distanceScaleError_p);
	// 制御パラメータ
	setLogHeaderStrF("batteryVoltage_V", batteryVoltage_V);
	setLogHeaderStrF("optimalTrace", optimalTrace);
	setLogHeaderStrF("autoStart", autoStart);
	setLogHeaderStrF("emcStop", emcStop);
	if (autoRunState.currentPlanValid && autoStart > 0U)
	{
		setLogHeaderStr("autoRunNumber", autoRunState.currentRunNumber);
		setLogHeaderStrS("requestedMode", autoRunModeName(autoRunState.currentMode));
		setLogHeaderStr("primaryLogNumber", autoRunPrimaryLogForHeader(&autoRunState, (int16_t)logFileNumber));
		setLogHeaderStr("slipSourceLogNumber", autoRunState.slipSourceLogNumber);
	}
	else
	{
		setLogHeaderStr("autoRunNumber", 0);
		setLogHeaderStrS("requestedMode", "MANUAL");
		setLogHeaderStr("primaryLogNumber", runPrimaryLogNumber);
		setLogHeaderStr("slipSourceLogNumber", runSlipSourceLogNumber);
	}
	// ゴール誤検出調査用。行ログが終了直前で途切れても累積値を確認できる。
	setLogHeaderStr("sgMarkerAtLogEnd", (int32_t)SGmarker);
	setLogHeaderStr("encRightMarkerAtLogEnd_p", encRightMarker);
	setLogHeaderStr("routeControllerVersion", PATH_ROUTE_CONTROLLER_VERSION);
	setLogHeaderStr("routeSourceLog", pathRouteSourceLog());
	setLogHeaderStr("shortcutLevel", pathRouteShortcutLevel());

    // 共通定義の出力対象だけを保存順で出力し、既存15項目の名前と書式を維持する。
#define SPEED_PARAM_LOG_0(member)
#define SPEED_PARAM_LOG_1(member) setLogHeaderStrF("tgtParam." #member, tgtParam.member);
#define SPEED_PARAM_LOG(member, initial, label, step, minimum, maximum, unit, metadata) \
    SPEED_PARAM_LOG_##metadata(member)
    SPEED_PARAM_FIELDS(SPEED_PARAM_LOG)
#undef SPEED_PARAM_LOG
#undef SPEED_PARAM_LOG_1
#undef SPEED_PARAM_LOG_0

	setLogHeaderStrF("lineTraceCtrl.kp", lineTraceCtrl.kp);
	setLogHeaderStrF("lineTraceCtrl.ki", lineTraceCtrl.ki);
	setLogHeaderStrF("lineTraceCtrl.kd", lineTraceCtrl.kd);
	setLogHeaderStrF("lineTraceOmegaFBCtrl.kp", lineTraceOmegaFBCtrl.kp);
	setLogHeaderStrF("lineTraceOmegaFBCtrl.ki", lineTraceOmegaFBCtrl.ki);
	setLogHeaderStrF("lineTraceOmegaFBCtrl.kd", lineTraceOmegaFBCtrl.kd);
	setLogHeaderStrF("veloCtrl.kp", veloCtrl.kp);
	setLogHeaderStrF("veloCtrl.ki", veloCtrl.ki);
	setLogHeaderStrF("veloCtrl.kd", veloCtrl.kd);
	setLogHeaderStrF("speedFeedForwardGain", speedFeedForwardGain);
	setLogHeaderStrF("yawRateCtrl.kp", yawRateCtrl.kp);
	setLogHeaderStrF("yawRateCtrl.ki", yawRateCtrl.ki);
	setLogHeaderStrF("yawRateCtrl.kd", yawRateCtrl.kd);
    strncat((char *)columnTitle, "\n", sizeof(columnTitle) - strlen((char *)columnTitle) - 1); // バッファサイズを指定して安全に改行を追加
}
/////////////////////////////////////////////////////////////////////
// モジュール名 initLog
// 処理概要     バイナリ保存用のファイルを作成
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void initLog(void)
{
	FRESULT fresult;		// f_write status
	lastRunSaved = false;
	lastPrimaryRouteValid = false;
	runImuCalibrationValid = IMU_CalibrationReady();
	runImuCalibrationSamples = IMU_CalibrationSamples();
	runImuCalibrationReadErrors = IMU_CalibrationReadErrors();
	memcpy(runLineSensorMax, lSensorMax, sizeof(runLineSensorMax));
	memcpy(runLineSensorMin, lSensorMin, sizeof(runLineSensorMin));
	runLineSensorCalibrationValid = isLineSensorCalibrationValid();
	runPrimaryLogNumber = 0;
	runSlipSourceLogNumber = 0;
	if (optimalTrace == BOOST_DISTANCE && runMemoryOwner == RUN_MEMORY_DISTANCE &&
		numPPADarry > 0 && numPPADarry <= OPT_BUFF_SIZE && analyzedNumber > 0)
	{
		// SLIPも同じ距離計画を使い、analyzedNumberは元の一次ログ番号を保持する。
		runPrimaryLogNumber = analyzedNumber;
		runSlipSourceLogNumber = analyzedSlipSourceNumber;
	}
	else if ((optimalTrace == BOOST_PATH_REPLAY || optimalTrace == BOOST_SHORTCUT) &&
		runMemoryOwner == RUN_MEMORY_PATH && pathRouteCount() > 1U && pathRouteSourceLog() > 0)
	{
		runPrimaryLogNumber = pathRouteSourceLog();
	}
	primaryRouteValidation = (PrimaryRouteValidation){false, false, false, 0, 0, 0.0F, 1U};
	// 走行ごとの記録件数を初期化する
	cntSend = 0;
	char fileName[20];
	int16_t next = calcNextLogNumber();
	if (!logRecoveryReady || next <= 0) { logRecoveryReady = false; return; }
	logFileNumber = (uint16_t)next;
	logBinaryName(fileName, sizeof(fileName), logFileNumber, "tmp");
	fresult = f_open(&fil_W, fileName, FA_CREATE_NEW | FA_WRITE | FA_READ);
	logTempOpen = (fresult == FR_OK);
	if (fresult != FR_OK)
	{
		logRecoveryReady = false;
		printf("error opening log file: %d\r\n", fresult); // エラー内容を出力
		// ファイルオープンに失敗した場合はmicroSDを使用不可とする
		return;          // ログ初期化を中止
	}
	logBufferWriterInit(&logBufferWriter, logBuffers);
#if _USE_EXPAND
	/* Best-effort contiguous preallocation to reduce FAT updates during logging. */
	fresult = f_expand(&fil_W, (FSIZE_t)LOG_TEMP_PREALLOC_BYTES, 1);
	if (fresult != FR_OK)
	{
		printf("f_expand failed: %d\r\n", fresult);
	}
#endif
	memset(logBuffers[0], 0, LOG_BINARY_HEADER_BYTES);
	UINT headerWritten = 0U;
	fresult = f_write(&fil_W, logBuffers[0], LOG_BINARY_HEADER_BYTES, &headerWritten);
	if (fresult == FR_OK && headerWritten != LOG_BINARY_HEADER_BYTES) fresult = FR_DISK_ERR;
	if (fresult == FR_OK) fresult = f_lseek(&fil_W, LOG_BINARY_HEADER_BYTES);
	if (fresult != FR_OK)
	{
		printf("initLog header/seek error: %d\r\n", fresult);
		f_close(&fil_W);
		logTempOpen = false;
		logRecoveryReady = false;
		logBufferWriter.writeFailed = true;
		return;
	}
	dbg_overflow = 0;
	markerOverflow = false;
	logOverflow = false;				// ログバッファ状態フラグをリセット
}
/////////////////////////////////////////////////////////////////////
// モジュール名 writeLogBufferPuts
// 処理概要     保存する変数をバッファに転送する
// 引数         c:8bit変数の数s:16bit変数の数i:32bit変数の数f:float変数の数
// 戻り値       true: バッファへ1記録保存成功、false: 未保存
/////////////////////////////////////////////////////////////////////
bool writeLogBufferPuts(void)
{
	if (modeLOG)
	{
		if (sd_is_analysis_active())
		{
			return false;
		}
		if (logBufferWriter.writeFailed || logBufferWriter.overflow || cntSend == UINT16_MAX)
		{
			if (cntSend == UINT16_MAX) logOverflow = true;
			return false;
		}
		// スキーマから固定レコードサイズを算出。
		if (!logBufferWriterBeginRecord(&logBufferWriter, (uint32_t)LOG_RECORD_SIZE_BYTES))
		{
			logOverflow = true;
			dbg_overflow++;
			return false;
		}

		// スキーマ順でバイナリ書き込みを展開。
#define LOG_SEND_U8(value) send8bit((uint8_t)(value))
#define LOG_SEND_U16(value) send16bit((uint16_t)(value))
#define LOG_SEND_S16(value) send16bit((uint16_t)(int16_t)(value))
#define LOG_SEND_U32(value) send32bit((uint32_t)(value))
#define LOG_SEND_S32(value) send32bit((uint32_t)(int32_t)(value))
#define LOG_SEND_F32(value) logSendFloat((float)(value))
#define LOG_SEND_FIELD(type, name, fmt, expr) LOG_SEND_##type(expr);
#define LOG_SEND_SKIP(type, name, fmt, expr)
		LOG_FIELD_LIST(LOG_SEND_FIELD, LOG_SEND_SKIP)
#undef LOG_SEND_FIELD
#undef LOG_SEND_SKIP
#undef LOG_SEND_U8
#undef LOG_SEND_U16
#undef LOG_SEND_S16
#undef LOG_SEND_U32
#undef LOG_SEND_S32
#undef LOG_SEND_F32

		if (logBufferWriterEndRecord(&logBufferWriter))
		{
			cntSend++;
			return true;
		}
		else
		{
			logOverflow = true;
			dbg_overflow++;
		}
	}
	return false;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 writeLogPuts
// 処理概要     バッファをSDカードに転送する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void writeLogPuts(void)
{
	FRESULT fresult;		// f_write status
	UINT writtenlog = 0;

	if (sd_is_analysis_active())
	{
		return; // avoid writes during analysis
	}

	if (sd_fatfs_is_locked())
	{
		return; // skip while SD is locked
	}
	if (!modeLOG && !logBufferWriter.flushPending)
	{
		return; // skip if no pending write
	}

	if (logBufferWriter.flushPending)
	{
		if (!sd_fatfs_try_lock())
		{
			return;
		}
		uint32_t writeLength = logBufferWriter.flushLength;
		fresult = f_write(&fil_W, logBufferWriter.flushBuffer, (UINT)writeLength, &writtenlog);
		if (fresult != FR_OK || writtenlog != (UINT)writeLength)
		{
			uint32_t primask = __get_PRIMASK();
			__disable_irq();
			(void)logBufferWriterWriteSucceeded(&logBufferWriter,
				fresult == FR_OK, writeLength, (uint32_t)writtenlog);
			__set_PRIMASK(primask);
			sd_fatfs_unlock();
			return;
		}

		{
			uint32_t primask = __get_PRIMASK();
			__disable_irq();
			logBufferWriterCompleteWrite(&logBufferWriter);
			__set_PRIMASK(primask);
		}
		sd_fatfs_unlock();
	}
}


/////////////////////////////////////////////////////////////////////
// モジュール名 endTempFile
// 処理概要     一時ファイル終了処理
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void endTempFile(void)
{
	if (logTempOpen) { f_close(&fil_W); logTempOpen = false; }
}
/////////////////////////////////////////////////////////////////////
// モジュール名 endLog
// 処理概要     バイナリを確定保存し、一次経路の有効性を検証する
// 引数         なし
// 戻り値       true: バイナリ保存完了 false: 保存失敗
/////////////////////////////////////////////////////////////////////
bool endLog(void)
{
	lastRunSaved = false;
	lastPrimaryRouteValid = false;
	modeLOG = false;
	logRecoveryReady = false;
	if (!logTempOpen) return false;
	FRESULT fresult;
	UINT writtenlog;
	while (logBufferWriter.flushPending || logBufferWriter.pendingPending) // 確定処理前に保留中の書込を完了する
	{
		writeLogPuts();
	}
	if (logBufferWriter.writeFailed || logOverflow || markerOverflow)
	{
		f_close(&fil_W);
		logTempOpen = false;
		return false;
	}

	// キュー排出後から確定完了までは、停止中I/Oと共有バッファを占有する。
	if (!sd_fatfs_lock(500U)) return false;
	uint32_t finalWriteLength = logBufferWriterPrepareFinalWrite(&logBufferWriter);
	if (finalWriteLength > 0U)
	{
		fresult = f_write(&fil_W, logBufferWriter.activeBuffer, (UINT)finalWriteLength, &writtenlog);
		if (fresult != FR_OK || writtenlog != (UINT)finalWriteLength)
		{
			(void)logBufferWriterWriteSucceeded(&logBufferWriter,
				fresult == FR_OK, finalWriteLength, (uint32_t)writtenlog);
			printf("f_write error in endLog: %d (%lu/%lu)\r\n", fresult,
				(unsigned long)writtenlog, (unsigned long)finalWriteLength);
			f_close(&fil_W);
			logTempOpen = false;
			sd_fatfs_unlock();
			return false;
		}
	}

	bool success = false;
	LogBinaryHeader header = {logFileNumber, LOG_SCHEMA_PROFILE_LIGHT, LOG_RECORD_SIZE_BYTES,
		cntSend, 0U, 0U, 0U, logStorageSchemaCrc(), sizeof(logSchemaText) - 1U,
        LOG_BINARY_COLUMNS, LOG_BINARY_DERIVED_VERSION};
	FSIZE_t dataEnd = LOG_BINARY_HEADER_BYTES + (FSIZE_t)cntSend * LOG_SIZE;
	const LogSaveCheck saved = {logBufferWriter.writeFailed, runImuCalibrationValid,
		runImuCalibrationSamples, runImuCalibrationReadErrors};
	if (f_lseek(&fil_W, dataEnd) != FR_OK || f_truncate(&fil_W) != FR_OK ||
		f_sync(&fil_W) != FR_OK || !logStorageScanRecords(&fil_W, cntSend, &saved, &primaryRouteValidation, &savedDataCrc)) goto finish;
	logBuildMetadata();
	header.metadataBytes = (uint32_t)strlen(columnTitle);
	header.dataCrc = savedDataCrc;
	header.metadataCrc = ~logBinaryCrcUpdate(UINT32_MAX, (const uint8_t *)columnTitle, header.metadataBytes);
	if (f_lseek(&fil_W, dataEnd) != FR_OK ||
        f_write(&fil_W, logSchemaText, header.schemaBytes, &writtenlog) != FR_OK || writtenlog != header.schemaBytes ||
		f_write(&fil_W, columnTitle, header.metadataBytes, &writtenlog) != FR_OK || writtenlog != header.metadataBytes) goto finish;
	logBinaryEncodeHeader(logBuffers[0], &header);
	if (f_lseek(&fil_W, 0U) != FR_OK ||
		f_write(&fil_W, logBuffers[0], LOG_BINARY_HEADER_BYTES, &writtenlog) != FR_OK || writtenlog != LOG_BINARY_HEADER_BYTES ||
		f_sync(&fil_W) != FR_OK) goto finish;
	success = true;
finish:
	if (f_close(&fil_W) != FR_OK) success = false;
	logTempOpen = false;
	char temporary[20], binary[20];
	logBinaryName(temporary, sizeof(temporary), logFileNumber, "tmp");
	logBinaryName(binary, sizeof(binary), logFileNumber, "bin");
	if (success && f_rename(temporary, binary) != FR_OK) success = false;
	if (success) {
		writeSavedLogNumber((int16_t)logFileNumber);
		lastPrimaryRouteValid = optimalTrace == BOOST_NONE && primaryRouteValidation.closureValid;
		lastRunSaved = true;
		logRecoveryReady = true;
		cntSend = 0;
	}
	sd_fatfs_unlock();
	return success;
}


/////////////////////////////////////////////////////////////////////
// モジュール名 logCheckStoredHeaders
// 処理概要     保存済みbinのヘッダを検証し未確定tmpの存在を確認する
// 引数         なし
// 戻り値       true: 列挙成功 false: 不正ログまたはI/Oエラー
/////////////////////////////////////////////////////////////////////
static bool logCheckStoredHeaders(void)
{
    DIR dir;
    FILINFO info;
    if (f_opendir(&dir, "/") != FR_OK) return false;
    bool success = true;
    FRESULT result;
    do {
        result = f_readdir(&dir, &info);
        if (result != FR_OK) { success = false; break; }
        uint16_t number;
        if (logBinaryParseName(info.fname, "tmp", &number)) logTempWarning = true;
        if (!logBinaryParseName(info.fname, "bin", &number)) continue;
        FIL file;
        LogBinaryHeader header;
        if (f_open(&file, info.fname, FA_READ | FA_OPEN_EXISTING) != FR_OK) { success = false; break; }
        bool valid = logStorageReadContainerHeader(&file, number, &header);
        if (f_close(&file) != FR_OK) valid = false;
        if (!valid) { success = false; break; }
    } while (info.fname[0] != '\0');
    if (f_closedir(&dir) != FR_OK) success = false;
    return success;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logPrepareBinaryStorage
// 処理概要     起動時にbinのヘッダ・実長と未確定tmpを確認し、原本を保持する
// 引数         なし
// 戻り値       true: 保存開始可能 false: I/Oまたはヘッダ不正
/////////////////////////////////////////////////////////////////////
bool logPrepareBinaryStorage(void)
{
    if (modeLOG || logTempOpen || logBufferWriter.flushPending || logBufferWriter.pendingPending ||
        !sd_fatfs_lock(500U)) { logRecoveryReady = false; return false; }
    logTempWarning = false;
    bool result = logCheckStoredHeaders();
    sd_fatfs_unlock();
    logRecoveryReady = result;
    return result;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logRecoveryIsReady
// 処理概要     binの保存・初期確認に成功し新規走行を開始可能か取得する
// 引数         なし
// 戻り値       true: 開始可能 false: 保存・復旧失敗
/////////////////////////////////////////////////////////////////////
bool logRecoveryIsReady(void) { return logRecoveryReady; }

/////////////////////////////////////////////////////////////////////
// モジュール名 logHasIncompleteTemp
// 処理概要     起動時に未確定ログを検出したか取得する
// 引数         なし
// 戻り値       true: 未確定ログあり false: なし
/////////////////////////////////////////////////////////////////////
bool logHasIncompleteTemp(void) { return logTempWarning; }

/////////////////////////////////////////////////////////////////////
// モジュール名 getFileNumbers
// 処理概要     ファイル名から番号を取得し配列に格納する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
int16_t getFileNumbers(void)
{
	DIR dir;		// Directory
	FILINFO fno;	// File Info
	FRESULT fresult;		// f_write status


	for(uint16_t j=0;j<FILENUMBER_NUM;j++){
		fileNumbers[j] = 0; // 配列を初期化
	}

    fresult = f_opendir(&dir, "/"); // directory open
	if (fresult == FR_OK)
	{
		endFileIndex = 0; // 最終インデックスを初期化
		do
		{
			fresult = f_readdir(&dir, &fno);
			if (fresult != FR_OK)
			{
				// ディレクトリ読み込みに失敗した場合はループを抜けてフラグを設定
				getFileNumbersError = true;
				printf("f_readdir error: %d\r\n", fresult); // エラー内容を出力
				break;
			}
			uint16_t binaryNumber;
			if (logBinaryParseName(fno.fname, "bin", &binaryNumber) && endFileIndex < FILENUMBER_NUM)
			{
				// 機体から解析できる現行binだけを選択対象とする。
				fileNumbers[endFileIndex] = (int16_t)binaryNumber;
				endFileIndex++;
			}
		} while (fno.fname[0] != 0); // ファイルの有無を確認

		// ファイル数を保存
		int16_t fileCount = endFileIndex;
		// バブルソートでファイル番号を昇順に並べ替え
		for (int16_t i = 0; i < fileCount - 1; i++)
		{
			for (int16_t j = i + 1; j < fileCount; j++)
			{
				if (fileNumbers[i] > fileNumbers[j])
				{
					int16_t tmp = fileNumbers[i];
					fileNumbers[i] = fileNumbers[j];
					fileNumbers[j] = tmp; // 要素を交換
				}
			}
		}
		// PCへの取込前に原本を失わないよう、機体側では古いログを自動削除しない。
		endFileIndex = fileCount - 1;	// 最終インデックスを更新
		fileIndexLog = endFileIndex;	// 現在のログ番号を最終インデックスに設定
	}

	f_closedir(&dir); // directory close

	return endFileIndex;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 setLogHeaderStr
// 処理概要     ログCSVのヘッダーに "変数名=値" を追記する
// 引数         name: 変数名 value: 値
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void setLogHeaderStr(char *name, int32_t value)
{
	char headerStr[64];

	snprintf((char *)headerStr, sizeof(headerStr), "%s=%ld,", name, (long)value); // バッファサイズを指定して安全に変換
	strncat((char *)columnTitle, (char *)headerStr, sizeof(columnTitle) - strlen((char *)columnTitle) - 1); // バッファサイズを指定して安全に結合
}
/////////////////////////////////////////////////////////////////////
// モジュール名 setLogHeaderStrF
// 処理概要     ログCSVのヘッダーに "変数名=値" を追記する (float用)
// 引数         name: 変数名 value: 値
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void setLogHeaderStrF(char *name, float value)
{
    char headerStr[64];

    snprintf((char *)headerStr, sizeof(headerStr), "%s=%4.2f,", name, (double)value);
    strncat((char *)columnTitle, (char *)headerStr, sizeof(columnTitle) - strlen((char *)columnTitle) - 1);
}
static void setLogHeaderStrFPrecision(char *name, float value, unsigned precision)
{
	char headerStr[96];
	snprintf(headerStr, sizeof(headerStr), "%s=%.*f,", name, (int)precision, (double)value);
	strncat(columnTitle, headerStr, sizeof(columnTitle) - strlen(columnTitle) - 1);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 setLogHeaderStrS
// 処理概要     ログCSVのヘッダーに "変数名=文字列" を追記する
// 引数         name:変数名 value:文字列値
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void setLogHeaderStrS(char *name, const char *value)
{
	char headerStr[96];

	snprintf(headerStr, sizeof(headerStr), "%s=%s,", name, value);
	strncat(columnTitle, headerStr, sizeof(columnTitle) - strlen(columnTitle) - 1);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 createDir
// 処理概要     ホームディレクトリに指定されたディレクトリが存在しなければ作成する
// 引数         ディレクトリ名
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void createDir(char *dirName)
{
	FRESULT fresult;		// f_write status
	DIR dir;         // Directory
	FILINFO fno; // File Info
	uint8_t exist = 0;

	fresult = f_opendir(&dir, "/"); // directory open
	if (fresult == FR_OK)
	{
		do
		{
			f_readdir(&dir, &fno);
			if (strcmp(fno.fname, dirName) == 0)
			{
				exist = 1; // dirNameディレクトリが存在する
				break;
			}
		} while (fno.fname[0] != 0); // ファイルの有無を確認

		if (!exist)
		{
			// dirNameディレクトリが存在しない場合は作成する
			f_mkdir(dirName);
		}
	}
	f_closedir(&dir); // 関数を抜ける前に必ずディレクトリを閉じる
}
/////////////////////////////////////////////////////////////////////
// モジュール名 send8bit
// 処理概要     8bit変数をアクティブバッファに送る
// 引数         変換する8bit変数
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void send8bit(uint8_t data)
{
	(void)logBufferWriterAppendU8(&logBufferWriter, data);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 send16bit
// 処理概要     16bit変数を1バイトごとに分割してアクティブバッファに送る
// 引数         変換する16bit変数
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void send16bit(uint16_t data)
{
	(void)logBufferWriterAppendU16(&logBufferWriter, data);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 send32bit
// 処理概要     32bit変数を1バイトごとに分割してアクティブバッファに送る
// 引数         変換する32bit変数
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void send32bit(uint32_t data)
{
	(void)logBufferWriterAppendU32(&logBufferWriter, data);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 logSendFloat
// 処理概要     floatをIEEE-754の生ビットとして32bit送信する
// 引数         送信するfloat値
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void logSendFloat(float value)
{
	union
	{
		float f;
		uint32_t i;
	} ftoi;

	ftoi.f = value;
	send32bit(ftoi.i);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 sdRepairSparseSettings
// 処理概要     欠落・範囲外の設定項目を既定値で補う
// 引数         values: 設定値, present: 読込成否, defaults: 既定値, minimums/maximums: 範囲, count: 項目数
// 戻り値       true: 補修あり false: 補修なし
/////////////////////////////////////////////////////////////////////
bool sdRepairSparseSettings(int *values, const bool *present,
	const int *defaults, const int *minimums, const int *maximums, size_t count)
{
	bool repaired = false;
	for (size_t i = 0U; i < count; i++)
	{
		if (!present[i] || values[i] < minimums[i] || values[i] > maximums[i])
		{
			values[i] = defaults[i];
			repaired = true;
		}
	}
	return repaired;
}


/////////////////////////////////////////////////////////////////////
// モジュール名 logStorageCanRead
// 処理概要     保存キューと一時ファイルが停止時読込へ切替可能か確認する
// 引数         なし
// 戻り値       true: 切替可能 false: 保存処理中
/////////////////////////////////////////////////////////////////////
bool logStorageCanRead(void)
{
    return !modeLOG && !logTempOpen && !logBufferWriter.flushPending && !logBufferWriter.pendingPending;
}
