//====================================//
// インクルード
//====================================//
#include "SDcard.h"
#include "logSource.h"
#include "autoRun.h"
#include "logBinary.h"
#include "courseLogCsv.h"
#include "courseAnalysis.h"
#include "firmware_version.h"
#include "IMU.h"
#include "logBufferWriter.h"
#include "markerSensor.h"
#include "sd_functions.h"
#include "stdio.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <limits.h>
//====================================//
// グローバル変数の宣
//====================================//
// MicroSD
FIL fil_W;
FIL fil_R;

// ログヘッダー
// 詳細デバッグ列を含むCSVのフォーマットと1行分を格納できるサイズにする。
#define LOG_COLUMN_TITLE_BUFFER_SIZE 4096U
#define LOG_FORMAT_BUFFER_SIZE       512U
#define LOG_CSV_LINE_BUFFER_SIZE    1024U
#define LOG_TEMP_PREALLOC_BYTES (2UL * 1024UL * 1024UL)
char columnTitle[LOG_COLUMN_TITLE_BUFFER_SIZE] = "", formatLog[LOG_FORMAT_BUFFER_SIZE] = "";

// ログバッファ
// Log buffers

#define CROSS_STRAIGHT_MM 100.0f
#define CROSSSEG_MAX 128
#define PRIMARY_ROUTE_GOAL_X_LIMIT_MM 60.0F
_Static_assert(BUFFER_SIZE_LOG == LOG_BUFFER_SIZE_BYTES, "log buffer size mismatch");
_Static_assert((BUFFER_SIZE_LOG % LOG_BUFFER_SECTOR_SIZE_BYTES) == 0U, "log buffer must be sector aligned");
_Static_assert(LOG_RECORD_SIZE_BYTES <= LOG_BUFFER_SIZE_BYTES, "log record must fit in one log buffer");
static LogBufferWriter logBufferWriter;
static _Alignas(LOG_BUFFER_SECTOR_SIZE_BYTES)
	uint8_t logBuffers[LOG_BUFFER_COUNT][LOG_BUFFER_SIZE_BYTES];
static float cross_start_mm[CROSSSEG_MAX];
static float cross_end_mm[CROSSSEG_MAX];
uint16_t cntSend = 0;
uint8_t *logaddress;
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
static uint16_t sourceCrossCount = 0U;
static LogProgressCallback logProgressCallback;
static LogConversionProgress conversionProgress;
static uint64_t conversionWorkDone, conversionWorkTotal;
static bool conversionBatchComplete;
static bool lastRunSaved = false;
static bool lastPrimaryRouteValid = false;
static bool runImuCalibrationValid = false;
static uint16_t runImuCalibrationSamples = 0U;
static uint16_t runImuCalibrationReadErrors = 0U;

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

static PrimaryRouteValidation primaryRouteValidation = {false, false, false, 0, 0, 0.0F, 1U};

// スキーマ順で生成するレコード配置。
typedef struct
{
#define LOG_STRUCT_FIELD(type, name, fmt, expr) LOG_CTYPE_##type name;
#define LOG_STRUCT_SKIP(type, name, fmt, expr)
	LOG_FIELD_LIST(LOG_STRUCT_FIELD, LOG_STRUCT_SKIP)
#undef LOG_STRUCT_FIELD
#undef LOG_STRUCT_SKIP
} LogRecord;

// スキーマ関連のヘルパー宣言。
static void logSendFloat(float value);
static uint8_t logReadU8(void);
static uint16_t logReadU16(void);
static int16_t logReadS16(void);
static uint32_t logReadU32(void);
static int32_t logReadS32(void);
static float logReadF32(void);

static void logReadRecord(LogRecord *rec);
static void logBuildColumns(void);
static uint32_t logBinarySchema(void);
static void logConversionNotify(LogConversionStage stage, uint32_t rows);
static bool readSavedLogNumber(int16_t *outNumber);
static void writeSavedLogNumber(int16_t fileNumber);
static int16_t calcNextLogNumber(void);
static void setLogHeaderStrFPrecision(char *name, float value, unsigned precision);

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
void sd_flush_log(void)
{
	if (sd_is_analysis_active())
	{
		return;
	}

	// drain pending writes before analysis
	while (logBufferWriter.flushPending || logBufferWriter.pendingPending)
	{
		writeLogPuts();
	}

	if (sd_fatfs_try_lock())
	{
		f_sync(&fil_W);
		sd_fatfs_unlock();
	}
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
// モジュール名 getNextLogNumber
// 処理概要     画面表示用の次ログ番号を取得する
// 引数         なし
// 戻り値       次のログ番号
/////////////////////////////////////////////////////////////////////
int16_t getNextLogNumber(void)
{
	// まだ採番前なら計算値のみ返す
	if (logFileNumber == 0)
	{
		return calcNextLogNumber();
	}
	// 直前採番の次を返す
	return (int16_t)(logFileNumber + 1);
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
	formatLog[0] = 0;   // バッファを安全に初期化

	updateBatteryVoltage(); // ログヘッダへ停止時点の電圧を残す
	updateImuTempEndTemperature(); // ログ終了時の温度を取得

	setLogHeaderStr("binaryLogNumber", logFileNumber);
	setLogHeaderStr("binaryDataCrc", (int32_t)savedDataCrc);
	setLogHeaderStr("binarySchema", (int32_t)logBinarySchema());
	// 1行目: 走行条件と校正値
	setLogHeaderStrS("fwVersion", FW_VERSION);
	setLogHeaderStrS("gitCommit", GIT_COMMIT);
	setLogHeaderStrS("buildDate", BUILD_DATE);
	setLogHeaderStrS("buildTime", BUILD_TIME);
	setLogHeaderStrS("branch", GIT_BRANCH);
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
		setLogHeaderStr("primaryLogNumber", 0);
		setLogHeaderStr("slipSourceLogNumber", 0);
	}
	// ゴール誤検出調査用。行ログが終了直前で途切れても累積値を確認できる。
	setLogHeaderStr("sgMarkerAtLogEnd", (int32_t)SGmarker);
	setLogHeaderStr("encRightMarkerAtLogEnd_p", encRightMarker);
	setLogHeaderStr("routeControllerVersion", PATH_ROUTE_CONTROLLER_VERSION);
	setLogHeaderStr("routeSourceLog", pathRouteSourceLog());
	setLogHeaderStr("shortcutLevel", pathRouteShortcutLevel());

	setLogHeaderStrF("tgtParam.bstStraight", tgtParam.bstStraight);
	setLogHeaderStrF("tgtParam.bst1500", tgtParam.bst1500);
	setLogHeaderStrF("tgtParam.bst1300", tgtParam.bst1300);
	setLogHeaderStrF("tgtParam.bst1000", tgtParam.bst1000);
	setLogHeaderStrF("tgtParam.bst800", tgtParam.bst800);
	setLogHeaderStrF("tgtParam.bst700", tgtParam.bst700);
	setLogHeaderStrF("tgtParam.bst600", tgtParam.bst600);
	setLogHeaderStrF("tgtParam.bst500", tgtParam.bst500);
	setLogHeaderStrF("tgtParam.bst400", tgtParam.bst400);
	setLogHeaderStrF("tgtParam.bst300", tgtParam.bst300);
	setLogHeaderStrF("tgtParam.bst200", tgtParam.bst200);
	setLogHeaderStrF("tgtParam.bst100", tgtParam.bst100);
	setLogHeaderStrF("tgtParam.acceleF", tgtParam.acceleF);
	setLogHeaderStrF("tgtParam.acceleD", tgtParam.acceleD);
	setLogHeaderStrF("tgtParam.decelLeadMm", tgtParam.decelLeadMm);

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
	primaryRouteValidation = (PrimaryRouteValidation){false, false, false, 0, 0, 0.0F, 1U};
	// CSV変換ループの実行回数を走行ごとに正しく制御するため送信カウンタをリセット
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
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void writeLogBufferPuts(void)
{
	if (modeLOG)
	{
		if (sd_is_analysis_active())
		{
			return;
		}
		if (logBufferWriter.writeFailed || logBufferWriter.overflow || cntSend == UINT16_MAX)
		{
			if (cntSend == UINT16_MAX) logOverflow = true;
			return;
		}
		// スキーマから固定レコードサイズを算出。
		if (!logBufferWriterBeginRecord(&logBufferWriter, (uint32_t)LOG_RECORD_SIZE_BYTES))
		{
			logOverflow = true;
			dbg_overflow++;
			return;
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
		}
		else
		{
			logOverflow = true;
			dbg_overflow++;
		}
	}
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
// バイナリの保存・検証
/////////////////////////////////////////////////////////////////////
// モジュール名 logScanRecords
// 処理概要     記録のCRCとクロス区間を取得し、保存時は一次経路も検証する
// 引数         file: 読込ファイル, rows: 行数, live: 走行終了時検証, dataCrc: CRC出力
// 戻り値       true: 読込成功 false: 読込失敗
/////////////////////////////////////////////////////////////////////
static bool logScanRecords(FIL *file, uint16_t rows, bool live, uint32_t *dataCrc)
{
	uint8_t log[LOG_SIZE];
	LogRecord rec;
	FRESULT fresult;
	UINT readByte;
	uint16_t j, time, beforeTime = 0, cross_count = 0;
	int16_t speed, beforeSpeed = 0;
	float dt, dist_mm = 0.0F, goalMarkerX = 0.0F;
	bool in_cross = false, goalMarkerBracketFound = false, totalPulseMonotonic = true;
	int32_t goalMarkerPulse = 0, previousTotalPulse = 0;
	int64_t correctedPulseTotal = 0;
	uint32_t crc = UINT32_MAX;
	if (f_lseek(file, LOG_BINARY_HEADER_BYTES) != FR_OK) return false;
	// クロスライン前後100mm直線化のため、2パスで補正する
	// pass1: 距離基準でクロスライン区間を抽出
	beforeTime = 0;
	beforeSpeed = 0;
	dist_mm = 0.0f;
	in_cross = false;
	clearXYcie();
	if (live) primaryRouteValidation = (PrimaryRouteValidation){false, false, false, 0, 0, 0.0F, 1U};
	if (live) primaryRouteValidation.goalMarkerOnsetValid =
		markerGetGoalOnsetPulse(&goalMarkerPulse) && goalMarkerPulse > 0;
	if (live) primaryRouteValidation.goalMarkerOnset_p = goalMarkerPulse;
	for (j = 0; j < rows; j++)
	{
		fresult = f_read(file, log, sizeof(log), &readByte);
		if (fresult != FR_OK || readByte != LOG_SIZE)
		{
			printf("f_read error in endLog\r\n");
			return false;
		}
		crc = logBinaryCrcUpdate(crc, log, sizeof(log));
		if (logProgressCallback != NULL) { conversionWorkDone++; logConversionNotify(LOG_CONVERT_SCAN, (uint32_t)j + 1U); }
		logaddress = log;
		logReadRecord(&rec);

		time = rec.cntlog;
		speed = (int16_t)rec.encCurrentN;
		int32_t totalPulse = (int32_t)rec.encTotalOptimal;
		int32_t correctedPulse = (int32_t)rec.encCurrentCorr_p;
		int32_t pulseDelta = totalPulse - previousTotalPulse;
		if (pulseDelta < 0 || correctedPulse < 0 || correctedPulse > INT16_MAX)
		{
			totalPulseMonotonic = false;
		}
		// encCurrentCorr_p は1ms当たりのパルス数。ログ間隔[ms]を掛けて累積距離と比較する。
		if (correctedPulse >= 0) correctedPulseTotal += (int64_t)correctedPulse * (time - beforeTime);
		if (abs((int32_t)speed - (int32_t)beforeSpeed) > 500)
		{
			speed = beforeSpeed;
		}
		beforeSpeed = speed;
		dt = (float)(time - beforeTime) / 1000.0f;
		float previousX = xycie.x;
		if (correctedPulse >= 0 && correctedPulse <= INT16_MAX)
		{
			calcXYcie(totalPulse, rec.imuAngle_Z);
		}
		if (live && !goalMarkerBracketFound && primaryRouteValidation.goalMarkerOnsetValid &&
			pulseDelta >= 0 && goalMarkerPulse >= previousTotalPulse && goalMarkerPulse <= totalPulse)
		{
			float ratio = (pulseDelta > 0) ?
				(float)(goalMarkerPulse - previousTotalPulse) / (float)pulseDelta : 1.0F;
			if (ratio < 0.0F) ratio = 0.0F;
			if (ratio > 1.0F) ratio = 1.0F;
			goalMarkerX = previousX + ((xycie.x - previousX) * ratio);
			goalMarkerBracketFound = true;
		}
		previousTotalPulse = totalPulse;
		dist_mm += calcDlMm(speed, dt);
		beforeTime = time;

		bool is_cross = (rec.courseMarker == 3);
		if (!in_cross && is_cross)
		{
			if (cross_count < CROSSSEG_MAX)
			{
				cross_start_mm[cross_count] = dist_mm;
			}
			in_cross = true;
		}
		else if (in_cross && !is_cross)
		{
			if (cross_count < CROSSSEG_MAX)
			{
				cross_end_mm[cross_count] = dist_mm;
				cross_count++;
			}
			in_cross = false;
		}
	}
	if (in_cross && cross_count < CROSSSEG_MAX)
	{
		cross_end_mm[cross_count] = dist_mm;
		cross_count++;
	}

	sourceCrossCount = cross_count;
	*dataCrc = ~crc;
	if (!live) return true;
	primaryRouteValidation.goalMarkerX_mm = goalMarkerX;
	if (optimalTrace == BOOST_NONE)
	{
		if (!primaryRouteValidation.goalMarkerOnsetValid || SGmarker < COUNT_GOAL)
		{
			primaryRouteValidation.reason = 2U;
		}
		else if (logOverflow || markerOverflow || logBufferWriter.writeFailed || dbg_overflow != 0U ||
			emcStop != 0U || j != rows)
		{
			primaryRouteValidation.reason = 3U;
		}
		else if (!runImuCalibrationValid ||
			runImuCalibrationSamples != IMU_CALIBRATION_SAMPLE_COUNT ||
			runImuCalibrationReadErrors != 0U)
		{
			primaryRouteValidation.reason = 9U;
		}
		else if (!goalMarkerBracketFound || goalMarkerPulse <= 0)
		{
			primaryRouteValidation.reason = 5U;
		}
		else
		{
			int64_t scaleError = (int64_t)previousTotalPulse - correctedPulseTotal;
			int64_t scaleErrorAbs = (scaleError < 0) ? -scaleError : scaleError;
			int64_t scaleTolerance = (int64_t)lroundf(fmaxf(
				50.0F * PULSE_MILLIMETER, (float)previousTotalPulse * 0.05F));
			if (scaleError > INT32_MAX) scaleError = INT32_MAX;
			if (scaleError < INT32_MIN) scaleError = INT32_MIN;
			primaryRouteValidation.distanceScaleError_p = (int32_t)scaleError;
			primaryRouteValidation.distanceScaleVerified =
				totalPulseMonotonic && previousTotalPulse > 0 && scaleErrorAbs <= scaleTolerance &&
				PULSE_METER == 58019;
			if (!primaryRouteValidation.distanceScaleVerified)
			{
				primaryRouteValidation.reason = 10U;
			}
			else if (!isfinite(goalMarkerX) || fabsf(goalMarkerX) > PRIMARY_ROUTE_GOAL_X_LIMIT_MM)
			{
				primaryRouteValidation.reason = 8U;
			}
			else
			{
				primaryRouteValidation.reason = 0U;
				primaryRouteValidation.closureValid = true;
			}
		}
	}
	else
	{
		primaryRouteValidation.reason = 1U;
	}
	return true;
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
	while (logBufferWriter.flushPending || logBufferWriter.pendingPending) // drain pending writes before CSV conversion
	{
		writeLogPuts();
	}
	if (logBufferWriter.writeFailed || logOverflow || markerOverflow)
	{
		f_close(&fil_W);
		logTempOpen = false;
		return false;
	}

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
			return false;
		}
	}

	bool success = false;
	LogBinaryHeader header = {logFileNumber, LOG_SCHEMA_PROFILE_LIGHT, LOG_RECORD_SIZE_BYTES,
		cntSend, 0U, 0U, 0U, logBinarySchema()};
	FSIZE_t dataEnd = LOG_BINARY_HEADER_BYTES + (FSIZE_t)cntSend * LOG_SIZE;
	if (f_lseek(&fil_W, dataEnd) != FR_OK || f_truncate(&fil_W) != FR_OK ||
		f_sync(&fil_W) != FR_OK || !logScanRecords(&fil_W, cntSend, true, &savedDataCrc)) goto finish;
	logBuildMetadata();
	header.metadataBytes = (uint32_t)strlen(columnTitle);
	header.dataCrc = savedDataCrc;
	header.metadataCrc = ~logBinaryCrcUpdate(UINT32_MAX, (const uint8_t *)columnTitle, header.metadataBytes);
	if (f_lseek(&fil_W, dataEnd) != FR_OK ||
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
	return success;
}


// 共通読込は停止中に1ファイルずつ実行し、既存ヘッダ用RAMを再利用する。
static struct {
    FIL *file;
    LogBinaryHeader header;
    LogRecord record;
    uint32_t row, crc;
    uint16_t previousTime;
    int16_t previousSpeed;
    float distanceMm, roc, x, y;
    uint8_t headerStage;
    FRESULT error;
    bool rowReady;
} binarySource;

/////////////////////////////////////////////////////////////////////
// モジュール名 logBinarySchema
// 処理概要     列名・型・順序・派生計算版からスキーマ識別CRCを取得する
// 引数         なし
// 戻り値       スキーマ識別CRC
/////////////////////////////////////////////////////////////////////
static uint32_t logBinarySchema(void)
{
    static const char schema[] = "derived-v1;"
#define LOG_SCHEMA_TEXT(type, name, fmt, expr) #type ":" #name ";"
        LOG_FIELD_LIST(LOG_SCHEMA_TEXT, LOG_SCHEMA_TEXT);
#undef LOG_SCHEMA_TEXT
    return ~logBinaryCrcUpdate(UINT32_MAX, (const uint8_t *)schema, sizeof(schema) - 1U);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logConversionNotify
// 処理概要     行数を基準とした変換進捗を通知する
// 引数         stage: 処理段階, rows: 処理済み行数
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void logConversionNotify(LogConversionStage stage, uint32_t rows)
{
    conversionProgress.stage = stage;
    conversionProgress.processedRows = rows;
    uint64_t percent = conversionWorkTotal ? (conversionWorkDone * 100U / conversionWorkTotal) : 0U;
    if (percent > 99U) percent = 99U;
    if (stage == LOG_CONVERT_DONE && conversionBatchComplete && conversionProgress.fileIndex == conversionProgress.fileCount) percent = 100U;
    conversionProgress.totalPercent = (uint8_t)percent;
    if (logProgressCallback) logProgressCallback(&conversionProgress);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logReadContainerHeader
// 処理概要     保存済みコンテナのヘッダ・形式・ファイル長を検証する
// 引数         file: ファイル, number: 期待番号, header: 出力先
// 戻り値       true: 有効 false: 読込失敗または未対応形式
/////////////////////////////////////////////////////////////////////
static bool logReadContainerHeader(FIL *file, uint16_t number, LogBinaryHeader *header)
{
    UINT read = 0U;
    return f_read(file, logBuffers[0], LOG_BINARY_HEADER_BYTES, &read) == FR_OK &&
        read == LOG_BINARY_HEADER_BYTES && logBinaryDecodeHeader(logBuffers[0], f_size(file), header) &&
        header->number == number && header->profile == LOG_SCHEMA_PROFILE_LIGHT &&
        header->recordBytes == LOG_SIZE && header->schema == logBinarySchema();
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logSourceIsBinary
// 処理概要     共通読込中のファイルがバイナリか判定する
// 引数         file: ファイル
// 戻り値       true: バイナリ false: CSV
/////////////////////////////////////////////////////////////////////
bool logSourceIsBinary(FIL *file) { return binarySource.file == file; }

/////////////////////////////////////////////////////////////////////
// モジュール名 logSourceOpen
// 処理概要     バイナリを優先して開き、欠落時だけ既存CSVを開く
// 引数         file: ファイル, csvName: 番号.csv, mode: 読込モード
// 戻り値       FatFs結果（不正バイナリはFR_INVALID_OBJECT）
/////////////////////////////////////////////////////////////////////
FRESULT logSourceOpen(FIL *file, const char *csvName, BYTE mode)
{
    uint16_t number;
    if (binarySource.file != NULL) return FR_LOCKED;
    if (!logBinaryParseName(csvName, "csv", &number)) return FR_INVALID_NAME;
    char name[20];
    logBinaryName(name, sizeof(name), number, "bin");
    FRESULT result = f_open(file, name, FA_READ | FA_OPEN_EXISTING);
    if (result == FR_NO_FILE) return f_open(file, csvName, mode);
    if (result != FR_OK) return result;
    LogBinaryHeader header;
    UINT read;
    uint32_t crc;
    if (!logReadContainerHeader(file, number, &header) ||
        f_lseek(file, LOG_BINARY_HEADER_BYTES + (FSIZE_t)header.rows * header.recordBytes) != FR_OK ||
        f_read(file, columnTitle, header.metadataBytes, &read) != FR_OK || read != header.metadataBytes) goto invalid;
    columnTitle[header.metadataBytes] = '\0';
    if (columnTitle[header.metadataBytes - 1U] != '\n' ||
        memchr(columnTitle, '\0', header.metadataBytes) != NULL ||
        ~logBinaryCrcUpdate(UINT32_MAX, (const uint8_t *)columnTitle, header.metadataBytes) != header.metadataCrc ||
        !logScanRecords(file, (uint16_t)header.rows, false, &crc) || crc != header.dataCrc ||
        f_lseek(file, LOG_BINARY_HEADER_BYTES) != FR_OK) goto invalid;
    memset(&binarySource, 0, sizeof(binarySource));
    binarySource.file = file;
    binarySource.header = header;
    binarySource.crc = UINT32_MAX;
    clearXYcie();
    return FR_OK;
invalid:
    (void)f_close(file);
    return FR_INVALID_OBJECT;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logSourceClose
// 処理概要     共通読込の状態を解放しファイルを閉じる
// 引数         file: ファイル
// 戻り値       FatFs結果
/////////////////////////////////////////////////////////////////////
FRESULT logSourceClose(FIL *file)
{
    if (binarySource.file == file) binarySource.file = NULL;
    return f_close(file);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logProcessRecord
// 処理概要     記録を速度補正・曲率直線化・保存方位によるXYへ変換する
// 引数         なし（共通読込状態を使用）
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void logProcessRecord(void)
{
    LogRecord *rec = &binarySource.record;
    int16_t speed = (int16_t)rec->encCurrentN;
    if (abs((int32_t)speed - binarySource.previousSpeed) > 500) {
        speed = binarySource.previousSpeed;
        rec->encCurrentN = (uint16_t)speed;
    }
    float dt = (float)(rec->cntlog - binarySource.previousTime) / 1000.0F;
    binarySource.roc = calcROC(speed, rec->gyroVal_Z, dt);
    calcXYcie((int32_t)rec->encTotalOptimal, rec->imuAngle_Z);
    binarySource.x = xycie.x;
    binarySource.y = xycie.y;
    binarySource.distanceMm += calcDlMm(speed, dt);
    binarySource.previousSpeed = speed;
    binarySource.previousTime = rec->cntlog;
    for (uint16_t i = 0; i < sourceCrossCount; i++) {
        float start = fmaxf(0.0F, cross_start_mm[i] - CROSS_STRAIGHT_MM);
        if (binarySource.distanceMm >= start && binarySource.distanceMm <= cross_end_mm[i] + CROSS_STRAIGHT_MM) {
            binarySource.roc = ROC_STRAIGHT_MAX;
            break;
        }
    }
    if (rec->courseMarker == 3U) binarySource.roc = ROC_STRAIGHT_MAX;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logSourceGets
// 処理概要     ヘッダを返し、データ部では文字列化せず1記録を復元する
// 引数         line: 出力先, capacity: 容量[B], file: ファイル
// 戻り値       成功時line、EOFまたはエラー時NULL（行値は専用APIから取得）
/////////////////////////////////////////////////////////////////////
char *logSourceGets(char *line, int capacity, FIL *file)
{
    if (!logSourceIsBinary(file)) return f_gets(line, capacity, file);
    binarySource.rowReady = false;
    if (binarySource.error != FR_OK || capacity < 2) return NULL;
    if (binarySource.headerStage < 2U) {
        if (binarySource.headerStage == 1U) {
            columnTitle[0] = formatLog[0] = '\0';
            logBuildColumns();
            strncat(columnTitle, "\n", sizeof(columnTitle) - strlen(columnTitle) - 1U);
            strncat(formatLog, "\n", sizeof(formatLog) - strlen(formatLog) - 1U);
        }
        size_t length = strlen(columnTitle);
        if (length >= (size_t)capacity) { binarySource.error = FR_INVALID_PARAMETER; return NULL; }
        memmove(line, columnTitle, length + 1U);
        binarySource.headerStage++;
        return line;
    }
    if (binarySource.row >= binarySource.header.rows) {
        if (~binarySource.crc != binarySource.header.dataCrc) binarySource.error = FR_INVALID_OBJECT;
        return NULL;
    }
    uint8_t bytes[LOG_SIZE];
    UINT read;
    FRESULT result = f_read(file, bytes, sizeof(bytes), &read);
    if (result != FR_OK || read != sizeof(bytes)) {
        binarySource.error = result == FR_OK ? FR_DISK_ERR : result;
        return NULL;
    }
    binarySource.crc = logBinaryCrcUpdate(binarySource.crc, bytes, sizeof(bytes));
    logaddress = bytes;
    logReadRecord(&binarySource.record);
    logProcessRecord();
    binarySource.row++;
    binarySource.rowReady = true;
    line[0] = '\n'; line[1] = '\0';
    return line;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logSourceError
// 処理概要     CSVまたはバイナリ読込のエラーを取得する
// 引数         file: ファイル
// 戻り値       0: エラーなし その他: エラー
/////////////////////////////////////////////////////////////////////
int logSourceError(FIL *file) { return logSourceIsBinary(file) ? (int)binarySource.error : f_error(file); }

/////////////////////////////////////////////////////////////////////
// モジュール名 logSourceEof
// 処理概要     メタデータ領域を除いた記録の終端を判定する
// 引数         file: ファイル
// 戻り値       非0: EOF 0: EOF以外
/////////////////////////////////////////////////////////////////////
int logSourceEof(FIL *file)
{
    return logSourceIsBinary(file) ? binarySource.row == binarySource.header.rows && binarySource.error == FR_OK : f_eof(file);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logSourceDistanceRow
// 処理概要     距離解析用の値を共通形式へ復元する
// 引数         file: ファイル, line: CSV行, columns: 列対応, row: 出力先
// 戻り値       true: 有効 false: 不正
/////////////////////////////////////////////////////////////////////
bool logSourceDistanceRow(FIL *file, const char *line, const CourseLogColumnMap *columns, CourseLogDistanceRow *row)
{
    if (!logSourceIsBinary(file)) return courseLogParseDistanceRow(line, columns, row);
    if (!binarySource.rowReady) return false;
    LogRecord *r = &binarySource.record;
    *row = (CourseLogDistanceRow){r->cntlog, r->encCurrentN, r->gyroVal_Z,
        r->courseMarker, (int32_t)r->encTotalOptimal, binarySource.roc};
    return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logSourceSlipRow
// 処理概要     スリップ解析用の値を共通形式へ復元する
// 引数         file: ファイル, line: CSV行, columns: 列対応, row: 出力先
// 戻り値       true: 有効 false: 不正
/////////////////////////////////////////////////////////////////////
bool logSourceSlipRow(FIL *file, const char *line, const CourseLogColumnMap *columns, CourseLogSlipRow *row)
{
    if (!logSourceIsBinary(file)) return courseLogParseSlipRow(line, columns, row);
    if (!binarySource.rowReady) return false;
    LogRecord *r = &binarySource.record;
    *row = (CourseLogSlipRow){r->courseMarker, (int32_t)r->encTotalOptimal, binarySource.roc,
        r->targetSpeed, r->optimalIndex, r->slipFlag, r->slipFlagLat};
    return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logSourceBinaryPoint
// 処理概要     バイナリ記録から経路生成用のXYと距離を取得する
// 引数         file: ファイル, x: X[mm], y: Y[mm], pulse: 累積距離[pulse]
// 戻り値       true: 有効な点 false: 不正
/////////////////////////////////////////////////////////////////////
bool logSourceBinaryPoint(FIL *file, float *x, float *y, float *pulse)
{
    if (!logSourceIsBinary(file) || !binarySource.rowReady) return false;
    *x = binarySource.x; *y = binarySource.y; *pulse = (float)binarySource.record.encTotalOptimal;
    return isfinite(*x) && isfinite(*y) && isfinite(*pulse);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logFormatSourceRow
// 処理概要     共通計算済み記録を従来のCSV書式へ変換する
// 引数         line: 出力先, capacity: 容量[B]
// 戻り値       true: 変換成功 false: 容量不足
/////////////////////////////////////////////////////////////////////
static bool logFormatSourceRow(char *line, size_t capacity)
{
    LogRecord rec = binarySource.record;
    float log_roc = binarySource.roc, log_x = binarySource.x, log_y = binarySource.y;
#define LOG_FORMAT_VALUE_U8(value) (value)
#define LOG_FORMAT_VALUE_U16(value) (value)
#define LOG_FORMAT_VALUE_S16(value) (value)
#define LOG_FORMAT_VALUE_U32(value) ((int32_t)(value))
#define LOG_FORMAT_VALUE_S32(value) ((long)(value))
#define LOG_FORMAT_VALUE_F32(value) (value)
#define LOG_CSV_ARG_STORED(type, name, fmt, expr) , LOG_FORMAT_VALUE_##type(rec.name)
#define LOG_CSV_ARG_DERIVED(type, name, fmt, expr) , LOG_FORMAT_VALUE_##type(expr)
    int length = snprintf(line, capacity, formatLog LOG_FIELD_LIST(LOG_CSV_ARG_STORED, LOG_CSV_ARG_DERIVED));
#undef LOG_CSV_ARG_STORED
#undef LOG_CSV_ARG_DERIVED
#undef LOG_FORMAT_VALUE_U8
#undef LOG_FORMAT_VALUE_U16
#undef LOG_FORMAT_VALUE_S16
#undef LOG_FORMAT_VALUE_U32
#undef LOG_FORMAT_VALUE_S32
#undef LOG_FORMAT_VALUE_F32
    return length >= 0 && (size_t)length < capacity;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logExistingCsvMatches
// 処理概要     同番号CSVの走行メタデータが元バイナリと同一か確認する
// 引数         name: CSV名, header: 元バイナリ情報
// 戻り値       true: 未作成または同一ログ false: 不一致またはI/Oエラー
/////////////////////////////////////////////////////////////////////
static bool logExistingCsvMatches(const char *name, const LogBinaryHeader *header)
{
    FIL file;
    FRESULT result = f_open(&file, name, FA_READ | FA_OPEN_EXISTING);
    if (result == FR_NO_FILE) return true;
    if (result != FR_OK) return false;
    uint32_t crc = UINT32_MAX;
    bool matches = true;
    for (uint32_t i = 0; i < header->metadataBytes; i++) {
        uint8_t byte;
        UINT read;
        if (f_read(&file, &byte, 1U, &read) != FR_OK || read != 1U) { matches = false; break; }
        crc = logBinaryCrcUpdate(crc, &byte, 1U);
    }
    if (f_close(&file) != FR_OK) matches = false;
    return matches && ~crc == header->metadataCrc;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logConvertOne
// 処理概要     1ログを作業CSVへ変換し、確定成功後に元バイナリを削除する
// 引数         number: ログ番号
// 戻り値       true: 全処理成功 false: 元バイナリを保持して失敗
/////////////////////////////////////////////////////////////////////
static bool logConvertOne(uint16_t number)
{
    char csv[20], part[20], bin[20], line[LOG_CSV_LINE_BUFFER_SIZE];
    FIL input, output;
    bool inputOpen = false, outputOpen = false, success = false;
    logBinaryName(csv, sizeof(csv), number, "csv");
    logBinaryName(part, sizeof(part), number, "part");
    logBinaryName(bin, sizeof(bin), number, "bin");
    logConversionNotify(LOG_CONVERT_SCAN, 0U);
    if (logSourceOpen(&input, csv, FA_READ | FA_OPEN_EXISTING) != FR_OK) goto cleanup;
    inputOpen = true;
    if (!logSourceIsBinary(&input) || !logExistingCsvMatches(csv, &binarySource.header) ||
        f_open(&output, part, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) goto cleanup;
    outputOpen = true;
    if (!logSourceGets(columnTitle, sizeof(columnTitle), &input) || f_puts(columnTitle, &output) < 0 ||
        !logSourceGets(columnTitle, sizeof(columnTitle), &input) || f_puts(columnTitle, &output) < 0) goto cleanup;
    logConversionNotify(LOG_CONVERT_WRITE, 0U);
    while (logSourceGets(line, sizeof(line), &input)) {
        if (!logFormatSourceRow(line, sizeof(line)) || f_puts(line, &output) < 0) goto cleanup;
        conversionWorkDone++;
        logConversionNotify(LOG_CONVERT_WRITE, binarySource.row);
    }
    if (logSourceError(&input) || !logSourceEof(&input)) goto cleanup;
    logConversionNotify(LOG_CONVERT_COMMIT, binarySource.header.rows);
    if (f_sync(&output) != FR_OK) goto cleanup;
    FRESULT closeResult = f_close(&output);
    outputOpen = false;
    if (closeResult != FR_OK) goto cleanup;
    closeResult = logSourceClose(&input);
    inputOpen = false;
    if (closeResult != FR_OK) goto cleanup;
    FILINFO info;
    FRESULT exists = f_stat(csv, &info);
    if (exists != FR_NO_FILE && exists != FR_OK) goto cleanup;
    // 元バイナリは残っているため、置換途中の電源断でも再変換できる。
    if (exists == FR_OK && f_unlink(csv) != FR_OK) goto cleanup;
    if (f_rename(part, csv) != FR_OK || f_unlink(bin) != FR_OK) goto cleanup;
    conversionWorkDone++;
    if (conversionProgress.fileIndex < conversionProgress.fileCount) logConversionNotify(LOG_CONVERT_DONE, conversionProgress.expectedRows);
    success = true;
cleanup:
    if (outputOpen) (void)f_close(&output);
    if (inputOpen) (void)logSourceClose(&input);
    if (!success) logConversionNotify(LOG_CONVERT_FAILED, conversionProgress.processedRows);
    return success;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logPendingInventory
// 処理概要     未変換ログの本数・作業量・次の番号を取得する
// 引数         after: 直前番号, next: 次番号, nextRows: 次の行数, count: 本数, work: 作業量
// 戻り値       true: 列挙成功 false: 不正ログまたはI/Oエラー
/////////////////////////////////////////////////////////////////////
static bool logPendingInventory(uint16_t after, uint16_t *next, uint32_t *nextRows, uint16_t *count, uint64_t *work)
{
    DIR dir;
    FILINFO info;
    *next = 0; *nextRows = 0; *count = 0; *work = 0;
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
        if (f_open(&file, info.fname, FA_READ | FA_OPEN_EXISTING) != FR_OK) { success = false; conversionProgress.logNumber = number; break; }
        bool valid = logReadContainerHeader(&file, number, &header);
        if (f_close(&file) != FR_OK) valid = false;
        if (!valid) { success = false; conversionProgress.logNumber = number; break; }
        if (*count == UINT16_MAX) { success = false; break; }
        (*count)++;
        *work += (uint64_t)header.rows * 2U + 1U;
        if (number > after && (*next == 0U || number < *next)) { *next = number; *nextRows = header.rows; }
    } while (info.fname[0] != '\0');
    if (f_closedir(&dir) != FR_OK) success = false;
    return success;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logConvertPending
// 処理概要     未変換ログを番号順にCSV化し、起動時の中断復旧にも使う
// 引数         progress: 進捗通知（NULLでも実行可）
// 戻り値       true: 全変換成功 false: 走行開始禁止を維持
/////////////////////////////////////////////////////////////////////
bool logConvertPending(LogProgressCallback progress)
{
    if (modeLOG || logTempOpen || !sd_fatfs_lock(500U)) { logRecoveryReady = false; return false; }
    bool success = true;
    sd_set_analysis_active(true);
    logProgressCallback = progress;
    logTempWarning = false;
    conversionProgress = (LogConversionProgress){0};
    conversionWorkDone = 0;
    conversionBatchComplete = false;
    uint16_t next, count, after = 0;
    uint32_t rows;
    if (!logPendingInventory(after, &next, &rows, &count, &conversionWorkTotal)) success = false;
    conversionProgress.fileCount = count;
    while (success && next != 0U) {
        conversionProgress.fileIndex++;
        conversionProgress.logNumber = next;
        conversionProgress.expectedRows = rows;
        if (!logConvertOne(next)) { success = false; break; }
        after = next;
        uint64_t remainingWork;
        if (!logPendingInventory(after, &next, &rows, &count, &remainingWork)) success = false;
    }
    if (!success) logConversionNotify(LOG_CONVERT_FAILED, conversionProgress.processedRows);
    else if (conversionProgress.fileCount > 0U) {
        conversionBatchComplete = true;
        logConversionNotify(LOG_CONVERT_DONE, conversionProgress.expectedRows);
    }
    logProgressCallback = NULL;
    sd_set_analysis_active(false);
    sd_fatfs_unlock();
    logRecoveryReady = success;
    return success;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logRecoveryIsReady
// 処理概要     復旧が完了して新規走行を開始可能か取得する
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
			uint16_t csvNumber;
			if (logBinaryParseName(fno.fname, "csv", &csvNumber) && endFileIndex < FILENUMBER_NUM)
			{
				// csvファイルのとき
				fileNumbers[endFileIndex] = (int16_t)csvNumber;
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
		// ログ数が上限を超えたら、FILENUMBER_ALARMの半分まで古いログを削除
		if (logRecoveryReady && fileCount > FILENUMBER_LIMIT)
		{
			int16_t targetFileCount = (int16_t)(FILENUMBER_ALARM / 2);
			if (targetFileCount < 1)
			{
				targetFileCount = 1;
			}
			int16_t toDelete = (int16_t)(fileCount - targetFileCount);
			if (toDelete < 1)
			{
				toDelete = 1;
			}
			for (int16_t i = 0; i < toDelete; i++)
			{
				char deleteName[16];
				snprintf(deleteName, sizeof(deleteName), "%d.csv", fileNumbers[i]);
				char pendingName[20];
				FILINFO pendingInfo;
				logBinaryName(pendingName, sizeof(pendingName), (uint16_t)fileNumbers[i], "bin");
				if (f_stat(pendingName, &pendingInfo) == FR_NO_FILE) f_unlink(deleteName);
			}
			// 配列を詰め直し
			for (int16_t i = 0; i < fileCount - toDelete; i++)
			{
				fileNumbers[i] = fileNumbers[i + toDelete];
			}
			fileCount -= toDelete;
		}
		endFileIndex = fileCount - 1;	// 最終インデックスを更新
		fileIndexLog = endFileIndex;	// 現在のログ番号を最終インデックスに設定
	}

	f_closedir(&dir); // directory close

	return endFileIndex;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 setLogStr
// 処理概要     ログCSVファイルのヘッダーとprintfのフォーマット文字列を生成
// 引数         column: ヘッダー文字列 format: フォーマット文字列
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void setLogStr(char *column, char *format)
{
	char columnStr[30];	// ヘッダー文字列を一時的に格納するバッファ
	char formatStr[30];	// フォーマット文字列を一時的に格納するバッファ

	// copy str to local variable
       snprintf((char *)columnStr, sizeof(columnStr), "%s", column); // バッファサイズを指定して安全にコピー
       snprintf((char *)formatStr, sizeof(formatStr), "%s", format); // バッファサイズを指定して安全にコピー

       strncat((char *)columnStr, ",", sizeof(columnStr) - strlen((char *)columnStr) - 1); // バッファサイズを指定して安全に結合
       strncat((char *)formatStr, ",", sizeof(formatStr) - strlen((char *)formatStr) - 1); // バッファサイズを指定して安全に結合
       strncat((char *)columnTitle, (char *)columnStr, sizeof(columnTitle) - strlen((char *)columnTitle) - 1); // バッファサイズを指定して安全に結合
       strncat((char *)formatLog, (char *)formatStr, sizeof(formatLog) - strlen((char *)formatLog) - 1);       // バッファサイズを指定して安全に結合
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
// モジュール名 SDtest
// 処理概要     SDカードの読み書きテスト
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void SDtest(void)
{
	FIL fil_T; // テスト用ファイル
	FRESULT fresult;		// f_write status

	fresult = f_open(&fil_T, "test.csv", FA_OPEN_ALWAYS | FA_WRITE); // create file
	uint32_t start = HAL_GetTick(); // SPI待ちにタイムアウトを設定
	while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY)
	{
		if (HAL_GetTick() - start > 1000)
		{
			break;
		}
	}
	f_close(&fil_T);
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
// モジュール名 logReadU8
// 処理概要     logaddressから1バイト読み出してポインタを進める
// 引数         なし
// 戻り値       読み出した8bit値
/////////////////////////////////////////////////////////////////////
static uint8_t logReadU8(void)
{
	return *logaddress++;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 logReadU16
// 処理概要     logaddressから16bitをビッグエンディアンで読み出す
// 引数         なし
// 戻り値       読み出した16bit値
/////////////////////////////////////////////////////////////////////
static uint16_t logReadU16(void)
{
	uint16_t hi = (uint16_t)*logaddress++;
	uint16_t lo = (uint16_t)*logaddress++;
	return (uint16_t)((hi << 8) | lo);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 logReadS16
// 処理概要     16bitを読み出して符号付きへ変換する
// 引数         なし
// 戻り値       読み出した16bitの符号付き値
/////////////////////////////////////////////////////////////////////
static int16_t logReadS16(void)
{
	return (int16_t)logReadU16();
}
/////////////////////////////////////////////////////////////////////
// モジュール名 logReadU32
// 処理概要     logaddressから32bitをビッグエンディアンで読み出す
// 引数         なし
// 戻り値       読み出した32bit値
/////////////////////////////////////////////////////////////////////
static uint32_t logReadU32(void)
{
	uint32_t value = 0;

	value = (uint32_t)*logaddress++ << 24;
	value |= (uint32_t)*logaddress++ << 16;
	value |= (uint32_t)*logaddress++ << 8;
	value |= (uint32_t)*logaddress++;
	return value;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 logReadS32
// 処理概要     32bitを読み出して符号付きへ変換する
// 引数         なし
// 戻り値       読み出した32bitの符号付き値
/////////////////////////////////////////////////////////////////////
static int32_t logReadS32(void)
{
	return (int32_t)logReadU32();
}
/////////////////////////////////////////////////////////////////////
// モジュール名 logReadF32
// 処理概要     32bitの生ビットをfloatへ変換する
// 引数         なし
// 戻り値       読み出したfloat値
/////////////////////////////////////////////////////////////////////
static float logReadF32(void)
{
	union
	{
		float f;
		uint32_t i;
	} ftoi;

	ftoi.i = logReadU32();
	return ftoi.f;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 logReadRecord
// 処理概要     LOG_FIELD_LISTの順で1レコードを復元する
// 引数         rec: 復元先のレコード
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void logReadRecord(LogRecord *rec)
{
#define LOG_READ_FIELD(type, name, fmt, expr) rec->name = logRead##type();
#define LOG_READ_SKIP(type, name, fmt, expr)
	LOG_FIELD_LIST(LOG_READ_FIELD, LOG_READ_SKIP)
#undef LOG_READ_FIELD
#undef LOG_READ_SKIP
}
/////////////////////////////////////////////////////////////////////
// モジュール名 logBuildColumns
// 処理概要     LOG_FIELD_LISTからCSVヘッダーとprintf形式を生成する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void logBuildColumns(void)
{
#define LOG_HEADER_FIELD(type, name, fmt, expr) setLogStr(#name, fmt);
	LOG_FIELD_LIST(LOG_HEADER_FIELD, LOG_HEADER_FIELD)
#undef LOG_HEADER_FIELD
}
