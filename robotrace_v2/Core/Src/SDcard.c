//====================================//
// インクルード
//====================================//
#include "SDcard.h"
#include "control.h"
#include "courseAnalysis.h"
#include "distanceEstimator.h"
#include "encoder.h"
#include "markerSensor.h"
#include "lineSensor.h"
#include "headingEstimator.h"
#include "runGuard.h"
#include "timer.h"
#include "firmware_version.h"
#include "sd_functions.h"
#include <math.h>
#include <stdlib.h>
#include "stdio.h"
#include <stdint.h>
#ifndef ROBOTRACE_ENABLE_SLIP_UPDATE
#define ROBOTRACE_ENABLE_SLIP_UPDATE 0
#endif
//====================================//
// グローバル変数の宣
//====================================//
// MicroSD
FIL fil_W;
FIL fil_R;

// ログヘッダー
// 詳細デバッグ列を含むCSVのフォーマットと1行分を格納できるサイズにする。
#define LOG_COLUMN_TITLE_BUFFER_SIZE 6144U
#define LOG_FORMAT_BUFFER_SIZE       512U
#define LOG_CSV_LINE_BUFFER_SIZE    1024U
char columnTitle[LOG_COLUMN_TITLE_BUFFER_SIZE] = "", formatLog[LOG_FORMAT_BUFFER_SIZE] = "";
static bool logHeaderOverflow = false;
static float gyroIntervalSum = 0.0f;
static uint16_t gyroIntervalSamples = 0U;
static int32_t encoderIntervalL = 0;
static int32_t encoderIntervalR = 0;
static int16_t logEncoderIntervalL = 0;
static int16_t logEncoderIntervalR = 0;
static float logGyroIntervalAverage = 0.0f;
static bool gyroSampleFault = false;
static bool encoderIntervalFault = false;
static HeadingCalibration headingCalibration = {0U, 58092U, 57945U, 10602U};
static HeadingCalibration runHeadingCalibration = {0U, 58092U, 57945U, 10602U};
static HeadingBiasEstimator headingEstimator;
static bool runImuCalibrationValid = false;
static uint16_t runImuCalibrationSamples = 0U;
static uint16_t runImuCalibrationReadErrors = 0U;
typedef struct
{
	float goalX_mm;
	float goalY_mm;
	float goalXGyro_mm;
	float goalYGyro_mm;
	float goalXFusedDiagnostic_mm;
	float goalYFusedDiagnostic_mm;
	float goalS_mm;
	int32_t goalPulse_p;
	uint8_t valid;
	uint8_t reason; // Schema 10: 0有効、1非一次、2マーカー、3ログ、5区間外、6エンコーダ、8 X超過、9 IMU校正、10 距離未検証
} LogClosure;
static LogClosure logClosure;
static bool lastRunSaved = false;
static float runStartOmegaKp;
static float runStartOmegaKi;
static float runStartOmegaKd;
static uint32_t runStartLineCalibrationHash;

static void setLogHeaderStrFPrecision(char *name, float value, uint8_t precision);

/////////////////////////////////////////////////////////////////////
// モジュール名 logCaptureRunStartSettings
// 処理概要     動的ゲイン変更前の設定とライン校正値の照合値を保存する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void logCaptureRunStartSettings(void)
{
	runStartOmegaKp = lineTraceOmegaFBCtrl.kp;
	runStartOmegaKi = lineTraceOmegaFBCtrl.ki;
	runStartOmegaKd = lineTraceOmegaFBCtrl.kd;
	uint32_t hash = 2166136261UL;
	for (uint8_t i = 0U; i < NUM_SENSORS; i++)
	{
		uint16_t values[2] = {lSensorMin[i], lSensorMax[i]};
		for (uint8_t j = 0U; j < 2U; j++)
		{
			hash = (hash ^ (uint8_t)values[j]) * 16777619UL;
			hash = (hash ^ (uint8_t)(values[j] >> 8)) * 16777619UL;
		}
	}
	runStartLineCalibrationHash = hash;
	runHeadingCalibration = headingCalibration;
	runImuCalibrationValid = IMU_CalibrationReady();
	runImuCalibrationSamples = IMU_CalibrationSamples();
	runImuCalibrationReadErrors = IMU_CalibrationReadErrors();
}

/////////////////////////////////////////////////////////////////////
// モジュール名 writeHeadingCalibrationSettings
// 処理概要     一次経路の左右車輪校正値をSDへ保存する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void writeHeadingCalibrationSettings(void)
{
	FIL file;
	if (f_open(&file, PATH_SETTING "heading_cal.txt", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
	{
		f_printf(&file, "%u,%lu,%lu,%u", headingCalibration.enabled,
			(unsigned long)headingCalibration.pulsePerMeterL,
			(unsigned long)headingCalibration.pulsePerMeterR,
			headingCalibration.effectiveTreadCentiMm);
		f_close(&file);
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 readImuTempCompensation
// 処理概要     SDカードからBMI088ジャイロZ温度係数を読み込み、不正時は0へ修復する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
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
// モジュール名 readHeadingCalibrationSettings
// 処理概要     SD校正値を部分反映し、破損時は無効化して修復する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void readHeadingCalibrationSettings(void)
{
	FIL file;
	char valueText[64] = {0};
	int values[4] = {0, 58092, 57945, 10602};
	char trailing = '\0';
	int parsed = 0;
	bool repair = false;
	headingCalibration = (HeadingCalibration){0U, 58092U, 57945U, 10602U};
	if (f_open(&file, PATH_SETTING "heading_cal.txt", FA_OPEN_EXISTING | FA_READ) == FR_OK)
	{
		if (f_gets(valueText, sizeof(valueText), &file) != NULL)
		{
			parsed = sscanf(valueText, "%d,%d,%d,%d%c", &values[0], &values[1],
				&values[2], &values[3], &trailing);
		}
		else repair = true;
		f_close(&file);
	}
	else repair = true;
	if (parsed >= 1 && (values[0] == 0 || values[0] == 1))
		headingCalibration.enabled = (uint8_t)values[0];
	else repair = true;
	if (parsed >= 2 && values[1] >= 50000 && values[1] <= 65000)
		headingCalibration.pulsePerMeterL = (uint32_t)values[1];
	else repair = true;
	if (parsed >= 3 && values[2] >= 50000 && values[2] <= 65000)
		headingCalibration.pulsePerMeterR = (uint32_t)values[2];
	else repair = true;
	if (parsed >= 4 && values[3] >= 9000 && values[3] <= 14000)
		headingCalibration.effectiveTreadCentiMm = (uint16_t)values[3];
	else repair = true;
	if (parsed != 4) repair = true;
	if (repair)
	{
		headingCalibration.enabled = 0U;
		writeHeadingCalibrationSettings();
	}
	// 左右実測値が走行距離用の共通換算値から2%を超える場合は経路生成に使わない。
	if (headingCalibration.pulsePerMeterL < (uint32_t)(PULSE_METER * 0.98f) ||
		headingCalibration.pulsePerMeterL > (uint32_t)(PULSE_METER * 1.02f) ||
		headingCalibration.pulsePerMeterR < (uint32_t)(PULSE_METER * 0.98f) ||
		headingCalibration.pulsePerMeterR > (uint32_t)(PULSE_METER * 1.02f))
	{
		// 正常な既存ファイルは保存値を維持し、この走行では診断専用にする。
		headingCalibration.enabled = 0U;
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 logLastRunWasSaved
// 処理概要     直前のCSVが最後まで正常保存されたか取得する
// 引数         なし
// 戻り値       true:保存完了 false:未検証
/////////////////////////////////////////////////////////////////////
bool logLastRunWasSaved(void) { return lastRunSaved; }

/////////////////////////////////////////////////////////////////////
// モジュール名 logLastClosureValid
// 処理概要     直前の一次走行ログの経路採用可否を取得する
// 引数         なし
// 戻り値       true:経路採用可 false:経路不採用
/////////////////////////////////////////////////////////////////////
bool logLastClosureValid(void) { return lastRunSaved && logClosure.valid != 0U; }

/////////////////////////////////////////////////////////////////////
// モジュール名 logLastClosureReason
// 処理概要     直前の一次走行ログの経路採否理由を取得する
// 引数         なし
// 戻り値       理由コード
/////////////////////////////////////////////////////////////////////
uint8_t logLastClosureReason(void) { return logClosure.reason; }
/////////////////////////////////////////////////////////////////////
// モジュール名 logResetMotionInterval
// 処理概要     走行開始時にログ区間の角速度・左右パルス積算を初期化する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void logResetMotionInterval(void)
{
	gyroIntervalSum = 0.0f;
	gyroIntervalSamples = 0U;
	encoderIntervalL = 0;
	encoderIntervalR = 0;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 logAccumulateMotion1ms
// 処理概要     同じ1ms区間の角速度と左右エンコーダを積算する
// 引数         gyroDegPerSec: 角速度[deg/s], encoderL/R: 左右パルス[pulse]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void logAccumulateMotion1ms(float gyroDegPerSec, int16_t encoderL, int16_t encoderR)
{
	if (isfinite(gyroDegPerSec) && gyroIntervalSamples < UINT16_MAX)
	{
		gyroIntervalSum += gyroDegPerSec;
		gyroIntervalSamples++;
		encoderIntervalL += encoderL;
		encoderIntervalR += encoderR;
	}
	else
	{
		gyroSampleFault = true;
	}
}

// ログバッファ
// Log buffers

#define CROSS_STRAIGHT_MM 100.0f
#define CROSSSEG_MAX 128
#define LOG_BUFFER_COUNT 3
#define LOG_TEMP_PREALLOC_BYTES (2UL * 1024UL * 1024UL)
uint8_t logBuffer[LOG_BUFFER_COUNT][BUFFER_SIZE_LOG];
uint8_t *activeBuf = logBuffer[0];
uint8_t *flushBuf = logBuffer[1];
uint8_t *pendingBuf = logBuffer[2];
int16_t logBuffIndex = 0;
uint32_t logBuffSendIndex = 0;
uint32_t logBuffPendingIndex = 0;
volatile bool sendSD = false;
volatile bool sendSD_pending = false;
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
static volatile bool create_log_ready = false;

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
static float logReadF32(void);
static void logReadRecord(LogRecord *rec);
static void logBuildColumns(void);
static bool logAppendText(char *buffer, size_t bufferSize, const char *text);
static uint8_t *logGetFreeBuffer(void);
static bool readSavedLogNumber(int16_t *outNumber);
static void writeSavedLogNumber(int16_t fileNumber);
static int16_t calcNextLogNumber(void);


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
	while (sendSD)
	{
		writeLogPuts();
	}
	if (sendSD_pending)
	{
		uint32_t primask = __get_PRIMASK();
		__disable_irq();
		sendSD = true;
		__set_PRIMASK(primask);
		writeLogPuts();
		while (sendSD)
		{
			writeLogPuts();
		}
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
		total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5); // total capacity
		printf("SD_SIZE: \t%lu\r\n", total);
		free_space = (uint32_t)(fre_clust * pfs->csize * 0.5); // empty capacity
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
	int16_t savedNumber = 0;

	// 保存値があればその次の番号
	if (readSavedLogNumber(&savedNumber))
	{
		return (int16_t)(savedNumber + 1);
	}

	// SD内ログがあれば最大+1
	if (endFileIndex >= 0)
	{
		return (int16_t)(fileNumbers[endFileIndex] + 1);
	}

	// どちらも無ければ1から開始
	return 1;
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
// モジュール名 createLog
// 処理概要     ログファイルを作成し、ヘッダ情報を出力する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void createLog(void)
{
	FRESULT fresult;		// f_write status
	char fileName[32];
	UINT written = 0;
	UINT total = 0;
	create_log_ready = false;

	// 初回は保存値/SD内最大から次番号を決定
	if (logFileNumber == 0)
	{
		logFileNumber = (uint16_t)calcNextLogNumber();
	}
	else
	{
		// 2回目以降は連番で増加
		logFileNumber++; // next log number
	}

	// 最新ログ番号+1にして絵尾久ファイル名を生成 (バッファサイズを指定して安全に文字列化)
	snprintf((char *)fileName, sizeof(fileName), "%d", logFileNumber);
	// バッファサイズを指定して安全に拡張子を追加
	strncat((char *)fileName, ".csv", sizeof(fileName) - strlen((char *)fileName) - 1);
	fresult = f_open(&fil_W, fileName, FA_CREATE_ALWAYS | FA_WRITE); // create/overwrite file
	if (fresult != FR_OK)
	{
		// ファイルオープンに失敗した場合はログ作成を中止する
		return; // エラーが発生したため処理を終了
	}
	// 作成できた番号を保存ファイルへ反映
	writeSavedLogNumber((int16_t)logFileNumber);
	// 追加: 作成したログ番号を一覧に反映（savedLogNo 計算の整合を取る）
	const int16_t maxN = (int16_t)(sizeof(fileNumbers) / sizeof(fileNumbers[0]));
	if (endFileIndex < (maxN - 1))
	{
		endFileIndex++;
		fileNumbers[endFileIndex] = (int16_t)logFileNumber;
		fileIndexLog = endFileIndex;
	}

	columnTitle[0] = 0; // バッファを安全に初期化
	formatLog[0] = 0;   // バッファを安全に初期化
	logHeaderOverflow = false;

	updateBatteryVoltage(); // ログヘッダへ停止時点の電圧を残す
	updateImuTempEndTemperature(); // ログヘッダへ終了時点のBMI088温度を残す

	// 1行目: メタデータ
	setLogHeaderStrS("fwVersion", FW_VERSION);
	setLogHeaderStrS("gitCommit", GIT_COMMIT);
	setLogHeaderStrS("buildDate", BUILD_DATE);
	setLogHeaderStrS("buildTime", BUILD_TIME);
	setLogHeaderStrS("branch", GIT_BRANCH);
	setLogHeaderStr("logSchemaVersion", LOG_SCHEMA_VERSION);
	setLogHeaderStrFPrecision("gyroScaleCoeff", COEFF_DPD, 6U);
	setLogHeaderStr("imuTempCalibrationValid", imuTempCalibrationValid ? 1 : 0);
	setLogHeaderStrFPrecision("imuTempCalibrationStart_C", imuTempCalibrationStart_C, 3U);
	setLogHeaderStrFPrecision("imuTempCalibration_C", imuTempCalibration_C, 3U);
	setLogHeaderStrFPrecision("imuTempCalibrationEnd_C", imuTempCalibrationEnd_C, 3U);
	setLogHeaderStrU("imuTempCalibrationSamples", imuTempCalibrationSamples);
	setLogHeaderStrU("imuTempCalibrationReadErrors", imuTempCalibrationReadErrors);
	setLogHeaderStrFPrecision("imuGyroOffsetZ_dps", angleOffset[2], 3U);
	setLogHeaderStr("imuTempCompEnabled", imuTempCorrectionEnabled ? 1 : 0);
	setLogHeaderStrFPrecision("imuTempCoeff_dpsPerC", imuTempCoeff_dpsPerC, 6U);
	setLogHeaderStrFPrecision("imuTempEnd_C", imuTempEnd_C, 3U);
	setLogHeaderStrU("encoderPulsePerMeter", PULSE_METER);
	setLogHeaderStr("distanceScaleVerified", PRIMARY_DISTANCE_SCALE_VERIFIED);
	setLogHeaderStr("imuCalibrationValid", runImuCalibrationValid ? 1 : 0);
	setLogHeaderStrU("imuCalibrationSamples", runImuCalibrationSamples);
	setLogHeaderStrU("imuCalibrationReadErrors", runImuCalibrationReadErrors);
	setLogHeaderStr("headingCalibration.enabled", runHeadingCalibration.enabled);
	setLogHeaderStrU("headingCalibration.pulsePerMeterL", runHeadingCalibration.pulsePerMeterL);
	setLogHeaderStrU("headingCalibration.pulsePerMeterR", runHeadingCalibration.pulsePerMeterR);
	setLogHeaderStrF("headingCalibration.effectiveTread_mm",
		(float)runHeadingCalibration.effectiveTreadCentiMm * 0.01f);
	DistanceEstimatorDiagnostics distanceDiagnostics = DistanceEstimator_GetDiagnostics();
	uint32_t maxAbsFusedDeltaP = 0U;
	if (isfinite(distanceDiagnostics.maxAbsFusedDeltaM) &&
		distanceDiagnostics.maxAbsFusedDeltaM > 0.0F)
	{
		maxAbsFusedDeltaP = (uint32_t)(distanceDiagnostics.maxAbsFusedDeltaM *
			(float)PULSE_METER + 0.5F);
	}
	setLogHeaderStrU("logRecordSizeBytes", (uint32_t)LOG_RECORD_SIZE_BYTES);
	setLogHeaderStrU("logExpectedRows", cntSend);
	RunTimingDiagnostics timing = Timer_GetRunDiagnostics();
	setLogHeaderStrU("timing.isrMax_us", (uint32_t)(((uint64_t)timing.maxIsrCycles * 1000000ULL) / SystemCoreClock));
	setLogHeaderStrU("timing.imuReadMax_us", (uint32_t)(((uint64_t)timing.maxImuReadCycles * 1000000ULL) / SystemCoreClock));
	setLogHeaderStrU("timing.isrOverrunCount", timing.isrOverrunCount);
	setLogHeaderStrU("timing.imuReadErrorCount", timing.imuReadErrorCount);
	setLogHeaderStrU("timing.lineUpdateMaxInterval_ms", timing.maxLineUpdateIntervalMs);
	setLogHeaderStrU("timing.lineStaleCycleCount", timing.lineStaleCycleCount);
	setLogHeaderStrU("timing.adcPhaseMismatchCount", timing.adcPhaseMismatchCount);
	setLogHeaderStrU("timing.startResetMeasured", timing.startResetMeasured);
	setLogHeaderStrU("timing.startResetDelay_ms", timing.startResetDelayMs);
	setLogHeaderStr("timing.startResetPulseDelta_p", timing.startResetPulseDelta);
	setLogHeaderStr("slipUpdateEnabled", ROBOTRACE_ENABLE_SLIP_UPDATE);
	setLogHeaderStrF("runStartOmega.kp", runStartOmegaKp);
	setLogHeaderStrF("runStartOmega.ki", runStartOmegaKi);
	setLogHeaderStrF("runStartOmega.kd", runStartOmegaKd);
	setLogHeaderStrU("runStartLineCalibrationFNV1a32", runStartLineCalibrationHash);
	setLogHeaderStrU("dbgOverflowFinal", dbg_overflow);
	setLogHeaderStrU("logOverflowFinal", logOverflow ? 1U : 0U);
	setLogHeaderStr("gyroSampleFault", gyroSampleFault ? 1 : 0);
	setLogHeaderStr("encoderIntervalFault", encoderIntervalFault ? 1 : 0);
	setLogHeaderStrF("headingKalman.bias_dps", headingEstimator.biasDps);
	setLogHeaderStrU("headingKalman.accepted", headingEstimator.accepted);
	setLogHeaderStrU("headingKalman.rejected", headingEstimator.rejected);
	setLogHeaderStr("headingKalman.invalid", headingEstimator.invalid ? 1 : 0);
	setLogHeaderStrF("distanceKalman.sigmaAccel_mps2", DISTANCE_ESTIMATOR_SIGMA_ACCEL_MPS2);
	setLogHeaderStrF("distanceKalman.sigmaEncoder_mps", DISTANCE_ESTIMATOR_SIGMA_ENCODER_MPS);
	setLogHeaderStrF("distanceKalman.biasRandomWalk_mps2_sqrt_s", DISTANCE_ESTIMATOR_BIAS_RANDOM_WALK_MPS2_SQRT_S);
	setLogHeaderStrF("distanceKalman.initialBiasSigma_mps2", DISTANCE_ESTIMATOR_INITIAL_BIAS_SIGMA_MPS2);
	setLogHeaderStrU("distanceKalman.innovationRejectCount",
		distanceDiagnostics.innovationRejectCount);
	setLogHeaderStrU("distanceKalman.fallbackCount", distanceDiagnostics.fallbackCount);
	setLogHeaderStrU("distanceKalman.invalidUpdateCount",
		distanceDiagnostics.invalidUpdateCount);
	setLogHeaderStrU("distanceKalman.maxAbsFusedDelta_p", maxAbsFusedDeltaP);
	setLogHeaderStrU("distanceKalman.outputGuardCount",
		Control_GetDistanceFusionOutputGuardCount());
	// 制御パラメータ
	setLogHeaderStrF("batteryVoltage_V", batteryVoltage_V);
	setLogHeaderStrF("optimalTrace", optimalTrace);
	setLogHeaderStrF("autoStart", autoStart);
	setLogHeaderStrF("emcStop", emcStop);
	setLogHeaderStr("analysisSourceLog", analysisRunSourceLog());
	setLogHeaderStr("slipSourceLog", analysisRunSlipSourceLog());
	// ゴール誤検出調査用。行ログが終了直前で途切れても累積値を確認できる。
	setLogHeaderStr("sgMarkerAtLogEnd", (int32_t)SGmarker);
	setLogHeaderStr("startMarkerOnsetValid", (int32_t)startMarkerOnsetValid);
	setLogHeaderStr("goalMarkerOnsetValid", (int32_t)goalMarkerOnsetValid);
	setLogHeaderStr("goalMarkerOnset_p", logClosure.goalPulse_p);
	setLogHeaderStr("closureValid", (int32_t)logClosure.valid);
	setLogHeaderStr("closureReason", (int32_t)logClosure.reason);
	setLogHeaderStrF("goalMarkerXRaw_mm", logClosure.goalX_mm);
	setLogHeaderStrF("goalMarkerYRaw_mm", logClosure.goalY_mm);
	setLogHeaderStrF("goalMarkerXGyro_mm", logClosure.goalXGyro_mm);
	setLogHeaderStrF("goalMarkerYGyro_mm", logClosure.goalYGyro_mm);
	setLogHeaderStrF("goalMarkerXFusedDiagnostic_mm", logClosure.goalXFusedDiagnostic_mm);
	setLogHeaderStrF("goalMarkerYFusedDiagnostic_mm", logClosure.goalYFusedDiagnostic_mm);
	setLogHeaderStrF("goalMarkerS_mm", logClosure.goalS_mm);
	setLogHeaderStr("encRightMarkerAtLogEnd_p", encRightMarker);
	setLogHeaderStr("routeControllerVersion", PATH_ROUTE_CONTROLLER_VERSION);
	setLogHeaderStr("routeSourceLog", pathRunRouteSourceLog());
	setLogHeaderStr("routePointCount", pathRunRouteCount());
	setLogHeaderStrU("routeGeometryCrc32", pathRunRouteGeometryCrc32());
	setLogHeaderStr("shortcutRequestedLevel", pathRunRouteRequestedLevel());
	setLogHeaderStr("shortcutLevel", pathRunRouteShortcutLevel());
	setLogHeaderStr("shortcutBuildStatus", pathRunRouteShortcutBuildStatus());
	setLogHeaderStr("shortcutCorridorCount", pathRunRouteShortcutCorridorCount());
	setLogHeaderStrF("shortcutReduction_mm", pathRunRouteShortcutReductionMm());
	ShortcutSettings runSettings = pathRunStartSettings();
	ShortcutSettings generationSettings = pathRunGenerationSettings();
	setLogHeaderStr("shortcutSettings.maxLevel", runSettings.maxLevel);
	setLogHeaderStr("shortcutSettings.lookaheadBaseMm", runSettings.lookaheadBaseMm);
	setLogHeaderStr("shortcutSettings.lookaheadPerMpsMm", runSettings.lookaheadPerMpsMm);
	setLogHeaderStr("shortcutSettings.kLateral_x100", runSettings.kLateral_x100);
	setLogHeaderStr("shortcutSettings.kHeading_x100", runSettings.kHeading_x100);
	setLogHeaderStr("shortcutSettings.lineAlpha_x1000", runSettings.lineAlpha_x1000);
	setLogHeaderStr("shortcutSettings.lineThetaGain_x1e9", runSettings.lineThetaGain_x1e9);
	setLogHeaderStr("routeShortcutSettings.maxLevel", generationSettings.maxLevel);
	setLogHeaderStr("routeShortcutSettings.lookaheadBaseMm", generationSettings.lookaheadBaseMm);
	setLogHeaderStr("routeShortcutSettings.lookaheadPerMpsMm", generationSettings.lookaheadPerMpsMm);
	setLogHeaderStr("routeShortcutSettings.kLateral_x100", generationSettings.kLateral_x100);
	setLogHeaderStr("routeShortcutSettings.kHeading_x100", generationSettings.kHeading_x100);
	setLogHeaderStr("routeShortcutSettings.lineAlpha_x1000", generationSettings.lineAlpha_x1000);
	setLogHeaderStr("routeShortcutSettings.lineThetaGain_x1e9", generationSettings.lineThetaGain_x1e9);

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
	setLogHeaderStrF("tgtParam.pathReplay", tgtParam.pathReplay);

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
	setLogHeaderStrF("yawCtrl.kp", yawCtrl.kp);
	setLogHeaderStrF("yawCtrl.ki", yawCtrl.ki);
	setLogHeaderStrF("yawCtrl.kd", yawCtrl.kd);
	setLogHeaderStrF("distCtrl.kp", distCtrl.kp);
	setLogHeaderStrF("distCtrl.ki", distCtrl.ki);
	setLogHeaderStrF("distCtrl.kd", distCtrl.kd);
	if (!logAppendText(columnTitle, sizeof(columnTitle), "\n"))
	{
		logHeaderOverflow = true;
	}
	if (logHeaderOverflow)
	{
		printf("createLog header truncated\r\n");
		f_close(&fil_W);
		f_unlink(fileName);
		create_log_ready = false;
		return;
	}
	total = (UINT)strlen(columnTitle);
	fresult = f_write(&fil_W, columnTitle, total, &written);
	if (fresult != FR_OK || written != total)
	{
		printf("createLog metadata write error: %d (%lu/%lu)\r\n", fresult, (unsigned long)written, (unsigned long)total);
		f_close(&fil_W);
		f_unlink(fileName);
		return;
	}

	// 2行目: ログ列名。フォーマット文字列も同時に生成する。
	columnTitle[0] = 0;
	formatLog[0] = 0;
	logHeaderOverflow = false;
	logBuildColumns();
	if (!logAppendText(columnTitle, sizeof(columnTitle), "\n") ||
		!logAppendText(formatLog, sizeof(formatLog), "\n"))
	{
		logHeaderOverflow = true;
	}
	if (logHeaderOverflow)
	{
		printf("createLog column header truncated\r\n");
		f_close(&fil_W);
		f_unlink(fileName);
		return;
	}
	total = (UINT)strlen(columnTitle);
	written = 0;
	fresult = f_write(&fil_W, columnTitle, total, &written);
	if (fresult != FR_OK || written != total)
	{
		printf("createLog column header write error: %d (%lu/%lu)\r\n", fresult, (unsigned long)written, (unsigned long)total);
		f_close(&fil_W);
		f_unlink(fileName);
		return;
	}
	fresult = f_sync(&fil_W);
	if (fresult != FR_OK)
	{
		printf("createLog f_sync error: %d\r\n", fresult);
		f_close(&fil_W);
		f_unlink(fileName);
		return;
	}
	create_log_ready = true;
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
	gyroSampleFault = false;
	encoderIntervalFault = false;
	HeadingBiasEstimator_Reset(&headingEstimator);
	// CSV変換ループの実行回数を走行ごとに正しく制御するため送信カウンタをリセット
	cntSend = 0;
	fresult = f_open(&fil_W, "temp", FA_CREATE_ALWAYS | FA_WRITE); // create/overwrite file
	if (fresult != FR_OK)
	{
		printf("error opening log file: %d\r\n", fresult); // エラー内容を出力
		initMSD = false; // ファイルオープンに失敗した場合はmicroSDを使用不可とする
		return;          // ログ初期化を中止
	}
	logBuffIndex = 0;					// 書込位置を初期化
#if _USE_EXPAND
	/* Best-effort contiguous preallocation to reduce FAT updates during logging. */
	fresult = f_expand(&fil_W, (FSIZE_t)LOG_TEMP_PREALLOC_BYTES, 1);
	if (fresult != FR_OK)
	{
		printf("f_expand failed: %d\r\n", fresult);
	}
#endif
	f_lseek(&fil_W, 0);
	activeBuf = logBuffer[0];			// アクティブバッファを初期化
	flushBuf = logBuffer[1];			// フラッシュバッファを初期化
	pendingBuf = logBuffer[2];
	logBuffSendIndex = logBuffIndex;	// バッファのバイト数を記録
	logBuffPendingIndex = 0;
	sendSD = false;						// 書き込み要求をリセット
	sendSD_pending = false;
	dbg_overflow = 0;
	logOverflow = false;				// ログバッファ状態フラグをリセット
}

static uint8_t *logGetFreeBuffer(void)
{
	for (uint32_t i = 0; i < LOG_BUFFER_COUNT; i++)
	{
		uint8_t *buf = logBuffer[i];
		if (buf == activeBuf)
		{
			continue;
		}
		if (sendSD && buf == flushBuf)
		{
			continue;
		}
		if (sendSD_pending && buf == pendingBuf)
		{
			continue;
		}
		return buf;
	}
	return NULL;
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
		// スキーマから固定レコードサイズを算出。
		const uint16_t requiredSize = (uint16_t)LOG_RECORD_SIZE_BYTES;

		if (logBuffIndex + requiredSize > BUFFER_SIZE_LOG)
		{
			if (sendSD && sendSD_pending)
			{
				logOverflow = true;
				dbg_overflow++;
				return;
			}

			uint8_t *newBuf = logGetFreeBuffer();
			if (newBuf == NULL)
			{
				logOverflow = true;
				dbg_overflow++;
				return;
			}

			if (sendSD)
			{
				logBuffPendingIndex = logBuffIndex;
				pendingBuf = activeBuf;
				sendSD_pending = true;
			}
			else
			{
				logBuffSendIndex = logBuffIndex;
				flushBuf = activeBuf;
				sendSD = true;
			}

			activeBuf = newBuf;
			logBuffIndex = 0;
		}

		// スキーマ順でバイナリ書き込みを展開。
		logGyroIntervalAverage = (gyroIntervalSamples > 0U) ?
			(gyroIntervalSum / (float)gyroIntervalSamples) : imuVal.gyro.z;
		if (gyroIntervalSamples == 0U || gyroIntervalSamples != cntLog) gyroSampleFault = true;
		if (encoderIntervalL < INT16_MIN || encoderIntervalL > INT16_MAX ||
			encoderIntervalR < INT16_MIN || encoderIntervalR > INT16_MAX)
		{
			encoderIntervalFault = true;
			logEncoderIntervalL = 0;
			logEncoderIntervalR = 0;
		}
		else
		{
			logEncoderIntervalL = (int16_t)encoderIntervalL;
			logEncoderIntervalR = (int16_t)encoderIntervalR;
		}
#define LOG_SEND_U8(value) send8bit((uint8_t)(value))
#define LOG_SEND_U16(value) send16bit((uint16_t)(value))
#define LOG_SEND_S16(value) send16bit((uint16_t)(int16_t)(value))
#define LOG_SEND_U32(value) send32bit((uint32_t)(value))
#define LOG_SEND_F32(value) logSendFloat((float)(value))
#define LOG_SEND_FIELD(type, name, fmt, expr) LOG_SEND_##type(expr);
#define LOG_SEND_SKIP(type, name, fmt, expr)
		LOG_FIELD_LIST(LOG_SEND_FIELD, LOG_SEND_SKIP)
		logResetMotionInterval();
#undef LOG_SEND_FIELD
#undef LOG_SEND_SKIP
#undef LOG_SEND_U8
#undef LOG_SEND_U16
#undef LOG_SEND_S16
#undef LOG_SEND_U32
#undef LOG_SEND_F32

		cntSend++;
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
	if (!modeLOG && !sendSD)
	{
		return; // skip if no pending write
	}

	if (sendSD)
	{
		if (!sd_fatfs_try_lock())
		{
			return;
		}
		fresult = f_write(&fil_W, flushBuf, logBuffSendIndex, &writtenlog);
		if (fresult != FR_OK || writtenlog != logBuffSendIndex)
		{
			uint32_t primask = __get_PRIMASK();
			__disable_irq();
			sendSD = false;
			sendSD_pending = false;
			__set_PRIMASK(primask);
			sd_fatfs_unlock();
			return;
		}

		{
			uint32_t primask = __get_PRIMASK();
			__disable_irq();
			if (sendSD_pending)
			{
				flushBuf = pendingBuf;
				logBuffSendIndex = logBuffPendingIndex;
				sendSD_pending = false;
				sendSD = true;
			}
			else
			{
				sendSD = false;
			}
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
	f_close(&fil_W); // 一時ファイルを閉じる
}
/////////////////////////////////////////////////////////////////////
// モジュール名 endLog
// 処理概要     ロギング終了処理
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void endLog(void)
{
	lastRunSaved = false;
	modeLOG = false; // stop logging
	while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
	FRESULT fresult;		// f_write status
	FIL fil;
	uint8_t log[LOG_SIZE];
	char logStr[LOG_CSV_LINE_BUFFER_SIZE];
	UINT readByte, writtenlog;
	uint16_t j;
	uint16_t time, beforeTime = 0;
	int16_t speed, beforeSpeed = 0;
	float dt, zg;
	float log_roc, log_x, log_y, log_x_fused, log_y_fused;
	HeadingPose fusedPose = {0.0f, 0.0f, 0.0f};
	float travelled_mm = 0.0f;
	int32_t previousPulse_p = 0;
	bool logIntervalValid = true;
	bool goalBracketFound = false;
	LogRecord rec;

	float cross_start_mm[CROSSSEG_MAX];
	float cross_end_mm[CROSSSEG_MAX];
	uint16_t cross_count = 0;
	float dist_mm = 0.0f;
	bool in_cross = false;

	while (sendSD || sendSD_pending) // drain pending writes before CSV conversion
	{
		writeLogPuts();
	}

	logBuffSendIndex = logBuffIndex;
	fresult = f_write(&fil_W, activeBuf, logBuffSendIndex, &writtenlog);
	if (fresult != FR_OK || writtenlog != logBuffSendIndex)
	{
		printf("f_write error in endLog: %d (%lu/%lu)\r\n", fresult, (unsigned long)writtenlog, (unsigned long)logBuffSendIndex);
	}
	f_close(&fil_W);

	fresult = f_open(&fil, "temp", FA_OPEN_EXISTING | FA_READ);
	if (fresult != FR_OK)
	{
		printf("f_open error in endLog\r\n");
		return;
	}
	// 停止後の第1パス: 左右差によるバイアス推定は診断専用。経路には適用しない。
	HeadingBiasEstimator_Reset(&headingEstimator);
	if (optimalTrace == BOOST_NONE && runHeadingCalibration.enabled != 0U)
	{
		uint16_t previousTime = 0U;
		for (j = 0; j < cntSend; j++)
		{
			fresult = f_read(&fil, log, sizeof(log), &readByte);
			if (fresult != FR_OK || readByte != LOG_SIZE)
			{
				headingEstimator.invalid = true;
				break;
			}
			logaddress = log;
			logReadRecord(&rec);
			uint16_t intervalMs = (uint16_t)(rec.cntlog - previousTime);
			if (!HeadingBiasEstimator_Update(&headingEstimator, &runHeadingCalibration,
				rec.gyroVal_Z, rec.encIntervalL_p, rec.encIntervalR_p,
				(float)intervalMs * 0.001f)) break;
			previousTime = rec.cntlog;
		}
	}
	f_close(&fil);
	fresult = f_open(&fil, "temp", FA_OPEN_EXISTING | FA_READ);
	if (fresult != FR_OK)
	{
		printf("f_open error after heading estimate\r\n");
		return;
	}

	// 第2パス: クロスライン区間と経路用ジャイロXY・診断用融合XYを抽出する。
	beforeTime = 0;
	beforeSpeed = 0;
	dist_mm = 0.0f;
	travelled_mm = 0.0f;
	previousPulse_p = 0;
	in_cross = false;
	clearXYcie();
	fusedPose = (HeadingPose){0.0f, 0.0f, 0.0f};
	memset(&logClosure, 0, sizeof(logClosure));
	logClosure.goalPulse_p = goalMarkerOnset_p;
	logClosure.reason = (optimalTrace == BOOST_NONE) ? 2U : 1U;
	for (j = 0; j < cntSend; j++)
	{
		fresult = f_read(&fil, log, sizeof(log), &readByte);
		if (readByte != LOG_SIZE)
		{
			break;
		}
		if (fresult != FR_OK)
		{
			printf("f_read error in endLog\r\n");
			f_close(&fil);
			return;
		}
		logaddress = log;
		logReadRecord(&rec);

		time = rec.cntlog;
		speed = (int16_t)rec.encCurrentN;
		if (abs((int32_t)speed - (int32_t)beforeSpeed) > 500)
		{
			speed = beforeSpeed;
		}
		beforeSpeed = speed;
		dt = (float)(uint16_t)(time - beforeTime) / 1000.0f;
		if (optimalTrace == BOOST_NONE)
		{
			int32_t pulse_p = (int32_t)rec.encTotalOptimal;
			int32_t delta_p = pulse_p - previousPulse_p;
			float previousX = xycie.x;
			float previousY = xycie.y;
			float previousFusedX = fusedPose.x_mm;
			float previousFusedY = fusedPose.y_mm;
			float previousS = travelled_mm;
			uint16_t intervalMs = (uint16_t)(time - beforeTime);
			if (intervalMs == 0U || intervalMs > 100U || delta_p < 0 ||
				!isfinite(rec.gyroVal_Z)) logIntervalValid = false;
			if (logIntervalValid)
			{
				calcXYcie(delta_p, rec.gyroVal_Z, (float)intervalMs / 1000.0f);
				HeadingPose_Advance(&fusedPose, (float)delta_p / PULSE_MILLIMETER,
					rec.gyroVal_Z - headingEstimator.biasDps,
					(float)intervalMs / 1000.0f);
				travelled_mm += (float)delta_p / PULSE_MILLIMETER;
				if (!goalBracketFound && logClosure.goalPulse_p > 0 &&
					previousPulse_p <= logClosure.goalPulse_p &&
					logClosure.goalPulse_p <= pulse_p && delta_p > 0)
				{
					float ratio = (float)(logClosure.goalPulse_p - previousPulse_p) / (float)delta_p;
					logClosure.goalXGyro_mm = previousX + ratio * (xycie.x - previousX);
					logClosure.goalYGyro_mm = previousY + ratio * (xycie.y - previousY);
					logClosure.goalX_mm = logClosure.goalXGyro_mm;
					logClosure.goalY_mm = logClosure.goalYGyro_mm;
					logClosure.goalXFusedDiagnostic_mm = previousFusedX + ratio * (fusedPose.x_mm - previousFusedX);
					logClosure.goalYFusedDiagnostic_mm = previousFusedY + ratio * (fusedPose.y_mm - previousFusedY);
					logClosure.goalS_mm = previousS + ratio * (travelled_mm - previousS);
					goalBracketFound = true;
				}
			}
			previousPulse_p = pulse_p;
		}
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
	if (optimalTrace == BOOST_NONE)
	{
		DistanceEstimatorDiagnostics diagnostics = DistanceEstimator_GetDiagnostics();
		if (!startMarkerOnsetValid || !goalMarkerOnsetValid || SGmarker < COUNT_GOAL)
			logClosure.reason = 2U;
		else if (!logIntervalValid || gyroSampleFault || j != cntSend || emcStop != 0 || logOverflow ||
			dbg_overflow != 0U || diagnostics.invalidUpdateCount != 0U ||
			Control_GetDistanceFusionOutputGuardCount() != 0U)
			logClosure.reason = 3U;
		else if (encoderIntervalFault)
			logClosure.reason = 6U;
		else if (!RunGuard_PrimaryImuCalibrated(runImuCalibrationValid,
			runImuCalibrationSamples, runImuCalibrationReadErrors,
			IMU_CALIBRATION_SAMPLE_COUNT))
			logClosure.reason = 9U;
		else if (!goalBracketFound || logClosure.goalS_mm <= 0.0f)
			logClosure.reason = 5U;
		else if (!isfinite(logClosure.goalX_mm) || fabsf(logClosure.goalX_mm) > 20.0f)
			logClosure.reason = 8U;
		else if (PRIMARY_DISTANCE_SCALE_VERIFIED == 0U)
			logClosure.reason = 10U;
		else
		{
			logClosure.reason = 0U;
			logClosure.valid = 1U;
		}
	}
	if (in_cross && cross_count < CROSSSEG_MAX)
	{
		cross_end_mm[cross_count] = dist_mm;
		cross_count++;
	}

	f_close(&fil);
	createLog();
	if (!create_log_ready)
	{
		printf("endLog: createLog failed\r\n");
		return;
	}
	fresult = f_open(&fil, "temp", FA_OPEN_EXISTING | FA_READ);
	if (fresult != FR_OK)
	{
		printf("f_open error in endLog\r\n");
		f_close(&fil_W);
		return;
	}
	clearXYcie();
	fusedPose = (HeadingPose){0.0f, 0.0f, 0.0f};
	beforeTime = 0;
	beforeSpeed = 0;
	dist_mm = 0.0f;
	travelled_mm = 0.0f;
	previousPulse_p = 0;

	// pass2: 抽出した区間の前後100mmを直線ROCに補正してCSV出力
	for (j = 0; j < cntSend; j++)
	{
		fresult = f_read(&fil, log, sizeof(log), &readByte);
		if (readByte != LOG_SIZE)
		{
			break;
		}
		if (fresult != FR_OK)
		{
			printf("f_read error in endLog\r\n");
			f_close(&fil_W);
			f_close(&fil);
			return;
		}
		logaddress = log;
		logReadRecord(&rec);

		time = rec.cntlog;
		speed = (int16_t)rec.encCurrentN;
		zg = rec.gyroVal_Z;

		if (abs((int32_t)speed - (int32_t)beforeSpeed) > 500)
		{
			speed = beforeSpeed;
			rec.encCurrentN = (uint16_t)beforeSpeed;
		}
		beforeSpeed = speed;

		dt = (float)(uint16_t)(time - beforeTime) / 1000.0f;
		log_roc = calcROC(speed, zg, dt);

		int32_t deltaPulse_p;
		if (optimalTrace == BOOST_NONE)
		{
			int32_t currentPulse_p = (int32_t)rec.encTotalOptimal;
			deltaPulse_p = currentPulse_p - previousPulse_p;
			previousPulse_p = currentPulse_p;
			travelled_mm += (float)deltaPulse_p / PULSE_MILLIMETER;
		}
		else
		{
			deltaPulse_p = (int32_t)rec.encCurrentCorr_p * (int32_t)(uint16_t)(time - beforeTime);
		}
		calcXYcie(deltaPulse_p, zg, dt);
		HeadingPose_Advance(&fusedPose, (float)deltaPulse_p / PULSE_MILLIMETER,
			zg - ((optimalTrace == BOOST_NONE) ? headingEstimator.biasDps : 0.0f), dt);
		log_x = xycie.x;
		log_y = xycie.y;
		log_x_fused = fusedPose.x_mm;
		log_y_fused = fusedPose.y_mm;
		dist_mm += calcDlMm(speed, dt);
		beforeTime = time;

		bool straight_zone = false;
		for (uint16_t i = 0; i < cross_count; i++)
		{
			float start_mm = cross_start_mm[i] - CROSS_STRAIGHT_MM;
			if (start_mm < 0.0f)
			{
				start_mm = 0.0f;
			}
			float end_mm = cross_end_mm[i] + CROSS_STRAIGHT_MM;
			if (dist_mm >= start_mm && dist_mm <= end_mm)
			{
				straight_zone = true;
				break;
			}
		}
		if (straight_zone || rec.courseMarker == 3)
		{
			log_roc = ROC_STRAIGHT_MAX;
		}

#define LOG_FORMAT_VALUE_U8(value) (value)
#define LOG_FORMAT_VALUE_U16(value) (value)
#define LOG_FORMAT_VALUE_S16(value) (value)
#define LOG_FORMAT_VALUE_U32(value) ((int32_t)(value))
#define LOG_FORMAT_VALUE_F32(value) (value)
#define LOG_CSV_ARG_STORED(type, name, fmt, expr) , LOG_FORMAT_VALUE_##type(rec.name)
#define LOG_CSV_ARG_DERIVED(type, name, fmt, expr) , LOG_FORMAT_VALUE_##type(expr)
		int csvLength = snprintf((char *)logStr, sizeof(logStr),
            (char *)formatLog LOG_FIELD_LIST(LOG_CSV_ARG_STORED, LOG_CSV_ARG_DERIVED));
#undef LOG_CSV_ARG_STORED
#undef LOG_CSV_ARG_DERIVED
#undef LOG_FORMAT_VALUE_U8
#undef LOG_FORMAT_VALUE_U16
#undef LOG_FORMAT_VALUE_S16
#undef LOG_FORMAT_VALUE_U32
#undef LOG_FORMAT_VALUE_F32

		if (csvLength < 0 || (size_t)csvLength >= sizeof(logStr))
		{
			printf("CSV log line truncated in endLog: %d bytes\r\n", csvLength);
			f_close(&fil_W);
			f_close(&fil);
			return;
		}

		if (f_puts(logStr, &fil_W) < 0)
		{
			printf("f_puts error in endLog\r\n");
			f_close(&fil_W);
			f_close(&fil);
			return;
		}
	}

	bool csvComplete = (j == cntSend && f_sync(&fil_W) == FR_OK);
	FRESULT csvCloseResult = f_close(&fil_W);
	f_close(&fil);
	if (!csvComplete || csvCloseResult != FR_OK)
	{
		printf("endLog: CSV incomplete\r\n");
		return;
	}

	f_unlink("temp");

	cntSend = 0;
	lastRunSaved = true;
}
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
	uint8_t *tp;

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
			if (strstr(fno.fname, ".csv") != NULL)
			{
				// csvファイルのとき
				tp = strtok(fno.fname, ".");              // 拡張子削除
				fileNumbers[endFileIndex] = atoi(tp);     // 文字列を数値に変換して保存
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
		if (fileCount > FILENUMBER_LIMIT)
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
				f_unlink(deleteName);
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
// モジュール名 logAppendText
// 処理概要     文字列をバッファーへ境界チェック付きで追記する
// 引数         buffer:追記先, bufferSize:追記先サイズ, text:追記文字列
// 戻り値       true:追記成功 false:容量不足
/////////////////////////////////////////////////////////////////////
static bool logAppendText(char *buffer, size_t bufferSize, const char *text)
{
	size_t currentLength = strlen(buffer);
	size_t textLength = strlen(text);
	if (currentLength >= bufferSize || textLength >= (bufferSize - currentLength))
	{
		return false;
	}
	memcpy(&buffer[currentLength], text, textLength + 1U);
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 setLogStr
// 処理概要     ヘッダー文字列に列名とフォーマットを追加する
// 引数         column:ヘッダー文字列 format:フォーマット文字列
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void setLogStr(char *column, char *format)
{
	char columnStr[30];	// ヘッダー文字列を一時的に格納するバッファ
	char formatStr[30];	// フォーマット文字列を一時的に格納するバッファ

	int columnLength = snprintf(columnStr, sizeof(columnStr), "%s,", column);
	int formatLength = snprintf(formatStr, sizeof(formatStr), "%s,", format);
	if (columnLength < 0 || (size_t)columnLength >= sizeof(columnStr) ||
		formatLength < 0 || (size_t)formatLength >= sizeof(formatStr))
	{
		logHeaderOverflow = true;
		return;
	}
	if (!logAppendText(columnTitle, sizeof(columnTitle), columnStr) ||
		!logAppendText(formatLog, sizeof(formatLog), formatStr))
	{
		logHeaderOverflow = true;
	}
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
	if (!logAppendText(columnTitle, sizeof(columnTitle), headerStr)) logHeaderOverflow = true;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 setLogHeaderStrU
// 処理概要     ログCSVのヘッダーに符号なし整数の"変数名=値"を追記する
// 引数         name:変数名 value:値
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void setLogHeaderStrU(char *name, uint32_t value)
{
	char headerStr[64];

	snprintf(headerStr, sizeof(headerStr), "%s=%lu,", name, (unsigned long)value);
	if (!logAppendText(columnTitle, sizeof(columnTitle), headerStr)) logHeaderOverflow = true;
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
	if (!logAppendText(columnTitle, sizeof(columnTitle), headerStr)) logHeaderOverflow = true;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 setLogHeaderStrFPrecision
// 処理概要     ログCSVのヘッダーに指定桁数の浮動小数値を追記する
// 引数         name:変数名 value:値 precision:小数点以下の桁数
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void setLogHeaderStrFPrecision(char *name, float value, uint8_t precision)
{
	char headerStr[64];

	snprintf(headerStr, sizeof(headerStr), "%s=%.*f,", name, (int)precision, (double)value);
	if (!logAppendText(columnTitle, sizeof(columnTitle), headerStr)) logHeaderOverflow = true;
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
	if (!logAppendText(columnTitle, sizeof(columnTitle), headerStr)) logHeaderOverflow = true;
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
	// アクティブバッファに値を格納し、書き込み位置を進める
	activeBuf[logBuffIndex++] = data;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 send16bit
// 処理概要     16bit変数を1バイトごとに分割してアクティブバッファに送る
// 引数         変換する16bit変数
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void send16bit(uint16_t data)
{
	activeBuf[logBuffIndex++] = (data >> 8); // 上位バイトをバッファに格納
	activeBuf[logBuffIndex++] = data;        // 下位バイトをバッファに格納
}
/////////////////////////////////////////////////////////////////////
// モジュール名 send32bit
// 処理概要     32bit変数を1バイトごとに分割してアクティブバッファに送る
// 引数         変換する32bit変数
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void send32bit(uint32_t data)
{
	activeBuf[logBuffIndex++] = (data >> 24); // 最上位バイトをバッファに格納
	activeBuf[logBuffIndex++] = (data >> 16); // 上位から2番目のバイトをバッファに格納
	activeBuf[logBuffIndex++] = (data >> 8);  // 上位から3番目のバイトをバッファに格納
	activeBuf[logBuffIndex++] = data;         // 最下位バイトをバッファに格納
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
