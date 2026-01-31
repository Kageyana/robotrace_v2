//====================================//
// インクルード
//====================================//
#include "SDcard.h"
#include "sd_functions.h"
#include "stdio.h"
#include <stdint.h>
//====================================//
// グローバル変数の宣
//====================================//
// MicroSD
FIL fil_W;
FIL fil_R;

// ログヘッダー
char columnTitle[2048] = "", formatLog[256] = "";

// ログバッファ
// Log buffers
#define LOG_BUFFER_COUNT 3
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
int16_t fileIndexLog = 0; // 現在使用しているログ番号
int16_t endFileIndex = 0; // ログの最終番号

// カウンタ
uint8_t cntLog = 0;
int32_t encLog = 0;
bool getFileNumbersError = false; // getFileNumbersでエラーが発生した際のフラグ

static volatile bool sd_fatfs_locked = false;
static volatile bool sd_analysis_active = false;

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
static uint8_t *logGetFreeBuffer(void);

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
// モジュール名 createLog
// 処理概要     ログファイルを作成する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void createLog(void)
{
	FRESULT fresult;		// f_write status
	char fileName[10];
	static uint16_t fileNumber = 0;

	if (fileNumber == 0)
	{
		// 追加: ログが1つも無いSDでも落ちないようにガード
		if (endFileIndex >= 0)
		{
			fileNumber = (uint16_t)(fileNumbers[endFileIndex] + 1);
		}
		else
		{
			fileNumber = 1;
		}
	}
	else
	{
		fileNumber++; // 次のログ番号に更新
	}

	// 最新ログ番号+1にして絵尾久ファイル名を生成 (バッファサイズを指定して安全に文字列化)
	snprintf((char *)fileName, sizeof(fileName), "%d", fileNumber);
	// バッファサイズを指定して安全に拡張子を追加
	strncat((char *)fileName, ".csv", sizeof(fileName) - strlen((char *)fileName) - 1);
	fresult = f_open(&fil_W, fileName, FA_OPEN_ALWAYS | FA_WRITE); // create file
	if (fresult != FR_OK)
	{
		// ファイルオープンに失敗した場合はログ作成を中止する
		return; // エラーが発生したため処理を終了
	}
	// 追加: 作成したログ番号を一覧に反映（savedLogNo 計算の整合を取る）
	const int16_t maxN = (int16_t)(sizeof(fileNumbers) / sizeof(fileNumbers[0]));
	if (endFileIndex < (maxN - 1))
	{
		endFileIndex++;
		fileNumbers[endFileIndex] = (int16_t)fileNumber;
		fileIndexLog = endFileIndex;
	}

	columnTitle[0] = 0; // バッファを安全に初期化
	formatLog[0] = 0;   // バッファを安全に初期化

	// ログヘッダー
	logBuildColumns();
	// 制御パラメータ
	setLogHeaderStrF("batteryVoltage_V", batteryVoltage_V);

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
    strncat((char *)columnTitle, "\n", sizeof(columnTitle) - strlen((char *)columnTitle) - 1); // バッファサイズを指定して安全に改行を追加
    strncat((char *)formatLog, "\n", sizeof(formatLog) - strlen((char *)formatLog) - 1);       // バッファサイズを指定して安全に改行を追加
	f_printf(&fil_W, columnTitle);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 initLog
// 処理概要     バイナリ保存用のファイルを作成
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// モジュール名 initLog
// 処理概要     バイナリ保存用のファイルを作成
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void initLog(void)
{
	FRESULT fresult;		// f_write status
	// CSV変換ループの実行回数を走行ごとに正しく制御するため送信カウンタをリセット
	cntSend = 0;
	fresult = f_open(&fil_W, "temp", FA_OPEN_ALWAYS | FA_WRITE); // create file
	if (fresult != FR_OK)
	{
		printf("error opening log file: %d\r\n", fresult); // エラー内容を出力
		initMSD = false; // ファイルオープンに失敗した場合はmicroSDを使用不可とする
		return;          // ログ初期化を中止
	}
	logBuffIndex = 0;					// 書込位置を初期化
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
#define LOG_SEND_U8(value) send8bit((uint8_t)(value))
#define LOG_SEND_U16(value) send16bit((uint16_t)(value))
#define LOG_SEND_S16(value) send16bit((uint16_t)(int16_t)(value))
#define LOG_SEND_U32(value) send32bit((uint32_t)(value))
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
	modeLOG = false; // ログ取得停止
	while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY); // SPIバスが空くまで待つ
	FRESULT fresult;		// f_write status
	FIL fil;
	uint8_t log[LOG_SIZE];
	char logStr[256];
	UINT readByte, writtenlog;
	uint16_t j;
	uint16_t time, beforeTime = 0, speed, beforeSpeed = 0;
	float dt, zg;
	float log_roc, log_x, log_y;
	LogRecord rec;

	while (sendSD) // 溜まったバッファをすべて書き出す
		writeLogPuts(); // 未処理バッファを書き込む

	logBuffSendIndex = logBuffIndex;                                         // バッファのバイト数を記録
	fresult = f_write(&fil_W, activeBuf, logBuffSendIndex, &writtenlog);     // 残りのデータを送信
	if (fresult != FR_OK)
	{
		printf("f_write error in endLog\r\n"); // エラー通知
	}
	f_close(&fil_W); // 一時ファイルを閉じる

	createLog(); // ログファイル(csv)を作成

	fresult = f_open(&fil, "temp", FA_OPEN_EXISTING | FA_READ); // 一時ファイルファイルを開く
	if (fresult != FR_OK)
	{
		printf("f_open error in endLog\r\n"); // エラー通知
		f_close(&fil_W); // 作成したログファイルを閉じる
		return; // 一時ファイルが読めないと変換できないため中断
	}

	clearXYcie(); // xy座標クリア
	for (j = 0; j < cntSend; j++)
	{
		fresult = f_read(&fil, log, sizeof(log), &readByte); // 読み込んだバイト数を取得するためポインタを渡す
		if (fresult != FR_OK)
		{
			printf("f_read error in endLog\r\n"); // エラー通知
			f_close(&fil_W); // CSVファイルを閉じる
			f_close(&fil); // 一時ファイルを閉じる
			return; // データが読めないと解析不能なため中断
		}
		logaddress = log;

		// 派生値計算の前にパック済みレコードを復元。
		logReadRecord(&rec);

		time = rec.cntlog;
		speed = rec.encCurrentN;
		zg = rec.gyroVal_Z;

		if (abs(speed - beforeSpeed) > 500)
		{
			speed = beforeSpeed;
			rec.encCurrentN = beforeSpeed;
		}
		beforeSpeed = speed;

		dt = (float)(time - beforeTime) / 1000.0f;
		log_roc = calcROC(speed, zg, dt);

		calcXYcie((int16_t)rec.encCurrentCorr_p, zg, dt);
		log_x = xycie.x;
		log_y = xycie.y;
		beforeTime = time;

		// ヘッダと同じ順でsnprintf引数を展開。
#define LOG_FORMAT_VALUE_U8(value) (value)
#define LOG_FORMAT_VALUE_U16(value) (value)
#define LOG_FORMAT_VALUE_S16(value) (value)
#define LOG_FORMAT_VALUE_U32(value) ((int32_t)(value))
#define LOG_FORMAT_VALUE_F32(value) (value)
#define LOG_CSV_ARG_STORED(type, name, fmt, expr) , LOG_FORMAT_VALUE_##type(rec.name)
#define LOG_CSV_ARG_DERIVED(type, name, fmt, expr) , LOG_FORMAT_VALUE_##type(expr)
		snprintf((char *)logStr, sizeof(logStr), (char *)formatLog LOG_FIELD_LIST(LOG_CSV_ARG_STORED, LOG_CSV_ARG_DERIVED));
#undef LOG_CSV_ARG_STORED
#undef LOG_CSV_ARG_DERIVED
#undef LOG_FORMAT_VALUE_U8
#undef LOG_FORMAT_VALUE_U16
#undef LOG_FORMAT_VALUE_S16
#undef LOG_FORMAT_VALUE_U32
#undef LOG_FORMAT_VALUE_F32

		f_puts(logStr, &fil_W);
	}

	f_sync(&fil_W);
	f_sync(&fil);
	f_close(&fil_W); // ログファイル(csv)
	f_close(&fil);	 // 一時ファイル

	f_unlink("temp"); // 一時ファイルを削除

	// 連続走行時にCSV変換ループが累積しないよう送信カウンタをリセット
	cntSend = 0;

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
	return (uint16_t)(((uint16_t)*logaddress++ << 8) | (uint16_t)*logaddress++);
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
