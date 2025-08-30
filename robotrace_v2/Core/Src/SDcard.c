//====================================//
// インクルード
//====================================//
#include "SDcard.h"
#include "fatfs.h"
#include <string.h>
//====================================//
// グローバル変数の宣
//====================================//
// MicroSD
FATFS fs;
FIL fil_W;
FIL fil_R;

// ログヘッダー
uint8_t columnTitle[512] = "", formatLog[256] = "";

// ログバッファ
#ifdef LOG_RUNNING_WRITE
uint8_t logBuffer[BUFFER_SIZE_LOG], logBufferSend[BUFFER_SIZE_LOG];
uint8_t *logBufferPointa; // RAM保存バッファ用ポインタ
int16_t logBuffIndex = 0;		// 書き込みバッファをリセット // 一時記録バッファ書込アドレス
uint32_t logBuffSendIndex = 0;
bool sendSD = false;
uint16_t cntSend = 0;
uint8_t *logaddress;
#else
typedef struct
{
	uint8_t time;
	uint8_t speed;
	float zg;
	int16_t targetSpeed;
	int16_t opIndex;
	int16_t spare;
} logData;
logData logVal[BUFFER_SIZW_LOG];
#endif
uint16_t logValIndex = 0;

typedef struct
{
	uint16_t index;
	int32_t distance;
	uint8_t marker;
} markerData;
markerData markerVal[BUFFER_SIZW_MARKER];
uint16_t markerValIndex = 0;

// ログファイルナンバー
int16_t fileNumbers[1000];
int16_t fileIndexLog = 0; // 現在使用しているログ番号
int16_t endFileIndex = 0; // ログの最終番号

// カウンタ
uint8_t cntLog = 0;
int32_t encLog = 0;

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
	FRESULT fresult;
	DIR dir;	 // Directory
	FILINFO fno; // File Info
	DWORD fre_clust;
	uint32_t total, free_space;
	uint8_t dirSetting = 0;
	FIL fil_T;

	// SDcardをマウント
	fresult = f_mount(&fs, "", 0);
	if (fresult == FR_OK)
	{
		// マウント成功
		initMSD = true;
		printf("SD CARD mounted successfully...\r\n");

		// 空き容量を計算
		f_getfree("", &fre_clust, &pfs);							// cluster size
		total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5); // total capacity
		printf("SD_SIZE: \t%lu\r\n", total);
		free_space = (uint32_t)(fre_clust * pfs->csize * 0.5); // empty capacity
		printf("SD free space: \t%lu\r\n", free_space);

		getFileNumbers();

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
	FRESULT fresult;
	DIR dir;	 // Directory
	FILINFO fno; // File Info
	uint8_t *tp, fileName[10];
	uint16_t fileNumber = 0;

	f_opendir(&dir, "/"); // directory open

	do
	{
		f_readdir(&dir, &fno);
		tp = strtok(fno.fname, "."); // 拡張子削除
		if (atoi(tp) > fileNumber)
		{						   // 番号比較
			fileNumber = atoi(tp); // 文字列を数値に変換
		}
	} while (fno.fname[0] != 0); // ファイルの有無を確認

	f_closedir(&dir); // directory close

	// ファイルナンバー作成
	if (fileNumber == 0)
	{
		// ファイルが無いとき
		fileNumber = 1;
	}
	else
	{
		// ファイルが有るとき
		fileNumber++; // index pulus
	}

	sprintf(fileName, "%d", fileNumber);						   // 数値を文字列に変換
	strcat(fileName, ".csv");									   // 拡張子を追加
	fresult = f_open(&fil_W, fileName, FA_OPEN_ALWAYS | FA_WRITE); // create file

	strcpy(columnTitle, "");
	strcpy(formatLog, "");
#ifdef LOG_RUNNING_WRITE
	setLogStr("cntlog", "%d");
	setLogStr("encCurrentN", "%d");
	setLogStr("gyroVal_Z", "%f");
	setLogStr("courseMarker", "%d");
	setLogStr("encTotalN", "%d");
	setLogStr("ROC", "%f");

	setLogStr("targetSpeed", "%d");
	setLogStr("optimalIndex", "%d");
	setLogStr("CurrentL", "%f");
	setLogStr("CurrentR", "%f");
	setLogStr("lineTraceCtrl", "%d");
	setLogStr("veloCtrl", "%d");

	setLogStr("x", "%f");
	setLogStr("y", "%f");
#else
	setLogStr("cntlog", "%d");
	setLogStr("encCurrentN", "%d");
	setLogStr("gyroVal_Z", "%f");
	setLogStr("courseMarker", "%d");
	setLogStr("encTotalN", "%d");
	setLogStr("ROC", "%f");
	setLogStr("x", "%f");
	setLogStr("y", "%f");
	setLogStr("CurrentL", "%d");
	setLogStr("CurrentR", "%d");
	// setLogStr("courseMarker",  "%d");
	// setLogStr("encTotalN",    "%d");
#endif
	strcat(columnTitle, "\n");
	strcat(formatLog, "\n");
	f_printf(&fil_W, columnTitle);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 initLog
// 処理概要     バイナリ保存用のファイルを作成
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void writeMarkerPos(uint32_t distance, uint8_t marker)
{
	markerVal[markerValIndex].index = logValIndex;
	markerVal[markerValIndex].distance = distance;
	markerVal[markerValIndex].marker = marker;
	markerValIndex++;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 initLog
// 処理概要     バイナリ保存用のファイルを作成
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void initLog(void)
{
	FRESULT fresult;
#ifdef LOG_RUNNING_WRITE
	fresult = f_open(&fil_W, "temp", FA_OPEN_ALWAYS | FA_WRITE); // create file
#else
	// 構造体配列の初期化
	memset(&logVal, 0, sizeof(logData) * BUFFER_SIZW_LOG);
	memset(&markerVal, 0, sizeof(markerData) * BUFFER_SIZW_MARKER);
	logValIndex = 0;
	markerValIndex = 0;
#endif
}
/////////////////////////////////////////////////////////////////////
// モジュール名 writeLogBufferPuts
// 処理概要     保存する変数をバッファに転送する
// 引数         rec: ログレコード
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
#ifdef LOG_RUNNING_WRITE
void writeLogBufferPuts(const LogRecord *rec)
{
	if (modeLOG)
	{
		memcpy(logBuffer + logBuffIndex, rec, sizeof(LogRecord));	// 構造体をバッファへコピー
		logBuffIndex += sizeof(LogRecord);	// コピーしたサイズ分インデックスを進める
		cntSend++;                              // 送信回数を記録

		if (logBuffIndex + LOG_SIZE > 512 && !sendSD)	// 512バイト(1セクタ)超で送信準備
		{
			logBuffSendIndex = logBuffIndex;        // 送信バイト数を保存
			memcpy(logBufferSend, logBuffer, logBuffSendIndex);     // 送信用配列にコピー
			
			logBuffIndex = 0;		// 書き込みバッファをリセット
			logBufferPointa = logBuffer;	// バッファ先頭にポインタを戻す
			sendSD = true;          // SDカード送信を指示
		}
	}
}
#endif
/////////////////////////////////////////////////////////////////////
// モジュール名 writeLogPuts
// 処理概要     バッファをSDカードに転送する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
#ifdef LOG_RUNNING_WRITE
void writeLogPuts(void)
{
	uint32_t writtenlog = 0;

	if (modeLOG)
	{
		if (sendSD)
		{
			sendSD = false;
			f_write(&fil_W, logBufferSend, logBuffSendIndex, writtenlog);
		}
	}
}
#endif
////////////////////////////////////////////////////////////////////
// モジュール名 writeLogBufferPrint
// 処理概要     保存する変数の値をバッファに転送する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
#ifndef LOG_RUNNING_WRITE
void writeLogBufferPrint(void)
{
	if (modeLOG)
	{
		logVal[logValIndex].time = cntLog;
		logVal[logValIndex].speed = encCurrentN;
		logVal[logValIndex].zg = BMI088val.gyro.z;
		logVal[logValIndex].opIndex = optimalIndex;
		logVal[logValIndex].targetSpeed = targetSpeed;
		logVal[logValIndex].opIndex = (int32_t)(motorCurrentL * 10000);
		// logVal[logValIndex].targetSpeed = (int32_t)(motorCurrentR * 10000);
		logValIndex++;
	}
}
#endif
/////////////////////////////////////////////////////////////////////
// モジュール名 writeLogPrint
// 処理概要     バッファをSDカードに転送する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
#ifndef LOG_RUNNING_WRITE
void writeLogPrint(void)
{
	uint8_t logStr[256];
	uint32_t i, totalTime = 0, distance;
	uint16_t indexM = 0, marker;

	clearXYcie(); // xy座標クリア
	for (i = 0; i < logValIndex; i++)
	{
		totalTime += logVal[i].time;
		calcXYcie(logVal[i].speed, logVal[i].zg, (float)logVal[i].time / 1000);

		if (i == markerVal[indexM].index)
		{
			marker = markerVal[indexM].marker;
			distance = markerVal[indexM].distance;
			indexM++;
		}
		else
		{
			marker = 0;
			distance = 0;
		}

		// 文字列に変換
		sprintf(logStr, formatLog,
				totalTime,
				logVal[i].speed,
				logVal[i].zg,
				marker,
				distance,
				calcROC(logVal[i].speed, logVal[i].zg, (float)logVal[i].time / 1000),

				xycie.x,
				xycie.y,
				logVal[i].opIndex,
				logVal[i].targetSpeed);

		// 文字列をSDカードに送信
		f_puts(logStr, &fil_W);
	}
}
#endif
/////////////////////////////////////////////////////////////////////
// モジュール名 endLog
// 処理概要     ロギング終了処理
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void endLog(void)
{
	initIMU = false; // IMUの使用を停止(SPIが競合するため)
	modeLOG = false; // ログ取得停止
	while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY)
		; // SPIバスが空くまで待つ

#ifdef LOG_RUNNING_WRITE
	FRESULT fresult;
	FIL fil;
	uint8_t log[LOG_SIZE];
	uint8_t logStr[256];
	uint16_t readByte, writtenlog, j, cnt;
	uint16_t marker, time, beforeTime = 0, speed, beforeSpeed = 0;
	uint32_t distance;
	float dt, zg;
	static union
	{
		float f;
		uint32_t i;
	} ftoi;

	uint8_t logval8[10];
	uint16_t logval16[10];
	uint32_t logval32[10];
	float logvalf[10];

	logBuffSendIndex = logBuffIndex;        // 送信バイト数を保存
	memcpy(logBufferSend, logBuffer, logBuffSendIndex);     // 送信用配列にコピー
	f_write(&fil_W, logBufferSend, logBuffSendIndex, writtenlog); // 残りのログをSDカードに送信
	f_close(&fil_W);        // 一時ファイルを閉じる

	createLog(); // ログファイル(csv)を作成

	fresult = f_open(&fil, "temp", FA_OPEN_EXISTING | FA_READ); // ログファイルを開く

	clearXYcie(); // xy座標クリア
	for (j = 0; j < cntSend; j++)
	{
		f_read(&fil, log, sizeof(log), readByte);
		logaddress = log; // 読み込んだ配列の先頭アドレスを取得

		// 型ごとに変数を復元
		for (cnt = 0; cnt < LOG_NUM_8BIT; cnt++)
			logval8[cnt] = logPut8bit();
		for (cnt = 0; cnt < LOG_NUM_16BIT; cnt++)
			logval16[cnt] = logPut16bit();
		for (cnt = 0; cnt < LOG_NUM_32BIT; cnt++)
			logval32[cnt] = logPut32bit();
		for (cnt = 0; cnt < LOG_NUM_FLOAT; cnt++)
		{
			ftoi.i = logPut32bit(); // 共用体を使用してfloat型のビット操作をできるようにする
			logvalf[cnt] = ftoi.f;
		}

		marker = logval8[1];
		time = logval16[0];
		speed = logval16[1];
		distance = logval32[0];
		zg = logvalf[0];

		if (abs(speed - beforeSpeed) > 500)
		{
			speed = beforeSpeed;
			logval16[1] = beforeSpeed;
		}
		beforeSpeed = speed;
		dt = (float)(time - beforeTime) / 1000; // 経過時間

		cnt = LOG_NUM_FLOAT;					 // float型のログの続きを使用する
		logvalf[cnt++] = calcROC(speed, zg, dt); // 曲率半径を計算

		dt = (float)(time - beforeTime) / 1000; // 経過時間
		calcXYcie(speed, zg, dt);				// xy座標を計算
		logvalf[cnt++] = xycie.x;
		logvalf[cnt++] = xycie.y;
		beforeTime = time; // 時間を更新

		// 文字列に変換
		sprintf(logStr, formatLog,
				time,
				speed,
				zg,
				marker,
				distance,
				logvalf[1],

				logval8[0],
				logval16[2],
				(float)logval16[3] / 10000,
				(float)logval16[4] / 10000,
				(int16_t)logval16[5],
				(int16_t)logval16[6],
				logvalf[2],
				logvalf[3]);

		// 文字列をSDカードに送信
		f_puts(logStr, &fil_W);
	}

	f_close(&fil_W); // ログファイル(csv)
	f_close(&fil);	 // 一時ファイル

#else
	createLog();	 // ログファイル作成
	writeLogPrint(); // ログ書き込み
	f_close(&fil_W);
#endif

	initIMU = true;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 getFileNumbers
// 処理概要     ファイル名から番号を取得し配列に格納する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void getFileNumbers(void)
{
	DIR dir;	 // Directory
	FILINFO fno; // File Info
	FRESULT fresult;
	uint8_t fileName[10];
	uint8_t *tp, i;

	fresult = f_opendir(&dir, "/"); // directory open
	if (fresult == FR_OK)
	{
		do
		{
			f_readdir(&dir, &fno);
			if (strstr(fno.fname, ".csv") != NULL)
			{
				// csvファイルのとき
				tp = strtok(fno.fname, ".");		  // 拡張子削除
				fileNumbers[endFileIndex] = atoi(tp); // 文字列を数値に変換
				endFileIndex++;
			}
		} while (fno.fname[0] != 0); // ファイルの有無を確認

		endFileIndex--;
		fileIndexLog = endFileIndex;
	}

	f_closedir(&dir); // directory close
}
/////////////////////////////////////////////////////////////////////
// モジュール名 setLogStr
// 処理概要     ログCSVファイルのヘッダーとprintfのフォーマット文字列を生成
// 引数         column: ヘッダー文字列 format: フォーマット文字列
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void setLogStr(uint8_t *column, uint8_t *format)
{
	uint8_t *columnStr[30], formatStr[30];

	// copy str to local variable
	strcpy(columnStr, column);
	strcpy(formatStr, format);

	strcat(columnStr, ",");
	strcat(formatStr, ",");
	strcat(columnTitle, columnStr);
	strcat(formatLog, formatStr);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 SDtest
// 処理概要     SDカードの読み書きテスト
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void SDtest(void)
{
	FIL fil_T;
	FRESULT fresult;

	fresult = f_open(&fil_T, "test.csv", FA_OPEN_ALWAYS | FA_WRITE); // create file
	while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY)
		;
	f_close(&fil_T);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 createDir
// 処理概要     ホームディレクトリに指定されたディレクトリが存在しなければ作成する
// 引数         ディレクトリ名
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void createDir(uint8_t *dirName)
{
	FRESULT fresult;
	DIR dir;	 // Directory
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
}
/////////////////////////////////////////////////////////////////////
// モジュール名 send8bit
// 処理概要     8bit変数をlogBufferに送る
// 引数         変換する8bit変数
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
#ifdef LOG_RUNNING_WRITE
void send8bit(uint8_t data)
{
	logBuffer[logBuffIndex++] = data;
}
#endif
/////////////////////////////////////////////////////////////////////
// モジュール名 send16bit
// 処理概要     16bit変数を1バイトごとに分割してlogBufferに送る
// 引数         変換する16bit変数
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
#ifdef LOG_RUNNING_WRITE
void send16bit(uint16_t data)
{
	logBuffer[logBuffIndex++] = data >> 8;
	logBuffer[logBuffIndex++] = data & 0xff;
}
#endif
/////////////////////////////////////////////////////////////////////
// モジュール名 send32bit
// 処理概要     32bit変数を1バイトごとに分割してlogBufferに送る
// 引数         変換する32bit変数
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
#ifdef LOG_RUNNING_WRITE
void send32bit(uint32_t data)
{
	logBuffer[logBuffIndex++] = data >> 24;
	logBuffer[logBuffIndex++] = (data & 0x00ff0000) >> 16;
	logBuffer[logBuffIndex++] = (data & 0x0000ff00) >> 8;
	logBuffer[logBuffIndex++] = data & 0x000000ff;
}
#endif
/////////////////////////////////////////////////////////////////////
// モジュール名 logPut8bit
// 処理概要     8bit変数を16bitに変換する
// 引数         なし
// 戻り値       変換したuint8_t型
/////////////////////////////////////////////////////////////////////
#ifdef LOG_RUNNING_WRITE
uint8_t logPut8bit(void)
{
	return *logaddress++;
}
#endif
/////////////////////////////////////////////////////////////////////
// モジュール名 logPut16bit
// 処理概要     8bit変数を16bitに変換する
// 引数         なし
// 戻り値       変換したuint16_t型
/////////////////////////////////////////////////////////////////////
#ifdef LOG_RUNNING_WRITE
uint16_t logPut16bit(void)
{
	uint16_t s;

	s = (uint16_t)((uint8_t)*logaddress++ * 0x100 +
				   (uint8_t)*logaddress++);

	return s;
}
#endif
/////////////////////////////////////////////////////////////////////
// モジュール名 logPut32bit
// 処理概要     8bit変数を32bitに変換する
// 引数         なし
// 戻り値       変換したuint32_t型
/////////////////////////////////////////////////////////////////////
#ifdef LOG_RUNNING_WRITE
uint32_t logPut32bit(void)
{
	uint32_t i;

	i = (uint32_t)(uint8_t)*logaddress++ * 0x1000000;
	i += (uint32_t)(uint8_t)*logaddress++ * 0x10000;
	i += (uint32_t)(uint8_t)*logaddress++ * 0x100;
	i += (uint32_t)(uint8_t)*logaddress++;

	return i;
}
#endif