//====================================//
// インクルード
//====================================//
#include "SDcard.h"
#include "fatfs.h"
#include <string.h>
#include <stdio.h>
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
uint8_t logBuffer[2][BUFFER_SIZE_LOG];               // バッファは2個で固定
uint8_t *activeBuf = logBuffer[0];                   // 書き込み中のバッファ
uint8_t *flushBuf = logBuffer[1];                    // SD書き込み待ちバッファ
int16_t logBuffIndex = 0;                            // 一時記録バッファ書込アドレス
uint32_t logBuffSendIndex = 0;                       // flushBufに溜まったバイト数
volatile bool sendSD = false;                        // flushBufをSDへ送るフラグ(割込み共有)
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
logData logVal[BUFFER_SIZE_LOG]; // 綴りの誤りを修正
#endif
uint16_t logValIndex = 0;
bool logOverflow = false; // ログバッファ上限超過フラグ

typedef struct
{
	uint16_t index;
	int32_t distance;
	uint8_t marker;
} markerData;
markerData markerVal[BUFFER_SIZE_MARKER]; // 綴りの誤りを修正
uint16_t markerValIndex = 0;
bool markerOverflow = false; // マーカーバッファ上限超過フラグ

// ログファイルナンバー
int16_t fileNumbers[1000];
int16_t fileIndexLog = 0; // 現在使用しているログ番号
int16_t endFileIndex = 0; // ログの最終番号

// カウンタ
uint8_t cntLog = 0;
int32_t encLog = 0;
bool getFileNumbersError = false; // getFileNumbersでエラーが発生した際のフラグ

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
	DWORD fre_clust;
	uint32_t total, free_space;

	// SDcardをマウント
	fresult = f_mount(&fs, "", 0);
	if (fresult == FR_OK)
	{
		// マウント成功
		initMSD = true;
		printf("SD CARD mounted successfully...\r\n");

		// 空き容量を計算
		fresult = f_getfree("", &fre_clust, &pfs);							// cluster size
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
	DIR dir;         // Directory
	FILINFO fno; // File Info
	uint8_t *tp, fileName[10];
	uint16_t fileNumber = 0;

	fresult = f_opendir(&dir, "/"); // directory open
	if (fresult != FR_OK)
	{
		printf("failed to open root directory: %d\r\n", fresult);
		// SDカードが未接続などでディレクトリを開けないため、ログ作成を中断する
		return;
			}

	do
	{
		fresult = f_readdir(&dir, &fno);
		if (fresult != FR_OK)
	{
			// エラーが発生した場合はループを抜けてファイル探索を中断
			break;
		}
		if (fno.fname[0] == 0)
		{
			// ファイル名が空の場合はループを終了
			break;
		}
		tp = strtok(fno.fname, ".");	// 拡張子削除
		if (tp != NULL && atoi(tp) > fileNumber)
		{								// 番号比較
			fileNumber = atoi(tp);		// 文字列を数値に変換
		}
	} while (fno.fname[0] != 0);		// ファイルの有無を確認

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

	snprintf((char *)fileName, sizeof(fileName), "%d", fileNumber); // バッファサイズを指定して安全に文字列化
	strncat((char *)fileName, ".csv", sizeof(fileName) - strlen((char *)fileName) - 1); // バッファサイズを指定して安全に拡張子を追加
	fresult = f_open(&fil_W, fileName, FA_OPEN_ALWAYS | FA_WRITE); // create file
	if (fresult != FR_OK)
	{
		// ファイルオープンに失敗した場合はログ作成を中止する
		return; // エラーが発生したため処理を終了
			}

	columnTitle[0] = 0; // バッファを安全に初期化
	formatLog[0] = 0;   // バッファを安全に初期化
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
void writeMarkerPos(uint32_t distance, uint8_t marker)
{
	// バッファ上限チェック
    if (markerValIndex < BUFFER_SIZE_MARKER) // 綴りの誤りを修正
	{
		markerVal[markerValIndex].index = logValIndex; // ログの位置を記録
		markerVal[markerValIndex].distance = distance;  // 走行距離を記録
		markerVal[markerValIndex].marker = marker;      // マーカー種別を記録
		markerValIndex++;                               // インデックス更新
	}
	else
	{
		markerOverflow = true; // 上限超過を記録
	}
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
	if (fresult != FR_OK)
	{
		printf("error opening log file: %d\r\n", fresult); // エラー内容を出力
		initMSD = false; // ファイルオープンに失敗した場合はmicroSDを使用不可とする
		return;          // ログ初期化を中止
			}
        logBuffIndex = 0;                                             // 書込位置を初期化
        activeBuf = logBuffer[0];                                     // アクティブバッファを初期化
        flushBuf = logBuffer[1];                                      // フラッシュバッファを初期化
	logBuffSendIndex = logBuffIndex;                                         // バッファのバイト数を記録
        sendSD = false;                                               // 書き込み要求をリセット
#else
    // 構造体配列の初期化
    memset(&logVal, 0, sizeof(logData) * BUFFER_SIZE_LOG);     // 綴りの誤りを修正
    memset(&markerVal, 0, sizeof(markerData) * BUFFER_SIZE_MARKER); // 綴りの誤りを修正
	logValIndex = 0;
	markerValIndex = 0;
#endif
}
/////////////////////////////////////////////////////////////////////
// モジュール名 writeLogBufferPuts
// 処理概要     保存する変数をバッファに転送する
// 引数         c:8bit変数の数s:16bit変数の数i:32bit変数の数f:float変数の数
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
#ifdef LOG_RUNNING_WRITE
void writeLogBufferPuts(uint8_t c, uint8_t s, uint8_t i, uint8_t f, ...)
{
	va_list args;
	uint8_t cnt = 0;
	static union
	{
		float f;
		uint32_t i;
	} ftoi;

	if (modeLOG)
	{
                uint16_t requiredSize = c + (s * sizeof(uint16_t)) + (i * sizeof(uint32_t)) + (f * sizeof(float)); // 引数数に応じたバッファ必要量を算出
		// 	バッファ配列に保存
		va_start(args, f);
		// logBuffer[0] = va_arg( args, uint8_t* );
		// 8bitデータをバッファへ送る
		for (cnt = 0; cnt < c; cnt++)
			send8bit(va_arg(args, unsigned int)); // 可変長引数の型昇格に合わせる
		// 16bitデータをバッファへ送る
		for (cnt = 0; cnt < s; cnt++)
			send16bit(va_arg(args, unsigned int)); // 可変長引数の型昇格に合わせる
		// 32bitデータをバッファへ送る
		for (cnt = 0; cnt < i; cnt++)
			send32bit(va_arg(args, uint32_t));
		// floatデータをバッファへ送る
		for (cnt = 0; cnt < f; cnt++)
		{
			ftoi.f = va_arg(args, double); // 共用体を使用してfloat型のビット操作をできるようにする
			send32bit(ftoi.i);
		}
		va_end(args);
		cntSend++; // 書き込み回数をカウント

		// バッファが512バイト付近まで溜まったら確認
		if (logBuffIndex + requiredSize > BUFFER_SIZE_LOG && !sendSD)
		{
			logBuffSendIndex = logBuffIndex;         // 書き込み待ちバッファのサイズを記録
			uint8_t *tmp = flushBuf;                 // flushBufのポインタを退避
			flushBuf = activeBuf;                    // 現在のバッファをflushBufに切り替え
			activeBuf = tmp;                         // 退避したバッファを新たなactiveに
			logBuffIndex = 0;                        // 新バッファの書込位置をリセット
			sendSD = true;                           // SD書き込みを要求
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
	FRESULT fresult;                            // f_writeの戻り値
	UINT writtenlog = 0; // 実際に書き込んだサイズ

	if (modeLOG)
	{
		if (sendSD) // 書き込み要求がある場合
		{
			fresult = f_write(&fil_W, flushBuf, logBuffSendIndex, &writtenlog); // flushBufをSDへ書き出す
			if (fresult != FR_OK || writtenlog != logBuffSendIndex)
			{
				sendSD = false; // エラー時は要求を解除
				return;
			}
			sendSD = false; // 書き込み完了フラグをクリア
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
		// バッファ上限チェック
        if (logValIndex < BUFFER_SIZE_LOG) // 綴りの誤りを修正
		{
			logVal[logValIndex].time = cntLog;                        // ログ時刻を記録
			logVal[logValIndex].speed = encCurrentN;                  // 現在速度を記録
			logVal[logValIndex].zg = BMI088val.gyro.z;                // 角速度を記録
			logVal[logValIndex].opIndex = optimalIndex;               // 最適軌道番号を記録
			logVal[logValIndex].targetSpeed = targetSpeed;           // 目標速度を記録
			logVal[logValIndex].spare = (int16_t)(motorCurrentL * 10000); // 予備情報を記録
			logValIndex++;                                           // インデックス更新
		}
		else
		{
			logOverflow = true; // 上限超過を記録
		}
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
        snprintf((char *)logStr, sizeof(logStr), (char *)formatLog, // バッファサイズを指定して安全に文字列化
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
	UINT readByte, writtenlog; // FatFsの読み書きサイズ
	uint16_t j, cnt;
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

	while (sendSD) // 溜まったバッファをすべて書き出す
		writeLogPuts(); // 未処理バッファを書き込む

	logBuffSendIndex = logBuffIndex;                                         // バッファのバイト数を記録
	fresult = f_write(&fil_W, activeBuf, logBuffSendIndex, &writtenlog);     // 残りのデータを送信
	if (fresult != FR_OK)
	{
		printf("f_write error in endLog\r\n"); // 追記: エラー通知
	}
	f_close(&fil_W); // 一時ファイルを閉じる

	createLog(); // ログファイル(csv)を作成

	fresult = f_open(&fil, "temp", FA_OPEN_EXISTING | FA_READ); // ログファイルを開く
	if (fresult != FR_OK)
	{
		printf("f_open error in endLog\r\n"); // 追記: エラー通知
		f_close(&fil_W); // 作成したログファイルを閉じる
		return; // 一時ファイルが読めないと変換できないため中断
		}
	clearXYcie(); // xy座標クリア
	for (j = 0; j < cntSend; j++)
	{
		fresult = f_read(&fil, log, sizeof(log), &readByte); // 読み込んだバイト数を取得するためポインタを渡す
		if (fresult != FR_OK)
	{
			printf("f_read error in endLog\r\n"); // 追記: エラー通知
			f_close(&fil_W); // CSVファイルを閉じる
			f_close(&fil); // 一時ファイルを閉じる
			return; // データが読めないと解析不能なため中断
			}
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
        snprintf((char *)logStr, sizeof(logStr), (char *)formatLog, // バッファサイズを指定して安全に文字列化
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
	DIR dir;         // Directory
	FILINFO fno; // File Info
	FRESULT fresult;
	uint8_t fileName[10];
	uint8_t *tp, i;

    fresult = f_opendir(&dir, "/"); // directory open
	if (fresult == FR_OK)
	{
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
	uint8_t columnStr[30];	// ヘッダー文字列を一時的に格納するバッファ
	uint8_t formatStr[30];	// フォーマット文字列を一時的に格納するバッファ

       // copy str to local variable
       snprintf((char *)columnStr, sizeof(columnStr), "%s", column); // バッファサイズを指定して安全にコピー
       snprintf((char *)formatStr, sizeof(formatStr), "%s", format); // バッファサイズを指定して安全にコピー

       strncat((char *)columnStr, ",", sizeof(columnStr) - strlen((char *)columnStr) - 1); // バッファサイズを指定して安全に結合
       strncat((char *)formatStr, ",", sizeof(formatStr) - strlen((char *)formatStr) - 1); // バッファサイズを指定して安全に結合
       strncat((char *)columnTitle, (char *)columnStr, sizeof(columnTitle) - strlen((char *)columnTitle) - 1); // バッファサイズを指定して安全に結合
       strncat((char *)formatLog, (char *)formatStr, sizeof(formatLog) - strlen((char *)formatLog) - 1);       // バッファサイズを指定して安全に結合
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
	FRESULT fresult;

	fresult = f_open(&fil_T, "test.csv", FA_OPEN_ALWAYS | FA_WRITE); // create file
	uint32_t start = HAL_GetTick(); // SPI待ちにタイムアウトを設定
	while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY)
	{
		if (HAL_GetTick() - start > SPI_TIMEOUT)
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
void createDir(uint8_t *dirName)
{
	FRESULT fresult;
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
#ifdef LOG_RUNNING_WRITE
void send8bit(uint8_t data)
{
	// アクティブバッファに値を格納し、書き込み位置を進める
	activeBuf[logBuffIndex++] = data;
}
#endif
/////////////////////////////////////////////////////////////////////
// モジュール名 send16bit
// 処理概要     16bit変数を1バイトごとに分割してアクティブバッファに送る
// 引数         変換する16bit変数
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
#ifdef LOG_RUNNING_WRITE
void send16bit(uint16_t data)
{
	// バイト順を変換して一時変数に格納
	uint16_t swapped = __builtin_bswap16(data);
	// アクティブバッファに高速コピー
	memcpy(&activeBuf[logBuffIndex], &swapped, sizeof(swapped));
	// 書き込み位置を更新
	logBuffIndex += sizeof(swapped);
}
#endif
/////////////////////////////////////////////////////////////////////
// モジュール名 send32bit
// 処理概要     32bit変数を1バイトごとに分割してアクティブバッファに送る
// 引数         変換する32bit変数
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
#ifdef LOG_RUNNING_WRITE
void send32bit(uint32_t data)
{
	// バイト順を変換して一時変数に格納
	uint32_t swapped = __builtin_bswap32(data);
	// アクティブバッファに高速コピー
	memcpy(&activeBuf[logBuffIndex], &swapped, sizeof(swapped));
	// 書き込み位置を更新
	logBuffIndex += sizeof(swapped);
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
