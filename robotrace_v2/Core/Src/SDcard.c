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
#ifdef LOG_RUNNING_WRITE
uint8_t logBuffer[2][BUFFER_SIZE_LOG];	// バッファは2個で固定
uint8_t *activeBuf = logBuffer[0];		// 書き込み中のバッファ
uint8_t *flushBuf = logBuffer[1];		// SD書き込み待ちバッファ
int16_t logBuffIndex = 0;				// 一時記録バッファ書込アドレス
uint32_t logBuffSendIndex = 0;			// flushBufに溜まったバイト数
volatile bool sendSD = false;			// flushBufをSDへ送るフラグ(割込み共有)
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
#ifndef LOG_RUNNING_WRITE
typedef struct
{
	uint16_t index;
	int32_t distance;
	uint8_t marker;
} markerData;
markerData markerVal[BUFFER_SIZE_MARKER]; // 綴りの誤りを修正
uint16_t markerValIndex = 0;
#endif
bool markerOverflow = false; // マーカーバッファ上限超過フラグ

// ログファイルナンバー
int16_t fileNumbers[FILENUMBER_NUM];
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
	FRESULT fresult;
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
#ifdef LOG_RUNNING_WRITE
	setLogStr("cntlog", "%d");
	setLogStr("encCurrentN", "%d");
	setLogStr("gyroVal_Z", "%f");
	setLogStr("courseMarker", "%d");
	setLogStr("encTotalOptimal", "%d");
	setLogStr("ROC", "%f");

	setLogStr("targetSpeed", "%d");
	setLogStr("optimalIndex", "%d");
	setLogStr("lineTraceCtrl", "%d");
	// setLogStr("velocity", "%f");
	setLogStr("targetAngularvelo", "%d");
	setLogStr("motorpwmL", "%d");
	setLogStr("motorpwmR", "%d");

	setLogStr("acceleVal_X", "%f");
	setLogStr("acceleVal_Y", "%f");
	setLogStr("slipRatio", "%f");
	setLogStr("slipRatioLat", "%f");
	setLogStr("slipFlag", "%d");
	setLogStr("slipFlagLat", "%d");
	setLogStr("motorCurrentL", "%f");
	setLogStr("motorCurrentR", "%f");
	setLogStr("slipDistScaleF", "%f");
	setLogStr("distEncRaw_p", "%d");
	setLogStr("distCorr_p", "%d");
	setLogStr("distSlipLoss_p", "%d");
	setLogStr("encCurrentCorr_p", "%d");
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
#ifndef LOG_RUNNING_WRITE
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
#endif
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
	logBuffSendIndex = logBuffIndex;	// バッファのバイト数を記録
	sendSD = false;						// 書き込み要求をリセット
	logOverflow = false;				// ログバッファ状態フラグをリセット
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

		// 追記するデータ量でバッファがあふれる場合は事前に入れ替えを行う
		if (logBuffIndex + requiredSize > BUFFER_SIZE_LOG)
		{
			if (sendSD)
			{
				writeLogPuts();			// flushBufが書き込み中なら即時書き込みを促して空きを確保する
				if (sendSD)
				{
					logOverflow = true;		// 空き確保に失敗したことを記録して後段で検知する
					return;				// 空きができるまで追記を保留し、バッファ破壊を防ぐ
				}
			}
			logBuffSendIndex = logBuffIndex; // 書き込み待ちバッファのサイズを記録
			uint8_t *tmp = flushBuf; // flushBufのポインタを退避
			flushBuf = activeBuf; // 現在のバッファをflushBufに切り替え
			activeBuf = tmp; // 退避したバッファを新たなactiveに
			logBuffIndex = 0; // 新バッファの書込位置をリセット
			sendSD = true; // SD書き込みを要求
		}

		// バッファ配列に保存
		va_start(args, f);
		// 8bitデータをバッファへ送る
		for (cnt = 0; cnt < c; cnt++)
			send8bit((uint8_t)va_arg(args, int)); // 可変長引数の型昇格に合わせる
		// 16bitデータをバッファへ送る
		for (cnt = 0; cnt < s; cnt++)
			send16bit((uint16_t)va_arg(args, int)); // 可変長引数の型昇格に合わせる
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
	FRESULT fresult;		// f_writeの戻り値
	UINT writtenlog = 0;	// 実際に書き込んだサイズ

	if (!modeLOG && !sendSD)
	{
		return; // ログ停止中で書き込み要求が無い場合は処理不要
	}

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

#ifdef LOG_RUNNING_WRITE
	FRESULT fresult;
	FIL fil;
	uint8_t log[LOG_SIZE];
	char logStr[256];
	UINT readByte, writtenlog; // FatFsの読み書きサイズ
	uint16_t j, cnt;
	uint16_t marker, time, beforeTime = 0, speed, beforeSpeed = 0;
	uint32_t distance;
	float dt, zg;
	uint8_t rocIdx = LOG_NUM_FLOAT;
	uint8_t xIdx = LOG_NUM_FLOAT + 1;
	uint8_t yIdx = LOG_NUM_FLOAT + 2;
	static union
	{
		float f;
		uint32_t i;
	} ftoi;

	uint8_t logval8[20];
	uint16_t logval16[20];
	uint32_t logval32[20];
	float logvalf[20];

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

		// コース解析に使用する変数を取得
		marker = logval8[0];
		time = logval16[0];
		speed = logval16[1];
		distance = logval32[0];
		zg = logvalf[0];

		// 異常値補正
		if (abs(speed - beforeSpeed) > 500)
		{
			speed = beforeSpeed;
			logval16[1] = beforeSpeed;
		}
		beforeSpeed = speed;

		dt = (float)(time - beforeTime) / 1000.0f; // 経過時間

		cnt = LOG_NUM_FLOAT;	// float型のログの続きを使用する
		logvalf[cnt++] = calcROC(speed, zg, dt); // 曲率半径を計算

		calcXYcie((int16_t)logval32[4], zg, dt);	// xy座標を計算
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
			logvalf[rocIdx],	// ROC

			logval8[1],		// targetSpeed
			logval16[2],	// optimalIndex
			(int16_t)logval16[3],	// lineTraceCtrl
			(int16_t)logval16[4],	// targetAngularvelo
			(int16_t)logval16[5],	// motorpwmL
			(int16_t)logval16[6],	// motorpwmR

			logvalf[1],		// acceleVal_X
			logvalf[2],		// acceleVal_Y
			logvalf[3],		// slipRatio(LPF後)
			logvalf[4],		// slipRatioLat(LPF後)
			logval8[2],		// slipFlag
			logval8[3],		// slipFlagLat
			logvalf[5],		// motorCurrentL
			logvalf[6],		// motorCurrentR
			logvalf[7],		// slipDistScaleF
			logval32[1],	// distEncRaw_p
			logval32[2],	// distCorr_p
			logval32[3],	// distSlipLoss_p
			logval32[4],	// encCurrentCorr_p

			logvalf[xIdx],	// x
			logvalf[yIdx]	// y
		);

		// 文字列をSDカードに送信
		f_puts(logStr, &fil_W);
	}

	f_close(&fil_W); // ログファイル(csv)
	f_close(&fil);	 // 一時ファイル

	f_unlink("temp"); // 一時ファイルを削除
	
	// 連続走行時にCSV変換ループが累積しないよう送信カウンタをリセット
	cntSend = 0;

#else
	createLog();	 // ログファイル作成
	writeLogPrint(); // ログ書き込み
	f_close(&fil_W);
#endif
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
	FRESULT fresult;
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
	FRESULT fresult;

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
	activeBuf[logBuffIndex++] = (data >> 8); // 上位バイトをバッファに格納
	activeBuf[logBuffIndex++] = data;        // 下位バイトをバッファに格納
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
	activeBuf[logBuffIndex++] = (data >> 24); // 最上位バイトをバッファに格納
	activeBuf[logBuffIndex++] = (data >> 16); // 上位から2番目のバイトをバッファに格納
	activeBuf[logBuffIndex++] = (data >> 8);  // 上位から3番目のバイトをバッファに格納
	activeBuf[logBuffIndex++] = data;         // 最下位バイトをバッファに格納
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

	s = (uint16_t)((uint8_t)*logaddress++ * 0x100 +	(uint8_t)*logaddress++);

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
