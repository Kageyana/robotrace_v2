//====================================//
// インクルード
//====================================//
#include "SDcard.h"
#include "fatfs.h"
//====================================//
// グローバル変数の宣
//====================================//
// MicroSD
FATFS     fs;
FIL       fil_W;
FIL       fil_R;

// ログヘッダー
uint8_t   columnTitle[512] = "", formatLog[256] = "";

// ログバッファ
#ifdef	LOG_RUNNING_WRITE
uint8_t		logBuffer[BUFFER_SIZE_LOG], logBufferSend[BUFFER_SIZE_LOG];
uint8_t		*logBufferPointa;		// RAM保存バッファ用ポインタ
int16_t		logBuffIndex=0;			// 一時記録バッファ書込アドレス
uint32_t	logBuffSendIndex=0;
bool		sendSD = false;
#else
typedef struct {
    uint16_t 	time;
    uint8_t 	marker;
    uint8_t 	speed;
    float 		zg;
    int32_t 	distance;
    uint8_t 	target;
    uint16_t 	optimalIndex;
} logData;
logData logVal[BUFFER_SIZW_LOG];
uint32_t	logValIndex=0;
#endif


// ログファイルナンバー
int16_t 	fileNumbers[1000], fileIndexLog = 0, endFileIndex = 0;

// カウンタ
uint16_t    cntLog = 0;
int32_t     encLog = 0;

/////////////////////////////////////////////////////////////////////
// モジュール名 initMicroSD
// 処理概要     SDカードの初期化
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
bool initMicroSD(void) {
	FATFS		*pfs;
	FRESULT		fresult;
	DIR			dir;			// Directory
	FILINFO		fno;			// File Info
	DWORD		fre_clust;
	uint32_t	total, free_space;
	uint8_t		dirSetting = 0;
	FIL			fil_T;

	// SDcardをマウント
	fresult = f_mount(&fs, "", 0);
	if (fresult == FR_OK) {
		// マウント成功
		initMSD = true;
		printf("SD CARD mounted successfully...\r\n");

		// 空き容量を計算
		f_getfree("", &fre_clust, &pfs); // cluster size
		total = (uint32_t)((pfs -> n_fatent - 2) * pfs -> csize * 0.5); // total capacity
		printf("SD_SIZE: \t%lu\r\n", total);
		free_space = (uint32_t)(fre_clust * pfs->csize*0.5);  // empty capacity
		printf("SD free space: \t%lu\r\n", free_space);

		getFileNumbers();

		// ディレクトリを作成
		createDir("setting");
		createDir("plot");

		return true;
	} else {
		// マウント失敗
		initMSD = false;
		printf ("error in mounting SD CARD...\r\n");
		return false;
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 createLog
// 処理概要     ログファイルを作成する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void createLog(void) {
	FRESULT   fresult;
	DIR dir;                    // Directory
	FILINFO fno;                // File Info
	uint8_t *tp, fileName[10];
	uint16_t fileNumber = 0;

	f_opendir(&dir,"/");  // directory open
	
	do {
		f_readdir(&dir,&fno);  
		tp = strtok(fno.fname,".");     // 拡張子削除
		if ( atoi(tp) > fileNumber ) {  // 番号比較
		fileNumber = atoi(tp);        // 文字列を数値に変換
		}
	} while(fno.fname[0] != 0); // ファイルの有無を確認

	f_closedir(&dir);     // directory close

	// ファイルナンバー作成
	if (fileNumber == 0) {
		// ファイルが無いとき
		fileNumber = 1;
	} else {
		// ファイルが有るとき
		fileNumber++;         // index pulus
	}

	sprintf(fileName,"%d",fileNumber);  // 数値を文字列に変換
	strcat(fileName, ".csv");           // 拡張子を追加
	fresult = f_open(&fil_W, fileName, FA_OPEN_ALWAYS | FA_WRITE);  // create file 

	strcpy(columnTitle, "");
	strcpy(formatLog, "");

	setLogStr("cntlog",       "%d");
	setLogStr("courseMarker",  "%d");
	setLogStr("encCurrentN",  "%d");
	setLogStr("gyroVal_Z",   "%d");
	setLogStr("encTotalN",    "%d");
	setLogStr("targetSpeed",    "%d");

	setLogStr("optimalIndex",  "%d");
	// setLogStr("ROC",  "%d");
	// setLogStr("x",  "%d");
	// setLogStr("y",  "%d");

	strcat(columnTitle,"\n");
	strcat(formatLog,"\n");
	// f_printf(&fil_W, columnTitle);

}
/////////////////////////////////////////////////////////////////////
// モジュール名 writeLogBufferPuts
// 処理概要     保存する変数をバッファに転送する
// 引数         c:8bit変数の数s:16bit変数の数i:32bit変数の数f:float変数の数
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
#ifdef	LOG_RUNNING_WRITE
void writeLogBufferPuts (uint8_t c, uint8_t s, uint8_t i, uint8_t f, ...) {
	va_list args;
	uint8_t cnt=0;
	static union {float f; uint32_t i;} ftoi;

	if (modeLOG) {
		// 	バッファ配列に保存
		va_start( args, f );
		// logBuffer[0] = va_arg( args, uint8_t* );
		for ( cnt=0; cnt<c; cnt++ ) send8bit( va_arg( args, uint32_t ) );
		for ( cnt=0; cnt<s; cnt++ ) send16bit( va_arg( args, uint32_t ) );
		for ( cnt=0; cnt<i; cnt++ ) send32bit( va_arg( args, uint32_t ) );
		for ( cnt=0; cnt<f; cnt++ ) {
			ftoi.f = va_arg( args, double );	// 共用体を使用してfloat型のビット操作をできるようにする
			send32bit( ftoi.i );
		}
		va_end(args);

		// バッファが512バイト付近まで溜まったら確認
		if( logBuffIndex + LOG_SIZE > 32) {
			logBuffSendIndex = logBuffIndex;		// バッファのバイト数を記録
			memcpy(logBufferSend, logBuffer, logBuffSendIndex);		// 送信用配列にコピー
			
			logBuffIndex = 0;	// バイト数リセット
			logBufferPointa = logBuffer;	// バッファ配列の先頭アドレスを設定
			sendSD = true;		// 送信開始
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
#ifdef	LOG_RUNNING_WRITE
void writeLogPuts(void) {
	uint32_t	writtenlog = 0;

	if (modeLOG) {
		if (sendSD) {
			sendSD = false;
			f_write(&fil_W,logBufferSend,logBuffSendIndex,writtenlog);
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
#ifndef	LOG_RUNNING_WRITE
void writeLogBufferPrint(void) {
  if (modeLOG) {
    logVal[logValIndex].time = cntLog;
    logVal[logValIndex].marker = courseMarkerLog;
    logVal[logValIndex].speed = encCurrentN;
    logVal[logValIndex].zg = BMI088val.gyro.z;
    logVal[logValIndex].distance = encTotalOptimal;
	// logVal[logValIndex].distance = distCtrl.pwm;
    logVal[logValIndex].target = targetSpeed;
    logVal[logValIndex].optimalIndex = optimalIndex;
	// logVal[logValIndex].optimalIndex = targetDist;
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
#ifndef	LOG_RUNNING_WRITE
void writeLogPrint(void) {
	uint32_t 	i,length = 0, beforeTime=0;
	float 		dt;

	clearXYcie();
	for(i = 0;i<logValIndex;i++) {
		dt = (float)(logVal[i].time - beforeTime) / 1000;
		calcXYcie(logVal[i].speed,logVal[i].zg, dt);

		f_printf(&fil_W, formatLog
			,logVal[i].time
			,logVal[i].marker
			,logVal[i].speed
			,(int32_t)(logVal[i].zg*10000)
			,logVal[i].distance
			,logVal[i].target
			,logVal[i].optimalIndex
			,(int32_t)(calcROC(logVal[i].speed,logVal[i].zg))
			,(int32_t)(xycie.x*10000)
			,(int32_t)(xycie.y*10000)
		);

		beforeTime = logVal[i].time;
	}
}
#endif
/////////////////////////////////////////////////////////////////////
// モジュール名 endLog
// 処理概要     ロギング終了処理
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void endLog(void) {
	initIMU = false;  // IMUの使用を停止(SPIが競合するため)
	modeLOG = false;  // ログ取得停止

#ifdef LOG_RUNNING_WRITE

#else
	// SPIバスが空くまで待つ
	while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY );
	createLog();    // ログファイル作成
	writeLogPrint();  // ログ書き込み
#endif

	f_close(&fil_W);
	initIMU = true;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 getFileNumbers
// 処理概要     ファイル名から番号を取得し配列に格納する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void getFileNumbers(void) {
	DIR 	dir;			// Directory
	FILINFO fno;			// File Info
	FRESULT	fresult;
	uint8_t fileName[10];
	uint8_t *tp, i;

	fresult = f_opendir(&dir,"/");  // directory open
	if (fresult == FR_OK) {
		do {
			f_readdir(&dir,&fno);     
			if (strstr(fno.fname,".csv") != NULL) {
				// csvファイルのとき
				tp = strtok(fno.fname,".");     // 拡張子削除
				fileNumbers[endFileIndex] = atoi(tp);        // 文字列を数値に変換
				endFileIndex++;
			}
		} while(fno.fname[0] != 0); // ファイルの有無を確認

		fileIndexLog = endFileIndex;
	}

	f_closedir(&dir);     // directory close
}
/////////////////////////////////////////////////////////////////////
// モジュール名 setLogStr
// 処理概要     ログCSVファイルのヘッダーとprintfのフォーマット文字列を生成
// 引数         column: ヘッダー文字列 format: フォーマット文字列
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void setLogStr(uint8_t* column, uint8_t* format) {
	uint8_t* columnStr[30], formatStr[30];

	// copy str to local variable
	strcpy(columnStr,column);
	strcpy(formatStr,format);

	strcat(columnStr,",");
	strcat(formatStr,",");
	strcat(columnTitle,columnStr);
	strcat(formatLog,formatStr);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 SDtest
// 処理概要     SDカードの読み書きテスト
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void SDtest(void) {
	FIL       fil_T;
	FRESULT   fresult;

	fresult = f_open(&fil_T, "test.csv", FA_OPEN_ALWAYS | FA_WRITE);  // create file
	while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY );
	f_close(&fil_T);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 createDir
// 処理概要     ホームディレクトリに指定されたディレクトリが存在しなければ作成する
// 引数         ディレクトリ名
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void createDir(uint8_t *dirName) {
	FRESULT		fresult;
	DIR			dir;			// Directory
	FILINFO		fno;			// File Info
	uint8_t		exist = 0;

	fresult = f_opendir(&dir,"/");  // directory open
	if (fresult == FR_OK) {
		do {
			f_readdir(&dir,&fno);     
			if (strcmp(fno.fname,dirName) == 0) {
				exist = 1;	// dirNameディレクトリが存在する
				break;
			}
		} while(fno.fname[0] != 0); // ファイルの有無を確認

		if (!exist) {
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
void send8bit (uint8_t data) {
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
void send16bit (uint16_t data) {
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
void send32bit (uint32_t data) {
	logBuffer[logBuffIndex++] = data >> 24;
	logBuffer[logBuffIndex++] = ( data & 0x00ff0000 ) >> 16;
	logBuffer[logBuffIndex++] = ( data & 0x0000ff00 ) >> 8;
	logBuffer[logBuffIndex++] = data & 0x000000ff;
}
#endif
// /////////////////////////////////////////////////////////////////////
// // モジュール名 CharToShort
// // 処理概要     unsigned char型変数をshort型に変換する
// // 引数         data:変換するsigned char型変数	offsetaddress: MicroSD内の順番
// // 戻り値       変換したshort型
// /////////////////////////////////////////////////////////////////////
// short CharToShort(unsigned char offsetaddress) {
// 	volatile short s;
	
// 	s = (short)( (unsigned char)msdBuff[msdBuffaddress + offsetaddress] * 0x100 + 
// 			(unsigned char)msdBuff[msdBuffaddress + offsetaddress + 1] );
					
// 	return s;				
// }
// /////////////////////////////////////////////////////////////////////
// // モジュール名 CharTouInt
// // 処理概要     unsigned char型変数をunsigned int型に変換する
// // 引数         data:変換するsigned char型変数	offsetaddress: MicroSD内の順番
// // 戻り値       変換したunsigned int型
// /////////////////////////////////////////////////////////////////////
// unsigned int CharTouInt(unsigned char offsetaddress) {
// 	volatile unsigned int i;
	
// 	i = (unsigned int)(unsigned char)msdBuff[msdBuffaddress + offsetaddress] * 0x1000000;
// 	i += (unsigned int)(unsigned char)msdBuff[msdBuffaddress + offsetaddress + 1] * 0x10000;
// 	i += (unsigned int)(unsigned char)msdBuff[msdBuffaddress + offsetaddress + 2] * 0x100;
// 	i += (unsigned int)(unsigned char)msdBuff[msdBuffaddress + offsetaddress + 3];
					
// 	return i;				
// }
