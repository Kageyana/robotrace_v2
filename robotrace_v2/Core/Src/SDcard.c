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
uint32_t  logBuffer[BUFFER_SIZW_LOG];
uint32_t  logIndex = 0 , sendLogNum = 0;

// ログファイルナンバー
int16_t fileNumbers[1000], fileIndexLog = 0, endFileIndex = 0;

// カウンタ
uint32_t    cntLog = 0;
/////////////////////////////////////////////////////////////////////
// モジュール名 initMicroSD
// 処理概要     SDカードの初期化
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
bool initMicroSD(void) {
  FATFS     *pfs;
  FRESULT   fresult;
  DWORD     fre_clust;
  uint32_t  total, free_space;
  FIL       fil_T;

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

    // getFileNumbers();

    fresult = f_open(&fil_T, "test.csv", FA_OPEN_ALWAYS | FA_WRITE);  // create file
    while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY );
    f_close(&fil_T);

    return true;
  } else {
    // マウント失敗
    initMSD = false;
    printf ("error in mounting SD CARD...\r\n");
    return false;
  }
}
/////////////////////////////////////////////////////////////////////
// モジュール名 initLog
// 処理概要     ロギング初期設定
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void initLog(void) {
  FRESULT   fresult;
  DIR dir;                    // Directory
  FILINFO fno;                // File Info
  uint8_t fileName[10];
  uint8_t *tp;
  uint16_t fileNumber = 0;

  f_opendir(&dir,"/");  // directory open
  do {
    f_readdir(&dir,&fno);
    if(fno.fname[0] != 0) {           // ファイルの有無を確認
      tp = strtok(fno.fname,".");     // 拡張子削除
      if ( atoi(tp) > fileNumber ) {  // 番号比較
        fileNumber = atoi(tp);        // 文字列を数値に変換
      }
    }
  } while(fno.fname[0] != 0);

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

  setLogStr("cntlog",       "%d");
  setLogStr("markerSensor",  "%d");
  setLogStr("encCurrentN",  "%d");
  setLogStr("gyroVal_Z",   "%d");
  setLogStr("encTotalN",    "%d");
  setLogStr("targetSpeed",    "%d");

  // setLogStr("patternTrace", "%d");
  // setLogStr("encCurrentR",  "%d");
  // setLogStr("encCurrentL",  "%d");  
  // setLogStr("encTotalR",    "%d");
  // setLogStr("encTotalL",    "%d");
  // setLogStr("angleSensor",  "%d");
  // setLogStr("modeCurve",    "%d");
  setLogStr("motorpwmR",  "%d");
  setLogStr("motorpwmL",  "%d");
  // setLogStr("lineTraceCtrl.pwm",    "%d");
  // setLogStr("veloCtrl.pwm",    "%d");
  // strcat(columnTitle,"lSensorf_0,");
  // strcat(columnTitle,"lSensorf_1,");
  // strcat(columnTitle,"lSensorf_2,");
  // strcat(columnTitle,"lSensorf_3,");
  // strcat(columnTitle,"lSensorf_4,");
  // strcat(columnTitle,"lSensorf_5,");
  // strcat(columnTitle,"lSensorf_6,");
  // strcat(columnTitle,"lSensorf_7,");
  // strcat(columnTitle,"lSensorf_8,");
  // strcat(columnTitle,"lSensorf_9,");
  // strcat(columnTitle,"lSensorf_10,");
  // strcat(columnTitle,"lSensorf_11,");
  // setLogStr("gyroVal_X",   "%d");
  // setLogStr("gyroVal_Y",   "%d");  
  // setLogStr("angle_X",   "%d");
  // setLogStr("angle_Y",   "%d");
  setLogStr("angle_Z",   "%d");
  // setLogStr("rawCurrentR",  "%d");
  // setLogStr("rawCurrentL",  "%d");
  // setLogStr("CurvatureRadius",  "%d");
  setLogStr("cntMarker",  "%d");
  setLogStr("optimalIndex",  "%d");
  setLogStr("ROC",  "%d");

  strcat(columnTitle,"\n");
  strcat(formatLog,"\n");
  f_printf(&fil_W, columnTitle);

  cntLog = 0;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 writeLogBuffer
// 処理概要     保存する変数の値をバッファに転送する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void writeLogBuffer (uint8_t valNum, ...) {
  va_list args;
  uint8_t count;
  // valNum : amount of variable

  va_start( args, valNum );
  for ( count = 0; count < valNum; count++ ) {
    // set logdata to logbuffer(ring buffer) ログデータをリングバッファに転送
    logBuffer[logIndex & BUFFER_SIZW_LOG - 1] = va_arg( args, int32_t );
    logIndex++;
  }
  logBuffer[logIndex & BUFFER_SIZW_LOG - 1] = "\n";
  logIndex++;
  va_end( args );
}
/////////////////////////////////////////////////////////////////////
// モジュール名 writeLogPut
// 処理概要     バッファをSDカードに転送する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void writeLogPut(void) {
  uint8_t str[32];

  if (sendLogNum < logIndex) {
    if (logBuffer[sendLogNum & BUFFER_SIZW_LOG - 1] == "\n") {
      f_puts(logBuffer[sendLogNum & BUFFER_SIZW_LOG - 1], &fil_W);
    } else {
      sprintf(str,"%d,",logBuffer[sendLogNum & BUFFER_SIZW_LOG - 1]);
      f_puts(str, &fil_W);
    }
    sendLogNum++;
  }
}
/////////////////////////////////////////////////////////////////////
// モジュール名 endLog
// 処理概要     ロギング終了処理
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void endLog(void) {
  modeLOG = false;
  while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY );
  f_close(&fil_W);

  // if (!optimalTrace) saveLogNumber(fileNumbers[endFileIndex]+1); // 探索走行のとき最新ログ番号を保存する
}
/////////////////////////////////////////////////////////////////////
// モジュール名 getFileNumbers
// 処理概要     ファイル名から番号を取得し配列に格納する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void getFileNumbers(void) {
  DIR dir;                    // Directory
  FILINFO fno;                // File Info
  uint8_t fileName[10];
  uint8_t *tp, i;

  f_opendir(&dir,"/");  // directory open
  do {
    f_readdir(&dir,&fno);
    if(fno.fname[0] != 0) {           // ファイルの有無を確認
      if (strstr(fno.fname,".CSV") != NULL) {
        // csvファイルのとき
        tp = strtok(fno.fname,".");     // 拡張子削除
        fileNumbers[endFileIndex] = atoi(tp);        // 文字列を数値に変換
        endFileIndex += 1;
      }
    }
  } while(fno.fname[0] != 0);
  endFileIndex -= 1;

  fileIndexLog = endFileIndex;

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