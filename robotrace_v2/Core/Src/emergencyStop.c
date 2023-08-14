//====================================//
// インクルード
//====================================//
#include "emergencyStop.h"
//====================================//
// グローバル変数の宣言
//====================================//
uint8_t emcStop = 0;

/////////////////////////////////////////////////////////////////////
// モジュール名 cntEmcStopAngleX
// 処理概要     緊急停止要因のカウント x軸角速度異常
// 引数         なし
// 戻り値       true:緊急停止 false:異常なし
/////////////////////////////////////////////////////////////////////
bool cntEmcStopAngleX(void) {
    static uint16_t cntAngleX;

    // 緊急停止条件
    if (fabs(BMI088val.gyro.x) > 2.0f) cntAngleX++;
    else    cntAngleX = 0;

    if (cntAngleX > STOP_COUNT_ANGLE_X) return true;
    else return false;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 cntEmcStopAngleY
// 処理概要     緊急停止要因のカウント y軸角速度異常
// 引数         なし
// 戻り値       true:緊急停止 false:異常なし
/////////////////////////////////////////////////////////////////////
bool cntEmcStopAngleY(void) {
    static uint16_t cntAngleY;

    // 緊急停止条件
    if (fabs(BMI088val.gyro.y) > 2.0f) cntAngleY++;
    else    cntAngleY = 0;

    if (cntAngleY > STOP_COUNT_ANGLE_Y) return true;
    else return false;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 cntEmcStopEncStop
// 処理概要     緊急停止要因のカウント エンコーダストップ
// 引数         なし
// 戻り値       true:緊急停止 false:異常なし
/////////////////////////////////////////////////////////////////////
bool cntEmcStopEncStop(void) {
    static uint16_t cntEncStop;

    // 緊急停止条件
    if (abs(encCurrentN) < 10) cntEncStop++;
    else    cntEncStop = 0;

    if (cntEncStop > STOP_COUNT_ENCODER_STOP) return true;
    else return false;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 cntEmcStopLinesensor
// 処理概要     緊急停止要因のカウント ラインセンサ飽和
// 引数         なし
// 戻り値       true:緊急停止 false:異常なし
/////////////////////////////////////////////////////////////////////
bool cntEmcStopLineSensor(void) {
    static uint16_t cntLineSensor = 0;

    // 緊急停止条件
    if (lSensor[4]+lSensor[5] > 6000) cntLineSensor++;
    else    cntLineSensor = 0;

    if (cntLineSensor > STOP_COUNT_LINESENSOR) return true;
    else return false;
}
