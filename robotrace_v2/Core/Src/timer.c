//====================================//
// インクルード
//====================================//
#include "timer.h"
//====================================//
// グローバル変数の宣
//====================================//
int32_t cnt5 = 0;
int32_t cnt10 = 0;
/////////////////////////////////////////////////////////////////////
// モジュール名 Interrupt1ms
// 処理概要     タイマー割り込み(1ms)
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void Interrupt1ms(void) {
    // Interrupt 1ms
    cntRun++;
    cnt5++;
    cnt10++;
    cntLog++;

    // Encoder
    getEncoder();

    // PID制御処理
    motorControlTrace();
    motorControlSpeed();

    // 走行中に処理
    if (patternTrace > 10 && patternTrace < 100) {
        // 緊急停止処理
        // if (cntEmcStopAngleX()) emcStop = STOP_ANGLE_X;
        // if (cntEmcStopAngleY()) emcStop = STOP_ANGLE_Y;
        if (cntEmcStopEncStop()) emcStop = STOP_ENCODER_STOP;
        if (cntEmcStopLineSensor()) emcStop = STOP_LINESENSOR;
        
        courseMarker = checkMarker();   // マーカー検知
        checkGoalMarker();              // ゴールマーカー処理

        // マーカーを通過した時
        if (courseMarker == 2 && beforeCourseMarker == 0) {
            cntMarker++;    // マーカーカウント
        }
        beforeCourseMarker = courseMarker;
    }

    // 走行前に処理
    if (patternTrace < 10 || patternTrace > 100) {
        getSwitches();  // スイッチの入力を取得
        countDown();
        cntSetup1++;
        cntSetup2++;
        cntSwitchUD++;
        cntSwitchLR++;
    }
    
    switch(cnt5) {
        case 1:
            if(initIMU) {
                BMI088getGyro();
                calcDegrees();
                checkCurve();
                motorControlYawRate();
                motorControlYaw();
            }
            break;
        case 5:
            cnt5 = 0;
            break;
    }

    switch (cnt10) {
        case 1:
            getADC2();
            getMotorCurrent();
            break;
        case 9:
            writeLogBuffer(); // バッファにログを保存
            break;
        case 10:
            cnt10 = 0;
            break;
    }
}
/////////////////////////////////////////////////////////////////////
// モジュール名 Interrupt100us
// 処理概要     タイマー割り込み(0.1ms)
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void Interrupt100us(void) {
    
}
/////////////////////////////////////////////////////////////////////
// モジュール名 Interrupt300ns
// 処理概要     タイマー割り込み(300ns)
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void Interrupt300ns(void) {
    // Interrupt 300ns
    sendColorData();
}
