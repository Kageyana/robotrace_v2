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
        // if (cntEmcStopEncStop()) emcStop = STOP_ENCODER_STOP;
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

        motorControlYawRate();
        motorControlYaw();
    }

    switch (cnt10)
    {
    case 1:
        if(initIMU) {
            BMI088getGyro();
            calcDegrees();
        }
        
        break;
    case 2:
        getADC2();
        break;
    case 9:
        if (modeLOG) {
                writeLogBuffer(
                    7,
                    cntLog,
                    getMarkerSensor(),
                    encCurrentN,
                    // (int32_t)(BMI088val.gyro.z*10000),
                    encTotalN,
                    targetSpeed,
                    // encCurrentR,
                    // encCurrentL,
                    // encTotalR,
                    // encTotalL,
                    // (int32_t)(angleSensor*10),
                    // modeCurve,
                    // motorpwmR,
                    // motorpwmL,
                    // lineTraceCtrl.pwm,
                    // veloCtrl.pwm,
                    // (int32_t)lSensorf[0],
                    // (int32_t)lSensorf[1],
                    // (int32_t)lSensorf[2],
                    // (int32_t)lSensorf[3],
                    // (int32_t)lSensorf[4],
                    // (int32_t)lSensorf[5],
                    // (int32_t)lSensorf[6],
                    // (int32_t)lSensorf[7],
                    // (int32_t)lSensorf[8],
                    // (int32_t)lSensorf[9],
                    // (int32_t)lSensorf[10],
                    // (int32_t)lSensorf[11],
                    // (int32_t)(BNO055val.gyro.x*10000),
                    // (int32_t)(BNO055val.gyro.y*10000),
                    // (int32_t)(BNO055val.angle.x*10000),
                    // (int32_t)(BNO055val.angle.y*10000),
                    // (int32_t)(BMI088val.angle.z*10000),
                    // rawCurrentR,
                    // rawCurrentL,
                    // (int32_t)(calcROC((float)encCurrentN, BNO055val.gyro.z) * 100)
                    // cntMarker
                    // optimalIndex,
                    // (int32_t)calcROC( encCurrentN, BMI088val.gyro.z)
                    distL,
                    distR
                );
            }
        break;
    case 10:
        cnt10 = 0;
        break;
    
    default:
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
    // Interrupt 100us
    if (modeLOG) writeLogPut();
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
