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
    motorControldist();

    // 走行中に処理
    if (patternTrace > 10 && patternTrace < 100) {
        // 緊急停止処理
        // if (cntEmcStopAngleX()) emcStop = STOP_ANGLE_X;
        // if (cntEmcStopAngleY()) emcStop = STOP_ANGLE_Y;
        if (cntEmcStopEncStop()) emcStop = STOP_ENCODER_STOP;
        // if (cntEmcStopLineSensor()) emcStop = STOP_LINESENSOR;
        
        courseMarker = checkMarker();   // マーカー検知
        checkGoalMarker();              // ゴールマーカー処理

        if( courseMarker > 0) {
            courseMarkerLog = courseMarker;
        }

        // カーブマーカーを通過した時
        if (courseMarker == 2 && beforeCourseMarker == 0) {
            cntMarker++;    // マーカーカウント
            if (optimalTrace == BOOST_DISTANCE) {
                // 距離基準2次走行のとき
                int32_t i, j, errorDistance, upperLimit, lowerLimit;

                for(i=pathedMarker;i<=numPPAMarry;i++) {
                    // 現在地から一番近いマーカーを探す
                    if (abs(encTotalOptimal - markerPos[i].distance) < encMM(50)) {
                        errorDistance = encTotalOptimal - DistanceOptimal;  // 現在の差を計算
                        encTotalOptimal = markerPos[i].distance;               // 距離を補正
                        DistanceOptimal = encTotalOptimal - errorDistance;  // 補正後の現在距離からの差分
                        optimalIndex = markerPos[i].indexPPAD;      // インデックス更新

                        pathedMarker = i;
                        break;
                    }
                }
            }
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
            // ショートカット走行の目標値インデックスを更新
            if (optimalTrace == BOOST_SHORTCUT && DistanceOptimal > 0) {
                // distLen = (float)encCurrentN * PALSE_MILLIMETER * 0.005; // 現在速度から5ms後の移動距離を計算
                optimalIndex = (int32_t)( encTotalOptimal / PALSE_MILLIMETER ) / CALCDISTANCE; // 50mmごとにショートカット配列を作っているので移動距離[mm]を50mmで割った商がインデクス
                if(optimalIndex+1 <= numPPADarry) {
                    optimalIndex++;
                }
                // xy座標計算
                // calcXYcie(encCurrentN,BMI088val.gyro.z);

                setShortCutTarget(); // 目標値更新
            }
            break;
        case 2:
            if(initIMU) {
                IMUstate = IMU_TRANSMIT;

                if(!calibratIMU) {
                    BMI088getGyro();    // 角速度取得
                    calcDegrees();      // 角度計算
                    if(optimalTrace == 0) checkCurve(); // 1次走行 カーブ検出

                    motorControlYawRate();  // 角速度制御
                    motorControlYaw();      // 角度制御
                } else {
                    calibrationIMU();
                }

                IMUstate = IMU_STOP;
            }
            break;
        case 3:
            break;
        case 5:
            // if(initIMU) {
            //     BMI088getAccele();
            // }
            cnt5 = 0;
            break;
    }
        
    if(patternTrace >= 12 && patternTrace < 100) {
        if( encLog >= encMM(CALCDISTANCE) ) {
            // CALCDISTANCEごとにログを保存
            writeLogBufferPrint(); // バッファにログを保存
            courseMarkerLog = 0;
            encLog = 0;
        }
    }
   
    switch (cnt10) {
        case 1:
            getADC2();
            getMotorCurrent();
            break;
        case 2:
            // if(initIMU) {
            //     BMI088getTemp();
            // }
            break;
        case 9:
            // if(patternTrace < 100) {
            //     writeLogBufferPrint(); // バッファにログを保存
            //     courseMarkerLog = 0;
            // }
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
    // if (IMUstate == IMU_STOP ) {
    //     writeLogPuts();
    // }
}   
/////////////////////////////////////////////////////////////////////
// モジュール名 Interrupt300ns
// 処理概要     タイマー割り込み(300ns)
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void Interrupt300ns(void) {
    // Interrupt 300ns
    // sendColorData();
}
