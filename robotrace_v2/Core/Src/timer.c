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
        if (cntEmcStopLineSensor()) emcStop = STOP_LINESENSOR;
        if (judgeOverSpeed()) emcStop = STOP_OVERSPEED;
        
        courseMarker = checkMarker();   // マーカー検知
        checkGoalMarker();              // ゴールマーカー処理

        // カーブマーカー,クロスラインを通過した時
        if (courseMarker == 0 && beforeCourseMarker > 0) {
            cntMarker++;    // マーカーカウント
            if (optimalTrace == BOOST_DISTANCE) {
                if(straightState) {
                    // 距離基準2次走行のとき
                    int32_t i, j, errorDistance = 0;

                    // for(i=pathedMarker;i<=numPPAMarry;i++) {
                    //     // 現在地から一番近いマーカーを探す
                    //     if (abs(encTotalOptimal - markerPos[i].distance) < encMM(100)) {
                    //         errorDistance = encTotalOptimal - DistanceOptimal;  // 現在の差を計算
                    //         encTotalOptimal = markerPos[i].distance;               // 距離を補正
                    //         DistanceOptimal = encTotalOptimal - errorDistance;  // 補正後の現在距離からの差分
                    //         optimalIndex = markerPos[i].indexPPAD;      // インデックス更新

                    //         if(i-5 < 0) {
                    //             pathedMarker = i-5;
                    //         } else {
                    //             pathedMarker = 0;
                    //         }
                    //         break;
                    //     }
                    // }
                    
                    for(i=pathedMarker;i<=numPPAMarry;i++) {
                        // 現在地から一番近いマーカーを探す
                        if (encTotalOptimal - markerPos[i].distance < 0) {
                            for (j=i;j>0;j--) {
                                if (abs(encTotalOptimal - markerPos[j].distance) < encMM(100)) {
                                    errorDistance = encTotalOptimal - DistanceOptimal;  // 現在の差を計算
                                    encTotalOptimal = markerPos[j].distance;               // 距離を補正
                                    DistanceOptimal = encTotalOptimal - errorDistance;  // 補正後の現在距離からの差分
                                    optimalIndex = markerPos[j].indexPPAD;      // インデックス更新

                                    if(j-5 < 0) {
                                        pathedMarker = j-5;
                                    } else {
                                        pathedMarker = 0;
                                    }
                                    straightState = false;
                                    straightMeter = 0;
                                    break;
                                }
                            }
                            if(errorDistance != 0) break;
                        }
                    }
                    
                }
            }
        }
            
        if (courseMarker == 0 && beforeCourseMarker > 0) {
            // マーカーの位置を記録
            writeMarkerPos(encTotalOptimal, beforeCourseMarker);
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

        wheelClick();
    }  
    
    switch(cnt5) {
        case 1:
            // xy座標計算
            // // ショートカット走行の目標値インデックスを更新
            // if (optimalTrace == BOOST_SHORTCUT && DistanceOptimal > 0) {
            //     // calcXYcie(encCurrentN,BMI088val.gyro.z, DEFF_TIME);
            //     // distLen = (float)encCurrentN * PALSE_MILLIMETER * 0.005; // 現在速度から5ms後の移動距離を計算
            //     // optimalIndex = (int32_t)( encTotalOptimal / PALSE_MILLIMETER ) / CALCDISTANCE_SHORTCUT; // 50mmごとにショートカット配列を作っているので移動距離[mm]を50mmで割った商がインデクス
            //     // if(optimalIndex+1 <= numPPADarry) {
            //     //     optimalIndex++;
            //     // }

            //     if(targetDist - encPID < 200) {
            //         if(optimalIndex+1 <= numPPADarry) {
            //             optimalIndex++;
            //         } 
            //     }
                
            //     setShortCutTarget(); // 目標値更新
            // }
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
        
    if (patternTrace > 11 && patternTrace < 100) {
        if( encLog >= encMM(CALCDISTANCE_SHORTCUT) ) {
            static float rocCorrection = 0;
            rocCorrection = calcROC(encCurrentN,BMI088val.gyro.z);
            if( fabs(rocCorrection) >= 700.0F) {
                straightMeter += CALCDISTANCE_SHORTCUT;
            } else {
                straightMeter = 0;
            }

            if (straightMeter >= 100 ) {
                straightState = true;
            }

            if (modeLOG) {
                // CALCDISTANCEごとにログを保存
#ifdef	LOG_RUNNING_WRITE
                writeLogBufferPuts(
                    LOG_NUM_8BIT,LOG_NUM_16BIT,LOG_NUM_32BIT,LOG_NUM_FLOAT
                    // 8bit
                    ,targetSpeed
                    // 16bit
                    ,cntLog
                    ,encCurrentN
                    ,optimalIndex
                    ,motorCurrentValL
                    ,motorCurrentValR
                    ,lineTraceCtrl.pwm
                    ,veloCtrl.pwm
                    // 32bit
                    ,encTotalOptimal
                    // float型
                    ,BMI088val.gyro.z
                );
#else
                writeLogBufferPrint(); // バッファにログを保存
#endif
                cntLog = 0;
                encLog = 0;
            }
        }
    }
   
    switch (cnt10) {
        case 1:
            getADC2();
            // getMotorCurrent();
            break;
        case 2:
            // if(initIMU) {
            //     BMI088getTemp();
            // }
            break;
        case 9:
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
    if (IMUstate == IMU_STOP ) {
#ifdef	LOG_RUNNING_WRITE
        writeLogPuts();
#endif
    }
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
