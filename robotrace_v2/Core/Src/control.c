﻿//====================================//
// インクルード
//====================================//
#include "control.h"
//====================================//
// グローバル変数の宣言
//====================================//
// モード関連
uint8_t	patternTrace = 0;
bool    modeDSP = true;		// ディスプレイ表示可否			false:消灯		true:表示
bool 	modeLOG = false;	// ログ取得状況			false:ログ停止	true:ログ取得中
bool    initMSD = false;	// microSD初期化状況	false:初期化失敗	true:初期化成功
bool    initLCD = false;    // LCD初期化状況		false:初期化失敗	true:初期化成功
bool    initIMU = false;    // IMU初期化状況		false:初期化失敗	true:初期化成功
bool    initCurrent = false;    // 電流センサ初期化状況		false:初期化失敗	true:初期化成功
uint8_t modeCurve = 0;		// カーブ判断			0:直線			1:カーブ進入

uint16_t 	analogVal1[NUM_SENSORS];		// ADC結果格納配列
uint16_t 	analogVal2[3];		// ADC結果格納配列

// 速度パラメータ関連
speedParam targetParam = {	
	PARAM_STRAIGHT, 
	PARAM_CURVE,
	PARAM_STOP,
	PARAM_BOOST_STRAIGHT,
	PARAM_BOOST_1500,
	PARAM_BOOST_800,
	PARAM_BOOST_700,
	PARAM_BOOST_600,
	PARAM_BOOST_600,
	PARAM_BOOST_400,
	PARAM_BOOST_300,
	PARAM_BOOST_200,
	PARAM_BOOST_100,
	MACHINEACCELE,
	MACHINEDECREACE,
	};

// タイマ関連
uint32_t 	cntRun = 0;
int16_t 	countdown;

// マーカー関連
uint8_t 	courseMarker;
uint8_t 	beforeCourseMarker;
uint32_t 	cntMarker = 0;
uint8_t 	courseMarkerLog;

// ログ関連
uint32_t 	goalTime = 0;

///////////////////////////////////////////////////////////////////////////
// モジュール名 systemInit
// 処理概要     初期化処理
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void initSystem (void) {

	// ADC
	if (HAL_ADC_Start_DMA(&hadc1, analogVal1, 10) != HAL_OK)	Error_Handler();
	if (HAL_ADC_Start_DMA(&hadc2, analogVal2, 3) != HAL_OK)		Error_Handler();

	// Encoder count
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_ALL);

	// Motor driver
	if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
	if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	motorPwmOut(0,0);

	// line sensor PWM
	if (HAL_TIM_PWM_Start_IT(&htim13, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
	powerLinesensors(0);

	// IMU
	initIMU = initBMI088();
	// microSD
	initMSD = initMicroSD();
	getLogNumber(); // 前回の解析ログナンバーを取得

	// Display
	if(TACTSW1 == 1) {
		modeDSP = true;
		ssd1306_Init();
		ssd1306_Fill(Black);
		
		// トップバー表示
		// 電池マーク
		showBatMark();
		
		ssd1306_UpdateScreen();
	} else {
		modeDSP = false;
	}
	

	// Timer interrupt
	HAL_TIM_Base_Start_IT(&htim6);

	// ledset(100,0,0);

	printf("hello \n");
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 systemLoop
// 処理概要     メインループ
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void loopSystem (void) {

	// 緊急停止処理
	if (patternTrace > 10 && patternTrace < 100 && emcStop > 0) {
		goalTime = cntRun;
		emargencyStop();
	}
	
	switch (patternTrace) {
      	case 0:
			if(modeDSP) {
				setup();
			} else {
				caribrateSensors();
			}
				
			if (start) {
				motorPwmOut(0,0);
				countdown = 5000;		// カウントダウンスタート
				calibratIMU = true;
				patternTrace = 1;
			}
			break;
		case 1:	
			// カウントダウンスタート
			if(modeDSP) {
				if ( countdown == 4000 ) {
					ssd1306_FillRectangle(0,15,127,63, Black); // メイン表示空白埋め
					ssd1306_SetCursor(56,28);
					ssd1306_printf(Font_16x26,"4");
					ssd1306_UpdateScreen();  // グラフィック液晶更新
				}
				if ( countdown == 3000 ) {
					ssd1306_FillRectangle(0,15,127,63, Black); // メイン表示空白埋め
					ssd1306_SetCursor(56,28);
					ssd1306_printf(Font_16x26,"3");
					ssd1306_UpdateScreen();  // グラフィック液晶更新
				}
				if ( countdown == 2000 ) {
					ssd1306_FillRectangle(0,15,127,63, Black); // メイン表示空白埋め
					ssd1306_SetCursor(56,28);
					ssd1306_printf(Font_16x26,"2");
					ssd1306_UpdateScreen();  // グラフィック液晶更新
				}
				if ( countdown == 1000 ) {
					ssd1306_FillRectangle(0,15,127,63, Black); // メイン表示空白埋め
					ssd1306_SetCursor(56,28);
					ssd1306_printf(Font_16x26,"1");
					ssd1306_UpdateScreen();  // グラフィック液晶更新
				}
			}

			if ( !calibratIMU ) {
			// if ( countdown <= 0 ) {
				motorPwmOut(0,0);	// モータドライバICのスリープモードを解除
				// Logファイル作成
				if (initMSD) initLog();
				
				// 変数初期化
				encTotalN = 0;
				encTotalOptimal = 0;
				distanceStart = 0;
				encRightMarker = encMM(600);
				cntRun = 0;
				BMI088val.angle.x = 0.0f;
				BMI088val.angle.y = 0.0f;
				BMI088val.angle.z = 0.0f;
				veloCtrl.Int = 0.0;
				yawRateCtrl.Int = 0.0;
				yawCtrl.Int = 0.0;

				patternTrace = 11;
			}
			break;

      	case 11:
			// 目標速度設定
			if (optimalTrace == 0){
				// 探索走行のとき
				if (modeCurve == 0) {
					setTargetSpeed(targetParam.straight);
				} else {
					setTargetSpeed(targetParam.curve);
				}
			} else if (optimalTrace == BOOST_DISTANCE) {
				// 距離基準2次走行
				// スタートマーカーを超えた時から距離計測開始
				if (SGmarker > 0 && distanceStart == 0) {
					distanceStart = encTotalOptimal;
				}
				// 一定区間ごとにインデックスを更新
				if (distanceStart > 0) {
					if (encTotalOptimal - distanceStart >= encMM(CALCDISTANCE)) {
						boostSpeed = PPAD[optimalIndex].boostSpeed;	// 目標速度を更新
						distanceStart = encTotalOptimal;	// 距離計測位置を更新
						if (optimalIndex+1 <= numPPADarry) {
							// 配列要素数を超えない範囲でインデックスを更新する
							optimalIndex++;
						}
					}
				} else if (distanceStart == 0) {
					boostSpeed = targetParam.straight;
				}
				// 目標速度に設定
				setTargetSpeed(boostSpeed);
			}
			
			// ライントレース
			motorPwmOutSynth( lineTraceCtrl.pwm, veloCtrl.pwm, 0, 0);
	 
			// ゴール判定
			if (SGmarker >= COUNT_GOAL ) {
				goalTime = cntRun;
				enc1 = 0;
				patternTrace = 101;
			}
			break;

      	case 101:
			// 停止速度まで減速
			if (enc1 >= encMM(500)) {
				setTargetSpeed(0);
			} else {
				setTargetSpeed(targetParam.stop);
			}
			motorPwmOutSynth( lineTraceCtrl.pwm, veloCtrl.pwm, 0, 0);
			
			if (encCurrentN == 0 && enc1 >= encMM(500)) {
				motorPwmOutSynth( 0, 0, 0, 0);
				if (modeLOG) endLog();	// ログ保存終了

				if(modeDSP) {
					ssd1306_FillRectangle(0,15,127,63, Black); // メイン表示空白埋め
					ssd1306_SetCursor(0,25);
					ssd1306_printf(Font_11x18,"Time %d",optimalTrace);
					ssd1306_SetCursor(0,45);
					ssd1306_printf(Font_11x18,"%6.3f[s]",(float)goalTime/1000);
					ssd1306_UpdateScreen();  // グラフィック液晶更新
				}
	
				patternTrace = 102;
				break;
			}

			break;

      	case 102: 
			motorPwmOutSynth( 0, 0, 0, 0);
			powerLinesensors(0);

			// if (swValTact == SW_PUSH) {
			// 	initSystem();
			// 	start = 0;
			// 	patternTrace = 0;
			// }
			break;
    
      	default:
        	break;
    } // switch case
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 emargencyStop
// 処理概要     緊急停止処理
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void emargencyStop (void) {
	motorPwmOutSynth( 0, 0, 0, 0);
	if (modeLOG) endLog(); // ログ保存終了

	if(modeDSP) {
		ssd1306_FillRectangle(0,15,127,63, Black); // メイン表示空白埋め

		ssd1306_SetCursor(36,25);
		ssd1306_printf(Font_11x18,"EMS!! %d",optimalTrace);

		ssd1306_SetCursor(0,45);
		switch(emcStop) {
			case STOP_ANGLE_X:
				ssd1306_printf(Font_7x10,"ANGLE_X");
				break;
			case STOP_ANGLE_Y:
				ssd1306_printf(Font_7x10,"ANGLE_Y");
				break;
			case STOP_ENCODER_STOP:
				ssd1306_printf(Font_7x10,"ENCODER_STOP");
				break;
			case STOP_LINESENSOR:
				ssd1306_printf(Font_7x10,"LINESENSOR");
				break;
		}
		
		ssd1306_UpdateScreen();  // グラフィック液晶更新
	}
	

	patternTrace = 102;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 countDown
// 処理概要     カウントダウン
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void countDown (void) { 
	if (countdown > 0) countdown--;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 checkCurve
// 処理概要     カーブとストレートの判定
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void checkCurve(void) {
	static uint8_t 	checkStraight, checkRight, checkLeft;
	float zg;

	zg = BMI088val.gyro.z;

	if (fabs(zg) < 50.0f) {
		// ストレート時
		checkRight = 0;
		checkLeft = 0;
		if (checkStraight == 0) {
			encCurve = 0;
			checkStraight = 1;
		}
		if(checkStraight == 1 && encCurve > encMM(100)){
			modeCurve = 0;
		}
	} else if (zg > 150.0f) {
		// 左カーブ時
		checkStraight = 0;
		checkRight = 0;
		if (checkLeft == 0) {
			encCurve = 0;
			checkLeft = 1;
		}
		if(checkLeft == 1 && encCurve > encMM(20)){
			modeCurve = 2;
		}
	} else if (zg < -150.0f) {
		// 右カーブ時
		checkStraight = 0;
		checkLeft = 0;
		if (checkRight == 0) {
			encCurve = 0;
			checkRight = 1;
		}
		if(checkRight == 1 && encCurve > encMM(20)){
			modeCurve = 1;
		}
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 getADC2
// 処理概要     AD値の取得
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void getADC2(void) {
	motorCurrentValL = analogVal2[0];
	motorCurrentValR = analogVal2[1];
	batteryVal = analogVal2[2];
}
