//====================================//
// インクルード
//====================================//
#include "control.h"
//====================================//
// グローバル変数の宣言
//====================================//
// モード関連
uint8_t	patternTrace = 0;
bool    modeLCD = true;		// LCD表示可否			false:消灯		true:表示
bool 	modeLOG = false;	// ログ取得状況			false:ログ停止	true:ログ取得中
bool    initMSD = false;	// microSD初期化状況	false:初期化失敗	true:初期化成功
bool    initLCD = false;    // LCD初期化状況		false:初期化失敗	true:初期化成功
bool    initIMU = false;    // IMU初期化状況		false:初期化失敗	true:初期化成功
bool    useIMU = false; 	// IMU使用状況			false:使用停止		true:使用中
bool    initCurrent = false;    // 電流センサ初期化状況		false:初期化失敗	true:初期化成功
uint8_t modeCurve = 0;		// カーブ判断			0:直線			1:カーブ進入

uint16_t 	analogVal[NUM_SENSORS];		// ADC結果格納配列

// 速度パラメータ関連
speedParam targetParam = {	
	PARAM_STRAIGHT, 
	PARAM_CURVE,
	PARAM_STOP,
	PARAM_BOOST_STRAIGHT,
	PARAM_BOOST_1500,
	PARAM_BOOST_800,
	PARAM_BOOST_600,
	PARAM_BOOST_400,
	PARAM_BOOST_200
	};

// タイマ関連
uint32_t 	cntRun = 0;
int16_t 	countdown;

// マーカー関連
uint8_t 	courseMarker;
uint8_t 	beforeCourseMarker;
uint32_t 	cntMarker = 0;

// ログ関連
uint32_t 	goalTime = 0;

///////////////////////////////////////////////////////////////////////////
// モジュール名 systemInit
// 処理概要     初期化処理
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void initSystem (void) {
	uint8_t rawData[6];

	// ADC
	if (HAL_ADC_Start_DMA(&hadc1, analogVal, 10) != HAL_OK)	Error_Handler();

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

	// Display
	ssd1306_Init();
	ssd1306_Fill(Black);
	
	// Timer interrupt
	HAL_TIM_Base_Start_IT(&htim6);
	// HAL_TIM_Base_Start_IT(&htim7);

	// トップバー表示
	// 電池マーク
	ssd1306_DrawRectangle(96,3,125,14,White);
	ssd1306_DrawRectangle(126,5,127,12,White);
	ssd1306_UpdateScreen();
	HAL_Delay(100);
	showBattery();	// バッテリ残量

	// printf("boot Klic_RT_v2\n");
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 systemLoop
// 処理概要     メインループ
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void loopSystem (void) {
	switch (patternTrace) {
      	case 0:
			setup();

			if (start) {
				ssd1306_SetCursor(30,25);
				ssd1306_printf(Font_11x18,"Ready");
				ssd1306_UpdateScreen();  // グラフィック液晶更新
				cntRun = 0;
				countdown = 6000;		// カウントダウンスタート
				powerLinesensors(1);	// ラインセンサ ON
				patternTrace = 1;
			}
			break;
		case 1:
			// カウントダウンスタート
			motorPwmOutSynth( lineTraceCtrl.pwm, 0, 0, 0);
//			if ( countdown >= 5000 ) ledOut(0x5);
//			if ( countdown < 5000 && countdown >= 4000 ) ledOut(0x4);
//			if ( countdown < 4000 && countdown >= 3000 ) ledOut(0x3);
//			if ( countdown < 3000 && countdown >= 2000 ) ledOut(0x2);
//			if ( countdown < 2000 && countdown >= 1000 ) ledOut(0x1);

			if ( countdown <= 1000 ) {
				motorPwmOut(0,0);	// モータドライバICのスリープモードを解除
				modeLCD = false;	// LCD OFF
				useIMU = true;		// IMU 使用開始
				// Logファイル作成
				// if (initMSD) initLog();
				
				// 変数初期化
				encTotalN = 0;
				// distanceStart = 0;
				encRightMarker = encMM(600);
				cntRun = 0;
				BMI088val.angle.x = 0.0f;
				BMI088val.angle.y = 0.0f;
				BMI088val.angle.z = 0.0f;
				veloCtrl.Int = 0.0;
				yawRateCtrl.Int = 0.0;
				yawCtrl.Int = 0.0;

				// modeLOG = true;    // log start
				patternTrace = 11;
			}
			break;

      	case 11:
			// 目標速度設定
			setTargetSpeed(targetParam.straight);
			// if (optimalTrace == 0){
			// 	// 探索走行のとき
			// 	if (modeCurve == 0) {
			// 		setTargetSpeed(targetParam.straight);
			// 	} else {
			// 		setTargetSpeed(targetParam.curve);
			// 	}
			// } else if (optimalTrace == BOOST_MARKER) {
			// 	// マーカー基準2次走行
			// 	boostSpeed = PPAM[cntMarker].boostSpeed;
            //     // 次のマーカー区間の曲率半径が小さい時、速度を抑える
            //     if ( cntMarker < numPPADarry && fabs(ROCmarker[cntMarker+1]) <= 200.0F ) {
            //         boostSpeed = boostSpeed - 4;
            //     }
            //     // 最低速度
            //     if ( boostSpeed < 13 ) boostSpeed = 13;

            //     // 目標速度に設定
            //     setTargetSpeed(boostSpeed);
			// } else if (optimalTrace == BOODT_DISTANCE) {
			// 	// 距離基準2次走行
			// 	// スタートマーカーを超えた時から距離計測開始
			// 	if (SGmarker > 0 && distanceStart == 0) {
			// 		distanceStart = encTotalN;
			// 	}
			// 	// 一定区間ごとにインデックスを更新
			// 	if (distanceStart > 0) {
			// 		if (encTotalN - distanceStart >= encMM(CALCDISTANCE)) {
			// 			boostSpeed = PPAD[optimalIndex].boostSpeed;	// 目標速度を更新
			// 			distanceStart = encTotalN;	// 距離計測位置を更新
			// 			if (optimalIndex+1 <= numPPADarry) {
			// 				// 配列要素数を超えない範囲でインデックスを更新する
			// 				optimalIndex++;
			// 			}
			// 		}
			// 	} else if (distanceStart == 0) {
			// 		boostSpeed = targetParam.boostStraight;
			// 	}
			// 	// 目標速度に設定
			// 	setTargetSpeed(boostSpeed);
			// }
			
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
			if (enc1 >= encMM(500)) {
				setTargetSpeed(0);
			} else {
				setTargetSpeed(targetParam.stop);
			}
			motorPwmOutSynth( lineTraceCtrl.pwm, veloCtrl.pwm, 0, 0);
			
			if (encCurrentN == 0 && enc1 >= encMM(500)) {
				emargencyStop();
				break;
			}
			
			break;

      	case 102:
			motorPwmOutSynth( 0, 0, 0, 0);
			powerLinesensors(0);

			ssd1306_SetCursor(24,25);
			ssd1306_printf(Font_11x18,"Finish");
			ssd1306_SetCursor(0,45);
			ssd1306_printf(Font_11x18,"Time:%2.1f",goalTime/1000);
			ssd1306_UpdateScreen();  // グラフィック液晶更新
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
	// if (modeLOG) endLog();
	useIMU = false;		// IMU 使用開始
	modeLCD = true;
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
	// static uint8_t 	checkStraight, checkRight, checkLeft;
	// static float	angleCurve;

	// angleCurve += BNO055val.gyro.z * DEFF_TIME;

	// if (fabs(BNO055val.gyro.z) < 1.9f) {
	// 	// ストレート時
	// 	checkRight = 0;
	// 	checkLeft = 0;
	// 	if (checkStraight == 0) {
	// 		encCurve = 0;
	// 		checkStraight = 1;
	// 	}
	// 	if(checkStraight == 1 && encCurve > encMM(100)){
	// 		modeCurve = 0;
	// 		angleCurve = 0;
	// 	}
	// } else if (BNO055val.gyro.z > 2.5f) {
	// 	// 左カーブ時
	// 	checkStraight = 0;
	// 	checkRight = 0;
	// 	if (checkLeft == 0) {
	// 		encCurve = 0;
	// 		checkLeft = 1;
	// 	}
	// 	if(checkLeft == 1 && encCurve > encMM(20)){
	// 		modeCurve = 2;
	// 	}
	// } else if (BNO055val.gyro.z < -2.5f) {
	// 	// 右カーブ時
	// 	checkStraight = 0;
	// 	checkLeft = 0;
	// 	if (checkRight == 0) {
	// 		encCurve = 0;
	// 		checkRight = 1;
	// 	}
	// 	if(checkRight == 1 && encCurve > encMM(20)){
	// 		modeCurve = 1;
	// 	}
	// }
}
