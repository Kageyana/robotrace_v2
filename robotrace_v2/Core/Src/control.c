//====================================//
// インクルード
//====================================//
#include "control.h"
#include "fatfs.h"
//====================================//
// グローバル変数の宣言
//====================================//
// モード関連
uint8_t	patternTrace = 0;
bool    modeDSP = false;		// ディスプレイ表示可否			false:消灯		true:表示
bool 	modeLOG = false;	// ログ取得状況			false:ログ停止	true:ログ取得中
bool    initMSD = false;	// microSD初期化状況	false:初期化失敗	true:初期化成功
bool    initLCD = false;    // LCD初期化状況		false:初期化失敗	true:初期化成功
bool    initIMU = false;    // IMU初期化状況		false:初期化失敗	true:初期化成功
bool    initCurrent = false;    // 電流センサ初期化状況		false:初期化失敗	true:初期化成功
uint8_t modeCurve = 0;		// カーブ判断			0:直線			1:カーブ進入

uint16_t 	analogVal1[NUM_SENSORS];		// ADC結果格納配列
uint16_t 	analogVal2[3];		// ADC結果格納配列

// 速度パラメータ関連
speedParam tgtParam = {	
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
	PARAM_SHORTCUT
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

	// microSD
	initMSD = initMicroSD();
	if(initMSD) {
		getLogNumber(); // 前回の解析ログナンバーを取得

		// 前回のPIDゲインを取得
		readPIDparameters(&lineTraceCtrl);
		readPIDparameters(&veloCtrl);
		readPIDparameters(&yawRateCtrl);
		readPIDparameters(&yawCtrl);
		readPIDparameters(&distCtrl);

		readLinesenval();	// ラインセンサオフセット値を取得
		readTgtspeeds();	// 目標速度を取得
	}
	// IMU
	initIMU = initBMI088();
	
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
	HAL_TIM_Base_Start_IT(&htim7);

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
				calibratIMU = true;		// IMUキャリブレーションを開始
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

			// IMUのキャリブレーションが終了したら走行開始
			// if ( !calibratIMU ) {
				powerLinesensors(1);	// ラインセンサ点灯

				// SDカードに変数保存
				initIMU = false;

				// PIDゲインを記録
				writePIDparameters(&lineTraceCtrl);
				writePIDparameters(&veloCtrl);
				writePIDparameters(&yawRateCtrl);
				writePIDparameters(&yawCtrl);
				writePIDparameters(&distCtrl);

				writeTgtspeeds();	// 目標速度を記録

#ifdef LOG_RUNNING_WRITE
				if(initMSD) createLog();    // ログファイル作成
#endif
				initIMU = true;

				// 変数初期化
				encRightMarker = encMM(600);
				veloCtrl.Int = 0.0;
				yawRateCtrl.Int = 0.0;



				patternTrace = 11;
			// }
			break;

		case 11:
			// スタートマーカー通過までの走行
			setTargetSpeed(tgtParam.straight); // 目標速度
			// ライントレース
			motorPwmOutSynth( lineTraceCtrl.pwm, veloCtrl.pwm, 0, 0);

			// スタートマーカーを通過したら本走行に移行
			if(SGmarker > 0) {
				// 変数初期化
				encTotalN = 0;
				encTotalOptimal = 0;
				encLog = 0;
				DistanceOptimal = 0;
				cntRun = 0;
				cntLog = 0;
				BMI088val.angle.x = 0.0F;
				BMI088val.angle.y = 0.0F;
				BMI088val.angle.z = 0.0F;
				yawCtrl.Int = 0.0;
				distCtrl.Int = 0.0;

				clearXYcie();	// 座標計算変数初期化

				if(initMSD) modeLOG = true;    // log start
				
				patternTrace = 12;
			}
			break;

      	case 12:
			// 目標速度設定
			if (optimalTrace == 0){
				// 探索走行のとき
				if (modeCurve == 0) {
					setTargetSpeed(tgtParam.straight);
				} else {
					setTargetSpeed(tgtParam.curve);
				}
				// ライントレース
				motorPwmOutSynth( lineTraceCtrl.pwm, veloCtrl.pwm, 0, 0);
			} else if (optimalTrace == BOOST_DISTANCE) {
				// 距離基準2次走行
				// スタートマーカーを超えた時から距離計測開始
				if (SGmarker > 0 && DistanceOptimal == 0) {
					DistanceOptimal = encTotalOptimal;
				}
				// 一定区間ごとにインデックスを更新
				if (DistanceOptimal > 0) {
					if (encTotalOptimal - DistanceOptimal >= encMM(CALCDISTANCE)) {
						boostSpeed = PPAD[optimalIndex].boostSpeed;	// 目標速度を更新
						DistanceOptimal = encTotalOptimal;	// 距離計測位置を更新
						if (optimalIndex+1 <= numPPADarry) {
							// 配列要素数を超えない範囲でインデックスを更新する
							optimalIndex++;
						}
					}
				} else if (DistanceOptimal == 0) {
					boostSpeed = tgtParam.straight;
				}
				// 目標速度に設定
				setTargetSpeed(boostSpeed);
				// ライントレース
				motorPwmOutSynth( lineTraceCtrl.pwm, veloCtrl.pwm, 0, 0);
			} else if (optimalTrace == BOOST_SHORTCUT) {
				// ショートカット2次走行
				// スタートマーカーを超えた時から距離計測開始
				if (SGmarker > 0 && DistanceOptimal == 0) {
					DistanceOptimal = encTotalOptimal;
					// 初期目標値をセット
					optimalIndex = 1;
					setShortCutTarget();
				}
				boostSpeed = tgtParam.shortCut;
				// 目標速度に設定
				setTargetSpeed(boostSpeed);
				// ライントレース
				motorPwmOutSynth( 0, veloCtrl.pwm, yawCtrl.pwm, distCtrl.pwm);
			}
			
			// ゴール判定
			if(optimalTrace != BOOST_SHORTCUT) {
				if (SGmarker >= COUNT_GOAL ) {
					goalTime = cntRun;
					enc1 = 0;
					patternTrace = 101;
				}
			} else {
				if (optimalIndex >= numPPADarry) {
					goalTime = cntRun;
					enc1 = 0;
					patternTrace = 101;
				}
			}
			
			break;

      	case 101:
			// 停止速度まで減速
			if (enc1 >= encMM(500)) {
				setTargetSpeed(0);
			} else {
				setTargetSpeed(tgtParam.stop);
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
	
				goalTime = cntRun;
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
			case STOP_OVERSPEED:
				ssd1306_printf(Font_7x10,"OVERSPEED");
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
	} else if (zg < -150.0f) {
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
	} else if (zg > 150.0f) {
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
///////////////////////////////////////////////////////////////////////////
// モジュール名 writePIDparameters
// 処理概要     PIDゲインをSDカードに記録する
// 引数         pid:pidParam型の変数
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void writeTgtspeeds(void) {
	FIL         fil;
    FRESULT     fresult;
	uint8_t     format[100]="", fileName[30] = PATH_SETTING;
    int16_t     i, ret=0;

	// ファイル読み込み
	strcat(fileName,FILENAME_TARGET_SPEED); // ファイル名追加
	strcat(fileName,".txt");   // 拡張子追加
    fresult = f_open(&fil, fileName, FA_OPEN_ALWAYS | FA_WRITE);  	// ファイルを開く
	
	if(fresult == FR_OK) {
		for(i=0;i<sizeof(speedParam)/sizeof(float);i++) {
			strcat(format,"%03d,");
		}

		f_printf(&fil, format
			,(int32_t)(round(tgtParam.straight*100))
			,(int32_t)(round(tgtParam.curve*100))
			,(int32_t)(round(tgtParam.stop*100))
			,(int32_t)(round(tgtParam.bstStraight*100))
			,(int32_t)(round(tgtParam.bst1500*100))
			,(int32_t)(round(tgtParam.bst800*100))
			,(int32_t)(round(tgtParam.bst700*100))
			,(int32_t)(round(tgtParam.bst600*100))
			,(int32_t)(round(tgtParam.bst500*100))
			,(int32_t)(round(tgtParam.bst400*100))
			,(int32_t)(round(tgtParam.bst300*100))
			,(int32_t)(round(tgtParam.bst200*100))
			,(int32_t)(round(tgtParam.bst100*100))
			,(int32_t)(round(tgtParam.acceleF*100))
			,(int32_t)(round(tgtParam.acceleD*100))
			,(int32_t)(round(tgtParam.shortCut*100))
		);
	}

	f_close(&fil);
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 readPIDparameters
// 処理概要     PIDゲインをSDカードから読み取る
// 引数         pid:pidParam型の変数
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void readTgtspeeds(void) {
	FIL         fil;
    FRESULT     fresult;
	uint8_t     format[100]="", fileName[30] = PATH_SETTING;
	int16_t 	param[20];
	TCHAR     	paramStr[100];
    int16_t     i, ret=0;

	// ファイル読み込み
	strcat(fileName,FILENAME_TARGET_SPEED); // ファイル名追加
	strcat(fileName,".txt");   // 拡張子追加
    fresult = f_open(&fil, fileName, FA_OPEN_EXISTING | FA_READ);  // ファイルを開く

	if(fresult == FR_OK) {
		for(i=0;i<sizeof(speedParam)/sizeof(float);i++) {
			f_gets(paramStr,5,&fil);				// 文字列取得 カンマ含む
			sscanf(paramStr,"%d,",&param[i]);		// 文字列→数値
		}

		tgtParam.straight		= (float)param[0]/100;
		tgtParam.curve			= (float)param[1]/100;
		tgtParam.stop			= (float)param[2]/100;
		tgtParam.bstStraight	= (float)param[3]/100;
		tgtParam.bst1500		= (float)param[4]/100;
		tgtParam.bst800			= (float)param[5]/100;
		tgtParam.bst700			= (float)param[6]/100;
		tgtParam.bst600			= (float)param[7]/100;
		tgtParam.bst500			= (float)param[8]/100;
		tgtParam.bst400			= (float)param[9]/100;
		tgtParam.bst300			= (float)param[10]/100;
		tgtParam.bst200			= (float)param[11]/100;
		tgtParam.bst100			= (float)param[12]/100;
		tgtParam.acceleF		= (float)param[13]/100;
		tgtParam.acceleD		= (float)param[14]/100;
		tgtParam.shortCut		= (float)param[15]/100;
	}

	f_close(&fil);
}