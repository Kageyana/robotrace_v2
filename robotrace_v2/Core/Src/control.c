//====================================//
// インクルード
//====================================//
#include "control.h"
#include "fatfs.h"
//====================================//
// グローバル変数の宣言
//====================================//
// モード関連
uint8_t patternTrace = 0;
bool modeDSP = false;	  // ディスプレイ表示可否			false:消灯		true:表示
bool modeLOG = false;	  // ログ取得状況			false:ログ停止	true:ログ取得中
bool initMSD = false;	  // microSD初期化状況	false:初期化失敗	true:初期化成功
bool initLCD = false;	  // LCD初期化状況		false:初期化失敗	true:初期化成功
bool initIMU = false;	  // IMU初期化状況		false:初期化失敗	true:初期化成功
bool initCurrent = false; // 電流センサ初期化状況		false:初期化失敗	true:初期化成功
uint8_t modeCurve = 0;	  // カーブ判断			0:直線			1:カーブ進入
uint8_t autoStart = 0;	  // 5走を自動で開始する

uint16_t analogVal1[NUM_SENSORS]; // ADC結果格納配列
uint16_t analogVal2[4];			  // ADC結果格納配列

// 速度パラメータ関連
speedParam tgtParam = {
	PARAM_STRAIGHT,
	PARAM_CURVE,
	PARAM_STOP,
	PARAM_BOOST_STRAIGHT,
	PARAM_BOOST_1500,
	PARAM_BOOST_1300,
	PARAM_BOOST_1000,
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
	PARAM_SHORTCUT};

// タイマ関連
uint32_t cntRun = 0;
int16_t countdown;

// マーカー関連
uint8_t courseMarker;
uint8_t beforeCourseMarker;
uint32_t cntMarker = 0;
uint8_t courseMarkerLog;
int32_t straightMeter = 0;
bool straightState = false;

// ログ関連
uint32_t goalTime = 0;

///////////////////////////////////////////////////////////////////////////
// モジュール名 systemInit
// 処理概要     初期化処理
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void initSystem(void)
{
	HAL_StatusTypeDef resultHAL[10] = {};
	bool statusGPIO = true;

	// ADC
	resultHAL[0] = HAL_ADC_Start_DMA(&hadc1, analogVal1, NUM_SENSORS);
	resultHAL[1] = HAL_ADC_Start_DMA(&hadc2, analogVal2, 4);

	// Encoder count
	resultHAL[2] = HAL_TIM_Encoder_Start(&ENC_TIM_HANDLER_R, TIM_CHANNEL_ALL);
	resultHAL[3] = HAL_TIM_Encoder_Start(&ENC_TIM_HANDLER_L, TIM_CHANNEL_ALL);

	// Motor driver
	resultHAL[4] = HAL_TIM_PWM_Start(&MOTOR_TIM_HANDLER, MOTOR_TIM_CH_L);
	resultHAL[5] = HAL_TIM_PWM_Start(&MOTOR_TIM_HANDLER, MOTOR_TIM_CH_R);
	resultHAL[6] = HAL_TIM_PWM_Start(&MOTOR_TIM_HANDLER, MOTOR_SUCTION_TIM_CH);

	__HAL_TIM_SET_COMPARE(&MOTOR_TIM_HANDLER, MOTOR_TIM_CH_L, 0);
	__HAL_TIM_SET_COMPARE(&MOTOR_TIM_HANDLER, MOTOR_TIM_CH_R, 0);
	__HAL_TIM_SET_COMPARE(&MOTOR_TIM_HANDLER, MOTOR_SUCTION_TIM_CH, 0);
	motorPwmOut(0, 0);
	MotorFanPwmOut(0);

	// line sensor PWM
	resultHAL[7] = HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_3);

	powerLineSensors(0);
	powerMarkerSensors(0);

	// AD値取得
	getADC2();
	HAL_Delay(100);

	// Extended board
	if (swValTactAD > 2000) // 5方向タクトスイッチのプルアップを検出したら拡張ボードを接続している
	{
		// Display
		modeDSP = true;
		ssd1306_Init();
		ssd1306_Fill(Black);
		// トップバー表示

		SchmittBatery(); // バッテリレベルを取得
		showBatMark();	 // 電池マーク
		showBattery();	 // 充電Lv表示
		ssd1306_SetCursor(0, 16);
		ssd1306_printf(Font_6x8, "Exboard connect");
		ssd1306_UpdateScreen();

		setLED(0, 0, 50, 0); // 初期化 成功 緑点灯
	}
	else
	{
		modeDSP = false;
		setLED(0, 50, 0, 0); // 初期化 失敗 赤点灯
	}
	sendLED();

	// microSD
	ssd1306_SetCursor(0, 28);
	if (insertSD())
	{
		initMSD = initMicroSD();
		if (initMSD)
		{
			getLogNumber(); // 前回の解析ログナンバーを取得

			// 前回のPIDゲインを取得
			readPIDparameters(&lineTraceCtrl);
			readPIDparameters(&veloCtrl);
			readPIDparameters(&yawRateCtrl);
			readPIDparameters(&yawCtrl);
			readPIDparameters(&distCtrl);

			readLinesenval(); // ラインセンサオフセット値を取得
			readTgtspeeds();  // 目標速度を取得

			if (modeDSP)
			{
				ssd1306_printf(Font_6x8, "MicroSD success");
			}
			setLED(1, 0, 50, 0); // 初期化 成功 緑点灯
		}
	}
	else
	{
		if (modeDSP)
		{
			ssd1306_printf(Font_6x8, "MicroSD failed");
		}
		setLED(1, 50, 0, 0); // 初期化 失敗 赤点灯
	}
	if (modeDSP)
	{
		ssd1306_UpdateScreen(); // グラフィック液晶更新
	}
	sendLED();

	// IMU
	initIMU = initBMI088();
	ssd1306_SetCursor(0, 40);
	if (initIMU)
	{
		if (modeDSP)
		{
			ssd1306_printf(Font_6x8, "IMU     success");
		}
		setLED(2, 0, 50, 0); // 初期化 成功 緑点灯
	}
	else
	{
		if (modeDSP)
		{
			ssd1306_printf(Font_6x8, "IMU     failed");
		}
		setLED(2, 50, 0, 0); // 初期化 失敗 赤点灯
	}
	if (modeDSP)
	{
		ssd1306_UpdateScreen(); // グラフィック液晶更新
	}
	sendLED();

	// Timer interrupt
	resultHAL[8] = HAL_TIM_Base_Start_IT(&htim6);
	resultHAL[9] = HAL_TIM_Base_Start_IT(&htim7);

	HAL_Delay(100);

	// 各機能スタート時ののエラーチェック
	ssd1306_SetCursor(0, 52);
	uint8_t j = 0;
	for (uint8_t i = 0; i < 10; i++)
	{
		j += resultHAL[i];
		if (j > 0)
		{
			statusGPIO = false;
		}
	}
	if (statusGPIO)
	{
		setLED(3, 0, 50, 0); // 初期化 成功 緑点灯
		if (modeDSP)
		{
			ssd1306_printf(Font_6x8, "GPIO    success");
		}
	}
	else
	{
		setLED(3, 50, 0, 0); // 初期化 失敗 赤点灯
		if (modeDSP)
		{
			ssd1306_printf(Font_6x8, "GPIO    failed");
		}
		Error_Handler();
	}
	if (modeDSP)
	{
		ssd1306_UpdateScreen(); // グラフィック液晶更新
	}
	sendLED();

	HAL_Delay(1000);

	// Sd card未挿入の警告
	if (!insertSD())
	{
		if (modeDSP)
		{
			ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
			ssd1306_SetCursor(15, 30);
			ssd1306_printf(Font_11x18, "No SDcard");
			ssd1306_UpdateScreen(); // グラフィック液晶更新

			HAL_Delay(1000);
		}
	}

	clearLED();
	// DWT初期化
	initCycleCounter();
	resetCycleCounter();
	enableCycleCounter(); // カウント開始

	printf("hello \n");
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 systemLoop
// 処理概要     メインループ
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void loopSystem(void)
{

	// 緊急停止処理
	if (patternTrace > 10 && patternTrace < 100 && emcStop > 0)
	{
		goalTime = cntRun;
		emargencyStop();
	}

	switch (patternTrace)
	{
	case 0:
		if (autoStart > 1)
		{
			// 2次走行
			motorPwmOut(0, 0);
			// 初期化
			if (autoStart == 2)
			{
				getFileNumbers(); // 1次走行のログ番号取得
			}
			SGmarker = 0;
			// 目標速度調整

			// コース解析
			ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
			ssd1306_SetCursor(0, 25);
			ssd1306_printf(Font_11x18, "Analizing");
			ssd1306_UpdateScreen(); // グラフィック液晶更新
			initIMU = false;
			numPPADarry = readLogDistance(fileNumbers[fileIndexLog]);
			optimalIndex = 0;
			initIMU = true;

			countdown = 2000;							  // カウントダウンスタート
			ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
			ssd1306_SetCursor(56, 28);
			ssd1306_printf(Font_16x26, "2");
			ssd1306_UpdateScreen(); // グラフィック液晶更新
			patternTrace = 1;
		}
		else
		{
			if (modeDSP)
			{
				setup();
			}
			else
			{
				// ディスプレイモジュールが接続されていない時
				setupNonDisp();
			}

			if (start || autoStart)
			{
				motorPwmOut(0, 0);
				countdown = 5000;							  // カウントダウンスタート
				ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
				ssd1306_SetCursor(56, 28);
				ssd1306_printf(Font_16x26, "5");
				ssd1306_UpdateScreen(); // グラフィック液晶更新
				patternTrace = 1;
			}
		}
		break;

	case 1:
		// カウントダウンスタート
		if (modeDSP)
		{
			if (countdown == 4000)
			{
				ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
				ssd1306_SetCursor(56, 28);
				ssd1306_printf(Font_16x26, "4");
				ssd1306_UpdateScreen(); // グラフィック液晶更新
			}
			if (countdown == 3000)
			{
				ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
				ssd1306_SetCursor(56, 28);
				ssd1306_printf(Font_16x26, "3");
				ssd1306_UpdateScreen(); // グラフィック液晶更新
			}
			if (countdown == 2000)
			{
				ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
				ssd1306_SetCursor(56, 28);
				ssd1306_printf(Font_16x26, "2");
				ssd1306_UpdateScreen(); // グラフィック液晶更新
			}
			if (countdown == 1000)
			{
				ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
				ssd1306_SetCursor(56, 28);
				ssd1306_printf(Font_16x26, "1");
				ssd1306_UpdateScreen(); // グラフィック液晶更新
				calibratIMU = true;		// IMUキャリブレーションを開始
			}
		}

		// IMUのキャリブレーションが終了したら走行開始
		if (!calibratIMU && countdown == 0)
		{
			powerLineSensors(1);   // ラインセンサ点灯
			powerMarkerSensors(1); // マーカーセンサ点灯

			// SDカードに変数保存
			initIMU = false;

			// PIDゲインを記録
			if (autoStart == 0)
			{
				writePIDparameters(&lineTraceCtrl);
				writePIDparameters(&veloCtrl);
				writePIDparameters(&yawRateCtrl);
				writePIDparameters(&yawCtrl);
				writePIDparameters(&distCtrl);
			}

			if (autoStart <= 1)
			{
				writeTgtspeeds(); // 目標速度を記録
			}

			if (initMSD)
				initLog(); // ログ一時ファイル作成

			initIMU = true;

			// 変数初期化
			encRightMarker = encMM(1000);
			veloCtrl.Int = 0.0;
			yawRateCtrl.Int = 0.0;

			patternTrace = 11;
		}
		break;

	case 11:
		// スタートマーカー通過までの走行
		setTargetSpeed(tgtParam.straight); // 目標速度
		// ライントレース
		motorPwmOutSynth(lineTraceCtrl.pwm, veloCtrl.pwm, 0, 0);

		// スタートマーカーを通過したら本走行に移行
		if (SGmarker > 0)
		{
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
			straightMeter = 0;
			straightState = false;

			clearXYcie(); // 座標計算変数初期化

			if (initMSD)
				modeLOG = true; // log start

			patternTrace = 12;
		}
		break;

	case 12:
		// 目標速度設定
		if (optimalTrace == 0)
		{
			// 探索走行のとき
			if (modeCurve == 0)
			{
				setTargetSpeed(tgtParam.straight);
			}
			else
			{
				setTargetSpeed(tgtParam.curve);
			}
			// ライントレース
			motorPwmOutSynth(lineTraceCtrl.pwm, veloCtrl.pwm, 0, 0);
		}
		else if (optimalTrace == BOOST_DISTANCE)
		{
			// 距離基準2次走行
			// スタートマーカーを超えた時から距離計測開始
			if (SGmarker > 0 && DistanceOptimal == 0)
			{
				DistanceOptimal = encTotalOptimal;
			}
			// 一定区間ごとにインデックスを更新
			if (DistanceOptimal > 0)
			{
				if (encTotalOptimal - DistanceOptimal >= encMM(CALCDISTANCE))
				{
					boostSpeed = PPAD[optimalIndex].boostSpeed; // 目標速度を更新
					DistanceOptimal = encTotalOptimal;			// 距離計測位置を更新
					if (optimalIndex + 1 <= numPPADarry)
					{
						// 配列要素数を超えない範囲でインデックスを更新する
						optimalIndex++;
					}
				}
			}
			else if (DistanceOptimal == 0)
			{
				boostSpeed = tgtParam.straight;
			}
			// 目標速度に設定
			setTargetSpeed(boostSpeed);
			// ライントレース
			motorPwmOutSynth(lineTraceCtrl.pwm, veloCtrl.pwm, 0, 0);
		}
		else if (optimalTrace == BOOST_SHORTCUT)
		{
			// ショートカット2次走行
			// スタートマーカーを超えた時から距離計測開始
			if (SGmarker > 0 && DistanceOptimal == 0)
			{
				DistanceOptimal = encTotalOptimal;
				// 初期目標値をセット
				optimalIndex = 1;
				setShortCutTarget();
			}
			boostSpeed = tgtParam.shortCut;
			// 目標速度に設定
			setTargetSpeed(boostSpeed);
			// ライントレース
			motorPwmOutSynth(0, veloCtrl.pwm, yawCtrl.pwm, distCtrl.pwm);
		}

		// ゴール判定
		if (optimalTrace != BOOST_SHORTCUT)
		{
			if (SGmarker >= COUNT_GOAL)
			{
				goalTime = cntRun;
				enc1 = 0;
				patternTrace = 101;
			}
		}
		else
		{
			if (optimalIndex >= numPPADarry)
			{
				goalTime = cntRun;
				enc1 = 0;
				patternTrace = 101;
			}
		}

		break;

	case 101:
		// 停止速度まで減速
		if (enc1 >= encMM(200))
		{
			setTargetSpeed(0);
		}
		else
		{
			setTargetSpeed(tgtParam.stop);
		}
		motorPwmOutSynth(lineTraceCtrl.pwm, veloCtrl.pwm, 0, 0);

		if (encCurrentN == 0)
		{
			motorPwmOutSynth(0, 0, 0, 0);

			ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
			ssd1306_SetCursor(0, 25);
			ssd1306_printf(Font_11x18, "log");
			ssd1306_SetCursor(0, 45);
			ssd1306_printf(Font_11x18, "Writing");
			ssd1306_UpdateScreen(); // グラフィック液晶更新

			if (modeLOG)
				endLog(); // ログ保存終了

			ssd1306_SetCursor(0, 45);
			ssd1306_printf(Font_11x18, "Written");
			ssd1306_UpdateScreen(); // グラフィック液晶更新

			if (autoStart > 0)
			{
				autoStart++;
				if (autoStart >= 3)
				{
					tgtParam.bstStraight *= PARAM_UP_STEP;
					tgtParam.bst1500 *= PARAM_UP_STEP;
					tgtParam.bst1300 *= PARAM_UP_STEP;
					tgtParam.bst1000 *= PARAM_UP_STEP;
					tgtParam.bst800 *= PARAM_UP_STEP;
					tgtParam.bst700 *= PARAM_UP_STEP;
					tgtParam.bst600 *= PARAM_UP_STEP;
					tgtParam.bst500 *= PARAM_UP_STEP;
					// tgtParam.bst400			*= PARAM_UP_STEP;
					// tgtParam.bst300			*= PARAM_UP_STEP;
					// tgtParam.bst200			*= PARAM_UP_STEP;
					// tgtParam.bst100			*= PARAM_UP_STEP;
				}

				if (autoStart > 5)
				{
					ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
					ssd1306_SetCursor(0, 25);
					ssd1306_printf(Font_11x18, "Auto run");
					ssd1306_SetCursor(0, 45);
					ssd1306_printf(Font_11x18, "Finish!");
					ssd1306_UpdateScreen(); // グラフィック液晶更新
					patternTrace = 102;
					break;
				}
				else
				{
					powerLineSensors(0);
					powerMarkerSensors(0);
					patternTrace = 0;
					break;
				}
			}
			else
			{
				if (modeDSP)
				{
					ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
					ssd1306_SetCursor(0, 25);
					ssd1306_printf(Font_11x18, "Time %d", optimalTrace);
					ssd1306_SetCursor(0, 45);
					ssd1306_printf(Font_11x18, "%6.3f[s]", (float)goalTime / 1000);
					ssd1306_UpdateScreen(); // グラフィック液晶更新
				}
			}

			goalTime = cntRun;
			patternTrace = 102;
			break;
		}

		break;

	case 102:
		motorPwmOutSynth(0, 0, 0, 0);
		powerLineSensors(0);
		powerMarkerSensors(0);

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
void emargencyStop(void)
{

	motorPwmOutSynth(0, 0, 0, 0);

	if (modeLOG)
		endLog(); // ログ保存終了

	if (modeDSP)
	{
		ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め

		ssd1306_SetCursor(36, 25);
		ssd1306_printf(Font_11x18, "EMS!! %d", optimalTrace);

		ssd1306_SetCursor(0, 45);
		switch (emcStop)
		{
		case STOP_ANGLE_X:
			ssd1306_printf(Font_7x10, "ANGLE_X");
			break;
		case STOP_ANGLE_Y:
			ssd1306_printf(Font_7x10, "ANGLE_Y");
			break;
		case STOP_ENCODER_STOP:
			ssd1306_printf(Font_7x10, "ENCODER_STOP");
			break;
		case STOP_LINESENSOR:
			ssd1306_printf(Font_7x10, "LINESENSOR");
			break;
		case STOP_OVERSPEED:
			ssd1306_printf(Font_7x10, "OVERSPEED");
			break;
		}

		ssd1306_UpdateScreen(); // グラフィック液晶更新
	}

	patternTrace = 102;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 countDown
// 処理概要     カウントダウン
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void countDown(void)
{
	if (countdown > 0)
		countdown--;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 checkCurve
// 処理概要     カーブとストレートの判定
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void checkCurve(void)
{
	static uint8_t checkStraight, checkRight, checkLeft;
	float zg;

	zg = BMI088val.gyro.z;

	if (fabs(zg) < 50.0f)
	{
		// ストレート時
		checkRight = 0;
		checkLeft = 0;
		if (checkStraight == 0)
		{
			encCurve = 0;
			checkStraight = 1;
		}
		if (checkStraight == 1 && encCurve > encMM(100))
		{
			modeCurve = 0;
		}
	}
	else if (zg < -150.0f)
	{
		// 左カーブ時
		checkStraight = 0;
		checkRight = 0;
		if (checkLeft == 0)
		{
			encCurve = 0;
			checkLeft = 1;
		}
		if (checkLeft == 1 && encCurve > encMM(20))
		{
			modeCurve = 2;
		}
	}
	else if (zg > 150.0f)
	{
		// 右カーブ時
		checkStraight = 0;
		checkLeft = 0;
		if (checkRight == 0)
		{
			encCurve = 0;
			checkRight = 1;
		}
		if (checkRight == 1 && encCurve > encMM(20))
		{
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
void getADC2(void)
{
	getMotorAD(analogVal2[0], analogVal2[1]);
	getBatteryAD(analogVal2[2]);
	getSwitchAD(analogVal2[3]);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 setEncoderVal
// 処理概要     エンコーダ値を変数に加算する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void setEncoderVal(void)
{
	// 外部変数
	enc1 += encCurrentN;			// 通常トレース用
	encRightMarker += encCurrentN;	// ゴールマーカ判定用
	encCurve += encCurrentN;		// カーブ処理用
	encTotalOptimal += encCurrentN; // 2次走行用
	encLog += encCurrentN;			// 一定距離ごとにログを保存する用
	encPID += encCurrentN;			// 距離制御用
	encClick += encCurrentL;		// ホイールクリック用
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 writePIDparameters
// 処理概要     PIDゲインをSDカードに記録する
// 引数         pid:pidParam型の変数
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void writeTgtspeeds(void)
{
	FIL fil;
	FRESULT fresult;
	uint8_t format[100] = "", fileName[30] = PATH_SETTING;
	int16_t i, ret = 0;

	// ファイル読み込み
	strcat(fileName, FILENAME_TARGET_SPEED);					 // ファイル名追加
	strcat(fileName, ".txt");									 // 拡張子追加
	fresult = f_open(&fil, fileName, FA_OPEN_ALWAYS | FA_WRITE); // ファイルを開く

	if (fresult == FR_OK)
	{
		for (i = 0; i < sizeof(speedParam) / sizeof(float); i++)
		{
			strcat(format, "%03d,");
		}

		f_printf(&fil, format, (int32_t)(round(tgtParam.straight * 100)), (int32_t)(round(tgtParam.curve * 100)), (int32_t)(round(tgtParam.stop * 100)), (int32_t)(round(tgtParam.bstStraight * 100)), (int32_t)(round(tgtParam.bst1500 * 100)), (int32_t)(round(tgtParam.bst1300 * 100)), (int32_t)(round(tgtParam.bst1000 * 100)), (int32_t)(round(tgtParam.bst800 * 100)), (int32_t)(round(tgtParam.bst700 * 100)), (int32_t)(round(tgtParam.bst600 * 100)), (int32_t)(round(tgtParam.bst500 * 100)), (int32_t)(round(tgtParam.bst400 * 100)), (int32_t)(round(tgtParam.bst300 * 100)), (int32_t)(round(tgtParam.bst200 * 100)), (int32_t)(round(tgtParam.bst100 * 100)), (int32_t)(round(tgtParam.acceleF * 100)), (int32_t)(round(tgtParam.acceleD * 100)), (int32_t)(round(tgtParam.shortCut * 100)));
	}

	f_close(&fil);
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 readPIDparameters
// 処理概要     PIDゲインをSDカードから読み取る
// 引数         pid:pidParam型の変数
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void readTgtspeeds(void)
{
	FIL fil;
	FRESULT fresult;
	uint8_t format[100] = "", fileName[30] = PATH_SETTING;
	int16_t param[20];
	TCHAR paramStr[100];
	int16_t i, ret = 0;

	// ファイル読み込み
	strcat(fileName, FILENAME_TARGET_SPEED);					  // ファイル名追加
	strcat(fileName, ".txt");									  // 拡張子追加
	fresult = f_open(&fil, fileName, FA_OPEN_EXISTING | FA_READ); // ファイルを開く

	if (fresult == FR_OK)
	{
		for (i = 0; i < sizeof(speedParam) / sizeof(float); i++)
		{
			f_gets(paramStr, 5, &fil);			// 文字列取得 カンマ含む
			sscanf(paramStr, "%d,", &param[i]); // 文字列→数値
		}

		tgtParam.straight = (float)param[0] / 100;
		tgtParam.curve = (float)param[1] / 100;
		tgtParam.stop = (float)param[2] / 100;
		tgtParam.bstStraight = (float)param[3] / 100;
		tgtParam.bst1500 = (float)param[4] / 100;
		tgtParam.bst1300 = (float)param[5] / 100;
		tgtParam.bst1000 = (float)param[6] / 100;
		tgtParam.bst800 = (float)param[7] / 100;
		tgtParam.bst700 = (float)param[8] / 100;
		tgtParam.bst600 = (float)param[9] / 100;
		tgtParam.bst500 = (float)param[10] / 100;
		tgtParam.bst400 = (float)param[11] / 100;
		tgtParam.bst300 = (float)param[12] / 100;
		tgtParam.bst200 = (float)param[13] / 100;
		tgtParam.bst100 = (float)param[14] / 100;
		tgtParam.acceleF = (float)param[15] / 100;
		tgtParam.acceleD = (float)param[16] / 100;
		tgtParam.shortCut = (float)param[17] / 100;
	}

	f_close(&fil);
}
