//====================================//
// インクルード
//====================================//
#include "control.h"
#include "BMI088.h"
#include "PIDcontrol.h"
#include "motor.h"
#include "fatfs.h"
#include "battery.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
//====================================//
// グローバル変数の宣言
//====================================//
// モード関連
uint8_t patternTrace = 0;
bool modeDSP = false;		// ディスプレイ表示可否	false:消灯		true:表示
bool modeLOG = false;		// ログ取得状況			false:ログ停止	true:ログ取得中
bool initMSD = false;		// microSD初期化状況	false:初期化失敗	true:初期化成功
bool initLCD = false;		// LCD初期化状況		false:初期化失敗	true:初期化成功
bool initIMU = false;		// IMU初期化状況		false:初期化失敗	true:初期化成功
bool initCurrent = false;	// 電流センサ初期化状況	false:初期化失敗	true:初期化成功
static bool softreset = false;		// ソフトウェアリセット	false:リセット未実行	true:リセット実行
uint8_t autoStart = 0;				// 5走を自動で開始する
int16_t autoStartAnalize = 0; 		// 自動走行で使用するログの解析番号

bool stateCrossLine = false;			// クロスライン検出状態

uint16_t analogValLSon[NUM_SENSORS+1]; // ADC結果格納配列
uint16_t analogValLSoff[NUM_SENSORS+1]; // ADC結果格納配列s
uint16_t analogVal2[4];			  // ADC結果格納配列
float batteryVoltage_V = 0.0f;	// DWT初期化後に取得したバッテリ電圧[V]を保持

// 速度パラメータ関連
speedParam tgtParam = {
	PARAM_SEARCH,
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
// スリップ検出用の状態（1ms割り込みで軽量に処理するためここで管理）
static float slipEncSpeedHist[SLIP_WINDOW_SAMPLES];		// 時間窓の開始時点のエンコーダ由来速度[m/s]（リングバッファ）
static uint16_t slipBufIndex = 0;						// リングバッファの書き込み位置
static float slipDeltaImu = 0.0f;						// 窓内の横方向残差[m/s^2]
static float slipDeltaEnc = 0.0f;						// 窓内の縦方向残差[m/s^2]
static float slipIndicatorRaw = 0.0f;					// 縦スリップ指標の一次LPF後の値
static float slipIndicatorFiltered = 0.0f;				// 横スリップ指標の一次LPF後の値（slipRatio相当）
static uint16_t slipHighCount = 0;						// スリップ立ち上がり判定用の連続カウンタ
static uint16_t slipLowCount = 0;						// スリップ解除判定用の連続カウンタ
static uint16_t slipHighCountLat = 0;					// 横スリップ立ち上がり判定用の連続カウンタ
static uint16_t slipLowCountLat = 0;					// 横スリップ解除判定用の連続カウンタ
static bool slipFlag = false;							// 縦スリップ判定フラグ（slipFlagLong互換）
static bool slipFlagLat = false;						// 横スリップ判定フラグ
static float slipThresholdHigh;							// スリップ検出高閾値
static float slipThresholdLow;							// スリップ検出低閾値
// スリップ検出用の内部状態（RAM節約のためSLIP_CUR_ENABLEでメンバ切り替え）
typedef struct {
	bool prevRunning;
	bool prevMoving;
	float axBias;
	float ayBias;
	float imuAxF;
	float imuAyF;
	float encAxF;
	float encAyF;
	bool turningState;
	bool slipPrimed;
#if SLIP_CUR_ENABLE
	float pwmSumF;
	float iSumF;
#endif
} SlipDetState;

static SlipDetState slipDetState = {0};
// タイマ関連
uint32_t cntRun = 0;
int16_t countdown;

// マーカー関連
uint8_t courseMarker;
uint8_t beforeCourseMarker;
uint16_t cntMarker = 0;
uint8_t courseMarkerLog = 0;

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
	HAL_StatusTypeDef resultHAL[12] = {};
	bool statusGPIO = true;
	int16_t cntFiles = 0;

	// GPIO初期化 ソフトリセット中の場合は実行しない
	if(!softreset)
	{
		// ADC
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
		// line sensor PWM
		resultHAL[7] = HAL_TIM_Base_Start_IT(&htim3);
		resultHAL[8] = HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_3);
		resultHAL[9] = HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
	}
	motorPwmOut(0, 0);
	MotorFanPwmOut(0);
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
		ssd1306_UpdateScreen_DMA(); // グラフィック液晶更新開始

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
		if(!softreset)
		{
			initMSD = initMicroSD();
		}

		if(initMSD || softreset)
		{
			cntFiles = getFileNumbers();	// 走行ログのファイル番号を取得
			getLogNumber();		// 前回の解析ログナンバーを取得

			// 前回のPIDゲインを取得
			readPIDparameters(&lineTraceCtrl);
			readPIDparameters(&veloCtrl);
			readSpeedFeedForwardGain(&speedFeedForwardGain);	// 速度フィードフォワード係数も読み出す
			readPIDparameters(&yawRateCtrl);
			readPIDparameters(&yawCtrl);
			readPIDparameters(&distCtrl);
			readPIDparameters(&lineTraceOmegaFBCtrl);

			readLinesenval(); // ラインセンサの最大値と最小値を取得
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
		SSD1306_DMA_Completed = 0;				// 全ページ送信完了フラグリセット
		while(!ssd1306_IsTransferCompleted());	// 全ページ送信完了まで待つ
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
		SSD1306_DMA_Completed = 0;				// 全ページ送信完了フラグリセット
		while(!ssd1306_IsTransferCompleted());	// 全ページ送信完了まで待つ
	}
	sendLED();

	//HAL各機能スタート時のエラーチェック
	if(!softreset)
	{
		// Timer interrupt
		resultHAL[10] = HAL_TIM_Base_Start_IT(&htim6);
		resultHAL[11] = HAL_TIM_Base_Start_IT(&htim7);

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
			SSD1306_DMA_Completed = 0;				// 全ページ送信完了フラグリセット
			while(!ssd1306_IsTransferCompleted());	// 全ページ送信完了まで待つ
		}
		sendLED();
	}
	
	HAL_Delay(1000);

	// Sd card未挿入の警告
	if (!insertSD())
	{
		if (modeDSP)
		{
			ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
			ssd1306_SetCursor(15, 30);
			ssd1306_printf(Font_11x18, "No SDcard");
			SSD1306_DMA_Completed = 0; // 全ページ送信完了フラグリセット
			while(!ssd1306_IsTransferCompleted());	// 全ページ送信完了まで待つ

			HAL_Delay(1000);
		}
	}
	else
	{
		if(cntFiles > FILENUMBER_LIMIT)
		{
			ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
			ssd1306_SetCursor(20, 16);
			ssd1306_printf(Font_11x18, "Too many");
			ssd1306_SetCursor(14, 36	);
			ssd1306_printf(Font_11x18, "Log files");
			SSD1306_DMA_Completed = 0; // 全ページ送信完了フラグリセット
			while(!ssd1306_IsTransferCompleted());	// 全ページ送信完了まで待つ

			HAL_Delay(1000);
		}
	}

	clearLED();
	// DWT初期化
	initCycleCounter();
	resetCycleCounter();
	enableCycleCounter(); // カウント開始

	batteryVoltage_V = AD2VOLTAGE(batteryAD); // バッテリ電圧を計算

	encClick = 0; // ホイールクリッククリア

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
	int16_t ret = 0;

	if (patternTrace > 10 && patternTrace < 100)        // 走行中のみ処理
	{
		logWriteTask();        // 割り込み外でSD書き込みを実行する
	}

    // 緊急停止処理
	if (patternTrace > 10 && patternTrace < 100 && emcStop > 0)
	{
		goalTime = cntRun;
		patternTrace = 255;
		emargencyStop();
	}

	switch (patternTrace)
	{
	case 0:
		if (autoStart > 1)
		{
			// 2次走行
			motorPwmOut(0, 0);

			// 目標速度調整
			// コース解析
			ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
			ssd1306_SetCursor(0, 25);
			ssd1306_printf(Font_11x18, "Analizing");

			ret = readLogDistance(autoStartAnalize);	// 1次走行のログ番号を使用してコース解析
			if(ret > 0)
			{
				// コース解析成功
				countdown = 2000;							  // カウントダウンスタート
				ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
				ssd1306_SetCursor(56, 28);
				ssd1306_printf(Font_16x26, "2");

				patternTrace = 1;
			}
			else
			{
				// コース解析失敗
				ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
				ssd1306_SetCursor(30, 25);
				ssd1306_printf(Font_11x18, "Failed");
				ssd1306_SetCursor(20, 50);
				ssd1306_printf(Font_6x8, "Error %d", ret);

				HAL_Delay(1000);

				patternTrace = 0;
				autoStart = 0;
				autoStartAnalize = 0;
			}
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

            if (setupFlags.start || autoStart)
			{
				motorPwmOut(0, 0);
				countdown = 5000;							  // カウントダウンスタート
				ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
				ssd1306_SetCursor(56, 28);
				ssd1306_printf(Font_16x26, "5");

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
				ssd1306_SetCursor(56, 28);
				ssd1306_printf(Font_16x26, "4");
			}
			if (countdown == 3000)
			{
				// ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
				ssd1306_SetCursor(56, 28);
				ssd1306_printf(Font_16x26, "3");
			}
			if (countdown == 2000)
			{
				ssd1306_SetCursor(56, 28);
				ssd1306_printf(Font_16x26, "2");
			}
			if (countdown == 1000)
			{
				ssd1306_SetCursor(56, 28);
				ssd1306_printf(Font_16x26, "1");
				calibratIMU = true;		// IMUキャリブレーションを開始
				calibrateMotorCurrent = true; // 電流センサキャリブレーションを開始
			}
		}

		// IMUのキャリブレーションが終了したら走行開始
		if (!calibratIMU && !calibrateMotorCurrent && countdown == 0)
		{
			powerLineSensors(1);   // ラインセンサ点灯

			// SDカードに変数保存
			// PIDゲインを記録
			if (autoStart == 0)
			{
				writePIDparameters(&lineTraceCtrl);
				writePIDparameters(&lineTraceOmegaFBCtrl);
				writePIDparameters(&veloCtrl);
				writeSpeedFeedForwardGain(speedFeedForwardGain);	// 速度フィードフォワード係数も保存
				writePIDparameters(&yawRateCtrl);
				writePIDparameters(&yawCtrl);
				writePIDparameters(&distCtrl);
			}

			if (autoStart <= 1)
			{
				writeTgtspeeds(); // 目標速度を記録
			}

			if (optimalTrace == BOOST_NONE)
			{
				// lineTraceOmegaFBCtrl.kp = 12;
				// lineTraceOmegaFBCtrl.ki = 0;
				// lineTraceOmegaFBCtrl.kd = 0;

				// veloCtrl.kp = 8;
				// veloCtrl.ki = 0;
				// veloCtrl.kd = 0;
				// speedFeedForwardGain = 150;
			}

			if (initMSD)
			{
				initLog(); // ログ一時ファイル作成
			}

			// 変数初期化
			encRightMarker = 0;
			veloCtrl.Int = 0.0;
			yawRateCtrl.Int = 0.0;

			if (modeDSP)
			{
				ssd1306_StopDMA();	// 走行開始で画面更新停止
			}
			patternTrace = 11;
        }
		break;

	case 11:
		// スタートマーカー通過までの走行
		if(optimalTrace == BOOST_DISTANCE)
		{
			if(PPAD[0].boostSpeed/2 > 1.5F)
			{
				setTargetSpeed(1.5F); // 目標速度
			}
			else
			{
				setTargetSpeed(PPAD[0].boostSpeed/2); // 目標速度
			}
			
		}
		else
		{
			setTargetSpeed(tgtParam.search); // 目標速度
		}
		// ライントレース
		motorPwmOutSynth(lineTraceCtrl.pwm, veloCtrl.pwm, 0, 0);
		// motorPwmOut(veloCtrlL.pwm,veloCtrlR.pwm);

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
			optimalIndex = 0;
			clearIMUval(); // IMU値初期化
			optimalIndex = 0;
			yawCtrl.Int = 0.0;
			distCtrl.Int = 0.0;

			clearXYcie(); // 座標計算変数初期化

			if (initMSD)
			{
				modeLOG = true; // log start
			}
			
			patternTrace = 12;
			break;
		}
		break;

	case 12:
		// 目標速度設定
		if (optimalTrace == BOOST_NONE)
		{
			// 探索走行
			setTargetSpeed(tgtParam.search);
			// ライントレース
			// motorPwmOutSynth(lineTraceOmegaFBCtrl.pwm, veloCtrl.pwm, 0, 0);
			motorPwmOut(veloCtrlL.pwm,veloCtrlR.pwm);
		}
		else if (optimalTrace == BOOST_DISTANCE)
		{
			// 距離基準2次走行
			if (numPPADarry > 0)
			{
				if (optimalIndex >= numPPADarry)
				{
					optimalIndex = numPPADarry - 1;
				}
				const int32_t segmentPulse = encMM(CALCDISTANCE);		// セグメント距離をパルスに変換
				int32_t remainingPulse = encTotalOptimal - DistanceOptimal;	// セグメント内の残りパルス数
				if (segmentPulse > 0)
				{
					while (remainingPulse >= segmentPulse && (optimalIndex + 1) < numPPADarry)
					{
						DistanceOptimal += segmentPulse;	// 次の距離セグメントへ
						remainingPulse -= segmentPulse;		// セグメント内の残りパルス数を更新
						optimalIndex++;						// 速度計画インデックス更新
					}
				}
				if (remainingPulse < 0)
				{
					remainingPulse = 0;
					DistanceOptimal = encTotalOptimal;
				}
				// 速度計画から速度を取得
				boostSpeed = PPAD[optimalIndex].boostSpeed;
				if ((optimalIndex + 1) >= numPPADarry)
				{
					DistanceOptimal = encTotalOptimal - remainingPulse;
				}
			}
			else
			{
				boostSpeed = tgtParam.search;
			}
			// 目標速度に設定
			setTargetSpeed(boostSpeed);
			// モーター出力
			// motorPwmOutSynth(lineTraceOmegaFBCtrl.pwm, veloCtrl.pwm, 0, 0);
			motorPwmOut(veloCtrlL.pwm,veloCtrlR.pwm);
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
			if (numPPADarry > 0 && optimalIndex >= numPPADarry - 1)
			{
				goalTime = cntRun;
				enc1 = 0;
				patternTrace = 101;
			}
		}
		break;

	case 101:
		if (modeDSP && !ssd1306_IsDMARunning())
		{
			ssd1306_UpdateScreen_DMA();        // 停止していた画面更新を再開
		}
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
			patternTrace = 102;
		}
		break;

	case 102:
		setTargetSpeed(0);
		motorPwmOutSynth(0, 0, 0, 0);

		if (modeLOG)
		{
			ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
			ssd1306_SetCursor(0, 25);
			ssd1306_printf(Font_11x18, "log %d",fileNumbers[endFileIndex]+1);
			ssd1306_SetCursor(0, 45);
			ssd1306_printf(Font_11x18, "Writing");

			endLog(); // ログ保存終了

			ssd1306_SetCursor(0, 45);
			ssd1306_printf(Font_11x18, "Written");
		}

		if (autoStart > 0)
		{
			// 自動走行モードのときは再度走行準備へ
			autoStart++;
			if(autoStart == 2)
			{
				autoStartAnalize = fileNumbers[endFileIndex]+1; // 1次走行のログ番号を保存
			}
			else if (autoStart >= 3)
			{
				if(autoStart == 3)
				{
					autoStartAnalize++; // 2次走行のログを解析するためにログ番号をインクリメント
				}
				// 3走目以降は速度を上げる
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
				// 5走終了
				ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
				ssd1306_SetCursor(0, 25);
				ssd1306_printf(Font_11x18, "Auto run");
				ssd1306_SetCursor(0, 45);
				ssd1306_printf(Font_11x18, "Finish!");

				patternTrace = 103;
				break;
			}
			else
			{
				// 走行準備へ
				powerLineSensors(0);
				powerMarkerSensors(0);
				// 初期化
				SGmarker = 0;	// スタートマーカー通過フラグクリア
				initMarkerSensor();	// マーカーセンサ初期化
				resetEmcStop();	// 緊急停止フラグクリア
				clearMarkerProcessState();	// マーカーセンサ処理状態クリア
				lineTraceOmegaFBCtrl.Int = 0;
				encTotalN = 0;	// エンコーダ総パルス数クリア
				stateCrossLine = false;		// クロスライン検出状態クリア

				patternTrace = 0;
				break;
			}
		}
		else
		{
			// 手動走行のときは停止
			if (modeDSP)
			{
				ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
				if (logOverflow || markerOverflow)
				{
					// バッファ上限エラー表示
					ssd1306_SetCursor(0, 25);						// 1行目の表示位置
					ssd1306_printf(Font_11x18, "buff overflow");	// エラーメッセージ1行目
					ssd1306_SetCursor(0, 45);						// 2行目の表示位置
					ssd1306_printf(Font_11x18, "error");			// エラーメッセージ2行目
				}
				else
				{
					ssd1306_SetCursor(0, 25);
					ssd1306_printf(Font_11x18, "Time");
					ssd1306_SetCursor(0, 45);
					ssd1306_printf(Font_11x18, "%6.3f[s]", (float)goalTime / 1000);
				}
			}
		}

		patternTrace = 103;
		break;

	case 103:
		motorPwmOutSynth(0, 0, 0, 0);
		powerLineSensors(0);
		powerMarkerSensors(0);
		
		// リセット待ち
		if(swValTact == SW_PUSH)
		{
			while(swValTact == SW_PUSH);	// スイッチが離されるまで待つ
			softreset = true;				// ソフトウェアリセット実行
			initSystem();			// 初期化処理実行
			initMarkerSensor();	// マーカーセンサ初期化
			resetEmcStop();		// 緊急停止フラグクリア
			setupFlags.start= 0;	// スタートフラグクリア
			autoStart = 0;			// 自動走行フラグクリア
			optimalTrace = 0;		// 2次走行モードクリア
			pattern.display = HEX_START; // 16進表示モードクリア
			pattern.beforeHex = 255;	// 16進表示モードクリア
			encClick = 0;				// ホイールクリッククリア
			encTotalN = 0;				// エンコーダ総パルス数クリア
			lineTraceOmegaFBCtrl.Int = 0;
			stateCrossLine = false;		// クロスライン検出状態クリア
			clearMarkerProcessState();	// マーカーセンサ処理状態クリア
			softreset = false;			// ソフトウェアリセット状態を解除
			patternTrace = 0;
		}
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
	// 機体停止処理
	setTargetSpeed(0);
	while (encCurrentN > 5)
	{
		motorPwmOutSynth(0, veloCtrl.pwm, 0, 0);
	}
	
	if (!ssd1306_IsDMARunning())
	{
		ssd1306_UpdateScreen_DMA();        // 停止していた画面更新を再開
	}

	motorPwmOut(0, 0);

	if (initMSD)
	{
		if (modeDSP)
		{
			ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
			ssd1306_SetCursor(0, 25);
			ssd1306_printf(Font_11x18, "log");
			ssd1306_SetCursor(0, 45);
			ssd1306_printf(Font_11x18, "Writing");
		}

		if(modeLOG)
		{
			endLog(); // ログ保存終了
		}
		else
		{
			endTempFile(); // 一時ファイルをクローズ
		}
		
	}

	if (modeDSP)
	{
		ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め

		ssd1306_SetCursor(0, 25);
		ssd1306_printf(Font_11x18, "EMS!! %d", autoStartAnalize);

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
		case STOP_LINESENSOR_BRIGHT:
			ssd1306_printf(Font_7x10, "LINESENSOR B");
			break;
		case STOP_LINESENSOR_UNBRIGHT:
			ssd1306_printf(Font_7x10, "LINESENSOR UB");
			break;
		case STOP_OVERSPEED:
			ssd1306_printf(Font_7x10, "OVERSPEED");
			break;
		}
	}

	patternTrace = 103;
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
// モジュール名 changeGain
// 処理概要     ROCに応じてゲインを切り替える
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void changeGain(void)
{
	static bool changeGain = false;
	static int16_t beforeGainP = 0, beforeGainD = 0;
	static bool softresetHandled = false;
	bool useStraightGain = false;
	bool useCrossLineGain = false;

	// ソフトウェアリセット時はゲイン変更状態を初期化する
	if (softreset && !softresetHandled)
	{
		if (changeGain)
		{
			lineTraceOmegaFBCtrl.kp = beforeGainP;
			lineTraceOmegaFBCtrl.kd = beforeGainD;
		}
		beforeGainP = 0;
		beforeGainD = 0;
		changeGain = false;
		softresetHandled = true;
	}
	else if (!softreset)
	{
		softresetHandled = false;
	}

	// 距離基準2次走行のみ、ROC閾値で直線/カーブに合わせてゲインを切り替える
	if (optimalTrace == BOOST_DISTANCE && numPPADarry > 0)
	{
		uint16_t rocIndex = optimalIndex;
		if (rocIndex >= (uint16_t)numPPADarry)
		{
			rocIndex = (uint16_t)(numPPADarry - 1);
		}
		if (PPAD[rocIndex].ROC > ROC_STRAIGHTTH)
		{
			useStraightGain = true;
		}
	}

	// クロスライン検出時のゲイン切り替え
	if (optimalTrace == BOOST_NONE && stateCrossLine)
	{
		useCrossLineGain = true;
	}

	if (useStraightGain || useCrossLineGain)
	{
		if (!changeGain)
		{
			beforeGainP = lineTraceOmegaFBCtrl.kp;
			beforeGainD = lineTraceOmegaFBCtrl.kd;
			lineTraceOmegaFBCtrl.kp = 2;
			lineTraceOmegaFBCtrl.kd = 0;
			changeGain = true;
		}
	}
	else
	{
		if (changeGain)
		{
			lineTraceOmegaFBCtrl.kp = beforeGainP;
			lineTraceOmegaFBCtrl.kd = beforeGainD;
			beforeGainP = 0;
			beforeGainD = 0;
			changeGain = false;
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
///////////////////////////////////////////////////////////////////////////
// モジュール名 lpf1
// 処理概要     1次LPF
// 引数         current: 現在値, input: 入力, coef: LPF係数
// 戻り値       更新後の値
///////////////////////////////////////////////////////////////////////////
static float lpf1(float current, float input, float coef)
{
	return current + coef * (input - current);
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 updateTurningHysteresis
// 処理概要     旋回状態のヒステリシス更新
// 引数         st: スリップ検出状態, absWz: yaw角速度絶対値
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
static void updateTurningHysteresis(SlipDetState *st, float absWz)
{
	// ヒステリシスによる旋回状態更新
	if (!st->turningState) {
		if (absWz > SLIP_GYRO_ON_RADS) {
			st->turningState = true;
		}
	} else {
		if (absWz < SLIP_GYRO_OFF_RADS) {
			st->turningState = false;
		}
	}
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 updateHysFlag
// 処理概要     ヒステリシス付きフラグ更新
// 引数         flag: 対象フラグ, highCount/lowCount: 連続カウンタ
//              value: 判定値, highTh/lowTh: 閾値
//              highReq/lowReq: 連続回数
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
static void updateHysFlag(bool *flag, uint16_t *highCount, uint16_t *lowCount,
	float value, float highTh, float lowTh,
	uint16_t highReq, uint16_t lowReq)
{
	// ヒステリシス判定（縦スリップ等の汎用）
	if (!*flag) {
		if (value > highTh) {
			if (++(*highCount) >= highReq) {
				*flag = true;
				*highCount = 0;
			}
		} else {
			*highCount = 0;
		}
	} else {
		if (value < lowTh) {
			if (++(*lowCount) >= lowReq) {
				*flag = false;
				*lowCount = 0;
			}
		} else {
			*lowCount = 0;
		}
	}
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 slipResetAll
// 処理概要     スリップ検出の内部状態をリセット
// 引数         st: スリップ検出状態
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
static void slipResetAll(SlipDetState *st)
{
	// 主要指標/カウンタを初期化
	slipDeltaImu = 0.0f;
	slipDeltaEnc = 0.0f;
	slipIndicatorRaw = 0.0f;
	slipIndicatorFiltered = 0.0f;
	slipHighCount = 0;
	slipLowCount = 0;
	slipHighCountLat = 0;
	slipLowCountLat = 0;
	slipFlag = false;
	slipFlagLat = false;
	st->slipPrimed = false;
	st->turningState = false;
#if SLIP_CUR_ENABLE
	// PWM/電流LPFのリセット
	st->pwmSumF = 0.0f;
	st->iSumF = 0.0f;
#endif

	// 微分・フィルタ系リセット
	st->axBias = 0.0f;
	st->ayBias = 0.0f;
	st->imuAxF = 0.0f;
	st->imuAyF = 0.0f;
	st->encAxF = 0.0f;
	st->encAyF = 0.0f;

	// 既存バッファをクリア
	for (uint16_t i = 0; i < SLIP_WINDOW_SAMPLES; i++) {
		slipEncSpeedHist[i] = 0.0f;
	}
	slipBufIndex = 0;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 slipPrimeSpeedHist
// 処理概要     リングバッファと各種LPFを初期化
// 引数         st: スリップ検出状態, encSpeed: 現在速度
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
static void slipPrimeSpeedHist(SlipDetState *st, float encSpeed)
{
	// バッファ初期化（0ではなく現在値で埋める）
	for (uint16_t i = 0; i < SLIP_WINDOW_SAMPLES; i++) {
		slipEncSpeedHist[i] = encSpeed;
	}
	slipBufIndex = 0;

	// フィルタ初期化
	st->imuAxF = 0.0f;
	st->imuAyF = 0.0f;
	st->encAxF = 0.0f;
	st->encAyF = 0.0f;

	// 指標初期化
	slipIndicatorRaw = 0.0f;
	slipIndicatorFiltered = 0.0f;
	slipHighCount = 0;
	slipLowCount = 0;
	slipHighCountLat = 0;
	slipLowCountLat = 0;
	slipDeltaImu = 0.0f;
	slipDeltaEnc = 0.0f;
	slipFlag = false;
	slipFlagLat = false;

	float ax0 = BMI088val.accele.y * 9.81f;
	float ay0 = BMI088val.accele.x * 9.81f;
	st->axBias = ax0;
	st->ayBias = ay0;
	float wz0 = BMI088val.gyro.z * DEG2RAD;
	st->turningState = (fabsf(wz0) > SLIP_GYRO_ON_RADS);

#if SLIP_CUR_ENABLE
	// PWM/電流LPF初期化（開始直後のゲート鈍化を防止）
	float pwmSum = fabsf((float)motorpwmL) + fabsf((float)motorpwmR);
	float iSum = fabsf(motorCurrentL) + fabsf(motorCurrentR);
	st->pwmSumF = pwmSum;
	st->iSumF = iSum;
#endif

	st->slipPrimed = true;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 updateSlipDetection
// 処理概要     時間窓でΔv_encとΔv_imuを比率化し、ヒステリシス付きでスリップ判定する
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void updateSlipDetection(void)
{
	// IMUが未初期化・キャリブ中はスリップ検出を行わない
	if (!initIMU || calibratIMU) {
		return;
	}

	// ---- 状態管理 ----
	SlipDetState *st = &slipDetState;

	// patternTrace=11(走行開始)でもスリップ検出を行う
	bool running = (patternTrace >= 11 && patternTrace < 100);

	//==========================================================
	// 走行外：遷移時だけリセット
	//==========================================================
	if (!running) {
		if (st->prevRunning) {
			// 走行終了時の全リセット
			slipResetAll(st);
		}
		st->prevRunning = false;
		st->prevMoving  = false;
		return;
	}
	st->prevRunning = true;

	// ---- エンコーダ速度 ----
	float encSpeed = (float)encCurrentN / PALSE_MILLIMETER;

	//==========================================================
	// 超低速：遷移時だけリセット（微分スパイク防止）
	//==========================================================
	if (fabsf(encSpeed) < SLIP_SPEED_SKIP_MPS) {

		if (st->prevMoving) {
			// 低速遷移時の全リセット
			slipResetAll(st);
		}

		st->prevMoving = false;

		float ax = BMI088val.accele.y * 9.81f;
		float ay = BMI088val.accele.x * 9.81f;
		float wz = BMI088val.gyro.z * DEG2RAD;
		float absWz = fabsf(wz);
		updateTurningHysteresis(st, absWz);
		// 旋回していない時だけバイアス更新(横Gの影響を受けないようにするため)
		if (!st->turningState) {
			st->axBias = lpf1(st->axBias, ax, SLIP_ACC_BIAS_COEF);
			st->ayBias = lpf1(st->ayBias, ay, SLIP_ACC_BIAS_COEF);
		}

		// ログ残差は停止中に残らないようゼロ化
		slipDeltaEnc = 0.0f;
		slipDeltaImu = 0.0f;
		// フィルタ値だけはゼロへ軽く収束
		slipIndicatorRaw = lpf1(slipIndicatorRaw, 0.0f, SLIP_LPF_COEF);
		slipIndicatorFiltered = lpf1(slipIndicatorFiltered, 0.0f, SLIP_LPF_COEF);
		return;
	}
	st->prevMoving = true;

	// リングバッファ初期化スパイク対策
	if (!st->slipPrimed) {
		// 開始直後のバッファ初期化
		slipPrimeSpeedHist(st, encSpeed);
		return; // 初回は判定しない
	}

	// ---- IMU加速度（ボディ座標）----
	float ax = BMI088val.accele.y * 9.81f;  // 前後
	float ay = BMI088val.accele.x * 9.81f;  // 左右

	// ---- yaw角速度（rad/s）----
	float wz = BMI088val.gyro.z * DEG2RAD;

	float dt = SLIP_SAMPLE_PERIOD_S;

	//==========================================================
	// ENC加速度（微分）
	//==========================================================
	// encSpeed をリングに保存して、SLIP_WINDOW_SAMPLESms前の値との差で dv/dt
	float encSpeedOld = slipEncSpeedHist[slipBufIndex];
	slipEncSpeedHist[slipBufIndex] = encSpeed;

	slipBufIndex++;
	if (slipBufIndex >= SLIP_WINDOW_SAMPLES) slipBufIndex = 0;

	float encAx = (encSpeed - encSpeedOld) / (SLIP_WINDOW_SAMPLES * dt);

	// カーブ時の横加速度（理想）：ay ≒ wz * v
	float encAy = wz * encSpeed;

	//==========================================================
	// IMUのDC成分（バイアス/傾き由来の重力漏れ等）を除去
	//==========================================================
	// 旋回していない時だけバイアス更新(横Gの影響を受けないようにするため)
	updateTurningHysteresis(st, fabsf(wz));
	if (!st->turningState) {
		st->axBias = lpf1(st->axBias, ax, SLIP_ACC_BIAS_COEF);
		st->ayBias = lpf1(st->ayBias, ay, SLIP_ACC_BIAS_COEF);
	}
	float imuAx = ax - st->axBias;
	float imuAy = ay - st->ayBias;

	//==========================================================
	// ノイズ低減LPF
	//==========================================================
	st->imuAxF = lpf1(st->imuAxF, imuAx, SLIP_ACC_LPF_COEF);
	st->imuAyF = lpf1(st->imuAyF, imuAy, SLIP_ACC_LPF_COEF);
	st->encAxF = lpf1(st->encAxF, encAx, SLIP_ACC_LPF_COEF);
	st->encAyF = lpf1(st->encAyF, encAy, SLIP_ACC_LPF_COEF);

	//==========================================================
	// 縦・横残差（判定/ログ用）
	//==========================================================
	float dx = st->encAxF - st->imuAxF;	// 縦方向(前後)残差
	float dy = st->encAyF - st->imuAyF;	// 横方向(左右)残差
	float absDx = fabsf(dx);
	float absDy = fabsf(dy);

	//==========================================================
	// PWM/電流の合計（惰性/低トルク判定用）
	//==========================================================
#if SLIP_CUR_ENABLE
	// PWM/電流のLPF更新（SLIP_CUR_ENABLE=1のみ）
	float pwmSum = fabsf((float)motorpwmL) + fabsf((float)motorpwmR);
	float iSum = fabsf(motorCurrentL) + fabsf(motorCurrentR);
	st->pwmSumF = lpf1(st->pwmSumF, pwmSum, SLIP_PWM_LPF_COEF);
	st->iSumF = lpf1(st->iSumF, iSum, SLIP_CUR_LPF_COEF);
#endif

#if SLIP_CUR_ENABLE
	//==========================================================
	// モータ電流によるノイズ補正スケール
	//==========================================================
	float curScale = 1.0f;
	if (!calibrateMotorCurrent && (st->iSumF > SLIP_CUR_MIN_A)) {
		float dI = st->iSumF - SLIP_CUR_BASE_A;
		if (dI > 0.0f) {
			curScale = 1.0f + SLIP_CUR_K * dI;
			if (curScale > SLIP_CUR_MAX_SCALE) {
				curScale = SLIP_CUR_MAX_SCALE;
			}
		}
	}
#else
	// SLIP_CUR_ENABLE=0では電流スケールを固定
	float curScale = 1.0f;
#endif

	//==========================================================
	// 低加速度ゲート（誤検出抑制）
	//==========================================================
	float accMag = fmaxf(
		fmaxf(fabsf(st->encAxF), fabsf(st->encAyF)),
		fmaxf(fabsf(st->imuAxF), fabsf(st->imuAyF))
	);

	// 横スリップ判定の有効化条件（直進ノイズ抑制）
	bool latEnabled = st->turningState && (fabsf(st->encAyF) > SLIP_LAT_ENCAY_MIN);
	// 横判定のカウントを許可する条件（PWMが小さい区間は止める）
#if SLIP_CUR_ENABLE
	bool latCountEnabled = latEnabled && (st->pwmSumF > SLIP_PWM_LAT_COUNT_MIN);
	// 惰性/低トルク時は横判定を強制クリアする
	bool latCoastHardClear = (!calibrateMotorCurrent)
			&& (st->pwmSumF < SLIP_PWM_COAST_MAX)
			&& (st->iSumF < SLIP_ISUM_COAST_MAX);
#else
	// SLIP_CUR_ENABLE=0ではPWM/電流ゲートを使わない
	bool latCountEnabled = latEnabled;
	bool latCoastHardClear = false;
#endif

	// 低加速度では見ない（縦/横の誤検出抑制）
	if (accMag < 0.8f) {
		dx = 0.0f;
		dy = 0.0f;
		absDx = 0.0f;
		absDy = 0.0f;
		slipHighCount = 0;
		// 解除カウントは維持して張り付き防止
		slipHighCountLat = 0;
		// フィルタ値は0へ収束させる（縦/横）
		slipIndicatorRaw = lpf1(slipIndicatorRaw, 0.0f, SLIP_LPF_COEF);
		if (latCoastHardClear) {
			// 惰性時は横指標を強制的にクリアして誤検知を抑制
			slipIndicatorFiltered = lpf1(slipIndicatorFiltered, 0.0f, SLIP_LAT_CLEAR_COEF);
			slipHighCountLat = 0;
			slipLowCountLat = 0;
			slipFlagLat = false;
		} else {
			slipIndicatorFiltered = lpf1(slipIndicatorFiltered, 0.0f, SLIP_LPF_COEF);
		}
	} else {
		// 縦スリップ指標（absDxをLPF）
		slipIndicatorRaw = lpf1(slipIndicatorRaw, absDx, SLIP_LPF_COEF);
		// 横スリップ指標（absDyをLPF、slipRatio相当）
		if (latCoastHardClear) {
			// 惰性時は横指標を強制的にクリアして誤検知を抑制
			slipIndicatorFiltered = lpf1(slipIndicatorFiltered, 0.0f, SLIP_LAT_CLEAR_COEF);
			slipHighCountLat = 0;
			slipLowCountLat = 0;
			slipFlagLat = false;
		} else if (latCountEnabled) {
			slipIndicatorFiltered = lpf1(slipIndicatorFiltered, absDy, SLIP_LPF_COEF);
		} else if (latEnabled) {
			// 旋回中だがPWMが小さい区間は0へ収束させる
			slipIndicatorFiltered = lpf1(slipIndicatorFiltered, 0.0f, SLIP_LPF_COEF);
		} else {
			slipIndicatorFiltered = lpf1(slipIndicatorFiltered, 0.0f, SLIP_LPF_COEF);
		}
	}

	// ログ用（低加速度時は0に収束）
	slipDeltaEnc = dx;	// 縦方向残差
	slipDeltaImu = dy;	// 横方向残差

	//==============================
	// 旋回が強い時だけ閾値を上げる
	//==============================
	slipThresholdHigh = SLIP_MISMATCH_HIGH;
	slipThresholdLow= SLIP_MISMATCH_LOW;

	if (st->turningState) {
		slipThresholdHigh *= SLIP_MISMATCH_HIGH_TURN;
		slipThresholdLow  *= SLIP_MISMATCH_LOW_TURN;
	}
	// 電流が大きい（=振動/ノイズが増えやすい）ときは閾値を少し上げて誤検知を抑える
	slipThresholdHigh *= curScale;
	slipThresholdLow  *= curScale;

	// ---- 縦スリップ ヒステリシス判定（absDxのみ）----
	updateHysFlag(&slipFlag, &slipHighCount, &slipLowCount,
		slipIndicatorRaw, slipThresholdHigh, slipThresholdLow,
		SLIP_HIGH_COUNT_REQ, SLIP_LOW_COUNT_REQ);

	// ---- 横スリップ判定（旋回中のみ）----
	float slipLatHigh = SLIP_LAT_HIGH * curScale;
	float slipLatLow  = SLIP_LAT_LOW  * curScale;
	if (!slipFlagLat) {
		if (latCountEnabled && slipIndicatorFiltered > slipLatHigh) {
			if (++slipHighCountLat >= SLIP_HIGH_COUNT_REQ) {
				slipFlagLat = true;
				slipHighCountLat = 0;
			}
		} else {
			slipHighCountLat = 0;
		}
		slipLowCountLat = 0;
	} else {
#if SLIP_CUR_ENABLE
		if (!latEnabled || st->pwmSumF <= SLIP_PWM_LAT_COUNT_MIN || slipIndicatorFiltered < slipLatLow) {
#else
		if (!latEnabled || slipIndicatorFiltered < slipLatLow) {
#endif
			if (++slipLowCountLat >= SLIP_LOW_COUNT_REQ) {
				slipFlagLat = false;
				slipLowCountLat = 0;
			}
		} else {
			slipLowCountLat = 0;
		}
	}
}
///////////////////////////////////////////////////////////////////////////
// モジュール名	getSlipDeltaImu
// 				getSlipDeltaEnc
// 				getSlipIndicatorFiltered
// 				getSlipFlag
// 				getSlipFlagLat
// 				getSlipthresholdHigh
// 				getSlipthresholdLow
// 処理概要     割り込み外からスリップ検出結果を参照するためのアクセサ
///////////////////////////////////////////////////////////////////////////
float getSlipDeltaImu(void)
{
	return slipDeltaImu;
}
float getSlipDeltaEnc(void)
{
	return slipDeltaEnc;
}
float getSlipIndicatorFiltered(void)
{
	return slipIndicatorFiltered;
}
bool getSlipFlag(void)
{
	return slipFlag;
}
bool getSlipFlagLat(void)
{
	return slipFlagLat;
}
float getSlipthresholdHigh(void)
{
	return slipThresholdHigh;
}
float getSlipthresholdLow(void)
{
	return slipThresholdLow;
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
	char format[100] = "", fileName[30] = PATH_SETTING;
	int16_t i;

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

		f_printf(&fil, format, (int32_t)(round(tgtParam.search * 100)),
										(int32_t)(round(tgtParam.stop * 100)),
										(int32_t)(round(tgtParam.bstStraight * 100)),
										(int32_t)(round(tgtParam.bst1500 * 100)),
										(int32_t)(round(tgtParam.bst1300 * 100)),
										(int32_t)(round(tgtParam.bst1000 * 100)),
										(int32_t)(round(tgtParam.bst800 * 100)),
										(int32_t)(round(tgtParam.bst700 * 100)),
										(int32_t)(round(tgtParam.bst600 * 100)),		
										(int32_t)(round(tgtParam.bst500 * 100)),
										(int32_t)(round(tgtParam.bst400 * 100)),
										(int32_t)(round(tgtParam.bst300 * 100)),
										(int32_t)(round(tgtParam.bst200 * 100)),
										(int32_t)(round(tgtParam.bst100 * 100)),
										(int32_t)(round(tgtParam.acceleF * 100)),
										(int32_t)(round(tgtParam.acceleD * 100)),
										(int32_t)(round(tgtParam.shortCut * 100)));
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
	char fileName[30] = PATH_SETTING;
	int16_t param[20];
	TCHAR paramStr[100];
	int16_t i;

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
		i=0;
		tgtParam.search = (float)param[i++] / 100;
		tgtParam.stop = (float)param[i++] / 100;
		tgtParam.bstStraight = (float)param[i++] / 100;
		tgtParam.bst1500 = (float)param[i++] / 100;
		tgtParam.bst1300 = (float)param[i++] / 100;
		tgtParam.bst1000 = (float)param[i++] / 100;
		tgtParam.bst800 = (float)param[i++] / 100;
		tgtParam.bst700 = (float)param[i++] / 100;
		tgtParam.bst600 = (float)param[i++] / 100;
		tgtParam.bst500 = (float)param[i++] / 100;
		tgtParam.bst400 = (float)param[i++] / 100;
		tgtParam.bst300 = (float)param[i++] / 100;
		tgtParam.bst200 = (float)param[i++] / 100;
		tgtParam.bst100 = (float)param[i++] / 100;
		tgtParam.acceleF = (float)param[i++] / 100;
		tgtParam.acceleD = (float)param[i++] / 100;
		tgtParam.shortCut = (float)param[i++] / 100;
	}

	f_close(&fil);
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 checkCrossLine
// 処理概要     ラインセンサのクロスライン検出
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void checkCrossLine(void)
{
	static int32_t encCrossLine = 0;

	if (lSensorMax[0] > lSensorMin[0])
	{
		// 中央2センサが閾値以上かつ、外側1センサが閾値以上でクロスライン検出
		if((lSensorCari[4] > TRACE_CROSSLINE_TH && lSensorCari[5] > TRACE_CROSSLINE_TH)
		&& (lSensorCari[3] > TRACE_CROSSLINE_TH || lSensorCari[6] > TRACE_CROSSLINE_TH))
		{
			// 両センサが白を検出
			if (!stateCrossLine)
			{
				stateCrossLine = true;
				encCrossLine = encTotalN;	// クロスライン通過時のエンコーダ値を保存
			}
		}
		
		if (stateCrossLine)
		{
			if((encTotalN - encCrossLine) > encMM(TRACE_CROSSLINE_DISTANCE))
			{
				// クロスラインから離れたらゲインを元に戻す
				stateCrossLine = false;
			}
		}
	}
}
