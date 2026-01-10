//====================================//
// インクルード
//====================================//
#include "control.h"
#include "BMI088.h"
#include "PIDcontrol.h"
#include "BMI088.h"
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

	// ---- 状態遷移検出用 ----
	static bool prevRunning = false;
	static bool prevMoving  = false;

	// ---- IMUバイアス推定（DC成分除去）----
	static float axBias = 0.0f;
	static float ayBias = 0.0f;

	// ---- 信号LPF（ノイズ低減）----
	static float imuAxF = 0.0f, imuAyF = 0.0f;
	static float encAxF = 0.0f, encAyF = 0.0f;
#if SLIP_CUR_ENABLE
	static float motorCurMagF = 0.0f; // |IL|+|IR| のLPF
#endif
	// ---- PWM/電流の簡易LPF（惰性判定用）----
	static float pwmSumF = 0.0f;
	static float iSumF = 0.0f;

	// ---- 旋回状態検出 ----
	static bool turningState = false;
	static bool slipPrimed = false;

	// patternTrace=11(走行開始)でもスリップ検出を行う
	bool running = (patternTrace >= 12 && patternTrace < 100);

	//==========================================================
	// 走行外：遷移時だけリセット
	//==========================================================
	if (!running) {
		if (prevRunning) {
			slipDeltaImu = 0.0f;
			slipDeltaEnc = 0.0f;
			slipIndicatorRaw = 0.0f;
			slipIndicatorFiltered = 0.0f;
			slipHighCount = 0;
			slipLowCount = 0;
			slipHighCountLat = 0;
			slipLowCountLat = 0;
			slipPrimed = false;
			turningState = false;
			slipFlag = false;
			slipFlagLat = false;
			// ---- 電流LPFリセット ----
#if SLIP_CUR_ENABLE
			motorCurMagF = 0.0f;
#endif
			// ---- PWM/電流LPFリセット ----
			pwmSumF = 0.0f;
			iSumF = 0.0f;

			// 微分・フィルタ系リセット
			axBias = ayBias = 0.0f;
			imuAxF = imuAyF = 0.0f;
			encAxF = encAyF = 0.0f;

			// 既存バッファがあるなら一応クリア（使わなくてもOK）
			for (uint16_t i = 0; i < SLIP_WINDOW_SAMPLES; i++) {
				slipEncSpeedHist[i] = 0.0f;
			}
			slipBufIndex = 0;
		}
		prevRunning = false;
		prevMoving  = false;
		return;
	}
	prevRunning = true;

	// ---- エンコーダ速度 ----
	float encSpeed = (float)encCurrentN / PALSE_MILLIMETER;

	//==========================================================
	// 超低速：遷移時だけリセット（微分スパイク防止）
	//==========================================================
	if (fabsf(encSpeed) < SLIP_SPEED_SKIP_MPS) {

		if (prevMoving) {
			axBias = ayBias = 0.0f;
			imuAxF = imuAyF = 0.0f;
			encAxF = encAyF = 0.0f;

			slipDeltaImu = 0.0f;
			slipDeltaEnc = 0.0f;
			slipIndicatorRaw = 0.0f;
			slipIndicatorFiltered = 0.0f;
			slipHighCount = 0;
			slipLowCount = 0;
			slipHighCountLat = 0;
			slipLowCountLat = 0;
			slipPrimed = false;
			turningState = false;
			slipFlag = false;
			slipFlagLat = false;
			// ---- 電流LPFリセット ----
#if SLIP_CUR_ENABLE
			motorCurMagF = 0.0f;
#endif
			// ---- PWM/電流LPFリセット ----
			pwmSumF = 0.0f;
			iSumF = 0.0f;

			for (uint16_t i = 0; i < SLIP_WINDOW_SAMPLES; i++) {
				slipEncSpeedHist[i] = 0.0f;
			}
			slipBufIndex = 0;
		}

		prevMoving = false;

		float ax = BMI088val.accele.y * 9.81f;
		float ay = BMI088val.accele.x * 9.81f;
		float wz = BMI088val.gyro.z * DEG2RAD;
		float absWz = fabsf(wz);
		if (!turningState) {
			if (absWz > SLIP_GYRO_ON_RADS) turningState = true;
		} else {
			if (absWz < SLIP_GYRO_OFF_RADS) turningState = false;
		}
		// 旋回していない時だけバイアス更新(横Gの影響を受けないようにするため)
		if (!turningState) {
			axBias += SLIP_ACC_BIAS_COEF * (ax - axBias);
			ayBias += SLIP_ACC_BIAS_COEF * (ay - ayBias);
		}

		// ログ残差は停止中に残らないようゼロ化
		slipDeltaEnc = 0.0f;
		slipDeltaImu = 0.0f;
		// フィルタ値だけはゼロへ軽く収束
		slipIndicatorRaw += SLIP_LPF_COEF * (0.0f - slipIndicatorRaw);
		slipIndicatorFiltered += SLIP_LPF_COEF * (0.0f - slipIndicatorFiltered);
		return;
	}
	prevMoving = true;

	// リングバッファ初期化スパイク対策
	if (!slipPrimed) {
		for (uint16_t i=0; i<SLIP_WINDOW_SAMPLES; i++) {
			slipEncSpeedHist[i] = encSpeed;   // 0じゃなく現在値で埋める
		}
		slipBufIndex = 0;

		// フィルタも初期化（必要なら）
		imuAxF = imuAyF = 0.0f;
		encAxF = encAyF = 0.0f;

		// 指標もリセット
		slipIndicatorRaw = 0.0f;
		slipIndicatorFiltered = 0.0f;
		slipHighCount = slipLowCount = 0;
		slipHighCountLat = slipLowCountLat = 0;
		slipDeltaImu = 0.0f;
		slipDeltaEnc = 0.0f;
		slipFlag = false;
		slipFlagLat = false;

		float ax0 = BMI088val.accele.y * 9.81f;
		float ay0 = BMI088val.accele.x * 9.81f;
		axBias = ax0;
		ayBias = ay0;
		float wz0 = BMI088val.gyro.z * DEG2RAD;
		turningState = (fabsf(wz0) > SLIP_GYRO_ON_RADS);

		// ---- PWM/電流LPF初期化 ----
		pwmSumF = 0.0f;
		iSumF = 0.0f;

		slipPrimed = true;
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
	if (!turningState) {
		if (fabsf(wz) > SLIP_GYRO_ON_RADS) turningState = true;
	}
	else
	{
		if (fabsf(wz) < SLIP_GYRO_OFF_RADS) turningState = false;
	}
	if (!turningState) {
		axBias += SLIP_ACC_BIAS_COEF * (ax - axBias);
		ayBias += SLIP_ACC_BIAS_COEF * (ay - ayBias);
	}
	float imuAx = ax - axBias;
	float imuAy = ay - ayBias;

	//==========================================================
	// ノイズ低減LPF
	//==========================================================
	imuAxF += SLIP_ACC_LPF_COEF * (imuAx - imuAxF);
	imuAyF += SLIP_ACC_LPF_COEF * (imuAy - imuAyF);
	encAxF += SLIP_ACC_LPF_COEF * (encAx - encAxF);
	encAyF += SLIP_ACC_LPF_COEF * (encAy - encAyF);

	//==========================================================
	// 縦・横残差（判定/ログ用）
	//==========================================================
	float dx = encAxF - imuAxF;	// 縦方向(前後)残差
	float dy = encAyF - imuAyF;	// 横方向(左右)残差
	float absDx = fabsf(dx);
	float absDy = fabsf(dy);

	//==========================================================
	// PWM/電流の合計（惰性/低トルク判定用）
	//==========================================================
	float pwmSum = fabsf((float)motorpwmL) + fabsf((float)motorpwmR);
	float iSum = fabsf(motorCurrentL) + fabsf(motorCurrentR);
	pwmSumF += SLIP_CUR_LPF_COEF * (pwmSum - pwmSumF);
	iSumF += SLIP_CUR_LPF_COEF * (iSum - iSumF);

#if SLIP_CUR_ENABLE
	//==========================================================
    // モータ電流によるノイズ補正スケール
	//==========================================================
	float motorCurMag = fabsf(motorCurrentL) + fabsf(motorCurrentR);
	motorCurMagF += SLIP_CUR_LPF_COEF * (motorCurMag - motorCurMagF);

    float curScale = 1.0f;
    if (!calibrateMotorCurrent && (motorCurMagF > SLIP_CUR_MIN_A)) {
        float dI = motorCurMagF - SLIP_CUR_BASE_A;
        if (dI > 0.0f) {
            curScale = 1.0f + SLIP_CUR_K * dI;
            if (curScale > SLIP_CUR_MAX_SCALE) curScale = SLIP_CUR_MAX_SCALE;
        }
    }
#else
    float curScale = 1.0f;
#endif

	//==========================================================
	// 低加速度ゲート（誤検出抑制）
	//==========================================================
	float accMag = fmaxf(
		fmaxf(fabsf(encAxF), fabsf(encAyF)),
		fmaxf(fabsf(imuAxF), fabsf(imuAyF))
	);

	// 横スリップ判定の有効化条件（直進ノイズ抑制）
	bool latEnabled = turningState && (fabsf(encAyF) > SLIP_LAT_ENCAY_MIN);
	// 横判定のカウントを許可する条件（PWMが小さい区間は止める）
	bool latCountEnabled = latEnabled && (pwmSumF > SLIP_PWM_LAT_COUNT_MIN);
	// 惰性/低トルク時は横判定を強制クリアする
	bool latCoastHardClear = (!calibrateMotorCurrent)
			&& (pwmSumF < SLIP_PWM_COAST_MAX)
			&& (iSumF < SLIP_ISUM_COAST_MAX);

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
		slipIndicatorRaw += SLIP_LPF_COEF * (0.0f - slipIndicatorRaw);
		if (latCoastHardClear) {
			// 惰性時は横指標を強制的にクリアして誤検知を抑制
			slipIndicatorFiltered += SLIP_LAT_CLEAR_COEF * (0.0f - slipIndicatorFiltered);
			slipHighCountLat = 0;
			slipLowCountLat = 0;
			slipFlagLat = false;
		} else {
			slipIndicatorFiltered += SLIP_LPF_COEF * (0.0f - slipIndicatorFiltered);
		}
	} else {
		// 縦スリップ指標（absDxをLPF）
		slipIndicatorRaw += SLIP_LPF_COEF * (absDx - slipIndicatorRaw);
		// 横スリップ指標（absDyをLPF、slipRatio相当）
		if (latCoastHardClear) {
			// 惰性時は横指標を強制的にクリアして誤検知を抑制
			slipIndicatorFiltered += SLIP_LAT_CLEAR_COEF * (0.0f - slipIndicatorFiltered);
			slipHighCountLat = 0;
			slipLowCountLat = 0;
			slipFlagLat = false;
		} else if (latCountEnabled) {
			slipIndicatorFiltered += SLIP_LPF_COEF * (absDy - slipIndicatorFiltered);
		} else if (latEnabled) {
			// 旋回中だがPWMが小さい区間は0へ収束させる
			slipIndicatorFiltered += SLIP_LPF_COEF * (0.0f - slipIndicatorFiltered);
		} else {
			slipIndicatorFiltered += SLIP_LPF_COEF * (0.0f - slipIndicatorFiltered);
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

	if (turningState) {
		slipThresholdHigh *= SLIP_MISMATCH_HIGH_TURN;
		slipThresholdLow  *= SLIP_MISMATCH_LOW_TURN;
	}
    // 電流が大きい（=振動/ノイズが増えやすい）ときは閾値を少し上げて誤検知を抑える
    slipThresholdHigh *= curScale;
    slipThresholdLow  *= curScale;

	// ---- 縦スリップ ヒステリシス判定（absDxのみ）----
	if (!slipFlag) {
		if (slipIndicatorRaw > slipThresholdHigh) {
			if (++slipHighCount >= SLIP_HIGH_COUNT_REQ) {
				slipFlag = true;
				slipHighCount = 0;
			}
		} else {
			slipHighCount = 0;
		}
	} else {
		if (slipIndicatorRaw < slipThresholdLow) {
			if (++slipLowCount >= SLIP_LOW_COUNT_REQ) {
				slipFlag = false;
				slipLowCount = 0;
			}
		} else {
			slipLowCount = 0;
		}
	}

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
		if (!latEnabled || pwmSumF <= SLIP_PWM_LAT_COUNT_MIN || slipIndicatorFiltered < slipLatLow) {
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
