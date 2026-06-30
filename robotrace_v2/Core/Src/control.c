//====================================//
// インクルード
//====================================//
#include "control.h"
#include "BMI088.h"
#include "PIDcontrol.h"
#include "encoder.h"
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
int16_t autoStartAnalyze = 0; 		// 自動走行で使用するログの解析番号

bool stateCrossLine = false;		// クロスライン検出状態
float rocrun = 2000;		// 曲率半径計算用変数

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
	PARAM_SHORTCUT,
	PARAM_DECEL_LEAD_MM};
// スリップ検出用の状態（1ms割り込みで軽量に処理するためここで管理）
static float slipEncSpeedHist[SLIP_WINDOW_SAMPLES];		// 時間窓の開始時点のエンコーダ由来速度[m/s]（リングバッファ）
static uint16_t slipBufIndex = 0;						// リングバッファの書き込み位置
static float slipDeltaImu = 0.0f;						// 窓内の横方向残差[m/s^2]
static float slipDeltaEnc = 0.0f;						// 窓内の縦方向残差[m/s^2]
static float slipIndicatorRaw = 0.0f;					// 縦スリップ指標の一次LPF後の値
static float slipIndicatorFiltered = 0.0f;				// 横スリップ指標の一次LPF後の値
static uint16_t slipHighCount = 0;						// スリップ立ち上がり判定用の連続カウンタ
static uint16_t slipLowCount = 0;						// スリップ解除判定用の連続カウンタ
static uint16_t slipHighCountLat = 0;					// 横スリップ立ち上がり判定用の連続カウンタ
static uint16_t slipLowCountLat = 0;					// 横スリップ解除判定用の連続カウンタ
static uint16_t slipISumOkCount = 0;					// Lat ON用の瞬時電流連続カウンタ
static bool slipFlag = false;							// 縦スリップ判定フラグ
static bool slipFlagLat = false;						// 横スリップ判定フラグ
static bool slipLongOnCountEnabledLog = false;			// 縦スリップONカウント許可条件
static uint16_t slipLongLowloadClearCount = 0;			// 低負荷継続時の縦スリップ保持解除カウンタ
static bool slipLatEnabledLog = false;					// 横滑り判定の有効条件
static bool slipLatOnCountEnabledLog = false;			// 横滑りONカウント許可条件
static float slipThresholdHighLog = 0.0f;				// 縦スリップON閾値[m/s^2]
static float slipThresholdLowLog = 0.0f;					// 縦スリップOFF閾値[m/s^2]
static float slipLatHighLog = 0.0f;						// 横スリップON閾値[m/s^2]
static float slipLatLowLog = 0.0f;						// 横スリップOFF閾値[m/s^2]
static float slipCurScaleLog = 1.0f;						// 電流由来の閾値スケール
static bool slipTurningStateLog = false;					// 旋回判定状態
static float slipAxBiasLog = 0.0f;						// 縦加速度バイアス[m/s^2]
static float slipAyBiasLog = 0.0f;						// 横加速度バイアス[m/s^2]
// スリップ距離補正用の状態
static float slipDistScaleRaw = 1.0f;					// 距離補正スケール（生）
static float slipDistScaleF = 1.0f;						// 距離補正スケール（LPF後）
// スリップ距離補正（パルス版）
static int32_t distEncRaw_p = 0;						// 生パルス積算
static int32_t distCorr_p = 0;							// 補正後パルス積算
static int32_t distSlipLoss_p = 0;						// 生 - 補正 の積算
static float distCorrFrac_p = 0.0f;						// 補正後パルスの小数残差
static int32_t encCurrentCorr_p = 0;					// 補正後の現在速度[pulse/1ms]
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
// スリップ距離補正（パルス版）
static void slipResetAll(SlipDetState *st);
static void slipDistReset(void);
static int32_t slipDistUpdateAndApply_p(float rawScale, int32_t dEncRaw_p);
static const char *getRunStartBlockReason(void);
static bool isRunStartAllowed(void);
static void showRunStartBlocked(const char *reason);
static bool blockRunStartIfNeeded(void);
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
		if(cntFiles > FILENUMBER_ALARM)
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
// モジュール名 getRunStartBlockReason
// 処理概要     走行開始を禁止する理由を取得する
// 引数         なし
// 戻り値       NULL:走行開始可能, 文字列:走行開始禁止理由
///////////////////////////////////////////////////////////////////////////
static const char *getRunStartBlockReason(void)
{
	if (!initIMU)
	{
		return "IMU failed";
	}
	if (!isLineSensorCalibrationValid())
	{
		return "LS calib";
	}
	return NULL;
}

///////////////////////////////////////////////////////////////////////////
// モジュール名 isRunStartAllowed
// 処理概要     現在の初期化状態で走行開始してよいか判定する
// 引数         なし
// 戻り値       true:走行開始可能 false:走行開始禁止
///////////////////////////////////////////////////////////////////////////
static bool isRunStartAllowed(void)
{
	return getRunStartBlockReason() == NULL;
}

///////////////////////////////////////////////////////////////////////////
// モジュール名 showRunStartBlocked
// 処理概要     走行開始禁止理由をディスプレイに表示する
// 引数         reason:走行開始禁止理由
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
static void showRunStartBlocked(const char *reason)
{
	if (!modeDSP)
	{
		return;
	}

	ssd1306_FillRectangle(0, 15, 127, 63, Black);
	ssd1306_SetCursor(15, 25);
	ssd1306_printf(Font_11x18, "Start NG");
	ssd1306_SetCursor(20, 50);
	ssd1306_printf(Font_6x8, "%s", reason);
	ssd1306_UpdateScreen();
	HAL_Delay(1000);
}

///////////////////////////////////////////////////////////////////////////
// モジュール名 blockRunStartIfNeeded
// 処理概要     走行開始禁止条件に該当する場合に開始要求を解除する
// 引数         なし
// 戻り値       true:開始要求を解除した false:走行開始可能
///////////////////////////////////////////////////////////////////////////
static bool blockRunStartIfNeeded(void)
{
	if (isRunStartAllowed())
	{
		return false;
	}

	motorPwmOut(0, 0);
	powerLineSensors(0);
	setupFlags.start = 0;
	autoStart = 0;
	autoStartAnalyze = 0;
	pattern.calibration = 1;
	showRunStartBlocked(getRunStartBlockReason());
	return true;
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

    // 緊急停止処理
	if (patternTrace > 10 && patternTrace < 100 && emcStop > 0)
	{
		goalTime = cntRun;
		patternTrace = 255;
		emergencyStop();
	}

	switch (patternTrace)
	{
	case 0:
		if ((setupFlags.start || autoStart) && blockRunStartIfNeeded())
		{
			break;
		}

		if (autoStart > 1)
		{
			// 2次走行
			motorPwmOut(0, 0);

			// 目標速度調整
			// コース解析
			ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
			ssd1306_SetCursor(0, 25);
			ssd1306_printf(Font_11x18, "Analyzing");
			ssd1306_SetCursor(0, 50);
			ssd1306_printf(Font_6x8, "log %d", autoStartAnalyze);	// 追加: 解析対象ログ番号を表示
			// Ensure SD write buffers are flushed before analysis
			sd_flush_log();

			if (autoStart == 2)
			{
				ret = readLogDistance(autoStartAnalyze);	// 追加: 2走目は1走目ログを解析
			}
			else
			{
				ret = readLogDistanceSlip(autoStartAnalyze);	// 追加: 3走目以降は直前ログをスリップ解析
				if (ret < 0)
				{
					ret = readLogDistance(autoStartAnalyze);	// 追加: 解析失敗時は距離解析へフォールバック
				}
			}
			if(ret > 0)
			{
				// コース解析成功
				countdown = 2000;							  // カウントダウンスタート
				ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
				ssd1306_SetCursor(56, 28);
				ssd1306_printf(Font_16x26, "%d", autoStart);	// 追加: 走行回数を表示

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
				autoStartAnalyze = 0;
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
				if (blockRunStartIfNeeded())
				{
					break;
				}

				motorPwmOut(0, 0);
				countdown = 2000;							  // カウントダウンスタート
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
			batteryVoltage_V = AD2VOLTAGE(batteryAD); // 走行開始直前の電圧を速度FFへ反映
			powerLineSensors(1);   // ラインセンサ点灯

			// SDカードに変数保存
			// PIDゲインを記録
			if(autoStart <= 1)
			{
				writePIDparameters(&lineTraceCtrl);
				writePIDparameters(&lineTraceOmegaFBCtrl);
				writePIDparameters(&veloCtrl);
				writeSpeedFeedForwardGain(speedFeedForwardGain);
				writePIDparameters(&yawRateCtrl);
				writePIDparameters(&yawCtrl);
				writePIDparameters(&distCtrl);

				writeTgtspeeds();  // 目標速度を保存
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
			// スリップ距離補正（パルス版）
			// スタート地点基準で状態を初期化
			// スリップ距離補正（パルス版）
			encTotalN = 0;
			encTotalOptimal = 0;
			encLog = 0;
			encPID = 0;
			enc1 = 0;
			encCurve = 0;
			encChangeGain = 0;
			encClick = 0;
			cntRun = 0;
			cntLog = 0;
			slipResetAll(&slipDetState);
			slipDistReset();
			DistanceOptimal = 0;
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
		if (enc1 >= encMM(10))
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

		int16_t savedLogNo = 0;	// 追加: 保存実績ログ番号
		int16_t endIdxBefore = endFileIndex;	// 追加: endLog前のログ末尾を保持
		// 追加: 表示用の予測ログ番号はSD空でも落ちないようガード
		int16_t predictedLogNo = getNextLogNumber();
		if (modeLOG)
		{
			ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
			ssd1306_SetCursor(0, 25);
			ssd1306_printf(Font_11x18, "log %d", predictedLogNo);
			ssd1306_SetCursor(0, 45);
			ssd1306_printf(Font_11x18, "Writing");

			endLog(); // ログ保存終了

			ssd1306_SetCursor(0, 45);
			ssd1306_printf(Font_11x18, "Written");
		}

		if (autoStart > 0)
		{
			// 追加: 保存成功時のみ解析対象を更新し、失敗時は自動走行を停止
			if (endFileIndex > endIdxBefore)
			{
				savedLogNo = fileNumbers[endFileIndex];	// 追加: 実際に保存されたログ番号を採用
				autoStartAnalyze = savedLogNo;
				// 自動走行モードのときは再度走行準備へ
				autoStart++;
			}
			else
			{
				autoStart = 0;
				autoStartAnalyze = 0;
			}

			if (autoStart > 5)
			{
				// 5走終了
				ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
				ssd1306_SetCursor(0, 15);
				ssd1306_printf(Font_11x18, "Auto run");
				ssd1306_SetCursor(0, 35);
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
					// buffer overflow display
					ssd1306_SetCursor(0, 25);
					ssd1306_printf(Font_11x18, "buff overflow");
					ssd1306_SetCursor(0, 45);
					ssd1306_printf(Font_11x18, "error");
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

	if (patternTrace > 10 && patternTrace < 100) // 走行中のみ処理（制御出力の後に実行）
	{
		logWriteTask(); // 割り込み外でSD書き込みを実行する
	}
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 emergencyStop
// 処理概要     緊急停止処理
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void emergencyStop(void)
{
	// 機体停止処理
	setTargetSpeed(0);
	while (encCurrentN > 5)
	{
		motorPwmOutSynth(0, veloCtrl.pwm, 0, 0);
	}
	motorPwmOut(0, 0);
	
	if (!ssd1306_IsDMARunning())
	{
		ssd1306_UpdateScreen_DMA();        // 停止していた画面更新を再開
	}

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

		ssd1306_SetCursor(0, 15);
		ssd1306_printf(Font_11x18, "EMS!! %d", autoStartAnalyze);

		ssd1306_SetCursor(0, 35);
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
	static bool curveGainHold = false; // 急カーブ進入時にカーブ用ゲインを保持する
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
		curveGainHold = false;
		encChangeGain = 0;
		softresetHandled = true;
	}
	else if (!softreset)
	{
		softresetHandled = false;
	}

	// 距離基準2次走行のみ、ROC閾値で直線/カーブに合わせてゲインを切り替える
	if (optimalTrace == BOOST_DISTANCE && numPPADarry > 0)
	{
		uint16_t rocIndexEnter = optimalIndex + GAIN_CHANGE_LOOKAHEAD_ENTER; // 先読みROCのインデックス
		uint16_t rocIndexExit = optimalIndex + GAIN_CHANGE_LOOKAHEAD_EXIT; // 復帰判定用の先読みROCインデックス
		float rocNow = rocrun;
		float rocEnter = 0.0F;
		float rocExit = 0.0F;
		bool curvePredicted = false;
		bool straightStable = false;
		useStraightGain = true; // デフォルトは直線用ゲイン

		// インデックスが配列範囲を超えないようにガード
		if (rocIndexEnter >= (uint16_t)numPPADarry)
		{
			rocIndexEnter = (uint16_t)(numPPADarry - 1);
		}
		if (rocIndexExit >= (uint16_t)numPPADarry)
		{
			rocIndexExit = (uint16_t)(numPPADarry - 1);
		}

		// 進入判定: 先読みROCと実測ROCのうちカーブ寄り（小さい側）を使う
		if (PPAD[rocIndexEnter].ROC > rocNow)
		{
			rocEnter = rocNow;
		}
		else
		{
			rocEnter = PPAD[rocIndexEnter].ROC;
		}

		// 復帰判定: 実測ROCと近傍先読みROCの両方が直線側であることを要求
		if (PPAD[rocIndexExit].ROC > rocNow)
		{
			rocExit = rocNow;
		}
		else
		{
			rocExit = PPAD[rocIndexExit].ROC;
		}

		curvePredicted = (rocEnter <= GAIN_CHANGE_ROC_ENTER);
		straightStable = (rocExit >= GAIN_CHANGE_ROC_EXIT);

		if (curveGainHold)
		{
			// カーブを抜けきるまでカーブ用ゲインを維持する
			useStraightGain = false;

			if (straightStable)
			{
				// 直線が一定距離続いたら直線用ゲインに戻す
				if (encChangeGain > encMM(GAIN_CHANGE_STRAIGHT_STABLE_MM))
				{
					curveGainHold = false;
					encChangeGain = 0;
					useStraightGain = true;
				}
			}
			else
			{
				// 直線安定が途切れたので復帰用の距離カウントをやり直す
				encChangeGain = 0;
			}
		}
		else
		{
			// 高速直線から急カーブへは先読みROCで早めにカーブ用ゲインへ切替える
			if (curvePredicted)
			{
				curveGainHold = true;
				encChangeGain = 0;
				useStraightGain = false;
			}
		}
	}
	else
	{
		curveGainHold = false;
		encChangeGain = 0;
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
	uint16_t highReq, uint16_t lowReq, bool highEnabled)
{
	// ヒステリシス判定（縦スリップ等の汎用）
	if (!*flag) {
		*lowCount = 0; // ★追加：OFF中は解除側カウントを必ず0に
		if (highEnabled && (value > highTh)) {
			if (++(*highCount) >= highReq) {
				*flag = true;
				*highCount = 0;
				*lowCount  = 0; // ★追加：ON遷移時も念のため
			}
		} else {
			*highCount = 0;
		}
	} else {
		*highCount = 0; // ★追加：ON中はON側カウントを必ず0に
		if (value < lowTh) {
			if (++(*lowCount) >= lowReq) {
				*flag = false;
				*lowCount = 0;
				*highCount = 0; // ★追加：OFF遷移時も念のため
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
	slipISumOkCount = 0;
	slipFlag = false;
	slipFlagLat = false;
	slipLongOnCountEnabledLog = false;
	slipLongLowloadClearCount = 0;
	slipLatEnabledLog = false;
	slipLatOnCountEnabledLog = false;
	slipThresholdHighLog = 0.0f;
	slipThresholdLowLog = 0.0f;
	slipLatHighLog = 0.0f;
	slipLatLowLog = 0.0f;
	slipCurScaleLog = 1.0f;
	slipTurningStateLog = false;
	slipAxBiasLog = 0.0f;
	slipAyBiasLog = 0.0f;
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
// モジュール名 slipDistReset
// 処理概要     距離補正の内部状態をリセット
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
static void slipDistReset(void)
{
	// 距離補正スケールと積算距離を初期化
	slipDistScaleRaw = 1.0f;
	slipDistScaleF = 1.0f;
	// スリップ距離補正（パルス版）
	distEncRaw_p = 0;
	distCorr_p = 0;
	distSlipLoss_p = 0;
	distCorrFrac_p = 0.0f;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 slipDistUpdateAndApply_p
// 処理概要     距離補正スケールのLPF更新とパルス補正
// 引数         rawScale: 距離補正スケール（生）
//              dEncRaw_p: 生パルス[1ms]
// 戻り値       補正後パルス[1ms]
///////////////////////////////////////////////////////////////////////////
static int32_t slipDistUpdateAndApply_p(float rawScale, int32_t dEncRaw_p)
{
	// 距離補正スケールをLPFで平滑化（悪化は速く/回復は遅く）
	float coef = (rawScale < slipDistScaleF) ? SLIP_DIST_LPF_COEF_DOWN : SLIP_DIST_LPF_COEF_UP;
	slipDistScaleF = lpf1(slipDistScaleF, rawScale, coef);

	// スリップ距離補正（パルス版）
	distEncRaw_p += dEncRaw_p;
	float tmp_p = ((float)dEncRaw_p * slipDistScaleF) + distCorrFrac_p;
	int32_t dCorr_i = (int32_t)tmp_p;
	distCorrFrac_p = tmp_p - (float)dCorr_i;
	distCorr_p += dCorr_i;
	int32_t dLoss_p = dEncRaw_p - dCorr_i;
	distSlipLoss_p += dLoss_p;

	encCurrentCorr_p = dCorr_i;
	return dCorr_i;
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
	slipLongOnCountEnabledLog = false;
	slipLongLowloadClearCount = 0;
	slipThresholdHighLog = 0.0f;
	slipThresholdLowLog = 0.0f;
	slipLatHighLog = 0.0f;
	slipLatLowLog = 0.0f;
	slipCurScaleLog = 1.0f;
	slipTurningStateLog = st->turningState;
	slipAxBiasLog = st->axBias;
	slipAyBiasLog = st->ayBias;

	float ax0 = imuVal.accele.y * 9.81f;
	float ay0 = imuVal.accele.x * 9.81f;
	float wz0 = imuVal.gyro.z * DEG2RAD;
	st->turningState = (fabsf(wz0) > SLIP_GYRO_ON_RADS);
	slipTurningStateLog = st->turningState;
	float pwmSum = fabsf((float)motorpwmL) + fabsf((float)motorpwmR);
	float iSum = fabsf(motorCurrentL) + fabsf(motorCurrentR);

	if (!st->turningState
			&& (pwmSum < SLIP_BIAS_PWM_MAX)
			&& (iSum < SLIP_BIAS_ISUM_MAX)) {
		st->axBias = ax0;
		st->ayBias = ay0;
	}
	slipAxBiasLog = st->axBias;
	slipAyBiasLog = st->ayBias;

#if SLIP_CUR_ENABLE
	// PWM/電流LPF初期化（開始直後のゲート鈍化を防止）
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
			// 距離補正の状態もリセット
			slipDistReset();
		}
		st->prevRunning = false;
		st->prevMoving  = false;
		return;
	}
	st->prevRunning = true;

	// ---- エンコーダ速度 ----
	float encSpeed = (float)encCurrentN / PULSE_MILLIMETER;

	//==========================================================
	// 超低速：遷移時だけリセット（微分スパイク防止）
	//==========================================================
	if (fabsf(encSpeed) < SLIP_SPEED_SKIP_MPS) {

		if (st->prevMoving) {
			// 低速遷移時の全リセット
			slipResetAll(st);
		}

		st->prevMoving = false;

		float ax = imuVal.accele.y * 9.81f;
		float ay = imuVal.accele.x * 9.81f;
		float wz = imuVal.gyro.z * DEG2RAD;
		float pwmSum = fabsf((float)motorpwmL) + fabsf((float)motorpwmR);
		float iSum = fabsf(motorCurrentL) + fabsf(motorCurrentR);
		float absWz = fabsf(wz);
		updateTurningHysteresis(st, absWz);
		// 低速かつ低トルク時だけバイアス更新する
		if (!st->turningState
				&& (pwmSum < SLIP_BIAS_PWM_MAX)
				&& (iSum < SLIP_BIAS_ISUM_MAX)) {
			st->axBias = lpf1(st->axBias, ax, SLIP_ACC_BIAS_COEF);
			st->ayBias = lpf1(st->ayBias, ay, SLIP_ACC_BIAS_COEF);
		}

		// ログ残差は停止中に残らないようゼロ化
		slipDeltaEnc = 0.0f;
		slipDeltaImu = 0.0f;
		// フィルタ値だけはゼロへ軽く収束
		slipIndicatorRaw = lpf1(slipIndicatorRaw, 0.0f, SLIP_LPF_COEF);
		// Latは専用LPFで0へ収束させる
		slipIndicatorFiltered = lpf1(slipIndicatorFiltered, 0.0f, SLIP_LPF_COEF_LAT);
		// 低速スキップ領域では距離補正スケールを1.0へ寄せる
		slipDistScaleRaw = 1.0f;
		slipLongOnCountEnabledLog = false;
		slipLongLowloadClearCount = 0;
		slipLatEnabledLog = false;
		slipLatOnCountEnabledLog = false;
		slipThresholdHighLog = 0.0f;
		slipThresholdLowLog = 0.0f;
		slipLatHighLog = 0.0f;
		slipLatLowLog = 0.0f;
		slipCurScaleLog = 1.0f;
		slipTurningStateLog = st->turningState;
		slipAxBiasLog = st->axBias;
		slipAyBiasLog = st->ayBias;
		return;
	}
	st->prevMoving = true;

	// リングバッファ初期化スパイク対策
	if (!st->slipPrimed) {
		// 開始直後のバッファ初期化
		slipPrimeSpeedHist(st, encSpeed);
		// 初回は距離補正スケールを1.0で積算
		slipDistScaleRaw = 1.0f;
		slipLongOnCountEnabledLog = false;
		slipLongLowloadClearCount = 0;
		slipLatEnabledLog = false;
		slipLatOnCountEnabledLog = false;
		slipThresholdHighLog = 0.0f;
		slipThresholdLowLog = 0.0f;
		slipLatHighLog = 0.0f;
		slipLatLowLog = 0.0f;
		slipCurScaleLog = 1.0f;
		slipTurningStateLog = st->turningState;
		slipAxBiasLog = st->axBias;
		slipAyBiasLog = st->ayBias;
		return; // 初回は判定しない
	}

	// ---- IMU加速度（ボディ座標）----
	float ax = imuVal.accele.y * 9.81f;  // 前後
	float ay = imuVal.accele.x * 9.81f;  // 左右

	// ---- yaw角速度（rad/s）----
	float wz = imuVal.gyro.z * DEG2RAD;

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
	// PWM/電流の合計（惰性/低トルク判定用）
	//==========================================================
	// 瞬時値はバイアス更新とON判定ゲートで使用する
	float pwmSum = fabsf((float)motorpwmL) + fabsf((float)motorpwmR);
	float iSum = fabsf(motorCurrentL) + fabsf(motorCurrentR);
#if SLIP_CUR_ENABLE
	// PWM/電流のLPF更新（SLIP_CUR_ENABLE=1のみ）
	st->pwmSumF = lpf1(st->pwmSumF, pwmSum, SLIP_PWM_LPF_COEF);
	st->iSumF = lpf1(st->iSumF, iSum, SLIP_CUR_LPF_COEF);
#endif

	//==========================================================
	// IMUのDC成分（バイアス/傾き由来の重力漏れ等）を除去
	//==========================================================
	updateTurningHysteresis(st, fabsf(wz));
	if (!st->turningState
			&& (fabsf(encAx) < SLIP_BIAS_ENCAX_MAX)
			&& (pwmSum < SLIP_BIAS_PWM_MAX)
			&& (iSum < SLIP_BIAS_ISUM_MAX)) {
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
	float longSpeedErr = fabsf(((float)targetSpeed / PULSE_MILLIMETER) - encSpeed);
#if SLIP_CUR_ENABLE
	float longLoadISum = st->iSumF;
#else
	float longLoadISum = iSum;
#endif
	bool slipLongOnCountEnabled = !((longSpeedErr < SLIP_LONG_LOWLOAD_SPEEDERR_MAX)
			&& (longLoadISum < SLIP_LONG_LOWLOAD_ISUM_MAX));
	// 横判定のカウントを許可する条件（PWMが小さい区間は止める）
	bool latCountEnabled = latEnabled;
	bool latOnCountEnabled = latEnabled;
	bool latCoastHardClear = false;
#if SLIP_CUR_ENABLE
	// SLIP_CUR_ENABLE=1ではPWM/電流ゲートを適用
	latCountEnabled = latEnabled && (st->pwmSumF > SLIP_PWM_LAT_COUNT_MIN);
	// 惰性/低トルク時は横判定を強制クリアする
	latCoastHardClear = (!calibrateMotorCurrent)
			&& (st->pwmSumF < SLIP_PWM_COAST_MAX)
			&& (st->iSumF < SLIP_ISUM_COAST_MAX);
	// LatのON判定はLPF後電流が連続で閾値以上の時だけ許可する
	bool iSumOk = (st->iSumF > SLIP_ISUM_LAT_COUNT_MIN);
	if (!latEnabled || latCoastHardClear) {
		slipISumOkCount = 0;
	} else if (iSumOk) {
		if (slipISumOkCount < SLIP_ISUM_LAT_COUNT_N) {
			slipISumOkCount++;
		}
	} else {
		slipISumOkCount = 0;
	}
	// LatのON判定はLPF後PWM、瞬時PWM、電流連続条件を満たす場合のみ進める
	latOnCountEnabled = latCountEnabled
			&& (pwmSum > SLIP_PWM_LAT_COUNT_MIN)
			&& (slipISumOkCount >= SLIP_ISUM_LAT_COUNT_N);
#else
	latOnCountEnabled = latCountEnabled
			&& (pwmSum > SLIP_PWM_LAT_COUNT_MIN);
#endif
	slipLongOnCountEnabledLog = slipLongOnCountEnabled;
	slipLatEnabledLog = latEnabled;
	slipLatOnCountEnabledLog = latOnCountEnabled;

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
			// Latは専用LPFで0へ収束させる
			slipIndicatorFiltered = lpf1(slipIndicatorFiltered, 0.0f, SLIP_LPF_COEF_LAT);
		}
	} else {
		// 縦スリップ指標（absDxをLPF）
		slipIndicatorRaw = lpf1(slipIndicatorRaw, absDx, SLIP_LPF_COEF);
		// 横スリップ指標（absDyをLPF）
		if (latCoastHardClear) {
			// 惰性時は横指標を強制的にクリアして誤検知を抑制
			slipIndicatorFiltered = lpf1(slipIndicatorFiltered, 0.0f, SLIP_LAT_CLEAR_COEF);
			slipHighCountLat = 0;
			slipLowCountLat = 0;
			slipFlagLat = false;
		} else if (latEnabled) {
			// 旋回中はPWM/電流ゲートに関係なくLat指標を追従させる
			slipIndicatorFiltered = lpf1(slipIndicatorFiltered, absDy, SLIP_LPF_COEF_LAT);
		} else {
			// 旋回外はLat指標を0へ収束させる
			slipIndicatorFiltered = lpf1(slipIndicatorFiltered, 0.0f, SLIP_LPF_COEF_LAT);
		}
	}

	// ログ用（低加速度時は0に収束）
	slipDeltaEnc = dx;	// 縦方向残差
	slipDeltaImu = dy;	// 横方向残差

	//==============================
	// 旋回が強い時だけ閾値を上げる
	//==============================
	// 閾値は毎回算出し、状態として保持しない
	float slipThresholdHigh = SLIP_MISMATCH_HIGH;
	float slipThresholdLow = SLIP_MISMATCH_LOW;

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
		SLIP_HIGH_COUNT_REQ, SLIP_LOW_COUNT_REQ, slipLongOnCountEnabled);
	if (slipFlag && !slipLongOnCountEnabled) {
		if (slipLongLowloadClearCount < SLIP_LONG_LOWLOAD_CLEAR_COUNT) {
			slipLongLowloadClearCount++;
		}
		if (slipLongLowloadClearCount >= SLIP_LONG_LOWLOAD_CLEAR_COUNT) {
			slipFlag = false;
			slipHighCount = 0;
			slipLowCount = 0;
			slipLongLowloadClearCount = 0;
		}
	} else {
		slipLongLowloadClearCount = 0;
	}

	// ---- 横スリップ判定（旋回中のみ）----
	float slipLatHigh = SLIP_LAT_HIGH * curScale;
	float slipLatLow  = SLIP_LAT_LOW  * curScale;
	slipThresholdHighLog = slipThresholdHigh;
	slipThresholdLowLog = slipThresholdLow;
	slipLatHighLog = slipLatHigh;
	slipLatLowLog = slipLatLow;
	slipCurScaleLog = curScale;
	slipTurningStateLog = st->turningState;
	slipAxBiasLog = st->axBias;
	slipAyBiasLog = st->ayBias;
	if (!slipFlagLat) {
		if (latOnCountEnabled) {
			// LatのON判定はPWM/電流ゲート許可時のみ進める
			if (slipIndicatorFiltered > slipLatHigh) {
				if (++slipHighCountLat >= SLIP_HIGH_COUNT_REQ_LAT) {
					slipFlagLat = true;
					slipHighCountLat = 0;
				}
			} else {
				slipHighCountLat = 0;
			}
		} else {
			// ゲート未達時はON側カウントを進めない
			slipHighCountLat = 0;
		}
		slipLowCountLat = 0;
	} else {
		// LatのOFF判定はゲート未達でも進めて解除できるようにする
		if (slipIndicatorFiltered < slipLatLow) {
			if (++slipLowCountLat >= SLIP_LOW_COUNT_REQ_LAT) {
				slipFlagLat = false;
				slipLowCountLat = 0;
			}
		} else {
			slipLowCountLat = 0;
		}
	}

	//==========================================================
	// 距離補正スケール算出（既存指標と閾値のみ使用）
	//==========================================================
	float scaleLong = 1.0f;
	float scaleLat = 1.0f;

	if (slipThresholdHigh > slipThresholdLow) {
		float sevLong = (slipIndicatorRaw - slipThresholdLow) / (slipThresholdHigh - slipThresholdLow);
		sevLong = fminf(fmaxf(sevLong, 0.0f), 1.0f);
		scaleLong = 1.0f - sevLong * (1.0f - SLIP_DIST_MIN_SCALE);
	}

	if (latOnCountEnabled && (slipLatHigh > slipLatLow)) {
		float sevLat = (slipIndicatorFiltered - slipLatLow) / (slipLatHigh - slipLatLow);
		sevLat = fminf(fmaxf(sevLat, 0.0f), 1.0f);
		scaleLat = 1.0f - sevLat * (1.0f - SLIP_DIST_MIN_SCALE_LAT);
	}

	// 縦/横のうち厳しい方を採用し、[MIN, 1]にクランプ
	slipDistScaleRaw = fminf(scaleLong, scaleLat);
	float minScale = fminf(SLIP_DIST_MIN_SCALE, SLIP_DIST_MIN_SCALE_LAT);
	slipDistScaleRaw = fminf(fmaxf(slipDistScaleRaw, minScale), 1.0f);
}
///////////////////////////////////////////////////////////////////////////
// モジュール名	getSlipIndicatorRaw
// 				getSlipIndicatorFiltered
// 				getSlipFlag
// 				getSlipFlagLat
// 処理概要     割り込み外からスリップ検出結果を参照するためのアクセサ
///////////////////////////////////////////////////////////////////////////
float getSlipIndicatorRaw(void)
{
	return slipIndicatorRaw;
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
bool getSlipLongOnCountEnabled(void)
{
	return slipLongOnCountEnabledLog;
}
uint16_t getSlipLongLowloadClearCount(void)
{
	return slipLongLowloadClearCount;
}
bool getSlipLatEnabled(void)
{
	return slipLatEnabledLog;
}
bool getSlipLatOnCountEnabled(void)
{
	return slipLatOnCountEnabledLog;
}
uint16_t getSlipISumOkCount(void)
{
	return slipISumOkCount;
}
float getSlipPwmSumF(void)
{
#if SLIP_CUR_ENABLE
	return slipDetState.pwmSumF;
#else
	return 0.0f;
#endif
}
float getSlipISumF(void)
{
#if SLIP_CUR_ENABLE
	return slipDetState.iSumF;
#else
	return 0.0f;
#endif
}
float getSlipEncAyF(void)
{
	return slipDetState.encAyF;
}
float getSlipImuAyF(void)
{
	return slipDetState.imuAyF;
}
float getSlipThresholdHigh(void)
{
	return slipThresholdHighLog;
}
float getSlipThresholdLow(void)
{
	return slipThresholdLowLog;
}
float getSlipLatHigh(void)
{
	return slipLatHighLog;
}
float getSlipLatLow(void)
{
	return slipLatLowLog;
}
float getSlipCurScale(void)
{
	return slipCurScaleLog;
}
bool getSlipTurningState(void)
{
	return slipTurningStateLog;
}
float getSlipAxBias(void)
{
	return slipAxBiasLog;
}
float getSlipAyBias(void)
{
	return slipAyBiasLog;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 Control_ApplyMarkerCorrection_p
// 処理概要     マーカー補正値をスリップ補正後パルスへ反映する
// 引数         diff_p: 補正量[パルス]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void Control_ApplyMarkerCorrection_p(int32_t diff_p)
{
	encTotalOptimal -= diff_p;	// 走行距離の補正反映
	distCorr_p -= diff_p;		// 補正後パルスを同期
	distCorrFrac_p = 0.0f;		// 端数を破棄して一致性を安定化
}
int32_t Control_GetEncCurrentCorr_p(void)
{
	return encCurrentCorr_p;
}
// スリップ距離補正（パルス版）
int32_t Control_GetDistEncRaw_p(void)
{
	return distEncRaw_p;
}
int32_t Control_GetDistCorr_p(void)
{
	return distCorr_p;
}
int32_t Control_GetDistSlipLoss_p(void)
{
	return distSlipLoss_p;
}
float Control_GetSlipDistScale(void)
{
	return slipDistScaleF;
}
float Control_GetSlipDistScaleRaw(void)
{
	return slipDistScaleRaw;
}

#define TARGET_SPEED_PARAM_COUNT ((int16_t)(sizeof(speedParam) / sizeof(float)))

///////////////////////////////////////////////////////////////////////////
// モジュール名 isTargetSpeedStoredValueInRange
// 処理概要     targetSpeeds.txtの保存値が項目ごとの許容範囲内か判定する
// 引数         index:速度パラメータ番号, value:保存値
// 戻り値       true:範囲内 false:範囲外
///////////////////////////////////////////////////////////////////////////
static bool isTargetSpeedStoredValueInRange(int16_t index, int16_t value)
{
	int16_t maxValue = 1000; // 10.00m/s

	if (index == 14 || index == 15)
	{
		maxValue = 2000; // 20.00m/s^2
	}
	else if (index == 17)
	{
		maxValue = 9900; // 99.00mm
	}

	return value >= 0 && value <= maxValue;
}

///////////////////////////////////////////////////////////////////////////
// モジュール名 applyTargetSpeedStoredValue
// 処理概要     targetSpeeds.txtの保存値を速度パラメータへ反映する
// 引数         index:速度パラメータ番号, value:保存値
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
static void applyTargetSpeedStoredValue(int16_t index, int16_t value)
{
	float converted = (float)value / 100.0F;

	switch (index)
	{
	case 0: tgtParam.search = converted; break;
	case 1: tgtParam.stop = converted; break;
	case 2: tgtParam.bstStraight = converted; break;
	case 3: tgtParam.bst1500 = converted; break;
	case 4: tgtParam.bst1300 = converted; break;
	case 5: tgtParam.bst1000 = converted; break;
	case 6: tgtParam.bst800 = converted; break;
	case 7: tgtParam.bst700 = converted; break;
	case 8: tgtParam.bst600 = converted; break;
	case 9: tgtParam.bst500 = converted; break;
	case 10: tgtParam.bst400 = converted; break;
	case 11: tgtParam.bst300 = converted; break;
	case 12: tgtParam.bst200 = converted; break;
	case 13: tgtParam.bst100 = converted; break;
	case 14: tgtParam.acceleF = converted; break;
	case 15: tgtParam.acceleD = converted; break;
	case 16: tgtParam.shortCut = converted; break;
	case 17: tgtParam.decelLeadMm = converted; break;
	default: break;
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 setEncoderVal
// 処理概要     エンコーダ値を変数に加算する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void setEncoderVal(void)
{
	static bool prevRunning = false;
	int32_t dEncRaw_p = (int32_t)encCurrentN;
	bool running = (patternTrace >= 11 && patternTrace < 100);
	int32_t dEncCorr_p = dEncRaw_p;

	if (running) {
		dEncCorr_p = slipDistUpdateAndApply_p(slipDistScaleRaw, dEncRaw_p);
	} else {
		if (prevRunning) {
			slipDistReset();
		}
		encCurrentCorr_p = dEncRaw_p;
	}
	prevRunning = running;

#if SLIP_DIST_CORRECTION_ENABLE
	int32_t dEncUse_p = dEncCorr_p;
#else
	(void)dEncCorr_p;
	int32_t dEncUse_p = dEncRaw_p;
#endif

	// 外部変数
	enc1 += dEncUse_p;				// 通常トレース用
	encRightMarker += dEncUse_p;	// ゴールマーカ判定用
	encCurve += dEncUse_p;			// カーブ処理用
	encChangeGain += dEncUse_p;		// ゲイン変更用
	encTotalOptimal += dEncUse_p; 	// 2次走行用
	encLog += dEncUse_p;			// 一定距離ごとにログを保存する用
	encPID += dEncUse_p;			// 距離制御用
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
	fresult = f_open(&fil, fileName, FA_CREATE_ALWAYS | FA_WRITE); // ファイルを開く

	if (fresult == FR_OK)
	{
		for (i = 0; i < sizeof(speedParam) / sizeof(float); i++)
		{
			strcat(format, "%04d,");
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
										(int32_t)(round(tgtParam.shortCut * 100)),
										(int32_t)(round(tgtParam.decelLeadMm * 100)));
		f_close(&fil);
	}
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 readTgtspeeds
// 処理概要     速度パラメータをSDカードから読み取る
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void readTgtspeeds(void)
{
	FIL fil;
	FRESULT fresult;
	char fileName[30] = PATH_SETTING;
	TCHAR paramStr[100];
	int16_t i;
	bool repair = false;

	// ファイル読み込み
	strcat(fileName, FILENAME_TARGET_SPEED);					  // ファイル名追加
	strcat(fileName, ".txt");									  // 拡張子追加
	fresult = f_open(&fil, fileName, FA_OPEN_EXISTING | FA_READ); // ファイルを開く

	if (fresult == FR_OK)
	{
		// speedParamの定義順で読み取る。途中で終端した場合はそこで打ち切る。
		for (i = 0; i < TARGET_SPEED_PARAM_COUNT; i++)
		{
			int16_t value = 0;
			if (f_gets(paramStr, 6, &fil) == NULL)
			{
				repair = true;
				break;
			}
			if (sscanf(paramStr, "%04hd,", &value) != 1)
			{
				repair = true;
				break;
			}
			if (isTargetSpeedStoredValueInRange(i, value))
			{
				applyTargetSpeedStoredValue(i, value);
			}
			else
			{
				repair = true;
			}
		}
		if (i < TARGET_SPEED_PARAM_COUNT)
		{
			repair = true;
		}
		f_close(&fil);
	}
	else
	{
		repair = true;
	}

	if (repair)
	{
		writeTgtspeeds();
	}
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

	if (isLineSensorCalibrationValid())
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

