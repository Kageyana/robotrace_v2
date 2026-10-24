//====================================//
// インクルード
//====================================//
#include "control.h"
#include "slipDetection.h"
#include "autoRun.h"
#include "pathPolicy.h"
#include "pathFollower.h"
#include "BMI088.h"
#include "PIDcontrol.h"
#include "encoder.h"
#include "motor.h"
#include "fatfs.h"
#include "battery.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define DISPLAY_INIT_DMA_TIMEOUT_MS 200U

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

static AutoRunPlan autoRunCurrentPlan = {0U, AUTO_RUN_MODE_INVALID, 0, 0};
static AutoRunConfigLoadResult autoRunConfigLoadResult = AUTO_RUN_CONFIG_LOAD_IO_ERROR;
bool stateCrossLine = false;		// クロスライン検出状態
float rocrun = 2000;		// 曲率半径計算用変数

uint16_t analogValLSon[NUM_SENSORS+1]; // ADC結果格納配列
uint16_t analogValLSoff[NUM_SENSORS+1]; // ADC結果格納配列s
uint16_t analogVal2[4];			  // ADC結果格納配列

// 速度パラメータ関連
speedParam tgtParam = {
#define SPEED_PARAM_DEFAULT(member, initial, label, step, minimum, maximum, unit) initial,
    SPEED_PARAM_FIELDS(SPEED_PARAM_DEFAULT)
#undef SPEED_PARAM_DEFAULT
};
// 経路追従へ渡す1msの生エンコーダ増分。未検証の滑り距離補正は行わない。
static int32_t encCurrentCorr_p = 0;
static const char *getRunStartBlockReason(void);
static bool isRunStartAllowed(void);
static void showRunStartBlocked(const char *reason);
static bool blockRunStartIfNeeded(void);
/////////////////////////////////////////////////////////////////////
// モジュール名 updateDisplayDmaAndWait
// 処理概要     初期化表示をDMA更新し、完了しない場合はタイムアウトで抜ける
// 引数         timeout_ms: 待ち時間上限[ms]
// 戻り値       true: 更新完了 false: タイムアウト
/////////////////////////////////////////////////////////////////////
static bool updateDisplayDmaAndWait(uint32_t timeout_ms)
{
	uint32_t startTick = HAL_GetTick();

	if (!modeDSP)
	{
		return true;
	}

	ssd1306_UpdateScreen_DMA();
	while (!ssd1306_IsTransferCompleted())
	{
		if ((HAL_GetTick() - startTick) >= timeout_ms)
		{
			ssd1306_StopDMA();
			SSD1306_DMA_Completed = 1;
			return false;
		}
	}

	return true;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 showSlipAnalysisProgress
// 処理概要     SLIP解析の段階と読込行数を画面に表示する
// 引数         stage: 解析段階、lineNo: 読み込んだ行数
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void showSlipAnalysisProgress(const char *stage, uint32_t lineNo)
{
	if (!modeDSP)
	{
		return;
	}
	ssd1306_FillRectangle(0, 42, 127, 63, Black);
	ssd1306_SetCursor(0, 42);
	ssd1306_printf(Font_6x8, "SLIP %d", autoRunCurrentPlan.slipSourceLogNumber);
	ssd1306_SetCursor(0, 53);
	ssd1306_printf(Font_6x8, "%s %lu", stage, (unsigned long)lineNo);
	if (!ssd1306_IsDMARunning())
	{
		(void)updateDisplayDmaAndWait(DISPLAY_INIT_DMA_TIMEOUT_MS);
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 finishBinaryLogStorage
// 処理概要     停止出力を保ち、endLogで確定したbinの保存状態を確認する
// 引数         なし
// 戻り値       true: 保存可能 false: 保存・初期確認失敗
/////////////////////////////////////////////////////////////////////
static bool finishBinaryLogStorage(void)
{
    motorCommandOut(0, 0);
    return logRecoveryIsReady();
}

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
	pathFollowerInvalidateRoute();
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
	motorCommandOut(0, 0);
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
		updateDisplayDmaAndWait(DISPLAY_INIT_DMA_TIMEOUT_MS); // グラフィック液晶更新

		setLED(0, 0, 50, 0); // 初期化 成功 緑点灯
	}
	else
	{
		modeDSP = false;
		setLED(0, 50, 0, 0); // 初期化 失敗 赤点灯
	}
	sendLED();

	// microSD
	autoRunState.configReady = false;
	autoRunConfigLoadResult = AUTO_RUN_CONFIG_LOAD_IO_ERROR;
	ssd1306_SetCursor(0, 28);
	if (insertSD())
	{
		if(!softreset || !initMSD)
		{
			initMSD = initMicroSD();
		}

		if(initMSD || softreset)
		{
			(void)logPrepareBinaryStorage(); // binのヘッダと保存状態だけを確認し、CSV化しない
			cntFiles = getFileNumbers();	// 走行ログのファイル番号を取得
			getLogNumber();		// 前回の解析ログナンバーを取得

			// 前回のPIDゲインを取得
			readPIDparameters(&lineTraceCtrl);
			readPIDparameters(&veloCtrl);
			readSpeedFeedForwardGain(&speedFeedForwardGain);	// 速度フィードフォワード係数も読み出す
			readPIDparameters(&yawRateCtrl);
			readPIDparameters(&lineTraceOmegaFBCtrl);

			readLinesenval(); // ラインセンサの最大値と最小値を取得
			readTgtspeeds();  // 目標速度を取得
			readShortcutSettings(); // 経路追従・ショートカット設定を取得
			autoRunConfigLoadResult = readAutoRunSettings(); // オートスタート2～5走目の方式を取得
			readImuTempCompensation(); // BMI088ジャイロZ温度係数を取得

			if (modeDSP)
			{
				if (!logRecoveryIsReady()) ssd1306_printf(Font_6x8, "SD BIN invalid");
				else if (logHasIncompleteTemp()) ssd1306_printf(Font_6x8, "SD TMP kept");
				else ssd1306_printf(Font_6x8, "MicroSD success");
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
		updateDisplayDmaAndWait(DISPLAY_INIT_DMA_TIMEOUT_MS); // 全ページ送信完了まで待つ
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
		updateDisplayDmaAndWait(DISPLAY_INIT_DMA_TIMEOUT_MS); // 全ページ送信完了まで待つ
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
			updateDisplayDmaAndWait(DISPLAY_INIT_DMA_TIMEOUT_MS); // 全ページ送信完了まで待つ
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
			updateDisplayDmaAndWait(DISPLAY_INIT_DMA_TIMEOUT_MS); // 全ページ送信完了まで待つ

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
			updateDisplayDmaAndWait(DISPLAY_INIT_DMA_TIMEOUT_MS); // 全ページ送信完了まで待つ

			HAL_Delay(1000);
		}
	}

	clearLED();
	// DWT初期化
	initCycleCounter();
	resetCycleCounter();
	enableCycleCounter(); // カウント開始

	updateBatteryVoltage(); // バッテリ電圧を計算

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
	if (insertSD() && !logRecoveryIsReady()) return "BIN storage failed";
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
	bool abortedAuto = autoStart > 0U;
	if (autoStart > 0U && !autoRunState.configReady)
	{
		const char *reason = (autoRunConfigLoadResult == AUTO_RUN_CONFIG_LOAD_IO_ERROR) ?
			"SD READ ERROR" : "CFG REPAIR NG";
		motorCommandOut(0, 0);
		powerLineSensors(0);
		setupFlags.start = 0;
		autoStart = 0;
		autoStartAnalyze = 0;
		autoRunAbortSeries(&autoRunState);
		(void)finishBinaryLogStorage();
		showRunStartBlocked(reason);
		return true;
	}
	if (isRunStartAllowed())
	{
		return false;
	}

	motorCommandOut(0, 0);
	powerLineSensors(0);
	setupFlags.start = 0;
	autoStart = 0;
	autoStartAnalyze = 0;
	autoRunAbortSeries(&autoRunState);
	if (abortedAuto) (void)finishBinaryLogStorage();
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
			if (!autoRunPrepareRun(&autoRunState, autoStart, &autoRunCurrentPlan))
			{
				autoStart = 0U;
				autoStartAnalyze = 0;
				autoRunAbortSeries(&autoRunState);
				(void)finishBinaryLogStorage();
				showRunStartBlocked("auto_run source missing");
				break;
			}
			// 2次走行
			motorCommandOut(0, 0);

			// 目標速度調整
			// コース解析
			ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
			ssd1306_SetCursor(0, 25);
			ssd1306_printf(Font_11x18, "Analyzing");
			ssd1306_SetCursor(0, 50);
			ssd1306_printf(Font_6x8, "%s %d", autoRunModeName(autoRunCurrentPlan.requestedMode),
				autoRunCurrentPlan.requestedMode == AUTO_RUN_MODE_SLIP ? autoRunCurrentPlan.slipSourceLogNumber :
				autoRunCurrentPlan.primaryLogNumber);
			// endLog() で確定済みのバイナリを直接解析する。

			switch (autoRunCurrentPlan.requestedMode)
			{
			case AUTO_RUN_MODE_DISTANCE:
				ret = readLogDistance(autoRunCurrentPlan.primaryLogNumber);
				break;
			case AUTO_RUN_MODE_SLIP:
				ret = readLogDistanceSlip(autoRunCurrentPlan.primaryLogNumber,
					autoRunCurrentPlan.slipSourceLogNumber, showSlipAnalysisProgress);
				break;
			case AUTO_RUN_MODE_PATH:
				ret = routeBuildFromLog(autoRunCurrentPlan.primaryLogNumber,
					pathPolicyGenerationLevel(false, shortcutSettings.maxLevel));
				break;
			case AUTO_RUN_MODE_SHORTCUT:
				ret = routeBuildFromLog(autoRunCurrentPlan.primaryLogNumber,
					pathPolicyGenerationLevel(true, shortcutSettings.maxLevel));
				break;
			default:
				ret = -1;
				break;
			}
			if(ret > 0)
			{
				// コース解析成功
				countdown = 7000;							  // 3秒カウントダウンを開始
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
				autoRunAbortSeries(&autoRunState);
				(void)finishBinaryLogStorage();
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
				if (autoStart == 1U)
				{
					autoRunBeginSeries(&autoRunState);
					if (!autoRunPrepareRun(&autoRunState, 1U, &autoRunCurrentPlan))
					{
						autoStart = 0U;
						autoRunAbortSeries(&autoRunState);
						showRunStartBlocked("auto_run config invalid");
						break;
					}
					optimalTrace = BOOST_NONE;
				}

				motorCommandOut(0, 0);
				countdown = 7000;							  // 3秒カウントダウンを開始
				ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
				ssd1306_SetCursor(56, 28);
				ssd1306_printf(Font_16x26, "3");

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
			}
		}

		// IMUのキャリブレーションが終了したら走行開始
		if (!calibratIMU && IMU_CalibrationReady() && !calibrateMotorCurrent && countdown == 0)
		{
			updateBatteryVoltage(); // 走行開始直前の電圧を反映
			powerLineSensors(1);   // ラインセンサ点灯

			// SDカードに変数保存
			// PIDゲインを記録
			if(autoStart <= 1)
			{
				if (autoStart == 1 && initMSD)
				{
					writeLinesenval(); // 1走目前に実施したラインセンサ校正値を保存
				}
				writePIDparameters(&lineTraceCtrl);
				writePIDparameters(&lineTraceOmegaFBCtrl);
				writePIDparameters(&veloCtrl);
				writeSpeedFeedForwardGain(speedFeedForwardGain);
				writePIDparameters(&yawRateCtrl);

				writeTgtspeeds();  // 目標速度を保存
			}

			if (initMSD)
			{
				initLog(); // 番号付きバイナリの作業ファイル作成
				if (!initMSD || !logRecoveryIsReady()) {
					motorCommandOut(0, 0); autoStart = 0; setupFlags.start = 0;
					autoRunAbortSeries(&autoRunState);
					(void)finishBinaryLogStorage();
					showRunStartBlocked("BIN open failed");
					patternTrace = 103; break;
				}
			}

			// 変数初期化
			encRightMarker = 0;
			veloCtrl.Int = 0.0f;
			yawRateCtrl.Int = 0.0f;

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
		motorCommandOutSynth(lineTraceCtrl.pwm, veloCtrl.pwm, 0);
		// motorCommandOut(veloCtrlL.pwm,veloCtrlR.pwm);

		// スタートマーカーを通過したら本走行に移行
		if (SGmarker > 0)
		{
			// 変数初期化
			// スリップ距離補正（パルス版）
			// スタート地点基準で状態を初期化
			// スリップ距離補正（パルス版）
			encTotalN = 0;
			encTotalL = 0;
			encTotalR = 0;
			encTotalOptimal = 0;
			encLog = 0;
			enc1 = 0;
			encCurve = 0;
			encChangeGain = 0;
			encClick = 0;
			cntRun = 0;
			cntLog = 0;
			SlipDetection_Reset(&slipV2);
			encCurrentCorr_p = 0;
			DistanceOptimal = 0;
			optimalIndex = 0;
			clearIMUval(); // IMU値初期化
			optimalIndex = 0;

			clearXYcie(); // 座標計算変数初期化
			if (optimalTrace == BOOST_PATH_REPLAY || optimalTrace == BOOST_SHORTCUT)
			{
				pathFollowerReset();
			}

			if (initMSD)
			{
				if (!initMSD || !logRecoveryIsReady()) {
					autoStart = 0; setupFlags.start = 0;
					autoRunAbortSeries(&autoRunState);
					(void)finishBinaryLogStorage();
					showRunStartBlocked("BIN open failed");
					patternTrace = 103; break;
				}
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
			// motorCommandOutSynth(lineTraceOmegaFBCtrl.pwm, veloCtrl.pwm, 0);
			motorCommandOut(veloCtrlL.pwm,veloCtrlR.pwm);
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
			// motorCommandOutSynth(lineTraceOmegaFBCtrl.pwm, veloCtrl.pwm, 0);
			motorCommandOut(veloCtrlL.pwm,veloCtrlR.pwm);
		}
		else if (optimalTrace == BOOST_PATH_REPLAY || optimalTrace == BOOST_SHORTCUT)
		{
			// 経路再走行。通常はヨーレート制御、ロスト時はライン制御へ連続的に切り替える。
			boostSpeed = pathFollowerGetTargetSpeedMps();
			setTargetSpeed(boostSpeed);
			uint16_t pathBlend = pathFollowerGetPathBlendPermille();
			int32_t steeringPwm = ((int32_t)yawRateCtrl.pwm * pathBlend)
				+ ((int32_t)lineTraceOmegaFBCtrl.pwm * (1000 - pathBlend));
			steeringPwm /= 1000;
			motorCommandOutSynth(0, veloCtrl.pwm, steeringPwm);
		}

		// 通常走行はゴールマーカー、PATH系は中間延長点の経路弧長をゴールとする。
		bool pathGoalMode = (optimalTrace == BOOST_PATH_REPLAY || optimalTrace == BOOST_SHORTCUT);
		if (pathPolicyGoalRequest(pathGoalMode, SGmarker >= COUNT_GOAL,
			pathGoalMode && pathFollowerGoalReached()))
		{
			goalTime = cntRun;
			enc1 = 0;
			patternTrace = 101;
		}
		break;

	case 101:
		if (modeDSP && !ssd1306_IsDMARunning())
		{
			ssd1306_UpdateScreen_DMA();        // 停止していた画面更新を再開
		}
		// 停止速度まで減速
		if (enc1 >= encMM(50))
		{
			setTargetSpeed(0);
		}
		else
		{
			setTargetSpeed(tgtParam.stop);
		}
		motorCommandOutSynth(lineTraceCtrl.pwm, veloCtrl.pwm, 0);

		if (encCurrentN == 0)
		{
			patternTrace = 102;
		}
		break;

	case 102:
		setTargetSpeed(0);
		motorCommandOutSynth(0, 0, 0);

		int16_t savedLogNo = 0;
		bool logSaved = false;
		// 追加: 表示用の予測ログ番号はSD空でも落ちないようガード
		int16_t predictedLogNo = getLastLogNumber();
		if (modeLOG)
		{
			ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
			ssd1306_SetCursor(0, 25);
			ssd1306_printf(Font_11x18, "log %d", predictedLogNo);
			ssd1306_SetCursor(0, 45);
			ssd1306_printf(Font_11x18, "Writing");

			logSaved = endLog(); // ログ保存終了

			ssd1306_SetCursor(0, 45);
			ssd1306_printf(Font_11x18, logSaved ? "Written" : "Write failed");
		}

		if (autoStart > 0)
		{
			if (logSaved)
			{
				savedLogNo = getLastLogNumber();
			}
			bool primaryRouteValid = (autoRunCurrentPlan.requestedMode != AUTO_RUN_MODE_PRIMARY) ||
				logLastPrimaryRouteValid();
			if (!primaryRouteValid || !autoRunCompleteRun(&autoRunState, &autoRunCurrentPlan, savedLogNo,
				(emcStop == 0U), logSaved))
			{
				if (modeDSP)
				{
					ssd1306_FillRectangle(0, 15, 127, 63, Black);
					ssd1306_SetCursor(0, 25);
					ssd1306_printf(Font_11x18, "Auto stop");
					ssd1306_SetCursor(0, 45);
					if (!logSaved)
						ssd1306_printf(Font_11x18, "Log failed");
					else if (!primaryRouteValid)
						ssd1306_printf(Font_6x8, "Route reason %u", logLastPrimaryRouteReason());
					else
						ssd1306_printf(Font_11x18, "Run invalid");
				}
				autoStart = 0;
				autoStartAnalyze = 0;
				autoRunAbortSeries(&autoRunState);
			}

			if (autoStart == 0U)
			{
				(void)finishBinaryLogStorage();
				patternTrace = 103;
				break;
			}
			if (autoStart >= 5U)
			{
				autoStart = 0U;
				autoStartAnalyze = 0;
				autoRunAbortSeries(&autoRunState);
				bool storageReady = finishBinaryLogStorage();
				// 5走終了。bin保存状態を確認して終了を表示する。
				if (storageReady) {
				ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
				ssd1306_SetCursor(0, 15);
				ssd1306_printf(Font_11x18, "Auto run");
				ssd1306_SetCursor(0, 35);
				ssd1306_printf(Font_11x18, "Finish!");
				}

				patternTrace = 103;
				break;
			}
			else
			{
				autoStartAnalyze = savedLogNo;
				autoStart++;
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
			// 手動走行はbin保存完了後にタイムを表示する。
			bool storageReady = finishBinaryLogStorage();
			if (modeDSP && storageReady)
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
		motorCommandOutSynth(0, 0, 0);
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
		motorCommandOutSynth(0, veloCtrl.pwm, 0);
	}
	motorCommandOut(0, 0);
	
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

	bool storageReady = finishBinaryLogStorage();
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

	if (modeDSP && !storageReady) {
		ssd1306_SetCursor(0, 53);
		ssd1306_printf(Font_6x8, "BIN storage failed");
	}
	autoStart = 0U;
	autoStartAnalyze = 0;
	autoRunAbortSeries(&autoRunState);
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
	{
		countdown--;
		if (countdown == 5000)
		{
			IMU_StartCalibration(); // 100サンプルを20ms間隔で取得する
			calibrateMotorCurrent = true;
		}
	}
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
/////////////////////////////////////////////////////////////////////
// モジュール名 Control_ApplyMarkerCorrection_p
// 処理概要     マーカー補正値をスリップ補正後パルスへ反映する
// 引数         diff_p: 補正量[パルス]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void Control_ApplyMarkerCorrection_p(int32_t diff_p)
{
	encTotalOptimal -= diff_p;	// 走行距離の補正反映
}
int32_t Control_GetEncCurrentCorr_p(void)
{
	return encCurrentCorr_p;
}

///////////////////////////////////////////////////////////////////////////
// モジュール名 applyTargetSpeedStoredValue
// 処理概要     targetSpeeds.txtの保存値を速度パラメータへ反映する
// 引数         index:速度パラメータ番号, value:保存値
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
static void applyTargetSpeedStoredValue(int16_t index, int16_t value)
{
    const float converted = (float)value / SPEED_PARAM_STORAGE_SCALE;
    switch (index)
    {
#define SPEED_PARAM_APPLY(member, initial, label, step, minimum, maximum, unit) \
        case SPEED_PARAM_##member: tgtParam.member = converted; break;
        SPEED_PARAM_FIELDS(SPEED_PARAM_APPLY)
#undef SPEED_PARAM_APPLY
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
	int32_t dEncUse_p = (int32_t)encCurrentN;
	encCurrentCorr_p = dEncUse_p;

	// 外部変数
	enc1 += dEncUse_p;				// 通常トレース用
	encRightMarker += dEncUse_p;	// ゴールマーカ判定用
	encCurve += dEncUse_p;			// カーブ処理用
	encChangeGain += dEncUse_p;		// ゲイン変更用
	encTotalOptimal += dEncUse_p; 	// 2次走行用
	encLog += dEncUse_p;			// 一定距離ごとにログを保存する用
	encClick += encCurrentL;		// ホイールクリック用
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 writeTgtspeeds
// 処理概要     共通定義の保存順で19項目の速度パラメータをSDへ記録する
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void writeTgtspeeds(void)
{
    FIL fil;
    char fileName[30] = PATH_SETTING;
    strcat(fileName, FILENAME_TARGET_SPEED);
    strcat(fileName, ".txt");
    if (f_open(&fil, fileName, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
    {
#define SPEED_PARAM_FORMAT(member, initial, label, step, minimum, maximum, unit) SPEED_PARAM_STORAGE_FORMAT
#define SPEED_PARAM_VALUE(member, initial, label, step, minimum, maximum, unit) \
        , (int32_t)roundf(tgtParam.member * SPEED_PARAM_STORAGE_SCALE)
        f_printf(&fil, SPEED_PARAM_FIELDS(SPEED_PARAM_FORMAT) SPEED_PARAM_FIELDS(SPEED_PARAM_VALUE));
#undef SPEED_PARAM_FORMAT
#undef SPEED_PARAM_VALUE
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
	int values[TARGET_SPEED_PARAM_COUNT] = {0};
	int defaults[TARGET_SPEED_PARAM_COUNT];
	int minimums[TARGET_SPEED_PARAM_COUNT] = {
#define SPEED_PARAM_MIN(member, initial, label, step, minimum, maximum, unit) (int)((minimum) * SPEED_PARAM_STORAGE_SCALE),
        SPEED_PARAM_FIELDS(SPEED_PARAM_MIN)
#undef SPEED_PARAM_MIN
    };
    int maximums[TARGET_SPEED_PARAM_COUNT] = {
#define SPEED_PARAM_MAX(member, initial, label, step, minimum, maximum, unit) (int)((maximum) * SPEED_PARAM_STORAGE_SCALE),
        SPEED_PARAM_FIELDS(SPEED_PARAM_MAX)
#undef SPEED_PARAM_MAX
    };
	bool present[TARGET_SPEED_PARAM_COUNT] = {false};
	bool repair = false;
    const float defaultSpeedValues[TARGET_SPEED_PARAM_COUNT] = {
#define SPEED_PARAM_CURRENT(member, initial, label, step, minimum, maximum, unit) tgtParam.member,
        SPEED_PARAM_FIELDS(SPEED_PARAM_CURRENT)
#undef SPEED_PARAM_CURRENT
    };
	for (int16_t i = 0; i < TARGET_SPEED_PARAM_COUNT; i++)
	{
		defaults[i] = (int)lroundf(defaultSpeedValues[i] * SPEED_PARAM_STORAGE_SCALE);

	}

	// ファイル読み込み
	strcat(fileName, FILENAME_TARGET_SPEED);					  // ファイル名追加
	strcat(fileName, ".txt");									  // 拡張子追加
	fresult = f_open(&fil, fileName, FA_OPEN_EXISTING | FA_READ); // ファイルを開く

	if (fresult == FR_OK)
	{
		// speedParamの定義順で読み取る。途中で終端した場合はそこで打ち切る。
		for (int16_t i = 0; i < TARGET_SPEED_PARAM_COUNT; i++)
		{
			int16_t value = 0;
			if (f_gets(paramStr, SPEED_PARAM_STORAGE_FIELD_BYTES + 1U, &fil) == NULL)
			{
				repair = true;
				break;
			}
			if (sscanf(paramStr, SPEED_PARAM_STORAGE_READ_FORMAT, &value) != 1)
			{
				repair = true;
				break;
			}
			values[i] = (int)value;
			present[i] = true;
		}
		f_close(&fil);
	}
	else
	{
		repair = true;
	}

	repair = pathPolicyMergeSparseSettings(values, present, defaults,
		minimums, maximums, TARGET_SPEED_PARAM_COUNT) || repair;
	for (int16_t i = 0; i < TARGET_SPEED_PARAM_COUNT; i++)
	{
		applyTargetSpeedStoredValue(i, (int16_t)values[i]);
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

