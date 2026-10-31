//====================================//
// インクルード
//====================================//
#include "timer.h"
#include "slipDetection.h"
#include "BMI088.h"
#include "PIDcontrol.h"
#include "battery.h"
#include "control.h"
#include "lineSensor.h"
#include "pathFollower.h"
#include "SDcard.h"
#include "encoder.h"
#include "setup.h"
#include <math.h>
#include <stdint.h>
//====================================//
// グローバル変数の宣言、定義
//====================================//
int32_t cnt5 = 0;
int32_t cnt10 = 0;
int32_t encPulse5ms = 0; // 5ms間のエンコーダパルスを累積
float bootTime;
static volatile bool logWriteReq = false;
static uint32_t slipGyroTime, slipAccelTime, slipCurrentTime, slipCurrentSequence;
static bool slipWasRunning, slipCurrentCalibrated;
//====================================//
// プロトタイプ宣言
//====================================//

/////////////////////////////////////////////////////////////////////
// モジュール名 updateSlipCurrentSample
// 処理概要     10ms電流取得の更新番号と校正完了を診断用に追跡する
// 引数         nowMs: 制御時刻[ms]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void updateSlipCurrentSample(uint32_t nowMs)
{
    if (calibrateMotorCurrent) {
        slipCurrentCalibrated = false;
        calibrationMotorCurrent();
        if (!calibrateMotorCurrent) {
            slipCurrentCalibrated = true;
            slipCurrentSequence = 0U; // 校正後の新しい取得まで有効にしない。
        }
    } else {
        getMotorCurrent();
        slipCurrentTime = nowMs;
        ++slipCurrentSequence;
    }
}
#if defined(ROBOTRACE_ISR_TIMING) && ROBOTRACE_ISR_TIMING
volatile IsrTimingStats isrTimingStats[ISR_TIMING_MODE_COUNT] = {0};
volatile IsrTimingStats isrIrqTimingStats[ISR_TIMING_MODE_COUNT] = {0};
volatile uint32_t isrIrqPendingTicks[ISR_TIMING_MODE_COUNT] = {0};
volatile uint16_t isrTimingIdleLoadUs = 0U;
volatile IsrTimingStats isrIdleIrqTimingStats = {0};
volatile uint32_t isrIdleIrqPendingTicks = 0U;
static uint32_t timingPreviousCounter;
static uint32_t timingIrqStart, timingIrqMode;
static bool timingIrqRunning;
static bool timingIrqIdleTest;
_Static_assert(BOOST_NONE == 0 && BOOST_DISTANCE == 2 &&
	BOOST_SHORTCUT == 3 && BOOST_PATH_REPLAY == 4 && BOOST_PATH_CURVE == 5, "ISR timing mode order changed");
/////////////////////////////////////////////////////////////////////
// モジュール名 recordTimingStats
// 処理概要     走行モード別に割り込み処理のサイクル数を整数で集計する
// 引数         stats: 集計先, cycles: 計測区間のサイクル数
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void recordTimingStats(volatile IsrTimingStats *stats, uint32_t cycles)
{
	// サンプル数上限では集計を止め、合計と平均の整合を維持する。
	if (stats->samples == UINT32_MAX) return;
	if (stats->samples == 0U || cycles < stats->minCycles) stats->minCycles = cycles;
	if (cycles > stats->maxCycles) stats->maxCycles = cycles;
	stats->totalCycles += cycles;
	stats->samples++;
	if (cycles >= SystemCoreClock / 1000U) stats->over1ms++;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 timerTimingResetClock
// 処理概要     DWT初期化後に周期間隔の基準を合わせる。集計値は保持する。
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void timerTimingResetClock(void)
{
	timingPreviousCounter = getCycleCounter();
}
/////////////////////////////////////////////////////////////////////
// モジュール名 timerTimingIrqBegin
// 処理概要     TIM6のHAL呼出し前の時刻と走行モードを保持する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void timerTimingIrqBegin(void)
{
	timingIrqStart = getCycleCounter();
	timingIrqMode = optimalTrace;
	timingIrqRunning = patternTrace >= 12U && patternTrace < 100U &&
		timingIrqMode < ISR_TIMING_MODE_COUNT && isSupportedRunMode((uint8_t)timingIrqMode);
	timingIrqIdleTest = patternTrace == 0U && setupFlags.start == 0U &&
		autoStart == 0U && modeLOG == 0U && isrTimingIdleLoadUs != 0U &&
		isrTimingIdleLoadUs <= ISR_TIMING_IDLE_LOAD_MAX_US;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 timerTimingIrqEnd
// 処理概要     HAL処理後のIRQ区間と次周期の保留を整数で集計する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void timerTimingIrqEnd(void)
{
	const uint32_t end = getCycleCounter();
	if (timingIrqIdleTest)
	{
		recordTimingStats(&isrIdleIrqTimingStats, end - timingIrqStart);
		if ((htim6.Instance->SR & TIM_FLAG_UPDATE) != 0U && isrIdleIrqPendingTicks < UINT32_MAX)
			++isrIdleIrqPendingTicks;
	}
	if (!timingIrqRunning) return;
	const bool pending = (htim6.Instance->SR & TIM_FLAG_UPDATE) != 0U;
	recordTimingStats(&isrIrqTimingStats[timingIrqMode], end - timingIrqStart);
	if (pending && isrIrqPendingTicks[timingIrqMode] < UINT32_MAX)
		++isrIrqPendingTicks[timingIrqMode];
}
/////////////////////////////////////////////////////////////////////
// モジュール名 timerTimingIdleLoad
// 処理概要     静止試験中だけ上限付きのCPU負荷を加え、取得IRQとの競合を測る
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void timerTimingIdleLoad(void)
{
	if (!timingIrqIdleTest || patternTrace != 0U || setupFlags.start != 0U ||
		autoStart != 0U || modeLOG != 0U) return;
	uint32_t loadUs = isrTimingIdleLoadUs;
	if (loadUs > ISR_TIMING_IDLE_LOAD_MAX_US) loadUs = ISR_TIMING_IDLE_LOAD_MAX_US;
	const uint32_t cycles = loadUs * (SystemCoreClock / 1000000U);
	const uint32_t start = getCycleCounter();
	uint32_t now;
	do
	{
		now = getCycleCounter();
	} while ((uint32_t)(now - start) < cycles);
}
#endif
/////////////////////////////////////////////////////////////////////
// モジュール名 Interrupt1ms
// 処理概要     タイマー割り込み(1ms)
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void Interrupt1ms(void)
{
	static int32_t logDistanceRemainder_p = 0;
	if (patternTrace < 12 || patternTrace >= 100)
	{
		logDistanceRemainder_p = 0;
	}

	// Interrupt 1ms
	cntRun++;
	cnt5++;
	cnt10++;
	cntLog++;

	// 割り込み時間計測
	uint32_t freqCount = getCycleCounter();
#if defined(ROBOTRACE_ISR_TIMING) && ROBOTRACE_ISR_TIMING
	// IRQ入口/出口を同じ時計で測るため、計測版ではDWTを連続運転する。
	const uint32_t currentCounter = freqCount;
	freqCount -= timingPreviousCounter;
	timingPreviousCounter = currentCounter;
	uint32_t isrTimingStart = getCycleCounter();
	// 途中でゴール・停止状態へ遷移しても、入口のモードへ最後の周期を記録する。
	bool isrTimingRunning = (patternTrace >= 12 && patternTrace < 100);
	uint32_t isrTimingMode = optimalTrace;
#else
	resetCycleCounter();
#endif
	bootTime = getTimeMs(freqCount);
	beginLineSensorControlFrame1ms();
	updateBatteryVoltage();

	// Encoder
	getEncoder();
	setEncoderVal();
	encPulse5ms += encCurrentN; // 5ms間のエンコーダパルスを累積

	// IMU処理
	bool slipGyroOk = false, slipAccelOk = false;
	if (initIMU)
	{
		if (!calibratIMU)
		{
			slipGyroOk = BMI088getGyro();	// 角速度取得
			if (slipGyroOk) slipGyroTime = cntRun;
			slipAccelOk = BMI088getAccele();	// 加速度取得
			if (slipAccelOk) slipAccelTime = cntRun;
			calcDegrees();		// コンプリメンタリフィルタで角度算出
			calcVelocity();		// 加速度から速度算出
			// motorControlYawRate();	// 角速度制御
		}
		else
		{
			calibrationIMU();
		}
	}

	// V2だけを1msごとに更新し、採用した縦横判定をログと次走SLIP計画へ渡す。
	ImuTangentialSample tangential = IMU_GetTangentialSample();
	SlipDetectionInput slipInput = {
		.pulseL = encCurrentL, .pulseR = encCurrentR,
		.pwmL = motorpwmL, .pwmR = motorpwmR,
		.forwardMps2 = IMU_GetLinearAccelerationYMps2(),
		.lateralMps2 = IMU_GetLinearAccelerationXMps2(), .yawDps = imuVal.gyro.z,
		.currentL = motorCurrentL, .currentR = motorCurrentR,
		.timeMs = cntRun, .gyroTimeMs = slipGyroTime, .accelTimeMs = slipAccelTime,
		.currentTimeMs = slipCurrentTime, .currentSequence = slipCurrentSequence,
		.running = patternTrace >= 12 && patternTrace < 100,
		.calibrating = calibratIMU, .gyroOk = slipGyroOk, .accelOk = slipAccelOk,
		.currentOk = slipCurrentCalibrated && !calibrateMotorCurrent && slipCurrentSequence != 0U,
		.yawAlphaRadps2 = tangential.alphaRadps2,
		.tangentialAppliedXMps2 = tangential.appliedXMps2, .tangentialOk = tangential.valid
	};
	if (slipInput.running && !slipWasRunning) SlipDetection_Reset(&slipV2);
	slipWasRunning = slipInput.running;
	SlipDetection_Update(&slipV2, &slipInput);

	// 経路モードは平均速度PIDとヨーレートPIDを使用する。
	bool pathModeActive = isPathRunMode(optimalTrace);
	const bool pathSteeringActive = pathModeActive && patternTrace >= 12 && patternTrace < 100;
	const PathFollowerState pathState = pathFollowerGetStatus();
	monitorLineSensorUpdate1ms(!pathSteeringActive || pathState == PATH_STATE_LINE_FALLBACK || pathState == PATH_STATE_REJOIN);
#if defined(ROBOTRACE_ISR_TIMING) && ROBOTRACE_ISR_TIMING
	recordLineSensorTiming1ms((uint8_t)isrTimingMode, patternTrace, isrTimingRunning);
#endif
	if (pathModeActive && patternTrace >= 12 && patternTrace < 100)
	{
		pathFollowerUpdatePose1ms(Control_GetEncCurrentCorr_p(), imuVal.gyro.z);
		// 最新の自己位置から経路と補正を更新し、同じ1ms周期の制御へ反映する。
		pathFollowerUpdateTarget1ms();
		motorControlTraceOmegaFB();
		motorControlSpeed();
		log_targetAngularVelocity = (int32_t)targetAngularVelocity;
		motorControlYawRate();
	}
	else if(patternTrace < 12 || patternTrace > 100)
	{
		// スタート直後とゴール後は通常のライン制御
		motorControlSpeed();
		if (patternTrace == 0 && autoStartCalibrationTurning && initIMU && !calibratIMU)
		{
			setupCalibrationTurn1ms(); // 校正旋回・復帰追従・停止出力を1ms周期で切り替える
		}
		else
		{
			motorControlTrace();
		}
	}
	else
	{
		// ライン追従＋左右独立速度制御
		motorControlTraceOmegaFB();

		int16_t delta = lineTraceOmegaFBCtrl.pwm;
		int16_t targetL = (int16_t)targetSpeed + delta;
		int16_t targetR = (int16_t)targetSpeed - delta;
		const int16_t TARGET_MAX = (int16_t)PULSE_MILLIMETER * 4;
		if (targetL > TARGET_MAX) targetL = TARGET_MAX;
		if (targetL < -TARGET_MAX) targetL = -TARGET_MAX;
		if (targetR > TARGET_MAX) targetR = TARGET_MAX;
		if (targetR < -TARGET_MAX) targetR = -TARGET_MAX;

		motorControlSpeedLR(targetL, targetR);
	}
	if (patternTrace > 10 && patternTrace < 100)
	{
		// 走行中に処理
		// 緊急停止処理
		// if (cntEmcStopAngleX()) emcStop = STOP_ANGLE_X;
		// if (cntEmcStopAngleY()) emcStop = STOP_ANGLE_Y;
		if (cntEmcStopEncStop())
			emcStop = STOP_ENCODER_STOP;
		if (cntEmcStopLineSensorBright() && !pathModeActive)
			emcStop = STOP_LINESENSOR_BRIGHT;
		// 完全逸脱は自己位置の推定状態に関係なく、PATH系を含む全モードで停止する。
		if (cntEmcStopLineSensorUnbright())
			emcStop = STOP_LINESENSOR_UNBRIGHT;
		if (judgeOverSpeed())
			emcStop = STOP_OVERSPEED;
		if (pathModeActive && pathFollowerGetStatus() == PATH_STATE_LOCALIZATION_LOST)
			emcStop = STOP_LOCALIZATION;

		changeGain();		// ROCに応じてゲインを切り替える
		checkCrossLine();	// クロスライン確認

		courseMarker = checkMarker();	// マーカー検知
		checkStartGoalMarker();			// ゴールマーカー処理
		processMarkerEvent();	      	// マーカー関連処理を関数に委譲
		if(courseMarker != 0)
			courseMarkerLog = courseMarker; // ログ用にマーカー状態を保存

		// 目標距離間隔ごとに処理し、1 msあたり最大1レコードに制限する。
		const int32_t logDistanceThreshold_p = encMM((int32_t)LOG_DISTANCE_MM);
		if (encLog >= logDistanceThreshold_p)
		{
			int32_t logDistanceStep_p = encLog - logDistanceRemainder_p;
			if (logDistanceStep_p < 0)
			{
				logDistanceStep_p = 0;
			}
			int32_t logDistanceStep_mm = (int32_t)lroundf((float)logDistanceStep_p / PULSE_MILLIMETER);

			// ROC(曲率半径)計算
			rocrun = calcROC(encCurrentN, BMI088val.gyro.z, (float)cntLog / 1000);
			if (rocrun >= 700.0F) // 直線判断
			{
				straightMeter += logDistanceStep_mm; // 実際に進んだ距離を積算
			}
			else
			{
				straightMeter = 0;
			}

			if (straightMeter >= STRAIGHT_STATE_THRESHOLD_MM) // 直線が閾値以上のとき
			{
				if (!straightState)
				{
					straightMarkerPending = true;
				}
				straightState = true;
			}
			else
			{
				straightState = false;
			}

			encLog -= logDistanceThreshold_p;
			if (encLog >= logDistanceThreshold_p)
			{
				encLog = 0; // 1ms更新の範囲を超えた距離分は捨て、記録負荷を抑える
			}
			logDistanceRemainder_p = encLog;

			if (modeLOG)
			{
				// 目標距離間隔ごとにログを保存
				if (writeLogBufferPuts()) SlipDetection_ResetInterval(&slipV2);
				courseMarkerLog = 0; // ログ用マーカー状態をリセット
				straightMarkerPendingLog = 0;
				cntLog = 0;
			}
		}
	}
	else
	{
		// 走行前に処理
		getSwitches();	// スイッチの入力を取得
		countDown();	// カウントダウン処理
		setupCount();	// セットアップ用タイマを更新
		wheelClick();	// セットアップ用ホイールクリック処理
	}

	switch (cnt5)
	{
	case 1:
		encPulse5ms = 0; // 累積値をリセット
		break;
	case 2:
		if (initIMU)
		{
			if (!calibratIMU)
			{
				if (!BMI088getTemp() || !BMI088val.tempValid || !isfinite(BMI088val.temp))
				{
					imuTempCorrectionEnabled = false;
				}
				imuVal.temp = BMI088val.temp;
			}
		}
		break;
	case 3:
		break;
	case 5:
		cnt5 = 0;
		break;
	}

	switch (cnt10)
	{
	case 1:
		updateSlipCurrentSample(cntRun);
		break;
	case 2:
		getVref();
		break;
	case 9:
		break;
	case 10:
		cnt10 = 0;
		break;
	}
#if defined(ROBOTRACE_ISR_TIMING) && ROBOTRACE_ISR_TIMING
	timerTimingIdleLoad();
#endif
	endLineSensorControlFrame1ms();
#if defined(ROBOTRACE_ISR_TIMING) && ROBOTRACE_ISR_TIMING
	uint32_t isrTimingEnd = getCycleCounter();
	if (isrTimingRunning && isrTimingMode < ISR_TIMING_MODE_COUNT && isSupportedRunMode((uint8_t)isrTimingMode))
		recordTimingStats(&isrTimingStats[isrTimingMode], isrTimingEnd - isrTimingStart);
#endif
}
/////////////////////////////////////////////////////////////////////
// モジュール名 Interrupt500us
// 処理概要     タイマー割り込み(0.5ms)
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void Interrupt500us(void)
{
	static uint8_t logWriteReqDivider = 0;
	logWriteReqDivider++;
	if (logWriteReqDivider >= 10U) // 0.5ms x 10 = 5ms
	{
		logWriteReqDivider = 0;
		logWriteReq = true;
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 logWriteTask
// 処理概要     SD書き込み要求処理
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void logWriteTask(void)
{
        if (logWriteReq)
        {
                if (sd_is_analysis_active())
                {
                        logWriteReq = false;
                        return;
                }
                writeLogPuts();        // SD?????????????
                logWriteReq = false;
        }
}
