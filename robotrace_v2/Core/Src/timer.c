//====================================//
// インクルード
//====================================//
#include "timer.h"
#include "BMI088.h"
#include "PIDcontrol.h"
#include "battery.h"
#include "control.h"
#include "lineSensor.h"
#include "pathFollower.h"
#include "runGuard.h"
#include <math.h>
#include <stdint.h>
#include <string.h>
#define STRAIGHT_STATE_THRESHOLD_MM	70	// 直線判定の距離閾値[mm]
#ifndef ROBOTRACE_ENABLE_SLIP_UPDATE
#define ROBOTRACE_ENABLE_SLIP_UPDATE 0
#endif
//====================================//
// グローバル変数の宣
//====================================//
int32_t cnt5 = 0;
int32_t cnt10 = 0;
int32_t encPulse5ms = 0; // 5ms間のエンコーダパルスを累積
float bootTime;
static volatile bool logWriteReq = false;
static volatile RunTimingDiagnostics runTiming;
static volatile uint32_t runTimingTick;
static volatile uint32_t lastLineUpdateTick;
static volatile uint32_t startMarkerTick;
static volatile int32_t startMarkerPulse;
static volatile bool runTimingActive;
static volatile bool lineUpdateSeen;
static volatile bool startMarkerSeen;

/////////////////////////////////////////////////////////////////////
// モジュール名 Timer_ResetRunDiagnostics
// 処理概要     走行開始時にRAM上の周期診断値を初期化する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void Timer_ResetRunDiagnostics(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	memset((void *)&runTiming, 0, sizeof(runTiming));
	runTimingTick = 0U;
	lastLineUpdateTick = 0U;
	startMarkerTick = 0U;
	startMarkerPulse = 0;
	lineUpdateSeen = false;
	startMarkerSeen = false;
	runTimingActive = true;
	__set_PRIMASK(primask);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 Timer_StopRunDiagnostics
// 処理概要     停止後のログ変換中に診断値が変わらないよう計測を止める
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void Timer_StopRunDiagnostics(void)
{
	runTimingActive = false;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 Timer_GetRunDiagnostics
// 処理概要     割り込みとの競合を避けて診断値を取得する
// 引数         なし
// 戻り値       走行周期の診断値
/////////////////////////////////////////////////////////////////////
RunTimingDiagnostics Timer_GetRunDiagnostics(void)
{
	RunTimingDiagnostics copy;
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	memcpy(&copy, (const void *)&runTiming, sizeof(copy));
	__set_PRIMASK(primask);
	return copy;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 Timer_NotifyLineUpdate
// 処理概要     新しい点灯・消灯組のライン値が完成した時刻を記録する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void Timer_NotifyLineUpdate(void)
{
	if (!runTimingActive) return;
	uint32_t tick = runTimingTick;
	uint32_t interval = tick - lastLineUpdateTick;
	if (interval > runTiming.maxLineUpdateIntervalMs)
		runTiming.maxLineUpdateIntervalMs = interval;
	lastLineUpdateTick = tick;
	lineUpdateSeen = true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 Timer_NotifyAdcPhaseMismatch
// 処理概要     LED位相が変わったADCサンプルの破棄回数を記録する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void Timer_NotifyAdcPhaseMismatch(void)
{
	if (runTimingActive) runTiming.adcPhaseMismatchCount++;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 Timer_NotifyStartMarkerDetected
// 処理概要     右スタートマーカーを最初に検出した時刻と距離を記録する
// 引数         pulse: 検出時の生エンコーダ累積値[pulse]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void Timer_NotifyStartMarkerDetected(int32_t pulse)
{
	if (!runTimingActive || startMarkerSeen) return;
	startMarkerTick = runTimingTick;
	startMarkerPulse = pulse;
	startMarkerSeen = true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 Timer_NotifyStartReferenceReset
// 処理概要     検出から距離原点リセットまでの遅れを確定する
// 引数         pulse: リセット直前の生エンコーダ累積値[pulse]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void Timer_NotifyStartReferenceReset(int32_t pulse)
{
	if (!runTimingActive || !startMarkerSeen) return;
	runTiming.startResetDelayMs = runTimingTick - startMarkerTick;
	runTiming.startResetPulseDelta = pulse - startMarkerPulse;
	runTiming.startResetMeasured = 1U;
	startMarkerSeen = false;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 Timer_IsLineObservationFresh
// 処理概要     最新ライン値が3制御周期未満か判定する
// 引数         なし
// 戻り値       true: 新鮮または診断未開始 false: 3周期以上未更新
/////////////////////////////////////////////////////////////////////
bool Timer_IsLineObservationFresh(void)
{
	return RunGuard_LineFresh(runTimingActive, lineUpdateSeen,
		runTimingTick, lastLineUpdateTick);
}
/////////////////////////////////////////////////////////////////////
// モジュール名 Timer_RecordIsrDuration
// 処理概要     1ms処理の実行時間と周期超過を記録する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void Timer_RecordIsrDuration(void)
{
	if (!runTimingActive) return;
	uint32_t cycles = DWT->CYCCNT;
	if (cycles > runTiming.maxIsrCycles) runTiming.maxIsrCycles = cycles;
	if (cycles > SystemCoreClock / 1000U) runTiming.isrOverrunCount++;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 Interrupt1ms
// 処理概要     タイマー割り込み(1ms)
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void Interrupt1ms(void)
{
	static bool calibrationWasActive = false;
	bool imuReadFailed = false;

	// Interrupt 1ms
	cntRun++;
	cnt5++;
	cnt10++;
	cntLog++;

	// 割り込み時間計測
	uint32_t freqCount = getCycleCounter();
	resetCycleCounter();
	if (runTimingActive)
	{
		runTimingTick++;
		uint32_t age = runTimingTick - lastLineUpdateTick;
		if (age > runTiming.maxLineUpdateIntervalMs)
			runTiming.maxLineUpdateIntervalMs = age;
		if (age >= 3U) runTiming.lineStaleCycleCount++;
	}
	bootTime = getTimeMs(freqCount);
	updateBatteryVoltage();

	// Encoder
	getEncoder();
	if (emcStop == STOP_IMU_READ)
	{
		motorCommandOut(0, 0);
		Timer_RecordIsrDuration();
		return;
	}

	// IMU処理
	if (initIMU)
	{
		bool imuUpdateReady = false;
		uint32_t imuReadStart = DWT->CYCCNT;
		if (!calibratIMU)
		{
			bool gyroOk = BMI088getGyro();	// 角速度取得
			bool accelOk = gyroOk && BMI088getAccele();	// 加速度取得
			if (runTimingActive)
			{
				uint32_t cycles = DWT->CYCCNT - imuReadStart;
				if (cycles > runTiming.maxImuReadCycles) runTiming.maxImuReadCycles = cycles;
			}
			if (RunGuard_ImuReadUsable(gyroOk, accelOk))
			{
				calcDegrees();		// コンプリメンタリフィルタで角度算出
				calcVelocity();		// 加速度から速度算出
				imuUpdateReady = true;
			}
			else imuReadFailed = true;
			// motorControlYawRate();	// 角速度制御
			// motorControlYaw();		// 角度制御
		}
		else
		{
			calibrationIMU();
			if (calibrationWasActive && !calibratIMU)
			{
				Control_RequestDistanceFusionReset();
				// 校正完了直後も新しい補正済みIMU値を距離更新へ渡す。
				bool gyroOk = BMI088getGyro();
				bool accelOk = gyroOk && BMI088getAccele();
				if (runTimingActive)
				{
					uint32_t cycles = DWT->CYCCNT - imuReadStart;
					if (cycles > runTiming.maxImuReadCycles) runTiming.maxImuReadCycles = cycles;
				}
				if (RunGuard_ImuReadUsable(gyroOk, accelOk))
				{
					calcDegrees();
					calcVelocity();
					imuUpdateReady = true;
				}
				else imuReadFailed = true;
			}
		}
		if (ROBOTRACE_ENABLE_SLIP_UPDATE && imuUpdateReady && patternTrace >= 12 && patternTrace < 100)
		{
			updateSlipDetection(); // スリップ検出（Δv比率とフラグ更新を1msで実行）
		}
		calibrationWasActive = calibratIMU;
	}
	else
	{
		calibrationWasActive = false;
	}
	if (imuReadFailed)
	{
		if (runTimingActive) runTiming.imuReadErrorCount++;
		if (patternTrace >= 11 && patternTrace <= 101)
		{
			emcStop = STOP_IMU_READ;
			motorCommandOut(0, 0);
			Timer_RecordIsrDuration();
			return; // 失敗したIMU値を距離融合・姿勢・モーター制御へ渡さない
		}
	}

	// 校正完了・走行開始の要求は、IMU補正後かつカルマン更新前に実行する。
	Control_ProcessDistanceFusionReset();

	// IMUの読出し・補正後にカルマン距離と距離系カウンタを更新する。
	setEncoderVal();
	if (modeLOG) logAccumulateMotion1ms(imuVal.gyro.z, encCurrentL, encCurrentR);
	// 走行終了要求は、最後の距離系カウンタを反映した同じ1ms処理で実行する。
	Control_ProcessDistanceFusionReset();
	encPulse5ms += encCurrentN; // 5ms間のエンコーダパルスを累積

	// 経路モードは平均速度PIDとヨーレートPIDを使用する。
	bool pathModeActive = (optimalTrace == BOOST_PATH_REPLAY || optimalTrace == BOOST_SHORTCUT);
	if (pathModeActive && patternTrace >= 12 && patternTrace < 100)
	{
		pathFollowerUpdatePose1ms(Control_GetEncCurrentCorr_p(), imuVal.gyro.z);
		motorControlTraceOmegaFB();
		motorControlSpeed();
		log_targetAngularVelocity = (int32_t)targetAngularVelocity;
		motorControlYawRate();
	}
	else if(patternTrace < 12 || patternTrace > 100)
	{
		// スタート直後とゴール後は通常のライン制御
		motorControlTrace();
		motorControlSpeed();
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
		if (cntEmcStopLineSensorUnbright() && !pathModeActive)
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

		// 一定距離ごとに処理
		if (encLog >= encMM(CALCDISTANCE_SHORTCUT))
		{
			// ROC(曲率半径)計算
			rocrun = calcROC(encCurrentN, BMI088val.gyro.z, (float)cntLog / 1000);
			if (rocrun >= 700.0F) // 直線判断
			{
				straightMeter += CALCDISTANCE_SHORTCUT; // 距離積算
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

			if (modeLOG)
			{
				// CALCDISTANCEごとにログを保存
				writeLogBufferPuts();
				courseMarkerLog = 0; // ログ用マーカー状態をリセット
				straightMarkerPendingLog = 0;
				encLog = 0;	// ログ用エンコーダパルスをリセット
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
		// 経路投影と目標更新は5ms周期で実行し、1ms割り込み負荷を抑える。
		if (pathModeActive && patternTrace >= 12 && patternTrace < 100)
		{
			pathFollowerUpdateTarget5ms();
		}
		encPulse5ms = 0; // 累積値をリセット
		break;
	case 2:
		if (initIMU)
		{
			if (!calibratIMU)
			{
				if (!BMI088getTemp())
				{
					imuTempCorrectionEnabled = false;
					if (runTimingActive) runTiming.imuReadErrorCount++;
					if (patternTrace >= 11 && patternTrace <= 101)
					{
						emcStop = STOP_IMU_READ;
						motorCommandOut(0, 0);
					}
				}
				else if (!BMI088val.tempValid || !isfinite(BMI088val.temp))
				{
					// 温度コードが無効な場合は走行を継続しつつ温度補正だけ停止する。
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
		if(!calibrateMotorCurrent)
		{
			getMotorCurrent();
		}
		else
		{
			calibrationMotorCurrent();
		}
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
	Timer_RecordIsrDuration();
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
/////////////////////////////////////////////////////////////////////
// モジュール名 Interrupt300ns
// 処理概要     タイマー割り込み(300ns)
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void Interrupt300ns(void)
{
	// Interrupt 300ns
	// sendColorData();
}
