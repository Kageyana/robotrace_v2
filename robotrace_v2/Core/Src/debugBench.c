#include "debugBench.h"

#ifdef DEBUG

#include "debug_bench_fixture_data.h"
#include "IMU.h"
#include "SDcard.h"
#include "control.h"
#include "courseAnalysis.h"
#include "main.h"
#include "motor.h"
#include "pathFollower.h"
#include <math.h>
#include <string.h>

#define DEBUG_BENCH_RESULT_MAGIC 0x42454E43UL
#define DEBUG_BENCH_RESULT_VERSION 2U
#define DEBUG_BENCH_RESET_MARKER_MAGIC 0x57444742UL
#define DEBUG_BENCH_STATE_IDLE 0U
#define DEBUG_BENCH_STATE_PREPARING 1U
#define DEBUG_BENCH_STATE_RUNNING 2U
#define DEBUG_BENCH_STATE_STOP_LATCHED 3U
#define DEBUG_BENCH_STATE_COMPLETE 4U
#define DEBUG_BENCH_STATE_RESET 5U

typedef struct
{
	uint32_t magic;
	uint32_t mode;
	uint32_t prescaler;
	uint32_t reload;
} DebugBenchResetMarker;

__attribute__((section(".noinit"), used)) static volatile DebugBenchResetMarker debugBenchResetMarker;

volatile uint32_t debugBenchRequestedMode = 0U;
volatile DebugBenchResult debugBenchResult;

static volatile uint32_t debugBenchState = DEBUG_BENCH_STATE_IDLE;
static volatile uint8_t debugBenchOutput = DEBUG_BENCH_OUTPUT_NORMAL;
static volatile uint32_t debugBenchElapsed = 0U;
static DebugBenchMode debugBenchActiveMode = DEBUG_BENCH_MODE_NONE;
static uint16_t debugBenchReplayIndex = 0U;
static bool debugBenchWatchdogEnabled = false;

/////////////////////////////////////////////////////////////////////
// モジュール名 debugBenchWatchdogRecoverAfterReset
// 処理概要     リセット後にIWDGを再起動して待機設定を反映し、測定結果を保持する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void debugBenchWatchdogRecoverAfterReset(void)
{
	uint32_t startTick;
	IWDG->KR = 0xAAAAU;
	RCC->CSR |= RCC_CSR_LSION;
	startTick = HAL_GetTick();
	while ((RCC->CSR & RCC_CSR_LSIRDY) == 0U)
	{
		if ((HAL_GetTick() - startTick) >= 500U)
		{
			debugBenchWatchdogEnabled = false;
			DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
			return;
		}
	}
	IWDG->KR = 0xCCCCU;
	IWDG->KR = 0xAAAAU;
	IWDG->KR = 0x5555U;
	startTick = HAL_GetTick();
	while ((IWDG->SR & (IWDG_SR_RVU | IWDG_SR_PVU)) != 0U)
	{
		if ((HAL_GetTick() - startTick) >= 250U)
		{
			IWDG->KR = 0xAAAAU;
			debugBenchWatchdogEnabled = true;
			DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
			return;
		}
	}
	IWDG->PR = 6U;
	IWDG->RLR = 4095U;
	startTick = HAL_GetTick();
	while ((IWDG->SR & (IWDG_SR_RVU | IWDG_SR_PVU)) != 0U)
	{
		if ((HAL_GetTick() - startTick) >= 250U)
		{
			IWDG->KR = 0xAAAAU;
			debugBenchWatchdogEnabled = true;
			DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
			return;
		}
	}
	if ((IWDG->PR & 7U) != 6U || (IWDG->RLR & 0x0FFFU) != 4095U)
	{
		IWDG->KR = 0xAAAAU;
		debugBenchWatchdogEnabled = true;
		DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
		return;
	}
	IWDG->KR = 0xAAAAU;
	debugBenchWatchdogEnabled = true;
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 debugBenchWatchdogSetStoppedTimeout
// 処理概要     モーター停止後のログ確定に備えてIWDGを最大猶予へ切り替える
// 引数         なし
// 戻り値       true:設定反映完了 false:リロード値更新待ちタイムアウト
/////////////////////////////////////////////////////////////////////
static bool debugBenchWatchdogSetStoppedTimeout(void)
{
	uint32_t startTick;
	volatile uint32_t timeout = 1000000U;
	if (!debugBenchWatchdogEnabled) return true;

	IWDG->KR = 0x5555U;
	IWDG->RLR = 0x0FFFU;
	startTick = HAL_GetTick();
	while ((IWDG->SR & IWDG_SR_RVU) != 0U && timeout > 0U)
	{
		if ((HAL_GetTick() - startTick) >= 250U) return false;
		timeout--;
	}
	if ((IWDG->SR & IWDG_SR_RVU) != 0U) return false;
	IWDG->KR = 0xAAAAU;
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 debugBenchCaptureResetCause
// 処理概要     起動直後にリセット要因を保存し、IWDGリセットを測定結果へ記録する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void debugBenchCaptureResetCause(void)
{
	uint32_t resetFlags = RCC->CSR;
	bool markerValid = (debugBenchResetMarker.magic == DEBUG_BENCH_RESET_MARKER_MAGIC &&
		debugBenchResetMarker.mode >= DEBUG_BENCH_MODE_PRIMARY &&
		debugBenchResetMarker.mode <= DEBUG_BENCH_MODE_SHORTCUT);
	debugBenchRequestedMode = 0U;
	debugBenchState = DEBUG_BENCH_STATE_IDLE;
	debugBenchOutput = DEBUG_BENCH_OUTPUT_NORMAL;
	debugBenchWatchdogEnabled = false;
	if ((resetFlags & RCC_CSR_IWDGRSTF) != 0U)
	{
		memset((void *)&debugBenchResult, 0, sizeof(debugBenchResult));
		debugBenchResult.magic = DEBUG_BENCH_RESULT_MAGIC;
		debugBenchResult.version = DEBUG_BENCH_RESULT_VERSION;
		debugBenchResult.state = DEBUG_BENCH_STATE_RESET;
		debugBenchResult.mode = markerValid ? debugBenchResetMarker.mode : DEBUG_BENCH_MODE_NONE;
		debugBenchResult.stopReason = DEBUG_BENCH_STOP_WATCHDOG_RESET;
		debugBenchResult.watchdogStarted = markerValid ? 1U : 0U;
		debugBenchResult.watchdogPrescaler = markerValid ? debugBenchResetMarker.prescaler : 0U;
		debugBenchResult.watchdogReload = markerValid ? debugBenchResetMarker.reload : 0U;
		debugBenchResult.resetCauseFlags = resetFlags;
		debugBenchWatchdogRecoverAfterReset();
	}
	debugBenchResetMarker.magic = 0U;
	RCC->CSR |= RCC_CSR_RMVF;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 debugBenchCanAcceptRequest
// 処理概要     通常停止状態かつ左右PWMがゼロの場合だけ測定要求を許可する
// 引数         なし
// 戻り値       true:要求許可 false:走行中または出力中
/////////////////////////////////////////////////////////////////////
static bool debugBenchCanAcceptRequest(void)
{
	return patternTrace == 0U && !modeLOG && autoStart == 0U &&
		setupFlags.start == 0U && setupFlags.clickStart == 0 &&
		motorpwmL == 0 && motorpwmR == 0 &&
		__HAL_TIM_GET_COMPARE(&MOTOR_TIM_HANDLER, MOTOR_TIM_CH_L) == 0U &&
		__HAL_TIM_GET_COMPARE(&MOTOR_TIM_HANDLER, MOTOR_TIM_CH_R) == 0U;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 debugBenchRejectRequest
// 処理概要     安全条件を満たさない測定要求を消費し、拒否理由を保存する
// 引数         mode:要求モード
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void debugBenchRejectRequest(uint32_t mode)
{
	memset((void *)&debugBenchResult, 0, sizeof(debugBenchResult));
	debugBenchResult.magic = DEBUG_BENCH_RESULT_MAGIC;
	debugBenchResult.version = DEBUG_BENCH_RESULT_VERSION;
	debugBenchResult.state = DEBUG_BENCH_STATE_COMPLETE;
	debugBenchResult.mode = mode;
	debugBenchResult.stopReason = DEBUG_BENCH_STOP_NOT_STOPPED;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 debugBenchWatchdogStart
// 処理概要     Debugベンチ中だけ独立ウォッチドッグを起動する
// 引数         なし
// 戻り値       true:起動成功 false:LSIまたは設定失敗
/////////////////////////////////////////////////////////////////////
static bool debugBenchWatchdogStart(void)
{
	uint32_t startTick;
	RCC->CSR |= RCC_CSR_LSION;
	startTick = HAL_GetTick();
	while ((RCC->CSR & RCC_CSR_LSIRDY) == 0U)
	{
		if ((HAL_GetTick() - startTick) >= 500U) return false;
	}

	DBGMCU->APB1FZ &= ~DBGMCU_APB1_FZ_DBG_IWDG_STOP;
	debugBenchResetMarker.mode = (uint32_t)debugBenchActiveMode;
	debugBenchResetMarker.prescaler = 6U;
	debugBenchResetMarker.reload = 255U;
	__DMB();
	debugBenchResetMarker.magic = DEBUG_BENCH_RESET_MARKER_MAGIC;
	__DMB();
	IWDG->KR = 0xCCCCU;
	debugBenchWatchdogEnabled = true;
	IWDG->KR = 0x5555U;
	IWDG->PR = 6U; // LSI / 256
	IWDG->RLR = 255U; // LSI周波数範囲で約1.4～3.9秒。
	startTick = HAL_GetTick();
	while ((IWDG->SR & (IWDG_SR_RVU | IWDG_SR_PVU)) != 0U)
	{
		if ((HAL_GetTick() - startTick) >= 250U) return false;
	}
	if ((IWDG->PR & 7U) != 6U || (IWDG->RLR & 0x0FFFU) != 255U) return false;
	IWDG->KR = 0xAAAAU;
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 debugBenchResultFromStorage
// 処理概要     SDcardモジュールの測定値をGDB参照用構造体へコピーする
// 引数         storage:SD測定結果, csvSaved:CSV作成結果
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void debugBenchResultFromStorage(const SdBenchStorageResult *storage, bool csvSaved)
{
	debugBenchResult.expectedRows = storage->expectedRows;
	debugBenchResult.csvRows = storage->csvRows;
	debugBenchResult.csvColumns = storage->csvColumns;
	debugBenchResult.csvColumnMismatchRows = storage->csvColumnMismatchRows;
	debugBenchResult.csvFirstBadRow = storage->csvFirstBadRow;
	debugBenchResult.csvFirstBadColumnCount = storage->csvFirstBadColumnCount;
	debugBenchResult.csvValidated = storage->csvValidated;
	debugBenchResult.firstCntlog = storage->firstCntlog;
	debugBenchResult.lastCntlog = storage->lastCntlog;
	debugBenchResult.cntlogMonotonic = storage->cntlogMonotonic;
	debugBenchResult.writeCount = storage->writeCount;
	debugBenchResult.writeAlignmentErrors = storage->writeAlignmentErrors;
	debugBenchResult.writeMetricOverflow = storage->writeMetricOverflow;
	debugBenchResult.maxWriteWaitMs = storage->maxWriteWaitMs;
	debugBenchResult.lastWritePosition = storage->lastWritePosition;
	debugBenchResult.lastWriteLength = storage->lastWriteLength;
	debugBenchResult.lastWriteWritten = storage->lastWriteWritten;
	debugBenchResult.lastWriteResult = storage->lastWriteResult;
	debugBenchResult.logOverflowCount = storage->logOverflowCount;
	debugBenchResult.sdWriteFailed = storage->writeFailed;
	debugBenchResult.state = DEBUG_BENCH_STATE_COMPLETE;
	debugBenchResult.magic = DEBUG_BENCH_RESULT_MAGIC;
	debugBenchResult.version = DEBUG_BENCH_RESULT_VERSION;
	debugBenchResult.watchdogStarted = debugBenchWatchdogEnabled ? 1U : 0U;
	debugBenchResetMarker.magic = 0U;
	debugBenchResult.success =
		(csvSaved && debugBenchResult.stopReason == DEBUG_BENCH_STOP_TRACE_END &&
		debugBenchResult.csvValidated != 0U && debugBenchResult.csvRows >= 128U &&
		debugBenchResult.cntlogMonotonic != 0U && debugBenchResult.writeCount > 0U &&
		debugBenchResult.multiSectorWriteCalls > 0U && debugBenchResult.maxSdWriteSectors > 1U &&
		debugBenchResult.writeAlignmentErrors == 0U &&
		debugBenchResult.writeMetricOverflow == 0U && debugBenchResult.pathLostCount == 0U &&
		debugBenchResult.oneMsPeriodOverruns == 0U && debugBenchResult.logOverflowCount == 0U &&
		debugBenchResult.sdWriteFailed == 0U && debugBenchResult.watchdogStarted != 0U) ? 1U : 0U;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 debugBenchComplete
// 処理概要     停止後にログ変換と検証を行い、GDB停止地点へ到達する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void debugBenchComplete(void)
{
	SdBenchStorageResult storage = {0};
	SdBenchRuntimeMetrics runtimeMetrics = {
		.maxInterruptCycles = debugBenchResult.maxInterruptCycles,
		.maxSyntheticPoseCycles = debugBenchResult.maxSyntheticPoseCycles,
		.maxRunEquivalentInterruptCycles = debugBenchResult.maxRunEquivalentInterruptCycles,
		.maxPeriodCycles = debugBenchResult.maxPeriodCycles,
		.oneMsPeriodOverruns = debugBenchResult.oneMsPeriodOverruns,
		.pathLostCount = debugBenchResult.pathLostCount,
		.lineBrightJudgeCount = debugBenchResult.lineBrightJudgeCount,
		.lineUnbrightJudgeCount = debugBenchResult.lineUnbrightJudgeCount,
		.overSpeedJudgeCount = debugBenchResult.overSpeedJudgeCount,
		.peakEncoderPulsesPerMs = debugBenchResult.peakEncoderPulsesPerMs,
		.sdWriteCallCount = debugBenchResult.sdWriteCallCount,
		.maxSdWriteSectors = debugBenchResult.maxSdWriteSectors,
		.multiSectorWriteCalls = debugBenchResult.multiSectorWriteCalls};
	uint32_t stopReason = debugBenchResult.stopReason;
	uint32_t elapsedMs = debugBenchElapsed;
	debugBenchOutput = DEBUG_BENCH_OUTPUT_STOPPED;
	modeLOG = false;
	patternTrace = 0U;
	motorCommandOut(0, 0);
	if (debugBenchWatchdogEnabled)
	{
		if (!debugBenchWatchdogSetStoppedTimeout())
		{
			stopReason = DEBUG_BENCH_STOP_WATCHDOG;
			debugBenchResult.stopReason = (uint32_t)stopReason;
		}
		DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
	}
	sdBenchSetRuntimeMetrics(&runtimeMetrics);
	bool csvSaved = sdBenchFinish(stopReason, elapsedMs, &storage);
	debugBenchResult.elapsedMs = elapsedMs;
	debugBenchResultFromStorage(&storage, csvSaved);
	debugBenchCompletedBreakpoint();
	for (;;)
	{
		debugBenchWatchdogRefresh();
		__WFI();
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 debugBenchFinishWithoutRun
// 処理概要     開始前エラーを結果へ設定し、可能なら空ログも確定する
// 引数         reason:停止理由, storageOpen:一時ログファイルが作成済みか
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void debugBenchFinishWithoutRun(DebugBenchStopReason reason, bool storageOpen)
{
	debugBenchOutput = DEBUG_BENCH_OUTPUT_STOPPED;
	motorCommandOut(0, 0);
	debugBenchResult.stopReason = (uint32_t)reason;
	debugBenchResult.elapsedMs = 0U;
	debugBenchResult.mode = (uint32_t)debugBenchActiveMode;
	if (storageOpen)
	{
		debugBenchComplete();
	}
	debugBenchResult.state = DEBUG_BENCH_STATE_COMPLETE;
	debugBenchResult.magic = DEBUG_BENCH_RESULT_MAGIC;
	debugBenchResult.version = DEBUG_BENCH_RESULT_VERSION;
	debugBenchResult.watchdogStarted = debugBenchWatchdogEnabled ? 1U : 0U;
	debugBenchResult.success = 0U;
	debugBenchResetMarker.magic = 0U;
	if (debugBenchWatchdogEnabled)
	{
		DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
	}
	debugBenchState = DEBUG_BENCH_STATE_COMPLETE;
	debugBenchCompletedBreakpoint();
	for (;;)
	{
		debugBenchWatchdogRefresh();
		__WFI();
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 debugBenchSetFixtureSettings
// 処理概要     12420.csvに記録されたPATH制御設定をDebug実行へ反映する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void debugBenchSetFixtureSettings(void)
{
	shortcutSettings = (ShortcutSettings){
		DEBUG_BENCH_SETTING_MAX_LEVEL,
		DEBUG_BENCH_SETTING_LOOKAHEAD_BASE_MM,
		DEBUG_BENCH_SETTING_LOOKAHEAD_PER_MPS_MM,
		DEBUG_BENCH_SETTING_KLATERAL_X100,
		DEBUG_BENCH_SETTING_KHEADING_X100,
		DEBUG_BENCH_SETTING_LINE_ALPHA_X1000,
		DEBUG_BENCH_SETTING_LINE_THETA_GAIN_X1E9};
}

/////////////////////////////////////////////////////////////////////
// モジュール名 debugBenchStart
// 処理概要     GDB指定モードの走行状態と専用ログを準備して固定PWMで開始する
// 引数         mode:測定モード
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void debugBenchStart(DebugBenchMode mode)
{
	const char *csvName;
	const char *metricsName;
	const char *modeName;
	uint32_t primask;
	memset((void *)&debugBenchResult, 0, sizeof(debugBenchResult));
	debugBenchOutput = DEBUG_BENCH_OUTPUT_STOPPED;
	debugBenchResetMarker.magic = 0U;
	debugBenchState = DEBUG_BENCH_STATE_PREPARING;
	debugBenchActiveMode = mode;
	debugBenchResult.mode = (uint32_t)mode;
	debugBenchResult.state = DEBUG_BENCH_STATE_PREPARING;
	debugBenchResult.magic = DEBUG_BENCH_RESULT_MAGIC;
	debugBenchResult.version = DEBUG_BENCH_RESULT_VERSION;
	debugBenchElapsed = 0U;
	debugBenchReplayIndex = 0U;
	if (mode != DEBUG_BENCH_MODE_PRIMARY && mode != DEBUG_BENCH_MODE_PATH_REPLAY &&
		mode != DEBUG_BENCH_MODE_SHORTCUT)
	{
		debugBenchFinishWithoutRun(DEBUG_BENCH_STOP_INVALID_MODE, false);
	}
	if (!initIMU)
	{
		debugBenchFinishWithoutRun(DEBUG_BENCH_STOP_SENSOR_INIT, false);
	}
	if (!IMU_CalibrationReady() || calibratIMU)
	{
		uint32_t calibrationStartTick;
		IMU_StartCalibration();
		calibrationStartTick = HAL_GetTick();
		while (calibratIMU && (HAL_GetTick() - calibrationStartTick) < 5000U)
		{
			HAL_Delay(1U);
		}
		if (calibratIMU || !IMU_CalibrationReady())
		{
			debugBenchFinishWithoutRun(DEBUG_BENCH_STOP_SENSOR_INIT, false);
		}
	}
	if (!initMSD)
	{
		debugBenchFinishWithoutRun(DEBUG_BENCH_STOP_SD_OPEN, false);
	}

	if (mode != DEBUG_BENCH_MODE_PRIMARY)
	{
		debugBenchSetFixtureSettings();
		uint8_t shortcutLevel = (mode == DEBUG_BENCH_MODE_SHORTCUT) ? 1U : 0U;
		if (!pathFollowerLoadDebugBenchRoute(debugBenchSourceRoute,
			(uint16_t)DEBUG_BENCH_ROUTE_COUNT, shortcutLevel, 12418))
		{
			debugBenchFinishWithoutRun(DEBUG_BENCH_STOP_ROUTE_SETUP, false);
		}
	}

#if LOG_SCHEMA_PROFILE_LIGHT
	const char *profilePrefix = "N";
#else
	const char *profilePrefix = "D";
#endif
	char outputCsv[13];
	char metricsCsv[13];
	if (mode == DEBUG_BENCH_MODE_PRIMARY)
	{
		csvName = (profilePrefix[0] == 'N') ? "NPRIM.CSV" : "DPRIM.CSV";
		metricsName = (profilePrefix[0] == 'N') ? "NPRIMW.CSV" : "DPRIMW.CSV";
		modeName = "PRIMARY";
	}
	else if (mode == DEBUG_BENCH_MODE_PATH_REPLAY)
	{
		csvName = (profilePrefix[0] == 'N') ? "NPATH.CSV" : "DPATH.CSV";
		metricsName = (profilePrefix[0] == 'N') ? "NPATHW.CSV" : "DPATHW.CSV";
		modeName = "PATH_REPLAY";
	}
	else
	{
		csvName = (profilePrefix[0] == 'N') ? "NSHORT.CSV" : "DSHORT.CSV";
		metricsName = (profilePrefix[0] == 'N') ? "NSHORTW.CSV" : "DSHORTW.CSV";
		modeName = "SHORTCUT";
	}
	(void)snprintf(outputCsv, sizeof(outputCsv), "%s", csvName);
	(void)snprintf(metricsCsv, sizeof(metricsCsv), "%s", metricsName);
	if (!sdBenchStart(outputCsv, metricsCsv, modeName))
	{
		debugBenchFinishWithoutRun(DEBUG_BENCH_STOP_SD_OPEN, false);
	}
	if (!debugBenchWatchdogStart())
	{
		debugBenchResult.watchdogStarted = 0U;
		debugBenchFinishWithoutRun(DEBUG_BENCH_STOP_WATCHDOG, true);
	}
	debugBenchResult.watchdogStarted = 1U;
	debugBenchResult.watchdogPrescaler = 6U;
	debugBenchResult.watchdogReload = 255U;
	debugBenchPreStartBreakpoint();
	primask = __get_PRIMASK();
	__disable_irq();
	debugBenchElapsed = 0U;
	debugBenchOutput = DEBUG_BENCH_OUTPUT_FIXED_100;
	debugBenchState = DEBUG_BENCH_STATE_RUNNING;
	debugBenchResult.state = DEBUG_BENCH_STATE_RUNNING;
	controlDebugBenchStart((uint8_t)mode);
	motorCommandOut(100, 100);
	__set_PRIMASK(primask);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 debugBenchMainLoop
// 処理概要     Debug専用のGDB制御、SD書込み、停止後確定処理を実行する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
bool debugBenchMainLoop(void)
{
	if (debugBenchState == DEBUG_BENCH_STATE_IDLE)
	{
		debugBenchWatchdogRefresh();
		if (debugBenchRequestedMode != 0U)
		{
			uint32_t request = debugBenchRequestedMode;
			debugBenchRequestedMode = 0U;
			if (!debugBenchCanAcceptRequest())
			{
				debugBenchRejectRequest(request);
				return false;
			}
			debugBenchStart((DebugBenchMode)request);
			return true;
		}
		return false;
	}
	if (debugBenchState == DEBUG_BENCH_STATE_RUNNING)
	{
		debugBenchWatchdogRefresh();
		logWriteTask();
		return true;
	}
	if (debugBenchState == DEBUG_BENCH_STATE_STOP_LATCHED)
	{
		debugBenchComplete();
	}
	if (debugBenchState == DEBUG_BENCH_STATE_COMPLETE)
	{
		debugBenchWatchdogRefresh();
		__WFI();
	}
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 debugBenchIsRunning
// 処理概要     Debugベンチが割り込み処理を含めて走行中か返す
// 引数         なし
// 戻り値       true:走行中 false:停止中
/////////////////////////////////////////////////////////////////////
bool debugBenchIsRunning(void)
{
	return debugBenchState == DEBUG_BENCH_STATE_RUNNING;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 debugBenchAdvance1ms
// 処理概要     走行時間を1ms進めて再生処理へ返す
// 引数         なし
// 戻り値       経過時間[ms]
/////////////////////////////////////////////////////////////////////
uint32_t debugBenchAdvance1ms(void)
{
	if (debugBenchState == DEBUG_BENCH_STATE_RUNNING)
	{
		debugBenchElapsed++;
		debugBenchResult.elapsedMs = debugBenchElapsed;
	}
	return debugBenchElapsed;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 debugBenchElapsedMs
// 処理概要     Debugベンチの経過時間を返す
// 引数         なし
// 戻り値       経過時間[ms]
/////////////////////////////////////////////////////////////////////
uint32_t debugBenchElapsedMs(void)
{
	return debugBenchElapsed;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 debugBenchMode
// 処理概要     現在のDebugベンチ測定モードを返す
// 引数         なし
// 戻り値       モード番号
/////////////////////////////////////////////////////////////////////
uint8_t debugBenchMode(void)
{
	return (uint8_t)debugBenchActiveMode;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 debugBenchMotorOutputMode
// 処理概要     モータ出力を100固定、停止、通常のいずれにするか返す
// 引数         なし
// 戻り値       出力モード
/////////////////////////////////////////////////////////////////////
uint8_t debugBenchMotorOutputMode(void)
{
	return debugBenchOutput;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 debugBenchApplyPlaybackHeading
// 処理概要     12420.csvの時刻・軌跡から進捗を補間し、経路位置と記録方位を反映する
// 引数         elapsedMs:走行開始後の時間[ms]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void debugBenchApplyPlaybackHeading(uint32_t elapsedMs)
{
	if (!debugBenchIsRunning() || debugBenchActiveMode == DEBUG_BENCH_MODE_PRIMARY) return;
	while ((uint32_t)(debugBenchReplayIndex + 1U) < DEBUG_BENCH_POSE_COUNT &&
		debugBenchReplayPose[debugBenchReplayIndex + 1U].time_ms < elapsedMs)
	{
		debugBenchReplayIndex++;
	}
	uint16_t nextIndex = (uint16_t)(debugBenchReplayIndex + 1U);
	if (nextIndex >= DEBUG_BENCH_POSE_COUNT) nextIndex = debugBenchReplayIndex;
	const DebugBenchPoseSample *before = &debugBenchReplayPose[debugBenchReplayIndex];
	const DebugBenchPoseSample *after = &debugBenchReplayPose[nextIndex];
	float headingRatio = 0.0F;
	if (after->time_ms > before->time_ms)
	{
		headingRatio = (float)(elapsedMs - before->time_ms) /
			(float)(after->time_ms - before->time_ms);
		if (headingRatio < 0.0F) headingRatio = 0.0F;
		if (headingRatio > 1.0F) headingRatio = 1.0F;
	}
	float beforeHeading = (float)before->heading_cdeg * 0.01F;
	float afterHeading = (float)after->heading_cdeg * 0.01F;
	float headingDelta = afterHeading - beforeHeading;
	while (headingDelta > 180.0F) headingDelta -= 360.0F;
	while (headingDelta < -180.0F) headingDelta += 360.0F;
	float headingDeg = beforeHeading + (headingDelta * headingRatio);
	float replayArcMm = (float)before->arc_mm +
		(((float)after->arc_mm - (float)before->arc_mm) * headingRatio);
	float replayTotalArcMm = (float)debugBenchReplayPose[DEBUG_BENCH_POSE_COUNT - 1U].arc_mm;
	uint16_t progressPermille = (replayTotalArcMm > 0.0F) ?
		(uint16_t)lroundf((replayArcMm * 1000.0F) / replayTotalArcMm) : 0U;
	pathFollowerSetDebugBenchProgressPose(progressPermille, headingDeg);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 debugBenchRecordSyntheticPoseCycles
// 処理概要     擬似位置の反映に使った最大CPUサイクル数を保存する
// 引数         poseCycles:擬似位置処理のCPUサイクル数
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void debugBenchRecordSyntheticPoseCycles(uint32_t poseCycles)
{
	if (!debugBenchIsRunning()) return;
	if (poseCycles > debugBenchResult.maxSyntheticPoseCycles)
	{
		debugBenchResult.maxSyntheticPoseCycles = poseCycles;
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 debugBenchRecordJudgements
// 処理概要     ベンチ中に成立したライン明暗・速度判定を診断用に記録する
// 引数         lineBright:明判定, lineUnbright:暗判定, overSpeed:速度判定
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void debugBenchRecordJudgements(bool lineBright, bool lineUnbright, bool overSpeed)
{
	if (!debugBenchIsRunning()) return;
	if (lineBright) debugBenchResult.lineBrightJudgeCount++;
	if (lineUnbright) debugBenchResult.lineUnbrightJudgeCount++;
	if (overSpeed) debugBenchResult.overSpeedJudgeCount++;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 debugBenchRecordPeriod
// 処理概要     1ms割り込み間隔の最大値と周期超過数を記録する
// 引数         periodCycles:前回割り込み開始からのCPUサイクル数
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void debugBenchRecordPeriod(uint32_t periodCycles)
{
	if (!debugBenchIsRunning()) return;
	if (periodCycles > debugBenchResult.maxPeriodCycles)
	{
		debugBenchResult.maxPeriodCycles = periodCycles;
	}
	uint32_t expected = SystemCoreClock / 1000U;
	if (periodCycles > expected + 100U)
	{
		debugBenchResult.oneMsPeriodOverruns++;
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 debugBenchRecordInterruptTime
// 処理概要     1ms割り込み全体と擬似位置処理を除いた時間を記録する
// 引数         interruptCycles:割り込み開始からのCPUサイクル数
//              syntheticPoseCycles:擬似位置処理のCPUサイクル数
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void debugBenchRecordInterruptTime(uint32_t interruptCycles, uint32_t syntheticPoseCycles)
{
	if (debugBenchState != DEBUG_BENCH_STATE_RUNNING &&
		debugBenchState != DEBUG_BENCH_STATE_STOP_LATCHED) return;
	if (interruptCycles > debugBenchResult.maxInterruptCycles)
	{
		debugBenchResult.maxInterruptCycles = interruptCycles;
	}
	if (syntheticPoseCycles > debugBenchResult.maxSyntheticPoseCycles)
	{
		debugBenchResult.maxSyntheticPoseCycles = syntheticPoseCycles;
	}
	uint32_t equivalentCycles = (interruptCycles > syntheticPoseCycles) ?
		(interruptCycles - syntheticPoseCycles) : 0U;
	if (equivalentCycles > debugBenchResult.maxRunEquivalentInterruptCycles)
	{
		debugBenchResult.maxRunEquivalentInterruptCycles = equivalentCycles;
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 debugBenchLatchStop
// 処理概要     割り込み側で停止理由とPWM停止を確定する
// 引数         reason:停止理由
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void debugBenchLatchStop(DebugBenchStopReason reason)
{
	if (debugBenchState != DEBUG_BENCH_STATE_RUNNING) return;
	debugBenchResult.stopReason = (uint32_t)reason;
	debugBenchResult.elapsedMs = debugBenchElapsed;
	debugBenchState = DEBUG_BENCH_STATE_STOP_LATCHED;
	debugBenchOutput = DEBUG_BENCH_OUTPUT_STOPPED;
	patternTrace = 0U;
	modeLOG = false;
	motorCommandOut(0, 0);
	if (debugBenchWatchdogEnabled)
	{
		DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 debugBenchWatchdogRefresh
// 処理概要     Debugベンチで有効化した独立ウォッチドッグを更新する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void debugBenchWatchdogRefresh(void)
{
	if (debugBenchWatchdogEnabled) IWDG->KR = 0xAAAAU;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 debugBenchCompletedBreakpoint
// 処理概要     測定完了後だけGDBが停止できる固定ブレークポイントを提供する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
__attribute__((noinline, used)) void debugBenchCompletedBreakpoint(void)
{
	__NOP();
}

/////////////////////////////////////////////////////////////////////
// モジュール名 debugBenchPreStartBreakpoint
// 処理概要     IWDG有効後かつPWMゼロの開始直前をGDBへ公開する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
__attribute__((noinline, used)) void debugBenchPreStartBreakpoint(void)
{
	__NOP();
}

#endif /* DEBUG */
