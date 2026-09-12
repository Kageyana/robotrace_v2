//====================================//
// インクルード
//====================================//
#include "imu_temp_log.h"
#include "BMI088.h"
#include "IMU.h"
#include "SDcard.h"
#include "encoder.h"
#include "lineSensor.h"
#include "markerSensor.h"
#include "motor.h"
#include "switch.h"
#include "fatfs.h"
#include <math.h>
#include <stdio.h>

#define IMU_TEMP_LOG_QUEUE_SIZE 8U
#define IMU_TEMP_LOG_MAX_FILE_NUMBER 99999U
#define IMU_TEMP_LED_BLINK_INTERVAL_MS 150U
#define IMU_TEMP_LED_BLINK_PHASES 4U
#define IMU_TEMP_LED_BRIGHTNESS 10

typedef enum
{
	IMU_TEMP_LED_NONE = 0,
	IMU_TEMP_LED_START,
	IMU_TEMP_LED_STOP,
	IMU_TEMP_LED_ERROR
} ImuTempLedNotification;

typedef struct
{
	uint32_t elapsed_ms;
	float temperature_C;
	float gyroZSum_dps;
	float gyroZSumSq_dps2;
	uint32_t sampleCount;
} ImuTempRecord;

static volatile bool chamberModeActive = false;
static volatile bool measurementActive = false;
static bool measurementSessionOpen = false;
static bool measurementStopRequested = false;
static volatile bool measurementErrorDetected = false;
static FIL measurementFile;
static ImuTempRecord measurementQueue[IMU_TEMP_LOG_QUEUE_SIZE];
static volatile uint8_t measurementQueueHead = 0U;
static volatile uint8_t measurementQueueTail = 0U;
static volatile uint8_t measurementQueueCount = 0U;
static volatile uint32_t measurementElapsedMs = 0U;
static volatile uint32_t measurementIntervalSamples = 0U;
static volatile float measurementIntervalSum = 0.0F;
static volatile float measurementIntervalSumSq = 0.0F;
static ImuTempLedNotification ledNotification = IMU_TEMP_LED_NONE;
static uint8_t ledNotificationPhase = 0U;
static uint32_t ledNotificationNextMs = 0U;
static uint8_t mainButtonShortPressPending = SW_NONE;

/////////////////////////////////////////////////////////////////////
// モジュール名 imuTempMeasurementHandleMainButtons
// 処理概要     左ボタン長押しで計測開始、右ボタン長押しで計測終了を処理する
// 引数         なし
// 戻り値       true: 計測操作中または恒温槽モード、false: 通常操作
/////////////////////////////////////////////////////////////////////
bool imuTempMeasurementHandleMainButtons(void)
{
	static uint8_t rawButtonBefore = SW_NONE;
	static uint8_t stableButton = SW_NONE;
	static uint8_t pressedButton = SW_NONE;
	static bool longPressHandled = false;
	static uint32_t rawButtonChangedAtMs = 0U;
	static uint32_t pressedAtMs = 0U;
	const uint32_t debounceMs = 20U;
	const uint32_t holdMs = 1000U;
	uint32_t now = HAL_GetTick();
	uint8_t rawButton = getSWMainTact();
	uint8_t mainButton;

	// 通常走行中は恒温槽モードへ移行しない。
	if (!chamberModeActive && patternTrace != 0U)
	{
		rawButtonBefore = rawButton;
		stableButton = rawButton;
		pressedButton = SW_NONE;
		longPressHandled = false;
		return false;
	}

	// 接点の瞬断で短押しや長押しが誤確定しないよう、入力を20 ms安定させる。
	if (rawButton != rawButtonBefore)
	{
		rawButtonBefore = rawButton;
		rawButtonChangedAtMs = now;
	}
	if (rawButton != stableButton)
	{
		if ((uint32_t)(now - rawButtonChangedAtMs) < debounceMs)
		{
			return rawButton != SW_NONE || stableButton != SW_NONE || chamberModeActive;
		}
		stableButton = rawButton;
	}
	mainButton = stableButton;

	// 誤操作防止のため、左右同時押しには機能を割り当てない。
	if (mainButton == SW_TACT_BOTH)
	{
		pressedButton = SW_NONE;
		longPressHandled = true;
		return true;
	}

	if (mainButton == SW_TACT_L || mainButton == SW_TACT_R)
	{
		if (pressedButton != mainButton)
		{
			pressedButton = mainButton;
			longPressHandled = false;
			pressedAtMs = now;
		}
		else if (!longPressHandled && (uint32_t)(now - pressedAtMs) >= holdMs)
		{
			if (mainButton == SW_TACT_L)
			{
				if (!measurementActive && !measurementSessionOpen)
				{
					imuTempMeasurementEnterChamberMode();
					(void)imuTempMeasurementStart();
				}
			}
			else if (chamberModeActive && measurementActive)
			{
				imuTempMeasurementStop();
			}
			longPressHandled = true;
		}
		return true;
	}

	// 専用モードへ入る前の短押しは、従来の走行選択として離した時に渡す。
	if (pressedButton != SW_NONE && !longPressHandled && !chamberModeActive)
	{
		mainButtonShortPressPending = pressedButton;
	}
	pressedButton = SW_NONE;
	longPressHandled = false;
	return chamberModeActive || measurementActive || measurementSessionOpen;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 imuTempMeasurementTakeMainButtonShortPress
// 処理概要     恒温槽操作で保留したmainボード短押しを1回だけ取得する
// 引数         なし
// 戻り値       SW_TACT_L、SW_TACT_R、またはSW_NONE
/////////////////////////////////////////////////////////////////////
uint8_t imuTempMeasurementTakeMainButtonShortPress(void)
{
	uint8_t mainButton = mainButtonShortPressPending;
	mainButtonShortPressPending = SW_NONE;
	return mainButton;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 setImuTempIndicatorColor
// 処理概要     mainボード上のフルカラーLEDを同一色で点灯する
// 引数         red: 赤輝度、green: 緑輝度、blue: 青輝度
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void setImuTempIndicatorColor(int red, int green, int blue)
{
	for (int i = 0; i < MAX_LED; i++)
	{
		setLED(i, red, green, blue);
	}
	sendLED();
}

/////////////////////////////////////////////////////////////////////
// モジュール名 startImuTempLedNotification
// 処理概要     IMU温度計測の開始・終了・異常点滅を開始する
// 引数         notification: 通知種別
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void startImuTempLedNotification(ImuTempLedNotification notification)
{
	ledNotification = notification;
	ledNotificationPhase = 0U;
	ledNotificationNextMs = HAL_GetTick();
}

/////////////////////////////////////////////////////////////////////
// モジュール名 updateImuTempLedNotification
// 処理概要     フルカラーLED通知をブロッキングせず更新する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void updateImuTempLedNotification(void)
{
	if (ledNotification == IMU_TEMP_LED_NONE || datasentflag)
	{
		return;
	}

	uint32_t now = HAL_GetTick();
	if ((int32_t)(now - ledNotificationNextMs) < 0)
	{
		return;
	}

	if (ledNotificationPhase >= IMU_TEMP_LED_BLINK_PHASES)
	{
		setImuTempIndicatorColor(0, 0, 0);
		ledNotification = IMU_TEMP_LED_NONE;
		return;
	}

	if ((ledNotificationPhase & 1U) == 0U)
	{
		if (ledNotification == IMU_TEMP_LED_START)
		{
			setImuTempIndicatorColor(0, IMU_TEMP_LED_BRIGHTNESS, 0);
		}
		else if (ledNotification == IMU_TEMP_LED_STOP)
		{
			setImuTempIndicatorColor(0, 0, IMU_TEMP_LED_BRIGHTNESS);
		}
		else
		{
			setImuTempIndicatorColor(IMU_TEMP_LED_BRIGHTNESS, 0, 0);
		}
	}
	else
	{
		setImuTempIndicatorColor(0, 0, 0);
	}
	ledNotificationPhase++;
	ledNotificationNextMs = now + IMU_TEMP_LED_BLINK_INTERVAL_MS;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 imuTempMeasurementEnterChamberMode
// 処理概要     恒温槽計測モードを電源再投入まで保持し、不要な周辺機能を停止する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void imuTempMeasurementEnterChamberMode(void)
{
	if (chamberModeActive)
	{
		return;
	}

	// 先に割り込み処理を専用経路へ切り替えてから、不要な周辺機能を停止する。
	chamberModeActive = true;
	motorCommandOut(0, 0);
	MotorFanPwmOut(0);
	powerLineSensors(0);
	powerMarkerSensors(0);

	(void)HAL_TIM_Base_Stop_IT(&htim3);
	(void)HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_3);
	(void)HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);
	(void)HAL_ADC_Stop_DMA(&hadc1);
	(void)HAL_ADC_Stop_DMA(&hadc2);
	(void)HAL_TIM_Encoder_Stop(&ENC_TIM_HANDLER_R, TIM_CHANNEL_ALL);
	(void)HAL_TIM_Encoder_Stop(&ENC_TIM_HANDLER_L, TIM_CHANNEL_ALL);
	(void)HAL_TIM_PWM_Stop(&MOTOR_TIM_HANDLER, MOTOR_TIM_CH_L);
	(void)HAL_TIM_PWM_Stop(&MOTOR_TIM_HANDLER, MOTOR_TIM_CH_R);
	(void)HAL_TIM_PWM_Stop(&MOTOR_TIM_HANDLER, MOTOR_SUCTION_TIM_CH);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 imuTempMeasurement1ms
// 処理概要     1ms周期でIMU温度係数計測値を集計する。SD書き込みは行わない。
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void imuTempMeasurement1ms(void)
{
	if (!measurementActive || !BMI088val.Initialized)
	{
		return;
	}

	float gyroZ = BMI088val.gyro.z;

	measurementElapsedMs++;
	measurementIntervalSamples++;
	measurementIntervalSum += gyroZ;
	measurementIntervalSumSq += gyroZ * gyroZ;

	if ((measurementElapsedMs % IMU_TEMP_LOG_INTERVAL_MS) == 0U)
	{
		uint32_t primask = __get_PRIMASK();
		__disable_irq();
		if (measurementQueueCount < IMU_TEMP_LOG_QUEUE_SIZE)
		{
			ImuTempRecord *record = &measurementQueue[measurementQueueHead];
			record->elapsed_ms = measurementElapsedMs;
			record->temperature_C = BMI088val.tempValid ? BMI088val.temp : BMI088_TEMP_INVALID_C;
			record->gyroZSum_dps = measurementIntervalSum;
			record->gyroZSumSq_dps2 = measurementIntervalSumSq;
			record->sampleCount = measurementIntervalSamples;
			measurementQueueHead = (uint8_t)((measurementQueueHead + 1U) % IMU_TEMP_LOG_QUEUE_SIZE);
			measurementQueueCount++;
		}
		else
		{
			measurementErrorDetected = true;
		}
		measurementIntervalSamples = 0U;
		measurementIntervalSum = 0.0F;
		measurementIntervalSumSq = 0.0F;
		__set_PRIMASK(primask);
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 imuTempMeasurementStart
// 処理概要     IMU温度係数計測ファイルを連番で作成し、計測を開始する
// 引数         なし
// 戻り値       true: 開始成功、false: 開始失敗
/////////////////////////////////////////////////////////////////////
bool imuTempMeasurementStart(void)
{
	if (measurementSessionOpen || !initMSD || !BMI088val.Initialized)
	{
		startImuTempLedNotification(IMU_TEMP_LED_ERROR);
		return false;
	}

	if (!sd_fatfs_lock(1000U))
	{
		startImuTempLedNotification(IMU_TEMP_LED_ERROR);
		return false;
	}

	FRESULT result = FR_DENIED;
	char fileName[48];
	for (uint32_t number = 1U; number <= IMU_TEMP_LOG_MAX_FILE_NUMBER; number++)
	{
		snprintf(fileName, sizeof(fileName), PATH_SETTING "imu_temp_%05lu.csv", (unsigned long)number);
		result = f_open(&measurementFile, fileName, FA_CREATE_NEW | FA_WRITE);
		if (result == FR_OK)
		{
			break;
		}
		if (result != FR_EXIST)
		{
			break;
		}
	}

	if (result == FR_OK)
	{
		int written = f_printf(&measurementFile,
			"elapsed_ms,temp_C,gyroZMean_dps,gyroZStd_dps\n");
		if (written < 0 || f_sync(&measurementFile) != FR_OK)
		{
			f_close(&measurementFile);
			result = FR_DISK_ERR;
		}
	}
	sd_fatfs_unlock();
	if (result != FR_OK)
	{
		startImuTempLedNotification(IMU_TEMP_LED_ERROR);
		return false;
	}

	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	measurementQueueHead = 0U;
	measurementQueueTail = 0U;
	measurementQueueCount = 0U;
	measurementElapsedMs = 0U;
	measurementIntervalSamples = 0U;
	measurementIntervalSum = 0.0F;
	measurementIntervalSumSq = 0.0F;
	measurementErrorDetected = false;
	measurementActive = true;
	__set_PRIMASK(primask);
	measurementSessionOpen = true;
	measurementStopRequested = false;
	startImuTempLedNotification(IMU_TEMP_LED_START);
	return true;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 imuTempMeasurementStop
// 処理概要     自然暖機計測の集計を停止し、保留レコードの書き込みを要求する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void imuTempMeasurementStop(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	measurementActive = false;
	__set_PRIMASK(primask);
	if (measurementSessionOpen)
	{
		measurementStopRequested = true;
	}
}

/////////////////////////////////////////////////////////////////////
// モジュール名 imuTempMeasurementTask
// 処理概要     メインループでIMU温度係数計測レコードをSDカードへ書き込む
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void imuTempMeasurementTask(void)
{
	updateImuTempLedNotification();
	if (!measurementSessionOpen || !sd_fatfs_try_lock())
	{
		return;
	}

	ImuTempRecord record;
	bool hasRecord = false;
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	if (measurementErrorDetected)
	{
		measurementActive = false;
		measurementStopRequested = true;
		measurementQueueHead = 0U;
		measurementQueueTail = 0U;
		measurementQueueCount = 0U;
	}
	if (measurementQueueCount > 0U)
	{
		record = measurementQueue[measurementQueueTail];
		measurementQueueTail = (uint8_t)((measurementQueueTail + 1U) % IMU_TEMP_LOG_QUEUE_SIZE);
		measurementQueueCount--;
		hasRecord = true;
	}
	__set_PRIMASK(primask);

	if (hasRecord)
	{
		float mean = 0.0F;
		float variance = 0.0F;
		if (record.sampleCount > 0U)
		{
			mean = record.gyroZSum_dps / (float)record.sampleCount;
			variance = (record.gyroZSumSq_dps2 / (float)record.sampleCount) - (mean * mean);
			if (variance < 0.0F)
			{
				variance = 0.0F;
			}
		}
		float standardDeviation = sqrtf(variance);
		char recordLine[96];
		int written = snprintf(recordLine, sizeof(recordLine), "%lu,%.3f,%.6f,%.6f\n",
			(unsigned long)record.elapsed_ms,
			record.temperature_C,
			mean,
			standardDeviation);
		bool writeOk = written > 0 && (size_t)written < sizeof(recordLine);
		if (writeOk)
		{
			UINT bytesWritten = 0U;
			FRESULT result = f_write(&measurementFile, recordLine, (UINT)written, &bytesWritten);
			writeOk = result == FR_OK && bytesWritten == (UINT)written;
		}
		if (writeOk)
		{
			writeOk = f_sync(&measurementFile) == FR_OK;
		}
		if (!writeOk)
		{
			primask = __get_PRIMASK();
			__disable_irq();
			measurementActive = false;
			measurementErrorDetected = true;
			measurementStopRequested = true;
			measurementQueueHead = 0U;
			measurementQueueTail = 0U;
			measurementQueueCount = 0U;
			__set_PRIMASK(primask);
		}
	}
	else if (measurementStopRequested)
	{
		bool closeError = f_sync(&measurementFile) != FR_OK;
		if (f_close(&measurementFile) != FR_OK)
		{
			closeError = true;
		}
		bool notifyError = measurementErrorDetected || closeError;
		measurementSessionOpen = false;
		measurementStopRequested = false;
		measurementErrorDetected = false;
		startImuTempLedNotification(notifyError ? IMU_TEMP_LED_ERROR : IMU_TEMP_LED_STOP);
	}

	sd_fatfs_unlock();
}

/////////////////////////////////////////////////////////////////////
// モジュール名 imuTempMeasurementIsChamberMode
// 処理概要     恒温槽計測専用モードの保持状態を取得する
// 引数         なし
// 戻り値       true: 専用モード、false: 通常モード
/////////////////////////////////////////////////////////////////////
bool imuTempMeasurementIsChamberMode(void)
{
	return chamberModeActive;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 imuTempMeasurementIsActive
// 処理概要     自然暖機計測の集計状態を取得する
// 引数         なし
// 戻り値       true: 集計中、false: 停止中
/////////////////////////////////////////////////////////////////////
bool imuTempMeasurementIsActive(void)
{
	return measurementActive;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 imuTempMeasurementSessionOpen
// 処理概要     自然暖機計測ファイルのオープン状態を取得する
// 引数         なし
// 戻り値       true: オープン中、false: クローズ済み
/////////////////////////////////////////////////////////////////////
bool imuTempMeasurementSessionOpen(void)
{
	return measurementSessionOpen;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 imuTempMeasurementOwnsIndicators
// 処理概要     IMU温度計測がmainボードLED表示を使用中か返す
// 引数         なし
// 戻り値       true: 計測または通知中、false: 非使用中
/////////////////////////////////////////////////////////////////////
bool imuTempMeasurementOwnsIndicators(void)
{
	return chamberModeActive || measurementActive || measurementSessionOpen || ledNotification != IMU_TEMP_LED_NONE;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 imuTempMeasurementElapsedMs
// 処理概要     自然暖機計測の経過時間を取得する
// 引数         なし
// 戻り値       経過時間[ms]
/////////////////////////////////////////////////////////////////////
uint32_t imuTempMeasurementElapsedMs(void)
{
	return measurementElapsedMs;
}
