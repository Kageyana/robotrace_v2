#ifndef IMU_TEMP_LOG_H_
#define IMU_TEMP_LOG_H_

#include <stdbool.h>
#include <stdint.h>

#define IMU_TEMP_LOG_INTERVAL_MS 2000U

void imuTempMeasurement1ms(void);
void imuTempMeasurementTask(void);
bool imuTempMeasurementHandleMainButtons(void);
uint8_t imuTempMeasurementTakeMainButtonShortPress(void);
void imuTempMeasurementEnterChamberMode(void);
bool imuTempMeasurementStart(void);
void imuTempMeasurementStop(void);
bool imuTempMeasurementIsChamberMode(void);
bool imuTempMeasurementIsActive(void);
bool imuTempMeasurementSessionOpen(void);
bool imuTempMeasurementOwnsIndicators(void);
uint32_t imuTempMeasurementElapsedMs(void);

#endif // IMU_TEMP_LOG_H_
