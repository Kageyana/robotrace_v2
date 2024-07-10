#ifndef BMI088_H_
#define BMI088_H_

//====================================//
// インクルード
//====================================//
#include "main.h"
//====================================//
// シンボル定義
//====================================//
//  unit settings
#define ACCELELSB 5460.0F
#define GYROLSB 16.384F
#define MAGPLSB 16.0F

#define DEFF_TIME 0.001F
#define COEFF_DPD -1.00F
#define IMU_TRANSMIT true
#define IMU_STOP false

/*レジスタアドレス*/
#define REG_GYRO_CHIP_ID 0x00
#define REG_RATE_X_LSB 0x02
#define REG_RATE_X_MSB 0x03
#define REG_RATE_Y_LSB 0x04
#define REG_RATE_Y_MSB 0x05
#define REG_RATE_Z_LSB 0x06
#define REG_RATE_Z_MSB 0x07
#define REG_GYRO_BANDWISTH 0x10
#define REG_GYRO_SOFTRESET 0x14

#define REG_ACC_CHIP_ID 0x00
#define REG_ACC_X_LSB 0x12
#define REG_ACC_CONF 0x40
#define REG_ACC_RANGE 0x41
#define REG_ACC_PWR_CTRL 0x7D
#define REG_ACC_SOFTRESET 0x7E

#define REG_TEMP_MSB 0x22

/***************************************自動生成関数*************************************/
#define CSB1_RESET HAL_GPIO_WritePin(IMU_CSB1_GPIO_Port, IMU_CSB1_Pin, GPIO_PIN_RESET)
#define CSB1_SET HAL_GPIO_WritePin(IMU_CSB1_GPIO_Port, IMU_CSB1_Pin, GPIO_PIN_SET)
#define CSB2_RESET HAL_GPIO_WritePin(IMU_CSB2_GPIO_Port, IMU_CSB2_Pin, GPIO_PIN_RESET)
#define CSB2_SET HAL_GPIO_WritePin(IMU_CSB2_GPIO_Port, IMU_CSB2_Pin, GPIO_PIN_SET)
#define BMI088TRANSFER HAL_SPI_Transmit(&hspi2, &txData, txNum, 1000)
#define BMI088RECEIVE HAL_SPI_Receive(&hspi2, &rxData, rxNum, 1000)
#define BMI088RECEIVES HAL_SPI_Receive(&hspi2, rxData, rxNum, 1000)
/***************************************************************************************/
typedef struct
{
	float x;
	float y;
	float z;
} axis;
typedef struct
{
	axis accele;
	axis gyro;
	axis gyroTotal;
	axis angle;
	float temp;
	uint16_t id;
} IMUval;
//====================================//
// グローバル変数の宣言
//====================================//
extern IMUval BMI088val;
extern bool calibratIMU;
//====================================//
// プロトタイプ宣言
//====================================//
uint8_t BMI088ReadByteG(uint8_t reg);
void BMI088WriteByteG(uint8_t reg, uint8_t val);
uint8_t BMI088ReadByteA(uint8_t reg);
void BMI088WriteByteA(uint8_t reg, uint8_t val);
void BMI088ReadAxisDataG(uint8_t reg, uint8_t *rxData, uint8_t rxNum);
void BMI088ReadAxisDataA(uint8_t reg, uint8_t *rxData, uint8_t rxNum);
bool initBMI088(void);
void BMI088getGyro(void);
void BMI088getAccele(void);
void BMI088getTemp(void);
void calcDegrees(void);
void calibrationIMU(void);

#endif // BMI088_H_
