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
#define ACCELELSB       100.0F
#define GYROLSB         16.384F
#define MAGPLSB         16.0F

#define DEFF_TIME       0.01F

/*************************************** 自動生成関数 *************************************/
#define CSB1_RESET 		        HAL_GPIO_WritePin(IMU_CSB1_GPIO_Port, IMU_CSB1_Pin, GPIO_PIN_RESET)
#define CSB1_SET   		        HAL_GPIO_WritePin(IMU_CSB1_GPIO_Port, IMU_CSB1_Pin, GPIO_PIN_SET)
#define CSB2_RESET 		        HAL_GPIO_WritePin(IMU_CSB2_GPIO_Port, IMU_CSB2_Pin, GPIO_PIN_RESET)
#define CSB2_SET   		        HAL_GPIO_WritePin(IMU_CSB2_GPIO_Port, IMU_CSB2_Pin, GPIO_PIN_SET)
#define BMI088TRANSFER	        HAL_SPI_Transmit(&hspi3,&txData,txNum,1000)
#define BMI088RECEIVE	        HAL_SPI_Receive(&hspi3,&rxData,rxNum,1000)
#define BMI088RECEIVES          HAL_SPI_Receive(&hspi3,rxData,rxNum,1000)
/*****************************************************************************************/
typedef struct {
    float x;
    float y;
    float z;
} axis;
typedef struct {
    axis accele;
    axis gyro;
    axis angle;
} IMUval;
//====================================//
// グローバル変数の宣言
//====================================//
extern IMUval 	BMI088val;
//====================================//
// プロトタイプ宣言
//====================================//
uint8_t BMI088ReadByteByte( uint8_t reg );
void    BMI088WriteByte( uint8_t reg, uint8_t val );
void    BMI088ReadAxisData(uint8_t reg, uint8_t *rxData );
bool    initBMI088(void);
void    BMI088getGyro(void);
void    calcDegrees(void);

#endif // BMI088_H_
