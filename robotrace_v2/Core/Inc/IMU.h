#ifndef IMU_H_
#define IMU_H_
//====================================//
// インクルード
//====================================//
#include "BMI088.h"
#include <stdbool.h>
#include <stdint.h>
//====================================//
// シンボル定義
//====================================//
#define DEFF_TIME 0.001F
#define COEFF_COMPFILTER 0.96F
#define COEFF_DPD -0.996F
//====================================//
// グローバル変数の宣言
//====================================//
extern bool calibratIMU;
extern volatile IMUval imuVal;
extern float angleOffset[3];
#ifdef USE_ACCELE
extern float acceleOffset[3];
#endif
//====================================//
// プロトタイプ宣言
//====================================//
void calcDegrees(void);
void calcVelocity(void);
void clearIMUval(void);
void calibrationIMU(void);

#endif // IMU_H_
