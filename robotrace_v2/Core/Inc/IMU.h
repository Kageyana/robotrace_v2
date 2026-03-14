#ifndef IMU_H_
#define IMU_H_
//====================================//
// インクルード
//====================================//
#include "BMI088.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
//====================================//
// シンボル定義
//====================================//
#define DEFF_TIME 0.001F				// 制御周期[s]（1ms）
#define COEFF_COMPFILTER 0.96F			// ジャイロと加速度のコンプリメンタリフィルタ係数（ジャイロの方が信頼できる場合は大きくする）
#define COEFF_DPD -0.996F				// ジャイロの符号と単位変換（DPS→RAD/s）を兼ねる係数（-1に近い値を調整して入れる）
#define USE_IMU_ROT_CENTER_CORRECTION	// ジャイロzを角加速度に変換して遠心加速度補正に使う（旋回中心からIMUまでの距離がある場合はON推奨）
#define IMU_OFFSET_X_M 0.0F 			// TODO: 旋回中心から IMU までのオフセット[m]。imuVal の座標系基準
#define IMU_OFFSET_Y_M 0.0236F 			// TODO: 旋回中心から IMU までのオフセット[m]。imuVal の座標系基準
#define GRAVITY_MPS2 9.80665F			// 重力加速度[m/s^2]
#define DEG2RAD (M_PI / 180.0F)			// deg→rad
#define RAD2DEG (180.0F / M_PI)			// rad→deg
#define DPS2RADS(dps) ((dps) * DEG2RAD)	// deg/s → rad/s
#define IMU_ALPHA_LPF_COEF 0.8F			// 角加速度のLPF係数（大きいほどノイズが減るが遅れる。0.8で約20ms程度の時定数）	
//====================================//
// グローバル変数の宣言
//====================================//
extern bool calibratIMU;		// IMUキャリブレーション中フラグ
extern volatile IMUval imuVal;	// IMUの実行時変数
extern float angleOffset[3];	// ジャイロオフセット[deg/s]
#ifdef USE_ACCELE
extern float acceleOffset[3];	// 加速度オフセット[g]
#endif
//====================================//
// プロトタイプ宣言
//====================================//
void calcDegrees(void);
void calcVelocity(void);
void clearIMUval(void);
void calibrationIMU(void);

#endif // IMU_H_
