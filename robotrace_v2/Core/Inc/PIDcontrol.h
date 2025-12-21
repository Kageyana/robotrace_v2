#ifndef LINETRACE_H_
#define LINETRACE_H_
//====================================//
// インクルード
//====================================//
#include "main.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
//====================================//
// シンボル定義
//====================================//

// PID制御用ゲイン定義(ライントレース 角速度FBなし)
#define KP1		28
#define KI1		0
#define KD1		200

// ライントレース用センサ重み係数（中央から外側に向かって適用）
#define TRACE_WEIGHT_CENTER		1.0f
#define TRACE_WEIGHT_INNER		1.01f
#define TRACE_WEIGHT_MIDDLE		1.02f
#define TRACE_WEIGHT_OUTER		1.03f
#define TRACE_WEIGHT_FAR		1.04f

#define TRACE_CROSSLINE_TH		2000	// クロスライン検出閾値
#define TRACE_CROSSLINE_DISTANCE 60		// クロスライン付近でゲインを変更する距離[mm]

// PID制御用ゲイン定義(速度制御)
#define KP2		10
#define KI2		0
#define KD2		0

// PID制御用ゲイン定義(角速度制御)
#define KP3		38
#define KI3		1
#define KD3		15

// PID制御用ゲイン定義(角度制御)
#define KP4		6
#define KI4		0
#define KD4		10

// PID制御用ゲイン定義(距離制御)
#define KP5		100
#define KI5		1
#define KD5		5

// PID制御用ゲイン定義(ライントレース 角速度FBあり)
#define KP6		7
#define KI6		0
#define KD6		25

// 速度フィードフォワード関連定義
#define SPEED_FEEDFORWARD_GAIN_DEFAULT          150      // Crr×1000 の初期値(例:0.020)
#define SPEED_FEEDFORWARD_GEAR_RATIO            2.0f    // ギア比 G
#define SPEED_FEEDFORWARD_EFFICIENCY            0.90f   // ギア効率 η
#define SPEED_FEEDFORWARD_KV_RPM_PER_V          2710.0f // Kv[rpm/V]
#define SPEED_FEEDFORWARD_S_RPM_PER_MNM         2240.0f // S[rpm/mNm]
#define SPEED_FEEDFORWARD_WHEEL_DIAMETER_MM     24.0f   // ホイール径 D[mm]
#define SPEED_FEEDFORWARD_MASS_KG               0.123f  // 車体質量 m[kg]
#define SPEED_FEEDFORWARD_GRAVITY               9.80665f        // 重力加速度 g[m/s^2]
#define SPEED_FEEDFORWARD_SIGN_DEADBAND_MM_S    0.001f  // sgn(v) 判定のデッドバンド[mm/s]
#define SPEED_FEEDFORWARD_PWM_MAX_DEFAULT       1000    // PWM_MAX のデフォルト値
#define SPEED_FEEDFORWARD_CRR_SCALE             0.001f  // speedFeedForwardGain→Crr 変換係数

typedef struct {
	uint8_t *name;
	int16_t kp;
	int16_t ki;
	int16_t kd;
	float	Int;
	int16_t pwm;
} pidParam;

//====================================//
// グローバル変数の宣言
//====================================//
extern uint8_t	targetSpeed;	// 目標速度
extern float	targetSpeedCommand_m_s;	// setTargetSpeedで指定した速度指令値[m/s]
extern float 	targetAngle;    // 目標角度
extern float    targetAngularVelocity;  // 目標角速度
extern int16_t  targetDist;		        // 目標X座標

extern pidParam lineTraceCtrl;
extern pidParam lineTraceOmegaFBCtrl;
extern pidParam veloCtrl;
extern pidParam yawRateCtrl;
extern pidParam yawCtrl;
extern pidParam distCtrl;
extern int16_t speedFeedForwardGain;

extern int32_t log_targetAngularVelocity; // ログ用目標角速度

//====================================//
// プロトタイプ宣言
//====================================//
void setTargetSpeed (float speed);
void setTargetAngularVelocity (float angularVelocity);
void setTargetAngle (float angle);
void setTargetDist (float dist);
void resetSpeedPID (void);
void writePIDparameters(pidParam *pid);
void readPIDparameters(pidParam *pid);
void writeSpeedFeedForwardGain(int16_t gain);
void readSpeedFeedForwardGain(int16_t *gain);
void motorControlTrace(void);
void motorControlTraceOmegaFB(void);
void motorControlSpeed(void);
void motorControlYawRate(void);
void motorControlYaw(void);
void motorControldist(void);

#endif // LINETRACE_H_
