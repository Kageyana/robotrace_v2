#ifndef LINETRACE_H_
#define LINETRACE_H_
//====================================//
// インクルード
//====================================//
#include "main.h"
#include "control.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
//====================================//
// 調整パラメータ
//====================================//
#define PID_GAIN_MIN 0 // SDから受け付ける各PIDゲインの下限[実装スケールの整数値]。
#define PID_GAIN_MAX 255 // SDから受け付ける各PIDゲインの上限[実装スケールの整数値]。
#define SPEED_FF_GAIN_MIN 0 // SDから受け付ける速度FFゲインの下限[Crr ×1000]。
#define SPEED_FF_GAIN_MAX 255 // SDから受け付ける速度FFゲインの上限[Crr ×1000]。

// PID・速度FFの既定値は、有効なSD設定値を優先する。
// PIDゲインの数値スケールは各制御関数の演算に従う。
// PID制御用ゲイン定義(ライントレース 角速度FBなし)
#define KP1		28 // 通常ライン追従の比例ゲイン既定値[実装スケールの整数値]。
#define KI1		0 // 通常ライン追従の積分ゲイン既定値[実装スケールの整数値]。
#define KD1		200 // 通常ライン追従の微分ゲイン既定値[実装スケールの整数値]。

// ライントレース用センサ重み係数（中央から外側に向かって適用）
#define TRACE_WEIGHT_CENTER		1.00f // 中央の左右ラインセンサーへ掛ける重み[無次元]。
#define TRACE_WEIGHT_INNER		1.01f // 内側の左右ラインセンサーへ掛ける重み[無次元]。
#define TRACE_WEIGHT_MIDDLE		1.02f // 中間の左右ラインセンサーへ掛ける重み[無次元]。
#define TRACE_WEIGHT_OUTER		1.03f // 外側の左右ラインセンサーへ掛ける重み[無次元]。
#define TRACE_WEIGHT_FAR		1.04f // 最外側の左右ラインセンサーへ掛ける重み[無次元]。

// PID制御用ゲイン定義(速度制御)
#define KP2		10 // 速度制御の比例ゲイン既定値[実装スケールの整数値]。
#define KI2		0 // 速度制御の積分ゲイン既定値[実装スケールの整数値]。
#define KD2		0 // 速度制御の微分ゲイン既定値[実装スケールの整数値]。

// PID制御用ゲイン定義(角速度制御)
#define KP3		38 // 角速度制御の比例ゲイン既定値[実装スケールの整数値]。
#define KI3		1 // 角速度制御の積分ゲイン既定値[実装スケールの整数値]。
#define KD3		15 // 角速度制御の微分ゲイン既定値[実装スケールの整数値]。

// PID制御用ゲイン定義(ライントレース 角速度FBあり)
#define KP6		7 // 角速度FB付きライン追従の比例ゲイン既定値[実装スケールの整数値]。
#define KI6		0 // 角速度FB付きライン追従の積分ゲイン既定値[実装スケールの整数値]。
#define KD6		25 // 角速度FB付きライン追従の微分ゲイン既定値[実装スケールの整数値]。

// BOOST_DISTANCE低速カーブで過大なライン角速度目標を抑える採用設定
#define LINE_TRACE_OMEGA_TARGET_CAP_ENABLE 1 // 低速DISTANCE走行のライン角速度目標制限[0:無効、1:有効]。
#define LINE_TRACE_OMEGA_TARGET_CAP_ABS_DEG_S 1200 // 制限を適用したときの目標角速度の絶対値上限[deg/s]。
#define LINE_TRACE_OMEGA_TARGET_CAP_MAX_SPEED_PULSE 90 // 目標角速度制限を適用する目標速度の上限[pulse/ms]。
#define LINE_TRACE_OMEGA_CURVE_FF_ENABLE 1 // 低速小Rカーブの曲率フィードフォワード[0:無効、1:有効]。
#define LINE_TRACE_OMEGA_CURVE_FF_ROC_MAX_MM 200 // 曲率FFを適用する曲率半径の絶対値上限[mm]。
#define LINE_TRACE_OMEGA_CURVE_FF_ROC_MIN_MM 80 // 曲率FFの計算に使う曲率半径の絶対値下限[mm]。
#define LINE_TRACE_OMEGA_CURVE_FF_MAX_SPEED_PULSE 95 // 曲率FFを適用する目標速度の上限[pulse/ms]。
#define LINE_TRACE_OMEGA_CURVE_FF_GAIN_PCT 100 // 曲率から計算した目標角速度へ掛ける倍率[%]。
#define LINE_TRACE_OMEGA_CURVE_FB_SCALE_PCT 35 // 曲率FF適用中のラインセンサーFBの倍率[%]。

// 速度フィードフォワード関連定義
#define SPEED_FEEDFORWARD_GAIN_DEFAULT          150 // 速度FFの転がり抵抗係数の既定値[Crr ×1000]。150でCrr=0.150。
#define SPEED_FEEDFORWARD_GEAR_RATIO            2.0f // 速度FFモデルのモーター回転数/車輪回転数[無次元]。
#define SPEED_FEEDFORWARD_EFFICIENCY            0.90f // 速度FFモデルのギア伝達効率[無次元]。
#define SPEED_FEEDFORWARD_KV_RPM_PER_V          2710.0f // 速度FFモデルのモーター速度定数[rpm/V]。
#define SPEED_FEEDFORWARD_S_RPM_PER_MNM         2240.0f // 速度FFモデルのモーター速度/トルク勾配[rpm/mNm]。
#define SPEED_FEEDFORWARD_WHEEL_DIAMETER_MM     23.5f // 速度FFモデルの車輪直径[mm]。
#define SPEED_FEEDFORWARD_MASS_KG               0.123f // 速度FFモデルの車体質量[kg]。
#define SPEED_FEEDFORWARD_GRAVITY               9.80665f // 速度FFモデルの重力加速度[m/s^2]。
#define SPEED_FEEDFORWARD_SIGN_DEADBAND_MM_S    0.001f // 速度FFの進行方向符号判定に使う不感帯[mm/s]。
#define SPEED_FEEDFORWARD_PWM_MAX_DEFAULT       1000 // 速度FF出力の既定の正規化モーター指令上限[指令値]。
#define SPEED_FEEDFORWARD_CRR_SCALE             0.001f // 保存速度FFゲインからCrrへ変換する係数[1/保存値]。

typedef struct {
	const char *name;
	int16_t kp;
	int16_t ki;
	int16_t kd;
	float	Int;
	int16_t pwm;
} pidParam;

//====================================//
// グローバル変数の宣言、定義
//====================================//
extern uint8_t	targetSpeed;	// 目標速度
extern float	targetSpeedCommand_m_s;	// setTargetSpeedで指定した速度指令値[m/s]
extern float    targetAngularVelocity;  // 目標角速度

extern pidParam lineTraceCtrl;
extern pidParam lineTraceOmegaFBCtrl;
extern pidParam veloCtrl;
extern pidParam veloCtrlL;
extern pidParam veloCtrlR;
extern pidParam yawRateCtrl;
extern int16_t speedFeedForwardGain;

extern int32_t log_targetAngularVelocity; // ログ用目標角速度

//====================================//
// プロトタイプ宣言
//====================================//
void setTargetSpeed (float speed);
void setTargetAngularVelocity (float angularVelocity);
void resetSpeedPID (void);
void writePIDparameters(pidParam *pid);
void readPIDparameters(pidParam *pid);
void writeSpeedFeedForwardGain(int16_t gain);
void readSpeedFeedForwardGain(int16_t *gain);
void motorControlTrace(void);
void motorControlTraceOmegaFB(void);
void motorControlSpeed(void);
void motorControlSpeedLR(int16_t targetEncL, int16_t targetEncR);
void motorControlYawRate(void);

#endif // LINETRACE_H_
