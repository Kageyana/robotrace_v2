#ifndef CONTROL_PARAMS_H_
#define CONTROL_PARAMS_H_

#include <stdint.h>

/**
 * @brief ライントレース用カスケード制御パラメータ構造体。
 */
typedef struct {
	float kp;				/**< 比例ゲイン */
	float ki;				/**< 積分ゲイン */
	float kd;				/**< 微分ゲイン */
	float i_limit;		/**< 積分リミット */
	float d_lpf;			/**< 微分ローパス時定数[s] */
	float ref_limit;		/**< 角速度指令の飽和値[rad/s] */
} LineToYawParams;

/**
 * @brief 角速度フィードバック用PIDパラメータ構造体。
 */
typedef struct {
	float kp;				/**< 比例ゲイン */
	float ki;				/**< 積分ゲイン */
	float kd;				/**< 微分ゲイン */
	float i_limit;		/**< 積分リミット */
	float out_limit;		/**< 出力飽和値 */
} YawPidParams;

/**
 * @brief 左右速度フィードバック用PIDパラメータ構造体。
 */
typedef struct {
	float kp;				/**< 比例ゲイン */
	float ki;				/**< 積分ゲイン */
	float kd;				/**< 微分ゲイン */
	float i_limit;		/**< 積分リミット */
} SpeedPidParams;

/**
 * @brief カスケード制御全体の調整パラメータ。
 */
typedef struct {
	LineToYawParams line;	/**< ライン偏差→角速度指令 */
	YawPidParams yaw;		/**< 角速度フィードバック */
	SpeedPidParams speed_l;	/**< 左輪速度フィードバック */
	SpeedPidParams speed_r;	/**< 右輪速度フィードバック */
	float duty_limit;		/**< 出力Dutyの最大絶対値 */
	float tread_half;		/**< トレッド半分[m] */
	float v_limit;		/**< 並進速度制限[m/s] */
} CascadeControlParams;

/** 既定のカスケード制御パラメータ */
extern const CascadeControlParams cascadeControlParamsDefault;
/** 現在利用中のカスケード制御パラメータ */
extern CascadeControlParams cascadeControlParams;

void CascadeControlParamsResetDefaults(CascadeControlParams *params);

#endif /* CONTROL_PARAMS_H_ */
