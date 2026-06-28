#ifndef ENCODER_H_
#define ENCODER_H_

//====================================//
// インクルード
//====================================//
#include "main.h"
//====================================//
// シンボル定義
//====================================//
#define PULSE_METER 53424		 // 1m走行時のカウント
#define PULSE_MILLIMETER 54.324F // 1mmのカウント↑を1/1000

#define ENC_TIM_HANDLER_R htim8
#define ENC_TIM_HANDLER_L htim4
#define ENC_TIM_R TIM8
#define ENC_TIM_L TIM4
//====================================//
// グローバル変数の宣言
//====================================//
extern int16_t encCurrentR;
extern int16_t encCurrentL;
extern int16_t encCurrentN;
extern int32_t encTotalR;
extern int32_t encTotalL;
extern int32_t encTotalN;

extern uint16_t encRawR, encRawL;
extern uint16_t encBufR, encBufL;

// 外部変数
extern int32_t enc1;
extern int32_t encRightMarker;
extern int32_t encCurve;
extern int32_t encChangeGain;
//====================================//
// プロトタイプ宣言
//====================================//
void getEncoder(void);
int32_t encMM(int16_t mm);
float encPulse(int32_t pulse);
float calcDlMm(int16_t velo, float dt);


#endif // ENCODER_H_
