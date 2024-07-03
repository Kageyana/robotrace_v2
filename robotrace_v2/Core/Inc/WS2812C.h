#ifndef WS2812C_H_
#define WS2812C_H_

//====================================//
// インクルード
//====================================//
#include "main.h"
//====================================//
// シンボル定義
//====================================//
#define MAX_LED 4
#define PERIOD_LED_TIM htim1.Init.Period  // counter period

typedef struct {
    uint8_t pattern;
    uint8_t r;
    uint8_t g;
    uint8_t b;
} RGBLED;
//====================================//
// グローバル変数の宣言
//====================================//
extern volatile bool datasentflag;
extern bool lineflag;
//====================================//
// プロトタイプ宣言
//====================================//
void setLED (int LEDnum, int Red, int Green, int Blue);
void sendLED (void);
void lineLED (void);
void r2b(RGBLED *led, uint8_t brightness);
void b2r(RGBLED *led, uint8_t brightness);
#endif // WS2812C_H_
