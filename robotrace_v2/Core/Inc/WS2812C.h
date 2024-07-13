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
#define PERIOD_LED_TIM htim1.Init.Period // counter period

typedef struct
{
	uint8_t pattern;
	int8_t r;
	int8_t g;
	int8_t b;
} RGBLED;
//====================================//
// グローバル変数の宣言
//====================================//
extern bool datasentflag;
extern bool lineflag;
//====================================//
// プロトタイプ宣言
//====================================//
void setLED(int LEDnum, int Red, int Green, int Blue);
void sendLED(void);
void fullColorLED(uint8_t brightness, uint8_t add);
void r2b(RGBLED *led, uint8_t brightness, uint8_t add);
void led_out(uint8_t data);
void clearLED(void);
#endif // WS2812C_H_
