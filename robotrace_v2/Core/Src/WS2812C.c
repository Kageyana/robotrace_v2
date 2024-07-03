//====================================//
// インクルード
//====================================//
#include "WS2812C.h"
//====================================//
// グローバル変数の宣言
//====================================//
volatile uint8_t LED_Data[MAX_LED][4];
volatile uint16_t pwmData[(24*MAX_LED)+230] = {0};
volatile bool datasentflag = false;

bool lineflag = false;
///////////////////////////////////////////////////////////////////////////
// モジュール名 setLED
// 処理概要     色ごとの輝度を設定する
// 引数         LEDnum:設定するLEDの番号(0から) rgb:色ごとの輝度(0～256)
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void setLED (int LEDnum, int Red, int Green, int Blue) {
	if (!datasentflag) {
		LED_Data[LEDnum][0] = LEDnum;
		LED_Data[LEDnum][1] = Green;
		LED_Data[LEDnum][2] = Red;
		LED_Data[LEDnum][3] = Blue;
	}
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 sendLED
// 処理概要     setLEDで設定したRGBデータを送信する
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void sendLED (void) {
	volatile uint32_t indx=0;
	volatile uint32_t color;

	if (!datasentflag) {
		for (int i= 0; i<MAX_LED; i++) {
			color = ((LED_Data[i][1]<<16) | (LED_Data[i][2]<<8) | (LED_Data[i][3]));

			for (int i=23; i>=0; i--) {
				if (color&(1<<i)) {
					pwmData[indx] = 156;  // 25/7 of 224
				} else {
					pwmData[indx] = 56;  //2/7 of 224
				}
				indx++;
			}
		}

		for (int i=0; i<230; i++) {
			pwmData[indx] = 0;
			indx++;
		}

		datasentflag = true;
		HAL_TIMEx_PWMN_Start_DMA(&htim1, TIM_CHANNEL_3, pwmData, indx);
	}
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 lineLED
// 処理概要     左(右)から一方向に色が変わる
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void lineLED (void)
{
	uint8_t r[MAX_LED],g[MAX_LED],b[MAX_LED],patternLED[MAX_LED];
	static RGBLED led[MAX_LED] = { 1,0,0,0};

	if(!lineflag) {
		for (int i= 0; i<MAX_LED; i++) {
			led[i].pattern = i+1;
		}
		lineflag = true;
	}

	if(lineflag) {
		for (int i= 0; i<MAX_LED; i++) {
			r2b(&led[i],10);
			setLED(i, led[i].r, led[i].g, led[i].b);
		}
		sendLED();
	}
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 r2b
// 処理概要     赤から青へ色をフルカラーに変える
// 引数         *led:RGB構造体のポインタ brightness:最大輝度(0～256)
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void r2b(RGBLED *led, uint8_t brightness){
	switch(led->pattern) {
		case 1:
			// 赤スタート
			// 緑増
			led->r = brightness;
			led->g++;
			led->b = 0;
			if(led->g == brightness) led->pattern=2;
			break;
		case 2:
			// 赤減
			led->r--;
			led->g = brightness;
			led->b = 0;
			if(led->r == 0) led->pattern=3;
			break;
		case 3:
			// 緑スタート
			// 青増
			led->r = 0;
			led->g = brightness;
			led->b++;
			if(led->b == brightness) led->pattern=4;
			break;
		case 4:
			// 緑減
			led->r = 0;
			led->g--;
			led->b = brightness;
			if(led->g == 0) led->pattern=5;
			break;
		case 5:
			// 青スタート
			// 赤増
			led->r++;
			led->g = 0;
			led->b = brightness;
			if(led->r == brightness) led->pattern=6;
			break;
		case 6:
			// 青減
			led->r = brightness;
			led->g = 0;
			led->b--;
			if(led->b == 0) led->pattern=1;
			break;
	}
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 r2b
// 処理概要     赤から青へ色をフルカラーに変える
// 引数         *led:RGB構造体のポインタ brightness:最大輝度(0～256)
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void b2r(RGBLED *led, uint8_t brightness){
	switch(led->pattern) {
		case 1:
			// 赤スタート
			// 緑増
			led->r = brightness;
			led->g++;
			led->b = 0;
			if(led->g == brightness) led->pattern=2;
			break;
		case 2:
			// 赤減
			led->r--;
			led->g = brightness;
			led->b = 0;
			if(led->r == 0) led->pattern=3;
			break;
		case 3:
			// 緑スタート
			// 青増
			led->r = 0;
			led->g = brightness;
			led->b++;
			if(led->b == brightness) led->pattern=4;
			break;
		case 4:
			// 緑減
			led->r = 0;
			led->g--;
			led->b = brightness;
			if(led->g == 0) led->pattern=5;
			break;
		case 5:
			// 青スタート
			// 赤増
			led->r++;
			led->g = 0;
			led->b = brightness;
			if(led->r == brightness) led->pattern=6;
			break;
		case 6:
			// 青減
			led->r = brightness;
			led->g = 0;
			led->b--;
			if(led->b == 0) led->pattern=1;
			break;
	}
}