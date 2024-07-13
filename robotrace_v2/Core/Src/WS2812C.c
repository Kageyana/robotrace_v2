//====================================//
// インクルード
//====================================//
#include "WS2812C.h"
//====================================//
// グローバル変数の宣言
//====================================//
uint8_t LED_Data[MAX_LED][4];
uint16_t pwmData[(24 * MAX_LED) + 230] = {0};
bool datasentflag = false;

bool lineflag = false;
///////////////////////////////////////////////////////////////////////////
// モジュール名 setLED
// 処理概要     色ごとの輝度を設定する
// 引数         LEDnum:設定するLEDの番号(0から) rgb:色ごとの輝度(0～255)
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void setLED(int LEDnum, int Red, int Green, int Blue)
{
	if (!datasentflag)
	{
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
void sendLED(void)
{
	volatile uint32_t indx = 0;
	volatile uint32_t color;

	if (!datasentflag)
	{
		for (int i = 0; i < MAX_LED; i++)
		{
			color = ((LED_Data[i][1] << 16) | (LED_Data[i][2] << 8) | (LED_Data[i][3]));

			for (int i = 23; i >= 0; i--)
			{
				if (color & (1 << i))
				{
					pwmData[indx] = 156; // 5/7 of 224
				}
				else
				{
					pwmData[indx] = 56; // 2/7 of 224
				}
				indx++;
			}
		}

		for (int i = 0; i < 230; i++)
		{
			pwmData[indx] = 0;
			indx++;
		}

		datasentflag = true;
		HAL_TIMEx_PWMN_Start_DMA(&htim1, TIM_CHANNEL_3, pwmData, indx);
	}
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 fullColorLED
// 処理概要     4つのLEDの色を順番に変える
// 引数         brightness:最大輝度(0～127) add 輝度の変化量
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void fullColorLED(uint8_t brightness, uint8_t add)
{
	static RGBLED led[MAX_LED] = {1, 0, 0, 0};

	if (!lineflag)
	{
		for (int i = 0; i < MAX_LED; i++)
		{
			led[i].pattern = i + 1;
		}
		lineflag = true;
	}

	if (lineflag)
	{
		for (int i = 0; i < MAX_LED; i++)
		{
			r2b(&led[i], brightness, add);
			setLED(i, led[i].r, led[i].g, led[i].b);
		}
		sendLED();
	}
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 r2b
// 処理概要     赤から青へ色をフルカラーに変える
// 引数         *led:RGB構造体のポインタ brightness:最大輝度(0～127) add 輝度の変化量
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void r2b(RGBLED *led, uint8_t brightness, uint8_t add)
{
	switch (led->pattern)
	{
	case 1:
		// 赤スタート
		// 緑増
		led->r = brightness;
		led->g += add;
		led->b = 0;
		if (led->g >= brightness)
		{
			led->g = brightness;
			led->pattern = 2;
		}
		break;
	case 2:
		// 赤減
		if (led->r == 0)
			led->r = brightness; // 初期値
		led->r -= add;
		led->g = brightness;
		led->b = 0;
		if (led->r <= 0)
		{
			led->r = 0;
			led->pattern = 3;
		}
		break;
	case 3:
		// 緑スタート
		// 青増
		led->r = 0;
		led->g = brightness;
		led->b += add;
		if (led->b >= brightness)
		{
			led->b = brightness;
			led->pattern = 4;
		}
		break;
	case 4:
		// 緑減
		if (led->g == 0)
			led->g = brightness; // 初期値
		led->r = 0;
		led->g -= add;
		led->b = brightness;
		if (led->g <= 0)
		{
			led->g = 0;
			led->pattern = 5;
		}
		break;
	case 5:
		// 青スタート
		// 赤増
		led->r += add;
		led->g = 0;
		led->b = brightness;
		if (led->r >= brightness)
		{
			led->r = brightness;
			led->pattern = 6;
		}
		break;
	case 6:
		// 青減
		if (led->b == 0)
			led->b = brightness; // 初期値
		led->r = brightness;
		led->g = 0;
		led->b -= add;
		if (led->b <= 0)
		{
			led->b = 0;
			led->pattern = 1;
		}
		break;
	}
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 led_out
// 処理概要     赤色で2進数表示
// 引数         表示したい2進数4bit
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void led_out(uint8_t data)
{
	for (int i = 0; i < MAX_LED; i++)
	{
		if (data & (1 << i))
		{
			setLED(i, 10, 0, 0);
		}
		else
		{
			setLED(i, 0, 0, 0);
		}
	}
	sendLED();
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 clearLED
// 処理概要     LED全消灯
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void clearLED(void)
{
	for (int i = 0; i < MAX_LED; i++)
	{
		setLED(i, 0, 0, 0);
	}
	sendLED();
}