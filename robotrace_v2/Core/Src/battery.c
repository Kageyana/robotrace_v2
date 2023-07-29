//====================================//
// インクルード
//====================================//
#include "battery.h"
//====================================//
// グローバル変数の宣言
//====================================//
uint16_t motorCurrentL, motorCurrentR, batteryVal;
/////////////////////////////////////////////////////////////////////
// モジュール名 getADC2
// 処理概要     AD値の取得
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void getADC2(void) {
    HAL_ADC_Start(&hadc2);
    if( HAL_ADC_PollForConversion(&hadc2, 1) == HAL_OK ) {
        motorCurrentL = HAL_ADC_GetValue(&hadc2);
    }

    HAL_ADC_Start(&hadc2);
    if( HAL_ADC_PollForConversion(&hadc2, 1) == HAL_OK ) {
        motorCurrentR = HAL_ADC_GetValue(&hadc2);
    }

    HAL_ADC_Start(&hadc2);
    if( HAL_ADC_PollForConversion(&hadc2, 1) == HAL_OK ) {
        batteryVal = HAL_ADC_GetValue(&hadc2);
    }
    HAL_ADC_Stop(&hadc2);
}

/////////////////////////////////////////////////////////////////////
// モジュール名 showBattery
// 処理概要     グラフィック液晶にバッテリ残量を表示する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void showBattery (void) {
    if (batteryVal > 2036) {
        // 8.0V以上のとき
        ssd1306_DrawRectangle(115,4,124,13, Black);
        ssd1306_DrawRectangle(116,5,123,12, Black);
        ssd1306_FillRectangle(117,6,121,11, White);
    } else {
        ssd1306_FillRectangle(115,4,123,13, Black);
    }

    if (batteryVal > 1909) {
        // 7.5V～8.0Vのとき
        ssd1306_DrawRectangle(106,4,115,13, Black);
        ssd1306_DrawRectangle(107,5,114,12, Black);
        ssd1306_FillRectangle(108,6,113,11, White);
    } else {
        ssd1306_FillRectangle(106,4,115,13, Black);
    }

    if (batteryVal > 1720) {
        // 7.5V未満のとき
        ssd1306_DrawRectangle(97,4,106,13, Black);
        ssd1306_DrawRectangle(98,5,105,12, Black);
        ssd1306_FillRectangle(99,6,104,11, White);
    } else {
        ssd1306_FillRectangle(97,4,106,13, Black);
    }

    if (batteryVal < 500) {
        ssd1306_FillRectangle(97,4,124,13, Black);
        ssd1306_SetCursor(103,5);
        ssd1306_printf(Font_6x8,"USB");
    }
}