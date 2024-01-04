//====================================//
// インクルード
//====================================//
#include "battery.h"
//====================================//
// グローバル変数の宣言
//====================================//
uint16_t    batteryVal;
uint8_t     batteryLevel;

/////////////////////////////////////////////////////////////////////
// モジュール名 SchmittBatery
// 処理概要     バッテリ残量(AD値)にヒステリシスをもたせる
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void SchmittBatery (void) {
    static bool batteryThreshold[4] = {false, false, false, false};

    if (batteryVal < 500) {
        batteryLevel = BAT_LV_NONE;
    }

    if (batteryVal > 1881 && !batteryThreshold[1]) {
        // 7.45V以上のとき
        batteryLevel = BAT_LV_1;
        batteryThreshold[1] = true;
    } else if (batteryVal < 1869 && batteryThreshold[1]) {
        // 7.40V以上のとき
        batteryLevel = BAT_LV_0;
        batteryThreshold[1] = false;
    }

    if (batteryVal > 1933 && !batteryThreshold[2]) {
        // 7.65V以上のとき
        batteryLevel = BAT_LV_2;
        batteryThreshold[2] = true;
    } else if (batteryVal < 1919 && batteryThreshold[2]) {
        // 7.60V以上のとき
        batteryLevel = BAT_LV_1;
        batteryThreshold[2] = false;
    }

    if (batteryVal > 2035 && !batteryThreshold[3]) {
        // 8.05V以上のとき
        batteryLevel = BAT_LV_3;
        batteryThreshold[3] = true;
    } else if (batteryVal < 2023 && batteryThreshold[3]) {
        // 8.00V以下のとき
        batteryLevel = BAT_LV_2;
        batteryThreshold[3] = false;
    }
}
/////////////////////////////////////////////////////////////////////
// モジュール名 showBattery
// 処理概要     グラフィック液晶にバッテリ残量を表示する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void showBattery (void) {

    showBatMark();
    switch(batteryLevel){
        case BAT_LV_0:
            ssd1306_FillRectangle(96,2,123,11, Black);
            break;

        case BAT_LV_1:
            // 7.4Vのとき
            ssd1306_FillRectangle(98,4,103,9, White);
            ssd1306_FillRectangle(107,4,112,9, Black);
            ssd1306_FillRectangle(116,4,121,9, Black);
            break;

        case BAT_LV_2:
            // 7.6Vのとき
            ssd1306_FillRectangle(98,4,103,9, White);
            ssd1306_FillRectangle(107,4,112,9, White);
            ssd1306_FillRectangle(116,4,121,9, Black);
            break;

        case BAT_LV_3:
            // 8.0Vのとき
            ssd1306_FillRectangle(98,4,103,9, White);
            ssd1306_FillRectangle(107,4,112,9, White);
            ssd1306_FillRectangle(116,4,121,9, White);
            break;

        case BAT_LV_NONE:
            // バスパワー駆動のとき
            ssd1306_FillRectangle(96,2,123,11, Black);
            ssd1306_SetCursor(102,3);
            ssd1306_printf(Font_6x8,"USB");
            break;
    }
}
/////////////////////////////////////////////////////////////////////
// モジュール名 showBatMark
// 処理概要     グラフィック液晶にバッテリマークを表示する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void showBatMark (void) {
    // 電池マーク
	ssd1306_DrawRectangle(95,1,124,12,White);
	ssd1306_FillRectangle(125,3,127,10,White);
}