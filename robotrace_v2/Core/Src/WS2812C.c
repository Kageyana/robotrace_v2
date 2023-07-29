//====================================//
// インクルード
//====================================//
#include "WS2812C.h"
//====================================//
// グローバル変数の宣言
//====================================//
int32_t ledBuff;            // 送信バッファ
int32_t ledIndex = 0;
int32_t ledBrightIndex = 0;
bool    ledsend = false;    // 送信フラグ
int8_t cntBit = 23;         // 送信bitのカウント
int8_t cntState = 0;        // 0/1の状態カウント

/////////////////////////////////////////////////////////////////////
// モジュール名 ledset
// 処理概要     WS2812Cへのデータ送信バッファに書き込む
// 引数         r,g,b 赤、緑、青の光量を8bitで指定する
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void ledset(uint8_t r, uint8_t g, uint8_t b) {
    if (!ledsend) {
        ledBuff = (int32_t)(g<<16)+(int32_t)(r<<8)+b; // データ結合
        ledIndex++;

        ledsend = true;
        HAL_GPIO_WritePin(RGBLED_GPIO_Port, RGBLED_Pin, GPIO_PIN_RESET);
        HAL_TIM_Base_Start_IT(&htim10);
    }
}
/////////////////////////////////////////////////////////////////////
// モジュール名 sendColorData
// 処理概要     WS2812Cへデータ送信 1データあたり900ns(cntState = 3)
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void sendColorData(void) {
    int32_t sendBit=0, i;
    i = ledBuff;
    if (ledsend) {
        sendBit = ledBuff >> cntBit; // 指定bitを最下位bitに移動する
        sendBit = sendBit & 0x00000001;
        if(sendBit) {
            // 1 codeを送信するとき
            if (cntState < 2) {
                HAL_GPIO_WritePin(RGBLED_GPIO_Port, RGBLED_Pin, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(RGBLED_GPIO_Port, RGBLED_Pin, GPIO_PIN_RESET);
            }
            if (cntState >= 4) {
                cntState = 0;
                cntBit--;
            }
        } else {
            // 0 codeを送信するとき
            if (cntState < 1) {
                HAL_GPIO_WritePin(RGBLED_GPIO_Port, RGBLED_Pin, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(RGBLED_GPIO_Port, RGBLED_Pin, GPIO_PIN_RESET);
            }
            if (cntState >= 3) {
                cntState = 0;
                cntBit--;
            }
        }
        cntState++;
        
        // 
        if (cntBit < 0) {
            HAL_GPIO_WritePin(RGBLED_GPIO_Port, RGBLED_Pin, GPIO_PIN_RESET);
            ledsend = false;
            cntBit = 23;
            ledBrightIndex++;
            HAL_TIM_Base_Stop_IT(&htim10);
        }
    }
}
