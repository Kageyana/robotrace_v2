#ifndef SWITCH_H_
#define SWITCH_H_

//====================================//
// インクルード
//====================================//
#include "main.h"
//====================================//
// シンボル定義
//====================================//
#define SW_NONE     0x0
#define SW_UP		0x1
#define SW_PUSH 	0x2
#define	SW_LEFT	    0x3
#define SW_RIGHT	0x4
#define SW_DOWN	    0x5

#define SW_TACT_R   0x6
#define SW_TACT_L   0x7

/*************************************** 自動生成関数 *************************************/
// タクトスイッチ
#define TACTSW1	HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9)		// 上
#define TACTSW2	HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9)		// 左
#define TACTSW3	HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8)		// 右
#define TACTSW4	HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)		// 下
#define TACTSW5	HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8)		// 押し込み
// ロータリーコードスイッチ
#define DIPSW1	HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)
#define DIPSW2	HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)
#define DIPSW3	HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13)
#define DIPSW4	HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)

// メインボード上のタクトスイッチ
#define ButtonR	HAL_GPIO_ReadPin(ButtonR_GPIO_Port,ButtonR_Pin)
#define ButtonL	HAL_GPIO_ReadPin(ButtonL_GPIO_Port,ButtonL_Pin)
/******************************************************************************************/

//====================================//
// グローバル変数の宣言
//====================================//
extern uint8_t		swValTact;
extern uint8_t		swValRotary;
extern uint8_t		swValMainTact;

// タイマ関連
extern uint16_t		cntSW;			// アナログ入力スイッチのチャタリング防止用
//====================================//
// プロトタイプ宣言
//====================================//
uint8_t getSWrotary(void);
uint8_t getSWtact(void);
uint8_t getSWMainTact(void);
void    getSwitches(void);
#endif // SWITCH_H_
