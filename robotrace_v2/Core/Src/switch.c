//====================================//
// インクルード
//====================================//
#include "switch.h"
//====================================//
// グローバル変数の宣言
//====================================//
// スイッチ関連
uint8_t swValTact;
uint8_t swValMainTact;

uint16_t swValTactAD; // 5方向タクトスイッチのAD値

// タイマ関連
uint16_t cntSW = 0; // 5方向タクトスイッチのチャタリング防止用
/////////////////////////////////////////////////////////////////////
// モジュール名 getSwitches
// 処理概要  	スイッチの読み込み(10msごとに実行)
// 引数     	なし
// 戻り値    	なし
/////////////////////////////////////////////////////////////////////
void getSwitches(void)
{
	// 5方向タクトスイッチ読み込み
	swValTact = getSW5aAxisTact();
	// メインボード上のタクトスイッチ読み込み
	swValMainTact = getSWMainTact();
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 getSW5aAxisTact
// 処理概要     タクトスイッチ値を16進数で取得
// 引数         なし
// 戻り値       スイッチ値 0～7
///////////////////////////////////////////////////////////////////////////
uint8_t getSW5aAxisTact(void)
{
	uint8_t ret = SW_NONE;

	if (swValTactAD >= 2150 && swValTactAD <= 2190)
		ret = SW_UP;
	if (swValTactAD >= 1200 && swValTactAD <= 1240)
		ret = SW_LEFT;
	if (swValTactAD >= 2800 && swValTactAD <= 2840)
		ret = SW_RIGHT;
	if (swValTactAD >= 3310 && swValTactAD <= 3350)
		ret = SW_DOWN;
	if (swValTactAD <= 100)
		ret = SW_PUSH;

	return ret;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 getSWMainTact
// 処理概要     タクトスイッチ値を16進数で取得
// 引数         なし
// 戻り値       スイッチ値 0～7
///////////////////////////////////////////////////////////////////////////
uint8_t getSWMainTact(void)
{
	uint8_t ret = SW_NONE;

	if (BTN_R == 0)
		ret = SW_TACT_R;
	if (BTN_L == 0)
		ret = SW_TACT_L;

	return ret;
}