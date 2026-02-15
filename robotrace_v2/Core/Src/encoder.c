//====================================//
// インクルード
//====================================//
#include "encoder.h"
//====================================//
// グローバル変数の宣言
//====================================//
int16_t encCurrentR = 0;
int16_t encCurrentL = 0;
int16_t encCurrentN = 0;
int32_t encTotalR = 0;
int32_t encTotalL = 0;
int32_t encTotalN = 0;

// 外部変数
int32_t enc1 = 0;
int32_t encRightMarker = 0;
int32_t encCurve = 0;
int32_t encChangeGain = 0;

uint16_t encRawR = 0, encRawL = 0;
uint16_t encBufR = 0, encBufL = 0;
/////////////////////////////////////////////////////////////////////
// モジュール名 getEncoder
// 処理概要     1ms間のエンコーダカウントを算出する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void getEncoder(void)
{
	// uint16_t encRawR = 0, encRawL = 0;
	// static uint16_t encBufR = 0, encBufL = 0;

	// エンコーダカウントを取得
	encRawR = ENC_TIM_R->CNT;
	encRawL = ENC_TIM_L->CNT;

	// 正常に1ms割り込みしたとき更新
	// if ((uint16_t)(bootTime * 100) > 90 && (uint16_t)(bootTime * 100) < 110)
	// {
		// 1msあたりのカウント
		encCurrentR = encRawR - encBufR;
		encCurrentL = encBufL - encRawL;
		encCurrentN = (encCurrentR + encCurrentL) / 2;
		// カウントの積算(回転方向が逆なのでマイナスで積算)
		encTotalR += encCurrentR;
		encTotalL += encCurrentL;
		encTotalN += encCurrentN;
	// }
	// 前回値を更新
	encBufR = encRawR;
	encBufL = encRawL;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 encMM
// 処理概要     mmをエンコーダのパルス数に変換して返す
// 引数         mm:変換する長さ[mm]
// 戻り値       変換したパルス数
///////////////////////////////////////////////////////////////////////////
int32_t encMM(int16_t mm)
{
	return PALSE_MILLIMETER * abs(mm);
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 encPulse
// 処理概要     エンコーダのパルス数をm/sに変換して返す
// 引数         pulse:変換するパルス数
// 戻り値       変換した速度[m/s]
///////////////////////////////////////////////////////////////////////////
float encPulse(int32_t pulse)
{
	return (float)abs(pulse) / PALSE_MILLIMETER;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 calcDlMm
// 処理概要     速度と経過時間から移動距離[mm]を算出する
// 引数         velo:エンコーダ速度(10msあたりのパルス数), dt:経過時間[s]
// 戻り値       移動距離[mm]
///////////////////////////////////////////////////////////////////////////
float calcDlMm(int16_t velo, float dt)
{
	return encPulse((int32_t)velo) * dt * 1000.0f;
}

