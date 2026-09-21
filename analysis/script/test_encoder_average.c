#include <assert.h>
#include <stdint.h>
#include <stdlib.h>

typedef struct {
	uint16_t CNT;
} EncoderTestTimer;

static EncoderTestTimer testTimerL;
static EncoderTestTimer testTimerR;

#define ENCODER_H_
#define ENC_TIM_L (&testTimerL)
#define ENC_TIM_R (&testTimerR)
#define PULSE_MILLIMETER 58.019F
#include "../../robotrace_v2/Core/Src/encoder.c"

/////////////////////////////////////////////////////////////////////
// モジュール名 reset_encoder_test
// 処理概要     エンコーダ平均のホスト試験状態を初期化する
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void reset_encoder_test(void)
{
	encTotalL = 0;
	encTotalR = 0;
	encTotalN = 0;
	encHalfPulseRemainder = 0;
}

/////////////////////////////////////////////////////////////////////
// モジュール名 sample_encoder
// 処理概要     左右1周期分のパルスを模擬する
// 引数         left: 左パルス数[pulse]、right: 右パルス数[pulse]
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
static void sample_encoder(int16_t left, int16_t right)
{
	encBufL = (uint16_t)(left > 0 ? left : 0);
	testTimerL.CNT = (uint16_t)(left < 0 ? -left : 0);
	encBufR = (uint16_t)(right < 0 ? -right : 0);
	testTimerR.CNT = (uint16_t)(right > 0 ? right : 0);
	getEncoder();
}

int main(void)
{
	reset_encoder_test();
	for (int i = 0; i < 1000; ++i) {
		sample_encoder(1, 0);
	}
	assert(encTotalL == 1000 && encTotalR == 0);
	assert(encTotalN == 500 && encHalfPulseRemainder == 0);

	reset_encoder_test();
	for (int i = 0; i < 1000; ++i) {
		sample_encoder(-1, 0);
	}
	assert(encTotalL == -1000 && encTotalR == 0);
	assert(encTotalN == -500 && encHalfPulseRemainder == 0);

	reset_encoder_test();
	for (int i = 0; i < 999; ++i) {
		sample_encoder(i % 3 == 0 ? -1 : 1, 0);
		assert(abs((encTotalL + encTotalR) - 2 * encTotalN) <= 1);
	}
	return 0;
}
