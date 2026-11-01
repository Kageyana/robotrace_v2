#ifndef ANALYSIS_DISPLAY_H_
#define ANALYSIS_DISPLAY_H_

//====================================//
// インクルード
//====================================//
#include <stdint.h>

//====================================//
// 調整パラメータ
//====================================//
#define ANALYSIS_DISPLAY_FRAME_MS 200U // 解析中の画面更新間隔[ms]。待機せず最大5fpsで単発DMA送信する。
#define ANALYSIS_DISPLAY_CYCLE_MS 2000U // 円環の伸縮・回転周期[ms]。

//====================================//
// グローバル変数の宣言、定義
//====================================//

//====================================//
// プロトタイプ宣言
//====================================//
void analysisDisplayBegin(const char *mode, int16_t logNumber);
void analysisDisplayProgress(const char *stage, uint32_t lineNo);
void analysisDisplayPoll(void);
void analysisDisplayEnd(void);

#endif // ANALYSIS_DISPLAY_H_
