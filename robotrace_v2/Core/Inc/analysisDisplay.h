#ifndef ANALYSIS_DISPLAY_H_
#define ANALYSIS_DISPLAY_H_

//====================================//
// インクルード
//====================================//
#include <stdint.h>

//====================================//
// 調整パラメータ
//====================================//
#define ANALYSIS_DISPLAY_FRAME_MS 50U // 円環の更新間隔[ms]。部分DMAで最大20fps、転送完了は待たない。
#define ANALYSIS_DISPLAY_CYCLE_MS 2000U // 円環の伸縮・回転周期[ms]。
#define ANALYSIS_DISPLAY_PROGRESS_MS 200U // 進捗行の最短更新間隔[ms]。秒表示と分けて転送量を制限する。

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
