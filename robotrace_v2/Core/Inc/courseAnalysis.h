#ifndef COURSEANALYSIS_H_
#define COURSEANALYSIS_H_
//====================================//
// インクルード
//====================================//
#include "main.h"
#include "stdlib.h"
#include "math.h"
//====================================//
// シンボル定義
//====================================//
#define ANALYSISBUFFSIZE    512
#define DELTATIME           0.01F   // ログ保存周期[s]
#define CALCDISTANCE        50      // 距離解析ステップ[mm]
#define MACHINEACCELE       3.4F    // 加速度[m/s^2]
#define MACHINEDECREACE     3.4F    // 減速度[m/s^2]
#define BOOST_MARKER        1       // マーカー基準2次走行
#define BOOST_DISTANCE      2       // 距離基準2次走行
#define BOOST_SHORTCUT      3       // ショートカット2次走行
#define SEARCHRANGE         150     // 距離補正時の距離検索範囲[mm]
#define DPS2RDS             0.0174533F  // deg/s→rad/s
#define SHORTCUTWINDOW      32      // ショートカットコース生成時の移動平均サンプル数

typedef struct {
    float   ROC;
    float   boostSpeed;
} AnalysisData;

typedef struct {
    int32_t distance;
    int32_t indexPPAD;
} EventPos;

typedef struct {
    float x;
    float y;
} Courseplot;
//====================================//
// グローバル変数の宣言
//====================================//
// 2次走行関係
extern uint8_t  optimalTrace;
extern uint16_t optimalIndex;
extern int16_t  numPPADarry;
extern int16_t  numPPAMarry;
extern int16_t  pathedMarker;
extern float    boostSpeed;
extern int32_t  DistanceOptimal;
extern int16_t  analizedNumber;
extern int32_t  encTotalOptimal;

// 解析関係
extern AnalysisData PPAD[ANALYSISBUFFSIZE];
extern EventPos     markerPos[ANALYSISBUFFSIZE];
extern Courseplot   xycie;
//====================================//
// プロトタイプ宣言
//====================================//
float		calcROC(float velo, float angvelo);
void		saveLogNumber(int16_t fileNumber);
void		getLogNumber(void);
int16_t		readLogDistance(int logNumber);
float		asignVelocity(float ROC);
int			cmpfloat(const void * n1, const void * n2);
int16_t		readLogTest(int logNumber);
int16_t		calcXYcies(int logNumber);
void		calcXYcie(float encpulse, float angVelo);
void        clearXYcie(void);

#endif // COURSEANALYSIS_H_
