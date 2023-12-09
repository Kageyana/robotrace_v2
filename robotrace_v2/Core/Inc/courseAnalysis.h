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
#define OPT_BUFF_SIZE           700
#define OPT_SHORT_BUFF_SIZE     2000
#define DELTATIME               0.01F   // ログ保存周期[s]
#define CALCDISTANCE            50      // 距離解析ステップ[mm]
#define CALCDISTANCE_SHORTCUT   10      // 距離解析ステップ(ショートカット走行)[mm]
#define MACHINEACCELE       3.4F    // 加速度[m/s^2]
#define MACHINEDECREACE     3.4F    // 減速度[m/s^2]
#define BOOST_MARKER        1       // マーカー基準2次走行
#define BOOST_DISTANCE      2       // 距離基準2次走行
#define BOOST_SHORTCUT      3       // ショートカット2次走行
#define SEARCHRANGE         150     // 距離補正時の距離検索範囲[mm]
#define DEG2RAD             M_PI/180.0F  // deg→rad
#define RAD2DEG             180.0F/M_PI  // rad→deg
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
    float w;
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
extern int32_t  encPID;

// 解析関係
extern AnalysisData PPAD[OPT_BUFF_SIZE];
extern EventPos     markerPos[OPT_BUFF_SIZE];
extern Courseplot   xycie;
extern Courseplot   shortCutxycie[OPT_SHORT_BUFF_SIZE];
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
void		calcXYcie(float encpulse, float angVelo, float dt);
void        clearXYcie(void);
void        setShortCutTarget(void);

#endif // COURSEANALYSIS_H_
