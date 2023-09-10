#ifndef COURSEANALYSIS_H_
#define COURSEANALYSIS_H_
//====================================//
// インクルード
//====================================//
#include "main.h"
//====================================//
// シンボル定義
//====================================//
#define ANALYSISBUFFSIZE    512
#define DELTATIME           0.01F   // ログ保存周期[s]
#define CALCDISTANCE        50      // 距離解析ステップ[mm]
#define MACHINEACCELE       2.0F    // 加速度[m/s^2]
#define MACHINEDECREACE     1.5F    // 減速度[m/s^2]
#define BOOST_MARKER        1
#define BOOST_DISTANCE      2
#define SEARCHRANGE         150     // 距離補正時の距離検索範囲[mm]
#define DPS2RDS             0.0174533F  // deg/s→rad/s

typedef struct {
    int32_t time;
    uint8_t marker;
    float   velocity;
    float   angularVelocity;
    int32_t distance;
    float   ROC;
    float   boostSpeed;
} AnalysisData;

typedef struct {
    int32_t distance;
    int32_t indexPPAD;
} EventPos;
//====================================//
// グローバル変数の宣言
//====================================//
// 2次走行関係
extern uint8_t  optimalTrace;
extern uint16_t optimalIndex;
extern int16_t  numPPADarry;
extern int16_t  numPPAMarry;
extern float    boostSpeed;
extern int32_t  distanceStart, distanceEnd;
extern int16_t  analizedNumber;

// 解析関係
extern AnalysisData PPAM[ANALYSISBUFFSIZE];
extern AnalysisData PPAD[ANALYSISBUFFSIZE];
extern EventPos     markerPos[ANALYSISBUFFSIZE];
extern float        ROCmarker[ANALYSISBUFFSIZE];
//====================================//
// プロトタイプ宣言
//====================================//
float       calcROC(float velo, float angvelo);
void        saveLogNumber(int16_t fileNumber);
void        getLogNumber(void);
uint16_t    readLogMarker(int logNumber);
int16_t    readLogDistance(int logNumber);
float       asignVelocity(float ROC);
int         cmpfloat(const void * n1, const void * n2);
#endif // COURSEANALYSIS_H_
