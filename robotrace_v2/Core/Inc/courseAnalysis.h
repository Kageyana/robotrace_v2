#ifndef COURSEANALYSIS_H_
#define COURSEANALYSIS_H_
//====================================//
// インクルード
//====================================//
#include "main.h"
#include "encoder.h"
#include "stdlib.h"
#include "math.h"
//====================================//
// シンボル定義
//====================================//
#define OPT_BUFF_SIZE 1000
#define OPT_SHORT_BUFF_SIZE 1000
#define CALCDISTANCE 50				// 距離解析ステップ[mm]
#define CALCDISTANCE_SHORTCUT 10 	// 距離解析ステップ(ショートカット走行)[mm]
#define MACHINEACCELE 3.4F			// 加速度[m/s^2]
#define MACHINEDECREACE 3.0F		// 減速度[m/s^2]

#define BOOST_NONE 0				// 1次走行
#define BOOST_MARKER 1				// マーカー基準2次走行
#define BOOST_DISTANCE 2			// 距離基準2次走行
#define BOOST_SHORTCUT 3			// ショートカット2次走行

#define SEARCHRANGE 150				// 距離補正時の距離検索範囲[mm]

#define DEG2RAD M_PI / 180.0F		// deg→rad
#define RAD2DEG 180.0F / M_PI		// rad→deg
#define DPS2RADS(dps)   ((dps) * DEG2RAD)	// deg/s → rad/s

#define SHORTCUTWINDOW 4			// ショートカットコース生成時の移動平均サンプル数

#define ROC_STRAIGHTTH 1300.0F		// 直線とみなす曲率半径の閾値[mm]

//====================================//
// 3次走行用スリップ解析(2次ログ)の調整用定数
//====================================//
#define CA_SECOND_LOG_LINE_BUFSIZE 1600	// 2次ログ1行バッファサイズ
#define CA_SLIP_CNT_MIN 5					// スリップ回数のノイズ除外閾値
#define CA_SLIP_FRAC_FULL 0.60f			// risk=1.0とみなすスリップ割合
#define CA_SLIP_EXPAND_1 0.30f				// 近傍拡張係数(±1)
#define CA_SLIP_EXPAND_2 0.10f				// 近傍拡張係数(±2)
#define CA_SLIP_DOWN_RISK 0.18f			// riskに応じた基本減速ゲイン
#define CA_SLIP_DOWN_LONG_EXTRA 0.02f		// 縦スリップ追加減速
#define CA_SLIP_DOWN_LAT_EXTRA 0.03f		// 横スリップ追加減速
#define CA_SLIP_MIN_SCALE 0.90f			// 最小スケール(減速下限)
#define CA_SLIP_UP_STRAIGHT 0.060f			// 直線での微増速
#define CA_SLIP_UP_CURVE 0.032f				// カーブでの微増速

// #define WRITE_BOOSTSPEED_LOG 	 // 速度計画ログを書き出すかどうかのフラグ

// ファイル名
#define FILENAME_ANALIZENUMBER "analize"

typedef struct
{
	int16_t ROC;
	float boostSpeed;
} AnalysisData;

typedef struct
{
	int32_t distance;
	int32_t indexPPAD;
} EventPos;

typedef struct
{
	float x;
	float y;
	float w;
} Courseplot;
//====================================//
// グローバル変数の宣言
//====================================//
// 2次走行関係
extern uint8_t optimalTrace;
extern uint16_t optimalIndex;
extern int16_t numPPADarry;
extern int16_t numPPAMarry;
extern int16_t indexSC;
extern int16_t pathedMarker;
extern float boostSpeed;
extern int32_t DistanceOptimal;
extern int16_t analizedNumber;
extern int32_t encTotalOptimal;
extern int32_t encPID;
extern int32_t straightMeter;
extern bool straightState;
extern bool straightMarkerPending;
extern uint8_t straightMarkerPendingLog;

// 解析関係
extern AnalysisData PPAD[OPT_BUFF_SIZE];
extern EventPos markerPos[OPT_BUFF_SIZE];
extern Courseplot xycie;
extern Courseplot shortCutxycie[OPT_SHORT_BUFF_SIZE];
//====================================//
// プロトタイプ宣言
//====================================//
float calcROC(int16_t velo, float angvelo, float dt);
void saveLogNumber(int16_t fileNumber);
void getLogNumber(void);
int16_t readLogDistance(int logNumber);
int16_t readLogDistanceSlip(int logNumber);
float asignVelocity(int16_t ROC);
int cmpfloat(const void *n1, const void *n2);
int16_t readLogTest(int logNumber);
int16_t calcXYcies(int logNumber);
void calcXYcie(int16_t encpulse, float angVelo, float dt);
void clearXYcie(void);
void setShortCutTarget(void);
void processMarkerEvent(void);
void clearMarkerProcessState(void);

#endif // COURSEANALYSIS_H_
