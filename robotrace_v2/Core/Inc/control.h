#ifndef CONTROL_H_
#define CONTROL_H_
//====================================//
// インクルード
//====================================//
#include "main.h"
//====================================//
// シンボル定義
//====================================//
// 機体諸元

// 速度パラメータ関連
#define PARAM_STRAIGHT              1.3F
#define PARAM_CURVE                 1.0F
#define PARAM_STOP                  0.8F
#define PARAM_SHORTCUT              0.5F
#define PARAM_BOOST_STRAIGHT        2.4F
#define PARAM_BOOST_1500            2.2F
#define PARAM_BOOST_800             2.0F
#define PARAM_BOOST_700             1.8F
#define PARAM_BOOST_600             1.7F
#define PARAM_BOOST_500             1.6F
#define PARAM_BOOST_400             1.5F
#define PARAM_BOOST_300             1.4F
#define PARAM_BOOST_200             1.3F
#define PARAM_BOOST_100             1.2F

#define PARAM_UP_STEP               1.02F

// ゴール
#define COUNT_GOAL              2       // ゴールマーカーを読む回数

// スタートモード
#define START_SERACH            3
#define START_OPTIMAL           4

// ファイル名
#define FILENAME_TARGET_SPEED "targetSpeeds"

typedef struct {
    float straight;
    float curve;
    float stop;
    float bstStraight;
    float bst1500;
    float bst800;
    float bst700;
    float bst600;
    float bst500;
    float bst400;
    float bst300;
    float bst200;
    float bst100;
    float acceleF;
    float acceleD;
    float shortCut;
} speedParam;
//====================================//
// グローバル変数の宣言
//====================================//
// パターン、モード関連
extern uint8_t  patternTrace;	// パターン番号
extern bool     modeDSP;		// LCD表示選択
extern bool     modeLOG;        // ログ取得状況
extern bool     initMSD;        // microSD初期化状況
extern bool     initLCD;        // LCD初期化状況
extern bool     initIMU;        // IMU初期化状況
extern bool     initCurrent;    // 電流センサ初期化状況
extern uint8_t  modeCurve;	    // カーブ判断 0:直線 1:カーブ進入
extern uint8_t  autoStart;       // 5走を自動で開始する

extern uint16_t analogVal1[10];         // ADC結果格納配列
extern uint16_t analogVal2[3];         // ADC結果格納配列

// パラメータ関連
extern speedParam tgtParam;

// マーカー関連
extern uint8_t  courseMarker;
extern uint8_t  beforeCourseMarker;
extern uint32_t cntMarker;
extern uint8_t  courseMarkerLog;
extern int32_t  straightMeter;
extern bool     straightState;

// タイマ関連
extern uint32_t cntRun;
extern int16_t 	countdown;
//====================================//
// プロトタイプ宣言
//====================================//
void initSystem (void);
void loopSystem (void);
void emargencyStop (void);
void countDown (void);
void checkCurve(void);
void getADC2(void);
void writeTgtspeeds(void);
void readTgtspeeds(void);

#endif // CONTROL_H_
