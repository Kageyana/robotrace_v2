#ifndef CONTROL_H_
#define CONTROL_H_
//====================================//
// インクルード
//====================================//
#include "main.h"
#include "SDcard.h"
//====================================//
// シンボル定義
//====================================//
// 機体諸元

// 速度パラメータ関連
#define PARAM_SEARCH 1.0F
#define PARAM_STOP 0.8F
#define PARAM_SHORTCUT 0.5F
#define PARAM_BOOST_STRAIGHT 2.8F
#define PARAM_BOOST_1500 2.6F
#define PARAM_BOOST_1300 2.4F
#define PARAM_BOOST_1000 2.2F
#define PARAM_BOOST_800 2.0F
#define PARAM_BOOST_700 1.8F
#define PARAM_BOOST_600 1.6F
#define PARAM_BOOST_500 1.5F
#define PARAM_BOOST_400 1.4F
#define PARAM_BOOST_300 1.3F
#define PARAM_BOOST_200 1.2F
#define PARAM_BOOST_100 1.1F

#define PARAM_UP_STEP 1.02F
// スリップ検出パラメータ（1ms割り込みで使用するのでマクロで管理）
#define SLIP_WINDOW_SAMPLES		15U    // Δvを取る時間窓長[ms]（リングバッファ長）
#define SLIP_SAMPLE_PERIOD_S	DEFF_TIME // サンプリング周期[s]（1ms）
#define SLIP_SPEED_SKIP_MPS		0.1f	// 超低速時に判定をスキップする速度閾値[m/s]
#define SLIP_MISMATCH_HIGH		7.5f   // [m/s^2]
#define SLIP_MISMATCH_LOW		5.5f   // [m/s^2]
// 旋回中だけ上げる閾値
#define SLIP_MISMATCH_HIGH_TURN		2.2f 	// 係数倍
#define SLIP_MISMATCH_LOW_TURN		2.5f	// 係数倍
#define SLIP_HIGH_COUNT_REQ			10U		// 上側しきい値を超え続ける必要サンプル数
#define SLIP_LOW_COUNT_REQ			5U		// 下側しきい値を下回り続ける必要サンプル数
#define SLIP_LPF_COEF				0.05f	// スリップ指標に掛ける一次LPF係数

#define SLIP_ACC_BIAS_COEF		0.001f   // ≒ dt/1s（1秒時定数くらい）
#define SLIP_ACC_LPF_COEF		0.05f    // ≒ 20ms前後のLPF

#define SLIP_GYRO_THR_DPS		250.0f	// 旋回とみなす角速度閾値[deg/s]
#define SLIP_GYRO_THR_RADS		(DPS2RADS(SLIP_GYRO_THR_DPS))

// ゴール
#define COUNT_GOAL 5 // ゴールマーカーを読む回数

// スタートモード
#define START_SERACH 3
#define START_OPTIMAL 4

// クロスライン検出関連
#define TRACE_CROSSLINE_TH			2000	// クロスライン検出閾値
#define TRACE_CROSSLINE_DISTANCE	60		// クロスライン付近でゲインを変更する距離[mm]

// ファイル名
#define FILENAME_TARGET_SPEED "targetSpeeds"

typedef struct
{
	float search;
	float stop;
	float bstStraight;
	float bst1500;
	float bst1300;
	float bst1000;
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
extern uint8_t patternTrace;	// パターン番号
extern bool modeDSP;			// LCD表示選択
extern bool modeLOG;			// ログ取得状況
extern bool initMSD;			// microSD初期化状況
extern bool initLCD;			// LCD初期化状況
extern bool initIMU;			// IMU初期化状況
extern bool initCurrent;		// 電流センサ初期化状況
extern uint8_t autoStart;		// 5走を自動で開始する
extern bool stateCrossLine;		// クロスライン検出状態

extern uint16_t analogValLSon[10]; // ADC結果格納配列
extern uint16_t analogValLSoff[10]; // ADC結果格納配列
extern uint16_t analogVal2[4];	// ADC結果格納配列
extern float batteryVoltage_V;	// DWT初期化後に取得したバッテリ電圧[V]

// パラメータ関連
extern speedParam tgtParam;

// マーカー関連
extern uint8_t courseMarker;
extern uint8_t beforeCourseMarker;
extern uint16_t cntMarker;
extern uint8_t courseMarkerLog;

// タイマ関連
extern uint32_t cntRun;
extern int16_t countdown;
//====================================//
// プロトタイプ宣言
//====================================//
void initSystem(void);
void loopSystem(void);
void emargencyStop(void);
void countDown(void);
void checkROC(void);
void getADC2(void);
void setEncoderVal(void);
void writeTgtspeeds(void);
void readTgtspeeds(void);
void checkCrossLine(void);
void updateSlipDetection(void);
float getSlipDeltaImu(void);
float getSlipDeltaEnc(void);
float getSlipIndicatorFiltered(void);
bool getSlipFlag(void);

#endif // CONTROL_H_
