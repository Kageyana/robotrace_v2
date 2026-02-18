#ifndef CONTROL_H_
#define CONTROL_H_
//====================================//
// インクルード
//====================================//
#include "main.h"
#include "SDcard.h"
#include <stdbool.h>
#include <stdint.h>
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
#define PARAM_DECEL_LEAD_MM 30.0F

#define PARAM_UP_STEP 1.02F

// スリップ検出パラメータ（1ms割り込みで使用するのでマクロで管理）
#define SLIP_WINDOW_SAMPLES		30U    // Δvを取る窓長[サンプル]（リングバッファ長）
#define SLIP_SAMPLE_PERIOD_S	DEFF_TIME // サンプリング周期[s]（1ms）
#define SLIP_SPEED_SKIP_MPS		0.1f	// 超低速時に判定をスキップする速度閾値[m/s]
#define SLIP_MISMATCH_HIGH		16.0f   // [m/s^2]
#define SLIP_MISMATCH_LOW		12.0f   // [m/s^2]
// 旋回中だけ上げる閾値
#define SLIP_MISMATCH_HIGH_TURN		2.2f 	// 係数倍
#define SLIP_MISMATCH_LOW_TURN		2.2f	// 係数倍
#define SLIP_HIGH_COUNT_REQ			6U		// 上側しきい値を超え続ける必要サンプル数
#define SLIP_LOW_COUNT_REQ			6U		// 下側しきい値を下回り続ける必要サンプル数
#define SLIP_LPF_COEF				0.015f	// スリップ指標に掛ける一次LPF係数
#define SLIP_LPF_COEF_LAT			0.05f	// Lat指標は速め（~20ms程度を想定）
#define SLIP_HIGH_COUNT_REQ_LAT		5U		// Lat ONは5ms連続で良い
#define SLIP_LOW_COUNT_REQ_LAT		5U		// Lat OFFも5ms連続（まずは同じ）

#define SLIP_ACC_BIAS_COEF		0.001f   // ≒ dt/1s（1秒時定数くらい）
#define SLIP_ACC_LPF_COEF		0.03f    // ≒ 20ms前後のLPF

#define SLIP_GYRO_ON_DPS		250.0f
#define SLIP_GYRO_OFF_DPS		220.0f
#define SLIP_GYRO_ON_RADS		(DPS2RADS(SLIP_GYRO_ON_DPS))
#define SLIP_GYRO_OFF_RADS		(DPS2RADS(SLIP_GYRO_OFF_DPS))

#define SLIP_LAT_HIGH			6.0f   // [m/s^2] 横滑りON
#define SLIP_LAT_LOW			4.5f   // [m/s^2] 横滑りOFF（ヒステリシス）
// 横滑り判定を有効にする最小の推定横加速度（encAy=wz*v、小さい旋回やノイズで反応しないため）
#define SLIP_LAT_ENCAY_MIN		3.0f   // [m/s^2]
// 横滑りの誤検知抑制用（PWM/電流で惰性を判定）
#define SLIP_PWM_LAT_COUNT_MIN	150.0f // 横判定カウントを許可する最低PWM合計（LPF後）
#define SLIP_PWM_COAST_MAX		150.0f // 惰性判定PWM合計（LPF後）
#define SLIP_ISUM_COAST_MAX		0.12f  // 惰性判定電流合計[A]（LPF後）
#define SLIP_ISUM_LAT_COUNT_MIN	0.80f  // [A] Lat ON判定に使う瞬時電流和の下限（要ログで再調整）
#define SLIP_ISUM_LAT_COUNT_N	3U     // [count] iSumが閾値以上である連続サンプル数（短スパイク除去）
#define SLIP_LAT_CLEAR_COEF		0.10f  // 惰性時に横指標を0へ戻す係数（SLIP_LPF_COEFより強め）
#define SLIP_PWM_LPF_COEF		0.02f  // PWM合計のLPF係数

#define SLIP_CUR_ENABLE            1
#define SLIP_CUR_LPF_COEF          0.05f   // 電流LPF（0〜1）
#define SLIP_CUR_BASE_A            0.60f   // ここまでは補正しない基準電流[A]
#define SLIP_CUR_K                 0.35f   // 補正ゲイン [1/A]
#define SLIP_CUR_MAX_SCALE         1.25f   // 閾値スケール上限
#define SLIP_CUR_MIN_A             0.10f   // ほぼ惰性/停止時は補正を無効化

// スリップ距離補正パラメータ
#define SLIP_DIST_CORRECTION_ENABLE	0		// encTotalOptimal補正の有効/無効
#define SLIP_DIST_MIN_SCALE			0.8f	// 要調整
#define SLIP_DIST_MIN_SCALE_LAT		0.9f	// 要調整
#define SLIP_DIST_LPF_COEF_DOWN		0.15f	// 悪化追従
#define SLIP_DIST_LPF_COEF_UP		0.06f	// 回復追従

// ゴール
#define COUNT_GOAL 6 // ゴールマーカーを読む回数

// スタートモード
#define START_SERACH 3
#define START_OPTIMAL 4

// クロスライン検出関連
#define TRACE_CROSSLINE_TH			2000	// クロスライン検出閾値
#define TRACE_CROSSLINE_DISTANCE	60		// クロスライン付近でゲインを変更する距離[mm]

// 走行中ゲイン変更関連
#define GAIN_CHANGE_ROC_ENTER			600		// カーブ用ゲインへ切替えるROC閾値[mm]（小さいほどカーブ）
#define GAIN_CHANGE_ROC_EXIT			600		// 直線用ゲインへ戻すROC閾値[mm]（大きいほど直線）
#define GAIN_CHANGE_LOOKAHEAD_ENTER		4		// カーブ進入判定用の先読みインデックス
#define GAIN_CHANGE_LOOKAHEAD_EXIT		2		// 直線復帰判定用の先読みインデックス
#define GAIN_CHANGE_STRAIGHT_STABLE_MM	50		// 直線安定がこの距離続いたら直線用ゲインへ復帰

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
	float decelLeadMm;
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
extern float rocrun; 			// 曲率半径計算用変数

extern uint16_t analogValLSon[11]; // ADC結果格納配列
extern uint16_t analogValLSoff[11]; // ADC結果格納配列
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
void changeGain(void);
void getADC2(void);
void setEncoderVal(void);
void writeTgtspeeds(void);
void readTgtspeeds(void);
void checkCrossLine(void);
void updateSlipDetection(void);
void Control_ApplyMarkerCorrection_p(int32_t diff_p);
float getSlipIndicatorRaw(void);
float getSlipIndicatorFiltered(void);
bool getSlipFlag(void);
bool getSlipFlagLat(void);
// スリップ距離補正（パルス版）
int32_t Control_GetEncCurrentCorr_p(void);
int32_t Control_GetDistEncRaw_p(void);
int32_t Control_GetDistCorr_p(void);
int32_t Control_GetDistSlipLoss_p(void);
float Control_GetSlipDistScale(void);
float Control_GetSlipDistScaleRaw(void);

#endif // CONTROL_H_
