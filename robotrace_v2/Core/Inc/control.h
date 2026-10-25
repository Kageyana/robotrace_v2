#ifndef CONTROL_H_
#define CONTROL_H_
//====================================//
// インクルード
//====================================//
#include "main.h"
#include "speedParameters.h"
#include "SDcard.h"
#include "IMU.h"
#include <stdbool.h>
#include <stdint.h>
//====================================//
// 調整パラメータ
//====================================//
#define DISPLAY_INIT_DMA_TIMEOUT_MS 200U // 初期化・設定画面で表示DMAの完了を待つ上限[ms]。

// 速度パラメータの既定値。有効なSD設定値を優先する。
#define PARAM_SEARCH 1.0F // 一次走行の速度の既定値[m/s]。
#define PARAM_STOP 0.8F // ゴール後の停止動作に使う速度の既定値[m/s]。
#define PARAM_SHORTCUT 0.5F // SHORTCUT走行の速度上限の既定値[m/s]。
#define PARAM_PATH_REPLAY 1.0F // PATH走行の速度上限の既定値[m/s]。
#define PARAM_BOOST_STRAIGHT 2.8F // 直線区間の速度の既定値[m/s]。
#define PARAM_BOOST_1500 2.6F // 曲率半径1500mmの区間の速度の既定値[m/s]。
#define PARAM_BOOST_1300 2.4F // 曲率半径1300mmの区間の速度の既定値[m/s]。
#define PARAM_BOOST_1000 2.2F // 曲率半径1000mmの区間の速度の既定値[m/s]。
#define PARAM_BOOST_800 2.0F // 曲率半径800mmの区間の速度の既定値[m/s]。
#define PARAM_BOOST_700 1.8F // 曲率半径700mmの区間の速度の既定値[m/s]。
#define PARAM_BOOST_600 1.6F // 曲率半径600mmの区間の速度の既定値[m/s]。
#define PARAM_BOOST_500 1.5F // 曲率半径500mmの区間の速度の既定値[m/s]。
#define PARAM_BOOST_400 1.4F // 曲率半径400mmの区間の速度の既定値[m/s]。
#define PARAM_BOOST_300 1.3F // 曲率半径300mmの区間の速度の既定値[m/s]。
#define PARAM_BOOST_200 1.2F // 曲率半径200mmの区間の速度の既定値[m/s]。
#define PARAM_BOOST_100 1.1F // 曲率半径100mmの区間の速度の既定値[m/s]。
#define PARAM_DECEL_LEAD_MM 30.0F // DISTANCE速度計画の減速先行距離の既定値[mm]。

#define PARAM_UP_STEP 1.02F // 速度パラメータ増加用の倍率[無次元]。現行コードでは未使用。

// ゴール
#define COUNT_GOAL 6 // 停止動作へ移るゴールマーカー検出回数[回]。

// クロスライン検出関連
#define TRACE_CROSSLINE_TH			2000 // クロスライン検出に使うラインセンサーの閾値[正規化AD値]。
#define TRACE_CROSSLINE_DISTANCE	60 // クロスライン付近でゲインを変更する距離[mm]。

// 走行中ゲイン変更関連
#define GAIN_CHANGE_ROC_ENTER			500 // カーブ用ゲインへ切り替える曲率半径の閾値[mm]。
#define GAIN_CHANGE_ROC_EXIT			600 // 直線用ゲインへ戻す曲率半径の閾値[mm]。
#define GAIN_CHANGE_LOOKAHEAD_ENTER		4 // カーブ進入判定で先読みする速度計画の要素数[点]。
#define GAIN_CHANGE_LOOKAHEAD_EXIT		2 // 直線復帰判定で先読みする速度計画の要素数[点]。
#define GAIN_CHANGE_STRAIGHT_STABLE_MM	20 // 直線用ゲインへの復帰に必要な連続直線距離[mm]。

//====================================//
// シンボル定義
//====================================//
// スタートモード
#define START_SERACH 3
#define START_OPTIMAL 4

// ファイル名
#define FILENAME_TARGET_SPEED "targetSpeeds"


//====================================//
// グローバル変数の宣言、定義
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
void emergencyStop(void);
void countDown(void);
void changeGain(void);
void getADC2(void);
void setEncoderVal(void);
void writeTgtspeeds(void);
void readTgtspeeds(void);
void checkCrossLine(void);
void Control_ApplyMarkerCorrection_p(int32_t diff_p);
int32_t Control_GetEncCurrentCorr_p(void);

#endif // CONTROL_H_
