#ifndef SETUP_H_
#define SETUP_H_
//======================================//
// インクルード
//======================================//
#include "main.h"
//======================================//
// グローバル変数の宣言
//======================================//
#define UD 0 // 5方向タクトスイッチの上下方向
#define LR 1 // 5方向タクトスイッチの左右方向

#define PUSHTIME 5 // 長押し判定時間[x50ms]

#define CALIBRATIONSPEED -1500.0F // ラインセンサのキャリブレーション時の角速度[rad/s]

#define HEX_START 0x0
#define HEX_SPEED_PARAM 0x1
#define HEX_LOG 0x2
#define HEX_CALIBRATION 0x3
#define HEX_SENSORS 0x4
#define HEX_PID_TRACE 0x5
#define HEX_PID_SPEED 0x6
#define HEX_PID_ANGULAR 0x7
#define HEX_PID_ANGLE 0x8
#define HEX_PID_DIST 0x9

// パターン構造体
typedef struct
{
        int16_t display;                // 表示パターン
        int16_t sensors;                // センサメニュー
        int16_t beforeSensors;          // 前回のセンサメニュー
        uint8_t beforeHex;              // 前回の表示HEX
        int16_t sensorLine;             // ラインセンサ項目
        int16_t sensorAccele;           // 加速度センサ項目
        int16_t sensorGyro;             // ジャイロセンサ項目
        int16_t parameter1;             // パラメータ1
        int16_t parameter2;             // パラメータ2
        int16_t parameter3;             // パラメータ3
        int16_t parameter4;             // パラメータ4
        int16_t gain;                   // ゲイン選択
        int16_t speedseting;            // 速度設定
        int16_t log;                    // ログメニュー
        int16_t calibration;            // キャリブレーション
        int16_t click;                  // クリックメニュー
        // 新しいパターンを追加する場合はここにメンバとコメントを追加
} Pattern;

//======================================//
// グローバル変数の宣言
//======================================//
// パターン関連
extern uint8_t start;
extern Pattern pattern;

// タイマ関連
extern uint16_t cntSetup1;
extern uint16_t cntSetup2;
extern uint16_t cntSwitchUD;	 // スイッチ判定用右
extern uint16_t cntSwitchLR;	 // スイッチ判定用左
extern uint16_t cntSwitchUDLong; // スイッチ長押し判定用右
extern uint16_t cntSwitchLRLong; // スイッチ長押し判定用左

// パラメータ関連
extern uint8_t fixSpeed;
extern int32_t encClick;

// フラグ関連
extern uint8_t trace_test;

//======================================//
// プロトタイプ宣言
//======================================//
void setup(void);
void data_select(uint8_t *data, uint8_t button);
void dataTuningUD(int16_t *data, int16_t add, int16_t min, int16_t max);
void dataTuningLR(int16_t *data, int16_t add, int16_t min, int16_t max);
void dataTuningUDF(float *data, float add, float min, float max);
void setupNonDisp(void);
void wheelClick(void);

#endif /* SETUP_H_ */
