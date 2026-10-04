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
#define TYPE_INT16 0 // int16_t型
#define TYPE_FLOAT 1 // float型

#define PUSHTIME 5 // 長押し判定時間[x50ms]
#define WHEEL_CLICK 800 // ホイールクリックのパルス数

// 校正の調整項目。復帰は既存ラインPIDを使用し、探索速度・制動解除幅は算出する。
#define CALIBRATIONSPEED -800.0F // 校正旋回の角速度[deg/s]
#define AUTO_CALIBRATION_RETURN_SPEED -300.0F // 白線捕捉前の復帰最大角速度[deg/s]、負値で指定
#define AUTO_CALIBRATION_SEARCH_WINDOW_DEG 30.0F // 元姿勢付近の探索幅[deg]、捕捉後は2倍
#define AUTO_CALIBRATION_LINE_THRESHOLD 2000U // 中央左センサー4の白線捕捉閾値[正規化AD値]
#define AUTO_CALIBRATION_MIN_CONTRAST 500U // 追従に必要な最大・最小値の差[正規化AD値]
#define AUTO_CALIBRATION_LINE_MAX_WIDTH 4U // 白線候補の最大センサー数[個]
#define AUTO_CALIBRATION_POSITION_TOLERANCE 0.35F // 制動を開始する中央からの偏差[センサー間隔]、0以上0.5未満
#define AUTO_CALIBRATION_STOP_RATE_DPS 250.0F // 復帰判定の角速度上限[deg/s]、250では静止を保証しない
#define AUTO_CALIBRATION_SETTLE_MS 50U // 復帰条件の連続成立時間[ms]

#define HEX_START 0x0
#define HEX_SPEED_PARAM 0x1
#define HEX_LOG 0x2
#define HEX_CALIBRATION 0x3
#define HEX_SENSORS 0x4
#define HEX_PID_TRACE 0x5
#define HEX_PID_TRACE_OMEGA 0x6
#define HEX_PID_SPEED 0x7
#define HEX_PID_ANGULAR 0x8

// センサテストを表す列挙体
typedef enum
{
	// 1から順にテストIDを割り当てる
	TEST_MOTOR = 1,       // モータテスト
	TEST_IMU_DEG,         // IMU角度表示
	TEST_IMU_ACCEL,       // IMU加速度表示
	TEST_MARKER,          // マーカーセンサ
	TEST_SWITCH,          // タクトスイッチ
	TEST_BATTERY,         // バッテリ電圧
	TEST_LINESENSOR,      // ラインセンサ
	TEST_RGBLED           // RGBLED
} SensorTestId;

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

// セットアップ状態を管理するフラグ構造体
typedef struct
{
	uint8_t start;       // 0:セットアップ中 1:セットアップ完了
	int8_t clickStart;   // スタート方向
} SetupFlags;

// テスト関連のフラグ構造体
typedef struct
{
	uint8_t motor_test;      // モータテスト
	uint8_t trace_test;      // トレーステスト
	uint8_t beforeMotorTest; // テスト状態保存用
	uint8_t lineSensor_test; // ラインセンサテスト
} TestFlags;

//======================================//
// グローバル変数の宣言
//======================================//
// パターン関連
extern SetupFlags setupFlags;
extern Pattern pattern;

// パラメータ関連
extern uint8_t fixSpeed;
extern int32_t encClick;

// フラグ関連
extern TestFlags testFlags;
extern volatile bool autoStartCalibrationTurning; // 自動開始前の旋回校正中だけ角速度PIDを更新


//======================================//
// プロトタイプ宣言
//======================================//
void setup(void);
void data_select(uint8_t *data, uint8_t button);
void dataTuning(void *data, float add, float min, float max, uint8_t dir, uint8_t type);
void dataTuningUD(int16_t *data, int16_t add, int16_t min, int16_t max);
void dataTuningLR(int16_t *data, int16_t add, int16_t min, int16_t max);
void dataTuningUDF(float *data, float add, float min, float max);
void setupNonDisp(void);
void wheelClick(void);
void setupCount(void);
void setupCalibrationTurn1ms(void);

#endif /* SETUP_H_ */
