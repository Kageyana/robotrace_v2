//====================================//
// インクルード
//====================================//
#include "setup.h"
//====================================//
// グローバル変数の宣言
//====================================//
SetupFlags setupFlags = {0}; // セットアップ状態を管理

// タイマ関連のカウンタをまとめた構造体
typedef struct
{
	uint16_t cntSetup1;           // セットアップで使用
	uint16_t cntSetup2;           // セットアップで使用
	uint16_t cntSwitchUD;         // スイッチ判定用右
	uint16_t cntSwitchLR;         // スイッチ判定用左
	uint16_t cntSwitchUDLong;     // スイッチ長押し判定用右
	uint16_t cntSwitchLRLong;     // スイッチ長押し判定用左
} SetupTimer; // タイマ関連構造体

static SetupTimer setupTimer = {0}; // タイマ関連変数


// スイッチ関連
int8_t pushLR = 0;
int8_t pushUD = 0;

// パターン関連
uint8_t push1 = 0;
Pattern pattern = {
	.display = 0,
	.sensors = TEST_MOTOR, // 初期選択:モータテスト
	.beforeSensors = 0,
	.beforeHex = 255,
	.sensorLine = 1,
	.sensorAccele = 1,
	.sensorGyro = 1,
	.parameter1 = 1,
	.parameter2 = 1,
	.parameter3 = 1,
	.parameter4 = 1,
	.gain = 3,
	.speedseting = 1,
	.log = 1,
	.calibration = 1,
	.click = 1
}; // パターンの状態を保持
// フラグ関連
TestFlags testFlags = {0};

// パラメータ関連
int16_t motorTestPwm = 200;
int32_t encClick = 0;
//======================================//
// プロトタイプ宣言
//======================================//
static void setup_sensors(void); 		// センサ表示とテストメニューを制御する処理
static void setup_pid_trace(void);		// ゲイン調整(直線トレース)
static void setup_pid_dist(void);		// ゲイン調整(距離)
static void setup_pid_angle(void);		// ゲイン調整(角度)
static void setup_pid_angular(void);	// ゲイン調整(角速度)
static void setup_pid_speed(void);		// ゲイン調整(速度)
static void setup_start(void);			// スタート待ち画面とキャリブレーションを制御する処理
static void setup_log(void);			// ログ解析と表示を制御
static void setup_calibration(void);	// キャリブレーション(ラインセンサ)
static void setup_speed_param(void);	// 速度パラメータ調整
static void test_motor(void);			// モータテスト
static void test_imu_deg(void);			// IMU角度表示
static void test_imu_accel(void);		// IMU加速度表示
static void test_marker(void);			// マーカーセンサ
static void test_switch(void); 			// タクトスイッチ
static void test_battery(void);			// バッテリ電圧
static void test_linesensor(void); 		// ラインセンサ
static void test_rgbled(void); 			// RGBLED
static void init_sensor_test(const char* title, FontDef font, uint8_t x); // 表示初期化

// センサテストで使用する関数ポインタ型
typedef void (*SensorTestFunc)(void);
// テストIDと実行関数を保持する構造体
typedef struct
{
	SensorTestId id; // テスト種別
	SensorTestFunc func; // 実行関数
} SensorTest; // センサテスト情報

// センサテストのテーブル {列挙体, 関数ポインタ}
static const SensorTest sensorTestTable[] = {
	{TEST_MOTOR, test_motor}, // モータテスト
	{TEST_IMU_DEG, test_imu_deg}, // IMU角度表示
	{TEST_IMU_ACCEL, test_imu_accel}, // IMU加速度表示
	{TEST_MARKER, test_marker}, // マーカーセンサ
	{TEST_SWITCH, test_switch}, // タクトスイッチ
	{TEST_BATTERY, test_battery}, // バッテリ電圧
	{TEST_LINESENSOR, test_linesensor}, // ラインセンサ
	{TEST_RGBLED, test_rgbled} // RGBLED
}; // IDと処理の対応テーブル

// 速度パラメータの情報を保持する構造体
typedef struct
{
	const char* label; // 表示文字列
	float* value; // 対応する変数ポインタ
	float step; // 増分値
	float min; // 最小値
	float max; // 最大値
	const char* unit; // 表示単位
} SpeedParamInfo; // 速度パラメータ情報

// 速度パラメータの一覧テーブル
static const SpeedParamInfo speedParamTable[] = {
	{"STRAIGHT", &tgtParam.straight, 0.1, 0.0, 10.0, "m/s"},
	{"CURVE", &tgtParam.curve, 0.1, 0.0, 10.0, "m/s"},
	{"STOP", &tgtParam.stop, 0.1, 0.0, 10.0, "m/s"},
	{"BST STRT", &tgtParam.bstStraight, 0.1, 0.0, 10.0, "m/s"},
	{"BST 1500", &tgtParam.bst1500, 0.1, 0.0, 10.0, "m/s"},
	{"BST 1300", &tgtParam.bst1300, 0.1, 0.0, 10.0, "m/s"},
	{"BST 1000", &tgtParam.bst1000, 0.1, 0.0, 10.0, "m/s"},
	{"BST 800", &tgtParam.bst800, 0.1, 0.0, 10.0, "m/s"},
	{"BST 700", &tgtParam.bst700, 0.1, 0.0, 10.0, "m/s"},
	{"BST 600", &tgtParam.bst600, 0.1, 0.0, 10.0, "m/s"},
	{"BST 500", &tgtParam.bst500, 0.1, 0.0, 10.0, "m/s"},
	{"BST 400", &tgtParam.bst400, 0.1, 0.0, 10.0, "m/s"},
	{"BST 300", &tgtParam.bst300, 0.1, 0.0, 10.0, "m/s"},
	{"BST 200", &tgtParam.bst200, 0.1, 0.0, 10.0, "m/s"},
	{"BST 100", &tgtParam.bst100, 0.1, 0.0, 10.0, "m/s"},
	{"BST acceleF", &tgtParam.acceleF, 0.1, 0.0, 10.0, "m/ss"},
	{"BST acceleD", &tgtParam.acceleD, 0.1, 0.0, 10.0, "m/ss"},
	{"BST shortCut", &tgtParam.shortCut, 0.1, 0.0, 10.0, "m/s"}
}; // 速度パラメータの対応テーブル
///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 setup_speed_param
// 処理概要     速度パラメータ調整
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
static void setup_speed_param(void)
{
	static uint8_t beforePparam = 0; // 前回のパラメータ項目を保持

	if (pattern.display != pattern.beforeHex)
	{
		// 切替時に実行
		ssd1306_printf(Font_6x8, "Parameter");
	}

	dataTuningLR(&pattern.parameter1, 1, 1, 18); // パラメータ項目切替

	if (beforePparam != pattern.parameter1)
	{
		ssd1306_FillRectangle(0, 16, 127, 63, Black);
	}

	// 選択番号に対応する速度パラメータを処理
	for (uint8_t i = 0; i < sizeof(speedParamTable) / sizeof(speedParamTable[0]); i++)
	{
		if (pattern.parameter1 == i + 1)
		{
			dataTuningUDF(speedParamTable[i].value, speedParamTable[i].step, speedParamTable[i].min, speedParamTable[i].max);
			ssd1306_SetCursor(0, 24);
			ssd1306_printf(Font_6x8, "%s:%3gm/%s", speedParamTable[i].label, *speedParamTable[i].value, speedParamTable[i].unit);
			break;
		}
	}

	beforePparam = pattern.parameter1; // 選択項目を記録
}

///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 init_sensor_test
// 処理概要     センサテスト画面の初期化
// 引数         title : 表示文字列
//              font  : 使用フォント
//              x     : タイトル表示位置
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
static void init_sensor_test(const char* title, FontDef font, uint8_t x)
{
	ssd1306_FillRectangle(0, 16, 127, 63, Black); // 表示領域をクリア
	ssd1306_SetCursor(x, 16); // タイトルの位置設定
	ssd1306_printf(font, title); // タイトルを描画
}

///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 test_motor
// 処理概要     モータテスト
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
static void test_motor(void)
{
	if (pattern.sensors != pattern.beforeSensors)
	{
		// 切替時に実行
		init_sensor_test("Motor", Font_6x8, 47); // 画面クリアとタイトル表示
		testFlags.motor_test = 0;
	}
	// Duty表示
	ssd1306_SetCursor(35, 30);
	ssd1306_printf(Font_6x8, "Duty:%4d", motorTestPwm);

	// Left
	ssd1306_SetCursor(0, 42);
	ssd1306_printf(Font_6x8, "enc:%5.0f", encTotalL / PALSE_MILLIMETER); // Encoder
	ssd1306_SetCursor(0, 52);
	ssd1306_printf(Font_6x8, "Cur:%5.2f", motorCurrentL); // Current

	// // Right
	ssd1306_SetCursor(70, 42);
	ssd1306_printf(Font_6x8, "enc:%5.0f", encTotalR / PALSE_MILLIMETER); // Encoder
	ssd1306_SetCursor(70, 52);
	ssd1306_printf(Font_6x8, "Cur:%5.2f", motorCurrentR); // Current

	dataTuningUD(&motorTestPwm, 100, -500, 500); // PWM値を調整
	data_select(&testFlags.motor_test, SW_PUSH); // モータテストの開始/停止
	if (testFlags.motor_test == 1)
	{
		motorPwmOut(motorTestPwm, motorTestPwm);
	}
	else
	{
		motorPwmOut(0, 0);
	}

	// testFlags.motor_test 1→0のとき 2にする
	if (testFlags.motor_test != testFlags.beforeMotorTest && testFlags.motor_test == 0)
	{
		testFlags.motor_test = 2;
	}
	// 2のときホイールの回転が止まったらtestFlags.motor_test=0にする
	if (testFlags.motor_test == 2 && encCurrentL == 0)
	{
		testFlags.motor_test = 0;
	}
	testFlags.beforeMotorTest = testFlags.motor_test; // 次回比較用に状態を保存
}

///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 test_imu_deg
// 処理概要     IMU角度表示
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
static void test_imu_deg(void)
{
	if (pattern.sensors != pattern.beforeSensors)
	{
		// 切替時に実行
		init_sensor_test("IMU[deg]", Font_7x10, 36); // 画面クリアとタイトル表示
		testFlags.motor_test = 1;
	}

	if (!calibratIMU)
	{
		ssd1306_SetCursor(64, 30);
		ssd1306_printf(Font_7x10, "xd:%6.1f", BMI088val.angle.x);
		ssd1306_SetCursor(64, 42);
		ssd1306_printf(Font_7x10, "yd:%6.1f", BMI088val.angle.y);
		ssd1306_SetCursor(64, 54);
		ssd1306_printf(Font_7x10, "zd:%6.1f", BMI088val.angle.z);
	}

	if (swValTact == SW_PUSH)
	{
		BMI088val.angle.x = 0;
		BMI088val.angle.y = 0;
		BMI088val.angle.z = 0;
	}

	if (swValTact == SW_UP)
	{
		ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
		ssd1306_SetCursor(22, 28);
		ssd1306_printf(Font_7x10, "Calibration");
		ssd1306_SetCursor(53, 42);
		ssd1306_printf(Font_7x10, "Now");
		ssd1306_UpdateScreen();

		calibratIMU = true;
		HAL_Delay(1000);
	}
}

///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 test_imu_accel
// 処理概要     IMU加速度表示
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
static void test_imu_accel(void)
{
	if (pattern.sensors != pattern.beforeSensors)
	{
		// 切替時に実行
		init_sensor_test("IMU[g]", Font_7x10, 36); // 画面クリアとタイトル表示
	}

	ssd1306_SetCursor(0, 30);
	ssd1306_printf(Font_7x10, "xa:%6.1f", BMI088val.accele.x);
	ssd1306_SetCursor(0, 42);
	ssd1306_printf(Font_7x10, "ya:%6.1f", BMI088val.accele.y);
	ssd1306_SetCursor(0, 54);
	ssd1306_printf(Font_7x10, "za:%6.1f", BMI088val.accele.z);

	ssd1306_SetCursor(64, 30);
	ssd1306_printf(Font_7x10, "T:%4.1f", BMI088val.temp);
}

///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 test_marker
// 処理概要     マーカーセンサ
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
static void test_marker(void)
{
	if (pattern.sensors != pattern.beforeSensors)
	{
		// 切替時に実行
		init_sensor_test("Marker sensors", Font_7x10, 15); // 画面クリアとタイトル表示
	}
	ssd1306_SetCursor(0, 30);
	ssd1306_printf(Font_7x10, "sensors:%d", getMarkerSensor());
	ssd1306_SetCursor(0, 45);
	ssd1306_printf(Font_7x10, "britght:%d", testFlags.motor_test);

	data_select(&testFlags.motor_test, SW_PUSH);
	if (testFlags.motor_test == 1)
	{
		powerMarkerSensors(1);
	}
	else
	{
		powerMarkerSensors(0);
	}
}

///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 test_switch
// 処理概要     タクトスイッチ
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
static void test_switch(void)
{
	if (pattern.sensors != pattern.beforeSensors)
	{
		// 切替時に実行
		init_sensor_test("Switches", Font_7x10, 32); // 画面クリアとタイトル表示
	}
	ssd1306_SetCursor(0, 30);
	ssd1306_printf(Font_7x10, "Board SW:%d", swValMainTact);

	ssd1306_SetCursor(0, 42);
	ssd1306_printf(Font_7x10, "5axis SW:%d", swValTact);
}

///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 test_battery
// 処理概要     バッテリ電圧
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
static void test_battery(void)
{
	if (pattern.sensors != pattern.beforeSensors)
	{
		// 切替時に実行
		init_sensor_test("Battery", Font_7x10, 32); // 画面クリアとタイトル表示
	}
	ssd1306_SetCursor(0, 30);
	ssd1306_printf(Font_7x10, "batteryADAD:%d", batteryAD);

	ssd1306_SetCursor(0, 42);
	ssd1306_printf(Font_7x10, "BatteryLv:%d", batteryLevel);
}

///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 test_linesensor
// 処理概要     ラインセンサ
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
static void test_linesensor(void)
{
	if (pattern.sensors != pattern.beforeSensors)
	{
		// 切替時に実行
		init_sensor_test("", Font_6x8, 0); // 画面クリア
		// センサ基板形状
		ssd1306_DrawArc(64, 81, 66, 90, 270, White);
		ssd1306_DrawArc(64, 81, 35, 90, 270, White);
		ssd1306_Line(2, 63, 34, 63, White);
		ssd1306_Line(93, 63, 126, 63, White);
		testFlags.motor_test = 0;
	}

	ssd1306_SetCursor(37, 22);
	ssd1306_printf(Font_6x8, "%4d", lSensorCari[4]);
	ssd1306_SetCursor(31, 30);
	ssd1306_printf(Font_6x8, "%4d", lSensorCari[3]);
	ssd1306_SetCursor(22, 38);
	ssd1306_printf(Font_6x8, "%4d", lSensorCari[2]);
	ssd1306_SetCursor(13, 46);
	ssd1306_printf(Font_6x8, "%4d", lSensorCari[1]);
	ssd1306_SetCursor(6, 54);
	ssd1306_printf(Font_6x8, "%4d", lSensorCari[0]);

	ssd1306_SetCursor(65, 22);
	ssd1306_printf(Font_6x8, "%4d", lSensorCari[5]);
	ssd1306_SetCursor(71, 30);
	ssd1306_printf(Font_6x8, "%4d", lSensorCari[6]);
	ssd1306_SetCursor(80, 38);
	ssd1306_printf(Font_6x8, "%4d", lSensorCari[7]);
	ssd1306_SetCursor(89, 46);
	ssd1306_printf(Font_6x8, "%4d", lSensorCari[8]);
	ssd1306_SetCursor(95, 54);
	ssd1306_printf(Font_6x8, "%4d", lSensorCari[9]);

	data_select(&testFlags.motor_test, SW_PUSH);
	if (testFlags.motor_test == 1)
	{
		powerLineSensors(1);
	}
	else
	{
		powerLineSensors(0);
	}
}

///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 test_rgbled
// 処理概要     RGBLED
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
static void test_rgbled(void)
{
	if (pattern.sensors != pattern.beforeSensors)
	{
		// 切替時に実行
		init_sensor_test("RGBLED", Font_7x10, 43); // 画面クリアとタイトル表示
	}

	data_select(&testFlags.motor_test, SW_PUSH);
	if (testFlags.motor_test == 1)
	{
		if (setupTimer.cntSetup2 > 50)
		{
			fullColorLED(10, 4);
			setupTimer.cntSetup2 = 0;
		}
	}

	if (testFlags.motor_test != testFlags.beforeMotorTest)
	{
		clearLED();
	}

	testFlags.beforeMotorTest = testFlags.motor_test;
}
/////////////////////////////////////////////////////////////////////////////////////
// モジュール名 setup_sensors
// 処理概要     センサ表示とテストメニューを制御
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////////////////////
static void setup_sensors(void)
{
	if (pattern.display != pattern.beforeHex)
	{
		// ページ切替時の初期処理
		ssd1306_printf(Font_6x8, "SENSORS  ");  // センサ画面表示
		pattern.beforeSensors = 100;    // 初期値
	}

        // センサメニューの項目切替（範囲は列挙体で指定）
        dataTuningLR(&pattern.sensors, 1, TEST_MOTOR, TEST_RGBLED); // センサメニュー項目切替
        // テーブルを走査して該当テストを探索
        for (uint8_t i = 0; i < sizeof(sensorTestTable) / sizeof(sensorTestTable[0]); i++)
        {
			// テーブルから選択IDに対応するテストを探索
			if (pattern.sensors == sensorTestTable[i].id)
			{
				sensorTestTable[i].func(); // 選択されたテストを実行
				break; // 一致したらループ終了
			}
        }
	pattern.beforeSensors = pattern.sensors;        // 選択状態の更新
}
/////////////////////////////////////////////////////////////////////////////////////
// モジュール名 setup_pid_dist
// 処理概要     ゲイン調整(距離)
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////////////////////
static void setup_pid_dist(void)
{
	if (pattern.display != pattern.beforeHex)
	{
		// 切替時に実行
		ssd1306_printf(Font_6x8, "Dist PID");

		ssd1306_SetCursor(0, 18);
		ssd1306_printf(Font_7x10, "kp:");
		ssd1306_SetCursor(0, 32);
		ssd1306_printf(Font_7x10, "ki:");
		ssd1306_SetCursor(0, 44);
		ssd1306_printf(Font_7x10, "kd:");
		ssd1306_SetCursor(60, 30);
		ssd1306_printf(Font_7x10, "pwm:");

		// 距離制御テスト用初期値
		setTargetDist(50.0);		// 目標距離を設定[mm]
		setTargetSpeed(0.3);		// 目標速度を設定[m/s]
	}

	// ゲイン表示
	dataTuningUD(&pattern.gain, 1, 3, 1);	// 上下ボタンで調整対象を選択
	if (testFlags.trace_test == 0)	// 動作開始前のみ調整を許可
	{
		ssd1306_SetCursor(21, 18);
		if (pattern.gain == 1)
			ssd1306_printfB(Font_7x10, "%3d", distCtrl.kp);
		else
			ssd1306_printf(Font_7x10, "%3d", distCtrl.kp);
		ssd1306_SetCursor(21, 32);
		if (pattern.gain == 2)
			ssd1306_printfB(Font_7x10, "%3d", distCtrl.ki);
		else
			ssd1306_printf(Font_7x10, "%3d", distCtrl.ki);
		ssd1306_SetCursor(21, 44);
		if (pattern.gain == 3)
			ssd1306_printfB(Font_7x10, "%3d", distCtrl.kd);
		else
			ssd1306_printf(Font_7x10, "%3d", distCtrl.kd);

		// 制御量表示
		ssd1306_SetCursor(88, 30);
		ssd1306_printf(Font_7x10, "%4d", distCtrl.pwm);	// 出力PWM値

		switch (pattern.gain)	// 選択したゲインを変更
		{
		case 1:
			// kp
			dataTuningLR(&distCtrl.kp, 1, 0, 255);
			break;
		case 2:
			// ki
			dataTuningLR(&distCtrl.ki, 1, 0, 255);
			break;
		case 3:
			// kd
			dataTuningLR(&distCtrl.kd, 1, 0, 255);
			break;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 setup_pid_trace
// 処理概要     ゲイン調整(直線トレース)
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
static void setup_pid_trace(void)
{
	if (pattern.display != pattern.beforeHex)
	{
		// 切替時に実行
		ssd1306_printf(Font_6x8, "Trace PID");

		ssd1306_SetCursor(0, 18);
		ssd1306_printf(Font_7x10, "kp:");
		ssd1306_SetCursor(0, 32);
		ssd1306_printf(Font_7x10, "ki:");
		ssd1306_SetCursor(0, 44);
		ssd1306_printf(Font_7x10, "kd:");
		ssd1306_SetCursor(60, 30);
		ssd1306_printf(Font_7x10, "pwm:");
	}

	data_select(&testFlags.trace_test, SW_PUSH); // PUSHでトレースON/OFFの選択
	// PUSHでトレースON/OFF
	if (testFlags.trace_test == 1)
	{
		motorPwmOutSynth(lineTraceCtrl.pwm, 0, 0, 0); // モータを指定PWMで駆動
		powerLineSensors(1);                          // ラインセンサを有効化
	}
	else
	{
		motorPwmOutSynth(0, 0, 0, 0);                // モータ停止
		powerLineSensors(0);                         // ラインセンサ停止
	}
	if (testFlags.trace_test != testFlags.beforeMotorTest && testFlags.trace_test == 0)
	{
		testFlags.trace_test = 2;                              // 停止待機状態へ遷移
	}
	if (testFlags.trace_test == 2 && encCurrentL == 0) // ホイールの回転が停止したら0
	{
		testFlags.trace_test = 0;                              // 完全停止後に終了
	}
	testFlags.beforeMotorTest = testFlags.trace_test;                        // 状態を保存

	// ゲイン表示
	dataTuningUD(&pattern.gain, 1, 3, 1);
	if (testFlags.trace_test == 0)
	{
		ssd1306_SetCursor(21, 18);
		if (pattern.gain == 1)
			ssd1306_printfB(Font_7x10, "%3d", lineTraceCtrl.kp);
		else
			ssd1306_printf(Font_7x10, "%3d", lineTraceCtrl.kp);
		ssd1306_SetCursor(21, 32);
		if (pattern.gain == 2)
			ssd1306_printfB(Font_7x10, "%3d", lineTraceCtrl.ki);
		else
			ssd1306_printf(Font_7x10, "%3d", lineTraceCtrl.ki);
		ssd1306_SetCursor(21, 44);
		if (pattern.gain == 3)
			ssd1306_printfB(Font_7x10, "%3d", lineTraceCtrl.kd);
		else
			ssd1306_printf(Font_7x10, "%3d", lineTraceCtrl.kd);

		// 制御量表示
		ssd1306_SetCursor(88, 30);
		ssd1306_printf(Font_7x10, "%4d", lineTraceCtrl.pwm);

		switch (pattern.gain)
		{
		case 1:
			// kp
			dataTuningLR(&lineTraceCtrl.kp, 1, 0, 255);
			break;
		case 2:
			// ki
			dataTuningLR(&lineTraceCtrl.ki, 1, 0, 255);
			break;
		case 3:
			// kd
			dataTuningLR(&lineTraceCtrl.kd, 1, 0, 255);
			break;
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////
// モジュール名 setup_pid_angular
// 処理概要     ゲイン調整(角速度)
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////////////////////
static void setup_pid_angular(void)
{
	if (pattern.display != pattern.beforeHex)
	{
		// 切替時に実行
		ssd1306_printf(Font_6x8, "YawRate PID");

		ssd1306_SetCursor(0, 18);
		ssd1306_printf(Font_7x10, "kp:");
		ssd1306_SetCursor(0, 32);
		ssd1306_printf(Font_7x10, "ki:");
		ssd1306_SetCursor(0, 44);
		ssd1306_printf(Font_7x10, "kd:");
		ssd1306_SetCursor(60, 30);
		ssd1306_printf(Font_7x10, "pwm:");

		setTargetAngularVelocity(0); // 角速度制御の目標値を初期化
		setTargetSpeed(0);           // 速度制御の目標値を初期化
	}

	data_select(&testFlags.trace_test, SW_PUSH); // トレースON/OFFを選択
	// PUSHでトレースON/OFF
	if (testFlags.trace_test == 1)
	{
		motorPwmOutSynth(0, veloCtrl.pwm, yawRateCtrl.pwm, 0);
	}
	else
	{
		motorPwmOutSynth(0, 0, 0, 0);
	}
	if (testFlags.trace_test != testFlags.beforeMotorTest && testFlags.trace_test == 0)
	{
		testFlags.trace_test = 2;
	}
	if (testFlags.trace_test == 2 && encCurrentL == 0) // ホイールの回転が停止したら0
	{
		testFlags.trace_test = 0;
	}
	testFlags.beforeMotorTest = testFlags.trace_test;

	// ゲイン表示
	dataTuningUD(&pattern.gain, 1, 3, 1);
	if (testFlags.trace_test == 0)
	{
		ssd1306_SetCursor(21, 18);
		if (pattern.gain == 1)
			ssd1306_printfB(Font_7x10, "%3d", yawRateCtrl.kp);
		else
			ssd1306_printf(Font_7x10, "%3d", yawRateCtrl.kp);
		ssd1306_SetCursor(21, 32);
		if (pattern.gain == 2)
			ssd1306_printfB(Font_7x10, "%3d", yawRateCtrl.ki);
		else
			ssd1306_printf(Font_7x10, "%3d", yawRateCtrl.ki);
		ssd1306_SetCursor(21, 44);
		if (pattern.gain == 3)
			ssd1306_printfB(Font_7x10, "%3d", yawRateCtrl.kd);
		else
			ssd1306_printf(Font_7x10, "%3d", yawRateCtrl.kd);

		// 制御量表示
		ssd1306_SetCursor(88, 30);
		ssd1306_printf(Font_7x10, "%4d", yawRateCtrl.pwm);

		switch (pattern.gain)
		{
		case 1:
			// kp
			dataTuningLR(&yawRateCtrl.kp, 1, 0, 255);
			break;
		case 2:
			// ki
			dataTuningLR(&yawRateCtrl.ki, 1, 0, 255);
			break;
		case 3:
			// kd
			dataTuningLR(&yawRateCtrl.kd, 1, 0, 255);
			break;
		}
	}
}
///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 setup_pid_speed
// 処理概要     ゲイン調整(速度)
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
static void setup_pid_speed(void)
{
	if (pattern.display != pattern.beforeHex)
	{
		// 切替時に表示項目を初期化
		ssd1306_printf(Font_6x8, "Speed PID");

		ssd1306_SetCursor(0, 18);
		ssd1306_printf(Font_7x10, "kp:");
		ssd1306_SetCursor(0, 32);
		ssd1306_printf(Font_7x10, "ki:");
		ssd1306_SetCursor(0, 44);
		ssd1306_printf(Font_7x10, "kd:");
		ssd1306_SetCursor(60, 30);
		ssd1306_printf(Font_7x10, "pwm:");
	}

	data_select(&testFlags.trace_test, SW_PUSH); // PUSHでトレースON/OFFの選択
	// PUSHでトレースON/OFF
	if (testFlags.trace_test == 1)
	{
		// トレースON時の制御
		powerLineSensors(1); // ラインセンサを有効化
		setTargetSpeed(0.0); // 目標速度をリセット
		motorPwmOutSynth(lineTraceCtrl.pwm, veloCtrl.kp, 0, 0); // モータを指定PWMで駆動
	}
	else
	{
		// トレースOFF時の制御
		motorPwmOutSynth(0, 0, 0, 0); // モータ停止
		powerLineSensors(0);          // ラインセンサ停止
	}

	// ゲイン表示
	dataTuningUD(&pattern.gain, 1, 3, 1); // 調整対象のゲインを選択
	if (testFlags.trace_test == 0)
	{
		ssd1306_SetCursor(21, 18);
		if (pattern.gain == 1)
			ssd1306_printfB(Font_7x10, "%3d", veloCtrl.kp);
		else
			ssd1306_printf(Font_7x10, "%3d", veloCtrl.kp);
			ssd1306_SetCursor(21, 32);
		if (pattern.gain == 2)
			ssd1306_printfB(Font_7x10, "%3d", veloCtrl.ki);
		else
			ssd1306_printf(Font_7x10, "%3d", veloCtrl.ki);
			ssd1306_SetCursor(21, 44);
		if (pattern.gain == 3)
			ssd1306_printfB(Font_7x10, "%3d", veloCtrl.kd);
		else
			ssd1306_printf(Font_7x10, "%3d", veloCtrl.kd);

		// 制御量表示
		ssd1306_SetCursor(88, 30);
		ssd1306_printf(Font_7x10, "%4d", veloCtrl.pwm);

		// 選択したゲインを調整
		switch (pattern.gain)
		{
		case 1:
			// kpゲインを調整
			dataTuningLR(&veloCtrl.kp, 1, 0, 255);
			break;
		case 2:
			// kiゲインを調整
			dataTuningLR(&veloCtrl.ki, 1, 0, 255);
			break;
		case 3:
			// kdゲインを調整
			dataTuningLR(&veloCtrl.kd, 1, 0, 255);
			break;
		}
	}
}
/////////////////////////////////////////////////////////////////////////////////////
// モジュール名 setup_calibration
// 処理概要     キャリブレーション(ラインセンサ)
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////////////////////
static void setup_calibration(void)
{
	if (pattern.display != pattern.beforeHex)
	{
		// 切替時に実行
		ssd1306_printf(Font_6x8, "Calibrate");
		pattern.calibration = 1;
	}

	switch (pattern.calibration)
	{
	case 1: // スイッチ入力待ち
	{
		setTargetSpeed(0); // 速度をゼロに設定
		ssd1306_SetCursor(65, 22);
	ssd1306_printf(Font_6x8, "%4d", lSensorCari[0]);

		data_select(&testFlags.trace_test, SW_PUSH); // SW_PUSH入力を監視
		if (testFlags.trace_test)
		{
				setupTimer.cntSetup1 = 0; // カウンタリセット
				pattern.calibration = 2; // 次のステップへ
		}
		break;
	}
	case 2: // 開始準備
	{
               if (setupTimer.cntSetup1 > 1000) // 一定時間待機
               {
                       uint8_t i;
	ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
	ssd1306_SetCursor(22, 28);
	ssd1306_printf(Font_7x10, "Calibration");
	ssd1306_SetCursor(53, 42);
	ssd1306_printf(Font_7x10, "Now");
	ssd1306_UpdateScreen(); // グラフィック液晶更新

                       // 配列初期化
                       for (i = 0; i < NUM_SENSORS; i++)
                       {
                               lSensorMax[i] = 0;            // 最大値初期化
                               lSensorMin[i] = UINT16_MAX;   // 最小値初期化
                       }

                       powerLineSensors(1);    // ラインセンサ点灯
                       modeCalLinesensors = 1; // キャリブレーション開始

                       // 手動で機体を動かしキャリブレーションする

                       pattern.calibration = 3; // 次のステップへ
               }
               break;
       }
	case 3: // スイッチ押下で終了
	{
		data_select(&testFlags.trace_test, SW_PUSH); // SW_PUSH入力を監視
		if (!testFlags.trace_test) // スイッチが離されたら
		{
			modeCalLinesensors = 0;                                           // キャリブレーション終了
			powerLineSensors(0);                                              // ラインセンサ消灯
			ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
			ssd1306_UpdateScreen();                                           // グラフィック液晶更新

			if (initMSD)
			{
				writeLinesenval(); // オフセット値をSDカードに書き込み
			}
			pattern.calibration = 1; // 最初の状態に戻る
		}
		break;
	}

	default:
	    break;
	}
}
///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 setup_pid_angle
// 処理概要     ゲイン調整(角度)
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
static void setup_pid_angle(void)
{
	if (pattern.display != pattern.beforeHex)
	{
		// 切替時に実行
		ssd1306_printf(Font_6x8, "Yaw PID");

		ssd1306_SetCursor(0, 18);
		ssd1306_printf(Font_7x10, "kp:");
		ssd1306_SetCursor(0, 32);
		ssd1306_printf(Font_7x10, "ki:");
		ssd1306_SetCursor(0, 44);
		ssd1306_printf(Font_7x10, "kd:");
		ssd1306_SetCursor(60, 30);
		ssd1306_printf(Font_7x10, "pwm:");

		setTargetDist(50);      // PID調整用の走行距離
		setTargetSpeed(0.3);    // PID調整用の走行速度
	}

	data_select(&testFlags.trace_test, SW_PUSH);       // PUSHでトレースON/OFF
	// if ( testFlags.trace_test == 1 ) {
	//      motorPwmOutSynth( 0, veloCtrl.pwm, distCtrl.pwm, 0 );
	// } else {
	//      motorPwmOutSynth( 0, 0, 0, 0 );
	// }

	// 上下スイッチで調整対象のゲインを選択
	dataTuningUD(&pattern.gain, 1, 3, 1);
	if (testFlags.trace_test == 0)
	{
		// 選択したゲインを表示
		ssd1306_SetCursor(21, 18);
		if (pattern.gain == 1)
			ssd1306_printfB(Font_7x10, "%3d", yawCtrl.kp);
		else
			ssd1306_printf(Font_7x10, "%3d", yawCtrl.kp);
		ssd1306_SetCursor(21, 32);
		if (pattern.gain == 2)
			ssd1306_printfB(Font_7x10, "%3d", yawCtrl.ki);
		else
			ssd1306_printf(Font_7x10, "%3d", yawCtrl.ki);
		ssd1306_SetCursor(21, 44);
		if (pattern.gain == 3)
			ssd1306_printfB(Font_7x10, "%3d", yawCtrl.kd);
		else
			ssd1306_printf(Font_7x10, "%3d", yawCtrl.kd);

		// 制御量表示
		ssd1306_SetCursor(88, 30);
		ssd1306_printf(Font_7x10, "%4d", yawCtrl.pwm);

		switch (pattern.gain)
		{
		case 1:
			// kpを左右スイッチで調整
			dataTuningLR(&yawCtrl.kp, 1, 0, 255);
			break;
		case 2:
			// kiを左右スイッチで調整
			dataTuningLR(&yawCtrl.ki, 1, 0, 255);
			break;
		case 3:
			// kdを左右スイッチで調整
			dataTuningLR(&yawCtrl.kd, 1, 0, 255);
			break;
		}
    }
}
///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 setup_log
// 処理概要     ログ解析表示と操作を制御
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
static void setup_log(void)
{
	static int16_t y = 0, offset, ret = 0; // y: 選択中のログNo, offset: 表示開始位置, ret: 解析結果
	uint8_t i, j; // i: 表示ループ用, j: スイッチ入力用

	if (pattern.display != pattern.beforeHex)
	{
		// 切替時に実行
		ssd1306_printf(Font_6x8, "microSD  ");
		y = endFileIndex + 1; // 前回解析したログを初期選択
		ssd1306_SetCursor(30, 16);
		ssd1306_printf(Font_6x8, "Dist <");
		ssd1306_SetCursor(80, 16);
		ssd1306_printf(Font_6x8, "> XYcalc");
		ssd1306_SetCursor(46, 25);
		ssd1306_printf(Font_6x8, "indexD:%4d", numPPADarry);
		ssd1306_SetCursor(46, 34);
		ssd1306_printf(Font_6x8, "indexS:%4d", indexSC);
		ssd1306_SetCursor(46, 43);
		ssd1306_printf(Font_6x8, "marker:%4d", numPPAMarry);
	}

	dataTuningUD(&y, 1, 0, endFileIndex + 1); // ログNoを上下スイッチで選択

	j = swValTact; // タクトスイッチの状態を取得
	if (j == SW_LEFT || j == SW_RIGHT) // 左右スイッチで解析を実行
	{
		ssd1306_FillRectangle(30, 25, 127, 63, Black); // メイン表示空白埋め
		ssd1306_SetCursor(46, 38);
		ssd1306_printf(Font_6x8, "Calculating");
		ssd1306_UpdateScreen(); // グラフィック液晶更新

		if (y == endFileIndex + 1)
		{
			y = fileIndexLog; // 前回解析したログを再解析
		}

		if (j == SW_LEFT)
		{
			// 距離基準解析
			ret = readLogDistance(fileNumbers[y]);
		}
		else if (j == SW_RIGHT)
		{
			// ショートカット解析
			ret = calcXYcies(fileNumbers[y]);
		}

		if (ret > 0)
		{
			optimalIndex = 0; // 解析結果インデックスをリセット
			ssd1306_FillRectangle(30, 25, 127, 63, Black); // メイン表示空白埋め
			ssd1306_SetCursor(46, 25);
			ssd1306_printf(Font_6x8, "indexD:%4d", numPPADarry);
			ssd1306_SetCursor(46, 34);
			ssd1306_printf(Font_6x8, "indexS:%4d", indexSC);
			ssd1306_SetCursor(46, 43);
			ssd1306_printf(Font_6x8, "marker:%4d", numPPAMarry);
		}
		else
		{
			ssd1306_FillRectangle(30, 25, 127, 63, Black); // メイン表示空白埋め
			ssd1306_SetCursor(64, 30);
			ssd1306_printf(Font_6x8, "Error");
			ssd1306_SetCursor(61, 38);
			ssd1306_printf(Font_6x8, "code:%d", ret);
		}
	}

	// ログNoの選択処理
	for (i = 0; i < 5; i++)
	{
		// 前回解析ログNoを選択しているとき
		if (y == endFileIndex + 1)
		{
			ssd1306_SetCursor(0, 16);
			ssd1306_printfB(Font_6x8, "%4d", fileNumbers[fileIndexLog]);
		}
		else
		{
			ssd1306_SetCursor(0, 16);
			ssd1306_printf(Font_6x8, "%4d", fileNumbers[fileIndexLog]);
		}

		// ログNoを選択するとき
		offset = endFileIndex - y - 4; // 前回解析Noと一番下のNoを除く表示中の4つ中一番上のインデックスを計算
		ssd1306_SetCursor(0, 24 + (8 * i));

		// 最新4つのデータを表示するとき
		if (offset < 0)
		{
			offset = 0;
		}

		if (endFileIndex - y == i || (i == 4 && offset > 0))
		{
			// 選択したログNoをハイライト表示
			ssd1306_printfB(Font_6x8, "%4d", fileNumbers[endFileIndex - offset - i]);
		}
		else
		{
			ssd1306_printf(Font_6x8, "%4d", fileNumbers[endFileIndex - offset - i]);
		}
    }
}

///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 setup_start
// 処理概要     スタート待ち画面とキャリブレーションを制御
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
static void setup_start(void)
{
	if (pattern.display != pattern.beforeHex)
	{
		// 切替時にスタート画面とブースト設定を表示
		ssd1306_printf(Font_6x8, "Start  ");
		ssd1306_SetCursor(30, 25);
		ssd1306_printf(Font_11x18, "Ready?");
		ssd1306_SetCursor(20, 50);
		switch (optimalTrace)
		{
		case BOOST_NONE:
			ssd1306_printf(Font_6x8, "BOOST NONE");
			break;
		case BOOST_MARKER:
			ssd1306_printf(Font_6x8, "BOOST MARKER");
			break;
		case BOOST_DISTANCE:
			ssd1306_printf(Font_6x8, "BOOST DISTANCE");
			break;
		case BOOST_SHORTCUT:
			ssd1306_printf(Font_6x8, "BOOST SHORTCUT");
			break;
		}
		pattern.calibration = 1;
	}

	// キャリブレーションの進行状況に応じて処理を分岐
	switch (pattern.calibration)
	{
	case 1: // スイッチ入力待ち
	{
		// 停止状態を維持
		setTargetSpeed(0);

                if (swValTact == SW_PUSH)
                {
                        if (lSensorMax[0] > lSensorMin[0])
                        {
                                // キャリブレーション実施済み
                                setupFlags.start = 1;
                        }
                        else
                        {
                                pattern.calibration = 2;
                        }
                }
                else if (swValTact == SW_RIGHT)
                {
                        // オートスタート
                        if (lSensorMax[0] > lSensorMin[0])
                        {
                                // キャリブレーション実施済み
                                autoStart = 1;
                        }
                        else
                        {
                                pattern.calibration = 2;
                        }
                }
		break;
	}
	case 2: // キャリブレーション未実施
	{
		veloCtrl.Int = 0;                                                         // I成分リセット
		ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
		ssd1306_SetCursor(22, 28);
		ssd1306_printf(Font_7x10, "Calibration");
		ssd1306_SetCursor(53, 42);
		ssd1306_printf(Font_7x10, "Now");
		ssd1306_UpdateScreen(); // グラフィック液晶更新

		testFlags.trace_test = true;
		setupTimer.cntSetup1 = 0;
		enc1 = 0;
		powerLineSensors(1); // 先に点灯させて安定させる

		pattern.calibration = 3;
		break;
	}
	case 3: // 開始準備
	{
		if (setupTimer.cntSetup1 > 1000)
		{
			veloCtrl.Int = 0;                // I成分リセット
			BMI088val.angle.z = 0.0; // 角度リセット
			yawRateCtrl.Int = 0.0;   // I成分リセット
			setTargetSpeed(0);               // 目標速度0[m/s]
			enc1 = 0;
			modeCalLinesensors = 1; // キャリブレーション開始
			pattern.calibration = 4;
		}
		break;
	}
	case 4: // 左旋回
	{
		setTargetAngularVelocity(CALIBRATIONSPEED);
		motorPwmOutSynth(0, veloCtrl.pwm, yawRateCtrl.pwm, 0);
		if (BMI088val.angle.z < -320.0)
		{
			pattern.calibration = 5;
		}
		break;
	}
	case 5: // 初期位置に戻る
	{
		setTargetAngularVelocity(-400.0F);
		motorPwmOutSynth(0, veloCtrl.pwm, yawRateCtrl.pwm, 0);
		if (lSensor[5] < 1000)
		{
			modeCalLinesensors = 0;
			countdown = 500;
			pattern.calibration = 6;
		}
		break;
	}
	case 6: // 停止
	{
		motorPwmOutSynth(lineTraceCtrl.pwm, veloCtrl.pwm, 0, 0);
		if (countdown <= 0)
		{
			powerLineSensors(0); // ラインセンサ消灯
			setupFlags.start = 1;
		}
		break;
	}
	default:
		break;
	}
}

///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 setup
// 処理概要     走行前設定
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
void setup(void)
{
	uint8_t cntLed, i, j, k;
	static uint8_t beforeBATLV;
	static int16_t x = 0, y = 0, offset, ret = 0;

	SchmittBatery(); // バッテリレベルを取得
	if (batteryLevel != beforeBATLV)
	{
		// バッテリレベルが変化したときに実行
		showBattery(); // バッテリ残量表示
	}

	// 左ホイールをロータリスイッチ代わりに使用する
	if (!testFlags.trace_test && !testFlags.motor_test)
	{
		if (abs(encClick) > 400)
		{
			if (encClick > 400)
			{
				pattern.display++;
				setupFlags.clickStart = 1;
			}
			else if (encClick < -400)
			{
				pattern.display--;
				setupFlags.clickStart = -1;
			}

			if (pattern.display > 0x9)
				pattern.display = 0;
			else if (pattern.display < 0)
				pattern.display = 0x9;
			encClick = 0;
		}
	}
	else
	{
		encClick = 0;
	}

	// ページ番号表示
	if (pattern.display != pattern.beforeHex)
	{
		if (!modeDSP) // ディスプレイが無いとき番号をLEDで表示
		{
			led_out(pattern.display);
		}

		// ロータリスイッチ切替時に実行
		showBattery(); // バッテリ残量表示

		// ロータリスイッチ値を表示
		ssd1306_SetCursor(0, 3);
		ssd1306_printf(Font_6x8, "No.%x", pattern.display);

		ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
		ssd1306_FillRectangle(24, 0, 94, 13, Black);  // ヘッダ表示空白埋め
		ssd1306_SetCursor(28, 3);					  // ヘッダタイトル位置
	}

	// ディップスイッチで項目選択
	switch (pattern.display)
	{
	//------------------------------------------------------------------
	// スタート待ち
	//------------------------------------------------------------------
	case HEX_START:
	{
		setup_start(); // スタート待ち画面とキャリブレーションを制御
		break;
	}
	//------------------------------------------------------------------
	// パラメータ調整(通常トレース)
	//------------------------------------------------------------------
	case HEX_SPEED_PARAM:
	{
		setup_speed_param(); // 速度パラメータ調整
		break;
	}
	//------------------------------------------------------------------
	// センサテスト
	//------------------------------------------------------------------
	case HEX_SENSORS:
	{
		setup_sensors(); // センサ表示とテストメニューを制御
		break;
	}
	//------------------------------------------------------------------
	// Log analysis
	//------------------------------------------------------------------
	case HEX_LOG:
	{
		setup_log(); // ログ解析表示と操作を制御
		break;
	}
	//------------------------------------------------------------------
	// キャリブレーション(ラインセンサ)
	//------------------------------------------------------------------
	case HEX_CALIBRATION:
	{
		setup_calibration(); // キャリブレーション(ラインセンサ)
		break;
	}
	//------------------------------------------------------------------
	// ゲイン調整(直線トレース)
	//------------------------------------------------------------------
	case HEX_PID_TRACE:
	{
		setup_pid_trace(); // ゲイン調整(直線トレース)
		break;
	}
	//------------------------------------------------------------------
	// ゲイン調整(速度)
	//------------------------------------------------------------------
	case HEX_PID_SPEED:
	{
		setup_pid_speed(); // 速度PIDゲイン調整
		break;
	}
	//------------------------------------------------------------------
	// ゲイン調整(角速度)
	//------------------------------------------------------------------
	case HEX_PID_ANGULAR:
	{
		setup_pid_angular(); // 角速度PIDの設定処理を呼び出し
		break;
	}
	//------------------------------------------------------------------
	// ゲイン調整(角度)
	//------------------------------------------------------------------
	case HEX_PID_ANGLE:
	{
		setup_pid_angle(); // ゲイン調整(角度)
		break;
	}
	//------------------------------------------------------------------
	// ゲイン調整(距離)
	//------------------------------------------------------------------
	case HEX_PID_DIST:
	{
		setup_pid_dist(); // 距離PID調整処理を実行
		break;
	}

	default:
	{
		ssd1306_SetCursor(30, 5);
		ssd1306_printf(Font_6x8, "None      ");
		ssd1306_FillRectangle(0, 16, 127, 63, Black);

		break;
	}
	} // switch

	// 前回値更新
	pattern.beforeHex = pattern.display;
	beforeBATLV = batteryLevel;

	if (!modeDSP)
	{
		sendLED();
	}

	if (!testFlags.trace_test && !calibratIMU)
	{
		ssd1306_UpdateScreen(); // グラフィック液晶更新
	}
}
///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 data_select
// 処理概要     タクトスイッチで0,1に変化させる
// 引数         data: 変化させる変数 button: どのスイッチで変化させるか
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
void data_select(uint8_t *data, uint8_t button)
{
	static uint8_t push = 0;

	if (swValTact == button)
	{
		if (*data == 1 && push == 0)
		{
			push = 1;
			*data = 0;
		}
		else if (*data == 0 && push == 0)
		{
			push = 1;
			*data = 1;
		}
	}
	else
	{
		push = 0;
	}
}
///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 dataTuning
// 処理概要     タクトスイッチの方向と数値型を指定して値を加減する
// 引数         data: 調整対象の変数 add: 変化量 min: 最小値 max: 最大値 dir: 方向 UD/LR type: 型 int16_t/float
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
void dataTuning(void *data, double add, double min, double max, uint8_t dir, uint8_t type)
{
	int16_t sign;
	uint16_t *cntSwitch;
	uint16_t *cntSwitchLong;
	int8_t *push;
	uint8_t swPlus;
	uint8_t swMinus;

	// max と min の差から加算方向を決定
	if (max - min > 0)
	{
		sign = 1;
	}
	else
	{
		sign = -1;
	}

	// 方向に応じて使用するスイッチとカウンタを設定
	if (dir == UD)
	{
		cntSwitch = &setupTimer.cntSwitchUD;
		cntSwitchLong = &setupTimer.cntSwitchUDLong;
		push = &pushUD;
		swPlus = SW_UP;     // 増加方向のスイッチ
		swMinus = SW_DOWN;  // 減少方向のスイッチ
	}
	else
	{
		cntSwitch = &setupTimer.cntSwitchLR;
		cntSwitchLong = &setupTimer.cntSwitchLRLong;
		push = &pushLR;
		swPlus = SW_RIGHT;  // 増加方向のスイッチ
		swMinus = SW_LEFT;  // 減少方向のスイッチ
	}

	// スイッチ入力に応じて値を更新
	if (*cntSwitch >= 50)        // 一定周期で判定
	{
		if (swValTact == swPlus || swValTact == swMinus)
		{
			(*cntSwitchLong)++; // 長押し時間を加算
			if (swValTact == swPlus)
			{
				// インクリメント処理
				if (*cntSwitchLong >= PUSHTIME)
				{ // 長押し時
					if (type == TYPE_FLOAT)
					{
						*(float *)data += sign * (float)add;
					}
					else
					{
						*(int16_t *)data += sign * (int16_t)add;
					}
				}
				else if (*push == 0)
				{ // 1回押し
					*push = 1;
					if (type == TYPE_FLOAT)
					{
						*(float *)data += sign * (float)add;
					}
					else
					{
						*(int16_t *)data += sign * (int16_t)add;
					}
				}
			}
			else if (swValTact == swMinus)
			{
				// デクリメント処理
				if (*cntSwitchLong >= PUSHTIME)
				{ // 長押し時
					if (type == TYPE_FLOAT)
					{
						*(float *)data -= sign * (float)add;
					}
					else
					{
						*(int16_t *)data -= sign * (int16_t)add;
					}
				}
				else if (*push == 0)
				{ // 1回押し
					*push = 1;
					if (type == TYPE_FLOAT)
					{
						*(float *)data -= sign * (float)add;
					}
					else
					{
						*(int16_t *)data -= sign * (int16_t)add;
					}
				}
			}
		}
		else
		{
			*push = 0;          // 押下状態をリセット
			*cntSwitchLong = 0; // 長押しカウンタをリセット
		}
		*cntSwitch = 0;             // 判定後にカウンタをリセット

		// 上限・下限を超えた場合の折り返し処理
		if (type == TYPE_FLOAT)
		{
			float *d = (float *)data;
			if (sign > 0)
			{
				if (*d > (float)max)
				{
					*d = (float)min;
				}
				else if (*d < (float)min)
				{
					*d = (float)max;
				}
			}
			else
			{
				if (*d > (float)min)
				{
					*d = (float)max;
				}
				else if (*d < (float)max)
				{
					*d = (float)min;
				}
			}
		}
		else
		{
			int16_t *d = (int16_t *)data;
			if (sign > 0)
			{
				if (*d > (int16_t)max)
				{
					*d = (int16_t)min;
				}
				else if (*d < (int16_t)min)
				{
					*d = (int16_t)max;
				}
			}
			else
			{
				if (*d > (int16_t)min)
				{
					*d = (int16_t)max;
				}
				else if (*d < (int16_t)max)
				{
					*d = (int16_t)min;
				}
			}
		}
	}
}
///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 dataTuningUD
// 処理概要     上下方向でint16_t値を調整するラッパー
// 引数         data: 調整対象の変数 add: 変化量 min: 最小値 max: 最大値
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
void dataTuningUD(int16_t *data, int16_t add, int16_t min, int16_t max)
{
	// 汎用関数へ上下方向とint16_t型を指定して委譲
	dataTuning(data, add, min, max, UD, TYPE_INT16);
}
///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 dataTuningLR
// 処理概要     左右方向でint16_t値を調整するラッパー
// 引数         data: 調整対象の変数 add: 変化量 min: 最小値 max: 最大値
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
void dataTuningLR(int16_t *data, int16_t add, int16_t min, int16_t max)
{
	// 汎用関数へ左右方向とint16_t型を指定して委譲
	dataTuning(data, add, min, max, LR, TYPE_INT16);
}
///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 dataTuningUDF
// 処理概要     上下方向でfloat値を調整するラッパー
// 引数         data: 調整対象の変数 add: 変化量 min: 最小値 max: 最大値
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
void dataTuningUDF(float *data, float add, float min, float max)
{
	// 汎用関数へ上下方向とfloat型を指定して委譲
	dataTuning(data, add, min, max, UD, TYPE_FLOAT);
}
///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 setupNonDisp
// 処理概要     拡張ボードを接続していないときのセットアップ
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
void setupNonDisp(void)
{
	static uint8_t mode = 0;

	switch (pattern.calibration)
	{
	case 1:
		setTargetSpeed(0);

		led_out(0x9);
		// スイッチ入力待ち
		if (swValMainTact == SW_TACT_L || swValMainTact == SW_TACT_R)
		{
			if (swValMainTact == SW_TACT_L)
			{
				mode = START_SERACH;
			}
			else if (swValMainTact == SW_TACT_R)
			{
				mode = START_OPTIMAL;
			}
			veloCtrl.Int = 0; // I成分リセット
                  if (lSensorMax[0] > lSensorMin[0])
			{
				// キャリブレーション実施済み
				setupFlags.start = 1;
			}
			else
			{
				// キャリブレーション未実施
				setupTimer.cntSetup1 = 0;
				enc1 = 0;
				powerLineSensors(1); // 先に点灯させて安定させる
				pattern.calibration = 2;
			}
		}
		break;

	case 2:
		// 開始準備
		if (setupTimer.cntSetup1 > 1000)
		{
			veloCtrl.Int = 0;		 // I成分リセット
			BMI088val.angle.z = 0.0; // 角度リセット
			yawRateCtrl.Int = 0.0;	 // I成分リセット
			setTargetSpeed(0);		 // 目標速度0[m/s]
			enc1 = 0;
			modeCalLinesensors = 1; // キャリブレーション開始
			pattern.calibration = 3;
		}
		break;

	case 3:
		// 左旋回
		setTargetAngularVelocity(CALIBRATIONSPEED);
		motorPwmOutSynth(0, veloCtrl.pwm, yawRateCtrl.pwm, 0);
		if (BMI088val.angle.z < -35.0)
		{
			pattern.calibration = 4;
		}
		break;

	case 4:
		// 停止
		setTargetSpeed(0);
		motorPwmOutSynth(0, veloCtrl.pwm, 0, 0);
		if (abs(encCurrentN) == 0)
		{
			pattern.calibration = 5;
		}
		break;

	case 5:
		// 右旋回
		setTargetAngularVelocity(-CALIBRATIONSPEED);
		motorPwmOutSynth(0, veloCtrl.pwm, yawRateCtrl.pwm, 0);
		if (BMI088val.angle.z > 35.0)
		{
			pattern.calibration = 6;
		}
		break;

	case 6:
		// 停止
		setTargetSpeed(0);
		motorPwmOutSynth(0, veloCtrl.pwm, 0, 0);
		if (abs(encCurrentN) == 0)
		{
			pattern.calibration = 7;
		}
		break;

	case 7:
		// 初期位置に戻る
		setTargetAngularVelocity(CALIBRATIONSPEED);
		motorPwmOutSynth(0, veloCtrl.pwm, yawRateCtrl.pwm, 0);
		if (lSensor[5] < 1000)
		{
			modeCalLinesensors = 0;
			pattern.calibration = 8;
		}
		break;

	case 8:
		// 停止
		motorPwmOutSynth(0, veloCtrl.pwm, 0, 0);
		if (abs(encCurrentN) == 0)
		{
			powerLineSensors(0); // ラインセンサ消灯
			if (mode == START_OPTIMAL)
			{
				// 距離基準解析
				numPPADarry = readLogDistance(analizedNumber);
				if (numPPADarry > 0)
				{
					optimalTrace = BOOST_DISTANCE;
					optimalIndex = 0;
				}
			}
			setupFlags.start = mode;
		}
		break;

	default:
		break;
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 setupCount
// 処理概要     セットアップ用タイマを加算
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void setupCount(void)
{
	setupTimer.cntSetup1++;
	setupTimer.cntSetup2++;
	setupTimer.cntSwitchUD++;
	setupTimer.cntSwitchLR++;
}
///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 wheelClick
// 処理概要     ホイールを短時間回転させクリック感を出す
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
void wheelClick(void)
{
	static uint8_t cnt = 0;
	uint16_t pwm = 200;

	switch (pattern.click)
	{
	case 1:
		if (setupFlags.clickStart != 0)
		{
			pattern.click = 2;
		}
		break;

	case 2:
		motorPwmOut(-pwm * setupFlags.clickStart, 0);
		cnt++;
		if (cnt >= 3)
		{
			cnt = 0;
			pattern.click = 3;
		}
		break;

	case 3:
		motorPwmOut(pwm * setupFlags.clickStart, 0);
		cnt++;
		if (cnt >= 3)
		{
			cnt = 0;
			pattern.click = 4;
		}
		break;

	case 4:
		motorPwmOut(0, 0);
		setupFlags.clickStart = 0;
		pattern.click = 1;
		break;
	}
}