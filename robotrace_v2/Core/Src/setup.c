//====================================//
// インクルード
//====================================//
#include "setup.h"
//====================================//
// グローバル変数の宣言
//====================================//
uint8_t start = 0; // 0:セットアップ中	1:セットアップ完了

// タイマ関連
uint16_t cntSetup1 = 0;		  // セットアップで使用
uint16_t cntSetup2 = 0;		  // セットアップで使用
uint16_t cntSwitchUD = 0;	  // スイッチ判定用右
uint16_t cntSwitchLR = 0;	  // スイッチ判定用左
uint16_t cntSwitchUDLong = 0; // スイッチ長押し判定用右
uint16_t cntSwitchLRLong = 0; // スイッチ長押し判定用左

// スイッチ関連
int8_t pushLR = 0;
int8_t pushUD = 0;

// パターン関連
uint8_t push1 = 0;
int16_t patternDisplay = 0;
int16_t patternSensors = 1;
int16_t beforeSensors = 0;
static uint8_t beforeHEX = 255;      // 前回の表示HEXを保持
int16_t patternSensorLine = 1;
int16_t patternSensorAccele = 1;
int16_t patternSensorGyro = 1;
int16_t patternParameter1 = 1;
int16_t patternParameter2 = 1;
int16_t patternParameter3 = 1;
int16_t patternParameter4 = 1;
int16_t patternGain = 3;
int16_t patternSpeedseting = 1;
int16_t patternLog = 1;
int16_t patternCalibration = 1;
int16_t patternClick = 1;

// フラグ関連
uint8_t motor_test = 0;
uint8_t trace_test = 0;
int8_t clickStart = 0;
static uint8_t beforeMotorTest = 0;  // モータテストの状態を保存

// パラメータ関連
int16_t motorTestPwm = 200;
int32_t encClick = 0;
//======================================//
// プロトタイプ宣言
//======================================//
static void setup_sensors(void); // センサ表示とテストメニューを制御する処理
static void setup_pid_trace(void); // ゲイン調整(直線トレース)
static void setup_pid_angle(void); // ゲイン調整(角度)
/////////////////////////////////////////////////////////////////////////////////////
// モジュール名 setup_sensors
// 処理概要     センサ表示とテストメニューを制御
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////////////////////
static void setup_sensors(void)
{
	if (patternDisplay != beforeHEX)
		{
			// ページ切替時の初期処理
			ssd1306_printf(Font_6x8, "SENSORS  ");	// センサ画面表示
			beforeSensors = 100;	// 初期値
		}

		// センサメニューの項目切替
		dataTuningLR(&patternSensors, 1, 1, 8);
		// 各種センサテストを実行
		switch (patternSensors)
		{
		case 1: // モータテスト
		{
			if (patternSensors != beforeSensors)
			{
				// 切替時に実行
				ssd1306_FillRectangle(0, 16, 127, 63, Black); // 黒塗り
				ssd1306_SetCursor(47, 16);
				ssd1306_printf(Font_6x8, "Motor");
				motor_test = 0;
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
			data_select(&motor_test, SW_PUSH); // モータテストの開始/停止
			if (motor_test == 1)
			{
				motorPwmOut(motorTestPwm, motorTestPwm);
			}
			else
			{
				motorPwmOut(0, 0);
			}

			// motor_test 1→0のとき 2にする
			if (motor_test != beforeMotorTest && motor_test == 0)
			{
				motor_test = 2;
			}
			// 2のときホイールの回転が止まったらmotor_test=0にする
			if (motor_test == 2 && encCurrentL == 0)
			{
				motor_test = 0;
			}
			beforeMotorTest = motor_test; // 次回比較用に状態を保存
			break;
		}
		case 2: // IMU角度表示
		{
			if (patternSensors != beforeSensors)
			{
				// 切替時に実行
				ssd1306_FillRectangle(0, 16, 127, 63, Black); // 黒塗り
				ssd1306_SetCursor(36, 16);
				ssd1306_printf(Font_7x10, "IMU[deg]");
				motor_test = 1;
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
			break;
		}
		case 3: // IMU加速度表示
		{
			if (patternSensors != beforeSensors)
			{
				// 切替時に実行
				ssd1306_FillRectangle(0, 16, 127, 63, Black); // 黒塗り
				ssd1306_SetCursor(36, 16);
				ssd1306_printf(Font_7x10, "IMU[g]");
			}

			ssd1306_SetCursor(0, 30);
			ssd1306_printf(Font_7x10, "xa:%6.1f", BMI088val.accele.x);
			ssd1306_SetCursor(0, 42);
			ssd1306_printf(Font_7x10, "ya:%6.1f", BMI088val.accele.y);
			ssd1306_SetCursor(0, 54);
			ssd1306_printf(Font_7x10, "za:%6.1f", BMI088val.accele.z);

			ssd1306_SetCursor(64, 30);
			ssd1306_printf(Font_7x10, "T:%4.1f", BMI088val.temp);
			break;
		}
		case 4: // マーカーセンサ
		{
			if (patternSensors != beforeSensors)
			{
				// 切替時に実行
				ssd1306_FillRectangle(0, 16, 127, 63, Black); // 黒塗り
				ssd1306_SetCursor(15, 16);
				ssd1306_printf(Font_7x10, "Marker sensors");
			}
			ssd1306_SetCursor(0, 30);
			ssd1306_printf(Font_7x10, "sensors:%d", getMarkerSensor());
			ssd1306_SetCursor(0, 45);
			ssd1306_printf(Font_7x10, "britght:%d", motor_test);

			data_select(&motor_test, SW_PUSH);
			if (motor_test == 1)
			{
				powerMarkerSensors(1);
			}
			else
			{
				powerMarkerSensors(0);
			}

			break;
		}
		case 5: // タクトスイッチ
		{
			if (patternSensors != beforeSensors)
			{
				// 切替時に実行
				ssd1306_FillRectangle(0, 16, 127, 63, Black); // 黒塗り
				ssd1306_SetCursor(32, 16);
				ssd1306_printf(Font_7x10, "Switches");
			}
			ssd1306_SetCursor(0, 30);
			ssd1306_printf(Font_7x10, "Board SW:%d", swValMainTact);

			ssd1306_SetCursor(0, 42);
			ssd1306_printf(Font_7x10, "5axis SW:%d", swValTact);

			break;
		}
		case 6: // バッテリ電圧
		{
			if (patternSensors != beforeSensors)
			{
				// 切替時に実行
				ssd1306_FillRectangle(0, 16, 127, 63, Black); // 黒塗り
				ssd1306_SetCursor(32, 16);
				ssd1306_printf(Font_7x10, "Battery");
			}
			ssd1306_SetCursor(0, 30);
			ssd1306_printf(Font_7x10, "batteryADAD:%d", batteryAD);

			ssd1306_SetCursor(0, 42);
			ssd1306_printf(Font_7x10, "BatteryLv:%d", batteryLevel);

			break;
		}
		case 7: // ラインセンサ
		{
			if (patternSensors != beforeSensors)
			{
				// 切替時に実行
				ssd1306_FillRectangle(0, 16, 127, 63, Black); // 黒塗り
				// センサ基板形状
				ssd1306_DrawArc(64, 81, 66, 90, 270, White);
				ssd1306_DrawArc(64, 81, 35, 90, 270, White);
				ssd1306_Line(2, 63, 34, 63, White);
				ssd1306_Line(93, 63, 126, 63, White);
				motor_test = 0;
			}

			if (lSensorOffset[0] > 0 && modeCalLinesensors == 0)
			{
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
			}
			else
			{
				ssd1306_SetCursor(37, 22);
				ssd1306_printf(Font_6x8, "%4d", lSensor[4]);
				ssd1306_SetCursor(31, 30);
				ssd1306_printf(Font_6x8, "%4d", lSensor[3]);
				ssd1306_SetCursor(22, 38);
				ssd1306_printf(Font_6x8, "%4d", lSensor[2]);
				ssd1306_SetCursor(13, 46);
				ssd1306_printf(Font_6x8, "%4d", lSensor[1]);
				ssd1306_SetCursor(6, 54);
				ssd1306_printf(Font_6x8, "%4d", lSensor[0]);

				ssd1306_SetCursor(65, 22);
				ssd1306_printf(Font_6x8, "%4d", lSensor[5]);
				ssd1306_SetCursor(71, 30);
				ssd1306_printf(Font_6x8, "%4d", lSensor[6]);
				ssd1306_SetCursor(80, 38);
				ssd1306_printf(Font_6x8, "%4d", lSensor[7]);
				ssd1306_SetCursor(89, 46);
				ssd1306_printf(Font_6x8, "%4d", lSensor[8]);
				ssd1306_SetCursor(95, 54);
				ssd1306_printf(Font_6x8, "%4d", lSensor[9]);
			}

			data_select(&motor_test, SW_PUSH);
			if (motor_test == 1)
			{
				powerLineSensors(1);
			}
			else
			{
				powerLineSensors(0);
			}

			break;
		}
		case 8: // RGBLED
		{
			if (patternSensors != beforeSensors)
			{
				// 切替時に実行
				ssd1306_FillRectangle(0, 16, 127, 63, Black); // 黒塗り
				ssd1306_SetCursor(43, 16);
				ssd1306_printf(Font_7x10, "RGBLED");
			}

			data_select(&motor_test, SW_PUSH);
			if (motor_test == 1)
			{
				if (cntSetup2 > 50)
				{
					fullColorLED(10, 4);
					cntSetup2 = 0;
				}
			}

			if (motor_test != beforeMotorTest)
			{
				clearLED();
			}

			beforeMotorTest = motor_test;
			break;
		}
	}
	beforeSensors = patternSensors;	// 選択状態の更新
}
///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 setup_pid_trace
// 処理概要     ゲイン調整(直線トレース)
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
static void setup_pid_trace(void)
{
	if (patternDisplay != beforeHEX)
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

	data_select(&trace_test, SW_PUSH); // PUSHでトレースON/OFFの選択
	// PUSHでトレースON/OFF
	if (trace_test == 1)
	{
		motorPwmOutSynth(lineTraceCtrl.pwm, 0, 0, 0); // モータを指定PWMで駆動
		powerLineSensors(1);                          // ラインセンサを有効化
	}
	else
	{
		motorPwmOutSynth(0, 0, 0, 0);                // モータ停止
		powerLineSensors(0);                         // ラインセンサ停止
	}
	if (trace_test != beforeMotorTest && trace_test == 0)
	{
		trace_test = 2;                              // 停止待機状態へ遷移
	}
	if (trace_test == 2 && encCurrentL == 0) // ホイールの回転が停止したら0
	{
		trace_test = 0;                              // 完全停止後に終了
	}
	beforeMotorTest = trace_test;                        // 状態を保存

	// ゲイン表示
	dataTuningUD(&patternGain, 1, 3, 1);
	if (trace_test == 0)
	{
		ssd1306_SetCursor(21, 18);
		if (patternGain == 1)
			ssd1306_printfB(Font_7x10, "%3d", lineTraceCtrl.kp);
		else
			ssd1306_printf(Font_7x10, "%3d", lineTraceCtrl.kp);
		ssd1306_SetCursor(21, 32);
		if (patternGain == 2)
			ssd1306_printfB(Font_7x10, "%3d", lineTraceCtrl.ki);
		else
			ssd1306_printf(Font_7x10, "%3d", lineTraceCtrl.ki);
		ssd1306_SetCursor(21, 44);
		if (patternGain == 3)
			ssd1306_printfB(Font_7x10, "%3d", lineTraceCtrl.kd);
		else
			ssd1306_printf(Font_7x10, "%3d", lineTraceCtrl.kd);

		// 制御量表示
		ssd1306_SetCursor(88, 30);
		ssd1306_printf(Font_7x10, "%4d", lineTraceCtrl.pwm);

		switch (patternGain)
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
///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 setup_pid_angle
// 処理概要     ゲイン調整(角度)
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
static void setup_pid_angle(void)
{
	if (patternDisplay != beforeHEX)
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

	data_select(&trace_test, SW_PUSH);       // PUSHでトレースON/OFF
	// if ( trace_test == 1 ) {
	//      motorPwmOutSynth( 0, veloCtrl.pwm, distCtrl.pwm, 0 );
	// } else {
	//      motorPwmOutSynth( 0, 0, 0, 0 );
	// }

	// 上下スイッチで調整対象のゲインを選択
	dataTuningUD(&patternGain, 1, 3, 1);
	if (trace_test == 0)
	{
		// 選択したゲインを表示
		ssd1306_SetCursor(21, 18);
		if (patternGain == 1)
			ssd1306_printfB(Font_7x10, "%3d", yawCtrl.kp);
		else
			ssd1306_printf(Font_7x10, "%3d", yawCtrl.kp);
		ssd1306_SetCursor(21, 32);
		if (patternGain == 2)
			ssd1306_printfB(Font_7x10, "%3d", yawCtrl.ki);
		else
			ssd1306_printf(Font_7x10, "%3d", yawCtrl.ki);
		ssd1306_SetCursor(21, 44);
		if (patternGain == 3)
			ssd1306_printfB(Font_7x10, "%3d", yawCtrl.kd);
		else
			ssd1306_printf(Font_7x10, "%3d", yawCtrl.kd);

		// 制御量表示
		ssd1306_SetCursor(88, 30);
		ssd1306_printf(Font_7x10, "%4d", yawCtrl.pwm);

		switch (patternGain)
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
// モジュール名 setup
// 処理概要     走行前設定
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
void setup(void)
{
	uint8_t cntLed, i, j, k;
	static uint8_t beforePparam, beforeBATLV;
	static int16_t x = 0, y = 0, offset, ret = 0;

	SchmittBatery(); // バッテリレベルを取得
	if (batteryLevel != beforeBATLV)
	{
		// バッテリレベルが変化したときに実行
		showBattery(); // バッテリ残量表示
	}

	// 左ホイールをロータリスイッチ代わりに使用する
	if (!trace_test && !motor_test)
	{
		if (abs(encClick) > 400)
		{
			if (encClick > 400)
			{
				patternDisplay++;
				clickStart = 1;
			}
			else if (encClick < -400)
			{
				patternDisplay--;
				clickStart = -1;
			}

			if (patternDisplay > 0x9)
				patternDisplay = 0;
			else if (patternDisplay < 0)
				patternDisplay = 0x9;
			encClick = 0;
		}
	}
	else
	{
		encClick = 0;
	}

	// ページ番号表示
	if (patternDisplay != beforeHEX)
	{
		if (!modeDSP) // ディスプレイが無いとき番号をLEDで表示
		{
			led_out(patternDisplay);
		}

		// ロータリスイッチ切替時に実行
		showBattery(); // バッテリ残量表示

		// ロータリスイッチ値を表示
		ssd1306_SetCursor(0, 3);
		ssd1306_printf(Font_6x8, "No.%x", patternDisplay);

		ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
		ssd1306_FillRectangle(24, 0, 94, 13, Black);  // ヘッダ表示空白埋め
		ssd1306_SetCursor(28, 3);					  // ヘッダタイトル位置
	}

	// ディップスイッチで項目選択
	switch (patternDisplay)
	{
	//------------------------------------------------------------------
	// スタート待ち
	//------------------------------------------------------------------
	case HEX_START:
	{
		if (patternDisplay != beforeHEX)
		{
			// 切替時に実行
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
			patternCalibration = 1;
		}

		switch (patternCalibration)
		{
		case 1: // スイッチ入力待ち
		{
			setTargetSpeed(0);

			if (swValTact == SW_PUSH)
			{
				if (lSensorOffset[0] > 0)
				{
					// キャリブレーション実施済み
					start = 1;
				}
				else
				{
					patternCalibration = 2;
				}
			}
			else if (swValTact == SW_RIGHT)
			{
				// オートスタート
				if (lSensorOffset[0] > 0)
				{
					// キャリブレーション実施済み
					autoStart = 1;
				}
				else
				{
					patternCalibration = 2;
				}
			}
			break;
		}
		case 2: // キャリブレーション未実施
		{
			veloCtrl.Int = 0;							  // I成分リセット
			ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
			ssd1306_SetCursor(22, 28);
			ssd1306_printf(Font_7x10, "Calibration");
			ssd1306_SetCursor(53, 42);
			ssd1306_printf(Font_7x10, "Now");
			ssd1306_UpdateScreen(); // グラフィック液晶更新

			trace_test = true;
			cntSetup1 = 0;
			enc1 = 0;
			powerLineSensors(1); // 先に点灯させて安定させる

			patternCalibration = 3;
			break;
		}
		case 3: // 開始準備
		{
			if (cntSetup1 > 1000)
			{
				veloCtrl.Int = 0;		 // I成分リセット
				BMI088val.angle.z = 0.0; // 角度リセット
				yawRateCtrl.Int = 0.0;	 // I成分リセット
				setTargetSpeed(0);		 // 目標速度0[m/s]
				enc1 = 0;
				modeCalLinesensors = 1; // キャリブレーション開始
				patternCalibration = 4;
			}
			break;
		}
		case 4: // 左旋回
		{
			setTargetAngularVelocity(CALIBRATIONSPEED);
			motorPwmOutSynth(0, veloCtrl.pwm, yawRateCtrl.pwm, 0);
			if (BMI088val.angle.z < -320.0)
			{
				patternCalibration = 5;
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
				patternCalibration = 6;
			}
			break;
		}
		case 6: // 停止
		{
			motorPwmOutSynth(lineTraceCtrl.pwm, veloCtrl.pwm, 0, 0);
			if (countdown <= 0)
			{
				powerLineSensors(0); // ラインセンサ消灯
				start = 1;
			}
			break;
		}
		default:
			break;
		}
		break;
	}
	//------------------------------------------------------------------
	// パラメータ調整(通常トレース)
	//------------------------------------------------------------------
	case HEX_SPEED_PARAM:
	{
		if (patternDisplay != beforeHEX)
		{
			// 切替時に実行
			ssd1306_printf(Font_6x8, "Parameter");
		}

		dataTuningLR(&patternParameter1, 1, 1, 18);

		if (beforePparam != patternParameter1)
		{
			ssd1306_FillRectangle(0, 16, 127, 63, Black);
		}

		switch (patternParameter1)
		{
		case 1: // 通常走行速度
		{
			dataTuningUDF(&tgtParam.straight, 0.1, 0.0, 10.0);
			ssd1306_SetCursor(0, 24);
			ssd1306_printf(Font_6x8, "STRAIGHT:%3gm/s", tgtParam.straight);
			break;
		}
		case 2: // 停止速度
		{
			dataTuningUDF(&tgtParam.curve, 0.1, 0.0, 10.0);
			ssd1306_SetCursor(0, 24);
			ssd1306_printf(Font_6x8, "CURVE:%3gm/s", tgtParam.curve);
			break;
		}
		case 3: // 停止速度
		{
			dataTuningUDF(&tgtParam.stop, 0.1, 0.0, 10.0);
			ssd1306_SetCursor(0, 24);
			ssd1306_printf(Font_6x8, "STOP:%3gm/s", tgtParam.stop);
			break;
		}
		case 4: // 2次走行_直線
		{
			dataTuningUDF(&tgtParam.bstStraight, 0.1, 0.0, 10.0);
			ssd1306_SetCursor(0, 24);
			ssd1306_printf(Font_6x8, "BST STRT:%3gm/s", tgtParam.bstStraight);
			break;
		}
		case 5: // 2次走行_R1500
		{
			dataTuningUDF(&tgtParam.bst1500, 0.1, 0.0, 10.0);
			ssd1306_SetCursor(0, 24);
			ssd1306_printf(Font_6x8, "BST 1500:%3gm/s", tgtParam.bst1500);
			break;
		}
		case 6: // 2次走行_R1300
		{
			dataTuningUDF(&tgtParam.bst1300, 0.1, 0.0, 10.0);
			ssd1306_SetCursor(0, 24);
			ssd1306_printf(Font_6x8, "BST 1300:%3gm/s", tgtParam.bst1300);
			break;
		}
		case 7: // 2次走行_R1000
		{
			dataTuningUDF(&tgtParam.bst1000, 0.1, 0.0, 10.0);
			ssd1306_SetCursor(0, 24);
			ssd1306_printf(Font_6x8, "BST 1000:%3gm/s", tgtParam.bst1000);
			break;
		}
		case 8: // 2次走行_R800
		{
			dataTuningUDF(&tgtParam.bst800, 0.1, 0.0, 10.0);
			ssd1306_SetCursor(0, 24);
			ssd1306_printf(Font_6x8, "BST 800:%3gm/s", tgtParam.bst800);
			break;
		}
		case 9: // 2次走行_R700
		{
			dataTuningUDF(&tgtParam.bst700, 0.1, 0.0, 10.0);
			ssd1306_SetCursor(0, 24);
			ssd1306_printf(Font_6x8, "BST 700:%3gm/s", tgtParam.bst700);
			break;
		}
		case 10: // 2次走行_R600
		{
			dataTuningUDF(&tgtParam.bst600, 0.1, 0.0, 10.0);
			ssd1306_SetCursor(0, 24);
			ssd1306_printf(Font_6x8, "BST 600:%3gm/s", tgtParam.bst600);
			break;
		}
		case 11: // 2次走行_R500
		{
			dataTuningUDF(&tgtParam.bst500, 0.1, 0.0, 10.0);
			ssd1306_SetCursor(0, 24);
			ssd1306_printf(Font_6x8, "BST 500:%3gm/s", tgtParam.bst500);
			break;
		}
		case 12: // 2次走行_R400
		{
			dataTuningUDF(&tgtParam.bst400, 0.1, 0.0, 10.0);
			ssd1306_SetCursor(0, 24);
			ssd1306_printf(Font_6x8, "BST 400:%3gm/s", tgtParam.bst400);
			break;
		}
		case 13: // 2次走行_R300
		{
			dataTuningUDF(&tgtParam.bst300, 0.1, 0.0, 10.0);
			ssd1306_SetCursor(0, 24);
			ssd1306_printf(Font_6x8, "BST 300:%3gm/s", tgtParam.bst300);
			break;
		}
		case 14: // 2次走行_R200
		{
			dataTuningUDF(&tgtParam.bst200, 0.1, 0.0, 10.0);
			ssd1306_SetCursor(0, 24);
			ssd1306_printf(Font_6x8, "BST 200:%3gm/s", tgtParam.bst200);
			break;
		}
		case 15: // 2次走行_R100
		{
			dataTuningUDF(&tgtParam.bst100, 0.1, 0.0, 10.0);
			ssd1306_SetCursor(0, 24);
			ssd1306_printf(Font_6x8, "BST 100:%3gm/s", tgtParam.bst100);
			break;
		}
		case 16: // 2次走行_加速度
		{
			dataTuningUDF(&tgtParam.acceleF, 0.1, 0.0, 10.0);
			ssd1306_SetCursor(0, 24);
			ssd1306_printf(Font_6x8, "BST acceleF:%3gm/ss", tgtParam.acceleF);
			break;
		}
		case 17: // 2次走行_減速度
		{
			dataTuningUDF(&tgtParam.acceleD, 0.1, 0.0, 10.0);
			ssd1306_SetCursor(0, 24);
			ssd1306_printf(Font_6x8, "BST acceleD:%3gm/ss", tgtParam.acceleD);
			break;
		}
		case 18: // 2次走行_減速度
		{
			dataTuningUDF(&tgtParam.shortCut, 0.1, 0.0, 10.0);
			ssd1306_SetCursor(0, 24);
			ssd1306_printf(Font_6x8, "BST shortCut:%3gm/s", tgtParam.shortCut);
			break;
		}
		}
		beforePparam = patternParameter1;
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
		if (patternDisplay != beforeHEX)
		{
			// 切替時に実行
			ssd1306_printf(Font_6x8, "microSD  ");
			y = endFileIndex + 1;
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

		ssd1306_SetCursor(0, 16);
		ssd1306_printf(Font_6x8, "%4d", fileNumbers[fileIndexLog]);

		dataTuningUD(&y, 1, 0, endFileIndex + 1);

		j = swValTact;
		if (j == SW_LEFT || j == SW_RIGHT)
		{
			ssd1306_FillRectangle(30, 25, 127, 63, Black); // メイン表示空白埋め
			ssd1306_SetCursor(46, 38);
			ssd1306_printf(Font_6x8, "Calculating");
			ssd1306_UpdateScreen(); // グラフィック液晶更新

			if (y == endFileIndex + 1)
			{
				y = fileIndexLog;
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
				optimalIndex = 0;
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

		break;
	}
	//------------------------------------------------------------------
	// キャリブレーション(ラインセンサ)
	//------------------------------------------------------------------
	case HEX_CALIBRATION:
	{
		if (patternDisplay != beforeHEX)
		{
			// 切替時に実行
			ssd1306_printf(Font_6x8, "Calibrate");
			patternCalibration = 1;
		}

		switch (patternCalibration)
		{
		case 1: // スイッチ入力待ち
		{
			setTargetSpeed(0);
			ssd1306_SetCursor(65, 22);
			ssd1306_printf(Font_6x8, "%4d", lSensorOffset[0]);

			data_select(&trace_test, SW_PUSH);
			if (trace_test)
			{
				cntSetup1 = 0;
				patternCalibration = 2;
			}
			break;
		}
		case 2: // 開始準備
		{
			if (cntSetup1 > 1000)
			{
				ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
				ssd1306_SetCursor(22, 28);
				ssd1306_printf(Font_7x10, "Calibration");
				ssd1306_SetCursor(53, 42);
				ssd1306_printf(Font_7x10, "Now");
				ssd1306_UpdateScreen(); // グラフィック液晶更新

				// 配列初期化
				memset(&lSensorOffset, 0, sizeof(uint16_t) * NUM_SENSORS);

				powerLineSensors(1);	// ラインセンサ点灯
				modeCalLinesensors = 1; // キャリブレーション開始

				// 手動で機体を動かしキャリブレーションする

				patternCalibration = 3;
			}
			break;
		}
		case 3: // スイッチ押下で終了
		{
			data_select(&trace_test, SW_PUSH);
			if (!trace_test)
			{
				modeCalLinesensors = 0;						  // キャリブレーション終了
				powerLineSensors(0);						  // ラインセンサ消灯
				ssd1306_FillRectangle(0, 15, 127, 63, Black); // メイン表示空白埋め
				ssd1306_UpdateScreen();						  // グラフィック液晶更新

				if (initMSD)
				{
					initIMU = false;
					writeLinesenval(); // オフセット値をSDカードに書き込み
					initIMU = true;
				}
				patternCalibration = 1;
			}
			break;
		}

		default:
			break;
		}
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
		if (patternDisplay != beforeHEX)
		{
			// 切替時に実行
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

		data_select(&trace_test, SW_PUSH);
		// PUSHでトレースON/OFF
		if (trace_test == 1)
		{
			powerLineSensors(1);
			setTargetSpeed(0.0);
			motorPwmOutSynth(lineTraceCtrl.pwm, veloCtrl.kp, 0, 0);
		}
		else
		{
			motorPwmOutSynth(0, 0, 0, 0);
			powerLineSensors(0);
		}

		// ゲイン表示
		dataTuningUD(&patternGain, 1, 3, 1);
		if (trace_test == 0)
		{
			ssd1306_SetCursor(21, 18);
			if (patternGain == 1)
				ssd1306_printfB(Font_7x10, "%3d", veloCtrl.kp);
			else
				ssd1306_printf(Font_7x10, "%3d", veloCtrl.kp);
			ssd1306_SetCursor(21, 32);
			if (patternGain == 2)
				ssd1306_printfB(Font_7x10, "%3d", veloCtrl.ki);
			else
				ssd1306_printf(Font_7x10, "%3d", veloCtrl.ki);
			ssd1306_SetCursor(21, 44);
			if (patternGain == 3)
				ssd1306_printfB(Font_7x10, "%3d", veloCtrl.kd);
			else
				ssd1306_printf(Font_7x10, "%3d", veloCtrl.kd);

			// 制御量表示
			ssd1306_SetCursor(88, 30);
			ssd1306_printf(Font_7x10, "%4d", veloCtrl.pwm);

			switch (patternGain)
			{
			case 1:
				// kp
				dataTuningLR(&veloCtrl.kp, 1, 0, 255);
				break;
			case 2:
				// ki
				dataTuningLR(&veloCtrl.ki, 1, 0, 255);
				break;
			case 3:
				// kd
				dataTuningLR(&veloCtrl.kd, 1, 0, 255);
				break;
			}
		}
		break;
	}
	//------------------------------------------------------------------
	// ゲイン調整(角速度)
	//------------------------------------------------------------------
	case HEX_PID_ANGULAR:
	{
		if (patternDisplay != beforeHEX)
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

			setTargetAngularVelocity(0);
			setTargetSpeed(0);
		}

		data_select(&trace_test, SW_PUSH);
		// PUSHでトレースON/OFF
		if (trace_test == 1)
		{
			motorPwmOutSynth(0, veloCtrl.pwm, yawRateCtrl.pwm, 0);
		}
		else
		{
			motorPwmOutSynth(0, 0, 0, 0);
		}
		if (trace_test != beforeMotorTest && trace_test == 0)
		{
			trace_test = 2;
		}
		if (trace_test == 2 && encCurrentL == 0) // ホイールの回転が停止したら0
		{
			trace_test = 0;
		}
		beforeMotorTest = trace_test;

		// ゲイン表示
		dataTuningUD(&patternGain, 1, 3, 1);
		if (trace_test == 0)
		{
			ssd1306_SetCursor(21, 18);
			if (patternGain == 1)
				ssd1306_printfB(Font_7x10, "%3d", yawRateCtrl.kp);
			else
				ssd1306_printf(Font_7x10, "%3d", yawRateCtrl.kp);
			ssd1306_SetCursor(21, 32);
			if (patternGain == 2)
				ssd1306_printfB(Font_7x10, "%3d", yawRateCtrl.ki);
			else
				ssd1306_printf(Font_7x10, "%3d", yawRateCtrl.ki);
			ssd1306_SetCursor(21, 44);
			if (patternGain == 3)
				ssd1306_printfB(Font_7x10, "%3d", yawRateCtrl.kd);
			else
				ssd1306_printf(Font_7x10, "%3d", yawRateCtrl.kd);

			// 制御量表示
			ssd1306_SetCursor(88, 30);
			ssd1306_printf(Font_7x10, "%4d", yawRateCtrl.pwm);

			switch (patternGain)
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
		if (patternDisplay != beforeHEX)
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

			setTargetDist(50.0);
			setTargetSpeed(0.3);
		}

		// ゲイン表示
		dataTuningUD(&patternGain, 1, 3, 1);
		if (trace_test == 0)
		{
			ssd1306_SetCursor(21, 18);
			if (patternGain == 1)
				ssd1306_printfB(Font_7x10, "%3d", distCtrl.kp);
			else
				ssd1306_printf(Font_7x10, "%3d", distCtrl.kp);
			ssd1306_SetCursor(21, 32);
			if (patternGain == 2)
				ssd1306_printfB(Font_7x10, "%3d", distCtrl.ki);
			else
				ssd1306_printf(Font_7x10, "%3d", distCtrl.ki);
			ssd1306_SetCursor(21, 44);
			if (patternGain == 3)
				ssd1306_printfB(Font_7x10, "%3d", distCtrl.kd);
			else
				ssd1306_printf(Font_7x10, "%3d", distCtrl.kd);

			// 制御量表示
			ssd1306_SetCursor(88, 30);
			ssd1306_printf(Font_7x10, "%4d", distCtrl.pwm);

			switch (patternGain)
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
	beforeHEX = patternDisplay;
	beforeBATLV = batteryLevel;

	if (!modeDSP)
	{
		sendLED();
	}

	if (!trace_test && !calibratIMU)
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
// モジュール名 dataTuningUD
// 処理概要     タクトスイッチで整数型dataを加減する
// 引数         data: 加減させる変数 add: 0: 変化量 dir: 0:上下 1:左右
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
void dataTuningUD(int16_t *data, int16_t add, int16_t min, int16_t max)
{
	int16_t sign;

	if (max - min > 0)
	{
		sign = 1;
	}
	else
	{
		sign = -1;
	}

	if (cntSwitchUD >= 50)
	{
		if (swValTact == SW_UP || swValTact == SW_DOWN)
		{
			cntSwitchUDLong++; // 長押し時間計測
			if (swValTact == SW_UP)
			{
				// インクリメント
				if (cntSwitchUDLong >= PUSHTIME)
				{ // 長押し処理
					*data += sign * add;
				}
				else if (pushUD == 0)
				{ // 1回押し処理
					pushUD = 1;
					*data += sign * add;
				}
			}
			else if (swValTact == SW_DOWN)
			{
				// デクリメント
				if (cntSwitchUDLong >= PUSHTIME)
				{ // 長押し処理
					*data -= sign * add;
				}
				else if (pushUD == 0)
				{ // 1回押し処理
					pushUD = 1;
					*data -= sign * add;
				}
			}
		}
		else
		{
			pushUD = 0;
			cntSwitchUDLong = 0;
		}
		cntSwitchUD = 0;

		if (sign > 0)
		{
			if (*data > max)
			{
				*data = min;
			}
			else if (*data < min)
			{
				*data = max;
			}
		}
		else
		{
			if (*data > min)
			{
				*data = max;
			}
			else if (*data < max)
			{
				*data = min;
			}
		}
	}
}
///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 dataTuningLR
// 処理概要     タクトスイッチで整数型dataを加減する
// 引数         data: 加減させる変数 add: 0: 変化量 dir: 0:上下 1:左右
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
void dataTuningLR(int16_t *data, int16_t add, int16_t min, int16_t max)
{
	int16_t sign;

	if (max - min > 0)
	{
		sign = 1;
	}
	else
	{
		sign = -1;
	}

	if (cntSwitchLR >= 50)
	{
		if (swValTact == SW_LEFT || swValTact == SW_RIGHT)
		{
			cntSwitchLRLong++; // 長押し時間計測
			if (swValTact == SW_RIGHT)
			{
				// インクリメント
				if (cntSwitchLRLong >= PUSHTIME)
				{ // 長押し処理
					*data += sign * add;
				}
				else if (pushLR == 0)
				{ // 1回押し処理
					pushLR = 1;
					*data += sign * add;
				}
			}
			else if (swValTact == SW_LEFT)
			{
				// デクリメント
				if (cntSwitchLRLong >= PUSHTIME)
				{ // 長押し処理
					*data -= sign * add;
				}
				else if (pushLR == 0)
				{ // 1回押し処理
					pushLR = 1;
					*data -= sign * add;
				}
			}
		}
		else
		{
			pushLR = 0;
			cntSwitchLRLong = 0;
		}
		cntSwitchLR = 0;

		if (sign > 0)
		{
			if (*data > max)
			{
				*data = min;
			}
			else if (*data < min)
			{
				*data = max;
			}
		}
		else
		{
			if (*data > min)
			{
				*data = max;
			}
			else if (*data < max)
			{
				*data = min;
			}
		}
	}
}
///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 dataTuningUDF
// 処理概要     タクトスイッチでfloat型dataを加減する
// 引数         data: 加減させる変数 add: 0: 変化量 dir: 0:上下 1:左右
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
void dataTuningUDF(float *data, float add, float min, float max)
{
	int16_t sign;

	if (max - min > 0)
	{
		sign = 1;
	}
	else
	{
		sign = -1;
	}

	if (cntSwitchUD >= 50)
	{
		if (swValTact == SW_UP || swValTact == SW_DOWN)
		{
			cntSwitchUDLong++; // 長押し時間計測
			if (swValTact == SW_UP)
			{
				// インクリメント
				if (cntSwitchUDLong >= PUSHTIME)
				{ // 長押し処理
					*data += sign * add;
				}
				else if (pushUD == 0)
				{ // 1回押し処理
					pushUD = 1;
					*data += sign * add;
				}
			}
			else if (swValTact == SW_DOWN)
			{
				// デクリメント
				if (cntSwitchUDLong >= PUSHTIME)
				{ // 長押し処理
					*data -= sign * add;
				}
				else if (pushUD == 0)
				{ // 1回押し処理
					pushUD = 1;
					*data -= sign * add;
				}
			}
		}
		else
		{
			pushUD = 0;
			cntSwitchUDLong = 0;
		}
		cntSwitchUD = 0;

		if (sign > 0)
		{
			if (*data > max)
			{
				*data = min;
			}
			else if (*data < min)
			{
				*data = max;
			}
		}
		else
		{
			if (*data > min)
			{
				*data = max;
			}
			else if (*data < max)
			{
				*data = min;
			}
		}
	}
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

	switch (patternCalibration)
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
			if (lSensorOffset[0] > 0)
			{
				// キャリブレーション実施済み
				start = 1;
			}
			else
			{
				// キャリブレーション未実施
				cntSetup1 = 0;
				enc1 = 0;
				powerLineSensors(1); // 先に点灯させて安定させる
				patternCalibration = 2;
			}
		}
		break;

	case 2:
		// 開始準備
		if (cntSetup1 > 1000)
		{
			veloCtrl.Int = 0;		 // I成分リセット
			BMI088val.angle.z = 0.0; // 角度リセット
			yawRateCtrl.Int = 0.0;	 // I成分リセット
			setTargetSpeed(0);		 // 目標速度0[m/s]
			enc1 = 0;
			modeCalLinesensors = 1; // キャリブレーション開始
			patternCalibration = 3;
		}
		break;

	case 3:
		// 左旋回
		setTargetAngularVelocity(CALIBRATIONSPEED);
		motorPwmOutSynth(0, veloCtrl.pwm, yawRateCtrl.pwm, 0);
		if (BMI088val.angle.z < -35.0)
		{
			patternCalibration = 4;
		}
		break;

	case 4:
		// 停止
		setTargetSpeed(0);
		motorPwmOutSynth(0, veloCtrl.pwm, 0, 0);
		if (abs(encCurrentN) == 0)
		{
			patternCalibration = 5;
		}
		break;

	case 5:
		// 右旋回
		setTargetAngularVelocity(-CALIBRATIONSPEED);
		motorPwmOutSynth(0, veloCtrl.pwm, yawRateCtrl.pwm, 0);
		if (BMI088val.angle.z > 35.0)
		{
			patternCalibration = 6;
		}
		break;

	case 6:
		// 停止
		setTargetSpeed(0);
		motorPwmOutSynth(0, veloCtrl.pwm, 0, 0);
		if (abs(encCurrentN) == 0)
		{
			patternCalibration = 7;
		}
		break;

	case 7:
		// 初期位置に戻る
		setTargetAngularVelocity(CALIBRATIONSPEED);
		motorPwmOutSynth(0, veloCtrl.pwm, yawRateCtrl.pwm, 0);
		if (lSensor[5] < 1000)
		{
			modeCalLinesensors = 0;
			patternCalibration = 8;
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
			start = mode;
		}
		break;

	default:
		break;
	}
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

	switch (patternClick)
	{
	case 1:
		if (clickStart != 0)
		{
			patternClick = 2;
		}
		break;

	case 2:
		motorPwmOut(-pwm * clickStart, 0);
		cnt++;
		if (cnt >= 3)
		{
			cnt = 0;
			patternClick = 3;
		}
		break;

	case 3:
		motorPwmOut(pwm * clickStart, 0);
		cnt++;
		if (cnt >= 3)
		{
			cnt = 0;
			patternClick = 4;
		}
		break;

	case 4:
		motorPwmOut(0, 0);
		clickStart = 0;
		patternClick = 1;
		break;
	}
}