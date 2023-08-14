//====================================//
// インクルード
//====================================//
#include "setup.h"
//====================================//
// グローバル変数の宣言
//====================================//
uint8_t 	start = 0;		// 0:セットアップ中	1:セットアップ完了

// タイマ関連
uint16_t	cntSetup1 = 0;			// セットアップで使用
uint16_t	cntSetup2 = 0;			// セットアップで使用
uint16_t	cntSwitchUD = 0;		// スイッチ判定用右
uint16_t 	cntSwitchLR = 0;		// スイッチ判定用左
uint16_t	cntSwitchUDLong = 0;	// スイッチ長押し判定用右
uint16_t	cntSwitchLRLong = 0;	// スイッチ長押し判定用左

// スイッチ関連
int8_t pushLR = 0;
int8_t pushUD = 0;

// パターン関連
uint8_t push1 = 0;
int16_t patternSensors = 1;
int16_t beforeSensors = 0;
int16_t patternSensorLine = 1;
int16_t patternSensorAccele = 1;
int16_t patternSensorGyro = 1;
int16_t patternParameter1 = 1;
int16_t patternParameter2 = 1;
int16_t patternParameter3 = 1;
int16_t patternParameter4 = 1;
int16_t patternGain = 1;
int16_t patternSpeedseting = 1;
int16_t patternLog = 1;
int16_t patternCalibration = 1;

// フラグ関連
uint8_t motor_test = 0;
uint8_t trace_test = 0;
int16_t	calTimes = 1;
int16_t	calTimesNow = 0;
uint8_t bright = 0;
uint8_t showDisplay = 1;

// パラメータ関連
int16_t motorTestPwm = 200;
///////////////////////////////////////////////////////////////
// モジュール名 setup
// 処理概要     走行前設定
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////
void setup( void )
{
	uint8_t cntLed;
	static uint8_t beforePparam, beforeBATLV, beforeHEX = 255;

	SchmittBatery();    // バッテリレベルを取得
	if (batteryLevel != beforeBATLV) {
		// バッテリレベルが変化したときに実行
		showBattery();	// バッテリ残量表示
	}

	if (swValRotary != beforeHEX) 	{
		// ロータリスイッチ切替時に実行

		// ロータリスイッチ値を表示
		ssd1306_SetCursor(0,3);
		ssd1306_printf(Font_6x8,"No.%x",swValRotary);

		
		ssd1306_FillRectangle(0,15,127,63, Black); // メイン表示空白埋め
		ssd1306_FillRectangle(24,0,94,13, Black); // ヘッダ表示空白埋め
		ssd1306_SetCursor(30,3); // ヘッダタイトル位置
	}

	// ディップスイッチで項目選択
	switch ( swValRotary ) {
		// //------------------------------------------------------------------
		// // スタート待ち
		// //------------------------------------------------------------------
		case HEX_START:
			if (swValRotary != beforeHEX) 	{
				// 切替時に実行
				ssd1306_printf(Font_6x8,"Start  ");
				ssd1306_SetCursor(30,25);
				ssd1306_printf(Font_11x18,"Ready?");
			}

			data_select( &start, SW_PUSH );
			
			break;
		// //------------------------------------------------------------------
		// // パラメータ調整(通常トレース)
		// //------------------------------------------------------------------
		case HEX_SPEED_PARAM:
			if (swValRotary != beforeHEX) 	{
				// 切替時に実行
				ssd1306_printf(Font_6x8,"Parameter");
			}

			dataTuningLR( &patternParameter1, 1, 1, 10);

			beforePparam = patternParameter1;
			if (beforePparam != patternParameter1) {
				ssd1306_FillRectangle(0,16,127,63, Black);
			}

			switch( patternParameter1 ) {
				case 1:
					// 通常走行速度
					dataTuningUDF( &targetParam.straight, 0.1, 0.0, 10.0 );
					ssd1306_SetCursor(0,24);
					ssd1306_printf(Font_6x8,"STRAIGHT:%3gm/s", targetParam.straight);
					break;
				case 2:
					// 停止速度
					dataTuningUDF( &targetParam.curve, 0.1, 0.0, 10.0 );
					ssd1306_SetCursor(0,24);
					ssd1306_printf(Font_6x8,"CURVE:%3gm/s", targetParam.curve);
					break;
				case 3:
					// 停止速度
					dataTuningUDF( &targetParam.stop, 0.1, 0.0, 10.0 );
					ssd1306_SetCursor(0,24);
					ssd1306_printf(Font_6x8,"STOP:%3gm/s", targetParam.stop);
					break;
				case 4:
					// 2次走行_直線
					dataTuningUDF( &targetParam.boostStraight, 0.1, 0.0, 10.0 );
					ssd1306_SetCursor(0,24);
					ssd1306_printf(Font_6x8,"BST STRT:%3gm/s", targetParam.boostStraight);
					break;
				case 5:
					// 2次走行_R1500
					dataTuningUDF( &targetParam.boost1500, 0.1, 0.0, 10.0 );
					ssd1306_SetCursor(0,24);
					ssd1306_printf(Font_6x8,"BST 1500:%3gm/s", targetParam.boost1500);
					break;
				case 6:
					// 2次走行_R800
					dataTuningUDF( &targetParam.boost800, 0.1, 0.0, 10.0 );
					ssd1306_SetCursor(0,24);
					ssd1306_printf(Font_6x8,"BST 800:%3gm/s", targetParam.boost800);
					break;
				case 7:
					// 2次走行_R1600
					dataTuningUDF( &targetParam.boost600, 0.1, 0.0, 10.0 );
					ssd1306_SetCursor(0,24);
					ssd1306_printf(Font_6x8,"BST 600:%3gm/s", targetParam.boost600);
					break;
				case 8:
					// 2次走行_R400
					dataTuningUDF( &targetParam.boost400, 0.1, 0.0, 10.0 );
					ssd1306_SetCursor(0,24);
					ssd1306_printf(Font_6x8,"BST 400:%3gm/s", targetParam.boost400);
					break;
				case 9:
					// 2次走行_R200
					dataTuningUDF( &targetParam.boost200, 0.1, 0.0, 10.0 );
					ssd1306_SetCursor(0,24);
					ssd1306_printf(Font_6x8,"BST 200:%3gm/s", targetParam.boost200);
					break;
				case 10:
					// 2次走行_R200
					dataTuningUDF( &targetParam.boost100, 0.1, 0.0, 10.0 );
					ssd1306_SetCursor(0,24);
					ssd1306_printf(Font_6x8,"BST 100:%3gm/s", targetParam.boost100);
					break;
			}
			break;
		//------------------------------------------------------------------
		// Sensors test
		//------------------------------------------------------------------
		case HEX_SENSORS:
			if (swValRotary != beforeHEX) 	{
				// 切替時に実行
				ssd1306_printf(Font_6x8,"SENSORS  ");
				beforeSensors = 100;
			}

			dataTuningLR( &patternSensors, 1, 1, 6 );
			switch( patternSensors ) {
				case 1:
					if (patternSensors != beforeSensors) 	{
						// 切替時に実行
						ssd1306_FillRectangle(0,16,127,63, Black); // 黒塗り
						ssd1306_SetCursor(47,16);
						ssd1306_printf(Font_6x8,"Motor");
					}
					// Duty
					ssd1306_SetCursor(35,30);
					ssd1306_printf(Font_6x8,"Duty:%4d",motorTestPwm);

					// Left
					ssd1306_SetCursor(0,42);
					ssd1306_printf(Font_6x8,"enc:%5.0f",encTotalL/PALSE_MILLIMETER);	// Encoder
					ssd1306_SetCursor(0,52);
					ssd1306_printf(Font_6x8,"Cur:%5.2f",motorCurrentL); // Current

					// Right
					ssd1306_SetCursor(70,42);
					ssd1306_printf(Font_6x8,"enc:%5.0f",encTotalR/PALSE_MILLIMETER); 	// Encoder
					ssd1306_SetCursor(70,52);
					ssd1306_printf(Font_6x8,"Cur:%5.2f",motorCurrentR); // Current

					dataTuningUD ( &motorTestPwm, 100, -500, 500 );
					data_select( &motor_test, SW_PUSH );
					if ( motor_test == 1 ) {
						motorPwmOut(motorTestPwm,motorTestPwm);
					} else {
						motorPwmOut(0, 0);
					}
					break;

				case 2:
					if (patternSensors != beforeSensors) 	{
						// 切替時に実行
						ssd1306_FillRectangle(0,16,127,63, Black); // 黒塗り
						ssd1306_SetCursor(36,16);
						ssd1306_printf(Font_7x10,"IMU[deg]");
					}
					
					ssd1306_SetCursor(0,30);
					ssd1306_printf(Font_7x10,"xg:%6.1f",BMI088val.gyro.x);
					ssd1306_SetCursor(0,42);
					ssd1306_printf(Font_7x10,"yg:%6.1f",BMI088val.gyro.y);
					ssd1306_SetCursor(0,54);
					ssd1306_printf(Font_7x10,"zg:%6.1f",BMI088val.gyro.z);

					ssd1306_SetCursor(64,30);
					ssd1306_printf(Font_7x10,"xd:%6.1f",BMI088val.angle.x);
					ssd1306_SetCursor(64,42);
					ssd1306_printf(Font_7x10,"yd:%6.1f",BMI088val.angle.y);
					ssd1306_SetCursor(64,54);
					ssd1306_printf(Font_7x10,"zd:%6.1f",BMI088val.angle.z);

					if (swValTact == SW_PUSH) {
						BMI088val.angle.x = 0;
						BMI088val.angle.y = 0;
						BMI088val.angle.z = 0;
					}
					break;
				case 3:
					if (patternSensors != beforeSensors) 	{
						// 切替時に実行
						ssd1306_FillRectangle(0,16,127,63, Black); // 黒塗り
						ssd1306_SetCursor(32,16);
						ssd1306_printf(Font_7x10,"Side sensors");
					}
					ssd1306_SetCursor(0,30);
					ssd1306_printf(Font_7x10,"Marker sensors:%d",getMarkerSensor());

					break;
				case 4:
					if (patternSensors != beforeSensors) 	{
						// 切替時に実行
						ssd1306_FillRectangle(0,16,127,63, Black); // 黒塗り
						ssd1306_SetCursor(32,16);
						ssd1306_printf(Font_7x10,"Switches");
					}
					ssd1306_SetCursor(0,30);
					ssd1306_printf(Font_7x10,"Board SW:%d",swValMainTact);

					ssd1306_SetCursor(0,42);
					ssd1306_printf(Font_7x10,"5axis SW:%d",swValTact);

					break;
				case 5:
					if (patternSensors != beforeSensors) 	{
						// 切替時に実行
						ssd1306_FillRectangle(0,16,127,63, Black); // 黒塗り
						ssd1306_SetCursor(32,16);
						ssd1306_printf(Font_7x10,"Battery");
					}
					ssd1306_SetCursor(0,30);
					ssd1306_printf(Font_7x10,"BatteryValAD:%d",batteryVal);

					ssd1306_SetCursor(0,42);
					ssd1306_printf(Font_7x10,"BatteryLv:%d",batteryLevel);

					break;
				case 6:
					if (patternSensors != beforeSensors) 	{
						// 切替時に実行
						ssd1306_FillRectangle(0,16,127,63, Black); // 黒塗り
						// センサ基板形状
						ssd1306_DrawArc(64,81,66,90,270,White);
						ssd1306_DrawArc(64,81,35,90,270,White);
						ssd1306_Line(2,63,34,63,White);
						ssd1306_Line(93,63,126,63,White);
					}

					if (lSensorOffset[0] > 0 && modeCalLinesensors == 0) {
						ssd1306_SetCursor(37,22);
						ssd1306_printf(Font_6x8,"%4d",lSensorCari[4]);
						ssd1306_SetCursor(31,30);
						ssd1306_printf(Font_6x8,"%4d",lSensorCari[3]);
						ssd1306_SetCursor(22,38);
						ssd1306_printf(Font_6x8,"%4d",lSensorCari[2]);
						ssd1306_SetCursor(13,46);
						ssd1306_printf(Font_6x8,"%4d",lSensorCari[1]);
						ssd1306_SetCursor(6,54);
						ssd1306_printf(Font_6x8,"%4d",lSensorCari[0]);

						ssd1306_SetCursor(65,22);
						ssd1306_printf(Font_6x8,"%4d",lSensorCari[5]);
						ssd1306_SetCursor(71,30);
						ssd1306_printf(Font_6x8,"%4d",lSensorCari[6]);
						ssd1306_SetCursor(80,38);
						ssd1306_printf(Font_6x8,"%4d",lSensorCari[7]);
						ssd1306_SetCursor(89,46);
						ssd1306_printf(Font_6x8,"%4d",lSensorCari[8]);
						ssd1306_SetCursor(95,54);
						ssd1306_printf(Font_6x8,"%4d",lSensorCari[9]);
					} else {
						ssd1306_SetCursor(37,22);
						ssd1306_printf(Font_6x8,"%4d",lSensor[4]);
						ssd1306_SetCursor(31,30);
						ssd1306_printf(Font_6x8,"%4d",lSensor[3]);
						ssd1306_SetCursor(22,38);
						ssd1306_printf(Font_6x8,"%4d",lSensor[2]);
						ssd1306_SetCursor(13,46);
						ssd1306_printf(Font_6x8,"%4d",lSensor[1]);
						ssd1306_SetCursor(6,54);
						ssd1306_printf(Font_6x8,"%4d",lSensor[0]);

						ssd1306_SetCursor(65,22);
						ssd1306_printf(Font_6x8,"%4d",lSensor[5]);
						ssd1306_SetCursor(71,30);
						ssd1306_printf(Font_6x8,"%4d",lSensor[6]);
						ssd1306_SetCursor(80,38);
						ssd1306_printf(Font_6x8,"%4d",lSensor[7]);
						ssd1306_SetCursor(89,46);
						ssd1306_printf(Font_6x8,"%4d",lSensor[8]);
						ssd1306_SetCursor(95,54);
						ssd1306_printf(Font_6x8,"%4d",lSensor[9]);
					}

					data_select( &trace_test, SW_PUSH );
					if ( trace_test == 1 ) {
						powerLinesensors(1);
					} else {
						powerLinesensors(0);
					}

					break;
			}
			beforeSensors = patternSensors;
			break;
		// //------------------------------------------------------------------
		// // Log
		// //------------------------------------------------------------------
		case HEX_LOG:
			if (swValRotary != beforeHEX) 	{
				// 切替時に実行
				ssd1306_printf(Font_6x8,"microSD  ");
			}

			ssd1306_SetCursor(35,24);
			ssd1306_printf(Font_6x8,"init:%d %d",initMSD, modeLOG);

			if (swValTact == SW_UP && !modeLOG ) {
				initLog();
				modeLOG = true;    // log start

			}
			if (swValTact == SW_DOWN && modeLOG) {
				endLog();
			}
			break;
		// //------------------------------------------------------------------
		// // キャリブレーション(ラインセンサ) 
		// //------------------------------------------------------------------
		case HEX_CALIBRATION:
			if (swValRotary != beforeHEX) 	{
				// 切替時に実行
				ssd1306_printf(Font_6x8,"Calibrate");
				patternCalibration = 1;
			}
			// data_select( &trace_test, SW_PUSH );
			// // PUSHでトレースON/OFF
			// if ( trace_test == 1 ) {
			// 	modeCalLinesensors = 1; 	// キャリブレーション開始
			// 	powerLinesensors(1);
			// } else {
			// 	modeCalLinesensors = 0; 	// キャリブレーション終了
			// 	powerLinesensors(0);
			// }

			// ssd1306_SetCursor(0,24);
			// ssd1306_printf(Font_7x10,"flag:%d",modeCalLinesensors);
			// break;

			switch (patternCalibration) {
				case 1:
					// スイッチ入力待ち
					dataTuningUD( &calTimes, 1, 1, 9);

					setTargetSpeed(0);
					motorPwmOutSynth( 0, veloCtrl.pwm, 0, 0);
					if (swValTact == SW_PUSH) {
						cntSetup1 = 0;
						enc1 = 0;
						powerLinesensors(1);	// 先に点灯させて安定させる
						patternCalibration = 2;
					}
					break;

				case 2:
					// 開始準備
					if (cntSetup1 > 1000) {
						BMI088val.angle.z = 0.0;	// 角度リセット
						yawRateCtrl.Int = 0.0;		// I成分リセット
						useIMU = true;
						setTargetSpeed(0);			// 目標速度0[m/s]
						enc1 = 0;
						modeCalLinesensors = 1; 	// キャリブレーション開始
						patternCalibration = 3;
					}
					break;

				case 3:
					// 左旋回
					setTargetAngularVelocity(CALIBRATIONSPEED);
					motorPwmOutSynth(0, veloCtrl.pwm, yawRateCtrl.pwm, 0);
					if (BMI088val.angle.z > 45.0) {
						patternCalibration = 4;
					}
					break;

				case 4:
					// 停止
					setTargetSpeed(0);
					motorPwmOutSynth(0, veloCtrl.pwm, 0, 0);
					if (abs(encCurrentN) == 0) {
						patternCalibration = 5;
					}
					break;

				case 5:
					// 右旋回
					setTargetAngularVelocity(-CALIBRATIONSPEED);
					motorPwmOutSynth(0, veloCtrl.pwm, yawRateCtrl.pwm, 0);
					if (BMI088val.angle.z < -45) {
						patternCalibration = 6;
					}
					break;

				case 6:
					// 停止
					setTargetSpeed(0);
					motorPwmOutSynth(0, veloCtrl.pwm, 0, 0);
					if (abs(encCurrentN) == 0) {
						patternCalibration = 7;
					}
					break;

				case 7:
					// 初期位置に戻る
					setTargetAngularVelocity(CALIBRATIONSPEED);
					motorPwmOutSynth(0, veloCtrl.pwm, yawRateCtrl.pwm, 0);
					if (BMI088val.angle.z > 0) {
						modeCalLinesensors = 0;
						patternCalibration = 8;
					}
					break;

				case 8:
					// 停止
					motorPwmOutSynth( 0, veloCtrl.pwm, 0, 0);
					if (abs(encCurrentN) == 0) {
						calTimesNow++;
						if (calTimesNow >= calTimes) {
							calTimesNow = 0;
							useIMU = false;
							patternCalibration = 1;
						} else {
							patternCalibration = 3;
						}
					}
					break;
			
				default:
					break;
				}
			break;
		// //------------------------------------------------------------------
		// // ゲイン調整(直線トレース)
		// //------------------------------------------------------------------
		case HEX_PID_TRACE:
			if (swValRotary != beforeHEX) 	{
				// 切替時に実行
				ssd1306_printf(Font_6x8,"Trace PID");
			}

			
			data_select( &trace_test, SW_PUSH );
			// PUSHでトレースON/OFF
			if ( trace_test == 1 ) {
				showDisplay = 0;
				motorPwmOutSynth( lineTraceCtrl.pwm, 0, 0, 0);
				powerLinesensors(1);
			} else {
				showDisplay = 1;
				motorPwmOutSynth( 0, 0, 0, 0);
				powerLinesensors(0);
			}
			
			dataTuningLR( &patternGain, 1, 1, 3);

			if (trace_test == 0) {
				if ( (cntSetup1 / 250) % 2 != 0) {
					ssd1306_SetCursor(0,24);
					ssd1306_printf(Font_6x8,"kp:%3d",lineTraceCtrl.kp);
					ssd1306_SetCursor(0,34);
					ssd1306_printf(Font_6x8,"ki:%3d",lineTraceCtrl.ki);
					ssd1306_SetCursor(0,44);
					ssd1306_printf(Font_6x8,"kd:%3d",lineTraceCtrl.kd);

					ssd1306_SetCursor(70,34);
					ssd1306_printf(Font_6x8,"pwm:%5d",lineTraceCtrl.pwm);
				}
			
				switch( patternGain ) {
					case 1:
						// kp
						dataTuningUD ( &lineTraceCtrl.kp, 1, 0, 255 );
						//値を点滅
						if ( (cntSetup1 / 250) % 2 == 0 ) {
							ssd1306_SetCursor(0,24);
							ssd1306_printf(Font_6x8,"kp:   ");
						}
						break;
					case 2:
						// ki
						dataTuningUD ( &lineTraceCtrl.ki, 1, 0, 255 );
						//値を点滅
						if ( (cntSetup1 / 250) % 2 == 0 ) {
							ssd1306_SetCursor(0,34);
							ssd1306_printf(Font_6x8,"ki:   ");
						}
						break;
					case 3:
						// kd
						dataTuningUD ( &lineTraceCtrl.kd, 1, 0, 255 );
						//値を点滅
						if ( (cntSetup1 / 250) % 2 == 0 ) {
							ssd1306_SetCursor(0,44);
							ssd1306_printf(Font_6x8,"kd:   ");
						}
						break;
				}
			}
			
			break;
		// //------------------------------------------------------------------
		// // ゲイン調整(速度)
		// //------------------------------------------------------------------
		case HEX_PID_SPEED:
			if (swValRotary != beforeHEX) 	{
				// 切替時に実行
				ssd1306_printf(Font_6x8,"Speed PID");
			}

			data_select( &trace_test, SW_PUSH );
			// PUSHでトレースON/OFF
			if ( trace_test == 1 ) {
				showDisplay = 0;
				powerLinesensors(1);
				setTargetSpeed(0.0);
				motorPwmOutSynth( lineTraceCtrl.pwm, veloCtrl.kp, 0, 0);
			} else {
				showDisplay = 1;
				motorPwmOutSynth( 0, 0, 0, 0);
				powerLinesensors(0);
			}

			dataTuningLR( &patternGain, 1, 1, 3);
			
			if (trace_test == 0) {
				if ( (cntSetup1 / 250) % 2 != 0) {
					ssd1306_SetCursor(0,24);
					ssd1306_printf(Font_6x8,"kp:%3d",veloCtrl.kp);
					ssd1306_SetCursor(0,34);
					ssd1306_printf(Font_6x8,"ki:%3d",veloCtrl.ki);
					ssd1306_SetCursor(0,44);
					ssd1306_printf(Font_6x8,"kd:%3d",veloCtrl.kd);

					ssd1306_SetCursor(70,34);
					ssd1306_printf(Font_6x8,"pwm:%5d",veloCtrl.pwm);
				}
			
				switch( patternGain ) {
					case 1:
						// kp
						dataTuningUD ( &veloCtrl.kp, 1, 0, 255 );
						//値を点滅
						if ( (cntSetup1 / 250) % 2 == 0 ) {
							ssd1306_SetCursor(0,24);
							ssd1306_printf(Font_6x8,"kp:   ");
						}
						break;
					case 2:
						// ki
						dataTuningUD ( &veloCtrl.ki, 1, 0, 255 );
						//値を点滅
						if ( (cntSetup1 / 250) % 2 == 0 ) {
							ssd1306_SetCursor(0,34);
							ssd1306_printf(Font_6x8,"ki:   ");
						}
						break;
					case 3:
						// kd
						dataTuningUD ( &veloCtrl.kd, 1, 0, 255 );
						//値を点滅
						if ( (cntSetup1 / 250) % 2 == 0 ) {
							ssd1306_SetCursor(0,44);
							ssd1306_printf(Font_6x8,"kd:   ");
						}
						break;
				}
			}
		break;
		// //------------------------------------------------------------------
		// // ゲイン調整(角速度)
		// //------------------------------------------------------------------
		case HEX_PID_ANGULAR:
			if (swValRotary != beforeHEX) 	{
				// 切替時に実行
				ssd1306_printf(Font_6x8,"Angle PID");
			}
			
			setTargetAngularVelocity(0);
			setTargetSpeed(0);

			data_select( &trace_test, SW_PUSH );
			// PUSHでトレースON/OFF
			if ( trace_test == 1 ) {
				showDisplay = 0;
				useIMU = true;
				motorPwmOutSynth( 0, veloCtrl.pwm, yawRateCtrl.pwm, 0 );
			} else {
				showDisplay = 1;
				useIMU = false;
				motorPwmOutSynth( 0, 0, 0, 0 );
			}

			dataTuningLR( &patternGain, 1, 1, 3);
			
			if (trace_test == 0) {
				if ( (cntSetup1 / 250) % 2 != 0) {
					ssd1306_SetCursor(0,24);
					ssd1306_printf(Font_6x8,"kp:%3d",yawRateCtrl.kp);
					ssd1306_SetCursor(0,34);
					ssd1306_printf(Font_6x8,"ki:%3d",yawRateCtrl.ki);
					ssd1306_SetCursor(0,44);
					ssd1306_printf(Font_6x8,"kd:%3d",yawRateCtrl.kd);

					ssd1306_SetCursor(70,34);
					ssd1306_printf(Font_6x8,"pwm:%5d",yawRateCtrl.pwm);
				}
			
				switch( patternGain ) {
					case 1:
						// kp
						dataTuningUD ( &yawRateCtrl.kp, 1, 0, 255 );
						//値を点滅
						if ( (cntSetup1 / 250) % 2 == 0 ) {
							ssd1306_SetCursor(0,24);
							ssd1306_printf(Font_6x8,"kp:   ");
						}
						break;
					case 2:
						// ki
						dataTuningUD ( &yawRateCtrl.ki, 1, 0, 255 );
						//値を点滅
						if ( (cntSetup1 / 250) % 2 == 0 ) {
							ssd1306_SetCursor(0,34);
							ssd1306_printf(Font_6x8,"ki:   ");
						}
						break;
					case 3:
						// kd
						dataTuningUD ( &yawRateCtrl.kd, 1, 0, 255 );
						//値を点滅
						if ( (cntSetup1 / 250) % 2 == 0 ) {
							ssd1306_SetCursor(0,44);
							ssd1306_printf(Font_6x8,"kd:   ");
						}
						break;
				}
			}
			break;
		// //------------------------------------------------------------------
		// // ゲイン調整(角度)
		// //------------------------------------------------------------------
		// case HEX_PID_ANGLE:
			
		// 	break;

	default:
		ssd1306_SetCursor(30,5);
		ssd1306_printf(Font_6x8,"None      ");
		ssd1306_FillRectangle(0,16,127,63, Black);

		break;
	} // switch

	// 前回値更新
	beforeHEX = swValRotary;
	beforeBATLV= batteryLevel;

	if (showDisplay) {
		ssd1306_UpdateScreen();  // グラフィック液晶更新
	}
}
///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 data_select
// 処理概要     タクトスイッチで0,1に変化させる
// 引数         data: 変化させる変数 button: どのスイッチで変化させるか
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
void data_select ( uint8_t *data , uint8_t button ) {
	static uint8_t push = 0;

	if ( swValTact == button ) {
		if ( *data == 1 && push == 0) {
			push = 1;
			*data = 0;
		} else if ( *data == 0 && push == 0) {
			push = 1;
			*data = 1;
		}
	} else {
		push = 0;
	}
}
///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 dataTuningUD
// 処理概要     タクトスイッチで整数型dataを加減する
// 引数         data: 加減させる変数 add: 0: 変化量 dir: 0:上下 1:左右
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
void dataTuningUD ( int16_t *data, int16_t add, int16_t min, int16_t max) {
	if ( cntSwitchUD >= 50 ) {
		if ( swValTact == SW_UP || swValTact == SW_DOWN ) {
			cntSwitchUDLong++; // 長押し時間計測
			if ( swValTact == SW_UP  ) {
				// インクリメント
				if ( cntSwitchUDLong >= 20 ) {	// 長押し処理
					*data += add;
				} else if (pushUD == 0) {	// 1回押し処理
					pushUD = 1;
					*data += add;
				}
			} else if ( swValTact == SW_DOWN  ) {
				// デクリメント
				if ( cntSwitchUDLong >= 20 ) {	// 長押し処理
					*data -= add;
				} else if (pushUD == 0) {	// 1回押し処理
					pushUD = 1;
					*data -= add;
				}
			}
		} else {
			pushUD = 0;
			cntSwitchUDLong = 0;
		}
		cntSwitchUD = 0;

		if ( *data > max) {
			*data = min;
		} else if ( *data < min ) {
			*data = max;
		}
	}
}
///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 dataTuningLR
// 処理概要     タクトスイッチで整数型dataを加減する
// 引数         data: 加減させる変数 add: 0: 変化量 dir: 0:上下 1:左右
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
void dataTuningLR ( int16_t *data, int16_t add, int16_t min, int16_t max) {
	if ( cntSwitchLR >= 50 ) {
		if ( swValTact == SW_LEFT || swValTact == SW_RIGHT ) {
			cntSwitchLRLong++; // 長押し時間計測
			if ( swValTact == SW_RIGHT  ) {
				// インクリメント
				if ( cntSwitchLRLong >= 20 ) {	// 長押し処理
					*data += add;
				} else if (pushLR == 0) {	// 1回押し処理
					pushLR = 1;
					*data += add;
				}
			} else if ( swValTact == SW_LEFT  ) {
				// デクリメント
				if ( cntSwitchLRLong >= 20 ) {	// 長押し処理
					*data -= add;
				} else if (pushLR == 0) {	// 1回押し処理
					pushLR = 1;
					*data -= add;
				}
			}
		} else {
			pushLR = 0;
			cntSwitchLRLong = 0;
		}
		cntSwitchLR = 0;

		if ( *data > max) {
			*data = min;
		} else if ( *data < min ) {
			*data = max;
		}
	}
}
///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 dataTuningUDF
// 処理概要     タクトスイッチでfloat型dataを加減する
// 引数         data: 加減させる変数 add: 0: 変化量 dir: 0:上下 1:左右
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
void dataTuningUDF ( float *data, float add, float min, float max) {
	if ( cntSwitchUD >= 50 ) {
		if ( swValTact == SW_UP || swValTact == SW_DOWN ) {
			cntSwitchUDLong++; // 長押し時間計測
			if ( swValTact == SW_UP  ) {
				// インクリメント
				if ( cntSwitchUDLong >= 20 ) {	// 長押し処理
					*data += add;
				} else if (pushUD == 0) {	// 1回押し処理
					pushUD = 1;
					*data += add;
				}
			} else if ( swValTact == SW_DOWN  ) {
				// デクリメント
				if ( cntSwitchUDLong >= 20 ) {	// 長押し処理
					*data -= add;
				} else if (pushUD == 0) {	// 1回押し処理
					pushUD = 1;
					*data -= add;
				}
			}
		} else {
			pushUD = 0;
			cntSwitchUDLong = 0;
		}
		cntSwitchUD = 0;

		if ( *data > max) {
			*data = min;
		} else if ( *data < min ) {
			*data = max;
		}
	}
}
