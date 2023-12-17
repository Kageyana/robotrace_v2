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
int16_t patternDisplay = 0;
int16_t patternSensors = 1;
int16_t beforeSensors = 0;
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
int16_t patternClick=1;

// フラグ関連
uint8_t motor_test = 0;
uint8_t trace_test = 0;
uint8_t clickStart=0;

// パラメータ関連
int16_t motorTestPwm = 200;
int32_t encClick = 0;
///////////////////////////////////////////////////////////////
// モジュール名 setup
// 処理概要     走行前設定
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////
void setup( void )
{
	uint8_t cntLed,i,j,k;
	static uint8_t beforePparam, beforeBATLV, beforeHEX = 255;
	static int16_t x=0,y=0;

	SchmittBatery();    // バッテリレベルを取得
	if (batteryLevel != beforeBATLV) {
		// バッテリレベルが変化したときに実行
		showBattery();	// バッテリ残量表示
	}

#ifdef USE_WHEEL
	// 右ホイールをロータリスイッチ代わりに使用する
	if (!trace_test && !motor_test ) {
		if(abs(encClick) > 400) {
			if(encClick > 400) patternDisplay++;
			else if(encClick < -400) patternDisplay--;

			if(patternDisplay > 0x9) patternDisplay = 0;
			else if(patternDisplay < 0) patternDisplay = 0x9;

			clickStart = 1;
			encClick = 0;
		}
	}
#else
	// ロータリスイッチ値を使用する
	patternDisplay = swValRotary;
#endif

	if (patternDisplay != beforeHEX) 	{
		// ロータリスイッチ切替時に実行

		// ロータリスイッチ値を表示
		ssd1306_SetCursor(0,3);
		ssd1306_printf(Font_6x8,"No.%x",patternDisplay);

		
		ssd1306_FillRectangle(0,15,127,63, Black);	// メイン表示空白埋め
		ssd1306_FillRectangle(24,0,94,13, Black);	// ヘッダ表示空白埋め
		ssd1306_SetCursor(30,3); // ヘッダタイトル位置

		showBattery();	// バッテリ残量表示
	}

	// ディップスイッチで項目選択
	switch ( patternDisplay ) {
		//------------------------------------------------------------------
		// スタート待ち
		//------------------------------------------------------------------
		case HEX_START:
			if (patternDisplay != beforeHEX) 	{
				// 切替時に実行
				ssd1306_printf(Font_6x8,"Start  ");
				ssd1306_SetCursor(30,25);
				ssd1306_printf(Font_11x18,"Ready?");
				patternCalibration = 1;
			}
		
			switch (patternCalibration) {
				case 1:
					setTargetSpeed(0);
					data_select( &trace_test, SW_PUSH );
					// スイッチ入力待ち
					if (trace_test) {
						veloCtrl.Int = 0;	// I成分リセット
						if(lSensorOffset[0] > 0) {
							// キャリブレーション実施済み
							start = 1;
						} else {
							// キャリブレーション未実施
							ssd1306_FillRectangle(0,15,127,63, Black); // メイン表示空白埋め
							ssd1306_SetCursor(22,28);
							ssd1306_printf(Font_7x10,"Calibration");
							ssd1306_SetCursor(53,42);
							ssd1306_printf(Font_7x10,"Now");
							ssd1306_UpdateScreen();  // グラフィック液晶更新

							cntSetup1 = 0;
							enc1 = 0;
							powerLinesensors(1);	// 先に点灯させて安定させる
							patternCalibration = 2;
						}
					}
					break;

				case 2:
					// 開始準備
					if (cntSetup1 > 1000) {
						veloCtrl.Int = 0;			// I成分リセット
						BMI088val.angle.z = 0.0;	// 角度リセット
						yawRateCtrl.Int = 0.0;		// I成分リセット
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
					if (BMI088val.angle.z < -320.0) {
						patternCalibration = 4;
					}
					break;

				case 4:
					// 初期位置に戻る
					setTargetAngularVelocity(-400.0F);
					motorPwmOutSynth(0, veloCtrl.pwm, yawRateCtrl.pwm, 0);
					if (lSensor[5] < 1000) {
						modeCalLinesensors = 0;
						powerLinesensors(0);	// ラインセンサ消灯
						countdown = 500;
						patternCalibration = 5;
					}
					break;

				case 5:
					// 停止
					motorPwmOutSynth( lineTraceCtrl.pwm, veloCtrl.pwm, 0, 0);
					if (countdown <= 0) {
						start = 1;
					}
					break;
			
				default:
					break;
			}
			break;
		//------------------------------------------------------------------
		// パラメータ調整(通常トレース)
		//------------------------------------------------------------------
		case HEX_SPEED_PARAM:
			if (patternDisplay != beforeHEX) 	{
				// 切替時に実行
				ssd1306_printf(Font_6x8,"Parameter");
			}

			dataTuningLR( &patternParameter1, 1, 1, 13);

			if (beforePparam != patternParameter1) {
				ssd1306_FillRectangle(0,16,127,63, Black);
			}

			switch( patternParameter1 ) {
				case 1:
					// 通常走行速度
					dataTuningUDF( &tgtParam.straight, 0.1, 0.0, 10.0 );
					ssd1306_SetCursor(0,24);
					ssd1306_printf(Font_6x8,"STRAIGHT:%3gm/s", tgtParam.straight);
					break;
				case 2:
					// 停止速度
					dataTuningUDF( &tgtParam.curve, 0.1, 0.0, 10.0 );
					ssd1306_SetCursor(0,24);
					ssd1306_printf(Font_6x8,"CURVE:%3gm/s", tgtParam.curve);
					break;
				case 3:
					// 停止速度
					dataTuningUDF( &tgtParam.stop, 0.1, 0.0, 10.0 );
					ssd1306_SetCursor(0,24);
					ssd1306_printf(Font_6x8,"STOP:%3gm/s", tgtParam.stop);
					break;
				case 4:
					// 2次走行_直線
					dataTuningUDF( &tgtParam.bstStraight, 0.1, 0.0, 10.0 );
					ssd1306_SetCursor(0,24);
					ssd1306_printf(Font_6x8,"BST STRT:%3gm/s", tgtParam.bstStraight);
					break;
				case 5:
					// 2次走行_R1500
					dataTuningUDF( &tgtParam.bst1500, 0.1, 0.0, 10.0 );
					ssd1306_SetCursor(0,24);
					ssd1306_printf(Font_6x8,"BST 1500:%3gm/s", tgtParam.bst1500);
					break;
				case 6:
					// 2次走行_R800
					dataTuningUDF( &tgtParam.bst800, 0.1, 0.0, 10.0 );
					ssd1306_SetCursor(0,24);
					ssd1306_printf(Font_6x8,"BST 800:%3gm/s", tgtParam.bst800);
					break;
				case 7:
					// 2次走行_R1600
					dataTuningUDF( &tgtParam.bst600, 0.1, 0.0, 10.0 );
					ssd1306_SetCursor(0,24);
					ssd1306_printf(Font_6x8,"BST 600:%3gm/s", tgtParam.bst600);
					break;
				case 8:
					// 2次走行_R400
					dataTuningUDF( &tgtParam.bst400, 0.1, 0.0, 10.0 );
					ssd1306_SetCursor(0,24);
					ssd1306_printf(Font_6x8,"BST 400:%3gm/s", tgtParam.bst400);
					break;
				case 9:
					// 2次走行_R200
					dataTuningUDF( &tgtParam.bst200, 0.1, 0.0, 10.0 );
					ssd1306_SetCursor(0,24);
					ssd1306_printf(Font_6x8,"BST 200:%3gm/s", tgtParam.bst200);
					break;
				case 10:
					// 2次走行_R100
					dataTuningUDF( &tgtParam.bst100, 0.1, 0.0, 10.0 );
					ssd1306_SetCursor(0,24);
					ssd1306_printf(Font_6x8,"BST 100:%3gm/s", tgtParam.bst100);
					break;
				case 11:
					// 2次走行_加速度
					dataTuningUDF( &tgtParam.acceleF, 0.1, 0.0, 10.0 );
					ssd1306_SetCursor(0,24);
					ssd1306_printf(Font_6x8,"BST acceleF:%3gm/ss", tgtParam.acceleF);
					break;
				case 12:
					// 2次走行_減速度
					dataTuningUDF( &tgtParam.acceleD, 0.1, 0.0, 10.0 );
					ssd1306_SetCursor(0,24);
					ssd1306_printf(Font_6x8,"BST acceleD:%3gm/ss", tgtParam.acceleD);
					break;
				case 13:
					// 2次走行_減速度
					dataTuningUDF( &tgtParam.shortCut, 0.1, 0.0, 10.0 );
					ssd1306_SetCursor(0,24);
					ssd1306_printf(Font_6x8,"BST shortCut:%3gm/s", tgtParam.shortCut);
					break;
			}
			beforePparam = patternParameter1;
			break;
		//------------------------------------------------------------------
		// Sensors test
		//------------------------------------------------------------------
		case HEX_SENSORS:
			if (patternDisplay != beforeHEX) 	{
				// 切替時に実行
				ssd1306_printf(Font_6x8,"SENSORS  ");
				beforeSensors = 100;
			}

			dataTuningLR( &patternSensors, 1, 1, 7 );
			switch( patternSensors ) {
				case 1:
					if (patternSensors != beforeSensors) 	{
						// 切替時に実行
						ssd1306_FillRectangle(0,16,127,63, Black); // 黒塗り
						ssd1306_SetCursor(47,16);
						ssd1306_printf(Font_6x8,"Motor");
						motor_test = 0;					
					}
					// Duty
					ssd1306_SetCursor(35,30);
					ssd1306_printf(Font_6x8,"Duty:%4d",motorTestPwm);

					// Left
					ssd1306_SetCursor(0,42);
					ssd1306_printf(Font_6x8,"enc:%5.0f",encTotalL/PALSE_MILLIMETER);	// Encoder
					ssd1306_SetCursor(0,52);
					ssd1306_printf(Font_6x8,"Cur:%5.2f",motorCurrentL); // Current

					// // Right
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
					
					if(!calibratIMU) {
						// ssd1306_SetCursor(0,30);
						// ssd1306_printf(Font_7x10,"xg:%6.1f",BMI088val.gyro.x);
						// ssd1306_SetCursor(0,42);
						// ssd1306_printf(Font_7x10,"yg:%6.1f",BMI088val.gyro.y);
						// ssd1306_SetCursor(0,54);
						// ssd1306_printf(Font_7x10,"zg:%6.1f",BMI088val.gyro.z);

						ssd1306_SetCursor(64,30);
						ssd1306_printf(Font_7x10,"xd:%6.1f",BMI088val.angle.x);
						ssd1306_SetCursor(64,42);
						ssd1306_printf(Font_7x10,"yd:%6.1f",BMI088val.angle.y);
						ssd1306_SetCursor(64,54);
						ssd1306_printf(Font_7x10,"zd:%6.1f",BMI088val.angle.z);
					}
					

					if (swValTact == SW_PUSH) {
						BMI088val.angle.x = 0;
						BMI088val.angle.y = 0;
						BMI088val.angle.z = 0;
					}

					if (swValTact == SW_UP) {
						ssd1306_FillRectangle(0,15,127,63, Black); // メイン表示空白埋め
						ssd1306_SetCursor(22,28);
						ssd1306_printf(Font_7x10,"Calibration");
						ssd1306_SetCursor(53,42);
						ssd1306_printf(Font_7x10,"Now");
						ssd1306_UpdateScreen();

						calibratIMU = true;
						HAL_Delay(800);
					}
					break;
				case 3:
					if (patternSensors != beforeSensors) 	{
						// 切替時に実行
						ssd1306_FillRectangle(0,16,127,63, Black); // 黒塗り
						ssd1306_SetCursor(36,16);
						ssd1306_printf(Font_7x10,"IMU[g]");
					}
					
					ssd1306_SetCursor(0,30);
					ssd1306_printf(Font_7x10,"xa:%6.1f",BMI088val.accele.x);
					ssd1306_SetCursor(0,42);
					ssd1306_printf(Font_7x10,"ya:%6.1f",BMI088val.accele.y);
					ssd1306_SetCursor(0,54);
					ssd1306_printf(Font_7x10,"za:%6.1f",BMI088val.accele.z);

					ssd1306_SetCursor(64,30);
					ssd1306_printf(Font_7x10,"T:%4.1f",BMI088val.temp);
					break;
				case 4:
					if (patternSensors != beforeSensors) 	{
						// 切替時に実行
						ssd1306_FillRectangle(0,16,127,63, Black); // 黒塗り
						ssd1306_SetCursor(32,16);
						ssd1306_printf(Font_7x10,"Side sensors");
					}
					ssd1306_SetCursor(0,30);
					ssd1306_printf(Font_7x10,"Marker sensors:%d",getMarkerSensor());

					break;
				case 5:
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
				case 6:
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
				case 7:
					if (patternSensors != beforeSensors) 	{
						// 切替時に実行
						ssd1306_FillRectangle(0,16,127,63, Black); // 黒塗り
						// センサ基板形状
						ssd1306_DrawArc(64,81,66,90,270,White);
						ssd1306_DrawArc(64,81,35,90,270,White);
						ssd1306_Line(2,63,34,63,White);
						ssd1306_Line(93,63,126,63,White);
						motor_test = 0;
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

					data_select( &motor_test, SW_PUSH );
					if ( motor_test == 1 ) {
						powerLinesensors(1);
					} else {
						powerLinesensors(0);
					}

					break;
			}
			beforeSensors = patternSensors;
			break;
		//------------------------------------------------------------------
		// Log analysis
		//------------------------------------------------------------------
		case HEX_LOG:
			if (patternDisplay != beforeHEX) 	{
				// 切替時に実行
				ssd1306_printf(Font_6x8,"microSD  ");

				// 前回解析したログ番号を探す
				k = endFileIndex-1;
				for(i=0;i<5;i++) {
					if(0+(20*i) < 128 || k >= 0) {
						for(j=0;j<5;j++) {
							if(24+(8*j) < 64 || k >= 0) {
								if(k == fileIndexLog) {
									x = i;
									y = j;
									break;
								}
								k--;
							} else {
								break;
							}
						}
						if(k == fileIndexLog) {
							break;
						}
					} else {
						break;
					}
				}
			}

			ssd1306_SetCursor(35,16);
			ssd1306_printf(Font_6x8,"init:%d %7d",initMSD,numPPADarry);

			dataTuningUD( &y, 1, 4, 0);
			dataTuningLR( &x, 1, 0, 4);
			
			// i-jとx-yが一致したとき文字色反転
			k = endFileIndex-1;
			for(i=0;i<5;i++) {
				if(0+(20*i) < 128 || k >= 0) {
					for(j=0;j<5;j++) {
						if(24+(8*j) < 64 || k >= 0) {
							ssd1306_SetCursor(0+(20*i),24+(8*j));	// 原点 0,24
							if ( i == x && j == y ) {
								if (swValTact == SW_PUSH) {
									initIMU = false;
									// 距離基準解析
									// numPPADarry = k;
									numPPADarry = calcXYcies(fileNumbers[k]);
									// numPPADarry = readLogDistance(fileNumbers[k]);
									// numPPADarry = readLogTest(fileNumbers[k]);

									if (numPPADarry > 0) {
										optimalIndex = 0;
										HAL_Delay(100);
									}
									initIMU = true;
								}
								ssd1306_printfB(Font_6x8,"%d",fileNumbers[k]);
							} else {
								ssd1306_printf(Font_6x8,"%d",fileNumbers[k]);
							}
							k--;
						} else {
							break;
						}
					}
				} else {
					break;
				}
			}

			break;
		//------------------------------------------------------------------
		// キャリブレーション(ラインセンサ) 
		//------------------------------------------------------------------
		case HEX_CALIBRATION:
			if (patternDisplay != beforeHEX) 	{
				// 切替時に実行
				ssd1306_printf(Font_6x8,"Calibrate");
				patternCalibration = 1;
			}

			switch (patternCalibration) {
				case 1:
					// スイッチ入力待ち
					setTargetSpeed(0);
					ssd1306_SetCursor(65,22);
					ssd1306_printf(Font_6x8,"%4d",lSensorOffset[0]);

					data_select( &trace_test, SW_PUSH );
					if (trace_test) {
						cntSetup1 = 0;
						patternCalibration = 2;
					}
					break;

				case 2:
					// 開始準備
					if (cntSetup1 > 1000) {
						ssd1306_FillRectangle(0,15,127,63, Black); // メイン表示空白埋め
						ssd1306_SetCursor(22,28);
						ssd1306_printf(Font_7x10,"Calibration");
						ssd1306_SetCursor(53,42);
						ssd1306_printf(Font_7x10,"Now");
						ssd1306_UpdateScreen();  // グラフィック液晶更新

						// 配列初期化
						memset(&lSensorOffset, 0, sizeof(uint16_t) * NUM_SENSORS);

						powerLinesensors(1);		// ラインセンサ点灯
						modeCalLinesensors = 1; 	// キャリブレーション開始

						// 手動で機体を動かしキャリブレーションする
						
						patternCalibration = 3;
					}
					break;
				
				case 3:
					// スイッチ押下で終了
					data_select( &trace_test, SW_PUSH );
					if (!trace_test) {
						modeCalLinesensors = 0; 	// キャリブレーション終了
						powerLinesensors(0);		// ラインセンサ消灯
						ssd1306_FillRectangle(0,15,127,63, Black); // メイン表示空白埋め
						ssd1306_UpdateScreen();  // グラフィック液晶更新

						if(initMSD) {
							initIMU = false;
							writeLinesenval();	// オフセット値をSDカードに書き込み
							initIMU = true;
						}
						patternCalibration = 1;
					}
					break;
		
				default:
					break;
				}
			break;
		//------------------------------------------------------------------
		// ゲイン調整(直線トレース)
		//------------------------------------------------------------------
		case HEX_PID_TRACE:
			if (patternDisplay != beforeHEX) 	{
				// 切替時に実行
				ssd1306_printf(Font_6x8,"Trace PID");

				ssd1306_SetCursor(0,18);
				ssd1306_printf(Font_7x10,"kp:");
				ssd1306_SetCursor(0,32);
				ssd1306_printf(Font_7x10,"ki:");
				ssd1306_SetCursor(0,44);
				ssd1306_printf(Font_7x10,"kd:");
				ssd1306_SetCursor(60,30);
				ssd1306_printf(Font_7x10,"pwm:");
			}

			
			data_select( &trace_test, SW_PUSH );
			// PUSHでトレースON/OFF
			if ( trace_test == 1 ) {
				motorPwmOutSynth( lineTraceCtrl.pwm, 0, 0, 0);
				powerLinesensors(1);
			} else {
				motorPwmOutSynth( 0, 0, 0, 0);
				powerLinesensors(0);
			}
			
			// ゲイン表示
			dataTuningUD( &patternGain, 1, 3, 1);
			if (trace_test == 0) {
				ssd1306_SetCursor(21,18);
				if (patternGain == 1) 	ssd1306_printfB(Font_7x10,"%3d",lineTraceCtrl.kp);
				else 					ssd1306_printf(Font_7x10,"%3d",lineTraceCtrl.kp);
				ssd1306_SetCursor(21,32);
				if (patternGain == 2)	ssd1306_printfB(Font_7x10,"%3d",lineTraceCtrl.ki);
				else 					ssd1306_printf(Font_7x10,"%3d",lineTraceCtrl.ki);
				ssd1306_SetCursor(21,44);
				if (patternGain == 3) 	ssd1306_printfB(Font_7x10,"%3d",lineTraceCtrl.kd);
				else 					ssd1306_printf(Font_7x10,"%3d",lineTraceCtrl.kd);
				
				// 制御量表示
				ssd1306_SetCursor(88,30);
				ssd1306_printf(Font_7x10,"%4d",lineTraceCtrl.pwm);
			
				switch( patternGain ) {
					case 1:
						// kp
						dataTuningLR ( &lineTraceCtrl.kp, 1, 0, 255 );
						break;
					case 2:
						// ki
						dataTuningLR ( &lineTraceCtrl.ki, 1, 0, 255 );
						break;
					case 3:
						// kd
						dataTuningLR ( &lineTraceCtrl.kd, 1, 0, 255 );
						break;
				}
			}
			
			break;
		//------------------------------------------------------------------
		// ゲイン調整(速度)
		//------------------------------------------------------------------
		case HEX_PID_SPEED:
			if (patternDisplay != beforeHEX) 	{
				// 切替時に実行
				ssd1306_printf(Font_6x8,"Speed PID");

				ssd1306_SetCursor(0,18);
				ssd1306_printf(Font_7x10,"kp:");
				ssd1306_SetCursor(0,32);
				ssd1306_printf(Font_7x10,"ki:");
				ssd1306_SetCursor(0,44);
				ssd1306_printf(Font_7x10,"kd:");
				ssd1306_SetCursor(60,30);
				ssd1306_printf(Font_7x10,"pwm:");
			}

			data_select( &trace_test, SW_PUSH );
			// PUSHでトレースON/OFF
			if ( trace_test == 1 ) {
				powerLinesensors(1);
				setTargetSpeed(0.0);
				motorPwmOutSynth( lineTraceCtrl.pwm, veloCtrl.kp, 0, 0);
			} else {
				motorPwmOutSynth( 0, 0, 0, 0);
				powerLinesensors(0);
			}

			// ゲイン表示
			dataTuningUD( &patternGain, 1, 3, 1);
			if (trace_test == 0) {
				ssd1306_SetCursor(21,18);
				if (patternGain == 1) 	ssd1306_printfB(Font_7x10,"%3d",veloCtrl.kp);
				else 					ssd1306_printf(Font_7x10,"%3d",veloCtrl.kp);
				ssd1306_SetCursor(21,32);
				if (patternGain == 2)	ssd1306_printfB(Font_7x10,"%3d",veloCtrl.ki);
				else 					ssd1306_printf(Font_7x10,"%3d",veloCtrl.ki);
				ssd1306_SetCursor(21,44);
				if (patternGain == 3) 	ssd1306_printfB(Font_7x10,"%3d",veloCtrl.kd);
				else 					ssd1306_printf(Font_7x10,"%3d",veloCtrl.kd);
				
				
				// 制御量表示
				ssd1306_SetCursor(88,30);
				ssd1306_printf(Font_7x10,"%4d",veloCtrl.pwm);
			
				switch( patternGain ) {
					case 1:
						// kp
						dataTuningLR ( &veloCtrl.kp, 1, 0, 255 );
						break;
					case 2:
						// ki
						dataTuningLR ( &veloCtrl.ki, 1, 0, 255 );
						break;
					case 3:
						// kd
						dataTuningLR ( &veloCtrl.kd, 1, 0, 255 );
						break;
				}
			}
		break;
		//------------------------------------------------------------------
		// ゲイン調整(角速度)
		//------------------------------------------------------------------
		case HEX_PID_ANGULAR:
			if (patternDisplay != beforeHEX) 	{
				// 切替時に実行
				ssd1306_printf(Font_6x8,"YawRate PID");

				ssd1306_SetCursor(0,18);
				ssd1306_printf(Font_7x10,"kp:");
				ssd1306_SetCursor(0,32);
				ssd1306_printf(Font_7x10,"ki:");
				ssd1306_SetCursor(0,44);
				ssd1306_printf(Font_7x10,"kd:");
				ssd1306_SetCursor(60,30);
				ssd1306_printf(Font_7x10,"pwm:");

				setTargetAngularVelocity(0);
				setTargetSpeed(0);
			}

			data_select( &trace_test, SW_PUSH );
			// PUSHでトレースON/OFF
			if ( trace_test == 1 ) {
				motorPwmOutSynth( 0, veloCtrl.pwm, yawRateCtrl.pwm, 0 );
			} else {
				motorPwmOutSynth( 0, 0, 0, 0 );
			}

			// ゲイン表示
			dataTuningUD( &patternGain, 1, 3, 1);		
			if (trace_test == 0) {
				ssd1306_SetCursor(21,18);
				if (patternGain == 1) 	ssd1306_printfB(Font_7x10,"%3d",yawRateCtrl.kp);
				else 					ssd1306_printf(Font_7x10,"%3d",yawRateCtrl.kp);
				ssd1306_SetCursor(21,32);
				if (patternGain == 2)	ssd1306_printfB(Font_7x10,"%3d",yawRateCtrl.ki);
				else 					ssd1306_printf(Font_7x10,"%3d",yawRateCtrl.ki);
				ssd1306_SetCursor(21,44);
				if (patternGain == 3) 	ssd1306_printfB(Font_7x10,"%3d",yawRateCtrl.kd);
				else 					ssd1306_printf(Font_7x10,"%3d",yawRateCtrl.kd);
				
				// 制御量表示
				ssd1306_SetCursor(88,30);
				ssd1306_printf(Font_7x10,"%4d",yawRateCtrl.pwm);
			
				switch( patternGain ) {
					case 1:
						// kp
						dataTuningLR ( &yawRateCtrl.kp, 1, 0, 255 );
						break;
					case 2:
						// ki
						dataTuningLR ( &yawRateCtrl.ki, 1, 0, 255 );
						break;
					case 3:
						// kd
						dataTuningLR ( &yawRateCtrl.kd, 1, 0, 255 );
						break;
				}
			}
			break;

		//------------------------------------------------------------------
		// ゲイン調整(角度)
		//------------------------------------------------------------------
		case HEX_PID_ANGLE:
			if (patternDisplay != beforeHEX) 	{
				// 切替時に実行
				ssd1306_printf(Font_6x8,"Yaw PID");

				ssd1306_SetCursor(0,18);
				ssd1306_printf(Font_7x10,"kp:");
				ssd1306_SetCursor(0,32);
				ssd1306_printf(Font_7x10,"ki:");
				ssd1306_SetCursor(0,44);
				ssd1306_printf(Font_7x10,"kd:");
				ssd1306_SetCursor(60,30);
				ssd1306_printf(Font_7x10,"pwm:");

				setTargetDist(50);
				setTargetSpeed(0.3);
			}

			data_select( &trace_test, SW_PUSH );
			// PUSHでトレースON/OFF
			// if ( trace_test == 1 ) {
			// 	motorPwmOutSynth( 0, veloCtrl.pwm, distCtrl.pwm, 0 );
			// } else {
			// 	motorPwmOutSynth( 0, 0, 0, 0 );
			// }

			// ゲイン表示
			dataTuningUD( &patternGain, 1, 3, 1);		
			if (trace_test == 0) {
				ssd1306_SetCursor(21,18);
				if (patternGain == 1) 	ssd1306_printfB(Font_7x10,"%3d",yawCtrl.kp);
				else 					ssd1306_printf(Font_7x10,"%3d",yawCtrl.kp);
				ssd1306_SetCursor(21,32);
				if (patternGain == 2)	ssd1306_printfB(Font_7x10,"%3d",yawCtrl.ki);
				else 					ssd1306_printf(Font_7x10,"%3d",yawCtrl.ki);
				ssd1306_SetCursor(21,44);
				if (patternGain == 3) 	ssd1306_printfB(Font_7x10,"%3d",yawCtrl.kd);
				else 					ssd1306_printf(Font_7x10,"%3d",yawCtrl.kd);
				
				// 制御量表示
				ssd1306_SetCursor(88,30);
				ssd1306_printf(Font_7x10,"%4d",yawCtrl.pwm);
			
				switch( patternGain ) {
					case 1:
						// kp
						dataTuningLR ( &yawCtrl.kp, 1, 0, 255 );
						break;
					case 2:
						// ki
						dataTuningLR ( &yawCtrl.ki, 1, 0, 255 );
						break;
					case 3:
						// kd
						dataTuningLR ( &yawCtrl.kd, 1, 0, 255 );
						break;
				}
			}
			break;	
		//------------------------------------------------------------------
		// ゲイン調整(距離)
		//------------------------------------------------------------------
		case HEX_PID_DIST:
			if (patternDisplay != beforeHEX) 	{
				// 切替時に実行
				ssd1306_printf(Font_6x8,"Dist PID");

				ssd1306_SetCursor(0,18);
				ssd1306_printf(Font_7x10,"kp:");
				ssd1306_SetCursor(0,32);
				ssd1306_printf(Font_7x10,"ki:");
				ssd1306_SetCursor(0,44);
				ssd1306_printf(Font_7x10,"kd:");
				ssd1306_SetCursor(60,30);
				ssd1306_printf(Font_7x10,"pwm:");

				setTargetDist(50.0);
				setTargetSpeed(0.3);
			}

			// ゲイン表示
			dataTuningUD( &patternGain, 1, 3, 1);		
			if (trace_test == 0) {
				ssd1306_SetCursor(21,18);
				if (patternGain == 1) 	ssd1306_printfB(Font_7x10,"%3d",distCtrl.kp);
				else 					ssd1306_printf(Font_7x10,"%3d",distCtrl.kp);
				ssd1306_SetCursor(21,32);
				if (patternGain == 2)	ssd1306_printfB(Font_7x10,"%3d",distCtrl.ki);
				else 					ssd1306_printf(Font_7x10,"%3d",distCtrl.ki);
				ssd1306_SetCursor(21,44);
				if (patternGain == 3) 	ssd1306_printfB(Font_7x10,"%3d",distCtrl.kd);
				else 					ssd1306_printf(Font_7x10,"%3d",distCtrl.kd);
				
				// 制御量表示
				ssd1306_SetCursor(88,30);
				ssd1306_printf(Font_7x10,"%4d",distCtrl.pwm);
			
				switch( patternGain ) {
					case 1:
						// kp
						dataTuningLR ( &distCtrl.kp, 1, 0, 255 );
						break;
					case 2:
						// ki
						dataTuningLR ( &distCtrl.ki, 1, 0, 255 );
						break;
					case 3:
						// kd
						dataTuningLR ( &distCtrl.kd, 1, 0, 255 );
						break;
				}
			}
			break;


	default:
		ssd1306_SetCursor(30,5);
		ssd1306_printf(Font_6x8,"None      ");
		ssd1306_FillRectangle(0,16,127,63, Black);

		break;
	} // switch

	// 前回値更新
	beforeHEX = patternDisplay;
	beforeBATLV = batteryLevel;

	if (!trace_test && !calibratIMU) {
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
	int16_t sign;

	if(max - min > 0) {
		sign = 1;
	} else {
		sign = -1;
	}

	if ( cntSwitchUD >= 50 ) {
		if ( swValTact == SW_UP || swValTact == SW_DOWN ) {
			cntSwitchUDLong++; // 長押し時間計測
			if ( swValTact == SW_UP  ) {
				// インクリメント
				if ( cntSwitchUDLong >= 20 ) {	// 長押し処理
					*data += sign * add;
				} else if (pushUD == 0) {	// 1回押し処理
					pushUD = 1;
					*data += sign * add;
				}
			} else if ( swValTact == SW_DOWN  ) {
				// デクリメント
				if ( cntSwitchUDLong >= 20 ) {	// 長押し処理
					*data -= sign * add;
				} else if (pushUD == 0) {	// 1回押し処理
					pushUD = 1;
					*data -= sign * add;
				}
			}
		} else {
			pushUD = 0;
			cntSwitchUDLong = 0;
		}
		cntSwitchUD = 0;

		if (sign > 0) {
			if ( *data > max) {
				*data = min;
			} else if ( *data < min ) {
				*data = max;
			}
		} else {
			if ( *data > min) {
				*data = max;
			} else if ( *data < max ) {
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
void dataTuningLR ( int16_t *data, int16_t add, int16_t min, int16_t max) {
	int16_t sign;

	if(max - min > 0) {
		sign = 1;
	} else {
		sign = -1;
	}

	if ( cntSwitchLR >= 50 ) {
		if ( swValTact == SW_LEFT || swValTact == SW_RIGHT ) {
			cntSwitchLRLong++; // 長押し時間計測
			if ( swValTact == SW_RIGHT  ) {
				// インクリメント
				if ( cntSwitchLRLong >= 20 ) {	// 長押し処理
					*data += sign * add;
				} else if (pushLR == 0) {	// 1回押し処理
					pushLR = 1;
					*data += sign * add;
				}
			} else if ( swValTact == SW_LEFT  ) {
				// デクリメント
				if ( cntSwitchLRLong >= 20 ) {	// 長押し処理
					*data -= sign * add;
				} else if (pushLR == 0) {	// 1回押し処理
					pushLR = 1;
					*data -= sign * add;
				}
			}
		} else {
			pushLR = 0;
			cntSwitchLRLong = 0;
		}
		cntSwitchLR = 0;

		if (sign > 0) {
			if ( *data > max) {
				*data = min;
			} else if ( *data < min ) {
				*data = max;
			}
		} else {
			if ( *data > min) {
				*data = max;
			} else if ( *data < max ) {
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
void dataTuningUDF ( float *data, float add, float min, float max) {
	int16_t sign;

	if(max - min > 0) {
		sign = 1;
	} else {
		sign = -1;
	}

	if ( cntSwitchUD >= 50 ) {
		if ( swValTact == SW_UP || swValTact == SW_DOWN ) {
			cntSwitchUDLong++; // 長押し時間計測
			if ( swValTact == SW_UP  ) {
				// インクリメント
				if ( cntSwitchUDLong >= 20 ) {	// 長押し処理
					*data += sign * add;
				} else if (pushUD == 0) {	// 1回押し処理
					pushUD = 1;
					*data += sign * add;
				}
			} else if ( swValTact == SW_DOWN  ) {
				// デクリメント
				if ( cntSwitchUDLong >= 20 ) {	// 長押し処理
					*data -= sign * add;
				} else if (pushUD == 0) {	// 1回押し処理
					pushUD = 1;
					*data -= sign * add;
				}
			}
		} else {
			pushUD = 0;
			cntSwitchUDLong = 0;
		}
		cntSwitchUD = 0;

		if (sign > 0) {
			if ( *data > max) {
				*data = min;
			} else if ( *data < min ) {
				*data = max;
			}
		} else {
			if ( *data > min) {
				*data = max;
			} else if ( *data < max ) {
				*data = min;
			}
		}
	}
}
///////////////////////////////////////////////////////////////////////////////////////
// モジュール名 caribrateSensors
// 処理概要     機体を超信地旋回させてラインセンサをキャリブレーションする
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////////////////
void caribrateSensors(void) {
	static uint8_t mode = 0;

	switch (patternCalibration) {
		case 1:
			setTargetSpeed(0);

			// スイッチ入力待ち
			if (swValMainTact == SW_TACT_L || swValMainTact == SW_TACT_R) {
				if (swValMainTact == SW_TACT_L) {
					mode = START_SERACH;
				} else if (swValMainTact == SW_TACT_R) {
					mode = START_OPTIMAL;
				}
				veloCtrl.Int = 0;	// I成分リセット
				if(lSensorOffset[0] > 0) {
					// キャリブレーション実施済み
					start = 1;
				} else {
					// キャリブレーション未実施
					cntSetup1 = 0;
					enc1 = 0;
					powerLinesensors(1);	// 先に点灯させて安定させる
					patternCalibration = 2;
				}
			}
			break;

		case 2:
			// 開始準備
			if (cntSetup1 > 1000) {
				veloCtrl.Int = 0;			// I成分リセット
				BMI088val.angle.z = 0.0;	// 角度リセット
				yawRateCtrl.Int = 0.0;		// I成分リセット
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
			if (BMI088val.angle.z < -35.0) {
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
			if (BMI088val.angle.z > 35.0) {
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
			if (lSensor[5] < 1000) {
				modeCalLinesensors = 0;
				patternCalibration = 8;
			}
			break;

		case 8:
			// 停止
			motorPwmOutSynth( 0, veloCtrl.pwm, 0, 0);
			if (abs(encCurrentN) == 0) {
				powerLinesensors(0);	// ラインセンサ消灯
				if(mode == START_OPTIMAL) {
					// 距離基準解析
					numPPADarry = readLogDistance(analizedNumber);
					if (numPPADarry > 0) {
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
void wheelClick(void) {
	static uint8_t cnt=0;

	switch(patternClick) {
		case 1:
			if(clickStart == 1) {
				patternClick = 2;
			}
			break;

		case 2:
			motorPwmOut(200,0);
			cnt++;
			if(cnt >= 3) {
				cnt = 0;
				patternClick = 3;
			}
			break;

		case 3:
			motorPwmOut(-200,0);
			cnt++;
			if(cnt >= 3) {
				cnt = 0;
				patternClick = 4;
			}
			break;

		case 4:
			motorPwmOut(0,0);
			clickStart = 0;
			patternClick = 1;
			break;
	}
}