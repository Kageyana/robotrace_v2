//====================================//
// インクルード
//====================================//
#include "timer.h"
//====================================//
// グローバル変数の宣
//====================================//
int32_t cnt5 = 0;
int32_t cnt10 = 0;
int32_t encPulse5ms = 0; // 5ms間のエンコーダパルスを累積
float bootTime;
static volatile bool logWriteReq = false;
/////////////////////////////////////////////////////////////////////
// モジュール名 Interrupt1ms
// 処理概要     タイマー割り込み(1ms)
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void Interrupt1ms(void)
{

	// Interrupt 1ms
	cntRun++;
	cnt5++;
	cnt10++;
	cntLog++;

	// 割り込み時間計測
	uint32_t freqCount = getCycleCounter();
	resetCycleCounter();
	bootTime = getTimeMs(freqCount);

	// Encoder
	getEncoder();
	setEncoderVal();
	encPulse5ms += encCurrentN; // 5ms間のエンコーダパルスを累積

	// PID制御処理
	motorControlTrace();
	motorControlSpeed();
	motorControldist();

	// IMU処理
	if (initIMU)
	{
		if (!calibratIMU)
		{
			BMI088getGyro();	// 角速度取得
			calcDegrees();	// コンプリメンタリフィルタで角度算出
			if (optimalTrace == 0)
				checkCurve();	   // 1次走行 カーブ検出
			motorControlYawRate(); // 角速度制御
			motorControlYaw();	   // 角度制御
		}
		else
		{
			calibrationIMU();
		}
	}

	// 走行中に処理
	if (patternTrace > 10 && patternTrace < 100)
	{
		// 緊急停止処理
		// if (cntEmcStopAngleX()) emcStop = STOP_ANGLE_X;
		// if (cntEmcStopAngleY()) emcStop = STOP_ANGLE_Y;
		if (cntEmcStopEncStop())
			emcStop = STOP_ENCODER_STOP;
		if (cntEmcStopLineSensor())
			emcStop = STOP_LINESENSOR;
		if (judgeOverSpeed())
			emcStop = STOP_OVERSPEED;

		courseMarker = checkMarker(); // マーカー検知
		checkGoalMarker();	      // ゴールマーカー処理
		processMarkerEvent();	      // マーカー関連処理を関数に委譲
	}

	if (patternTrace < 10 || patternTrace > 100)
	{
		// 走行前に処理
		getSwitches(); // スイッチの入力を取得
		countDown();
		setupCount(); // セットアップ用タイマを更新

		wheelClick();
	}
	else
	{
		// 走行中に処理
		if (encLog >= encMM(CALCDISTANCE_SHORTCUT))
		{
			// 一定距離ごとに処理
			static float rocCorrection = 0;
			// ROC(曲率半径)計算
			rocCorrection = calcROC(encCurrentN, BMI088val.gyro.z, (float)cntLog / 1000);
			if (fabs(rocCorrection) >= 700.0F) // 直線判断
			{
				straightMeter += CALCDISTANCE_SHORTCUT; // 距離積算
			}
			else
			{
				straightMeter = 0;
			}

			if (straightMeter >= 100) // 直線が100mm以上のとき
			{
				straightState = true;
			}

			if (modeLOG)
			{
				// CALCDISTANCEごとにログを保存
#ifdef LOG_RUNNING_WRITE
				writeLogBufferPuts(
					LOG_NUM_8BIT,
					LOG_NUM_16BIT,
					LOG_NUM_32BIT,
					LOG_NUM_FLOAT,
					// 8bit
					targetSpeed,
					courseMarker,
					modeCurve,
					// 16bit
					cntRun,
					encCurrentN,
					optimalIndex,
					// motorpwmL,
					// motorpwmR,
					// (int16_t)(motorCurrentL * 10000),
					// (int16_t)(motorCurrentR * 10000),
					lineTraceCtrl.pwm,
					veloCtrl.pwm,
					// ライントレースセンサ値
					lineTraceSenL,
					lineTraceSenR,
					lineTraceDev,
					// 32bit
					encTotalOptimal,
					// float型
					BMI088val.gyro.z
				);
#else
				writeLogBufferPrint(); // バッファにログを保存
#endif
				cntLog = 0;
				encLog = 0;
			}
		}
	}

	switch (cnt5)
	{
	case 1:
		// xy座標計算
		// // ショートカット走行の目標値インデックスを更新
		if (optimalTrace == BOOST_SHORTCUT && DistanceOptimal > 0)
		{
			calcXYcie(encPulse5ms, BMI088val.gyro.z, DEFF_TIME); // 累積パルスを使用してxy座標計算
			// distLen = (float)encCurrentN * PALSE_MILLIMETER * 0.005; // 現在速度から5ms後の移動距離を計算
			optimalIndex = (int32_t)(encTotalOptimal / PALSE_MILLIMETER) / CALCDISTANCE_SHORTCUT; // 50mmごとにショートカット配列を作っているので移動距離[mm]を50mmで割った商がインデクス
			if (optimalIndex + 1 <= numPPADarry)
			{
				optimalIndex++;
			}

			if (targetDist - encPID < 200)
			{
				if (optimalIndex + 1 <= numPPADarry)
				{
						optimalIndex++;
				}
			}

			setShortCutTarget(); // 目標値更新
		}
		encPulse5ms = 0; // 累積値をリセット
		break;
	case 2:
		if (initIMU)
		{
			if (!calibratIMU)
			{
				BMI088getAccele();	// 加速度取得（角度補正に使用）
				BMI088getTemp(); 	// 温度取得
			}
		}
		break;
	case 3:
		break;
	case 5:
		cnt5 = 0;
		break;
	}

	switch (cnt10)
	{
	case 1:
		getMotorCurrent();
		break;
	case 2:
		break;
	case 9:
		break;
	case 10:
		cnt10 = 0;
		break;
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 Interrupt100us
// 処理概要     タイマー割り込み(0.1ms)
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void Interrupt100us(void)
{
#ifdef LOG_RUNNING_WRITE
        logWriteReq = true;
#endif
}
/////////////////////////////////////////////////////////////////////
// モジュール名 logWriteTask
// 処理概要     SD書き込み要求処理
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void logWriteTask(void)
{
#ifdef LOG_RUNNING_WRITE
        if (logWriteReq)
        {
                writeLogPuts();        // 割り込み外でSD書き込みを実行する
                logWriteReq = false;
        }
#endif
}
/////////////////////////////////////////////////////////////////////
// モジュール名 Interrupt300ns
// 処理概要     タイマー割り込み(300ns)
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void Interrupt300ns(void)
{
	// Interrupt 300ns
	// sendColorData();
}
