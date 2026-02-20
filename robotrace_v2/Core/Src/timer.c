//====================================//
// インクルード
//====================================//
#include "timer.h"
#include "BMI088.h"
#include "PIDcontrol.h"
#include "control.h"
#include "lineSensor.h"
#include <stdint.h>
#define STRAIGHT_STATE_THRESHOLD_MM	70	// 直線判定の距離閾値[mm]
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

	// IMU処理
	if (initIMU)
	{
		if (!calibratIMU)
		{
			BMI088getGyro();	// 角速度取得
			BMI088getAccele();	// 加速度取得
			calcDegrees();		// コンプリメンタリフィルタで角度算出
			calcVelocity();		// 加速度から速度算出
			// スリップ距離補正（パルス版）
			if (patternTrace >= 12 && patternTrace < 100)
			{
				updateSlipDetection(); // スリップ検出（Δv比率とフラグ更新を1msで実行）
			}
			// motorControlYawRate();	// 角速度制御
			// motorControlYaw();		// 角度制御
		}
		else
		{
			calibrationIMU();
		}
	}

	// PID制御処理
	if(patternTrace < 12 || patternTrace > 100)
	{
		// スタート直後とゴール後は通常のライン制御
		motorControlTrace();
		motorControlSpeed();
	}
	else 
	{
		if(optimalTrace == BOOST_NONE || optimalTrace == BOOST_DISTANCE)
		{
			// ライン追従＋左右独立速度制御
			motorControlTraceOmegaFB();

			int16_t delta = lineTraceOmegaFBCtrl.pwm;
			int16_t targetL = (int16_t)targetSpeed + delta;
			int16_t targetR = (int16_t)targetSpeed - delta;
			// クリップ（例：目標が大きくなりすぎないように。上限は実機で調整）
			const int16_t TARGET_MAX = (int16_t)PALSE_MILLIMETER * 4;	// 4.0m/s
			if (targetL >  TARGET_MAX) targetL =  TARGET_MAX;
			if (targetL < -TARGET_MAX) targetL = -TARGET_MAX;
			if (targetR >  TARGET_MAX) targetR =  TARGET_MAX;
			if (targetR < -TARGET_MAX) targetR = -TARGET_MAX;

			motorControlSpeedLR(targetL, targetR);
		} 
		else if(optimalTrace == BOOST_SHORTCUT)
		{
			motorControldist();
			motorControlSpeed();
			motorControlYaw();		// 角度制御
		}
	}

	if (patternTrace > 10 && patternTrace < 100)
	{
		// 走行中に処理
		// 緊急停止処理
		// if (cntEmcStopAngleX()) emcStop = STOP_ANGLE_X;
		// if (cntEmcStopAngleY()) emcStop = STOP_ANGLE_Y;
		if (cntEmcStopEncStop())
			emcStop = STOP_ENCODER_STOP;
		if (cntEmcStopLineSensorBright() && optimalTrace != BOOST_SHORTCUT)
			emcStop = STOP_LINESENSOR_BRIGHT;
		if (cntEmcStopLineSensorUnbright() && optimalTrace != BOOST_SHORTCUT)
			emcStop = STOP_LINESENSOR_UNBRIGHT;
		if (judgeOverSpeed())
			emcStop = STOP_OVERSPEED;

		changeGain();		// ROCに応じてゲインを切り替える
		checkCrossLine();	// クロスライン確認

		courseMarker = checkMarker();	// マーカー検知
		checkStartGoalMarker();			// ゴールマーカー処理
		processMarkerEvent();	      	// マーカー関連処理を関数に委譲
		if(courseMarker != 0)
			courseMarkerLog = courseMarker; // ログ用にマーカー状態を保存

		// 一定距離ごとに処理
		if (encLog >= encMM(CALCDISTANCE_SHORTCUT))
		{
			// ROC(曲率半径)計算
			rocrun = calcROC(encCurrentN, BMI088val.gyro.z, (float)cntLog / 1000);
			if (rocrun >= 700.0F) // 直線判断
			{
				straightMeter += CALCDISTANCE_SHORTCUT; // 距離積算
			}
			else
			{
				straightMeter = 0;
			}

			if (straightMeter >= STRAIGHT_STATE_THRESHOLD_MM) // 直線が閾値以上のとき
			{
				if (!straightState)
				{
					straightMarkerPending = true;
				}
				straightState = true;
			}
			else
			{
				straightState = false;
			}

			if (modeLOG)
			{
				// CALCDISTANCEごとにログを保存
				writeLogBufferPuts();
				courseMarkerLog = 0; // ログ用マーカー状態をリセット
				straightMarkerPendingLog = 0;
				encLog = 0;	// ログ用エンコーダパルスをリセット
				cntLog = 0;
			}
		}
	}
	else
	{
		// 走行前に処理
		getSwitches();	// スイッチの入力を取得
		countDown();	// カウントダウン処理
		setupCount();	// セットアップ用タイマを更新
		wheelClick();	// セットアップ用ホイールクリック処理
	}

	switch (cnt5)
	{
	case 1:
		// xy座標計算
		// // ショートカット走行の目標値インデックスを更新
		if (optimalTrace == BOOST_SHORTCUT && DistanceOptimal > 0)
		{
			calcXYcie(encPulse5ms, BMI088val.gyro.z, DEFF_TIME); // 累積パルスを使用してxy座標計算
			// 現在座標に対して半径R/先行距離L条件でoptimalIndexを更新し、目標角度を反映
			updateShortCutOptimalIndexByRange();
		}
		encPulse5ms = 0; // 累積値をリセット
		break;
	case 2:
		if (initIMU)
		{
			if (!calibratIMU)
			{
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
		if(!calibrateMotorCurrent)
		{
			getMotorCurrent();
		}
		else
		{
			calibrationMotorCurrent();
		}
		break;
	case 2:
		getVref();
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
	static uint8_t logWriteReqDivider = 0;
	logWriteReqDivider++;
	if (logWriteReqDivider >= 10U) // 100us x 10 = 1ms
	{
		logWriteReqDivider = 0;
		logWriteReq = true;
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 logWriteTask
// 処理概要     SD書き込み要求処理
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void logWriteTask(void)
{
        if (logWriteReq)
        {
                if (sd_is_analysis_active())
                {
                        logWriteReq = false;
                        return;
                }
                writeLogPuts();        // SD?????????????
                logWriteReq = false;
        }
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
