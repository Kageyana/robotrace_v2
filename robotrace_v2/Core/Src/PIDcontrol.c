//====================================//
// インクルード
//====================================//
#include "PIDcontrol.h"
#include "fatfs.h"
//====================================//
// グローバル変数の宣言
//====================================//
pidParam 	lineTraceCtrl = { "line", KP1, KI1, KD1, 0, 0};
pidParam 	veloCtrl = { "speed", KP2, KI2, KD2, 0, 0};
pidParam 	yawRateCtrl = { "yawRate", KP3, KI3, KD3, 0, 0};
pidParam 	yawCtrl = { "yaw", KP4, KI4, KD4, 0, 0};
pidParam 	distCtrl = { "dist", KP5, KI5, KD5, 0, 0};

uint8_t		targetSpeed;			// 目標速度
float 		targetAngle;			// 目標角速度
float   	targetAngularVelocity;	// 目標角度
int16_t		targetDist;				// 目標X座標

///////////////////////////////////////////////////////////////////////////
// モジュール名 setTargetSpeed
// 処理概要     目標速度の設定
// 引数         目標速度の整数倍値
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void setTargetSpeed (float speed) {
	targetSpeed = (int16_t)(speed*PALSE_MILLIMETER);
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 setTargetAngularVelocity
// 処理概要     目標角速度の設定
// 引数         目標角速度[deg/s]
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void setTargetAngularVelocity (float angularVelocity) {
	targetAngularVelocity = angularVelocity;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 setTargetAngle
// 処理概要     目標角度の設定
// 引数         目標角度[deg]
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void setTargetAngle (float angle) {
	targetAngle = angle;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 setTargetX
// 処理概要     目標x座標の設定
// 引数         目標x座標[mm]
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void setTargetDist (float dist) {
	targetDist = (int16_t)(dist*PALSE_MILLIMETER);;
	encPID = 0;
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motorControlTrace
// 処理概要     ライントレース時の制御量の計算
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void motorControlTrace( void ) {
	int32_t 		iP, iI, iD, iRet, Dev, Dif, senL, senR;
	static int32_t 	traceBefore;
	
	//サーボモータ用PWM値計算
	if (lSensorOffset[0] > 0)	{
		senL = (lSensorCari[4]) + (lSensorCari[3]*0.7) + (lSensorCari[2]*0.5) + (lSensorCari[1]*0.3) + (lSensorCari[0]*0.2);
		senR = (lSensorCari[5]) + (lSensorCari[6]*0.7) + (lSensorCari[7]*0.5) + (lSensorCari[8]*0.3) + (lSensorCari[9]*0.2);
	} else {
		// senL = (lSensor[4]) + (lSensor[3]*0.8) + (lSensor[2]*0.7) + (lSensor[1]*0.5) + (lSensor[0]*0.3);
		// senR = (lSensor[5]) + (lSensor[6]*0.8) + (lSensor[7]*0.7) + (lSensor[8]*0.5) + (lSensor[9]*0.3);
		senL = (lSensor[5]);
		senR = (lSensor[6]);
	}
	Dev = senL - senR;
		
	// I成分積算
	lineTraceCtrl.Int += (float)Dev * 0.001;
	if ( lineTraceCtrl.Int > 10000.0 ) lineTraceCtrl.Int = 10000.0;		// I成分リミット
	else if ( lineTraceCtrl.Int < -10000.0 ) lineTraceCtrl.Int = -10000.0;
	Dif = ( Dev - traceBefore ) * 1;	// dゲイン1/1000倍

	iP = lineTraceCtrl.kp * Dev;	// 比例
	iI = lineTraceCtrl.ki * lineTraceCtrl.Int;	// 積分
	iD = lineTraceCtrl.kd * Dif;	// 微分
	iRet = iP + iI + iD;
	iRet = iRet >> 8;				// PWMを0～1000近傍に収める

	// PWMの上限の設定
	if ( iRet >  900 ) iRet = 900;
	if ( iRet <  -900 ) iRet = -900;
	
	lineTraceCtrl.pwm = iRet;
	traceBefore = Dev;				// 次回はこの値が1ms前の値となる
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motorControlSpeed
// 処理概要     モーターの制御量の計算
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void motorControlSpeed( void ) {
	int32_t 		iP, iI, iD, iRet, Dev, Dif;
	static int16_t 	targetSpeedBefore, encoderBefore;
	
	// 駆動モーター用PWM値計算
	Dev = (int16_t)targetSpeed - encCurrentN;	// 偏差
	// 目標値を変更したらI成分リセット
	if ( targetSpeed != targetSpeedBefore ) veloCtrl.Int = 0;
	
	veloCtrl.Int += (float)Dev * 0.001;	// 時間積分
	Dif = Dev - encoderBefore;		// 微分　dゲイン1/1000倍
	
	iP = veloCtrl.kp * Dev;		// 比例
	iI = veloCtrl.ki * veloCtrl.Int;		// 積分
	iD = veloCtrl.kd * Dif;		// 微分
	iRet = iP + iI + iD;
	iRet = iRet >> 1;
	
	// PWMの上限の設定
	if ( iRet >  900 ) iRet = 900;
	if ( iRet <  -900 ) iRet = -900;
	
	veloCtrl.pwm = iRet;
	encoderBefore = Dev;				// 次回はこの値が1ms前の値となる
	targetSpeedBefore = targetSpeed;	// 前回の目標値を記録
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motorControlYaw
// 処理概要     角速度制御時の制御量の計算
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void motorControlYawRate(void) {
	float 			iP, iI, iD, Dev, Dif;
	static float	angularVelocityBefore;
	static float 	targetAngularVelocityBefore;
	int32_t 		iRet;
	
	Dev = (targetAngularVelocity - BMI088val.gyro.z) * 1;	// 目標値-現在値
	// I成分積算
	yawRateCtrl.Int += Dev * 0.005;
	// 目標値を変更したらI成分リセット
	if ( targetAngularVelocity != targetAngularVelocityBefore ) yawRateCtrl.Int = 0;
	Dif = ( Dev - angularVelocityBefore ) * 2;	// dゲイン1/500倍

	iP = yawRateCtrl.kp * Dev;	// 比例
	iI = yawRateCtrl.ki * yawRateCtrl.Int;	// 積分
	iD = yawRateCtrl.kd * Dif;	// 微分
	iRet = (int32_t)iP + iI + iD;
	iRet = iRet >> 4;				// PWMを0～1000近傍に収める

	// PWMの上限の設定
	if ( iRet >  900 ) iRet =  900;
	if ( iRet <  -900 ) iRet = -900;
	
	yawRateCtrl.pwm = iRet;
	angularVelocityBefore = Dev;	// 次回はこの値が1ms前の値となる
	targetAngularVelocityBefore = targetAngularVelocity;	// 前回の目標値を記録
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motorControlYaw
// 処理概要     角速度制御時の制御量の計算
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void motorControlYaw(void) {
	float 			iP, iI, iD, Dev, Dif;
	static float 	angleBefore;
	static float 	targetAngleBefore;
	int32_t 		iRet;
	
	Dev = (targetAngle - BMI088val.angle.z) * 20;	// 目標値-現在値
	// I成分積算
	yawCtrl.Int += Dev * 0.005;
	// 目標値を変更したらI成分リセット
	// if ( targetAngle != targetAngleBefore ) yawCtrl.Int = 0;
	Dif = ( Dev - angleBefore ) * 1;	// dゲイン1/1000倍

	iP = yawCtrl.kp * Dev;	// 比例
	iI = yawCtrl.ki * yawCtrl.Int;	// 積分
	iD = yawCtrl.kd * Dif;	// 微分
	iRet = (int32_t)iP + iI + iD;
	iRet = iRet >> 2;				// PWMを0～1000近傍に収める

	// PWMの上限の設定
	if ( iRet >  900 ) iRet =  900;
	if ( iRet <  -900 ) iRet = -900;
	
	yawCtrl.pwm = iRet;
	angleBefore = Dev;					// 次回はこの値が1ms前の値となる
	targetAngleBefore = targetAngle;	// 前回の目標値を記録
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 motorControldist
// 処理概要     距離制御時の制御量の計算
// 引数         なし
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void motorControldist(void) {
	int32_t 		iP, iI, iD, Dev, Dif, iRet;
	static int32_t 	distBefore, targetDistBefore;
	
	Dev = (targetDist - encPID) * 1;	// 目標値-現在値
	// I成分積算
	distCtrl.Int += Dev * 0.001;
	// 目標値を変更したらI成分リセット
	// if ( targetDist != targetDistBefore ) distCtrl.Int = 0;
	Dif = ( Dev - distBefore ) * 1;	// dゲイン1/1000倍

	iP = distCtrl.kp * Dev;				// 比例
	iI = distCtrl.ki * distCtrl.Int;	// 積分
	iD = distCtrl.kd * Dif;				// 微分
	iRet = (int32_t)iP + iI + iD;
	iRet = iRet >> 1;				// PWMを0～1000近傍に収める

	// PWMの上限の設定
	if ( iRet >  900 ) iRet =  900;
	if ( iRet <  -900 ) iRet = -900;
	
	distCtrl.pwm = iRet;
	distBefore = Dev;				// 次回はこの値が1ms前の値となる
	targetDistBefore = targetDist;	// 前回の目標値を記録
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 writePIDparameters
// 処理概要     PIDゲインをSDカードに記録する
// 引数         pid:pidParam型の変数
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void writePIDparameters(pidParam *pid) {
	FIL         fil;
    FRESULT     fresult;
	uint8_t     fileName[20] = PATH_SETTING;
    int16_t     ret=0;

	// ファイル読み込み
	strcat(fileName,pid->name); // ファイル名追加
	strcat(fileName,".txt");   // 拡張子追加
    fresult = f_open(&fil, fileName, FA_OPEN_ALWAYS | FA_WRITE);  	// ファイルを開く

	if(fresult == FR_OK) {
		f_printf(&fil, "%03d,%03d,%03d",pid->kp, pid->ki, pid->kd);
	}

	f_close(&fil);
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 readPIDparameters
// 処理概要     PIDゲインをSDカードから読み取る
// 引数         pid:pidParam型の変数
// 戻り値       なし
///////////////////////////////////////////////////////////////////////////
void readPIDparameters(pidParam *pid) {
	FIL         fil;
    FRESULT     fresult;
	uint8_t     fileName[20] = PATH_SETTING;
	TCHAR     	gain[20];
    int16_t     ret=0;

	// ファイル読み込み
	strcat(fileName,pid->name); // ファイル名追加
	strcat(fileName,".txt");   // 拡張子追加
    fresult = f_open(&fil, fileName, FA_OPEN_EXISTING | FA_READ);  // ファイルを開く

	if(fresult == FR_OK) {
		f_gets(gain,sizeof(gain),&fil);							// 文字列取得
		sscanf(gain,"%d,%d,%d",&pid->kp,&pid->ki,&pid->kd);		// 文字列→数値
	}

	f_close(&fil);
}