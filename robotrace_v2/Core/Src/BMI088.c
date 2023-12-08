//====================================//
// インクルード
//====================================//
#include "BMI088.h"
//====================================//
// グローバル変数の宣
//====================================//
axis accele = { 0.0F, 0.0F, 0.0F};
axis gyro = { 0.0F, 0.0F, 0.0F};
axis angle = { 0.0F, 0.0F, 0.0F};
IMUval 	BMI088val;

int16_t	angleOffset[3] = {0,0,0};
bool  	calibratIMU = false;
bool    IMUstate = IMU_STOP;
/////////////////////////////////////////////////////////////////////
// モジュール名 BMI088ReadByteG
// 処理概要     指定レジスタの値を読み出す(ジャイロセンサ部)
// 引数         reg: レジスタのアドレス
// 戻り値       読み出した値
////////////////////////////////////////////////////////////////////
uint8_t BMI088ReadByteG( uint8_t reg ) {
	uint8_t txData,rxData;
	uint8_t txNum=1,rxNum=1;
	
	txData = reg | 0x80;
	CSB2_RESET;
	BMI088TRANSFER;
    BMI088RECEIVE;
	CSB2_SET;
	
	return rxData;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 BMI088WriteByteG
// 処理概要     指定レジスタに値を書き込む(ジャイロセンサ部)
// 引数         reg: レジスタのアドレス val: 書き込む値
// 戻り値       なし
////////////////////////////////////////////////////////////////////
void BMI088WriteByteG( uint8_t reg, uint8_t val )  {
	uint8_t txData[2]={reg & 0x7F,val};
	uint8_t txNum=2;
	
	CSB2_RESET;
	BMI088TRANSFER;
	CSB2_SET;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 BMI088ReadAxisDataG
// 処理概要     指定レジスタの読み出し(ジャイロセンサ部)
// 引数         reg:レジスタアドレス
// 戻り値       読み出したデータ
/////////////////////////////////////////////////////////////////////
void BMI088ReadAxisDataG(uint8_t reg, uint8_t *rxData, uint8_t rxNum ) {
    uint8_t txData;
	uint8_t txNum=1;

	txData = reg | 0x80;

    CSB2_RESET;
	BMI088TRANSFER;
    BMI088RECEIVES;
	CSB2_SET;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 BMI088ReadByteA
// 処理概要     指定レジスタの値を読み出す(加速度センサ部)
// 引数         reg: レジスタのアドレス
// 戻り値       読み出した値
////////////////////////////////////////////////////////////////////
uint8_t BMI088ReadByteA( uint8_t reg ) {
	uint8_t txData,rxData;
	uint8_t txNum=1,rxNum=1;
	
	txData = reg | 0x80;
	CSB1_RESET;
	BMI088TRANSFER;
    BMI088RECEIVE;
	CSB1_SET;
	
	return rxData;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 BMI088WriteByteA
// 処理概要     指定レジスタに値を書き込む(加速度センサ部)
// 引数         reg: レジスタのアドレス val: 書き込む値
// 戻り値       なし
////////////////////////////////////////////////////////////////////
void BMI088WriteByteA( uint8_t reg, uint8_t val )  {
	uint8_t txData[2]={reg & 0x7F,val};
	uint8_t txNum=2;
	
	CSB1_RESET;
	BMI088TRANSFER;
	CSB1_SET;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 BMI088ReadAxisDataA
// 処理概要     指定レジスタの読み出し(加速度センサ部)
// 引数         reg:レジスタアドレス
// 戻り値       読み出したデータ
/////////////////////////////////////////////////////////////////////
void BMI088ReadAxisDataA(uint8_t reg, uint8_t *rxData, uint8_t rxNum ) {
    uint8_t txData;
	uint8_t txNum=1;

	txData = reg | 0x80;

    CSB1_RESET;
	BMI088TRANSFER;
    BMI088RECEIVES;
	CSB1_SET;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 initBMI088
// 処理概要     初期設定パラメータの書き込み
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
bool initBMI088(void) {
	uint8_t rawData[8];

    if(BMI088ReadByteG(REG_GYRO_CHIP_ID) == 0xf) {
        // コンフィグ設定

        // 加速
		BMI088WriteByteA(REG_ACC_SOFTRESET,0xB6);	// ソフトウェアリセット
		HAL_Delay(5);
		
        BMI088ReadByteA(REG_GYRO_CHIP_ID); 		// 加速度センサSPIモードに切り替え(SPIダミーリード)
		HAL_Delay(5);
		BMI088WriteByteA(REG_ACC_RANGE,0x01);	// レンジを6gに設定
		BMI088WriteByteA(REG_ACC_CONF,0xA9);	// ODRを200Hzに設定
		BMI088ReadAxisDataA(REG_ACC_CHIP_ID,rawData,3);
		BMI088val.id = rawData[1];
		BMI088WriteByteA(REG_ACC_PWR_CTRL,0x04);	// 加速度センサ計測開始
		HAL_Delay(5);
		

        // ジャイロ
		BMI088WriteByteG(REG_GYRO_SOFTRESET,0xB6);	// ソフトウェアリセット
        BMI088WriteByteG(REG_GYRO_BANDWISTH,0x83);	// ODRを200Hzに設定
        // モード変更
 
        return true;
    } else {
        return false;
    }
}
/////////////////////////////////////////////////////////////////////
// モジュール名 BMI088getGyro
// 処理概要     角速度の取得
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void BMI088getGyro(void) {
    uint8_t rawData[6];
	int16_t gyroVal[3];

	// 角速度の生データを取得
	BMI088ReadAxisDataG(REG_RATE_Z_LSB,rawData,2);
	// LSBとMSBを結合
	// gyroVal[0] = ((rawData[1] << 8) | rawData[0]) - angleOffset[0];
	// gyroVal[1] = ((rawData[3] << 8) | rawData[2]) - angleOffset[1];
	gyroVal[2] = ((rawData[1] << 8) | rawData[0]) - angleOffset[2];

	// BMI088val.gyro.x = (float)gyroVal[0] / GYROLSB * COEFF_DPD;
	// BMI088val.gyro.y = (float)gyroVal[1] / GYROLSB * COEFF_DPD;
	BMI088val.gyro.z = (float)gyroVal[2] / GYROLSB * COEFF_DPD;
	
}
/////////////////////////////////////////////////////////////////////
// モジュール名 BMI088getAccele
// 処理概要     加速度の取得
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void BMI088getAccele(void) {
    uint8_t rawData[8];
	int16_t accelVal[3];

	// 加速度の生データを取得
	BMI088ReadAxisDataA(REG_ACC_X_LSB,rawData,6);
	// LSBとMSBを結合
	accelVal[0] = (rawData[1] << 8) | rawData[0];
	accelVal[1] = (rawData[3] << 8) | rawData[2];
	accelVal[2] = (rawData[5] << 8) | rawData[4];

	BMI088val.accele.x = (float)accelVal[0] / ACCELELSB;
	BMI088val.accele.y = (float)accelVal[1] / ACCELELSB;
	BMI088val.accele.z = (float)accelVal[2] / ACCELELSB;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 BMI088getTemp
// 処理概要     温度の取得
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void BMI088getTemp(void) {
    uint8_t rawData[3];
	uint16_t tempValu;
	int16_t tempVal;

	// 温度の生データを取得
	BMI088ReadAxisDataA(REG_TEMP_MSB,rawData,2);
	// LSBとMSBを結合
	tempValu = (uint16_t)((rawData[0] << 3) | (rawData[1] >> 5));
	if(tempValu > 1023) {
		tempVal = ~tempValu + 0x8000;
	} else {
		tempVal = tempValu;
	}

	BMI088val.temp = ((float)tempVal * 0.125F) + 23.0F;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 calcDegrees
// 処理概要     角度の計算
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void calcDegrees(void) {
    BMI088val.angle.x += BMI088val.gyro.x * DEFF_TIME;
    BMI088val.angle.y += BMI088val.gyro.y * DEFF_TIME;
    BMI088val.angle.z += BMI088val.gyro.z * DEFF_TIME;   
}
/////////////////////////////////////////////////////////////////////
// モジュール名 cariblationIMU
// 処理概要     角速度キャリブレーション
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void calibrationIMU (void) {
	static int32_t angleInt[3];
	static uint16_t i = 0;
	uint8_t rawData[6];
	int16_t gyroVal[3];

	
	if(i<(uint32_t)(5.0/DEFF_TIME)) {
		// 角速度の生データを取得
		BMI088ReadAxisDataG(REG_RATE_X_LSB,rawData,6);
		// LSBとMSBを結合
		gyroVal[0] = (rawData[1] << 8) | rawData[0];
		gyroVal[1] = (rawData[3] << 8) | rawData[2];
		gyroVal[2] = (rawData[5] << 8) | rawData[4];

		angleInt[0] += gyroVal[0];
		angleInt[1] += gyroVal[1];
		angleInt[2] += gyroVal[2];
		i++;
	} else {
		i = 0;
		angleOffset[0] = angleInt[0] / 1000;
		angleOffset[1] = angleInt[1] / 1000;
		angleOffset[2] = angleInt[2] / 1000;
		angleInt[0] = 0;
		angleInt[1] = 0;
		angleInt[2] = 0;
		calibratIMU = false;
	}
	
}