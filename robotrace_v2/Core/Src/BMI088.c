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
    if(BMI088ReadByteG(REG_GYRO_CHIP_ID) == 0xf) {
        // コンフィグ設定

        // 加速
        BMI088ReadByteA(REG_GYRO_CHIP_ID); // 加速度センサ起動(SPIダミーリード)
		// BMI088WriteByteA(REG_ACC_SOFTRESET,0xB6);	// ソフトウェアリセット
		HAL_Delay(1);
		BMI088WriteByteA(REG_ACC_PWR_CTRL,0x04);	// 加速度センサ計測開始
		HAL_Delay(450);
		BMI088WriteByteA(REG_ACC_RANGE,0x01);	// レンジを6gに設定
		BMI088WriteByteA(REG_ACC_CONF,0xA9);	// ODRを200Hzに設定

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
	BMI088ReadAxisDataG(REG_RATE_X,rawData,6);
	// LSBとMSBを結合
	gyroVal[0] = (rawData[1] << 8) | rawData[0];
	gyroVal[1] = (rawData[3] << 8) | rawData[2];
	gyroVal[2] = (rawData[5] << 8) | rawData[4];

	BMI088val.gyro.x = (float)gyroVal[0] / GYROLSB;
	BMI088val.gyro.y = (float)gyroVal[1] / GYROLSB;
	BMI088val.gyro.z = (float)gyroVal[2] / GYROLSB;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 BMI088getTemp
// 処理概要     角速度の取得
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void BMI088getTemp(void) {
    uint8_t rawData[8];
	int16_t accelVal[3];

	// 角速度の生データを取得
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
// モジュール名 calcDegrees
// 処理概要     角度の計算
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void calcDegrees(void) {
    BMI088val.angle.x += BMI088val.gyro.x * DEFF_TIME * COEFF_DPD;
    BMI088val.angle.y += BMI088val.gyro.y * DEFF_TIME * COEFF_DPD;
    BMI088val.angle.z += BMI088val.gyro.z * DEFF_TIME * COEFF_DPD;   
}
