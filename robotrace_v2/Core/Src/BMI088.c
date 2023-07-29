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

int16_t gyData[3];
/////////////////////////////////////////////////////////////////////
// モジュール名 BMI088ReadByte
// 処理概要     指定レジスタの値を読み出す
// 引数         reg: レジスタのアドレス
// 戻り値       読み出した値
////////////////////////////////////////////////////////////////////
uint8_t BMI088ReadByteByte( uint8_t reg ) {
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
// モジュール名 BMI088WriteByte
// 処理概要     指定レジスタに値を書き込む
// 引数         reg: レジスタのアドレス val: 書き込む値
// 戻り値       なし
////////////////////////////////////////////////////////////////////
void BMI088WriteByte( uint8_t reg, uint8_t val )  {
	uint8_t txData[2]={reg & 0x7F,val};
	uint8_t txNum=2;
	
	CSB2_RESET;
	BMI088TRANSFER;
	CSB2_SET;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 BMI088ReadAxisData
// 処理概要     指定レジスタの読み出し
// 引数         reg:レジスタアドレス
// 戻り値       読み出したデータ
/////////////////////////////////////////////////////////////////////
void BMI088ReadAxisData(uint8_t reg, uint8_t *rxData ) {
    uint8_t txData;
	uint8_t txNum=1,rxNum=6;

	txData = reg | 0x80;

    CSB2_RESET;
	BMI088TRANSFER;
    BMI088RECEIVES;
	CSB2_SET;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 initBMI088
// 処理概要     初期設定パラメータの書き込み
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
bool initBMI088(void) {
    if(BMI088ReadByteByte(0x00) == 0xf) {
		BMI088WriteByte(0x14,0xB6);	// ソフトウェアリセット
		HAL_Delay(40);
        // コンフィグ設定

        // 加速
        
        // ジャイロ
        BMI088WriteByte(0x10,0x84);	// BANDWIDTHを200Hzに設定
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
	int16_t gyroVal[3], i;

	// 角速度の生データを取得
	BMI088ReadAxisData(0x02,rawData);
	for(i=0;i<3;i++) {
		gyroVal[i] = (rawData[(i*2)+1] << 8) | rawData[i*2];	// LSBとMSBを結合
	}

    BMI088val.gyro.x = (float)gyroVal[0] / GYROLSB;
    BMI088val.gyro.y = (float)gyroVal[1] / GYROLSB;
    BMI088val.gyro.z = (float)gyroVal[2] / GYROLSB;

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
