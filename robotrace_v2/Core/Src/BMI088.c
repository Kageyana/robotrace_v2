//====================================//
// インクルード
//====================================//
#include "main.h"
#include "BMI088.h"
//====================================//
// グローバル変数の宣
//====================================//
volatile IMUval BMI088val;
/////////////////////////////////////////////////////////////////////
// モジュール名 BMI088ReadByteG
// 処理概要     指定レジスタの値を読み出す(ジャイロセンサ部)
// 引数         reg: レジスタのアドレス
// 戻り値       読み出した値
////////////////////////////////////////////////////////////////////
static uint8_t BMI088readByte(bool sensorType, uint8_t reg)
{
	uint8_t txData[2]={reg | 0x80, 0x0}, rxData[2] = {0x0};

	if(sensorType == ACCELE)
	{
		CSB1_RESET;
	} else {
		CSB2_RESET;
	}

	HAL_SPI_TransmitReceive(&SPI_Handle_IMU, txData, rxData, sizeof(txData), 1000);

	if(sensorType == ACCELE)
	{
		CSB1_SET;
	} else {
		CSB2_SET;
	}

	return rxData[1];
}
/////////////////////////////////////////////////////////////////////
// モジュール名 BMI088WriteByteG
// 処理概要     指定レジスタに値を書き込む(ジャイロセンサ部)
// 引数         reg: レジスタのアドレス val: 書き込む値
// 戻り値       なし
////////////////////////////////////////////////////////////////////
static void BMI088writeByte(bool sensorType, uint8_t reg, uint8_t val)
{
	uint8_t txData[2] = {reg, val}, rxData[2];

	if(sensorType == ACCELE)
	{
		CSB1_RESET;
	} else {
		CSB2_RESET;
	}

	HAL_SPI_TransmitReceive(&SPI_Handle_IMU, txData, rxData, sizeof(txData), 1000);;

	if(sensorType == ACCELE)
	{
		CSB1_SET;
	} else {
		CSB2_SET;
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 BMI088ReadAxisDataG
// 処理概要     指定レジスタの読み出し(ジャイロセンサ部)
// 引数         reg:レジスタアドレス
// 戻り値       読み出したデータ
/////////////////////////////////////////////////////////////////////
static void BMI088readAxisData(bool sensorType, uint8_t reg, uint8_t *rxData, uint8_t rxNum)
{
	uint8_t txData[20] = {0}, rxDatabuff[20];

	txData[0] = reg | 0x80; // 送信用データに変換

	if(sensorType == ACCELE)
	{
		CSB1_RESET;
	} else {
		CSB2_RESET;
	}

	HAL_SPI_TransmitReceive(&SPI_Handle_IMU, txData, rxDatabuff, rxNum+1, 1000);
	memcpy(rxData,rxDatabuff+1,rxNum); // レジスタ送信時の受信データを除いてコピー

	if(sensorType == ACCELE)
	{
		CSB1_SET;
	} else {
		CSB2_SET;
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 initBMI088
// 処理概要     初期設定パラメータの書き込み
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
bool initBMI088(void)
{
	HAL_Delay(20);
	BMI088readByte(ACCELE, REG_ACC_CHIP_ID); // 加速度センサSPIモードに切り替え(SPIダミーリード)
	BMI088writeByte(ACCELE, REG_ACC_PWR_CTRL, 0x04); // 加速度センサノーマルモードに移行
	HAL_Delay(10);
	BMI088readByte(ACCELE, REG_ACC_CHIP_ID); // 加速度センサSPIモードに切り替え(SPIダミーリード)
	BMI088val.Aid = BMI088readByte(ACCELE, REG_ACC_CHIP_ID); // ノーマルモード移行前にチップIDを読む
	BMI088val.Gid = BMI088readByte(GYRO, REG_GYRO_CHIP_ID);
	
	if (BMI088val.Gid == 0x0f && BMI088val.Aid == 0x1e)
	{
		// コンフィグ設定
		// 加速度
		BMI088writeByte(ACCELE, REG_ACC_PWR_CTRL, 0x04); // 加速度センサノーマルモードに移行
		BMI088writeByte(ACCELE, REG_ACC_RANGE, 0x01); // レンジを6gに設定
		BMI088writeByte(ACCELE, REG_ACC_CONF, 0xAc);  // ODRを1600Hzに設定
		HAL_Delay(10);
		
		// ジャイロ
		BMI088writeByte(GYRO, REG_GYRO_SOFTRESET, 0xB6); // ソフトウェアリセット
		HAL_Delay(10);
		BMI088writeByte(GYRO, REG_GYRO_BANDWISTH, 0x02); // ODRを1kHz バンドフィルタ116Hzに設定
		BMI088writeByte(GYRO, REG_GYRO_RANGE, 0x00);	// レンジを2000dpsに設定

		BMI088val.Initialized = 1;

		return true;
	}
	else
	{
		return false;
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール名 BMI088getGyro
// 処理概要     角速度の取得
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void BMI088getGyro(void)
{
	if(!BMI088val.Initialized)
	{
		return;
	}
	uint8_t rawData[6];
	int16_t gyroVal[3];

	// 角速度の生データを取得
	BMI088readAxisData(GYRO, REG_RATE_X_LSB, rawData, 6); // x,y,z軸の生データを取得
	// LSBとMSBを結合
	gyroVal[0] = ((rawData[1] << 8) | rawData[0]); // x軸角速度
	gyroVal[1] = ((rawData[3] << 8) | rawData[2]); // y軸角速度
	gyroVal[2] = ((rawData[5] << 8) | rawData[4]); // z軸角速度

	BMI088val.gyro.x = (float)gyroVal[0] / GYROLSB; // x軸角速度[deg/s]
	BMI088val.gyro.y = (float)gyroVal[1] / GYROLSB; // y軸角速度[deg/s]
	BMI088val.gyro.z = (float)gyroVal[2] / GYROLSB; // z軸角速度[deg/s]
}
/////////////////////////////////////////////////////////////////////
// モジュール名 BMI088getAccele
// 処理概要     加速度の取得（角度補正に使用）
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void BMI088getAccele(void)
{
#ifdef USE_ACCELE
	if(!BMI088val.Initialized)
	{
		return;
	}
	uint8_t rawData[8];
	int16_t accelVal[3];

	// 加速度の生データを取得
	BMI088readAxisData(ACCELE, REG_ACC_X_LSB, rawData, 7);
	// LSBとMSBを結合
	// 最初のデータは破棄する
	accelVal[0] = ((rawData[2] << 8) | rawData[1]);
	accelVal[1] = ((rawData[4] << 8) | rawData[3]);
	accelVal[2] = ((rawData[6] << 8) | rawData[5]);

	BMI088val.accele.x = (float)accelVal[0] / ACCELELSB; // x軸加速度[g]
	BMI088val.accele.y = (float)accelVal[1] / ACCELELSB * -1; // y軸加速度[g]
	BMI088val.accele.z = (float)accelVal[2] / ACCELELSB; // z軸加速度[g]
#endif
}
/////////////////////////////////////////////////////////////////////
// モジュール名 BMI088getTemp
// 処理概要     温度の取得
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void BMI088getTemp(void)
{
	if(!BMI088val.Initialized)
	{
		return;
	}
	uint8_t rawData[3];
	uint16_t tempValu;
	int16_t tempVal;

	// 温度の生データを取得
	BMI088readAxisData(ACCELE, REG_TEMP_MSB, rawData, 3);
	// LSBとMSBを結合
	tempValu = (rawData[1] << 3) | (rawData[2] >> 5);
	if (tempValu > 1023)
	{
		tempVal = ~tempValu + 0x8000;
	}
	else
	{
		tempVal = tempValu;
	}

	BMI088val.temp = ((float)tempVal * 0.125F) + 23.0F;
}
