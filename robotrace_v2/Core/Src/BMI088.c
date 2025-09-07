//====================================//
// インクルード
//====================================//
#include "BMI088.h"
#include <math.h> // 加速度補正の計算で使用
//====================================//
// グローバル変数の宣
//====================================//
axis accele = {0.0F, 0.0F, 0.0F};
axis gyro = {0.0F, 0.0F, 0.0F};
axis gyroTotal = {0.0F, 0.0F, 0.0F};
axis angle = {0.0F, 0.0F, 0.0F};
volatile IMUval BMI088val;

int16_t angleOffset[3] = {0, 0, 0};
bool calibratIMU = false;
/////////////////////////////////////////////////////////////////////
// モジュール名 BMI088ReadByteG
// 処理概要     指定レジスタの値を読み出す(ジャイロセンサ部)
// 引数         reg: レジスタのアドレス
// 戻り値       読み出した値
////////////////////////////////////////////////////////////////////
uint8_t BMI088readByte(bool sensorType, uint8_t reg)
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
void BMI088writeByte(bool sensorType, uint8_t reg, uint8_t val)
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
void BMI088readAxisData(bool sensorType, uint8_t reg, uint8_t *rxData, uint8_t rxNum)
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
	HAL_Delay(10);
	BMI088readByte(ACCELE, REG_ACC_CHIP_ID); // 加速度センサSPIモードに切り替え(SPIダミーリード)
	HAL_Delay(100);
	BMI088val.Aid = BMI088readByte(ACCELE, REG_ACC_CHIP_ID); // ノーマルモード移行前にチップIDを読む
	BMI088val.Gid = BMI088readByte(GYRO, REG_GYRO_CHIP_ID);
	
	if (BMI088val.Gid == 0x0f || BMI088val.Aid == 0x1e)
	{
		// コンフィグ設定

		// 加速度
		BMI088writeByte(ACCELE, REG_ACC_RANGE, 0x00); // レンジを3gに設定
		BMI088writeByte(ACCELE, REG_ACC_CONF, 0xA9);  // ODRを200Hzに設定
		BMI088writeByte(ACCELE, REG_ACC_PWR_CTRL, 0x04); // 加速度センサノーマルモードに移行
		HAL_Delay(450);
		
		// ジャイロ
		BMI088writeByte(GYRO, REG_GYRO_SOFTRESET, 0xB6); // ソフトウェアリセット
		HAL_Delay(100);
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
	gyroVal[0] = ((rawData[1] << 8) | rawData[0]) - angleOffset[0]; // x軸角速度
	gyroVal[1] = ((rawData[3] << 8) | rawData[2]) - angleOffset[1]; // y軸角速度
	gyroVal[2] = ((rawData[5] << 8) | rawData[4]) - angleOffset[2]; // z軸角速度

	BMI088val.gyro.x = (float)gyroVal[0] / GYROLSB * COEFF_DPD; // x軸角速度[deg/s]
	BMI088val.gyro.y = (float)gyroVal[1] / GYROLSB * COEFF_DPD; // y軸角速度[deg/s]
	BMI088val.gyro.z = (float)gyroVal[2] / GYROLSB * COEFF_DPD; // z軸角速度[deg/s]
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
	accelVal[0] = (rawData[2] << 8) | rawData[1];
	accelVal[1] = (rawData[4] << 8) | rawData[3];
	accelVal[2] = (rawData[6] << 8) | rawData[5];

	BMI088val.accele.x = (float)accelVal[0] / ACCELELSB; // x軸加速度[g]
	BMI088val.accele.y = (float)accelVal[1] / ACCELELSB; // y軸加速度[g]
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
/////////////////////////////////////////////////////////////////////
// モジュール名 calcDegrees
// 処理概要     角度の計算（加速度とジャイロの融合）
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void calcDegrees(void)
{
	if(!BMI088val.Initialized)
	{
		return;
	}
	// ジャイロ積分による角度更新 度数法
    BMI088val.angle.x += BMI088val.gyro.x * DEFF_TIME;  // pitch
    BMI088val.angle.y += BMI088val.gyro.y * DEFF_TIME;  // roll
    BMI088val.angle.z += BMI088val.gyro.z * DEFF_TIME;  // yaw（補正しない）

#ifdef USE_ACCELE
    // 加速度からのピッチ・ロール角算出 度数法
    volatile float pitchAccele = atan2f(BMI088val.accele.y, BMI088val.accele.z) * 180.0f / M_PI;
    volatile float rollAccele = atan2f(BMI088val.accele.x,sqrtf(BMI088val.accele.y * BMI088val.accele.y +BMI088val.accele.z * BMI088val.accele.z)) * 180.0f / M_PI;

    // 相補フィルタでドリフト補正
    BMI088val.angle.x = COEFF_COMPFILTER * BMI088val.angle.x + (1.0f - COEFF_COMPFILTER) * pitchAccele;
    BMI088val.angle.y = COEFF_COMPFILTER * BMI088val.angle.y + (1.0f - COEFF_COMPFILTER) * rollAccele;
#endif
    // Z軸（ヨー角）は加速度では補正できないのでそのまま
}
/////////////////////////////////////////////////////////////////////
// モジュール名 cariblationIMU
// 処理概要     角速度キャリブレーション
// 引数         なし
// 戻り値       なし
/////////////////////////////////////////////////////////////////////
void calibrationIMU(void)
{
	static int32_t angleInt[3];
	static uint16_t i = 0;
	uint8_t rawData[6];
	int16_t gyroVal[3];

	if (i < (uint32_t)(1.0 / DEFF_TIME))
	{
		// 角速度の生データを取得
		BMI088readAxisData(GYRO, REG_RATE_X_LSB, rawData, 6);
		// LSBとMSBを結合
		gyroVal[0] = (rawData[1] << 8) | rawData[0];
		gyroVal[1] = (rawData[3] << 8) | rawData[2];
		gyroVal[2] = (rawData[5] << 8) | rawData[4];

		angleInt[0] += gyroVal[0];
		angleInt[1] += gyroVal[1];
		angleInt[2] += gyroVal[2];
		i++;
	}
	else
	{
		angleOffset[0] = angleInt[0] / i;
		angleOffset[1] = angleInt[1] / i;
		angleOffset[2] = angleInt[2] / i;
		angleInt[0] = 0;
		angleInt[1] = 0;
		angleInt[2] = 0;
		i = 0;
		calibratIMU = false;
	}
}
