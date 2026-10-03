//====================================//
// インクルード
//====================================//
#include "main.h"
#include "BMI088.h"
//====================================//
// グローバル変数の宣言
//====================================//
volatile IMUval BMI088val;
/////////////////////////////////////////////////////////////////////
// モジュール名 BMI088readByte
// 処理概要     センサー別のSPIダミーを除去し、単一レジスタを読む
// 引数         sensorType: センサー種別、reg: アドレス、value: 読出し先
// 戻り値       true: 読出し成功、false: SPIエラー
////////////////////////////////////////////////////////////////////
static bool BMI088readByte(bool sensorType, uint8_t reg, uint8_t *value)
{
	uint8_t txData[3] = {reg | 0x80, 0x00, 0x00}, rxData[3] = {0};
	uint16_t size = (sensorType == ACCELE) ? 3U : 2U;

	if(sensorType == ACCELE)
	{
		CSB1_RESET;
	} else {
		CSB2_RESET;
	}

	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&SPI_Handle_IMU, txData, rxData, size, 1000);

	if(sensorType == ACCELE)
	{
		CSB1_SET;
	} else {
		CSB2_SET;
	}

	if (status == HAL_OK)
	{
		*value = rxData[size - 1U];
	}
	return status == HAL_OK;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 BMI088writeByte
// 処理概要     初期化用レジスタを書き込み、次のアクセスまで待つ
// 引数         sensorType: センサー種別、reg: アドレス、val: 書込み値
// 戻り値       true: 書込み成功、false: SPIエラー
////////////////////////////////////////////////////////////////////
static bool BMI088writeByte(bool sensorType, uint8_t reg, uint8_t val)
{
	uint8_t txData[2] = {reg, val}, rxData[2];

	if(sensorType == ACCELE)
	{
		CSB1_RESET;
	} else {
		CSB2_RESET;
	}

	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&SPI_Handle_IMU, txData, rxData, sizeof(txData), 1000);

	if(sensorType == ACCELE)
	{
		CSB1_SET;
	} else {
		CSB2_SET;
	}
	// 初期化専用。Suspend時の450usを含む書込み後の待機を確保する。
	HAL_Delay(1);
	return status == HAL_OK;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 BMI088checkRegister
// 処理概要     読出し専用・予約ビットを除き初期化設定を照合する
// 引数         sensorType: 種別、reg: アドレス、mask: 比較ビット、expected: 期待値
// 戻り値       true: 通信成功かつ設定一致、false: 通信失敗または不一致
/////////////////////////////////////////////////////////////////////
static bool BMI088checkRegister(bool sensorType, uint8_t reg, uint8_t mask, uint8_t expected)
{
	uint8_t value = 0;
	return BMI088readByte(sensorType, reg, &value) && (value & mask) == expected;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 BMI088ReadAxisDataG
// 処理概要     指定レジスタの読み出し(ジャイロセンサ部)
// 引数         sensorType:センサー種別、reg:レジスタアドレス、rxData:受信先、rxNum:受信バイト数
// 戻り値       true:読出し成功 false:SPIエラー
/////////////////////////////////////////////////////////////////////
static bool BMI088readAxisData(bool sensorType, uint8_t reg, uint8_t *rxData, uint8_t rxNum)
{
	uint8_t txData[20] = {0}, rxDatabuff[20];

	txData[0] = reg | 0x80; // 送信用データに変換

	if(sensorType == ACCELE)
	{
		CSB1_RESET;
	} else {
		CSB2_RESET;
	}

	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&SPI_Handle_IMU, txData, rxDatabuff, rxNum+1, 1000);
	if (status == HAL_OK)
	{
		memcpy(rxData,rxDatabuff+1,rxNum); // レジスタ送信時の受信データを除いてコピー
	}

	if(sensorType == ACCELE)
	{
		CSB1_SET;
	} else {
		CSB2_SET;
	}
	return status == HAL_OK;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 initBMI088
// 処理概要     センサーID、設定書込み、読戻しを検証して初期化する
// 引数         なし
// 戻り値       true: 初期化成功、false: 通信失敗または設定不一致
/////////////////////////////////////////////////////////////////////
bool initBMI088(void)
{
	uint8_t value = 0;
	BMI088val.Initialized = 0;
	BMI088val.Aid = 0;
	BMI088val.Gid = 0;
	BMI088val.tempValid = false;
	HAL_Delay(30); // 電源投入後のジャイロ起動を待つ。
	// 最初の加速度アクセスはSPIモード切替用。応答値は採用しない。
	if (!BMI088readByte(ACCELE, REG_ACC_CHIP_ID, &value) ||
		!BMI088readByte(ACCELE, REG_ACC_CHIP_ID, &value)) return false;
	BMI088val.Aid = value;
	if (!BMI088readByte(GYRO, REG_GYRO_CHIP_ID, &value)) return false;
	BMI088val.Gid = value;
	if (BMI088val.Aid != 0x1e || BMI088val.Gid != 0x0f) return false;

	if (!BMI088writeByte(GYRO, REG_GYRO_SOFTRESET, 0xB6)) return false;
	HAL_Delay(30); // リセット完了前に設定を書き込まない。
	if (!BMI088writeByte(GYRO, REG_GYRO_BANDWISTH, 0x02) ||
		!BMI088writeByte(GYRO, REG_GYRO_RANGE, 0x00)) return false;

	// Suspend解除とセンサーONの両方を明示する。
	if (!BMI088writeByte(ACCELE, REG_ACC_PWR_CONF, 0x00) ||
		!BMI088writeByte(ACCELE, REG_ACC_PWR_CTRL, 0x04) ||
		!BMI088writeByte(ACCELE, REG_ACC_RANGE, 0x01) ||
		!BMI088writeByte(ACCELE, REG_ACC_CONF, 0xAC)) return false;
	HAL_Delay(10); // 加速度の設定反映と最初の有効サンプルを待つ。

	// 帯域レジスタのbit7は読出し専用で常に1のため比較対象から除く。
	if (!BMI088checkRegister(GYRO, REG_GYRO_CHIP_ID, 0xFF, 0x0F) ||
		!BMI088checkRegister(GYRO, REG_GYRO_BANDWISTH, 0x7F, 0x02) ||
		!BMI088checkRegister(GYRO, REG_GYRO_RANGE, 0x07, 0x00) ||
		!BMI088checkRegister(GYRO, REG_GYRO_LPM1, 0xA0, 0x00) ||
		!BMI088checkRegister(ACCELE, REG_ACC_CHIP_ID, 0xFF, 0x1E) ||
		!BMI088checkRegister(ACCELE, REG_ACC_PWR_CONF, 0x03, 0x00) ||
		!BMI088checkRegister(ACCELE, REG_ACC_PWR_CTRL, 0x04, 0x04) ||
		!BMI088checkRegister(ACCELE, REG_ACC_RANGE, 0x03, 0x01) ||
		!BMI088checkRegister(ACCELE, REG_ACC_CONF, 0xFF, 0xAC)) return false;

	BMI088val.Initialized = 1;
	return true;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 BMI088getGyro
// 処理概要     角速度の取得
// 引数         なし
// 戻り値       true:取得成功 false:読出し失敗
/////////////////////////////////////////////////////////////////////
bool BMI088getGyro(void)
{
	if(!BMI088val.Initialized)
	{
		return false;
	}
	uint8_t rawData[6];
	int16_t gyroVal[3];

	// 角速度の生データを取得
	if (!BMI088readAxisData(GYRO, REG_RATE_X_LSB, rawData, 6)) return false;
	// LSBとMSBを結合
	gyroVal[0] = ((rawData[1] << 8) | rawData[0]); // x軸角速度
	gyroVal[1] = ((rawData[3] << 8) | rawData[2]); // y軸角速度
	gyroVal[2] = ((rawData[5] << 8) | rawData[4]); // z軸角速度

	BMI088val.gyro.x = (float)gyroVal[0] / GYROLSB; // x軸角速度[deg/s]
	BMI088val.gyro.y = (float)gyroVal[1] / GYROLSB; // y軸角速度[deg/s]
	BMI088val.gyro.z = (float)gyroVal[2] / GYROLSB; // z軸角速度[deg/s]
	return true;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 BMI088getAccele
// 処理概要     加速度の取得（角度補正に使用）
// 引数         なし
// 戻り値       true:取得成功 false:読出し失敗
/////////////////////////////////////////////////////////////////////
bool BMI088getAccele(void)
{
#ifdef USE_ACCELE
	if(!BMI088val.Initialized)
	{
		return false;
	}
	uint8_t rawData[8];
	int16_t accelVal[3];

	// 加速度の生データを取得
	if (!BMI088readAxisData(ACCELE, REG_ACC_X_LSB, rawData, 7)) return false;
	// LSBとMSBを結合
	// 最初のデータは破棄する
	accelVal[0] = ((rawData[2] << 8) | rawData[1]);
	accelVal[1] = ((rawData[4] << 8) | rawData[3]);
	accelVal[2] = ((rawData[6] << 8) | rawData[5]);

	BMI088val.accele.x = (float)accelVal[0] / ACCELELSB; // x軸加速度[g]
	BMI088val.accele.y = (float)accelVal[1] / ACCELELSB * -1; // y軸加速度[g]
	BMI088val.accele.z = (float)accelVal[2] / ACCELELSB; // z軸加速度[g]
#endif
	return true;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 BMI088getTemp
// 処理概要     温度の取得
// 引数         なし
// 戻り値       true:SPI読出し成功 false:読出し失敗
/////////////////////////////////////////////////////////////////////
bool BMI088getTemp(void)
{
	if(!BMI088val.Initialized)
	{
		return false;
	}
	uint8_t rawData[3];
	uint16_t temperatureCode;
	float temperatureC = BMI088_TEMP_INVALID_C;

	// 加速度センサーのSPI読み出しは先頭1バイトがダミーのため破棄する
	if (!BMI088readAxisData(ACCELE, REG_TEMP_MSB, rawData, 3))
	{
		BMI088val.tempValid = false;
		BMI088val.temp = BMI088_TEMP_INVALID_C;
		return false;
	}
	temperatureCode = ((uint16_t)rawData[1] << 3) | ((uint16_t)rawData[2] >> 5);
	BMI088val.tempRaw = temperatureCode;
	BMI088val.tempValid = BMI088DecodeTemperature(rawData[1], rawData[2], &temperatureC);
	BMI088val.temp = BMI088val.tempValid ? temperatureC : BMI088_TEMP_INVALID_C;
	return true;
}
/////////////////////////////////////////////////////////////////////
// モジュール名 BMI088DecodeTemperature
// 処理概要     BMI088温度レジスタを11ビット2の補数で摂氏へ復号する
// 引数         tempMsb: 温度MSB、tempLsb: 温度LSB、temperatureC: 復号結果[°C]
// 戻り値       true: 有効、false: 無効値
/////////////////////////////////////////////////////////////////////
bool BMI088DecodeTemperature(uint8_t tempMsb, uint8_t tempLsb, float *temperatureC)
{
	uint16_t temperatureCode = ((uint16_t)tempMsb << 3) | ((uint16_t)tempLsb >> 5);
	int16_t signedCode;

	if (temperatureC == NULL || temperatureCode == BMI088_TEMP_INVALID_CODE)
	{
		return false;
	}

	if ((temperatureCode & 0x0400U) != 0U)
	{
		signedCode = (int16_t)temperatureCode - 2048;
	}
	else
	{
		signedCode = (int16_t)temperatureCode;
	}
	*temperatureC = 23.0F + ((float)signedCode * 0.125F);
	return true;
}
